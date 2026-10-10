#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include "serial_arm/dynamics/calibration_retime.hpp"
#include "serial_arm/dynamics/calibration_recovery_alignment.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/torque_residual_observer.hpp"
#include "serial_arm/robot.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef SERIAL_ARM_DEFAULT_CONFIG_PATH
#define SERIAL_ARM_DEFAULT_CONFIG_PATH "config/arm.yaml"
#endif

namespace {

using namespace serial_arm;

constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

struct CliOptions {
    std::string config_path{ SERIAL_ARM_DEFAULT_CONFIG_PATH };
    std::string compare_lhs_path;
    std::string compare_rhs_path;
    std::string hardware_plugin;
    std::string hardware_config;
    std::string robot_profile;
    std::string profiles_file;
    HardwareConfigOverrides hardware_overrides;
    bool show_help{ false };
    bool compare_config{ false };
    bool machine_mode{ false };
};

struct HardwareConnectionSummary {
    std::string bus;
    std::string serial_port;
    int baudrate{ 0 };
};

struct StreamState {
    bool enabled{ false };
    bool completion_reported{ false };
    double speed_scale{ 0.3 };
    JointVector target_pos;
    JointVector ref_pos;
    JointVector ref_vel;
    Robot::TimePoint last_update_time{};
    bool has_last_update_time{ false };
};

struct FrictionCalibrationTrajectory {
    std::vector<JointVector> positions;
    double sample_dt{ 0.02 };
};

enum class ModelCalibrationPhase {
    IDLE = 0,
    TEACHING,
    WAITING_REPLAY_CONFIRMATION,
    STATIC_REVERSE,
    STATIC_FORWARD,
    GRAVITY_FITTING,
    FRICTION_REVERSE_SLOW,
    FRICTION_FORWARD_SLOW,
    FRICTION_REVERSE_FAST,
    FRICTION_FORWARD_FAST,
    FRICTION_FITTING,
    PAUSED,
    COMPLETE,
    CANCELLED,
    FAILED,
};

struct ModelCalibrationPoseTarget {
    std::size_t trajectory_index{ 0 };
    std::size_t pose_group{ 0 };
    bool validation{ false };
    double information_score{ 0.0 };
};


struct GravityCorrectionPackage {
    std::string urdf_fingerprint;
    std::vector<std::string> joint_names;
    std::vector<GravityFirstMoment> first_moments;
    JointVector torque_bias;
    JointVector torque_threshold;
    bool static_pass{ false };
    bool friction_pass{ false };
    FrictionResidualModelCfg friction;
};

GravityCorrectionPackage load_gravity_correction_package(
    const std::string& path,
    const std::vector<std::string>& expected_joint_names,
    const std::string& expected_urdf_path) {
    const YAML::Node root = YAML::LoadFile(path);
    if(!root || !root.IsMap() || !root["schema"] || root["schema"].as<std::string>() != "serial_arm_gravity_correction") {
        throw std::runtime_error("invalid gravity correction schema");
    }
    GravityCorrectionPackage package;
    package.urdf_fingerprint = root["urdf_fingerprint"] ? root["urdf_fingerprint"].as<std::string>() : std::string{};
    if(package.urdf_fingerprint.empty() || package.urdf_fingerprint != model_calibration_file_fingerprint(expected_urdf_path)) {
        throw std::runtime_error("gravity correction URDF fingerprint mismatch");
    }
    if(!root["joint_names"] || !root["joint_names"].IsSequence()) throw std::runtime_error("gravity correction joint_names are missing");
    package.joint_names = root["joint_names"].as<std::vector<std::string>>();
    if(package.joint_names != expected_joint_names) throw std::runtime_error("gravity correction joint order mismatch");
    package.static_pass = root["static_pass"] && root["static_pass"].as<bool>();
    package.friction_pass = root["friction_pass"] && root["friction_pass"].as<bool>();
    if(!package.static_pass) throw std::runtime_error("gravity correction static validation did not pass");
    const YAML::Node moments = root["first_moments"];
    if(!moments || !moments.IsMap()) throw std::runtime_error("gravity correction first_moments are missing");
    for(auto it = moments.begin(); it != moments.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const JointVector values = it->second.as<JointVector>();
        if(values.size() != 3 || !std::all_of(values.begin(), values.end(), [](double value){ return std::isfinite(value); })) {
            throw std::runtime_error("invalid gravity correction first moment for " + name);
        }
        package.first_moments.push_back(GravityFirstMoment{name, Eigen::Vector3d(values[0], values[1], values[2])});
    }
    auto required_vector = [&](const YAML::Node& parent, const char* key) {
        if(!parent || !parent[key]) throw std::runtime_error(std::string("gravity correction missing ") + key);
        JointVector values = parent[key].as<JointVector>();
        if(values.size() != expected_joint_names.size() || !std::all_of(values.begin(), values.end(), [](double value){ return std::isfinite(value); })) {
            throw std::runtime_error(std::string("invalid gravity correction vector ") + key);
        }
        return values;
    };
    const YAML::Node residual = root["residual"];
    package.torque_bias = required_vector(residual, "torque_bias");
    package.torque_threshold = required_vector(residual, "torque_threshold");
    package.friction.enabled = false;
    if(package.friction_pass && residual["friction"] && residual["friction"].IsMap()) {
        const YAML::Node friction = residual["friction"];
        package.friction.enabled = true;
        package.friction.velocity_transition = friction["velocity_transition"] ? friction["velocity_transition"].as<double>() : 0.03;
        package.friction.positive_coulomb = required_vector(friction, "positive_coulomb");
        package.friction.positive_viscous = required_vector(friction, "positive_viscous");
        package.friction.negative_coulomb = required_vector(friction, "negative_coulomb");
        package.friction.negative_viscous = required_vector(friction, "negative_viscous");
    }
    return package;
}

struct ModelCalibrationTaskOptions {
    std::size_t pose_budget{ 8 };
    std::size_t minimum_training_groups{ 4 };
    double validation_fraction{ 0.30 };
    double static_hold_s{ 0.30 };
    double static_sample_s{ 0.70 };
    double static_timeout_s{ 5.0 };
    double static_max_velocity{ 0.03 };
    double static_max_acceleration{ 1.5 };
    double minimum_information_score{ 1.0e-4 };
    double max_com_offset_m{ 0.05 };
    double regularization{ 1.0e-2 };
    double svd_relative_threshold{ 1.0e-4 };
    std::string identification_mode{ "global" };
    std::vector<std::string> locked_links;

};

std::string to_string(ModelCalibrationPhase value) {
    switch(value) {
        case ModelCalibrationPhase::IDLE: return "idle";
        case ModelCalibrationPhase::TEACHING: return "teaching";
        case ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION: return "waiting_replay_confirmation";
        case ModelCalibrationPhase::STATIC_REVERSE: return "static_reverse";
        case ModelCalibrationPhase::STATIC_FORWARD: return "static_forward";
        case ModelCalibrationPhase::GRAVITY_FITTING: return "gravity_fitting";
        case ModelCalibrationPhase::FRICTION_REVERSE_SLOW: return "friction_reverse_slow";
        case ModelCalibrationPhase::FRICTION_FORWARD_SLOW: return "friction_forward_slow";
        case ModelCalibrationPhase::FRICTION_REVERSE_FAST: return "friction_reverse_fast";
        case ModelCalibrationPhase::FRICTION_FORWARD_FAST: return "friction_forward_fast";
        case ModelCalibrationPhase::FRICTION_FITTING: return "friction_fitting";
        case ModelCalibrationPhase::PAUSED: return "paused";
        case ModelCalibrationPhase::COMPLETE: return "complete";
        case ModelCalibrationPhase::CANCELLED: return "cancelled";
        case ModelCalibrationPhase::FAILED: return "failed";
    }
    return "unknown";
}

std::string to_string(RobotState value) {
    switch(value) {
        case RobotState::UNCONFIGURED: return "UNCONFIGURED";
        case RobotState::INACTIVE: return "INACTIVE";
        case RobotState::ACTIVE: return "ACTIVE";
        case RobotState::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}

std::string to_string(JointImpedanceMode value) {
    switch(value) {
        case JointImpedanceMode::RIGID_HOLD: return "RIGID_HOLD";
        case JointImpedanceMode::RIGID_TRACKING: return "RIGID_TRACKING";
        case JointImpedanceMode::COMPLIANT_HOLD: return "COMPLIANT_HOLD";
        case JointImpedanceMode::COMPLIANT_DRAG: return "COMPLIANT_DRAG";
        case JointImpedanceMode::COMPLIANT_TRACKING: return "COMPLIANT_TRACKING";
    }
    return "UNKNOWN";
}

std::string to_string(ModelFeedforwardMode value) {
    switch(value) {
        case ModelFeedforwardMode::NONE: return "NONE";
        case ModelFeedforwardMode::GRAVITY: return "GRAVITY";
        case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS: return "FULL_INVERSE_DYNAMICS";
    }
    return "UNKNOWN";
}

std::string to_string(FaultHoldMode value) {
    switch(value) {
        case FaultHoldMode::RIGID_HOLD: return "RIGID_HOLD";
        case FaultHoldMode::COMPLIANT_RECOVERY: return "COMPLIANT_RECOVERY";
    }
    return "UNKNOWN";
}

std::string to_string(RobotErr value) {
    switch(value) {
        case RobotErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case RobotErr::ALREADY_CONFIGURED: return "ALREADY_CONFIGURED";
        case RobotErr::INVALID_CFG: return "INVALID_CFG";
        case RobotErr::NULL_MOTOR_BUS: return "NULL_MOTOR_BUS";
        case RobotErr::MOTOR_BUS_SIZE_MISMATCH: return "MOTOR_BUS_SIZE_MISMATCH";
        case RobotErr::WRITE_DISABLED: return "WRITE_DISABLED";
        case RobotErr::NOT_ACTIVE: return "NOT_ACTIVE";
        case RobotErr::NOT_INACTIVE: return "NOT_INACTIVE";
        case RobotErr::ALREADY_ACTIVE: return "ALREADY_ACTIVE";
        case RobotErr::FAULTED: return "FAULTED";
        case RobotErr::NOT_FAULTED: return "NOT_FAULTED";
        case RobotErr::INVALID_TIME: return "INVALID_TIME";
        case RobotErr::MOTOR_BUS_CONNECT_FAILED: return "MOTOR_BUS_CONNECT_FAILED";
        case RobotErr::MOTOR_BUS_ACTIVATE_FAILED: return "MOTOR_BUS_ACTIVATE_FAILED";
        case RobotErr::MOTOR_BUS_READ_FAILED: return "MOTOR_BUS_READ_FAILED";
        case RobotErr::MOTOR_BUS_WRITE_FAILED: return "MOTOR_BUS_WRITE_FAILED";
        case RobotErr::MOTOR_BUS_DEACTIVATE_FAILED: return "MOTOR_BUS_DEACTIVATE_FAILED";
        case RobotErr::MOTOR_BUS_RECOVER_FAILED: return "MOTOR_BUS_RECOVER_FAILED";
        case RobotErr::MAPPER_FAILED: return "MAPPER_FAILED";
        case RobotErr::CTRLLER_FAILED: return "CTRLLER_FAILED";
        case RobotErr::SAFETY_FAILED: return "SAFETY_FAILED";
        case RobotErr::MODEL_FEEDFORWARD_FAILED: return "MODEL_FEEDFORWARD_FAILED";
        case RobotErr::INVALID_MODEL_FEEDFORWARD: return "INVALID_MODEL_FEEDFORWARD";
        case RobotErr::INTERACTION_FAILED: return "INTERACTION_FAILED";
        case RobotErr::FAULT_RECOVERY_NOT_ALLOWED: return "FAULT_RECOVERY_NOT_ALLOWED";
    }
    return "UNKNOWN";
}

std::string to_string(MotorBusErr value) {
    switch(value) {
        case MotorBusErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case MotorBusErr::NOT_CONNECTED: return "NOT_CONNECTED";
        case MotorBusErr::NOT_ACTIVE: return "NOT_ACTIVE";
        case MotorBusErr::INVALID_CFG: return "INVALID_CFG";
        case MotorBusErr::OPEN_FAILED: return "OPEN_FAILED";
        case MotorBusErr::READ_FAILED: return "READ_FAILED";
        case MotorBusErr::WRITE_FAILED: return "WRITE_FAILED";
        case MotorBusErr::INVALID_STATE: return "INVALID_STATE";
        case MotorBusErr::INVALID_CMD: return "INVALID_CMD";
        case MotorBusErr::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case MotorBusErr::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case MotorBusErr::TIMEOUT: return "TIMEOUT";
        case MotorBusErr::ENABLE_FAILED: return "ENABLE_FAILED";
        case MotorBusErr::MODE_SWITCH_FAILED: return "MODE_SWITCH_FAILED";
        case MotorBusErr::STOP_FAILED: return "STOP_FAILED";
        case MotorBusErr::DISABLE_FAILED: return "DISABLE_FAILED";
        case MotorBusErr::RECOVER_FAILED: return "RECOVER_FAILED";
    }
    return "UNKNOWN";
}

std::string to_string(HardwareLoaderErr value) {
    switch(value) {
        case HardwareLoaderErr::OPEN_FAILED: return "OPEN_FAILED";
        case HardwareLoaderErr::SYMBOL_FAILED: return "SYMBOL_FAILED";
        case HardwareLoaderErr::CREATE_FAILED: return "CREATE_FAILED";
        case HardwareLoaderErr::CONFIGURE_FAILED: return "CONFIGURE_FAILED";
        case HardwareLoaderErr::CONFIG_OPEN_FAILED: return "CONFIG_OPEN_FAILED";
        case HardwareLoaderErr::CONFIG_SYNTAX_ERROR: return "CONFIG_SYNTAX_ERROR";
        case HardwareLoaderErr::INVALID_OVERRIDE: return "INVALID_OVERRIDE";
    }
    return "UNKNOWN";
}

std::string to_string(JointCtrllerErr value) {
    switch(value) {
        case JointCtrllerErr::OK: return "OK";
        case JointCtrllerErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case JointCtrllerErr::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case JointCtrllerErr::ALREADY_INITIALIZED: return "ALREADY_INITIALIZED";
        case JointCtrllerErr::INVALID_CFG: return "INVALID_CFG";
        case JointCtrllerErr::INVALID_STATE: return "INVALID_STATE";
        case JointCtrllerErr::INVALID_DT: return "INVALID_DT";
        case JointCtrllerErr::INVALID_MODEL_FEEDFORWARD: return "INVALID_MODEL_FEEDFORWARD";
        case JointCtrllerErr::INVALID_IMPEDANCE_MODE: return "INVALID_IMPEDANCE_MODE";
        case JointCtrllerErr::INVALID_CMD_SIZE: return "INVALID_CMD_SIZE";
        case JointCtrllerErr::INVALID_CMD_VALUE: return "INVALID_CMD_VALUE";
        case JointCtrllerErr::INVALID_FULL_CMD: return "INVALID_FULL_CMD";
        case JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE: return "CMD_NOT_ALLOWED_IN_MODE";
        case JointCtrllerErr::FULL_CMD_NOT_ALLOWED: return "FULL_CMD_NOT_ALLOWED";
    }
    return "UNKNOWN";
}

std::string to_string(JointActuatorMapErr value) {
    switch(value) {
        case JointActuatorMapErr::OK: return "OK";
        case JointActuatorMapErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case JointActuatorMapErr::INVALID_CFG: return "INVALID_CFG";
        case JointActuatorMapErr::INVALID_JOINT_STATE: return "INVALID_JOINT_STATE";
        case JointActuatorMapErr::INVALID_ACTUATOR_STATE: return "INVALID_ACTUATOR_STATE";
        case JointActuatorMapErr::INVALID_JOINT_CMD: return "INVALID_JOINT_CMD";
        case JointActuatorMapErr::INVALID_ACTUATOR_CMD: return "INVALID_ACTUATOR_CMD";
        case JointActuatorMapErr::INVALID_CONVERSION_VALUE: return "INVALID_CONVERSION_VALUE";
    }
    return "UNKNOWN";
}

std::string to_string(SafetyErr value) {
    switch(value) {
        case SafetyErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case SafetyErr::INVALID_CFG: return "INVALID_CFG";
        case SafetyErr::INVALID_DT: return "INVALID_DT";
        case SafetyErr::INVALID_STATE_AGE: return "INVALID_STATE_AGE";
        case SafetyErr::INVALID_CMD_AGE: return "INVALID_CMD_AGE";
        case SafetyErr::STATE_TIMEOUT: return "STATE_TIMEOUT";
        case SafetyErr::CMD_TIMEOUT: return "CMD_TIMEOUT";
        case SafetyErr::INVALID_JOINT_STATE_SIZE: return "INVALID_JOINT_STATE_SIZE";
        case SafetyErr::INVALID_ACTUATOR_STATE_SIZE: return "INVALID_ACTUATOR_STATE_SIZE";
        case SafetyErr::NON_FINITE_JOINT_STATE: return "NON_FINITE_JOINT_STATE";
        case SafetyErr::NON_FINITE_ACTUATOR_STATE: return "NON_FINITE_ACTUATOR_STATE";
        case SafetyErr::JOINT_POS_LIMIT: return "JOINT_POS_LIMIT";
        case SafetyErr::JOINT_VEL_LIMIT: return "JOINT_VEL_LIMIT";
        case SafetyErr::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case SafetyErr::ACTUATOR_NOT_ENABLED: return "ACTUATOR_NOT_ENABLED";
        case SafetyErr::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case SafetyErr::INVALID_CMD_SIZE: return "INVALID_CMD_SIZE";
        case SafetyErr::NON_FINITE_CMD: return "NON_FINITE_CMD";
        case SafetyErr::CMD_POS_LIMIT: return "CMD_POS_LIMIT";
        case SafetyErr::CMD_VEL_LIMIT: return "CMD_VEL_LIMIT";
        case SafetyErr::CMD_EFFORT_LIMIT: return "CMD_EFFORT_LIMIT";
        case SafetyErr::CMD_KP_LIMIT: return "CMD_KP_LIMIT";
        case SafetyErr::CMD_KD_LIMIT: return "CMD_KD_LIMIT";
        case SafetyErr::CMD_POS_STEP_LIMIT: return "CMD_POS_STEP_LIMIT";
        case SafetyErr::CMD_VEL_STEP_LIMIT: return "CMD_VEL_STEP_LIMIT";
    }
    return "UNKNOWN";
}

std::string to_string(ModelFeedforwardErr value) {
    switch(value) {
        case ModelFeedforwardErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case ModelFeedforwardErr::INVALID_INPUT: return "INVALID_INPUT";
        case ModelFeedforwardErr::INVALID_MODE: return "INVALID_MODE";
        case ModelFeedforwardErr::COMPUTE_FAILED: return "COMPUTE_FAILED";
    }
    return "UNKNOWN";
}

std::string to_string(DynamicsErr value) {
    switch(value) {
        case DynamicsErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case DynamicsErr::ALREADY_CONFIGURED: return "ALREADY_CONFIGURED";
        case DynamicsErr::NOT_UPDATED: return "NOT_UPDATED";
        case DynamicsErr::INVALID_CFG: return "INVALID_CFG";
        case DynamicsErr::URDF_LOAD_FAILED: return "URDF_LOAD_FAILED";
        case DynamicsErr::JOINT_NOT_FOUND: return "JOINT_NOT_FOUND";
        case DynamicsErr::JOINT_NOT_1DOF: return "JOINT_NOT_1DOF";
        case DynamicsErr::MODEL_SIZE_MISMATCH: return "MODEL_SIZE_MISMATCH";
        case DynamicsErr::FRAME_NOT_FOUND: return "FRAME_NOT_FOUND";
        case DynamicsErr::INVALID_INPUT_SIZE: return "INVALID_INPUT_SIZE";
        case DynamicsErr::NON_FINITE_INPUT: return "NON_FINITE_INPUT";
        case DynamicsErr::GRAVITY_SCALE_OUT_OF_RANGE: return "GRAVITY_SCALE_OUT_OF_RANGE";
        case DynamicsErr::COMPUTE_FAILED: return "COMPUTE_FAILED";
    }
    return "UNKNOWN";
}

void print_vector(const std::string& name, const std::vector<double>& values) {
    std::cout << std::left << std::setw(24) << name << " [";
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << std::fixed << std::setprecision(6) << values[i];
    }
    std::cout << "]\n";
}

void print_joint_yaml_map(const std::string& key, const std::vector<std::string>& joint_names, const JointVector& values) {
    std::cout << key << ": {";
    for(std::size_t i = 0; i < values.size() && i < joint_names.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << joint_names[i] << ": " << std::fixed << std::setprecision(6) << values[i];
    }
    std::cout << "}\n";
}

void print_joint_yaml_bool_map(
    const std::string& key,
    const std::vector<std::string>& joint_names,
    const std::vector<std::uint8_t>& values)
{
    std::cout << key << ": {";
    for(std::size_t i = 0; i < values.size() && i < joint_names.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << joint_names[i] << ": " << (values[i] ? "true" : "false");
    }
    std::cout << "}\n";
}

void print_int_vector(const std::string& name, const std::vector<int>& values) {
    std::cout << std::left << std::setw(24) << name << " [";
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << values[i];
    }
    std::cout << "]\n";
}

void print_matrix(const std::string& name, const Eigen::MatrixXd& matrix) {
    std::cout << name << " (" << matrix.rows() << "x" << matrix.cols() << "):\n";
    std::cout << std::fixed << std::setprecision(6) << matrix << '\n';
}

void print_pose(const std::string& name, const Eigen::Isometry3d& pose) {
    std::cout << name << ":\n";
    std::cout << std::fixed << std::setprecision(6) << pose.matrix() << '\n';
}

void print_fault(const RobotFault& fault) {
    std::cout << "RobotFault: " << to_string(fault.code) << '\n';
    switch(fault.code) {
        case RobotErr::MOTOR_BUS_CONNECT_FAILED:
        case RobotErr::MOTOR_BUS_ACTIVATE_FAILED:
        case RobotErr::MOTOR_BUS_READ_FAILED:
        case RobotErr::MOTOR_BUS_WRITE_FAILED:
        case RobotErr::MOTOR_BUS_DEACTIVATE_FAILED:
        case RobotErr::MOTOR_BUS_RECOVER_FAILED:
            std::cout << "  MotorBusErr: " << to_string(fault.motor_bus_err) << '\n';
            break;
        case RobotErr::MAPPER_FAILED:
            std::cout << "  JointActuatorMapErr: " << to_string(fault.mapper_err) << '\n';
            break;
        case RobotErr::CTRLLER_FAILED:
            std::cout << "  JointCtrllerErr: " << to_string(fault.ctrller_err) << '\n';
            break;
        case RobotErr::SAFETY_FAILED:
            std::cout << "  SafetyErr: " << to_string(fault.safety_fault.code);
            if(fault.safety_fault.index != kInvalidIndex) std::cout << ", index=" << fault.safety_fault.index;
            std::cout << ", value=" << fault.safety_fault.value << ", limit=" << fault.safety_fault.limit << '\n';
            break;
        case RobotErr::MODEL_FEEDFORWARD_FAILED:
        case RobotErr::INVALID_MODEL_FEEDFORWARD:
            std::cout << "  ModelFeedforwardErr: " << to_string(fault.model_feedforward_err) << '\n';
            break;
        case RobotErr::INTERACTION_FAILED:
            std::cout << "  InteractionControllerErr: " << static_cast<int>(fault.interaction_err) << '\n';
            break;
        default:
            break;
    }
}

bool parse_cli(int argc, char** argv, CliOptions& options) {
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--config") {
            if(i + 1 >= argc) return false;
            options.config_path = argv[++i];
        }
        else if(arg == "--hardware-plugin") {
            if(i + 1 >= argc) return false;
            options.hardware_plugin = argv[++i];
        }
        else if(arg == "--hardware-config") {
            if(i + 1 >= argc) return false;
            options.hardware_config = argv[++i];
        }
        else if(arg == "--robot-profile") {
            if(i + 1 >= argc) return false;
            options.robot_profile = argv[++i];
        }
        else if(arg == "--profile-file") {
            if(i + 1 >= argc) return false;
            options.profiles_file = argv[++i];
        }
        else if(arg == "--serial-port") {
            if(i + 1 >= argc) return false;
            options.hardware_overrides.serial_port = argv[++i];
            if(options.hardware_overrides.serial_port->empty()) {
                std::cerr << "Invalid serial-port: empty value\n";
                return false;
            }
        }
        else if(arg == "--baudrate") {
            if(i + 1 >= argc) return false;
            const std::string value = argv[++i];
            std::istringstream input(value);
            int baudrate = 0;
            char extra = 0;
            if(!(input >> baudrate) || (input >> extra) || baudrate <= 0) {
                std::cerr << "Invalid baudrate: " << value << '\n';
                return false;
            }
            options.hardware_overrides.baudrate = baudrate;
        }
        else if(arg == "--bus") {
            if(i + 1 >= argc) return false;
            options.hardware_overrides.bus = argv[++i];
            if(options.hardware_overrides.bus->empty()) {
                std::cerr << "Invalid bus: empty value\n";
                return false;
            }
        }
        else if(arg == "--machine") {
            options.machine_mode = true;
        }
        else if(arg == "--compare-config") {
            if(i + 2 >= argc) return false;
            options.compare_config = true;
            options.compare_lhs_path = argv[++i];
            options.compare_rhs_path = argv[++i];
        }
        else if(arg == "--help" || arg == "-h") {
            options.show_help = true;
        }
        else {
            std::cerr << "未知参数: " << arg << '\n';
            return false;
        }
    }
    return true;
}

void print_usage(const char* program) {
    std::cout << "用法: " << program << " --robot-profile <name> [--profile-file <path>] [--serial-port <path>] [--baudrate <n>] [--bus <name>]\n";
    std::cout << "路径: " << program << " [--config <path>] [--hardware-plugin <name>] [--hardware-config <path>]\n";
    std::cout << "机器接口: " << program << " --machine --robot-profile <name> [connection overrides]\n";
    std::cout << "比较: " << program << " --hardware-plugin <name> --hardware-config <path> --compare-config <config-a.yaml> <config-b.yaml>\n";
    std::cout << "  --serial-port  Override serial port from robot profile hardware configuration\n";
    std::cout << "  --baudrate     Override serial baudrate\n";
    std::cout << "  --bus          Override hardware bus\n";
    std::cout << "说明: runtime.write_enabled=true 使用 Hardware Backend；false 使用离线 mock 后端\n";
}

tl::expected<HardwareConnectionSummary, std::string> load_hardware_connection_summary(
    const std::string& hardware_config,
    const HardwareConfigOverrides& overrides) {
    try {
        const YAML::Node root = YAML::LoadFile(hardware_config);
        if(!root || !root.IsMap()) {
            return tl::make_unexpected("Hardware Config 加载失败: root must be a map");
        }

        HardwareConnectionSummary summary;
        const YAML::Node buses = root["buses"];

        if(overrides.bus) {
            summary.bus = *overrides.bus;
        }
        else {
            std::string discovered_bus;
            for(const auto& item : root) {
                const std::string key = item.first.as<std::string>();
                if(key == "buses" || !item.second.IsMap() || !item.second["bus"]) continue;
                const std::string candidate = item.second["bus"].as<std::string>();
                if(discovered_bus.empty()) discovered_bus = candidate;
                else if(discovered_bus != candidate) {
                    return tl::make_unexpected("Hardware Config 加载失败: ambiguous bus references");
                }
            }
            if(!discovered_bus.empty()) {
                summary.bus = discovered_bus;
            }
            else if(buses && buses.IsMap() && buses.size() == 1) {
                summary.bus = buses.begin()->first.as<std::string>();
            }
        }

        YAML::Node physical_node;
        if(!summary.bus.empty() && buses && buses.IsMap()) {
            physical_node = buses[summary.bus];
        }
        if(!physical_node || !physical_node.IsMap()) {
            return tl::make_unexpected("Hardware Config 加载失败: missing buses." + summary.bus);
        }

        const YAML::Node serial_node = physical_node["serial_port"] ?
            physical_node["serial_port"] : physical_node["device"];
        if(serial_node) summary.serial_port = serial_node.as<std::string>();
        if(physical_node["baudrate"]) summary.baudrate = physical_node["baudrate"].as<int>();

        if(overrides.serial_port) summary.serial_port = *overrides.serial_port;
        if(overrides.baudrate) summary.baudrate = *overrides.baudrate;
        return summary;
    }
    catch(const YAML::BadFile&) {
        return tl::make_unexpected("Hardware Config 加载失败: " + hardware_config);
    }
    catch(const YAML::Exception& error) {
        return tl::make_unexpected(std::string("Hardware Config 加载失败: ") + error.what());
    }
}

bool read_line(const std::string& prompt, std::string& line) {
    std::cout << prompt;
    std::cout.flush();
    return static_cast<bool>(std::getline(std::cin, line));
}

std::optional<int> read_int(const std::string& prompt) {
    std::string line;
    if(!read_line(prompt, line)) return std::nullopt;
    std::istringstream input(line);
    int value = 0;
    char extra = 0;
    if(!(input >> value) || (input >> extra)) return std::nullopt;
    return value;
}

std::optional<double> read_double(const std::string& prompt) {
    std::string line;
    if(!read_line(prompt, line)) return std::nullopt;
    std::istringstream input(line);
    double value = 0.0;
    char extra = 0;
    if(!(input >> value) || !std::isfinite(value) || (input >> extra)) return std::nullopt;
    return value;
}

std::optional<JointVector> read_vector(const std::string& prompt, std::size_t size) {
    std::string line;
    if(!read_line(prompt, line)) return std::nullopt;
    std::istringstream input(line);
    JointVector values(size, 0.0);
    for(double& value : values) {
        if(!(input >> value) || !std::isfinite(value)) return std::nullopt;
    }
    std::string extra;
    if(input >> extra) return std::nullopt;
    return values;
}

bool is_tracking_mode(JointImpedanceMode mode) {
    return mode == JointImpedanceMode::RIGID_TRACKING || mode == JointImpedanceMode::COMPLIANT_TRACKING;
}

bool is_zero_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return value == 0.0; });
}

class MockMotorBus final : public MotorBus {
public:
    explicit MockMotorBus(std::size_t size) {
        state_.pos.assign(size, 0.0);
        state_.vel.assign(size, 0.0);
        state_.tor.assign(size, 0.0);
        state_.online.assign(size, 1);
        state_.enabled.assign(size, 1);
        state_.err_code.assign(size, 0);
        capabilities_.resize(size);
        for(std::size_t i = 0; i < size; ++i) capabilities_[i].actuator_name = "mock" + std::to_string(i + 1);
    }

    tl::expected<void, MotorBusErr> configure(const std::string& config_path) override {
        static_cast<void>(config_path);
        return {};
    }

    tl::expected<void, MotorBusErr> connect() override {
        connected_ = true;
        return {};
    }

    tl::expected<ActuatorState, MotorBusErr> read() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        return state_;
    }

    tl::expected<void, MotorBusErr> activate() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        active_ = true;
        return {};
    }

    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
        state_.pos = cmd.pos;
        state_.vel = cmd.vel;
        state_.tor = cmd.tor;
        return {};
    }

    tl::expected<void, MotorBusErr> stop() override { return {}; }
    tl::expected<void, MotorBusErr> deactivate() override {
        active_ = false;
        return {};
    }
    tl::expected<void, MotorBusErr> recover() override { return {}; }
    const HardwareCapabilities& capabilities() const noexcept override { return capabilities_; }
    void cleanup() noexcept override {
        active_ = false;
        connected_ = false;
    }
    std::size_t size() const noexcept override { return state_.pos.size(); }

private:
    ActuatorState state_;
    HardwareCapabilities capabilities_;
    bool connected_{ false };
    bool active_{ false };
};

class TerminalApp {
public:
    TerminalApp(
        RobotCfg cfg,
        std::string config_path,
        std::string hardware_plugin,
        std::string hardware_config,
        HardwareConfigOverrides hardware_overrides,
        HardwareConnectionSummary connection_summary,
        std::string robot_profile,
        bool machine_mode = false)
        : cfg_(std::move(cfg)),
        config_path_(std::move(config_path)),
        hardware_plugin_(std::move(hardware_plugin)),
        hardware_config_(std::move(hardware_config)),
        hardware_overrides_(std::move(hardware_overrides)),
        connection_summary_(std::move(connection_summary)),
        robot_profile_(std::move(robot_profile)),
        machine_mode_(machine_mode) {

    }

    ~TerminalApp() {
        model_calibration_cancel_.store(true);
        model_calibration_teach_stop_.store(true);
        model_calibration_pause_.store(false);
        model_calibration_cv_.notify_all();
        cycle_cv_.notify_all();
        if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
        if(model_calibration_worker_.joinable()) model_calibration_worker_.join();
        model_calibration_recorder_.stop();
        machine_friction_stop_.store(true);
        cycle_cv_.notify_all();
        if(machine_friction_thread_.joinable()) machine_friction_thread_.join();
        stop_worker();
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() == RobotState::ACTIVE) {
            (void)robot_.deactivate();
        }
        else if(robot_.get_state() == RobotState::FAULT) {
            (void)robot_.force_deactivate();
        }
    }

    tl::expected<void, std::string> initialize() {
        const auto dynamics_result = dynamics_.configure(cfg_.dynamics);
        if(!dynamics_result) return tl::make_unexpected("Dynamics configure() 失败: " + to_string(dynamics_result.error()));
        model_calibration_links_cache_.clear();
        if(const auto links=dynamics_.get_gravity_regression(JointVector(cfg_.joint_names.size(),0.0)); links)
            model_calibration_links_cache_=links->links;

        RobotCfg robot_cfg = cfg_;
        if(!cfg_.dynamics.gravity_correction_path.empty()) {
            try {
                if(cfg_.runtime.model_feedforward_mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
                    return tl::make_unexpected(std::string("gravity correction is incompatible with FULL_INVERSE_DYNAMICS"));
                }
                if(cfg_.capability.admittance.observer.mode == AdmittanceObserverMode::FULL_ID) {
                    return tl::make_unexpected(std::string("gravity correction is incompatible with FULL_ID observer mode"));
                }
                const GravityCorrectionPackage package = load_gravity_correction_package(
                    cfg_.dynamics.gravity_correction_path, cfg_.joint_names, cfg_.dynamics.urdf_path);
                model_calibration_applied_original_gravity_scale_ = cfg_.dynamics.gravity_scale;
                model_calibration_applied_original_admittance_ = cfg_.capability.admittance;
                JointVector unit_scale(cfg_.joint_names.size(), 1.0);
                const auto scale_result = dynamics_.set_gravity_scale(unit_scale);
                if(!scale_result) return tl::make_unexpected(std::string("failed to activate gravity correction unit scale"));
                const auto override_result = dynamics_.set_gravity_first_moment_override(package.first_moments);
                if(!override_result) return tl::make_unexpected(std::string("failed to activate configured gravity correction"));
                robot_cfg.dynamics.gravity_scale = unit_scale;
                robot_cfg.capability.admittance.calibration.torque_bias = package.torque_bias;
                robot_cfg.capability.admittance.calibration.torque_threshold = package.torque_threshold;
                robot_cfg.capability.admittance.calibration.friction = package.friction;
                cfg_.dynamics.gravity_scale = unit_scale;
                cfg_.capability.admittance = robot_cfg.capability.admittance;
                model_calibration_candidate_applied_ = true;
                persistent_gravity_correction_loaded_ = true;
            }
            catch(const std::exception& error) {
                dynamics_.clear_gravity_first_moment_override();
                return tl::make_unexpected(std::string("gravity correction load failed: ") + error.what());
            }
        }

        auto hardware_result = hardware_loader_.load(hardware_plugin_, hardware_config_, hardware_overrides_);
        if(!hardware_result) return tl::make_unexpected("HardwareLoader 失败: " + to_string(hardware_result.error()));
        std::unique_ptr<MotorBus> hardware_bus = std::move(hardware_result.value());
        actuator_info_ = hardware_bus->capabilities();

        std::unique_ptr<MotorBus> motor_bus;
        if(cfg_.runtime.write_enabled) {
            motor_bus = std::move(hardware_bus);
        }
        else {
            robot_cfg.runtime.write_enabled = true;
            motor_bus = std::make_unique<MockMotorBus>(cfg_.joint_names.size());
        }
        ModelFeedforwardFn model_feedforward = [this](ModelFeedforwardMode mode, const JointState& state, const JointVector& acc, const JointVector& ref_acc, double) {
            const auto update_result = dynamics_.update(state, acc, ref_acc);
            if(!update_result) {
                last_dynamics_err_ = update_result.error();
                return tl::expected<JointVector, ModelFeedforwardErr>(tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED));
            }
            last_dynamics_err_.reset();

            switch(mode) {
                case ModelFeedforwardMode::NONE:
                    return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
                case ModelFeedforwardMode::GRAVITY:
                    return tl::expected<JointVector, ModelFeedforwardErr>(dynamics_.get_gravity_compensation());
                case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS:
                    return tl::expected<JointVector, ModelFeedforwardErr>(dynamics_.get_inverse_dynamics());
            }
            return tl::expected<JointVector, ModelFeedforwardErr>(tl::make_unexpected(ModelFeedforwardErr::INVALID_MODE));
            };

        InteractionModelStateFn interaction_model_state = [this](const JointState& state, double) -> tl::expected<InteractionModelState, ModelFeedforwardErr> {
            if(!dynamics_.is_updated() || dynamics_.get_state().pos != state.pos) {
                return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
            }
            InteractionModelState snapshot;
            snapshot.gravity = dynamics_.get_gravity_compensation();
            snapshot.coriolis = dynamics_.get_coriolis();
            const auto& mass = dynamics_.get_mass_matrix();
            if(mass.rows() != static_cast<Eigen::Index>(state.pos.size()) ||
                mass.cols() != static_cast<Eigen::Index>(state.pos.size())) {
                return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
            }
            snapshot.mass_matrix.assign(state.pos.size(), JointVector(state.pos.size(), 0.0));
            for(std::size_t i = 0; i < state.pos.size(); ++i) {
                for(std::size_t j = 0; j < state.pos.size(); ++j) {
                    snapshot.mass_matrix[i][j] = mass(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
                }
            }
            return snapshot;
            };

        const auto robot_result = robot_.configure(
            robot_cfg,
            std::move(motor_bus),
            std::move(model_feedforward),
            std::move(interaction_model_state));
        if(!robot_result) {
            std::ostringstream message;
            message << "Robot configure() 失败: " << to_string(robot_result.error().code);
            return tl::make_unexpected(message.str());
        }

        start_worker();
        return {};
    }

    int run() {
        print_banner();
        while(!quit_.load()) {
            print_menu();
            const auto selection = read_int("请选择: ");
            if(!selection) {
                if(std::cin.eof()) {
                    safe_exit();
                    break;
                }
                std::cout << "输入无效，请输入菜单编号\n";
                continue;
            }
            if(!handle_menu(*selection)) break;
        }
        return 0;
    }

    int run_machine() {
        std::streambuf* protocol_buffer = std::cout.rdbuf();
        struct StreamRestore {
            std::ostream& stream;
            std::streambuf* buffer;
            ~StreamRestore() { stream.rdbuf(buffer); }
        } restore{ std::cout, protocol_buffer };
        std::cout.rdbuf(std::cerr.rdbuf());
        std::ostream protocol(protocol_buffer);
        std::mutex protocol_mutex;
        std::atomic<bool> telemetry_running{ true };

        auto emit_reply = [&](long long id, const std::string& result, const std::string& error = std::string{}) {
            std::lock_guard<std::mutex> output_lock(protocol_mutex);
            protocol << "{\"id\":" << id << ',';
            if(error.empty()) protocol << "\"result\":" << result;
            else protocol << "\"error\":\"" << machine_json_escape(error) << "\"";
            protocol << "}\n" << std::flush;
        };

        std::thread telemetry([&]() {
            while(telemetry_running.load()) {
                try {
                    const std::string snapshot = machine_snapshot_json();
                    std::lock_guard<std::mutex> output_lock(protocol_mutex);
                    protocol << "{\"event\":\"telemetry\",\"time_ms\":"
                             << std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count()
                             << ",\"data\":" << snapshot << "}\n" << std::flush;
                }
                catch(...) {}
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        auto stop_telemetry = [&]() {
            telemetry_running.store(false);
            if(telemetry.joinable()) telemetry.join();
        };

        try {
            std::string line;
            while(std::getline(std::cin, line)) {
                if(line.empty()) continue;
                long long id = 0;
                try {
                    if(line.size() > 1024 * 1024) throw std::runtime_error("request exceeds 1 MiB");
                    const YAML::Node request = YAML::Load(line);
                    if(!request || !request.IsMap()) throw std::runtime_error("request must be an object");
                    if(request["id"]) id = request["id"].as<long long>();
                    if(!request["method"] || !request["method"].IsScalar()) throw std::runtime_error("method is required");
                    const std::string method = request["method"].as<std::string>();
                    const YAML::Node params = request["params"];

                    if(method == "status") {
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "activate") {
                        machine_activate();
                        emit_reply(id, "true");
                    }
                    else if(method == "deactivate") {
                        machine_deactivate();
                        emit_reply(id, "true");
                    }
                    else if(method == "park") {
                        // The demonstration is already stopped in this phase.
                        // Parking is an explicit operator decision to abandon
                        // pending replay, preserving its saved task data. Do
                        // not silently interrupt an active replay/recording.
                        if(model_calibration_phase_.load() == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)
                            machine_model_calibration_cancel();
                        machine_require_no_calibration("park");
                        park_and_deactivate();
                        if(machine_robot_state() != RobotState::INACTIVE) {
                            const std::string detail = park_last_error_.empty() ? "park did not reach INACTIVE" : park_last_error_;
                            throw std::runtime_error(detail);
                        }
                        emit_reply(id, "true");
                    }
                    else if(method == "clear_fault") {
                        machine_clear_fault();
                        emit_reply(id, "true");
                    }
                    else if(method == "fault_compliant") {
                        machine_fault_compliant();
                        emit_reply(id, "true");
                    }
                    else if(method == "fault_rigid") {
                        machine_fault_rigid();
                        emit_reply(id, "true");
                    }
                    else if(method == "set_impedance_mode") {
                        if(!params || !params["mode"]) throw std::runtime_error("mode is required");
                        machine_set_impedance_mode(machine_parse_impedance_mode(params["mode"].as<std::string>()));
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "set_model_feedforward_mode") {
                        if(!params || !params["mode"]) throw std::runtime_error("mode is required");
                        machine_set_model_feedforward_mode(machine_parse_feedforward_mode(params["mode"].as<std::string>()));
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "set_gravity_scale") {
                        machine_set_gravity_scale(machine_node_vector(params["values"], "values"));
                        emit_reply(id, "true");
                    }
                    else if(method == "move_to") {
                        const double speed_scale = params && params["speed_scale"] ? params["speed_scale"].as<double>() : 0.3;
                        machine_move_to(machine_node_vector(params["positions"], "positions"), speed_scale, false);
                        emit_reply(id, "true");
                    }
                    else if(method == "move_relative") {
                        const double speed_scale = params && params["speed_scale"] ? params["speed_scale"].as<double>() : 0.3;
                        machine_move_to(machine_node_vector(params["delta"], "delta"), speed_scale, true);
                        emit_reply(id, "true");
                    }
                    else if(method == "get_admittance") {
                        emit_reply(id, machine_admittance_json());
                    }
                    else if(method == "set_admittance") {
                        machine_set_admittance(params);
                        emit_reply(id, machine_admittance_json());
                    }
                    else if(method == "calibration_begin") {
                        if(!params || !params["kind"]) throw std::runtime_error("calibration kind is required");
                        machine_calibration_begin(params["kind"].as<std::string>());
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "calibration_capture") {
                        machine_calibration_capture();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "calibration_finish") {
                        machine_calibration_finish();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "calibration_cancel") {
                        machine_calibration_cancel();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "friction_record_begin") {
                        machine_friction_record_begin();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "friction_record_stop") {
                        machine_friction_record_stop();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "friction_replay_start") {
                        machine_friction_replay_start();
                        emit_reply(id, machine_calibration_status_json());
                    }
                    else if(method == "model_calibration_teach_begin") {
                        machine_model_calibration_teach_begin(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_import_trajectory") {
                        machine_model_calibration_import_trajectory(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_alignment_begin") {
                        machine_model_calibration_alignment_begin(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_alignment_finish") {
                        machine_model_calibration_alignment_finish(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_teach_stop") {
                        machine_model_calibration_teach_stop(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_start") {
                        machine_model_calibration_start(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_pause") {
                        machine_model_calibration_pause(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_resume") {
                        machine_model_calibration_resume(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_cancel") {
                        machine_model_calibration_cancel_request(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_status") {
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_apply") {
                        machine_model_calibration_apply(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "model_calibration_restore") {
                        machine_model_calibration_restore(params);
                        emit_reply(id, machine_model_calibration_status_json());
                    }
                    else if(method == "hold") {
                        machine_hold();
                        emit_reply(id, "true");
                    }
                    else if(method == "shutdown") {
                        if(machine_calibration_kind_ != "none") machine_calibration_cancel();
                        if(model_calibration_task_active()) { machine_model_calibration_cancel(); if(model_calibration_worker_.joinable()) model_calibration_worker_.join(); }
                        park_and_deactivate();
                        if(machine_robot_state() == RobotState::FAULT) machine_deactivate();
                        if(machine_robot_state() != RobotState::INACTIVE) throw std::runtime_error("shutdown could not deactivate Robot");
                        emit_reply(id, "true");
                        break;
                    }
                    else {
                        throw std::runtime_error("unknown request method");
                    }
                }
                catch(const std::exception& error) {
                    emit_reply(id, "null", error.what());
                }
            }
        }
        catch(...) {
            if(machine_friction_thread_.joinable()) {
                machine_friction_stop_.store(true); cycle_cv_.notify_all(); machine_friction_thread_.join();
            }
            stop_telemetry();
            throw;
        }

        if(machine_calibration_kind_ != "none") machine_calibration_cancel();
        if(machine_friction_thread_.joinable()) {
            machine_friction_stop_.store(true); cycle_cv_.notify_all(); machine_friction_thread_.join();
        }
        stop_telemetry();
        if(machine_robot_state() == RobotState::ACTIVE) park_and_deactivate();
        if(machine_robot_state() == RobotState::FAULT) machine_deactivate();
        quit_.store(true);
        return 0;
    }

private:
    static std::string machine_json_escape(const std::string& value) {
        std::ostringstream out;
        for(const unsigned char ch : value) {
            switch(ch) {
                case '\\': out << "\\\\"; break;
                case '"': out << "\\\""; break;
                case '\n': out << "\\n"; break;
                case '\r': out << "\\r"; break;
                case '\t': out << "\\t"; break;
                default:
                    if(ch < 0x20) {
                        out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(ch) << std::dec << std::setfill(' ');
                    }
                    else out << static_cast<char>(ch);
            }
        }
        return out.str();
    }

    static void machine_json_string(std::ostream& out, const std::string& value) {
        out << '"' << machine_json_escape(value) << '"';
    }

    template <typename T>
    static void machine_json_vector(std::ostream& out, const std::vector<T>& values) {
        out << '[';
        for(std::size_t i = 0; i < values.size(); ++i) {
            if(i) out << ',';
            out << +values[i];
        }
        out << ']';
    }

    static void machine_json_joint_vector(std::ostream& out, const JointVector& values) {
        out << '[';
        for(std::size_t i = 0; i < values.size(); ++i) {
            if(i) out << ',';
            out << std::setprecision(17) << values[i];
        }
        out << ']';
    }

    static JointVector machine_node_vector(const YAML::Node& node, const char* name) {
        if(!node || !node.IsSequence()) throw std::runtime_error(std::string(name) + " must be an array");
        JointVector values;
        values.reserve(node.size());
        for(const auto& item : node) {
            const double value = item.as<double>();
            if(!std::isfinite(value)) throw std::runtime_error(std::string(name) + " contains non-finite value");
            values.push_back(value);
        }
        return values;
    }

    static JointImpedanceMode machine_parse_impedance_mode(const std::string& value) {
        if(value == "RIGID_HOLD") return JointImpedanceMode::RIGID_HOLD;
        if(value == "RIGID_TRACKING") return JointImpedanceMode::RIGID_TRACKING;
        if(value == "COMPLIANT_HOLD") return JointImpedanceMode::COMPLIANT_HOLD;
        if(value == "COMPLIANT_DRAG") return JointImpedanceMode::COMPLIANT_DRAG;
        if(value == "COMPLIANT_TRACKING") return JointImpedanceMode::COMPLIANT_TRACKING;
        throw std::runtime_error("unknown impedance mode");
    }

    static ModelFeedforwardMode machine_parse_feedforward_mode(const std::string& value) {
        if(value == "NONE") return ModelFeedforwardMode::NONE;
        if(value == "GRAVITY") return ModelFeedforwardMode::GRAVITY;
        if(value == "FULL_INVERSE_DYNAMICS") return ModelFeedforwardMode::FULL_INVERSE_DYNAMICS;
        throw std::runtime_error("unknown model feedforward mode");
    }

    static std::runtime_error machine_robot_error(const char* action, const RobotFault& fault) {
        return std::runtime_error(std::string(action) + " failed: " + to_string(fault.code));
    }

    RobotState machine_robot_state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return robot_.get_state();
    }

    bool model_calibration_task_active() const {
        const auto phase = model_calibration_phase_.load();
        return phase != ModelCalibrationPhase::IDLE && phase != ModelCalibrationPhase::COMPLETE &&
            phase != ModelCalibrationPhase::CANCELLED && phase != ModelCalibrationPhase::FAILED;
    }

    void machine_require_no_calibration(const char* action) const {
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) {
            throw std::runtime_error(std::string(action) + " is blocked while a calibration task is active");
        }
    }

    void machine_activate() {
        machine_require_no_calibration("activate");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.activate();
        if(!result) throw machine_robot_error("activate", result.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_deactivate() {
        if(model_calibration_task_active()) {
            model_calibration_cancel_.store(true);
            model_calibration_teach_stop_.store(true);
            model_calibration_pause_.store(false);
            model_calibration_cv_.notify_all();
            cycle_cv_.notify_all();
        }
        else machine_require_no_calibration("deactivate");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.force_deactivate();
        if(!result) throw machine_robot_error("deactivate", result.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_clear_fault() {
        if(model_calibration_task_active()) {
            model_calibration_cancel_.store(true);
            model_calibration_pause_.store(false);
            model_calibration_cv_.notify_all();
        }
        else machine_require_no_calibration("clear_fault");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.clear_fault();
        if(!result) throw machine_robot_error("clear_fault", result.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_fault_compliant() {
        machine_require_no_calibration("fault_compliant");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.enter_fault_compliant_recovery();
        if(!result) throw machine_robot_error("fault_compliant", result.error());
        background_fault_reported_ = false;
    }

    void machine_fault_rigid() {
        machine_require_no_calibration("fault_rigid");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.return_to_fault_rigid_hold();
        if(!result) throw machine_robot_error("fault_rigid", result.error());
        background_fault_reported_ = false;
    }

    void machine_set_impedance_mode(JointImpedanceMode mode) {
        machine_require_no_calibration("set_impedance_mode");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.set_impedance_mode(mode);
        if(!result) throw machine_robot_error("set_impedance_mode", result.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_set_model_feedforward_mode(ModelFeedforwardMode mode) {
        machine_require_no_calibration("set_model_feedforward_mode");
        if(dynamics_.has_gravity_first_moment_override() && mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            throw std::runtime_error("FULL_INVERSE_DYNAMICS is unavailable while gravity correction is active");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result = robot_.set_model_feedforward_mode(mode);
        if(!result) throw machine_robot_error("set_model_feedforward_mode", result.error());
        cfg_.runtime.model_feedforward_mode = mode;
    }

    void machine_set_gravity_scale(const JointVector& values) {
        machine_require_no_calibration("set_gravity_scale");
        if(dynamics_.has_gravity_first_moment_override()) throw std::runtime_error("gravity_scale cannot be changed while gravity correction is active");
        if(values.size() != cfg_.joint_names.size()) throw std::runtime_error("gravity scale size does not match joint count");
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::INACTIVE) throw std::runtime_error("set_gravity_scale requires RobotState::INACTIVE");
        const auto result = dynamics_.set_gravity_scale(values);
        if(!result) throw std::runtime_error("set_gravity_scale failed: " + to_string(result.error()));
        cfg_.dynamics.gravity_scale = values;
    }

    void machine_validate_target(const JointVector& target, double speed_scale) const {
        if(target.size() != cfg_.joint_names.size()) throw std::runtime_error("target size does not match joint count");
        if(!std::isfinite(speed_scale) || speed_scale <= 0.0 || speed_scale > 1.0) throw std::runtime_error("speed_scale must be in (0, 1]");
        for(std::size_t i = 0; i < target.size(); ++i) {
            if(!std::isfinite(target[i])) throw std::runtime_error("target contains non-finite value");
            if(target[i] < cfg_.safety.limits.min_pos[i] || target[i] > cfg_.safety.limits.max_pos[i]) {
                throw std::runtime_error("target exceeds configured joint position limit: " + cfg_.joint_names[i]);
            }
        }
    }

    void machine_move_to(const JointVector& values, double speed_scale, bool relative) {
        machine_require_no_calibration("motion");
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("motion requires RobotState::ACTIVE");
        if(!is_tracking_mode(robot_.get_impedance_mode())) throw std::runtime_error("motion requires a tracking impedance mode");
        JointVector target = values;
        if(relative) {
            if(values.size() != cfg_.joint_names.size()) throw std::runtime_error("delta size does not match joint count");
            target = current_reference_pos();
            for(std::size_t i = 0; i < target.size(); ++i) target[i] += values[i];
        }
        machine_validate_target(target, speed_scale);
        begin_stream(target, speed_scale);
    }

    void machine_hold() {
        if(model_calibration_task_active()) model_calibration_pause_.store(true);
        else machine_require_no_calibration("hold");
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("hold requires RobotState::ACTIVE");
        clear_command_sources();
        const auto result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD);
        if(!result) throw machine_robot_error("hold", result.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    std::string machine_admittance_json() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& a = cfg_.capability.admittance;
        std::ostringstream out;
        out << std::setprecision(17) << "{\"enabled\":" << (a.enabled ? "true" : "false");
        out << ",\"observer_mode\":\"" << observer_mode_name(a.observer.mode) << "\"";
        out << ",\"joint_enabled\":"; machine_json_vector(out, a.joint_enabled);
        out << ",\"momentum_gain\":"; machine_json_joint_vector(out, a.observer.momentum_gain);
        out << ",\"mass\":"; machine_json_joint_vector(out, a.controller.mass);
        out << ",\"damping\":"; machine_json_joint_vector(out, a.controller.damping);
        out << ",\"stiffness\":"; machine_json_joint_vector(out, a.controller.stiffness);
        out << ",\"max_delta_q\":"; machine_json_joint_vector(out, a.controller.max_delta_q);
        out << ",\"max_delta_q_dot\":"; machine_json_joint_vector(out, a.controller.max_delta_q_dot);
        out << ",\"torque_bias\":"; machine_json_joint_vector(out, a.calibration.torque_bias);
        out << ",\"torque_threshold\":"; machine_json_joint_vector(out, a.calibration.torque_threshold);
        out << ",\"friction\":{\"enabled\":" << (a.calibration.friction.enabled ? "true" : "false")
            << ",\"velocity_transition\":" << a.calibration.friction.velocity_transition;
        out << ",\"positive_coulomb\":"; machine_json_joint_vector(out, a.calibration.friction.positive_coulomb);
        out << ",\"positive_viscous\":"; machine_json_joint_vector(out, a.calibration.friction.positive_viscous);
        out << ",\"negative_coulomb\":"; machine_json_joint_vector(out, a.calibration.friction.negative_coulomb);
        out << ",\"negative_viscous\":"; machine_json_joint_vector(out, a.calibration.friction.negative_viscous);
        out << "}}";
        return out.str();
    }

    void machine_set_admittance(const YAML::Node& params) {
        if(!params || !params.IsMap()) throw std::runtime_error("admittance params must be an object");
        AdmittanceCapabilityCfg candidate;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("set_admittance requires RobotState::ACTIVE");
            candidate = cfg_.capability.admittance;
        }
        auto set_vector = [&](const char* name, JointVector& target) {
            if(!params[name]) return;
            JointVector values = machine_node_vector(params[name], name);
            if(values.size() != cfg_.joint_names.size()) throw std::runtime_error(std::string(name) + " size does not match joint count");
            for(double value : values) if(!std::isfinite(value) || value <= 0.0) throw std::runtime_error(std::string(name) + " contains invalid value");
            target = std::move(values);
        };
        set_vector("mass", candidate.controller.mass);
        set_vector("damping", candidate.controller.damping);
        set_vector("stiffness", candidate.controller.stiffness);
        set_vector("max_delta_q", candidate.controller.max_delta_q);
        set_vector("max_delta_q_dot", candidate.controller.max_delta_q_dot);
        set_vector("momentum_gain", candidate.observer.momentum_gain);
        if(params["observer_mode"]) {
            const std::string mode = params["observer_mode"].as<std::string>();
            if(mode == "FULL_ID") candidate.observer.mode = AdmittanceObserverMode::FULL_ID;
            else if(mode == "MOMENTUM") candidate.observer.mode = AdmittanceObserverMode::MOMENTUM;
            else throw std::runtime_error("unknown observer mode");
        }
        if(dynamics_.has_gravity_first_moment_override() && candidate.observer.mode == AdmittanceObserverMode::FULL_ID) {
            throw std::runtime_error("FULL_ID observer is unavailable while gravity correction is active");
        }
        if(!apply_runtime_admittance_cfg(candidate)) throw std::runtime_error("set_admittance failed");
    }

    void machine_restore_calibration_runtime(bool restore_gravity) {
        if(!machine_calibration_context_valid_) return;
        if(restore_gravity) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = dynamics_.set_gravity_scale(machine_calibration_original_gravity_);
            if(result) cfg_.dynamics.gravity_scale = machine_calibration_original_gravity_;
        }
        set_tuning_impedance_mode(machine_calibration_original_mode_);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            robot_.set_admittance_suspended(machine_calibration_original_suspended_);
            last_output_.reset();
        }
    }

    static std::string machine_model_calibration_task_id() {
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        return "model-calibration-" + std::to_string(now);
    }

    static std::vector<std::string> machine_resource_paths() {
        std::vector<std::string> result;
        const char* value = std::getenv("SERIAL_ARM_RESOURCE_PATH");
        if(!value) return result;
        std::stringstream stream(value);
        std::string item;
        while(std::getline(stream, item, ':')) if(!item.empty()) result.push_back(item);
        return result;
    }

    void model_calibration_set_phase(ModelCalibrationPhase phase) {
        model_calibration_phase_.store(phase);
    }

    void model_calibration_update_progress(std::size_t completed, std::size_t total) {
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        model_calibration_completed_units_ = completed;
        model_calibration_total_units_ = total;
        model_calibration_progress_ = total ? static_cast<double>(completed) / static_cast<double>(total) : 0.0;
    }

    ModelCalibrationMetadata model_calibration_metadata() const {
        ModelCalibrationMetadata metadata;
        metadata.task_id = model_calibration_task_id_;
        metadata.profile = robot_profile_;
        metadata.core_config_path = config_path_;
        metadata.core_fingerprint = model_calibration_core_fingerprint_.empty() ? model_calibration_file_fingerprint(config_path_) : model_calibration_core_fingerprint_;
        metadata.urdf_path = cfg_.dynamics.urdf_path;
        metadata.urdf_fingerprint = model_calibration_urdf_fingerprint_.empty() ? model_calibration_file_fingerprint(cfg_.dynamics.urdf_path) : model_calibration_urdf_fingerprint_;
        metadata.resource_paths = machine_resource_paths();
        metadata.joint_names = cfg_.joint_names;
        metadata.base_frame = cfg_.dynamics.base_frame;
        metadata.tool_frame = cfg_.dynamics.tool_frame;
        metadata.gravity = cfg_.dynamics.gravity;
        metadata.original_gravity_scale = model_calibration_original_gravity_scale_;
        metadata.mapping_pos_ratio = cfg_.mapper.pos_ratio;
        metadata.mapping_tor_ratio = cfg_.mapper.tor_ratio;
        metadata.mapping_direction = cfg_.mapper.direction;
        metadata.mapping_joint_zero_offset = cfg_.mapper.joint_zero_offset;
        metadata.mapping_actuator_zero_offset = cfg_.mapper.actuator_zero_offset;
        metadata.load_description = "unspecified";
        metadata.pose_budget = model_calibration_options_.pose_budget;
        metadata.validation_fraction = model_calibration_options_.validation_fraction;
        metadata.max_com_offset_m = model_calibration_options_.max_com_offset_m;
        metadata.regularization = model_calibration_options_.regularization;
        metadata.svd_relative_threshold = model_calibration_options_.svd_relative_threshold;
        metadata.minimum_information_score = model_calibration_options_.minimum_information_score;
        metadata.identification_mode = model_calibration_options_.identification_mode;
        metadata.locked_links = model_calibration_options_.locked_links;
        for(const auto& link:model_calibration_links_cache_) metadata.identifiable_links.push_back(link.link_name);
        metadata.max_task_duration_s = 0.0; // 0: no user-configurable task deadline
        return metadata;
    }

    void model_calibration_restore_runtime_hold() {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_.set_admittance_suspended(model_calibration_original_suspended_);
        last_output_.reset();
    }

    void model_calibration_reset_task_state() {
        model_calibration_cancel_.store(false);
        model_calibration_pause_.store(false);
        model_calibration_resume_confirmed_.store(false);
        model_calibration_teach_stop_.store(false);
        model_calibration_pose_group_.store(0);
        model_calibration_validation_.store(false);
        model_calibration_direction_.store(0);
        model_calibration_speed_tier_.store(0);
        model_calibration_resume_requires_confirmation_ = false;
        model_calibration_error_.clear();
        model_calibration_trajectory_.reset();
        model_calibration_alignment_active_.store(false);
        model_calibration_source_reversed_.store(false);
        model_calibration_source_task_id_.clear();
        model_calibration_pose_targets_.clear();
        model_calibration_result_.reset();
        model_calibration_friction_result_.reset();
        model_calibration_friction_pass_ = false;
        model_calibration_candidate_applied_ = false;
        model_calibration_valid_static_samples_ = 0;
        model_calibration_estimated_duration_s_ = 0.0;
        model_calibration_original_duration_s_ = 0.0;
        model_calibration_path_length_rad_ = 0.0;
        model_calibration_geometric_waypoints_ = 0;
        model_calibration_original_samples_ = 0;
        model_calibration_replay_rate_ = 0.0;
        model_calibration_teaching_wall_duration_s_ = 0.0;
        model_calibration_teaching_started_at_.reset();
        model_calibration_started_at_.reset();
        model_calibration_update_progress(0, 0);
    }

    void machine_model_calibration_teach_begin(const YAML::Node& params) {
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) {
            throw std::runtime_error("another calibration task is active");
        }
        if(dynamics_.has_gravity_first_moment_override()) {
            throw std::runtime_error("restore the active gravity correction before starting a new model calibration task");
        }
        if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
        if(model_calibration_worker_.joinable()) model_calibration_worker_.join();
        model_calibration_recorder_.stop();
        model_calibration_reset_task_state();
        if(params && params["pose_budget"]) model_calibration_options_.pose_budget = std::clamp<std::size_t>(params["pose_budget"].as<std::size_t>(), 5, 16);
        if(params && params["validation_fraction"]) model_calibration_options_.validation_fraction = std::clamp(params["validation_fraction"].as<double>(), 0.15, 0.45);
        if(params && params["max_com_offset_m"]) model_calibration_options_.max_com_offset_m = std::clamp(params["max_com_offset_m"].as<double>(), 0.005, 0.20);
        if(params && params["regularization"]) model_calibration_options_.regularization = std::clamp(params["regularization"].as<double>(), 1.0e-8, 10.0);
        if(params && params["svd_relative_threshold"]) model_calibration_options_.svd_relative_threshold = std::clamp(params["svd_relative_threshold"].as<double>(), 1.0e-8, 0.2);
        if(params && params["minimum_information_score"]) model_calibration_options_.minimum_information_score = std::clamp(params["minimum_information_score"].as<double>(), 1.0e-8, 1.0e3);
        if(params && params["mode"]) model_calibration_options_.identification_mode=params["mode"].as<std::string>();
        if(params && params["locked_links"]) model_calibration_options_.locked_links=params["locked_links"].as<std::vector<std::string>>();
        if(model_calibration_options_.identification_mode!="global" && model_calibration_options_.identification_mode!="local") throw std::runtime_error("invalid identification mode");
        if(model_calibration_options_.identification_mode=="global") model_calibration_options_.locked_links.clear();
        if(model_calibration_links_cache_.empty()) throw std::runtime_error("no identifiable gravity links");
        std::set<std::string> choices;
        for(const auto& item:model_calibration_links_cache_) choices.insert(item.link_name);
        std::set<std::string> unique;
        for(const auto& name:model_calibration_options_.locked_links) {
            if(!choices.count(name) || !unique.insert(name).second) throw std::runtime_error("unknown or duplicate locked gravity link: "+name);
        }
        if(choices.size()<=unique.size()) throw std::runtime_error("at least one identifiable gravity link must remain unlocked");

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("model calibration teaching requires RobotState::ACTIVE");
            if(!dynamics_.is_updated()) throw std::runtime_error("dynamics is not ready");
            model_calibration_original_mode_ = robot_.get_impedance_mode();
            model_calibration_original_feedforward_ = robot_.get_model_feedforward_mode();
            model_calibration_original_admittance_ = cfg_.capability.admittance;
            model_calibration_original_gravity_scale_ = dynamics_.get_gravity_scale();
            model_calibration_original_suspended_ = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        model_calibration_task_id_ = machine_model_calibration_task_id();
        model_calibration_core_fingerprint_ = model_calibration_file_fingerprint(config_path_);
        model_calibration_urdf_fingerprint_ = model_calibration_file_fingerprint(cfg_.dynamics.urdf_path);
        const std::filesystem::path root = std::filesystem::current_path() / ".install" / "model-calibration";
        model_calibration_directory_ = (root / model_calibration_task_id_).string();
        const auto recorder = model_calibration_recorder_.start(model_calibration_directory_, model_calibration_metadata());
        if(!recorder) throw std::runtime_error(recorder.error());
        try {
            std::error_code source_error;
            const std::filesystem::path source_dir = std::filesystem::path(model_calibration_directory_) / "source";
            std::filesystem::create_directories(source_dir, source_error);
            if(source_error) throw std::runtime_error("failed to create model calibration source snapshot directory: " + source_error.message());
            std::filesystem::copy_file(config_path_, source_dir / "core.yaml", std::filesystem::copy_options::overwrite_existing, source_error);
            if(source_error) throw std::runtime_error("failed to snapshot core configuration: " + source_error.message());
            source_error.clear();
            std::filesystem::copy_file(cfg_.dynamics.urdf_path, source_dir / "model.urdf", std::filesystem::copy_options::overwrite_existing, source_error);
            if(source_error) throw std::runtime_error("failed to snapshot source URDF: " + source_error.message());
        } catch(...) {
            model_calibration_recorder_.stop();
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw;
        }
        model_calibration_set_phase(ModelCalibrationPhase::TEACHING);
        model_calibration_teaching_started_at_ = Robot::Clock::now();
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            model_calibration_recorder_.stop();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw std::runtime_error("failed to enter COMPLIANT_DRAG");
        }
        model_calibration_teach_thread_ = std::thread([this]() {
            // Preserve the already-validated ACTIVE-cycle samples if the robot
            // faults during teaching. This is file-only recovery: no motor
            // command or fault clear is ever issued by this thread.
            auto captured = collect_friction_drag_trajectory_until_stop(model_calibration_teach_stop_,
                [this](const FrictionCalibrationTrajectory& partial) {
                    const auto saved = model_calibration_recorder_.write_trajectory_checkpoint(partial.positions, partial.sample_dt);
                    if(!saved) std::cerr << "[示教检查点] " << saved.error() << '\n';
                });
            if(captured && captured->positions.size() >= 20) {
                const auto checkpoint = model_calibration_recorder_.write_trajectory_checkpoint(captured->positions, captured->sample_dt);
                if(!checkpoint) std::cerr << "[示教检查点] 最终保存失败: " << checkpoint.error() << '\n';
            }
            if(machine_robot_state() == RobotState::FAULT || model_calibration_fault_interrupted_.load()) {
                if(captured && captured->positions.size() >= 20) {
                    const auto saved = model_calibration_recorder_.write_trajectory(
                        captured->positions, captured->sample_dt);
                    if(!saved) std::cerr << "[示教故障] 保存中断轨迹失败: " << saved.error() << '\n';
                    else {
                        std::ofstream marker(std::filesystem::path(model_calibration_directory_) / "interrupted.txt");
                        if(marker) marker << "Teaching interrupted by robot fault. Saved data is a partial demonstration, not a calibrated model.\n";
                        std::cout << "[示教故障] 已保留故障前轨迹，需清除故障并重新进行安全检查后才能导入；不会自动回放\n";
                    }
                }
                model_calibration_recorder_.stop();
            }
            model_calibration_trajectory_ = std::move(captured);
        });
    }

    std::vector<ModelCalibrationPoseTarget> model_calibration_select_pose_targets(const FrictionCalibrationTrajectory& trajectory) {
        struct Candidate { std::size_t index; JointVector q; Eigen::VectorXd feature; double norm; };
        std::vector<Candidate> candidates;
        if(trajectory.positions.empty()) return {};
        const std::size_t stride = std::max<std::size_t>(1, trajectory.positions.size() / 48);
        for(std::size_t index = 0; index < trajectory.positions.size(); index += stride) {
            const JointVector& q = trajectory.positions[index];
            bool duplicate = false;
            for(const auto& existing : candidates) {
                if(existing.q.size() != q.size()) continue;
                double max_delta = 0.0;
                for(std::size_t joint = 0; joint < q.size(); ++joint) max_delta = std::max(max_delta, std::abs(existing.q[joint] - q[joint]));
                if(max_delta <= 1.0e-4) { duplicate = true; break; }
            }
            if(duplicate) continue;
            const auto regression = dynamics_.get_gravity_regression(q);
            if(!regression || regression->first_moment_regressor.cols() == 0) continue;
            Eigen::MatrixXd scaled = regression->first_moment_regressor;
            if(model_calibration_options_.identification_mode=="local") {
                for(std::size_t link=0;link<regression->links.size();++link)
                    if(std::find(model_calibration_options_.locked_links.begin(),model_calibration_options_.locked_links.end(),regression->links[link].link_name)!=model_calibration_options_.locked_links.end())
                        scaled.block(0,static_cast<Eigen::Index>(3*link),scaled.rows(),3).setZero();
            }
            for(std::size_t link = 0; link < regression->links.size(); ++link) {
                const double parameter_scale = std::max(1.0e-4, regression->links[link].mass * 0.05); // Geometry reference for information scoring; NOT a COM limit
                scaled.block(0, static_cast<Eigen::Index>(3 * link), scaled.rows(), 3) *= parameter_scale;
            }
            Eigen::Map<const Eigen::VectorXd> flat(scaled.data(), scaled.size());
            Candidate item{index, q, flat, flat.norm()};
            if(std::isfinite(item.norm) && item.norm >= model_calibration_options_.minimum_information_score) candidates.push_back(std::move(item));
        }
        if(candidates.size() < model_calibration_options_.minimum_training_groups + 1) return {};
        const std::size_t desired = std::min(model_calibration_options_.pose_budget, candidates.size());
        std::vector<std::size_t> chosen;
        chosen.reserve(desired);
        auto first = std::max_element(candidates.begin(), candidates.end(), [](const auto& a, const auto& b){ return a.norm < b.norm; });
        chosen.push_back(static_cast<std::size_t>(std::distance(candidates.begin(), first)));
        while(chosen.size() < desired) {
            double best = -1.0;
            std::size_t best_index = 0;
            for(std::size_t i = 0; i < candidates.size(); ++i) {
                if(std::find(chosen.begin(), chosen.end(), i) != chosen.end()) continue;
                double min_distance = std::numeric_limits<double>::infinity();
                for(const auto selected : chosen) {
                    min_distance = std::min(min_distance, (candidates[i].feature - candidates[selected].feature).norm());
                }
                if(min_distance > best) { best = min_distance; best_index = i; }
            }
            if(best <= model_calibration_options_.minimum_information_score) break;
            chosen.push_back(best_index);
        }
        if(chosen.size() < model_calibration_options_.minimum_training_groups + 1) return {};
        std::sort(chosen.begin(), chosen.end(), [&](std::size_t a, std::size_t b){ return candidates[a].index < candidates[b].index; });
        const std::size_t validation_count = std::max<std::size_t>(1, static_cast<std::size_t>(std::round(chosen.size() * model_calibration_options_.validation_fraction)));
        std::vector<ModelCalibrationPoseTarget> result;
        result.reserve(chosen.size());
        for(std::size_t i = 0; i < chosen.size(); ++i) {
            const bool validation = validation_count > 0 && ((i + 1) * validation_count / chosen.size() > i * validation_count / chosen.size());
            result.push_back(ModelCalibrationPoseTarget{candidates[chosen[i]].index, i + 1, validation, candidates[chosen[i]].norm});
        }
        if(static_cast<std::size_t>(std::count_if(result.begin(), result.end(), [](const auto& x){return !x.validation;})) < model_calibration_options_.minimum_training_groups) {
            for(auto it = result.rbegin(); it != result.rend(); ++it) {
                if(it->validation) { it->validation = false; break; }
            }
        }
        return result;
    }

    void model_calibration_require_task(const YAML::Node& params) const {
        if(model_calibration_task_id_.empty()) throw std::runtime_error("model calibration task is not initialized");
        if(!params || !params["task_id"] || params["task_id"].as<std::string>() != model_calibration_task_id_) {
            throw std::runtime_error("stale or missing model calibration task_id");
        }
    }

    // A recorded trajectory is a *new* task, never a resumed actuator command.
    // Loading it never sends a motion reference; the operator must subsequently
    // confirm model_calibration_start in the usual safety dialog.
    void machine_model_calibration_import_trajectory(const YAML::Node& params) {
        if(!params || !params["directory"]) throw std::runtime_error("task directory is required");
        if(machine_calibration_kind_ != "none" || model_calibration_task_active())
            throw std::runtime_error("finish or cancel the current calibration task before importing another");
        if(dynamics_.has_gravity_first_moment_override())
            throw std::runtime_error("restore active gravity correction before importing a task");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE || !dynamics_.is_updated())
                throw std::runtime_error("trajectory import requires ACTIVE robot and ready dynamics; it does not move the robot");
        }
        const auto directory = std::filesystem::canonical(std::filesystem::path(params["directory"].as<std::string>()));
        if(!std::filesystem::is_directory(directory)) throw std::runtime_error("task folder is missing");
        const YAML::Node meta = YAML::LoadFile((directory / "metadata.json").string());
        if(!meta || !meta["task_id"] || meta["task_id"].as<std::string>().empty())
            throw std::runtime_error("invalid recorded task identifier");
        const auto source_task_id = meta["task_id"].as<std::string>();
        if(!meta["core_fingerprint"] || !meta["urdf_fingerprint"] ||
           meta["core_fingerprint"].as<std::string>() != model_calibration_file_fingerprint(config_path_) ||
           meta["urdf_fingerprint"].as<std::string>() != model_calibration_file_fingerprint(cfg_.dynamics.urdf_path))
            throw std::runtime_error("recorded Core/URDF fingerprints differ from active robot; restore matching configuration before replay");
        if(meta["profile"] && !robot_profile_.empty() && meta["profile"].as<std::string>() != robot_profile_)
            throw std::runtime_error("recorded Profile differs from active robot");
        if(!meta["joint_names"] || !meta["joint_names"].IsSequence() ||
           meta["joint_names"].size() != cfg_.joint_names.size())
            throw std::runtime_error("recorded joint count does not match active robot");
        for(std::size_t i=0;i<cfg_.joint_names.size();++i)
            if(meta["joint_names"][i].as<std::string>() != cfg_.joint_names[i])
                throw std::runtime_error("recorded joint ordering does not match active robot");
        // The operator may adjust identification options before staging an old
        // trajectory; apply the same bounded options as normal teaching.
        if(params["pose_budget"]) model_calibration_options_.pose_budget = std::clamp<std::size_t>(params["pose_budget"].as<std::size_t>(), 5, 16);
        if(params["validation_fraction"]) model_calibration_options_.validation_fraction = std::clamp(params["validation_fraction"].as<double>(), 0.15, 0.45);
        if(params["max_com_offset_m"]) model_calibration_options_.max_com_offset_m = std::clamp(params["max_com_offset_m"].as<double>(), 0.005, 0.20);
        if(params["regularization"]) model_calibration_options_.regularization = std::clamp(params["regularization"].as<double>(), 1.0e-8, 10.0);
        if(params["svd_relative_threshold"]) model_calibration_options_.svd_relative_threshold = std::clamp(params["svd_relative_threshold"].as<double>(), 1.0e-8, 0.2);
        if(params["minimum_information_score"]) model_calibration_options_.minimum_information_score = std::clamp(params["minimum_information_score"].as<double>(), 1.0e-8, 1.0e3);
        if(params["mode"]) model_calibration_options_.identification_mode=params["mode"].as<std::string>();
        if(params["locked_links"]) model_calibration_options_.locked_links=params["locked_links"].as<std::vector<std::string>>();
        if(model_calibration_options_.identification_mode!="global" && model_calibration_options_.identification_mode!="local") throw std::runtime_error("invalid identification mode");
        if(model_calibration_options_.identification_mode=="global") model_calibration_options_.locked_links.clear();
        if(model_calibration_links_cache_.empty()) throw std::runtime_error("no identifiable gravity links");
        std::set<std::string> choices;
        for(const auto& item:model_calibration_links_cache_) choices.insert(item.link_name);
        std::set<std::string> unique;
        for(const auto& name:model_calibration_options_.locked_links) {
            if(!choices.count(name) || !unique.insert(name).second) throw std::runtime_error("unknown or duplicate locked gravity link: "+name);
        }
        if(choices.size()<=unique.size()) throw std::runtime_error("at least one identifiable gravity link must remain unlocked");
        // Protect even against metadata with manually edited joint calibration;
        // the entire Core fingerprint above includes the joint mapping.
        const auto trajectory_path = std::filesystem::is_regular_file(directory / "trajectory.csv") ?
            directory / "trajectory.csv" : directory / "trajectory.checkpoint.csv";
        // Checkpoints are partial, unverified teaching recordings. Apply all
        // fingerprint, joint-order, finite/range and retiming checks below.
        std::ifstream csv(trajectory_path);
        if(!csv) throw std::runtime_error("neither trajectory.csv nor trajectory.checkpoint.csv exists");
        std::string line, field;
        if(!std::getline(csv,line)) throw std::runtime_error("trajectory sample_dt header is missing");
        std::istringstream head(line);
        std::getline(head,field,',');
        if(field != "sample_dt" || !std::getline(head,field) || field.empty())
            throw std::runtime_error("invalid trajectory sample_dt header");
        FrictionCalibrationTrajectory candidate;
        candidate.sample_dt = std::stod(field);
        if(!std::isfinite(candidate.sample_dt) || candidate.sample_dt <= 0 || candidate.sample_dt > 0.5)
            throw std::runtime_error("invalid trajectory sample interval");
        if(!std::getline(csv,line)) throw std::runtime_error("trajectory columns are missing");
        std::ostringstream expected_header;
        expected_header << "index";
        for(const auto& name : cfg_.joint_names) expected_header << ',' << name;
        if(line != expected_header.str()) throw std::runtime_error("trajectory joint names do not match recorded metadata");
        while(std::getline(csv,line)) {
            if(line.empty()) throw std::runtime_error("empty row in trajectory.csv");
            std::istringstream row(line);
            if(!std::getline(row,field,',') || field != std::to_string(candidate.positions.size()))
                throw std::runtime_error("trajectory frame index is invalid");
            JointVector q;
            for(std::size_t j=0;j<cfg_.joint_names.size();++j) {
                if(!std::getline(row,field,',') || field.empty()) throw std::runtime_error("trajectory joint column is missing");
                std::size_t parsed=0;
                const double value=std::stod(field,&parsed);
                if(parsed != field.size() || !std::isfinite(value)) throw std::runtime_error("non-finite or invalid trajectory joint position");
                q.push_back(value);
            }
            if(std::getline(row,field,',')) throw std::runtime_error("trajectory row contains extra joint columns");
            candidate.positions.push_back(std::move(q));
            if(candidate.positions.size() > 300000) throw std::runtime_error("trajectory sample count exceeds import safety limit");
        }
        if(candidate.positions.size() < 2 || !friction_trajectory_inside_safe_replay_range(candidate))
            throw std::runtime_error("imported trajectory is too short or exceeds joint safety margins");
        // Prefer whichever recorded endpoint is already close to the robot.
        // Either direction traverses the exact same demonstrated geometric path;
        // this does NOT generate an unvalidated path from park to an endpoint.
        serial_arm::calibration_recovery::Choice endpoint_choice;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(last_output_ && robot_.get_state()==RobotState::ACTIVE && feedback_time_ns_>0) {
                const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                const double age_ms=now_ns>=feedback_time_ns_ ? static_cast<double>(now_ns-feedback_time_ns_)/1.0e6 : std::numeric_limits<double>::infinity();
                endpoint_choice=serial_arm::calibration_recovery::select_endpoint(
                    last_output_->joint_state.pos,last_output_->joint_state.vel,
                    candidate.positions.front(),candidate.positions.back(),age_ms);
            }
        }
        // Feedback is mandatory to choose the safe trajectory orientation.
        // Do not silently assume the original endpoint when feedback is missing.
        if(!endpoint_choice.valid)
            throw std::runtime_error("fresh robot feedback is required before importing a replay trajectory");
        if(endpoint_choice.reverse_recorded_path)
            std::reverse(candidate.positions.begin(),candidate.positions.end());
        // Always retime with *current* safety limits; never trust timestamps in
        // an imported CSV as a guarantee of velocity/acceleration feasibility.
        const double recorded_duration = candidate.sample_dt * static_cast<double>(candidate.positions.size()-1);
        const auto planned = serial_arm::calibration_retime::plan(candidate.positions,
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,cfg_.runtime.ctrl_frequency_hz);
        candidate.positions = planned.positions;
        candidate.sample_dt = planned.sample_dt;
        if(!friction_trajectory_inside_safe_replay_range(candidate))
            throw std::runtime_error("retimed imported trajectory exceeds safe joint range");
        FrictionCalibrationTrajectory sampled;
        sampled.sample_dt = candidate.sample_dt / 4.0;
        for(std::size_t k=0;k+1<candidate.positions.size();++k)
            for(int n=0;n<4;++n)
                sampled.positions.push_back(serial_arm::calibration_retime::interpolate(
                    candidate.positions,static_cast<double>(k)+0.25*n,candidate.sample_dt,1.0).position);
        sampled.positions.push_back(candidate.positions.back());
        if(!friction_trajectory_inside_safe_replay_range(sampled))
            throw std::runtime_error("interpolated trajectory violates joint position margins");
        const auto targets = model_calibration_select_pose_targets(candidate);
        if(targets.size() < model_calibration_options_.minimum_training_groups + 1)
            throw std::runtime_error("saved trajectory does not contain enough independent gravity poses");
        // Validation succeeded: create a distinct writable task, preserving
        // source metadata, frames and result byte-for-byte.
        if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
        if(model_calibration_worker_.joinable()) model_calibration_worker_.join();
        model_calibration_recorder_.stop();
        model_calibration_reset_task_state();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("robot state changed during import");
            model_calibration_original_mode_ = robot_.get_impedance_mode();
            model_calibration_original_feedforward_ = robot_.get_model_feedforward_mode();
            model_calibration_original_admittance_ = cfg_.capability.admittance;
            model_calibration_original_gravity_scale_ = dynamics_.get_gravity_scale();
            model_calibration_original_suspended_ = robot_.is_admittance_suspended();
        }
        model_calibration_source_reversed_.store(endpoint_choice.reverse_recorded_path);
        model_calibration_source_task_id_=source_task_id;
        model_calibration_task_id_=machine_model_calibration_task_id();
        model_calibration_core_fingerprint_=model_calibration_file_fingerprint(config_path_);
        model_calibration_urdf_fingerprint_=model_calibration_file_fingerprint(cfg_.dynamics.urdf_path);
        model_calibration_directory_=(std::filesystem::current_path()/".install"/"model-calibration"/model_calibration_task_id_).string();
        const auto started=model_calibration_recorder_.start(model_calibration_directory_,model_calibration_metadata());
        if(!started) throw std::runtime_error(started.error());
        try {
            const auto source=std::filesystem::path(model_calibration_directory_)/"source";
            std::filesystem::create_directories(source);
            std::filesystem::copy_file(config_path_,source/"core.yaml");
            std::filesystem::copy_file(cfg_.dynamics.urdf_path,source/"model.urdf");
            std::ofstream origin(std::filesystem::path(model_calibration_directory_)/"imported-from.txt");
            origin << directory.string() << '\n';
            if(!origin) throw std::runtime_error("failed to write trajectory provenance");
            auto written=model_calibration_recorder_.write_trajectory(candidate.positions,candidate.sample_dt);
            if(!written) throw std::runtime_error(written.error());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                robot_.set_admittance_suspended(true);
                last_output_.reset();
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("cannot enter RIGID_HOLD before playback confirmation");
        } catch(...) {
            model_calibration_recorder_.stop();
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw;
        }
        model_calibration_trajectory_=std::move(candidate);
        model_calibration_pose_targets_=targets;
        model_calibration_replay_rate_=1.0;
        model_calibration_original_duration_s_=recorded_duration;
        model_calibration_path_length_rad_=planned.joint_path_length_rad;
        model_calibration_geometric_waypoints_=planned.geometric_waypoints;
        model_calibration_original_samples_=planned.original_samples;
        const double static_seconds=targets.size()*(model_calibration_options_.static_hold_s + model_calibration_options_.static_sample_s + 0.5);
        model_calibration_estimated_duration_s_=2.0*model_calibration_trajectory_->sample_dt*(model_calibration_trajectory_->positions.size()-1)+static_seconds+2.0;
        model_calibration_update_progress(0,targets.size()+3);
        model_calibration_set_phase(ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION);
    }

    void machine_model_calibration_teach_stop(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load() != ModelCalibrationPhase::TEACHING) throw std::runtime_error("model calibration teaching is not active");
        if(model_calibration_teaching_started_at_) {
            model_calibration_teaching_wall_duration_s_ = std::chrono::duration<double>(
                Robot::Clock::now() - *model_calibration_teaching_started_at_).count();
        }
        model_calibration_teach_stop_.store(true);
        cycle_cv_.notify_all();
        if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
        if(!model_calibration_trajectory_) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = "demonstration trajectory is too short";
            model_calibration_recorder_.stop();
            // Teaching runs in COMPLIANT_DRAG. A rejected trajectory must not
            // leave that mode active merely because the GUI request failed.
            // Never try to override a latched robot FAULT.
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        *model_calibration_trajectory_ = smooth_friction_trajectory(*model_calibration_trajectory_);
        if(!friction_trajectory_inside_safe_replay_range(*model_calibration_trajectory_)) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = "demonstration trajectory exceeds safe replay range";
            model_calibration_recorder_.stop();
            // Teaching runs in COMPLIANT_DRAG. A rejected trajectory must not
            // leave that mode active merely because the GUI request failed.
            // Never try to override a latched robot FAULT.
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        model_calibration_original_duration_s_ = model_calibration_trajectory_->sample_dt *
            static_cast<double>(model_calibration_trajectory_->positions.size() - 1);
        try {
            const auto planned = serial_arm::calibration_retime::plan(
                model_calibration_trajectory_->positions,
                cfg_.safety.limits.max_vel,
                cfg_.safety.limits.max_acc,
                cfg_.runtime.ctrl_frequency_hz);
            {
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                model_calibration_path_length_rad_ = planned.joint_path_length_rad;
                model_calibration_geometric_waypoints_ = planned.geometric_waypoints;
                model_calibration_original_samples_ = planned.original_samples;
            }
            model_calibration_trajectory_->positions = planned.positions;
            model_calibration_trajectory_->sample_dt = planned.sample_dt;
            // The newly time-parameterized trajectory is safe at rate 1.0.
            // The two slow friction passes use rate 0.5, with extra margin.
            model_calibration_replay_rate_ = 1.0;
            if(!friction_trajectory_inside_safe_replay_range(*model_calibration_trajectory_))
                throw std::runtime_error("retimed trajectory exceeds safe replay range");
            // Validate the actual spline at quarter intervals: a continuous
            // Hermite spline may overshoot positions even if its knots are safe.
            FrictionCalibrationTrajectory interpolated;
            interpolated.sample_dt = planned.sample_dt / 4.0;
            for(std::size_t k=0; k+1<planned.positions.size(); ++k)
                for(int quarter=0; quarter<4; ++quarter)
                    interpolated.positions.push_back(serial_arm::calibration_retime::interpolate(
                        planned.positions,static_cast<double>(k)+0.25*quarter,planned.sample_dt,1.0).position);
            interpolated.positions.push_back(planned.positions.back());
            if(!friction_trajectory_inside_safe_replay_range(interpolated))
                throw std::runtime_error("interpolated replay exceeds joint safety margin");
        } catch(const std::exception& error) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = std::string("cannot safely plan demonstration: ") + error.what();
            model_calibration_recorder_.stop();
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        model_calibration_pose_targets_ = model_calibration_select_pose_targets(*model_calibration_trajectory_);
        if(model_calibration_pose_targets_.size() < model_calibration_options_.minimum_training_groups + 1) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = "demonstration does not provide enough independent gravity information";
            model_calibration_recorder_.stop();
            // Teaching runs in COMPLIANT_DRAG. A rejected trajectory must not
            // leave that mode active merely because the GUI request failed.
            // Never try to override a latched robot FAULT.
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        // Combined experimental calibration: one reverse replay with static
        // pauses and one forward dynamic replay. Both directions are logged for
        // friction estimation; the fitter must still pass observability checks.
        // This is a two-pass budget, NOT a guarantee of identified inertias.
        // Teaching wall time is independent of the replay clock.
        const double planned_pass_s = model_calibration_trajectory_->sample_dt *
            static_cast<double>(model_calibration_trajectory_->positions.size() - 1);
        const double static_collection_s = static_cast<double>(model_calibration_pose_targets_.size()) *
            (model_calibration_options_.static_hold_s + model_calibration_options_.static_sample_s + 0.5);
        model_calibration_estimated_duration_s_ = 2.0 * planned_pass_s + static_collection_s + 2.0;
        if(!std::isfinite(model_calibration_estimated_duration_s_)) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = "invalid planned calibration duration";
            model_calibration_recorder_.stop();
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        // No 600 s task deadline: arbitrary teaching length is allowed.
        // Joint limits, range checks, fault handling and manual cancellation remain mandatory.
        const auto trajectory_write = model_calibration_recorder_.write_trajectory(model_calibration_trajectory_->positions, model_calibration_trajectory_->sample_dt);
        if(!trajectory_write) {
            model_calibration_error_ = trajectory_write.error();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_recorder_.stop();
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) throw std::runtime_error("failed to enter RIGID_HOLD");
        model_calibration_set_phase(ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION);
        model_calibration_update_progress(0, model_calibration_pose_targets_.size() + 3);
    }

    bool model_calibration_within_time_limit() {
        // No artificial global time cap. The user sees an estimated duration
        // before confirming replay and can cancel at any point. Native joint
        // safety, command cycle watchdogs, and per-pose stability checks remain.
        return true;
    }

    bool model_calibration_wait_if_paused(ModelCalibrationPhase resume_phase) {
        if(model_calibration_cancel_.load() || !model_calibration_within_time_limit()) return false;
        if(!model_calibration_pause_.load()) return true;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clear_command_sources();
            const auto hold = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            if(!hold) return false;
            if(last_output_) model_calibration_pause_reference_ = last_output_->joint_state.pos;
        }
        model_calibration_set_phase(ModelCalibrationPhase::PAUSED);
        std::unique_lock<std::mutex> task_lock(model_calibration_mutex_);
        model_calibration_cv_.wait(task_lock, [this]() { return model_calibration_cancel_.load() || !model_calibration_pause_.load(); });
        if(model_calibration_cancel_.load() || !model_calibration_within_time_limit()) return false;
        model_calibration_resume_requires_confirmation_ = false;
        model_calibration_set_phase(resume_phase);
        return true;
    }

    bool model_calibration_send_reference(const JointVector& ref_pos, const JointVector& ref_vel) {
        std::uint64_t cursor = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) return false;
            cursor = cycle_counter_;
            const auto command = robot_.set_cmd(JointPosVelCmd{ref_pos, ref_vel}, Robot::Clock::now());
            if(!command) return false;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
            return model_calibration_cancel_.load() || cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
        });
        return updated && !model_calibration_cancel_.load() && robot_.get_state() == RobotState::ACTIVE && cycle_counter_ > cursor;
    }

    bool model_calibration_collect_static(const ModelCalibrationPoseTarget& target, const std::string& direction) {
        // Pose tags identify measured static frames only, never settling transients
        model_calibration_pose_group_.store(0);
        model_calibration_validation_.store(false);
        model_calibration_direction_.store(direction == "reverse" ? -1 : 1);
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD) ||
            !wait_for_static_tuning_pose(model_calibration_options_.static_hold_s, model_calibration_options_.static_timeout_s,
                model_calibration_options_.static_max_velocity, model_calibration_options_.static_max_acceleration)) return false;
        model_calibration_pose_group_.store(target.pose_group);
        model_calibration_validation_.store(target.validation);
        struct StaticPoseTagGuard {
            std::atomic<std::size_t>& group;
            std::atomic<bool>& validation;
            ~StaticPoseTagGuard() { group.store(0); validation.store(false); }
        } tag_guard{model_calibration_pose_group_, model_calibration_validation_};
        const auto deadline = Robot::Clock::now() + std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(model_calibration_options_.static_sample_s));
        std::uint64_t cursor = 0;
        { std::lock_guard<std::mutex> lock(mutex_); cursor = cycle_counter_; }
        while(Robot::Clock::now() < deadline) {
            if(model_calibration_cancel_.load() || !model_calibration_within_time_limit()) return false;
            const auto phase = model_calibration_phase_.load();
            if(!model_calibration_wait_if_paused(phase)) return false;
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() { return cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE || model_calibration_cancel_.load(); });
            if(robot_.get_state() != RobotState::ACTIVE || model_calibration_cancel_.load()) return false;
            if(updated && cycle_counter_ > cursor && last_output_) { cursor = cycle_counter_; ++model_calibration_valid_static_samples_; }
        }
        model_calibration_pose_group_.store(0);
        model_calibration_validation_.store(false);
        return set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING);
    }

    bool model_calibration_static_pass(bool reverse) {
        if(!model_calibration_trajectory_) return false;
        // Capture usable moving samples in the same traversal as gravity poses.
        model_calibration_speed_tier_.store(2);
        model_calibration_direction_.store(reverse ? -1 : 1);
        const auto& trajectory = *model_calibration_trajectory_;
        std::vector<JointVector> ordered = trajectory.positions;
        if(reverse) std::reverse(ordered.begin(), ordered.end());
        std::map<std::size_t, ModelCalibrationPoseTarget> targets;
        for(const auto& target : model_calibration_pose_targets_) {
            const std::size_t ordered_index = reverse ? trajectory.positions.size() - 1 - target.trajectory_index : target.trajectory_index;
            targets.emplace(ordered_index, target);
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING)) return false;
        const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        const double step = model_calibration_replay_rate_ * nominal_dt / trajectory.sample_dt;
        if(!std::isfinite(step) || step <= 0.0) return false;
        double progress = 0.0;
        std::size_t next_target = 0;
        std::vector<std::pair<std::size_t, ModelCalibrationPoseTarget>> target_list(targets.begin(), targets.end());
        while(progress < static_cast<double>(ordered.size() - 1)) {
            const auto phase = reverse ? ModelCalibrationPhase::STATIC_REVERSE : ModelCalibrationPhase::STATIC_FORWARD;
            if(!model_calibration_wait_if_paused(phase)) return false;
            const double next_progress = std::min(static_cast<double>(ordered.size() - 1), progress + step);
            const auto reference=serial_arm::calibration_retime::interpolate(
                ordered,next_progress,trajectory.sample_dt,model_calibration_replay_rate_);
            if(!model_calibration_send_reference(reference.position,reference.velocity)) return false;
            progress = next_progress;
            while(next_target < target_list.size() && static_cast<double>(target_list[next_target].first) <= progress + 1.0e-9) {
                if(!model_calibration_collect_static(target_list[next_target].second, reverse ? "reverse" : "forward")) return false;
                ++next_target;
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                ++model_calibration_completed_units_;
                model_calibration_progress_ = model_calibration_total_units_ ? static_cast<double>(model_calibration_completed_units_) / static_cast<double>(model_calibration_total_units_) : 0.0;
            }
        }
        model_calibration_speed_tier_.store(0);
        return true;
    }

    bool model_calibration_dynamic_pass(bool reverse, double playback_rate, int speed_tier, ModelCalibrationPhase phase) {
        if(!model_calibration_trajectory_) return false;
        const auto& trajectory=*model_calibration_trajectory_;
        std::vector<JointVector> ordered=trajectory.positions;
        if(reverse) std::reverse(ordered.begin(),ordered.end());
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING)) return false;
        model_calibration_direction_.store(reverse?-1:1);
        model_calibration_speed_tier_.store(speed_tier);
        const double nominal_dt=1.0/cfg_.runtime.ctrl_frequency_hz;
        const double step=playback_rate*nominal_dt/trajectory.sample_dt;
        if(!std::isfinite(step)||step<=0.0)return false;
        double progress=0.0;
        while(progress<static_cast<double>(ordered.size()-1)){
            if(!model_calibration_wait_if_paused(phase))return false;
            const double next_progress=std::min(static_cast<double>(ordered.size()-1),progress+step);
            const auto reference=serial_arm::calibration_retime::interpolate(
                ordered,next_progress,trajectory.sample_dt,playback_rate);
            if(!model_calibration_send_reference(reference.position,reference.velocity))return false;
            progress=next_progress;
        }
        {
            std::lock_guard<std::mutex> lock(model_calibration_mutex_);
            ++model_calibration_completed_units_;
            model_calibration_progress_=model_calibration_total_units_?static_cast<double>(model_calibration_completed_units_)/static_cast<double>(model_calibration_total_units_):0.0;
        }
        return true;
    }

    std::vector<AdmittanceFrictionSample> model_calibration_friction_samples(
        const std::vector<ModelCalibrationFrame>& frames,
        const std::string& direction,
        int speed_tier,
        const GravityCalibrationResult& gravity_result) {
        std::vector<AdmittanceFrictionSample> result;
        for(const auto& frame:frames){
            if(!frame.valid||frame.speed_tier!=(speed_tier==1?"slow":"fast")||frame.direction!=direction||
                (frame.phase.find("friction_")!=0 && frame.phase!="static_reverse"))continue;
            if(frame.position.size()!=cfg_.joint_names.size()||frame.velocity.size()!=cfg_.joint_names.size()||frame.torque.size()!=cfg_.joint_names.size()||frame.acceleration.size()!=cfg_.joint_names.size())continue;
            JointVector coriolis;
            Eigen::MatrixXd mass;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                JointState state;state.pos=frame.position;state.vel=frame.velocity;state.tor=frame.torque;
                const auto updated=dynamics_.update(state,frame.acceleration,frame.acceleration);
                if(!updated)continue;
                coriolis=dynamics_.get_coriolis();mass=dynamics_.get_mass_matrix();
            }
            const auto candidate_gravity=dynamics_.compute_gravity_with_first_moments(frame.position,gravity_result.first_moments);
            if(!candidate_gravity)continue;
            JointVector residual(cfg_.joint_names.size(),0.0);
            for(std::size_t i=0;i<cfg_.joint_names.size();++i){double torque=(*candidate_gravity)[i]+coriolis[i];for(std::size_t j=0;j<cfg_.joint_names.size();++j)torque+=mass(static_cast<Eigen::Index>(i),static_cast<Eigen::Index>(j))*frame.acceleration[j];residual[i]=torque-frame.torque[i]-gravity_result.torque_bias[i];}
            result.push_back(AdmittanceFrictionSample{frame.velocity,frame.acceleration,std::move(residual)});
        }
        return result;
    }

    bool model_calibration_validate_friction_holdout(
        const AdmittanceFrictionCalibrationResult& fit,
        const std::vector<AdmittanceFrictionSample>& validation) const {
        if(validation.empty()) return false;
        const std::size_t n=cfg_.joint_names.size();
        std::vector<double> before(n,0.0),after(n,0.0);std::vector<std::size_t> counts(n,0);
        for(const auto& sample:validation){
            for(std::size_t i=0;i<n;++i){const double v=sample.velocity[i];if(std::abs(v)<0.05||std::abs(sample.acceleration[i])>1.5)continue;const double prediction=v>=0.0?fit.positive_coulomb[i]+fit.positive_viscous[i]*std::abs(v):fit.negative_coulomb[i]+fit.negative_viscous[i]*std::abs(v);before[i]+=sample.residual_after_bias[i]*sample.residual_after_bias[i];const double e=sample.residual_after_bias[i]-prediction;after[i]+=e*e;++counts[i];}
        }
        bool pass=true;
        for(std::size_t i=0;i<n;++i){if(counts[i]<20){pass=false;continue;}const double b=std::sqrt(before[i]/static_cast<double>(counts[i]));const double a=std::sqrt(after[i]/static_cast<double>(counts[i]));if(!(a<=0.9*b))pass=false;}
        return pass;
    }

    void model_calibration_worker() {
        try {
            // Keep the original demonstrated path and its retimed safety limits.
            // Reuse the reverse traversal for gravity holds AND friction data.
            model_calibration_set_phase(ModelCalibrationPhase::STATIC_REVERSE);
            if(!model_calibration_static_pass(true))
                throw std::runtime_error(model_calibration_error_.empty() ? "reverse static replay failed or cancelled" : model_calibration_error_);
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("failed to return to RIGID_HOLD");
            model_calibration_set_phase(ModelCalibrationPhase::GRAVITY_FITTING);
            const auto recorder_status = model_calibration_recorder_.status();
            if(recorder_status.data_gap || recorder_status.write_failed)
                throw std::runtime_error(recorder_status.error.empty() ? "model calibration raw data contains a recording gap" : recorder_status.error);
            const auto groups = group_static_calibration_frames(model_calibration_recorder_.frames());
            GravityCalibrationOptions options;
            options.minimum_training_groups=model_calibration_options_.minimum_training_groups;
            options.regularization=model_calibration_options_.regularization;
            options.svd_relative_threshold=model_calibration_options_.svd_relative_threshold;
            options.identification_mode=model_calibration_options_.identification_mode;
            options.locked_links=model_calibration_options_.locked_links;
            const auto gravity=fit_gravity_calibration(dynamics_,groups,model_calibration_original_gravity_scale_,options);
            if(!gravity) throw std::runtime_error(gravity.error());
            model_calibration_result_=gravity.value();
            // Holdout validation is advisory for exported candidates. Never
            // abort data acquisition or suppress the candidate because of
            // an inaccurate initial URDF. Applying it to hardware is separate.
            if(!model_calibration_within_time_limit()) throw std::runtime_error(model_calibration_error_);
            {
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                ++model_calibration_completed_units_;
                model_calibration_progress_=static_cast<double>(model_calibration_completed_units_)/static_cast<double>(model_calibration_total_units_);
            }
            // Forward at the planner-approved rate. The reverse traversal was
            // logged already, avoiding four redundant full-path traversals.
            model_calibration_set_phase(ModelCalibrationPhase::FRICTION_FORWARD_FAST);
            if(!model_calibration_dynamic_pass(false,model_calibration_replay_rate_,2,ModelCalibrationPhase::FRICTION_FORWARD_FAST))
                throw std::runtime_error(model_calibration_error_.empty() ? "forward dynamic replay failed or cancelled" : model_calibration_error_);
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("failed to hold after replay");
            model_calibration_set_phase(ModelCalibrationPhase::FRICTION_FITTING);
            const auto all_frames=model_calibration_recorder_.frames();
            const auto reverse_samples=model_calibration_friction_samples(all_frames,"reverse",2,*model_calibration_result_);
            const auto forward_samples=model_calibration_friction_samples(all_frames,"forward",2,*model_calibration_result_);
            AdmittanceFrictionCalibrationCfg friction_cfg;
            friction_cfg.joints_count=cfg_.joint_names.size();
            friction_cfg.min_fit_velocity=0.05;
            friction_cfg.max_fit_acceleration=1.5;
            friction_cfg.min_speed_span=0.03;
            friction_cfg.min_samples_per_direction=30;
            friction_cfg.cross_validation_max_rms_ratio=0.8;
            const auto fit=calibrate_admittance_friction_cross_validated(reverse_samples,forward_samples,friction_cfg);
            model_calibration_friction_pass_=false;
            if(fit){
                model_calibration_friction_result_=fit.value();
                // Reduced-pass data need positive observability AND held-out
                // direction agreement; otherwise do not enable compensation.
                const auto& r=*model_calibration_friction_result_;
                model_calibration_friction_pass_=r.observable.size()==cfg_.joint_names.size() &&
                    r.validation_pass.size()==cfg_.joint_names.size() &&
                    std::all_of(r.observable.begin(),r.observable.end(),[](auto x){return x!=0;}) &&
                    std::all_of(r.validation_pass.begin(),r.validation_pass.end(),[](auto x){return x!=0;});
            }
            {
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                ++model_calibration_completed_units_;
                model_calibration_progress_=1.0;
            }
            model_calibration_pose_group_.store(0);
            model_calibration_validation_.store(false);
            model_calibration_direction_.store(0);
            model_calibration_speed_tier_.store(0);
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::COMPLETE);
            model_calibration_recorder_.stop();
            model_calibration_write_result_file();
        } catch(const std::exception& error) {
            model_calibration_error_=error.what();
            model_calibration_pose_group_.store(0);
            model_calibration_validation_.store(false);
            model_calibration_direction_.store(0);
            model_calibration_speed_tier_.store(0);
            if(machine_robot_state()==RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(model_calibration_fault_interrupted_.load() ? ModelCalibrationPhase::FAILED :
                (model_calibration_cancel_.load() ? ModelCalibrationPhase::CANCELLED : ModelCalibrationPhase::FAILED));
            model_calibration_recorder_.stop();
            model_calibration_write_result_file();
        }
    }

    // A supervised, manual alignment mode: no generated autonomous path is
    // permitted here because the standalone Core has no live planning scene.
    void machine_model_calibration_alignment_begin(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load()!=ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
           model_calibration_source_task_id_.empty() || !model_calibration_trajectory_)
            throw std::runtime_error("alignment is only supported for a staged imported demonstration");
        if(model_calibration_alignment_active_.load()) throw std::runtime_error("manual endpoint alignment is already active");
        if(!params || !params["supported"] || !params["supported"].as<bool>())
            throw std::runtime_error("operator must confirm the arm is physically supported and the work area is clear");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE || !last_output_ || feedback_time_ns_==0)
                throw std::runtime_error("endpoint alignment requires ACTIVE robot and fresh joint feedback");
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            if(now_ns<feedback_time_ns_ || double(now_ns-feedback_time_ns_)/1.0e6>serial_arm::calibration_recovery::kFeedbackMaxAgeMs)
                throw std::runtime_error("joint feedback is stale; manual alignment is not permitted");
            double velocity=0.0;
            for(double v:last_output_->joint_state.vel) {
                if(!std::isfinite(v)) throw std::runtime_error("non-finite joint speed during alignment");
                velocity=std::max(velocity,std::abs(v));
            }
            if(velocity>serial_arm::calibration_recovery::kVelocityToleranceRadS)
                throw std::runtime_error("arm must be stationary before enabling manual alignment");
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG))
            throw std::runtime_error("could not enter compliant drag for manual alignment");
        model_calibration_alignment_active_.store(true);
    }
    void machine_model_calibration_alignment_finish(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load()!=ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
           !model_calibration_alignment_active_.load())
            throw std::runtime_error("manual endpoint alignment is not active");
        if(machine_robot_state()!=RobotState::ACTIVE)
            throw std::runtime_error("robot is not ACTIVE; cannot finish manual alignment");
        // Only engage the stronger hold after the operator has stopped moving;
        // the pose itself may still be far from the saved endpoint.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(!last_output_ || feedback_time_ns_==0)
                throw std::runtime_error("cannot stop manual alignment: live joint feedback is unavailable");
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            if(now_ns<feedback_time_ns_ || double(now_ns-feedback_time_ns_)/1.0e6>serial_arm::calibration_recovery::kFeedbackMaxAgeMs)
                throw std::runtime_error("cannot stop manual alignment: joint feedback is stale");
            for(double v:last_output_->joint_state.vel)
                if(!std::isfinite(v) || std::abs(v)>serial_arm::calibration_recovery::kVelocityToleranceRadS)
                    throw std::runtime_error("stop moving and physically support the arm before switching to RIGID_HOLD");
        }
        // Hold the ACTUAL current pose, regardless of alignment outcome.
        // A non-matching pose remains blocked from starting replay.
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
            throw std::runtime_error("failed to enter rigid hold after manual alignment");
        model_calibration_alignment_active_.store(false);
    }

    void machine_model_calibration_start(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load()!=ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION||!model_calibration_trajectory_)throw std::runtime_error("model calibration is not waiting for replay confirmation");
        if(model_calibration_alignment_active_.load())
            throw std::runtime_error("finish manual endpoint alignment and hold before confirming replay");
        if(!params["released"]||!params["released"].as<bool>())throw std::runtime_error("explicit released confirmation is required");
        if(!model_calibration_source_task_id_.empty()) {
            // The first pass traverses the path in reverse, starting at its last
            // sample. A parked arm can be far away: NEVER jump to that target.
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE || !last_output_ || feedback_time_ns_==0)
                throw std::runtime_error("cannot confirm imported replay without fresh ACTIVE robot feedback");
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            const double age_ms=now_ns>=feedback_time_ns_ ? double(now_ns-feedback_time_ns_)/1.0e6 : std::numeric_limits<double>::infinity();
            const auto& start=model_calibration_trajectory_->positions.back();
            const auto& actual=last_output_->joint_state;
            const auto choice=serial_arm::calibration_recovery::select_endpoint(actual.pos,actual.vel,start,start,age_ms);
            if(!choice.valid) throw std::runtime_error("replay blocked: joint feedback is stale or does not match the imported trajectory");
            if(!choice.at_endpoint) {
                std::ostringstream message;
                message << std::fixed << std::setprecision(4)
                        << "imported replay start pose mismatch (max joint error=" << choice.max_error_rad
                        << " rad; max joint speed=" << choice.max_speed_rad_s
                        << " rad/s; allowed 0.08 rad and 0.05 rad/s). Use guided endpoint alignment, then reconfirm.";
                throw std::runtime_error(message.str());
            }
        }
        if(model_calibration_worker_.joinable())throw std::runtime_error("model calibration worker is already running");
        model_calibration_fault_interrupted_.store(false);model_calibration_cancel_.store(false);model_calibration_pause_.store(false);model_calibration_resume_confirmed_.store(false);
        model_calibration_started_at_ = Robot::Clock::now();
        model_calibration_set_phase(ModelCalibrationPhase::STATIC_REVERSE);
        model_calibration_worker_=std::thread([this](){model_calibration_worker();});
    }

    void machine_model_calibration_pause(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(!model_calibration_task_active()||model_calibration_phase_.load()==ModelCalibrationPhase::TEACHING||model_calibration_phase_.load()==ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)throw std::runtime_error("model calibration task is not in a pausable stage");
        model_calibration_pause_.store(true);model_calibration_cv_.notify_all();
    }

    void machine_model_calibration_resume(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(!model_calibration_pause_.load())throw std::runtime_error("model calibration task is not paused");
        JointVector current;
        bool feedback_fresh = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(last_output_) current = last_output_->joint_state.pos;
            if(feedback_time_ns_ > 0) {
                const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                const double age_ms = now_ns >= feedback_time_ns_ ? static_cast<double>(now_ns - feedback_time_ns_) / 1.0e6 : 0.0;
                feedback_fresh = age_ms <= 500.0;
            }
        }
        if(!feedback_fresh || current.size() != model_calibration_pause_reference_.size()) throw std::runtime_error("current feedback is unavailable or stale; resume is not allowed");
        double max_error=0.0;for(std::size_t i=0;i<current.size();++i)max_error=std::max(max_error,std::abs(current[i]-model_calibration_pause_reference_[i]));
        if(max_error>0.05&&(!params||!params["confirmed"]||!params["confirmed"].as<bool>())){model_calibration_resume_requires_confirmation_=true;throw std::runtime_error("robot position changed while paused; explicit resume confirmation is required");}
        model_calibration_resume_requires_confirmation_=false;model_calibration_resume_confirmed_.store(true);model_calibration_pause_.store(false);model_calibration_cv_.notify_all();
    }

    void machine_model_calibration_cancel_request(const YAML::Node& params) {
        model_calibration_require_task(params);
        machine_model_calibration_cancel();
    }

    void machine_model_calibration_cancel() {
        const auto phase=model_calibration_phase_.load();
        if(phase==ModelCalibrationPhase::IDLE||phase==ModelCalibrationPhase::COMPLETE||phase==ModelCalibrationPhase::CANCELLED||phase==ModelCalibrationPhase::FAILED)return;
        model_calibration_cancel_.store(true);model_calibration_teach_stop_.store(true);model_calibration_pause_.store(false);model_calibration_cv_.notify_all();cycle_cv_.notify_all();
        if(phase==ModelCalibrationPhase::TEACHING||phase==ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION){
            if(model_calibration_teach_thread_.joinable())model_calibration_teach_thread_.join();
            if(machine_robot_state()==RobotState::ACTIVE)(void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();model_calibration_set_phase(ModelCalibrationPhase::CANCELLED);model_calibration_recorder_.stop();
        }
    }

    void model_calibration_write_result_file() const {
        if(model_calibration_directory_.empty()) return;
        std::ofstream out(std::filesystem::path(model_calibration_directory_) / "result.json");
        if(!out) return;
        out << machine_model_calibration_status_json() << '\n';
    }

    void machine_model_calibration_apply(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_task_active())throw std::runtime_error("model calibration task is still active");
        if(!model_calibration_result_||!model_calibration_result_->static_pass)throw std::runtime_error("no validated gravity candidate is available");
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state()!=RobotState::INACTIVE)throw std::runtime_error("candidate application requires RobotState::INACTIVE");
        if(robot_.get_model_feedforward_mode()==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)throw std::runtime_error("candidate gravity is not compatible with FULL_INVERSE_DYNAMICS");
        if(cfg_.capability.admittance.observer.mode==AdmittanceObserverMode::FULL_ID)throw std::runtime_error("candidate gravity is not compatible with FULL_ID observer mode");
        const auto current_core=model_calibration_file_fingerprint(config_path_);const auto current_urdf=model_calibration_file_fingerprint(cfg_.dynamics.urdf_path);
        if(current_core!=model_calibration_core_fingerprint_||current_urdf!=model_calibration_urdf_fingerprint_)throw std::runtime_error("configuration or URDF changed after calibration");
        model_calibration_applied_original_gravity_scale_=dynamics_.get_gravity_scale();model_calibration_applied_original_admittance_=cfg_.capability.admittance;
        JointVector unit(cfg_.joint_names.size(),1.0);const auto scale=dynamics_.set_gravity_scale(unit);if(!scale)throw std::runtime_error("failed to set unit gravity scale");
        const auto override_result=dynamics_.set_gravity_first_moment_override(model_calibration_result_->first_moments);if(!override_result){(void)dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_);throw std::runtime_error("failed to apply gravity candidate");}
        auto candidate=cfg_.capability.admittance;candidate.calibration.torque_bias=model_calibration_result_->torque_bias;candidate.calibration.torque_threshold.resize(cfg_.joint_names.size());for(std::size_t i=0;i<cfg_.joint_names.size();++i){const double p99=i<model_calibration_result_->validation_candidate.p99.size()?model_calibration_result_->validation_candidate.p99[i]:0.0;const double noise=i<model_calibration_result_->noise_rms.size()?model_calibration_result_->noise_rms[i]:0.0;candidate.calibration.torque_threshold[i]=std::max({0.02,1.2*p99,3.0*noise});}
        if(model_calibration_friction_pass_&&model_calibration_friction_result_){candidate.calibration.friction.enabled=true;candidate.calibration.friction.velocity_transition=0.03;candidate.calibration.friction.positive_coulomb=model_calibration_friction_result_->positive_coulomb;candidate.calibration.friction.positive_viscous=model_calibration_friction_result_->positive_viscous;candidate.calibration.friction.negative_coulomb=model_calibration_friction_result_->negative_coulomb;candidate.calibration.friction.negative_viscous=model_calibration_friction_result_->negative_viscous;}else candidate.calibration.friction.enabled=false;
        const auto admittance=robot_.set_admittance_cfg(candidate);if(!admittance){dynamics_.clear_gravity_first_moment_override();(void)dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_);throw std::runtime_error("failed to apply candidate residual parameters");}
        cfg_.dynamics.gravity_scale=unit;cfg_.capability.admittance=candidate;model_calibration_candidate_applied_=true;
    }

    void machine_model_calibration_restore(const YAML::Node& params) {
        model_calibration_require_task(params);
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state()!=RobotState::INACTIVE)throw std::runtime_error("candidate restore requires RobotState::INACTIVE");
        if(!model_calibration_candidate_applied_)return;
        const auto candidate_admittance = cfg_.capability.admittance;
        const auto candidate_scale = dynamics_.get_gravity_scale();
        const auto candidate_moments = dynamics_.get_gravity_first_moment_override();
        const auto admittance=robot_.set_admittance_cfg(model_calibration_applied_original_admittance_);if(!admittance)throw std::runtime_error("failed to restore original residual parameters");
        dynamics_.clear_gravity_first_moment_override();
        const auto scale=dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_);
        if(!scale){
            (void)dynamics_.set_gravity_scale(candidate_scale);
            (void)dynamics_.set_gravity_first_moment_override(candidate_moments);
            (void)robot_.set_admittance_cfg(candidate_admittance);
            throw std::runtime_error("failed to restore original gravity scale");
        }
        cfg_.dynamics.gravity_scale=model_calibration_applied_original_gravity_scale_;cfg_.capability.admittance=model_calibration_applied_original_admittance_;model_calibration_candidate_applied_=false;
    }

    std::string machine_model_calibration_status_json() const {
        std::ostringstream out;const auto recorder=model_calibration_recorder_.status();
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        out<<std::setprecision(17)<<"{\"task_id\":\""<<machine_json_escape(model_calibration_task_id_)<<"\",\"phase\":\""<<to_string(model_calibration_phase_.load())<<"\",\"progress\":"<<model_calibration_progress_<<",\"completed\":"<<model_calibration_completed_units_<<",\"total\":"<<model_calibration_total_units_<<",\"valid_static_samples\":"<<model_calibration_valid_static_samples_<<",\"directory\":\""<<machine_json_escape(model_calibration_directory_)<<"\",\"candidate_applied\":"<<(model_calibration_candidate_applied_?"true":"false")<<",\"resume_confirmation_required\":"<<(model_calibration_resume_requires_confirmation_?"true":"false")<<",\"core_fingerprint\":\""<<machine_json_escape(model_calibration_core_fingerprint_)<<"\",\"urdf_fingerprint\":\""<<machine_json_escape(model_calibration_urdf_fingerprint_)<<"\",\"joint_names\":[";
        for(std::size_t i=0;i<cfg_.joint_names.size();++i){if(i)out<<',';out<<'\"'<<machine_json_escape(cfg_.joint_names[i])<<'\"';}
        out<<"],\"original_gravity_scale\":";machine_json_joint_vector(out,model_calibration_original_gravity_scale_);
        out<<",\"recorder\":{\"accepted\":"<<recorder.accepted_frames<<",\"written\":"<<recorder.written_frames<<",\"dropped\":"<<recorder.dropped_frames<<",\"data_gap\":"<<(recorder.data_gap?"true":"false")<<",\"write_failed\":"<<(recorder.write_failed?"true":"false")<<"}";
        if(!model_calibration_error_.empty())out<<",\"error\":\""<<machine_json_escape(model_calibration_error_)<<"\"";
        if(model_calibration_trajectory_)out<<",\"trajectory_samples\":"<<model_calibration_trajectory_->positions.size()<<",\"replay_rate\":"<<model_calibration_replay_rate_;
        out<<",\"identification_mode\":\""<<machine_json_escape(model_calibration_options_.identification_mode)<<"\",\"locked_links\":[";
        for(std::size_t i=0;i<model_calibration_options_.locked_links.size();++i){if(i)out<<',';out<<'"'<<machine_json_escape(model_calibration_options_.locked_links[i])<<'"';}
        out<<"],\"identifiable_links\":[";
        for(std::size_t i=0;i<model_calibration_links_cache_.size();++i){
            if(i)out<<','; const auto& info=model_calibration_links_cache_[i];
            out<<"{\"name\":\""<<machine_json_escape(info.link_name)<<"\",\"joint\":\""<<machine_json_escape(info.joint_name)<<"\",\"mass\":"<<info.mass<<",\"com\":["<<info.center_of_mass.x()<<","<<info.center_of_mass.y()<<","<<info.center_of_mass.z()<<"]}";
        }
        out<<']';
        out<<",\"planner_id\":\"local_hermite\",\"calibration_strategy\":\"two_pass_combined\"";
        out<<",\"alignment_active\":"<<(model_calibration_alignment_active_.load()?"true":"false");
        if(!model_calibration_source_task_id_.empty()) out<<",\"original_start_selected\":"<<(model_calibration_source_reversed_.load()?"true":"false");
        if(!model_calibration_source_task_id_.empty()) out<<",\"source_task_id\":\""<<machine_json_escape(model_calibration_source_task_id_)<<"\"";
        if(!model_calibration_source_task_id_.empty() && model_calibration_trajectory_ && !model_calibration_trajectory_->positions.empty()) {
            out<<",\"replay_start_joint_positions\":";
            machine_json_joint_vector(out,model_calibration_trajectory_->positions.back());
        }
        out<<",\"joint_path_length_rad\":"<<model_calibration_path_length_rad_
           <<",\"geometric_waypoints\":"<<model_calibration_geometric_waypoints_
           <<",\"original_samples\":"<<model_calibration_original_samples_;
        out<<",\"teaching_wall_duration_s\":"<<(model_calibration_phase_.load() == ModelCalibrationPhase::TEACHING && model_calibration_teaching_started_at_ ?
            std::chrono::duration<double>(Robot::Clock::now() - *model_calibration_teaching_started_at_).count() : model_calibration_teaching_wall_duration_s_)
           <<",\"recorded_duration_s\":"<<model_calibration_original_duration_s_;
        if(model_calibration_trajectory_ && model_calibration_trajectory_->positions.size() >= 2)
            out<<",\"trajectory_duration_s\":"<<model_calibration_trajectory_->sample_dt *
                static_cast<double>(model_calibration_trajectory_->positions.size() - 1)
               <<",\"trajectory_sample_dt_s\":"<<model_calibration_trajectory_->sample_dt;
        out<<",\"single_pass_duration_s\":"<< (model_calibration_trajectory_ && model_calibration_trajectory_->positions.size() >= 2 ?
            model_calibration_trajectory_->sample_dt * static_cast<double>(model_calibration_trajectory_->positions.size()-1) : 0.0)
           <<",\"estimated_duration_s\":"<<model_calibration_estimated_duration_s_
           <<",\"estimated_total_duration_s\":"<<(model_calibration_teaching_wall_duration_s_ + model_calibration_estimated_duration_s_)
           <<",\"max_task_duration_s\":0"
           <<",\"minimum_information_score\":"<<model_calibration_options_.minimum_information_score;
        out<<",\"pose_targets\":[";for(std::size_t i=0;i<model_calibration_pose_targets_.size();++i){if(i)out<<',';const auto& p=model_calibration_pose_targets_[i];out<<"{\"index\":"<<p.trajectory_index<<",\"group\":"<<p.pose_group<<",\"validation\":"<<(p.validation?"true":"false")<<",\"score\":"<<p.information_score<<'}';}out<<']';
        if(model_calibration_result_)out<<",\"gravity_result\":"<<gravity_calibration_result_json(*model_calibration_result_);
        out<<",\"friction_pass\":"<<(model_calibration_friction_pass_?"true":"false");
        if(model_calibration_friction_result_){out<<",\"friction\":{\"positive_coulomb\":";machine_json_joint_vector(out,model_calibration_friction_result_->positive_coulomb);out<<",\"positive_viscous\":";machine_json_joint_vector(out,model_calibration_friction_result_->positive_viscous);out<<",\"negative_coulomb\":";machine_json_joint_vector(out,model_calibration_friction_result_->negative_coulomb);out<<",\"negative_viscous\":";machine_json_joint_vector(out,model_calibration_friction_result_->negative_viscous);out<<"}";}
        out<<'}';return out.str();
    }

    void machine_calibration_begin(const std::string& kind) {
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) throw std::runtime_error("another calibration task is active");
        if(kind != "static" && kind != "validation") throw std::runtime_error("unsupported calibration kind");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("calibration requires RobotState::ACTIVE");
            if(!dynamics_.is_updated()) throw std::runtime_error("dynamics is not ready");
            machine_calibration_original_mode_ = robot_.get_impedance_mode();
            machine_calibration_original_admittance_ = cfg_.capability.admittance;
            machine_calibration_original_gravity_ = dynamics_.get_gravity_scale();
            machine_calibration_original_suspended_ = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        machine_calibration_context_valid_ = true;
        machine_calibration_error_.clear();
        machine_calibration_kind_ = kind;
        machine_calibration_phase_ = "waiting_user";
        machine_calibration_expected_ = kind == "static" ? 8 : 5;
        machine_calibration_poses_.clear();
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            machine_calibration_kind_ = "none";
            machine_restore_calibration_runtime(true);
            throw std::runtime_error("failed to enter COMPLIANT_DRAG");
        }
    }

    void machine_calibration_capture() {
        if(machine_calibration_kind_ != "static" && machine_calibration_kind_ != "validation") throw std::runtime_error("no static calibration task is active");
        if(machine_calibration_poses_.size() >= machine_calibration_expected_) throw std::runtime_error("all required poses are already captured");
        machine_calibration_phase_ = "collecting";
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD) || !wait_for_static_tuning_pose(0.30, 5.0, 0.03, 1.5)) {
            machine_calibration_phase_ = "waiting_user";
            throw std::runtime_error("pose did not become static within timeout");
        }
        auto samples = collect_static_pose_samples(0.0, 1.0);
        if(!samples) { machine_calibration_phase_ = "waiting_user"; throw std::runtime_error("static sample collection failed"); }
        machine_calibration_poses_.push_back(std::move(*samples));
        if(machine_calibration_poses_.size() < machine_calibration_expected_) {
            if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) throw std::runtime_error("failed to return to COMPLIANT_DRAG");
            machine_calibration_phase_ = "waiting_user";
        } else machine_calibration_phase_ = "ready_to_fit";
    }

    void machine_calibration_finish() {
        if(machine_calibration_poses_.size() != machine_calibration_expected_) throw std::runtime_error("required poses have not all been captured");
        machine_calibration_phase_ = "fitting";
        if(machine_calibration_kind_ == "static") {
            AdmittanceStaticCalibrationCfg config;
            config.joints_count = cfg_.joint_names.size();
            config.fallback_gravity_scale = machine_calibration_original_gravity_;
            config.gravity_observability_span = 0.25;
            config.threshold_margin = 1.2;
            config.threshold_max_margin = 1.05;
            const auto result = calibrate_admittance_static(machine_calibration_poses_, config);
            if(!result) { machine_calibration_phase_ = "failed"; machine_restore_calibration_runtime(true); throw std::runtime_error("static calibration fit failed"); }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto scale = dynamics_.set_gravity_scale(result->gravity_scale);
                if(!scale) throw std::runtime_error("gravity_scale apply failed");
                cfg_.dynamics.gravity_scale = result->gravity_scale;
            }
            auto candidate = machine_calibration_original_admittance_;
            candidate.calibration.torque_bias = result->torque_bias;
            candidate.calibration.torque_threshold = result->torque_threshold;
            candidate.calibration.friction.enabled = false;
            if(!apply_runtime_admittance_cfg(candidate)) throw std::runtime_error("calibration parameters apply failed");
            last_static_calibration_result_ = result.value();
            machine_restore_calibration_runtime(false);
            machine_calibration_phase_ = "complete";
        } else {
            AdmittanceStaticValidationCfg config;
            config.joints_count = cfg_.joint_names.size();
            config.gravity_scale = machine_calibration_original_gravity_;
            config.torque_bias = machine_calibration_original_admittance_.calibration.torque_bias;
            config.torque_threshold = machine_calibration_original_admittance_.calibration.torque_threshold;
            const auto result = evaluate_admittance_static_validation(machine_calibration_poses_, config);
            if(!result) { machine_calibration_phase_ = "failed"; machine_restore_calibration_runtime(false); throw std::runtime_error("static validation failed"); }
            last_static_validation_result_ = result.value();
            machine_restore_calibration_runtime(false);
            machine_calibration_phase_ = "complete";
        }
        machine_calibration_kind_ = "none";
        machine_calibration_context_valid_ = false;
    }

    void machine_calibration_cancel() {
        if(machine_calibration_kind_ == "friction_recording" || machine_calibration_kind_ == "friction_replaying") {
            machine_friction_stop_.store(true);
            cycle_cv_.notify_all();
            if(machine_friction_thread_.joinable()) machine_friction_thread_.join();
        }
        if(machine_calibration_context_valid_) machine_restore_calibration_runtime(true);
        machine_calibration_kind_ = "none";
        machine_calibration_phase_ = "cancelled";
        machine_calibration_poses_.clear();
        machine_friction_trajectory_.reset();
        machine_calibration_context_valid_ = false;
    }

    void machine_friction_record_begin() {
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) throw std::runtime_error("another calibration task is active");
        if(machine_friction_thread_.joinable()) machine_friction_thread_.join();
        machine_calibration_error_.clear();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("friction calibration requires RobotState::ACTIVE");
            machine_calibration_original_mode_ = robot_.get_impedance_mode();
            machine_calibration_original_admittance_ = cfg_.capability.admittance;
            machine_calibration_original_gravity_ = dynamics_.get_gravity_scale();
            machine_calibration_original_suspended_ = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        machine_calibration_context_valid_ = true;
        machine_calibration_kind_ = "friction_recording";
        machine_calibration_phase_ = "recording";
        machine_friction_stop_.store(false);
        machine_friction_trajectory_.reset();
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) throw std::runtime_error("failed to enter COMPLIANT_DRAG");
        machine_friction_thread_ = std::thread([this]() { machine_friction_trajectory_ = collect_friction_drag_trajectory_until_stop(machine_friction_stop_); });
    }

    void machine_friction_record_stop() {
        if(machine_calibration_kind_ != "friction_recording") throw std::runtime_error("friction recording is not active");
        machine_friction_stop_.store(true);
        cycle_cv_.notify_all();
        if(machine_friction_thread_.joinable()) machine_friction_thread_.join();
        if(!machine_friction_trajectory_) { machine_calibration_cancel(); throw std::runtime_error("friction trajectory is too short"); }
        *machine_friction_trajectory_ = smooth_friction_trajectory(*machine_friction_trajectory_);
        if(!friction_trajectory_inside_safe_replay_range(*machine_friction_trajectory_)) { machine_calibration_cancel(); throw std::runtime_error("friction trajectory exceeds safe replay range"); }
        machine_friction_replay_rate_ = friction_replay_rate(*machine_friction_trajectory_);
        if(!std::isfinite(machine_friction_replay_rate_) || machine_friction_replay_rate_ < 0.01) { machine_calibration_cancel(); throw std::runtime_error("demonstration is too fast for safe replay"); }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) throw std::runtime_error("failed to enter RIGID_HOLD");
        machine_calibration_kind_ = "friction_ready";
        machine_calibration_phase_ = "waiting_replay_confirmation";
    }

    void machine_friction_replay_start() {
        if(machine_calibration_kind_ != "friction_ready" || !machine_friction_trajectory_) throw std::runtime_error("friction trajectory is not ready");
        if(machine_friction_thread_.joinable()) throw std::runtime_error("friction worker is already active");
        machine_calibration_kind_ = "friction_replaying";
        machine_calibration_phase_ = "replaying_reverse";
        machine_friction_stop_.store(false);
        machine_friction_thread_ = std::thread([this]() { machine_friction_replay_worker(); });
    }

    void machine_friction_replay_worker() {
        try {
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING)) throw std::runtime_error("failed to enter RIGID_TRACKING");
            std::vector<AdmittanceFrictionSample> reverse_samples, forward_samples, full_reverse, full_forward;
            const bool compare_full = machine_calibration_original_admittance_.observer.mode == AdmittanceObserverMode::MOMENTUM;
            machine_calibration_phase_ = "replaying_reverse";
            if(!play_friction_calibration_pass(*machine_friction_trajectory_, true, machine_friction_replay_rate_, machine_calibration_original_admittance_, reverse_samples, compare_full ? &full_reverse : nullptr)) {
                if(machine_friction_stop_.load()) return;
                throw std::runtime_error("friction reverse replay failed");
            }
            machine_calibration_phase_ = "replaying_forward";
            if(!play_friction_calibration_pass(*machine_friction_trajectory_, false, machine_friction_replay_rate_, machine_calibration_original_admittance_, forward_samples, compare_full ? &full_forward : nullptr)) {
                if(machine_friction_stop_.load()) return;
                throw std::runtime_error("friction forward replay failed");
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) throw std::runtime_error("failed to return to RIGID_HOLD");
            machine_calibration_phase_ = "fitting";
            AdmittanceFrictionCalibrationCfg config;
            config.joints_count = cfg_.joint_names.size(); config.min_fit_velocity = 0.05; config.max_fit_acceleration = 1.5;
            config.min_speed_span = 0.03; config.min_samples_per_direction = 30; config.cross_validation_max_rms_ratio = 0.8;
            const auto result = calibrate_admittance_friction_cross_validated(reverse_samples, forward_samples, config);
            if(!result) throw std::runtime_error("friction fit failed");
            last_friction_calibration_result_ = result.value();
            if(compare_full) { const auto full = calibrate_admittance_friction_cross_validated(full_reverse, full_forward, config); if(full) last_full_id_friction_calibration_result_ = full.value(); }
            else last_full_id_friction_calibration_result_ = result.value();
            bool valid = result->observable.size() == cfg_.joint_names.size() && result->validation_pass.size() == cfg_.joint_names.size();
            for(std::size_t i=0;i<cfg_.joint_names.size()&&valid;++i) valid = result->observable[i] && result->validation_pass[i];
            if(!valid) throw std::runtime_error("friction cross validation failed");
            auto candidate = machine_calibration_original_admittance_;
            candidate.calibration.friction.enabled = true; candidate.calibration.friction.velocity_transition = 0.03;
            candidate.calibration.friction.positive_coulomb = result->positive_coulomb;
            candidate.calibration.friction.positive_viscous = result->positive_viscous;
            candidate.calibration.friction.negative_coulomb = result->negative_coulomb;
            candidate.calibration.friction.negative_viscous = result->negative_viscous;
            if(!apply_runtime_admittance_cfg(candidate)) throw std::runtime_error("friction parameters apply failed");
            machine_restore_calibration_runtime(false);
            machine_calibration_kind_ = "none"; machine_calibration_phase_ = "complete"; machine_calibration_context_valid_ = false;
        }
        catch(const std::exception& error) {
            if(!machine_friction_stop_.load()) machine_calibration_error_ = error.what();
            machine_calibration_phase_ = machine_friction_stop_.load() ? "cancelled" : "failed";
            if(machine_calibration_context_valid_) machine_restore_calibration_runtime(false);
            machine_calibration_kind_ = "none";
            machine_calibration_context_valid_ = false;
        }
    }

    std::string machine_calibration_status_json() const {
        std::ostringstream out;
        out << "{\"kind\":\"" << machine_json_escape(machine_calibration_kind_) << "\",\"phase\":\"" << machine_json_escape(machine_calibration_phase_) << "\"";
        out << ",\"captured\":" << machine_calibration_poses_.size() << ",\"expected\":" << machine_calibration_expected_;
        if(!machine_calibration_error_.empty()) out << ",\"error\":\"" << machine_json_escape(machine_calibration_error_) << "\"";
        if(machine_friction_trajectory_) out << ",\"trajectory_samples\":" << machine_friction_trajectory_->positions.size() << ",\"replay_rate\":" << machine_friction_replay_rate_;
        if(last_static_calibration_result_) {
            out << ",\"static\":{\"gravity_scale\":"; machine_json_joint_vector(out,last_static_calibration_result_->gravity_scale);
            out << ",\"torque_bias\":"; machine_json_joint_vector(out,last_static_calibration_result_->torque_bias);
            out << ",\"torque_threshold\":"; machine_json_joint_vector(out,last_static_calibration_result_->torque_threshold);
            out << ",\"observable\":"; machine_json_vector(out,last_static_calibration_result_->gravity_scale_observable); out << '}';
        }
        if(last_static_validation_result_) {
            out << ",\"validation\":{\"residual_rms\":"; machine_json_joint_vector(out,last_static_validation_result_->residual_rms);
            out << ",\"residual_p99\":"; machine_json_joint_vector(out,last_static_validation_result_->residual_p99);
            out << ",\"residual_max\":"; machine_json_joint_vector(out,last_static_validation_result_->residual_max);
            out << ",\"pass\":"; machine_json_vector(out,last_static_validation_result_->pass); out << '}';
        }
        if(last_friction_calibration_result_) {
            out << ",\"friction\":{\"positive_coulomb\":"; machine_json_joint_vector(out,last_friction_calibration_result_->positive_coulomb);
            out << ",\"positive_viscous\":"; machine_json_joint_vector(out,last_friction_calibration_result_->positive_viscous);
            out << ",\"negative_coulomb\":"; machine_json_joint_vector(out,last_friction_calibration_result_->negative_coulomb);
            out << ",\"negative_viscous\":"; machine_json_joint_vector(out,last_friction_calibration_result_->negative_viscous);
            out << ",\"validation_pass\":"; machine_json_vector(out,last_friction_calibration_result_->validation_pass); out << '}';
        }
        out << '}'; return out.str();
    }

    std::string machine_snapshot_json() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream out;
        out << std::setprecision(17);
        out << "{\"robot_state\":";
        machine_json_string(out, to_string(robot_.get_state()));
        out << ",\"fault_hold_mode\":";
        machine_json_string(out, to_string(robot_.get_fault_hold_mode()));
        out << ",\"impedance_mode\":";
        machine_json_string(out, to_string(robot_.get_impedance_mode()));
        out << ",\"model_feedforward_mode\":";
        machine_json_string(out, to_string(robot_.get_model_feedforward_mode()));
        out << ",\"valid\":" << (last_output_ ? "true" : "false");
        out << ",\"sequence\":" << cycle_counter_;
        if(feedback_time_ns_ > 0) {
            const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                Robot::Clock::now().time_since_epoch()).count());
            const double age_ms = now_ns >= feedback_time_ns_ ? static_cast<double>(now_ns - feedback_time_ns_) / 1.0e6 : 0.0;
            out << ",\"feedback_age_ms\":" << age_ms;
        }
        else out << ",\"feedback_age_ms\":null";
        out << ",\"write_enabled\":" << (cfg_.runtime.write_enabled ? "true" : "false");
        out << ",\"gravity_scale\":"; machine_json_joint_vector(out, cfg_.dynamics.gravity_scale);
        out << ",\"calibration\":" << machine_calibration_status_json();
        out << ",\"model_calibration\":" << machine_model_calibration_status_json();
        out << ",\"joint_names\":[";
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            if(i) out << ',';
            machine_json_string(out, cfg_.joint_names[i]);
        }
        out << ']';
        out << ",\"actuator_info\":[";
        for(std::size_t i = 0; i < actuator_info_.size(); ++i) {
            if(i) out << ',';
            const auto& item = actuator_info_[i];
            out << "{\"name\":";
            machine_json_string(out, item.actuator_name);
            out << ",\"min_pos\":" << item.min_pos
                << ",\"max_pos\":" << item.max_pos
                << ",\"max_vel\":" << item.max_vel
                << ",\"max_effort\":" << item.max_effort
                << ",\"max_kp\":" << item.max_kp
                << ",\"max_kd\":" << item.max_kd << '}';
        }
        out << ']';
        if(const auto fault = robot_.get_last_fault()) {
            out << ",\"last_fault\":";
            machine_json_string(out, to_string(fault->code));
            if(robot_.get_state() == RobotState::FAULT) {
                out << ",\"fault\":{\"code\":";
                machine_json_string(out, to_string(fault->code));
                if(fault->code == RobotErr::SAFETY_FAILED) {
                    const auto& safety = fault->safety_fault;
                    out << ",\"safety_code\":";
                    machine_json_string(out, to_string(safety.code));
                    if(safety.index != kInvalidIndex && safety.index < cfg_.joint_names.size()) {
                        out << ",\"joint_index\":" << safety.index << ",\"joint_name\":";
                        machine_json_string(out, cfg_.joint_names[safety.index]);
                    }
                    if(std::isfinite(safety.value)) out << ",\"value\":" << safety.value;
                    if(std::isfinite(safety.limit)) out << ",\"limit\":" << safety.limit;
                }
                out << '}';
            }
            else out << ",\"fault\":null";
        }
        else out << ",\"last_fault\":null,\"fault\":null";
        // Report effective runtime limits, not merely the URDF source limits.
        out << ",\"safety_limits\":{\"max_cmd_vel\":";
        machine_json_joint_vector(out, cfg_.safety.limits.max_vel);
        out << ",\"max_state_vel\":[";
        for(std::size_t i=0;i<cfg_.safety.limits.max_vel.size();++i) {
            if(i) out << ',';
            out << cfg_.safety.limits.max_vel[i] * cfg_.safety.state_vel_fault_ratio;
        }
        out << "]}";

        if(last_output_) {
            const auto& output = *last_output_;
            out << ",\"joint\":{\"pos\":";
            machine_json_joint_vector(out, output.joint_state.pos);
            out << ",\"vel\":";
            machine_json_joint_vector(out, output.joint_state.vel);
            out << ",\"tor\":";
            machine_json_joint_vector(out, output.joint_state.tor);
            out << ",\"ref_pos\":";
            machine_json_joint_vector(out, output.joint_cmd.pos);
            out << ",\"ref_vel\":";
            machine_json_joint_vector(out, output.joint_cmd.vel);
            out << ",\"model_feedforward\":";
            machine_json_joint_vector(out, output.model_feedforward);
            out << ",\"residual_raw\":";
            machine_json_joint_vector(out, output.residual_raw);
            out << ",\"tau_ext_hat\":";
            machine_json_joint_vector(out, output.tau_ext_hat);
            out << ",\"delta_q\":";
            machine_json_joint_vector(out, output.delta_q);
            out << '}';
            out << ",\"actuator\":{\"pos\":";
            machine_json_joint_vector(out, output.actuator_state.pos);
            out << ",\"vel\":";
            machine_json_joint_vector(out, output.actuator_state.vel);
            out << ",\"tor\":";
            machine_json_joint_vector(out, output.actuator_state.tor);
            out << ",\"online\":";
            machine_json_vector(out, output.actuator_state.online);
            out << ",\"enabled\":";
            machine_json_vector(out, output.actuator_state.enabled);
            out << ",\"err_code\":";
            machine_json_vector(out, output.actuator_state.err_code);
            out << '}';
        }

        if(dynamics_.is_updated()) {
            const auto& state = dynamics_.get_state();
            out << ",\"dynamics\":{\"gravity\":";
            machine_json_joint_vector(out, state.gravity);
            out << ",\"candidate_gravity\":";
            machine_json_joint_vector(out, state.candidate_gravity);
            out << ",\"effective_gravity\":";
            machine_json_joint_vector(out, state.effective_gravity);
            out << ",\"gravity_override_active\":" << (state.gravity_override_active ? "true" : "false");
            out << ",\"gravity_compensation\":";
            machine_json_joint_vector(out, state.gravity_compensation);
            out << ",\"coriolis\":";
            machine_json_joint_vector(out, state.coriolis);
            out << ",\"inverse_dynamics\":";
            machine_json_joint_vector(out, state.inverse_dynamics);
            out << ",\"center_of_mass\":[" << state.center_of_mass.x() << ',' << state.center_of_mass.y() << ',' << state.center_of_mass.z() << ']';
            out << ",\"mass_matrix\":[";
            for(Eigen::Index row = 0; row < state.mass_matrix.rows(); ++row) {
                if(row) out << ',';
                out << '[';
                for(Eigen::Index col = 0; col < state.mass_matrix.cols(); ++col) {
                    if(col) out << ',';
                    out << state.mass_matrix(row, col);
                }
                out << ']';
            }
            out << "]}";

            out << ",\"frames\":[";
            bool first = true;
            for(const auto& name : dynamics_.get_info().frame_names) {
                const auto pose = dynamics_.get_frame_pose(name);
                if(!pose) continue;
                if(!first) out << ',';
                first = false;
                const Eigen::Quaterniond q(pose->linear());
                out << "{\"name\":";
                machine_json_string(out, name);
                out << ",\"position\":[" << pose->translation().x() << ',' << pose->translation().y() << ',' << pose->translation().z() << ']';
                out << ",\"quaternion\":[" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << "]}";
            }
            out << ']';
        }
        out << '}';
        return out.str();
    }

    void start_worker() {
        worker_running_.store(true);
        worker_ = std::thread([this]() { worker_loop(); });
    }

    void stop_worker() {
        worker_running_.store(false);
        if(worker_.joinable()) worker_.join();
    }

    void worker_loop() {
        const auto period = std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(1.0 / cfg_.runtime.ctrl_frequency_hz));
        auto next_wakeup = Robot::Clock::now();
        while(worker_running_.load()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto now = Robot::Clock::now();
                if(now > next_wakeup) next_wakeup = now;
                if(robot_.get_state() == RobotState::ACTIVE) run_control_cycle(now);
                else if(robot_.get_state() == RobotState::FAULT && robot_.is_fault_holding()) {
                    const auto hold_result = robot_.maintain_fault_hold();
                    if(!hold_result) {
                        std::cout << "\n[故障保持刷新失败，硬件已降级失能]\n";
                        print_fault(hold_result.error());
                        std::cout << "请输入菜单编号继续\n";
                    }
                }
            }
            next_wakeup += period;
            std::this_thread::sleep_until(next_wakeup);
        }
    }

    void record_model_calibration_cycle(const RobotCycleOutput& output, Robot::TimePoint now) {
        const auto phase = model_calibration_phase_.load();
        if(phase == ModelCalibrationPhase::IDLE || phase == ModelCalibrationPhase::COMPLETE ||
            phase == ModelCalibrationPhase::CANCELLED || phase == ModelCalibrationPhase::FAILED) return;
        ModelCalibrationFrame frame;
        frame.monotonic_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        frame.cycle = cycle_counter_;
        frame.position = output.joint_state.pos;
        frame.velocity = output.joint_state.vel;
        frame.torque = output.joint_state.tor;
        frame.acceleration = output.joint_acc;
        frame.reference_position = output.joint_cmd.pos;
        frame.reference_velocity = output.joint_cmd.vel;
        frame.acceleration_source = "robot_joint_acc_filter";
        frame.impedance_mode = to_string(robot_.get_impedance_mode());
        frame.feedback_age_ms = 0.0;
        frame.phase = to_string(phase);
        const int direction = model_calibration_direction_.load();
        frame.direction = direction < 0 ? "reverse" : (direction > 0 ? "forward" : "none");
        const int speed = model_calibration_speed_tier_.load();
        frame.speed_tier = speed == 1 ? "slow" : (speed == 2 ? "fast" : "none");
        frame.pose_group = model_calibration_pose_group_.load();
        frame.validation = model_calibration_validation_.load();
        auto finite = [](const JointVector& values) { return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }); };
        frame.valid = frame.position.size() == cfg_.joint_names.size() && frame.velocity.size() == cfg_.joint_names.size() &&
            frame.torque.size() == cfg_.joint_names.size() && frame.acceleration.size() == cfg_.joint_names.size() &&
            frame.reference_position.size() == cfg_.joint_names.size() && frame.reference_velocity.size() == cfg_.joint_names.size() &&
            finite(frame.position) && finite(frame.velocity) && finite(frame.torque) && finite(frame.acceleration) &&
            finite(frame.reference_position) && finite(frame.reference_velocity);
        model_calibration_recorder_.push(std::move(frame));
    }

    void run_control_cycle(Robot::TimePoint now) {
        if(stream_.enabled) {
            const auto stream_result = update_stream(now);
            if(!stream_result) {
                report_background_fault(stream_result.error());
                return;
            }
        }

        const auto cycle_result = robot_.cycle(now);
        if(!cycle_result) {
            report_background_fault(cycle_result.error());
            return;
        }
        last_output_ = cycle_result.value();
        feedback_time_ns_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count());
        ++cycle_counter_;
        record_model_calibration_cycle(*last_output_, now);
        cycle_cv_.notify_all();
    }

    tl::expected<void, RobotFault> update_stream(Robot::TimePoint now) {
        if(stream_.ref_pos.size() != cfg_.joint_names.size() || stream_.ref_vel.size() != cfg_.joint_names.size()) {
            RobotFault fault;
            fault.code = RobotErr::INVALID_CFG;
            return tl::make_unexpected(fault);
        }

        double dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        if(stream_.has_last_update_time) dt = std::chrono::duration<double>(now - stream_.last_update_time).count();
        dt = std::clamp(dt, 1.0e-6, cfg_.safety.max_dt_s);

        JointVector next_pos = stream_.ref_pos;
        JointVector next_vel = stream_.ref_vel;
        bool complete = true;
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            const double error = stream_.target_pos[i] - stream_.ref_pos[i];
            const double max_vel = cfg_.safety.limits.max_vel[i] * stream_.speed_scale;
            const double max_acc = cfg_.safety.limits.max_acc[i];
            const double direction = error > 0.0 ? 1.0 : (error < 0.0 ? -1.0 : 0.0);
            const double braking_speed = std::sqrt(std::max(0.0, 2.0 * max_acc * std::abs(error)));
            const double target_vel = direction * std::min(max_vel, braking_speed);
            const double max_delta_vel = max_acc * dt;
            next_vel[i] = stream_.ref_vel[i] + std::clamp(target_vel - stream_.ref_vel[i], -max_delta_vel, max_delta_vel);

            const double candidate_pos = stream_.ref_pos[i] + next_vel[i] * dt;
            const bool crossed_target = error != 0.0 && (stream_.target_pos[i] - candidate_pos) * error <= 0.0;
            const double pos_tolerance = std::max(1.0e-5, max_vel * dt * 0.25);
            const double vel_tolerance = std::max(1.0e-4, max_delta_vel * 0.25);
            if(crossed_target || std::abs(error) <= pos_tolerance) {
                next_pos[i] = stream_.target_pos[i];
                next_vel[i] = stream_.ref_vel[i] + std::clamp(-stream_.ref_vel[i], -max_delta_vel, max_delta_vel);
            }
            else {
                next_pos[i] = candidate_pos;
            }

            if(std::abs(stream_.target_pos[i] - next_pos[i]) > pos_tolerance || std::abs(next_vel[i]) > vel_tolerance) complete = false;
        }

        const auto result = robot_.set_cmd(JointPosVelCmd{ next_pos, next_vel }, now);
        if(!result) return result;

        stream_.ref_pos = std::move(next_pos);
        stream_.ref_vel = std::move(next_vel);
        stream_.last_update_time = now;
        stream_.has_last_update_time = true;
        if(complete && !stream_.completion_reported) {
            stream_.completion_reported = true;
            std::cout << "\n[轨迹] 参考已到达目标，继续刷新并等待实测停放判据\n请输入菜单编号继续\n";
        }
        return {};
    }

    void report_background_fault(const RobotFault& fault) {
        clear_command_sources();
        if(background_fault_reported_) return;
        background_fault_reported_ = true;
        // A FAULT must interrupt replay/teaching even while a task is paused.
        // Leave the robot in its latched safe FAULT; do not auto-clear or resume.
        const auto calibration_phase = model_calibration_phase_.load();
        if(calibration_phase == ModelCalibrationPhase::TEACHING ||
           calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
           calibration_phase == ModelCalibrationPhase::PAUSED ||
           calibration_phase == ModelCalibrationPhase::STATIC_REVERSE ||
           calibration_phase == ModelCalibrationPhase::STATIC_FORWARD ||
           calibration_phase == ModelCalibrationPhase::FRICTION_REVERSE_SLOW ||
           calibration_phase == ModelCalibrationPhase::FRICTION_FORWARD_SLOW ||
           calibration_phase == ModelCalibrationPhase::FRICTION_REVERSE_FAST ||
           calibration_phase == ModelCalibrationPhase::FRICTION_FORWARD_FAST) {
            model_calibration_fault_interrupted_.store(true);
            model_calibration_cancel_.store(true);
            model_calibration_teach_stop_.store(true);
            model_calibration_pause_.store(false);
            model_calibration_cv_.notify_all();
            cycle_cv_.notify_all();
            // Teaching and waiting-for-confirmation have no replay worker to
            // transition them into FAILED, so latch that phase here.
            if(calibration_phase == ModelCalibrationPhase::TEACHING ||
               calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION) {
                model_calibration_error_ = std::string("model calibration stopped by safety fault: ") +
                    to_string(fault.code) +
                    (fault.code == RobotErr::SAFETY_FAILED ? std::string(" / ") + to_string(fault.safety_fault.code) : std::string());
                model_calibration_set_phase(ModelCalibrationPhase::FAILED);
                // WAITING has no worker to clean up its recorder; the already
                // saved trajectory.csv remains available for later inspection.
                if(calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)
                    model_calibration_recorder_.stop();
            }
        }
        std::cout << "\n[后台 cycle 失败]\n";
        print_fault(fault);
        if(last_dynamics_err_) std::cout << "  DynamicsErr: " << to_string(*last_dynamics_err_) << '\n';
        std::cout << "请输入菜单编号继续\n";
        cycle_cv_.notify_all();
    }

    void print_banner() const {
        std::cout << "\n==============================================\n";
        std::cout << " SerialArm Terminal Main\n";
        std::cout << " backend: " << (cfg_.runtime.write_enabled ? hardware_plugin_ : "offline") << '\n';
        std::cout << " config : " << config_path_ << '\n';
        if(!robot_profile_.empty()) std::cout << " profile: " << robot_profile_ << '\n';
        std::cout << " bus    : " << connection_summary_.bus << (hardware_overrides_.bus ? " (override)" : "") << '\n';
        std::cout << " serial : " << connection_summary_.serial_port << (hardware_overrides_.serial_port ? " (override)" : "") << '\n';
        std::cout << " baud   : " << connection_summary_.baudrate << (hardware_overrides_.baudrate ? " (override)" : "") << '\n';
        std::cout << "==============================================\n";
        if(cfg_.runtime.write_enabled) {
            std::cout << "[危险] 当前终端使用真机运行前必须确认机械臂已支撑、零位、方向、限位和电机型号正确\n";
        }
        else {
            std::cout << "[离线] runtime.write_enabled=false，不连接串口、不使能电机、不写入真实硬件\n";
        }
    }

    void print_menu() const {
        std::cout << "\n------------ 主菜单 ------------\n";
        std::cout << " 1. 状态查看\n";
        std::cout << " 2. 使能 / 失能 / 故障\n";
        std::cout << " 3. 模式与补偿\n";
        std::cout << " 4. 运动与命令\n";
        std::cout << " 5. 动力学与配置\n";
        std::cout << " 6. 调参与测试\n";
        std::cout << " 0. 回到停放姿态并安全退出\n";
    }

    bool handle_menu(int selection) {
        switch(selection) {
            case 0: if(safe_exit()) return false; break;
            case 1: handle_status_menu(); break;
            case 2: handle_power_fault_menu(); break;
            case 3: handle_mode_menu(); break;
            case 4: handle_motion_menu(); break;
            case 5: handle_dynamics_menu(); break;
            case 6: handle_tuning_menu(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
        return true;
    }

    void handle_status_menu() {
        std::cout << "\n------------ 状态查看 ------------\n";
        std::cout << " 1. 查看 Robot 状态与 getter 输出\n";
        std::cout << " 2. 查看全部 Joint / Actuator 周期状态\n";
        std::cout << " 3. 查看达妙执行器静态参数\n";
        std::cout << " 4. 查看完整配置摘要\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        switch(*selection) {
            case 1: show_robot_summary(); break;
            case 2: show_all_states(); break;
            case 3: show_actuator_info(); break;
            case 4: show_config_summary(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
    }

    void handle_power_fault_menu() {
        std::cout << "\n------------ 使能 / 失能 / 故障 ------------\n";
        std::cout << " 1. activate()\n";
        std::cout << " 2. 回到停放姿态并失能\n";
        std::cout << " 3. 立即停止并失能（危险）\n";
        std::cout << " 4. clear_fault()\n";
        std::cout << " 5. FAULT 进入受限柔性恢复\n";
        std::cout << " 6. FAULT 返回刚性保持\n";
        std::cout << " 7. 查看当前故障恢复模式\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        switch(*selection) {
            case 1: activate(); break;
            case 2: park_and_deactivate(); break;
            case 3: immediate_deactivate(); break;
            case 4: clear_fault(); break;
            case 5: enter_fault_compliant_recovery(); break;
            case 6: return_to_fault_rigid_hold(); break;
            case 7: show_fault_hold_mode(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
    }

    void handle_mode_menu() {
        std::cout << "\n------------ 模式与补偿 ------------\n";
        std::cout << " 1. 切换阻抗模式\n";
        std::cout << " 2. 切换模型前馈模式（仅 INACTIVE）\n";
        std::cout << " 3. 设置重力补偿比例（仅 INACTIVE）\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        switch(*selection) {
            case 1: set_impedance_mode(); break;
            case 2: set_model_feedforward_mode(); break;
            case 3: set_gravity_scale(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
    }

    void handle_motion_menu() {
        std::cout << "\n------------ 运动与命令 ------------\n";
        std::cout << " 1. 梯形参考移动到 6 轴绝对位置\n";
        std::cout << " 2. 梯形参考执行 6 轴相对移动\n";
        std::cout << " 3. 取消当前输入并切换到当前位置保持\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        switch(*selection) {
            case 1: start_absolute_stream(); break;
            case 2: start_relative_stream(); break;
            case 3: cancel_and_hold(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
    }

    void handle_dynamics_menu() {
        std::cout << "\n------------ 动力学与配置 ------------\n";
        std::cout << " 1. 查看完整动力学向量与末端位姿\n";
        std::cout << " 2. 查看质量矩阵与末端 Jacobian\n";
        std::cout << " 3. 读取指定 Frame 的缓存位姿与 Jacobian\n";
        std::cout << " 4. 查看完整配置摘要\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        switch(*selection) {
            case 1: show_dynamics_state(); break;
            case 2: show_dynamics_matrices(); break;
            case 3: show_frame_state(); break;
            case 4: show_config_summary(); break;
            default: std::cout << "未知菜单编号\n"; break;
        }
    }

    void handle_tuning_menu() {
        std::cout << "\n------------ 调参与测试 ------------\n";
        std::cout << " 1. 导纳控制\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        if(*selection == 1) handle_admittance_tuning_menu();
        else std::cout << "未知菜单编号\n";
    }

    void handle_admittance_tuning_menu() {
        while(true) {
            std::cout << "\n------------ 导纳控制 ------------\n";
            std::cout << " 1. 导纳参数标定\n";
            std::cout << " 2. M / D / K 设置\n";
            std::cout << " 3. 状态与诊断\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: run_admittance_calibration_menu(); break;
                case 2: run_admittance_controller_menu(); break;
                case 3: run_admittance_diagnostics_menu(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    void activate() {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.activate();
        if(!result) {
            std::cout << "activate() 失败：\n";
            print_fault(result.error());
            return;
        }
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "activate() 成功，后台 cycle() 自动运行\n";
        if(robot_.get_model_feedforward_mode() == ModelFeedforwardMode::GRAVITY && is_zero_vector(dynamics_.get_gravity_scale())) {
            std::cout << "[提示] 当前已选择 GRAVITY，但 gravity_scale 全为 0，实际重力补偿力矩仍为 0\n";
        }
    }

    void park_and_deactivate() {
        std::unique_lock<std::mutex> lock(mutex_);
        park_last_error_.clear();
        clear_command_sources();
        if(robot_.get_state() == RobotState::INACTIVE) {
            std::cout << "Robot 已经处于 INACTIVE\n";
            return;
        }
        if(robot_.get_state() == RobotState::FAULT) {
            park_last_error_ = "park aborted: robot is in FAULT; inspect the fault and use manual recovery (or manually choose immediate deactivate if safe)";
            std::cout << "Robot 当前处于 FAULT，不能执行停放轨迹，请先处理故障或使用立即失能\n";
            return;
        }
        if(!cfg_.shutdown.park_before_disable) {
            std::cout << "shutdown.park_before_disable=false，将直接执行立即失能\n";
            immediate_deactivate_locked();
            return;
        }

        // 停放使用低刚度 COMPLIANT_TRACKING 降低碰撞风险，同时暂停导纳避免 park reference 被外力偏移
        robot_.set_admittance_suspended(true);
        const auto resume_admittance = [this]() {
            robot_.set_admittance_suspended(false);
            };

        const auto mode_result = robot_.set_impedance_mode(JointImpedanceMode::COMPLIANT_TRACKING, Robot::Clock::now());
        if(!mode_result) {
            park_last_error_ = "park failed: cannot enter COMPLIANT_TRACKING; robot was not automatically disabled";
            std::cout << "切换 COMPLIANT_TRACKING 失败：\n";
            print_fault(mode_result.error());
            resume_admittance();
            return;
        }

        stream_.enabled = true;
        stream_.completion_reported = false;
        stream_.speed_scale = cfg_.shutdown.speed_scale;
        stream_.target_pos = cfg_.shutdown.park_pos;
        stream_.ref_pos = robot_.get_joint_state().pos;
        stream_.ref_vel.assign(cfg_.joint_names.size(), 0.0);
        stream_.last_update_time = Robot::Clock::now();
        stream_.has_last_update_time = true;
        last_output_.reset();
        background_fault_reported_ = false;

        std::cout << "开始回到配置的停放姿态\n";
        std::cout << "[停放] 先进行柔性接近，若 40% 时间内未满足就位判据则提前切换刚性跟踪\n";
        print_vector("park_pos", cfg_.shutdown.park_pos);

        const Robot::TimePoint started_at = Robot::Clock::now();
        Robot::TimePoint last_progress_at = started_at;
        std::optional<Robot::TimePoint> settled_at;
        bool rigid_finish_mode = false;
        // Reserve at least sixty percent of the deadline for accurate final convergence
        const double rigid_finish_switch_s = 0.40 * cfg_.shutdown.timeout_s;
        while(robot_.get_state() == RobotState::ACTIVE) {
            cycle_cv_.wait_for(lock, std::chrono::milliseconds(20));
            const Robot::TimePoint now = Robot::Clock::now();
            if(!last_output_) {
                if(std::chrono::duration<double>(now - started_at).count() >= cfg_.shutdown.timeout_s) {
                    clear_command_sources();
                    (void)robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, now);
                    park_last_error_ = "park timeout: fresh joint feedback unavailable; robot remains enabled in RIGID_HOLD";
                    std::cout << "停放超时：未获取到有效关节反馈，已进入 RIGID_HOLD 并取消失能\n";
                    resume_admittance();
                    return;
                }
                continue;
            }

            double max_position_error = 0.0;
            double max_velocity = 0.0;
            std::size_t position_index = 0;
            std::size_t velocity_index = 0;
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                const double position_error = std::abs(last_output_->joint_state.pos[i] - cfg_.shutdown.park_pos[i]);
                const double velocity = std::abs(last_output_->joint_state.vel[i]);
                if(position_error > max_position_error) {
                    max_position_error = position_error;
                    position_index = i;
                }
                if(velocity > max_velocity) {
                    max_velocity = velocity;
                    velocity_index = i;
                }
            }

            const bool reached = max_position_error <= cfg_.shutdown.position_tolerance && max_velocity <= cfg_.shutdown.velocity_tolerance;
            if(reached) {
                if(!settled_at) settled_at = now;
                if(std::chrono::duration<double>(now - *settled_at).count() >= cfg_.shutdown.settle_time_s) break;
            }
            else {
                settled_at.reset();
            }

            const double elapsed_s = std::chrono::duration<double>(now - started_at).count();
            if(!rigid_finish_mode && !reached && elapsed_s >= rigid_finish_switch_s) {
                const auto rigid_result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING, now);
                if(!rigid_result) {
                    clear_command_sources();
                    (void)robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, now);
                    park_last_error_ = "park failed: cannot switch to RIGID_TRACKING during final approach; robot remains enabled in RIGID_HOLD";
                    std::cout << "精确收敛阶段切换 RIGID_TRACKING 失败，已进入 RIGID_HOLD 并取消失能\n";
                    print_fault(rigid_result.error());
                    resume_admittance();
                    return;
                }
                rigid_finish_mode = true;
                std::cout << "[停放] 进入精确收敛阶段，切换 RIGID_TRACKING\n";
            }

            if(std::chrono::duration<double>(now - last_progress_at).count() >= 1.0) {
                std::cout << "[停放] 最大位置误差=" << max_position_error << " rad (" << cfg_.joint_names[position_index] << ")，最大速度="
                    << max_velocity << " rad/s (" << cfg_.joint_names[velocity_index] << ")\n";
                last_progress_at = now;
            }

            if(elapsed_s > cfg_.shutdown.timeout_s) {
                clear_command_sources();
                (void)robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, now);
                std::ostringstream detail;
                detail << "park timeout: position error=" << std::fixed << std::setprecision(6)
                       << max_position_error << " rad (" << cfg_.joint_names[position_index] << ", limit="
                       << cfg_.shutdown.position_tolerance << " rad); speed=" << max_velocity << " rad/s ("
                       << cfg_.joint_names[velocity_index] << ", limit=" << cfg_.shutdown.velocity_tolerance
                       << " rad/s); timeout=" << cfg_.shutdown.timeout_s
                       << "s. Robot remains enabled in RIGID_HOLD; no automatic disable. Check park_pos,"
                       << " feedback/zero offset, friction and tracking tuning before considering any tolerance change.";
                park_last_error_ = detail.str();
                std::cout << "停放流程超时，最终严格就位判据仍未满足，已切换 RIGID_HOLD 并取消失能\n";
                std::cout << "最差位置误差=" << max_position_error << " rad (" << cfg_.joint_names[position_index] << ")，严格允许="
                    << cfg_.shutdown.position_tolerance << " rad\n";
                std::cout << "最差速度=" << max_velocity << " rad/s (" << cfg_.joint_names[velocity_index] << ")，严格允许="
                    << cfg_.shutdown.velocity_tolerance << " rad/s\n";
                resume_admittance();
                return;
            }
        }

        if(robot_.get_state() != RobotState::ACTIVE) {
            park_last_error_ = "park interrupted: robot left ACTIVE during approach (" + to_string(robot_.get_state()) + ")";
            std::cout << "停放过程中 Robot 离开 ACTIVE：" << to_string(robot_.get_state()) << '\n';
            resume_admittance();
            return;
        }

        clear_command_sources();
        const auto hold_result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, Robot::Clock::now());
        if(!hold_result) {
            park_last_error_ = "park failed: cannot enter RIGID_HOLD; robot was not automatically disabled";
            std::cout << "停放后切换 RIGID_HOLD 失败：\n";
            print_fault(hold_result.error());
            resume_admittance();
            return;
        }

        const Robot::TimePoint hold_until = Robot::Clock::now() + std::chrono::milliseconds(200);
        while(robot_.get_state() == RobotState::ACTIVE && Robot::Clock::now() < hold_until) {
            cycle_cv_.wait_for(lock, std::chrono::milliseconds(20));
        }

        const auto result = robot_.deactivate();
        if(!result) {
            park_last_error_ = "park reached the configured pose but deactivate() failed; inspect hardware state";
            std::cout << "停放后 deactivate() 失败：\n";
            print_fault(result.error());
            resume_admittance();
            return;
        }
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "已到达停放姿态并失能，Robot 回到 INACTIVE\n";
    }

    void immediate_deactivate() {
        std::lock_guard<std::mutex> lock(mutex_);
        immediate_deactivate_locked();
    }

    void immediate_deactivate_locked() {
        clear_command_sources();
        const auto result = robot_.force_deactivate();
        if(!result) {
            std::cout << "立即失能失败：\n";
            print_fault(result.error());
            return;
        }
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "已立即失能，重力负载机械臂可能下落\n";
    }

    void clear_fault() {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.clear_fault();
        if(!result) {
            std::cout << "clear_fault() 失败：\n";
            print_fault(result.error());
            return;
        }
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "clear_fault() 成功，已清除外部命令并进入 ACTIVE + RIGID_HOLD\n";
    }

    void enter_fault_compliant_recovery() {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.enter_fault_compliant_recovery();
        if(!result) {
            std::cout << "enter_fault_compliant_recovery() 失败：\n";
            print_fault(result.error());
            return;
        }
        background_fault_reported_ = false;
        std::cout << "已进入 FAULT 受限柔性恢复\n";
    }

    void return_to_fault_rigid_hold() {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.return_to_fault_rigid_hold();
        if(!result) {
            std::cout << "return_to_fault_rigid_hold() 失败：\n";
            print_fault(result.error());
            return;
        }
        background_fault_reported_ = false;
        std::cout << "已返回 FAULT 刚性保持\n";
    }

    void show_fault_hold_mode() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "FaultHoldMode           : " << to_string(robot_.get_fault_hold_mode()) << '\n';
    }

    void set_impedance_mode() {
        std::cout << "\n1 RIGID_HOLD\n2 RIGID_TRACKING\n3 COMPLIANT_HOLD\n4 COMPLIANT_DRAG\n5 COMPLIANT_TRACKING\n";
        const auto selection = read_int("模式: ");
        if(!selection || *selection < 1 || *selection > 5) {
            std::cout << "模式输入无效\n";
            return;
        }

        const JointImpedanceMode mode = static_cast<JointImpedanceMode>(*selection - 1);
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        const auto result = robot_.set_impedance_mode(mode);
        if(!result) {
            std::cout << "set_impedance_mode() 失败：\n";
            print_fault(result.error());
            return;
        }
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "模式已切换为 " << to_string(mode) << "\n";
    }

    void set_model_feedforward_mode() {
        std::cout << "\n1 NONE\n2 GRAVITY\n3 FULL_INVERSE_DYNAMICS\n";
        const auto selection = read_int("模式: ");
        if(!selection || *selection < 1 || *selection > 3) {
            std::cout << "模式输入无效\n";
            return;
        }

        const ModelFeedforwardMode mode = static_cast<ModelFeedforwardMode>(*selection - 1);
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result = robot_.set_model_feedforward_mode(mode);
        if(!result) {
            std::cout << "set_model_feedforward_mode() 失败：\n";
            print_fault(result.error());
            return;
        }
        cfg_.runtime.model_feedforward_mode = mode;
        std::cout << "模型前馈模式已切换为 " << to_string(mode) << "\n";
        if(mode == ModelFeedforwardMode::GRAVITY && is_zero_vector(dynamics_.get_gravity_scale())) {
            std::cout << "[提示] gravity_scale 全为 0，当前 GRAVITY 模式不会产生实际补偿，请在 INACTIVE 状态使用“模式与补偿 > 设置重力补偿比例”\n";
        }
    }

    void start_absolute_stream() {
        const auto target = read_vector("输入目标位置 rad，以空格分隔: ", cfg_.joint_names.size());
        const auto speed_scale = read_double("速度比例 (0, 1]，建议 0.1~0.5: ");
        if(!target || !speed_scale || *speed_scale <= 0.0 || *speed_scale > 1.0) {
            std::cout << "目标位置或速度比例输入无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE || !is_tracking_mode(robot_.get_impedance_mode())) {
            std::cout << "请先 activate()，并切换到 RIGID_TRACKING 或 COMPLIANT_TRACKING\n";
            return;
        }
        begin_stream(*target, *speed_scale);
    }

    void start_relative_stream() {
        const auto delta = read_vector("输入相对位移 rad，以空格分隔: ", cfg_.joint_names.size());
        const auto speed_scale = read_double("速度比例 (0, 1]，建议 0.1~0.5: ");
        if(!delta || !speed_scale || *speed_scale <= 0.0 || *speed_scale > 1.0) {
            std::cout << "相对位移或速度比例输入无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE || !is_tracking_mode(robot_.get_impedance_mode())) {
            std::cout << "请先 activate()，并切换到 RIGID_TRACKING 或 COMPLIANT_TRACKING\n";
            return;
        }

        JointVector start_pos = current_reference_pos();
        JointVector target(start_pos.size(), 0.0);
        for(std::size_t i = 0; i < target.size(); ++i) target[i] = start_pos[i] + (*delta)[i];
        begin_stream(target, *speed_scale);
    }

    void begin_stream(const JointVector& target, double speed_scale) {
        clear_command_sources();
        stream_.enabled = true;
        stream_.completion_reported = false;
        stream_.speed_scale = speed_scale;
        stream_.target_pos = target;
        stream_.ref_pos = current_reference_pos();
        stream_.ref_vel = current_reference_vel();
        stream_.last_update_time = Robot::Clock::now();
        stream_.has_last_update_time = true;

        const JointState& measured = robot_.get_joint_state();
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            if(i < measured.pos.size() && std::abs(stream_.ref_pos[i] - measured.pos[i]) > 0.05) {
                std::cout << "[提示] " << cfg_.joint_names[i] << " 命令-实测位置滞后=" << stream_.ref_pos[i] - measured.pos[i] << " rad\n";
            }
        }
        std::cout << "已开始连续梯形参考，速度上限比例=" << speed_scale << "\n";
    }

    JointVector current_reference_pos() const {
        if(last_output_ && last_output_->joint_cmd.pos.size() == cfg_.joint_names.size()) return last_output_->joint_cmd.pos;
        const JointState& state = robot_.get_joint_state();
        if(state.pos.size() == cfg_.joint_names.size()) return state.pos;
        return JointVector(cfg_.joint_names.size(), 0.0);
    }

    JointVector current_reference_vel() const {
        if(last_output_ && last_output_->joint_cmd.vel.size() == cfg_.joint_names.size()) return last_output_->joint_cmd.vel;
        return JointVector(cfg_.joint_names.size(), 0.0);
    }

    void cancel_and_hold() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "Robot 当前不是 ACTIVE\n";
            return;
        }
        clear_command_sources();
        const auto result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD);
        if(!result) {
            std::cout << "切换 RIGID_HOLD 失败：\n";
            print_fault(result.error());
            return;
        }
        last_output_.reset();
        std::cout << "已取消外部命令并切换到当前位置刚性保持\n";
    }

    void show_robot_summary() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "RobotState             : " << to_string(robot_.get_state()) << '\n';
        std::cout << "JointImpedanceMode      : " << to_string(robot_.get_impedance_mode()) << '\n';
        std::cout << "ModelFeedforwardMode    : " << to_string(robot_.get_model_feedforward_mode()) << '\n';
        std::cout << "tracking_mode           : " << to_string(cfg_.runtime.tracking_impedance_mode) << '\n';
        std::cout << "admittance_capability   : " << (cfg_.capability.admittance.enabled ? "ENABLED" : "DISABLED") << '\n';
        std::cout << "FaultHoldMode           : " << to_string(robot_.get_fault_hold_mode()) << '\n';
        std::cout << "Dynamics configured     : " << std::boolalpha << dynamics_.is_configured() << '\n';
        std::cout << "Dynamics updated        : " << std::boolalpha << dynamics_.is_updated() << '\n';
        std::cout << "Fault rigid hold        : " << std::boolalpha << robot_.is_fault_holding() << '\n';
        std::cout << "Streaming command       : " << std::boolalpha << stream_.enabled << '\n';
        if(robot_.get_last_fault()) print_fault(*robot_.get_last_fault());
    }

    void show_all_states() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!last_output_) {
            std::cout << "尚无成功控制周期输出，请先 activate()\n";
            return;
        }

        const auto& output = *last_output_;
        std::cout << "\nJoint feedback:\n";
        std::cout << std::left
            << std::setw(10) << "joint"
            << std::setw(13) << "pos(rad)"
            << std::setw(13) << "vel(rad/s)"
            << std::setw(13) << "acc(rad/s2)"
            << std::setw(13) << "tor(Nm)" << '\n';
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            std::cout << std::left
                << std::setw(10) << cfg_.joint_names[i]
                << std::setw(13) << output.joint_state.pos[i]
                << std::setw(13) << output.joint_state.vel[i]
                << std::setw(13) << output.joint_acc[i]
                << std::setw(13) << output.joint_state.tor[i] << '\n';
        }

        std::cout << "\nJoint command:\n";
        std::cout << std::left
            << std::setw(10) << "joint"
            << std::setw(13) << "pos"
            << std::setw(13) << "vel"
            << std::setw(13) << "ref_acc"
            << std::setw(13) << "tor"
            << std::setw(13) << "model_ff"
            << std::setw(10) << "kp"
            << std::setw(10) << "kd" << '\n';
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            std::cout << std::left
                << std::setw(10) << cfg_.joint_names[i]
                << std::setw(13) << output.joint_cmd.pos[i]
                << std::setw(13) << output.joint_cmd.vel[i]
                << std::setw(13) << output.joint_ref_acc[i]
                << std::setw(13) << output.joint_cmd.tor[i]
                << std::setw(13) << output.model_feedforward[i]
                << std::setw(10) << output.joint_cmd.kp[i]
                << std::setw(10) << output.joint_cmd.kd[i] << '\n';
        }

        std::cout << "\nActuator feedback and backend command:\n";
        std::cout << std::left
            << std::setw(11) << "actuator"
            << std::setw(8) << "id"
            << std::setw(12) << "pos"
            << std::setw(12) << "vel"
            << std::setw(12) << "tor"
            << std::setw(8) << "online"
            << std::setw(8) << "enable"
            << std::setw(8) << "err"
            << std::setw(12) << "cmd_pos"
            << std::setw(12) << "cmd_vel"
            << std::setw(12) << "cmd_tor"
            << std::setw(10) << "kp"
            << std::setw(10) << "kd" << '\n';
        for(std::size_t i = 0; i < actuator_info_.size(); ++i) {
            std::cout << std::left
                << std::setw(11) << actuator_info_[i].actuator_name
                << std::setw(8) << i
                << std::setw(12) << output.actuator_state.pos[i]
                << std::setw(12) << output.actuator_state.vel[i]
                << std::setw(12) << output.actuator_state.tor[i]
                << std::setw(8) << static_cast<int>(output.actuator_state.online[i])
                << std::setw(8) << static_cast<int>(output.actuator_state.enabled[i])
                << std::setw(8) << output.actuator_state.err_code[i]
                << std::setw(12) << output.actuator_cmd.pos[i]
                << std::setw(12) << output.actuator_cmd.vel[i]
                << std::setw(12) << output.actuator_cmd.tor[i]
                << std::setw(10) << output.actuator_cmd.kp[i]
                << std::setw(10) << output.actuator_cmd.kd[i] << '\n';
        }
        std::cout << "cycle dt: " << output.dt << " s\n";
    }

    void show_dynamics_state() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!dynamics_.is_updated()) {
            std::cout << "Dynamics 尚未完成首次 update()，请先 activate() 并等待一个周期\n";
            return;
        }

        const DynamicsInfo& info = dynamics_.get_info();
        const DynamicsState& state = dynamics_.get_state();
        std::cout << "joints_count: " << info.joints_count << ", nq: " << info.nq << ", nv: " << info.nv << ", total_mass: " << info.total_mass << " kg\n";
        print_vector("q", state.pos);
        print_vector("dq", state.vel);
        print_vector("ddq_est", state.acc);
        print_vector("tau_feedback", state.tor);
        print_vector("ddq_ref", state.ref_acc);
        print_vector("gravity", state.gravity);
        print_vector("gravity_scale", dynamics_.get_gravity_scale());
        print_vector("gravity_compensation", state.gravity_compensation);
        print_vector("nonlinear", state.nonlinear);
        print_vector("coriolis", state.coriolis);
        print_vector("inverse_dynamics", state.inverse_dynamics);
        print_vector("forward_dynamics", state.forward_dynamics);
        print_pose("tool_pose", state.tool_pose);
    }

    void show_dynamics_matrices() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!dynamics_.is_updated()) {
            std::cout << "Dynamics 尚未完成首次 update()\n";
            return;
        }
        print_matrix("mass_matrix", dynamics_.get_mass_matrix());
        print_matrix("tool_jacobian", dynamics_.get_tool_jacobian());
    }

    void show_actuator_info() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << std::left
            << std::setw(12) << "actuator"
            << std::setw(12) << "min_pos"
            << std::setw(12) << "max_pos"
            << std::setw(12) << "max_vel"
            << std::setw(12) << "max_effort"
            << std::setw(12) << "max_kp"
            << std::setw(12) << "max_kd" << '\n';
        for(const auto& info : actuator_info_) {
            std::cout << std::left
                << std::setw(12) << info.actuator_name
                << std::setw(12) << info.min_pos
                << std::setw(12) << info.max_pos
                << std::setw(12) << info.max_vel
                << std::setw(12) << info.max_effort
                << std::setw(12) << info.max_kp
                << std::setw(12) << info.max_kd << '\n';
        }

        std::cout << "\nJoint/Actuator mapping:\n";
        print_vector("pos_ratio", cfg_.mapper.pos_ratio);
        print_vector("tor_ratio", cfg_.mapper.tor_ratio);
        print_int_vector("direction", cfg_.mapper.direction);
        print_vector("joint_zero_offset", cfg_.mapper.joint_zero_offset);
        print_vector("actuator_zero_offset", cfg_.mapper.actuator_zero_offset);
    }

    void set_gravity_scale() {
        const auto scale = read_vector("输入重力补偿比例 [0, 2]，以空格分隔（1.0=URDF 原模型，>1.0=补偿模型低估）: ", cfg_.joint_names.size());
        if(!scale) {
            std::cout << "输入无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::INACTIVE) {
            std::cout << "请先 deactivate()，重力补偿比例只允许在 INACTIVE 修改\n";
            return;
        }
        const auto result = dynamics_.set_gravity_scale(*scale);
        if(!result) {
            std::cout << "set_gravity_scale() 失败: " << to_string(result.error()) << '\n';
            if(result.error() == DynamicsErr::GRAVITY_SCALE_OUT_OF_RANGE) {
                std::cout << "重力补偿比例必须位于 [0, 2]，本次输入整组未生效\n";
            }
            return;
        }
        cfg_.dynamics.gravity_scale = *scale;
        std::cout << "重力补偿比例已更新该值只修改当前进程，不会回写 YAML\n";
    }


    bool ensure_admittance_tuning_active() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "请先 activate()，导纳标定/调参只允许在 ACTIVE 状态进行\n";
            return false;
        }
        if(!dynamics_.is_updated()) {
            std::cout << "动力学尚未完成首次更新，请稍后重试\n";
            return false;
        }
        return true;
    }

    bool set_tuning_impedance_mode(JointImpedanceMode mode) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result = robot_.set_impedance_mode(mode);
        if(!result) {
            std::cout << "切换阻抗模式失败：\n";
            print_fault(result.error());
            return false;
        }
        last_output_.reset();
        return true;
    }

    bool apply_runtime_admittance_cfg(const AdmittanceCapabilityCfg& candidate) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result = robot_.set_admittance_cfg(candidate);
        if(!result) {
            std::cout << "更新导纳参数失败：\n";
            print_fault(result.error());
            return false;
        }
        cfg_.capability.admittance = candidate;
        last_output_.reset();
        return true;
    }

    bool wait_for_static_tuning_pose(double stable_s, double timeout_s, double max_abs_vel, double max_abs_acc) {
        const auto deadline = Robot::Clock::now() +
            std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(timeout_s));
        std::optional<Robot::TimePoint> stable_since;
        std::uint64_t cursor = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cursor = cycle_counter_;
        }

        while(Robot::Clock::now() < deadline) {
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                return cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                });
            if(robot_.get_state() != RobotState::ACTIVE) return false;
            if(!updated || cycle_counter_ <= cursor || !last_output_) continue;
            cursor = cycle_counter_;

            const auto& vel = last_output_->joint_state.vel;
            const auto& acc = last_output_->joint_acc;
            if(vel.size() != cfg_.joint_names.size() || acc.size() != cfg_.joint_names.size()) {
                stable_since.reset();
                continue;
            }

            double peak_vel = 0.0;
            double peak_acc = 0.0;
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                peak_vel = std::max(peak_vel, std::abs(vel[i]));
                peak_acc = std::max(peak_acc, std::abs(acc[i]));
            }

            const auto now = Robot::Clock::now();
            if(peak_vel <= max_abs_vel && peak_acc <= max_abs_acc) {
                if(!stable_since) stable_since = now;
                const double held = std::chrono::duration<double>(now - *stable_since).count();
                if(held >= stable_s) return true;
            }
            else {
                stable_since.reset();
            }
        }
        return false;
    }

    std::optional<AdmittanceStaticPoseSamples> collect_static_pose_samples(double settle_s, double sample_s) {
        if(settle_s > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(settle_s));

        AdmittanceStaticPoseSamples pose;
        const auto deadline = Robot::Clock::now() + std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(sample_s));
        std::uint64_t cursor = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cursor = cycle_counter_;
        }

        while(Robot::Clock::now() < deadline) {
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                return cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                });
            if(robot_.get_state() != RobotState::ACTIVE) return std::nullopt;
            if(!updated || cycle_counter_ <= cursor || !last_output_ || !dynamics_.is_updated()) continue;
            cursor = cycle_counter_;
            pose.samples.push_back(AdmittanceStaticSample{
                dynamics_.get_gravity(),
                last_output_->joint_state.tor,
                });
        }
        if(pose.samples.empty()) return std::nullopt;
        return pose;
    }

    std::optional<std::vector<RobotCycleOutput>> collect_admittance_outputs(double settle_s, double sample_s) {
        if(settle_s > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(settle_s));

        std::vector<RobotCycleOutput> outputs;
        const auto deadline = Robot::Clock::now() + std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(sample_s));
        std::uint64_t cursor = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cursor = cycle_counter_;
        }
        while(Robot::Clock::now() < deadline) {
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                return cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                });
            if(robot_.get_state() != RobotState::ACTIVE) return std::nullopt;
            if(!updated || cycle_counter_ <= cursor || !last_output_) continue;
            cursor = cycle_counter_;
            outputs.push_back(*last_output_);
        }
        if(outputs.empty()) return std::nullopt;
        return outputs;
    }


    std::optional<FrictionCalibrationTrajectory> collect_friction_drag_trajectory_until_stop(
        const std::atomic<bool>& stop_recording,
        const std::function<void(const FrictionCalibrationTrajectory&)>& checkpoint = {}) {
        constexpr std::size_t kRecordStride = 4; // 200 Hz control -> about 50 Hz recorded path
        FrictionCalibrationTrajectory trajectory;
        trajectory.sample_dt = static_cast<double>(kRecordStride) / cfg_.runtime.ctrl_frequency_hz;

        std::uint64_t cursor = 0;
        std::size_t received_cycles = 0;
        std::size_t last_checkpoint_samples = 0;
        const std::size_t checkpoint_interval = std::max<std::size_t>(20, static_cast<std::size_t>(2.0 / trajectory.sample_dt));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cursor = cycle_counter_;
        }

        while(!stop_recording.load()) {
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                return stop_recording.load() || cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                });
            // A FAULT or deliberate disable must stop sampling immediately,
            // but must not discard samples from earlier valid control cycles.
            if(robot_.get_state() != RobotState::ACTIVE) break;
            if(stop_recording.load()) break;
            if(!updated || cycle_counter_ <= cursor || !last_output_) continue;
            cursor = cycle_counter_;
            if((received_cycles++ % kRecordStride) == 0) {
                trajectory.positions.push_back(last_output_->joint_state.pos);
                if(checkpoint && trajectory.positions.size() >= last_checkpoint_samples + checkpoint_interval) {
                    // Sampling has its own worker, but never write a file while
                    // holding the mutex that guards the real-time cycle state.
                    lock.unlock();
                    checkpoint(trajectory);
                    last_checkpoint_samples = trajectory.positions.size();
                }
            }
        }

        if(trajectory.positions.size() < 20) return std::nullopt;
        return trajectory;
    }

    FrictionCalibrationTrajectory smooth_friction_trajectory(const FrictionCalibrationTrajectory& input) const {
        FrictionCalibrationTrajectory result = input;
        if(input.positions.size() < 5) return result;
        for(std::size_t k = 2; k + 2 < input.positions.size(); ++k) {
            result.positions[k].assign(cfg_.joint_names.size(), 0.0);
            for(std::size_t joint = 0; joint < cfg_.joint_names.size(); ++joint) {
                double sum = 0.0;
                for(std::size_t j = k - 2; j <= k + 2; ++j) sum += input.positions[j][joint];
                result.positions[k][joint] = sum / 5.0;
            }
        }
        return result;
    }

    bool friction_trajectory_inside_safe_replay_range(const FrictionCalibrationTrajectory& trajectory) const {
        constexpr double kCalibrationInnerMargin = 0.05; // rad; calibration replay stays away from URDF hard limits
        const auto& limits = cfg_.safety.limits;
        for(const auto& q : trajectory.positions) {
            if(q.size() != cfg_.joint_names.size()) return false;
            for(std::size_t i = 0; i < q.size(); ++i) {
                if(!std::isfinite(q[i])) return false;
                if(i < limits.has_position_limit.size() && limits.has_position_limit[i] != 0) {
                    const double margin = std::max(kCalibrationInnerMargin, limits.pos_margin[i]);
                    const double lower = limits.min_pos[i] + margin;
                    const double upper = limits.max_pos[i] - margin;
                    if(lower < upper && (q[i] < lower || q[i] > upper)) {
                        std::cout << "[拒绝回放] " << cfg_.joint_names[i]
                            << " 示教轨迹进入距硬限位 " << margin
                            << " rad 的标定保护区；请重新示教更保守的轨迹\n";
                        return false;
                    }
                }
            }
        }
        return true;
    }

    double friction_replay_rate(const FrictionCalibrationTrajectory& trajectory) const {
        constexpr double kNominalReplayRate = 0.50; // 默认按示教时间轴的 0.5 倍速回放
        constexpr double kSafetyVelocityRatio = 0.35;
        constexpr double kSafetyAccelerationRatio = 0.35;
        double rate = kNominalReplayRate;
        if(trajectory.positions.size() < 2 || trajectory.sample_dt <= 0.0) return rate;

        for(std::size_t k = 1; k < trajectory.positions.size(); ++k) {
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                const double demonstrated_speed = std::abs(
                    trajectory.positions[k][i] - trajectory.positions[k - 1][i]) / trajectory.sample_dt;
                if(demonstrated_speed > 1.0e-9) {
                    const double safe_speed = kSafetyVelocityRatio * cfg_.safety.limits.max_vel[i];
                    if(safe_speed > 0.0) rate = std::min(rate, safe_speed / demonstrated_speed);
                }

                if(k >= 2) {
                    const double demonstrated_acceleration = std::abs(
                        trajectory.positions[k][i] - 2.0 * trajectory.positions[k - 1][i] +
                        trajectory.positions[k - 2][i]) /
                        (trajectory.sample_dt * trajectory.sample_dt);
                    if(demonstrated_acceleration > 1.0e-9) {
                        const double safe_acceleration =
                            kSafetyAccelerationRatio * cfg_.safety.limits.max_acc[i];
                        if(safe_acceleration > 0.0) {
                            rate = std::min(
                                rate,
                                std::sqrt(safe_acceleration / demonstrated_acceleration));
                        }
                    }
                }
            }
        }
        return std::min(rate, kNominalReplayRate);
    }

    bool play_friction_calibration_pass(
        const FrictionCalibrationTrajectory& trajectory,
        bool reverse,
        double playback_rate,
        const AdmittanceCapabilityCfg& admittance_cfg,
        std::vector<AdmittanceFrictionSample>& selected_samples,
        std::vector<AdmittanceFrictionSample>* full_id_samples = nullptr) {
        if(trajectory.positions.size() < 2 || trajectory.sample_dt <= 0.0) return false;

        GeneralizedMomentumObserver momentum_observer;
        if(admittance_cfg.observer.mode == AdmittanceObserverMode::MOMENTUM) {
            GeneralizedMomentumObserverCfg momentum_cfg;
            momentum_cfg.joints_count = cfg_.joint_names.size();
            momentum_cfg.gain = admittance_cfg.observer.momentum_gain;
            momentum_cfg.initial_residual = admittance_cfg.calibration.torque_bias;
            if(!momentum_observer.configure(momentum_cfg)) return false;

        }

        std::vector<JointVector> ordered = trajectory.positions;
        if(reverse) std::reverse(ordered.begin(), ordered.end());

        const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        const double index_step = playback_rate * nominal_dt / trajectory.sample_dt;
        if(!std::isfinite(index_step) || index_step <= 0.0) return false;

        JointVector previous_ref = ordered.front();
        double progress = 0.0;
        while(progress < static_cast<double>(ordered.size() - 1)) {
            if(machine_friction_stop_.load()) return false;
            const double next_progress = std::min(
                static_cast<double>(ordered.size() - 1), progress + index_step);
            const std::size_t lower = static_cast<std::size_t>(std::floor(next_progress));
            const std::size_t upper = std::min(lower + 1, ordered.size() - 1);
            const double ratio = next_progress - static_cast<double>(lower);

            JointVector ref_pos(cfg_.joint_names.size(), 0.0);
            JointVector ref_vel(cfg_.joint_names.size(), 0.0);
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                ref_pos[i] = ordered[lower][i] * (1.0 - ratio) + ordered[upper][i] * ratio;
                ref_vel[i] = (ref_pos[i] - previous_ref[i]) / nominal_dt;
            }

            std::uint64_t cursor = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(robot_.get_state() != RobotState::ACTIVE) return false;
                cursor = cycle_counter_;
                const auto command = robot_.set_cmd(JointPosVelCmd{ ref_pos, ref_vel }, Robot::Clock::now());
                if(!command) {
                    std::cout << "回放命令失败：\n";
                    print_fault(command.error());
                    return false;
                }
            }

            RobotCycleOutput output;
            JointVector model_torque;
            JointVector gravity;
            JointVector coriolis;
            std::vector<JointVector> mass_matrix;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                    return cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                    });
                if(!updated || robot_.get_state() != RobotState::ACTIVE || !last_output_) return false;
                output = *last_output_;

                // Replay calibration always uses actual state. FULL-ID is retained only as
                // a diagnostic reference; when MOMENTUM is selected the fitted friction
                // residual must come from the same observer family used at runtime
                const auto dynamics_result = dynamics_.update(
                    output.joint_state, output.joint_acc, output.joint_acc);
                if(!dynamics_result) {
                    std::cout << "回放动力学计算失败: " << to_string(dynamics_result.error()) << '\n';
                    return false;
                }
                model_torque = dynamics_.get_inverse_dynamics();
                gravity = dynamics_.get_gravity_compensation();
                coriolis = dynamics_.get_coriolis();
                const auto& mass = dynamics_.get_mass_matrix();
                const std::size_t n = cfg_.joint_names.size();
                if(mass.rows() != static_cast<Eigen::Index>(n) ||
                    mass.cols() != static_cast<Eigen::Index>(n)) return false;
                mass_matrix.assign(n, JointVector(n, 0.0));
                for(std::size_t i = 0; i < n; ++i) {
                    for(std::size_t j = 0; j < n; ++j) {
                        mass_matrix[i][j] = mass(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
                    }
                }
            }

            const std::size_t n = cfg_.joint_names.size();
            JointVector full_id_after_bias(n, 0.0);
            for(std::size_t i = 0; i < n; ++i) {
                full_id_after_bias[i] = model_torque[i] - output.joint_state.tor[i] -
                    admittance_cfg.calibration.torque_bias[i];
            }

            JointVector selected_after_bias = full_id_after_bias;
            if(admittance_cfg.observer.mode == AdmittanceObserverMode::MOMENTUM) {
                GeneralizedMomentumInput momentum_input;
                momentum_input.measured_torque = output.joint_state.tor;
                momentum_input.gravity = std::move(gravity);
                momentum_input.coriolis = std::move(coriolis);
                momentum_input.mass_matrix = std::move(mass_matrix);
                momentum_input.velocity = output.joint_state.vel;
                momentum_input.dt = output.dt > 0.0 ? output.dt : nominal_dt;
                const auto momentum = momentum_observer.update(momentum_input);
                if(!momentum) return false;
                selected_after_bias.resize(n);
                for(std::size_t i = 0; i < n; ++i) {
                    selected_after_bias[i] = momentum->tau_ext_hat[i] -
                        admittance_cfg.calibration.torque_bias[i];
                }
            }

            selected_samples.push_back(AdmittanceFrictionSample{
                output.joint_state.vel,
                output.joint_acc,
                std::move(selected_after_bias),
                });
            if(full_id_samples) {
                full_id_samples->push_back(AdmittanceFrictionSample{
                    output.joint_state.vel,
                    output.joint_acc,
                    std::move(full_id_after_bias),
                    });
            }

            previous_ref = std::move(ref_pos);
            progress = next_progress;
        }
        return true;
    }

    void print_friction_calibration_yaml(
        const FrictionResidualModelCfg& friction,
        const std::string& indent) const
    {
        std::cout << indent << "friction:\n";
        std::cout << indent << "  enabled: " << (friction.enabled ? "true" : "false") << '\n';
        std::cout << indent << "  velocity_transition: " << std::fixed << std::setprecision(6)
            << friction.velocity_transition << '\n';
        std::cout << indent << "  "; print_joint_yaml_map("positive_coulomb", cfg_.joint_names, friction.positive_coulomb);
        std::cout << indent << "  "; print_joint_yaml_map("positive_viscous", cfg_.joint_names, friction.positive_viscous);
        std::cout << indent << "  "; print_joint_yaml_map("negative_coulomb", cfg_.joint_names, friction.negative_coulomb);
        std::cout << indent << "  "; print_joint_yaml_map("negative_viscous", cfg_.joint_names, friction.negative_viscous);
    }

    bool run_admittance_friction_calibration(bool print_yaml = true) {
        if(!ensure_admittance_tuning_active()) return false;

        JointImpedanceMode original_mode;
        AdmittanceCapabilityCfg original_admittance;
        bool original_suspended{ false };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            original_mode = robot_.get_impedance_mode();
            original_admittance = cfg_.capability.admittance;
            original_suspended = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        const auto restore_runtime = [this, original_suspended, original_mode]() {
            set_tuning_impedance_mode(original_mode);
            std::lock_guard<std::mutex> lock(mutex_);
            robot_.set_admittance_suspended(original_suspended);
            last_output_.reset();
            };

        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            restore_runtime();
            return false;
        }

        std::cout << "摩擦参数标定：进入 COMPLIANT_DRAG 后记录示教轨迹\n";
        std::cout << "建议用约 20 s 完成示教，每个轴都要包含双向运动轨迹，并尽量包含快慢变化\n";
        std::cout << "当轨迹覆盖充分后先完全松手，再按 Enter 结束示教\n";
        std::string line;
        if(!read_line("准备好按 Enter 开始，输入 q 取消: ", line) || line == "q" || line == "Q") {
            std::cout << "已取消\n";
            restore_runtime();
            return false;
        }

        std::atomic<bool> stop_recording{ false };
        std::optional<FrictionCalibrationTrajectory> trajectory;
        std::thread recorder([&]() {
            trajectory = collect_friction_drag_trajectory_until_stop(stop_recording);
            });
        const bool input_ok = read_line(
            "示教中；每个轴完成双向快慢运动并完全松手后按 Enter 结束示教，输入 q 取消: ",
            line);
        stop_recording.store(true);
        cycle_cv_.notify_all();
        recorder.join();

        if(!input_ok || line == "q" || line == "Q") {
            std::cout << "已取消\n";
            restore_runtime();
            return false;
        }
        if(!trajectory) {
            std::cout << "摩擦参数标定：FAIL（示教轨迹记录失败或轨迹过短）\n";
            restore_runtime();
            return false;
        }

        // 用户按 Enter 前已经确认完全松手，此时先冻结当前位置，再准备安全回放
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) {
            restore_runtime();
            return false;
        }

        *trajectory = smooth_friction_trajectory(*trajectory);
        if(!friction_trajectory_inside_safe_replay_range(*trajectory)) {
            restore_runtime();
            return false;
        }

        const double playback_rate = friction_replay_rate(*trajectory);
        constexpr double kMinimumPracticalReplayRate = 0.01;
        if(!std::isfinite(playback_rate) || playback_rate < kMinimumPracticalReplayRate) {
            std::cout << "摩擦参数标定：FAIL（示教过快，请用更平滑的轨迹重试）\n";
            restore_runtime();
            return false;
        }

        const double estimated_pass_s = trajectory->sample_dt *
            static_cast<double>(trajectory->positions.size() - 1) / playback_rate;
        std::cout << "示教已结束并进入 RIGID_HOLD；机器人将先倒放再正放，单程约 "
            << std::fixed << std::setprecision(1) << estimated_pass_s << " s\n";
        if(!read_line("确认机器人已完全松手且回放环境安全后按 Enter 开始，输入 q 取消: ", line) || line == "q" || line == "Q") {
            std::cout << "已取消\n";
            restore_runtime();
            return false;
        }

        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING)) {
            restore_runtime();
            return false;
        }

        std::vector<AdmittanceFrictionSample> reverse_samples;
        std::vector<AdmittanceFrictionSample> forward_samples;
        std::vector<AdmittanceFrictionSample> full_id_reverse_samples;
        std::vector<AdmittanceFrictionSample> full_id_forward_samples;
        reverse_samples.reserve(trajectory->positions.size() * 2);
        forward_samples.reserve(trajectory->positions.size() * 2);
        const bool compare_full_id = original_admittance.observer.mode == AdmittanceObserverMode::MOMENTUM;

        std::cout << "回放 1/2：倒放...\n";
        if(!play_friction_calibration_pass(
            *trajectory,
            true,
            playback_rate,
            original_admittance,
            reverse_samples,
            compare_full_id ? &full_id_reverse_samples : nullptr)) {
            std::cout << "摩擦参数标定：FAIL（倒放中止）\n";
            restore_runtime();
            return false;
        }
        std::cout << "回放 2/2：正放...\n";
        if(!play_friction_calibration_pass(
            *trajectory,
            false,
            playback_rate,
            original_admittance,
            forward_samples,
            compare_full_id ? &full_id_forward_samples : nullptr)) {
            std::cout << "摩擦参数标定：FAIL（正放中止）\n";
            restore_runtime();
            return false;
        }

        // 两段回放结束后立即离开 tracking mode
        // 后续拟合与交叉验证可能超过 cmd_timeout_s，不能继续保留最后一条 external command
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) {
            restore_runtime();
            return false;
        }
        std::cout << "回放完成，已进入 RIGID_HOLD；开始摩擦拟合与交叉验证\n";

        AdmittanceFrictionCalibrationCfg calibration_cfg;
        calibration_cfg.joints_count = cfg_.joint_names.size();
        calibration_cfg.min_fit_velocity = 0.05;
        calibration_cfg.max_fit_acceleration = 1.5;
        calibration_cfg.min_speed_span = 0.03;
        calibration_cfg.min_samples_per_direction = 30;
        calibration_cfg.cross_validation_max_rms_ratio = 0.8;

        const auto result = calibrate_admittance_friction_cross_validated(
            reverse_samples, forward_samples, calibration_cfg);
        if(!result) {
            std::cout << "摩擦参数标定：FAIL（拟合失败）\n";
            restore_runtime();
            return false;
        }
        last_friction_calibration_result_ = result.value();

        if(compare_full_id) {
            const auto full_id_result = calibrate_admittance_friction_cross_validated(
                full_id_reverse_samples, full_id_forward_samples, calibration_cfg);
            if(full_id_result) last_full_id_friction_calibration_result_ = full_id_result.value();
            else last_full_id_friction_calibration_result_.reset();
        }
        else {
            last_full_id_friction_calibration_result_ = result.value();
        }

        const std::size_t n = cfg_.joint_names.size();
        bool all_valid = result->observable.size() == n && result->validation_pass.size() == n;
        for(std::size_t i = 0; i < n && all_valid; ++i) {
            all_valid = result->observable[i] != 0 && result->validation_pass[i] != 0;
        }
        if(!all_valid) {
            std::cout << "摩擦参数标定：FAIL（有轴未通过；详情见“状态与诊断”后重新示教）\n";
            restore_runtime();
            return false;
        }

        auto candidate = original_admittance;
        candidate.calibration.friction.enabled = true;
        candidate.calibration.friction.velocity_transition = 0.03;
        candidate.calibration.friction.positive_coulomb = result->positive_coulomb;
        candidate.calibration.friction.positive_viscous = result->positive_viscous;
        candidate.calibration.friction.negative_coulomb = result->negative_coulomb;
        candidate.calibration.friction.negative_viscous = result->negative_viscous;
        if(!apply_runtime_admittance_cfg(candidate)) {
            restore_runtime();
            return false;
        }
        restore_runtime();
        std::cout << "摩擦参数标定：PASS\n";
        if(print_yaml) {
            std::cout << "\n当前可写回 core.yaml 参数\n";
            print_admittance_persistence_block();
        }
        return true;
    }

    bool run_admittance_static_calibration(bool print_yaml = true) {
        if(!ensure_admittance_tuning_active()) return false;
        constexpr int kPoseCount = 8;
        constexpr double kStaticHoldS = 0.30;
        constexpr double kStaticTimeoutS = 5.0;
        constexpr double kStaticMaxVel = 0.03;
        constexpr double kStaticMaxAcc = 1.5;
        constexpr double kSampleS = 1.0;

        JointImpedanceMode original_mode;
        AdmittanceCapabilityCfg original_admittance;
        JointVector original_gravity_scale;
        bool original_suspended{ false };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            original_mode = robot_.get_impedance_mode();
            original_admittance = cfg_.capability.admittance;
            original_gravity_scale = dynamics_.get_gravity_scale();
            original_suspended = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }

        const auto restore_runtime = [this, original_suspended, original_mode]() {
            set_tuning_impedance_mode(original_mode);
            std::lock_guard<std::mutex> lock(mutex_);
            robot_.set_admittance_suspended(original_suspended);
            last_output_.reset();
            };
        const auto restore_gravity_scale = [this, &original_gravity_scale]() {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto restore = dynamics_.set_gravity_scale(original_gravity_scale);
            if(restore) cfg_.dynamics.gravity_scale = original_gravity_scale;
            };

        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            restore_runtime();
            return false;
        }

        std::cout << "静态残差标定：依次摆 8 个代表姿态；每次完全松手后按 Enter\n";
        std::vector<AdmittanceStaticPoseSamples> poses;
        poses.reserve(kPoseCount);
        for(int pose_index = 0; pose_index < kPoseCount; ++pose_index) {
            std::string line;
            std::ostringstream prompt;
            prompt << "姿态 " << (pose_index + 1) << "/" << kPoseCount
                << "，摆好并松手后按 Enter，输入 q 取消: ";
            if(!read_line(prompt.str(), line) || line == "q" || line == "Q") {
                std::cout << "已取消\n";
                restore_gravity_scale();
                restore_runtime();
                return false;
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD) ||
                !wait_for_static_tuning_pose(kStaticHoldS, kStaticTimeoutS, kStaticMaxVel, kStaticMaxAcc)) {
                std::cout << "静态残差标定：FAIL（未稳定，请重新执行标定）\n";
                restore_gravity_scale();
                restore_runtime();
                return false;
            }
            auto samples = collect_static_pose_samples(0.0, kSampleS);
            if(!samples) {
                std::cout << "静态残差标定：FAIL（采样中止）\n";
                restore_gravity_scale();
                restore_runtime();
                return false;
            }
            poses.push_back(std::move(*samples));
            if(pose_index + 1 < kPoseCount && !set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
                restore_gravity_scale();
                restore_runtime();
                return false;
            }
        }

        AdmittanceStaticCalibrationCfg calibration_cfg;
        calibration_cfg.joints_count = cfg_.joint_names.size();
        calibration_cfg.fallback_gravity_scale = original_gravity_scale;
        calibration_cfg.gravity_observability_span = 0.25;
        calibration_cfg.threshold_margin = 1.2;
        calibration_cfg.threshold_max_margin = 1.05;
        const auto result = calibrate_admittance_static(poses, calibration_cfg);
        if(!result) {
            std::cout << "静态残差标定：FAIL（计算失败）\n";
            restore_gravity_scale();
            restore_runtime();
            return false;
        }

        bool scale_applied = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto scale_result = dynamics_.set_gravity_scale(result->gravity_scale);
            if(scale_result) {
                cfg_.dynamics.gravity_scale = result->gravity_scale;
                scale_applied = true;
            }
        }
        if(!scale_applied) {
            std::cout << "静态残差标定：FAIL（gravity_scale 应用失败）\n";
            restore_gravity_scale();
            restore_runtime();
            return false;
        }

        auto calibrated_admittance = original_admittance;
        calibrated_admittance.calibration.torque_bias = result->torque_bias;
        calibrated_admittance.calibration.torque_threshold = result->torque_threshold;
        calibrated_admittance.calibration.friction.enabled = false;
        if(!apply_runtime_admittance_cfg(calibrated_admittance)) {
            restore_gravity_scale();
            apply_runtime_admittance_cfg(original_admittance);
            restore_runtime();
            return false;
        }
        last_static_calibration_result_ = result.value();
        restore_runtime();
        std::cout << "静态残差标定：PASS\n";
        if(print_yaml) {
            std::cout << "\n当前可写回 core.yaml 参数\n";
            print_admittance_persistence_block();
        }
        return true;
    }

    void print_static_validation_diagnostics(const AdmittanceStaticValidationResult& result) const {
        std::cout << "\n静态残差验证逐轴诊断\n";
        std::cout << "  判据: P99 为硬判据，MAX 单帧超限仅 WARN\n";
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            const bool p99_pass = i < result.pass.size() && result.pass[i] != 0;
            const bool max_warn = i < result.residual_max.size() &&
                i < result.guarded_max_limit.size() &&
                result.residual_max[i] > result.guarded_max_limit[i];
            const char* status = !p99_pass ? "P99_HARD_FAIL" : (max_warn ? "MAX_WARN" : "PASS");
            const double p99_util_pct = i < result.threshold_utilization.size() &&
                std::isfinite(result.threshold_utilization[i]) ?
                100.0 * result.threshold_utilization[i] : std::numeric_limits<double>::infinity();
            const double max_util_pct = i < result.guarded_max_utilization.size() &&
                std::isfinite(result.guarded_max_utilization[i]) ?
                100.0 * result.guarded_max_utilization[i] : std::numeric_limits<double>::infinity();

            std::cout << "  " << cfg_.joint_names[i]
                << " RMS=" << std::fixed << std::setprecision(6) << result.residual_rms[i]
                << " P99/阈值=" << result.residual_p99[i] << "/" << cfg_.capability.admittance.calibration.torque_threshold[i]
                << " (" << p99_util_pct << "%)"
                << " MAX/保护上限=" << result.residual_max[i] << "/" << result.guarded_max_limit[i]
                << " (" << max_util_pct << "%)"
                << " qstep=" << result.feedback_quantization_step[i]
                << " " << status << '\n';
        }
    }

    bool run_admittance_static_validation() {
        if(!ensure_admittance_tuning_active()) return false;
        constexpr int kPoseCount = 5;
        constexpr double kStaticHoldS = 0.30;
        constexpr double kStaticTimeoutS = 5.0;
        constexpr double kStaticMaxVel = 0.03;
        constexpr double kStaticMaxAcc = 1.5;
        constexpr double kSampleS = 1.0;

        JointImpedanceMode original_mode;
        AdmittanceCapabilityCfg original_admittance;
        JointVector gravity_scale;
        bool original_suspended{ false };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            original_mode = robot_.get_impedance_mode();
            original_admittance = cfg_.capability.admittance;
            gravity_scale = dynamics_.get_gravity_scale();
            original_suspended = robot_.is_admittance_suspended();
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        const auto restore_runtime = [this, original_suspended, original_mode]() {
            set_tuning_impedance_mode(original_mode);
            std::lock_guard<std::mutex> lock(mutex_);
            robot_.set_admittance_suspended(original_suspended);
            last_output_.reset();
            };

        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            restore_runtime();
            return false;
        }

        std::cout << "静态残差验证：再摆 5 个不同姿态；每次完全松手后按 Enter\n";
        std::vector<AdmittanceStaticPoseSamples> poses;
        poses.reserve(kPoseCount);
        for(int pose_index = 0; pose_index < kPoseCount; ++pose_index) {
            std::string line;
            std::ostringstream prompt;
            prompt << "验证 " << (pose_index + 1) << "/" << kPoseCount
                << "，摆好并松手后按 Enter，输入 q 取消: ";
            if(!read_line(prompt.str(), line) || line == "q" || line == "Q") {
                std::cout << "已取消\n";
                restore_runtime();
                return false;
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD) ||
                !wait_for_static_tuning_pose(kStaticHoldS, kStaticTimeoutS, kStaticMaxVel, kStaticMaxAcc)) {
                std::cout << "静态残差验证：FAIL（未稳定）\n";
                restore_runtime();
                return false;
            }
            auto samples = collect_static_pose_samples(0.0, kSampleS);
            if(!samples) {
                std::cout << "静态残差验证：FAIL（采样中止）\n";
                restore_runtime();
                return false;
            }
            poses.push_back(std::move(*samples));
            if(pose_index + 1 < kPoseCount && !set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
                restore_runtime();
                return false;
            }
        }

        AdmittanceStaticValidationCfg validation_cfg;
        validation_cfg.joints_count = cfg_.joint_names.size();
        validation_cfg.gravity_scale = gravity_scale;
        validation_cfg.torque_bias = original_admittance.calibration.torque_bias;
        validation_cfg.torque_threshold = original_admittance.calibration.torque_threshold;
        const auto result = evaluate_admittance_static_validation(poses, validation_cfg);
        restore_runtime();
        if(!result) {
            std::cout << "静态残差验证：FAIL（计算失败）\n";
            return false;
        }
        last_static_validation_result_ = result.value();
        print_static_validation_diagnostics(*result);
        const bool pass = std::all_of(result->pass.begin(), result->pass.end(), [](std::uint8_t value) {
            return value != 0;
            });
        const bool max_warn = std::any_of(
            result->guarded_max_utilization.begin(),
            result->guarded_max_utilization.end(),
            [](double value) { return value > 1.0; });
        if(pass) {
            std::cout << (max_warn ?
                "静态残差验证：PASS（存在 MAX 单帧 WARN，P99 硬判据通过）\n" :
                "静态残差验证：PASS\n");
        }
        else {
            std::cout << "静态残差验证：FAIL（至少一个关节 P99 超过 torque_threshold）\n";
        }
        return pass;
    }

    void print_admittance_capability_yaml() const {
        const auto& a = cfg_.capability.admittance;
        std::cout << "capability:\n";
        std::cout << "  admittance:\n";
        std::cout << "    enabled: " << (a.enabled ? "true" : "false") << '\n';
        std::cout << "    "; print_joint_yaml_bool_map("joint_enabled", cfg_.joint_names, a.joint_enabled);
        std::cout << "    observer:\n";
        std::cout << "      mode: " << observer_mode_name(a.observer.mode) << '\n';
        std::cout << "      "; print_joint_yaml_map("momentum_gain", cfg_.joint_names, a.observer.momentum_gain);
        std::cout << "    calibration:\n";
        std::cout << "      "; print_joint_yaml_map("torque_bias", cfg_.joint_names, a.calibration.torque_bias);
        std::cout << "      "; print_joint_yaml_map("torque_threshold", cfg_.joint_names, a.calibration.torque_threshold);
        print_friction_calibration_yaml(a.calibration.friction, "      ");
        std::cout << "    controller:\n";
        std::cout << "      "; print_joint_yaml_map("mass", cfg_.joint_names, a.controller.mass);
        std::cout << "      "; print_joint_yaml_map("damping", cfg_.joint_names, a.controller.damping);
        std::cout << "      "; print_joint_yaml_map("stiffness", cfg_.joint_names, a.controller.stiffness);
        std::cout << "      "; print_joint_yaml_map("max_delta_q", cfg_.joint_names, a.controller.max_delta_q);
        std::cout << "      "; print_joint_yaml_map("max_delta_q_dot", cfg_.joint_names, a.controller.max_delta_q_dot);
    }

    void print_admittance_persistence_block() const {
        std::cout << "model:\n";
        std::cout << "  "; print_joint_yaml_map("gravity_scale", cfg_.joint_names, cfg_.dynamics.gravity_scale);
        print_admittance_capability_yaml();
    }

    void run_admittance_parameter_calibration() {
        if(!ensure_admittance_tuning_active()) return;
        std::cout << "\n========== 导纳参数一次性标定 ==========\n";
        std::cout << "按顺序完成静态残差标定、静态残差验证和摩擦参数标定\n";
        std::string line;
        if(!read_line("按 Enter 开始，输入 q 取消: ", line) || line == "q" || line == "Q") return;

        last_static_calibration_result_.reset();
        last_static_validation_result_.reset();
        last_friction_calibration_result_.reset();
        last_full_id_friction_calibration_result_.reset();

        std::cout << "\n[1/3] 静态残差标定\n";
        if(!run_admittance_static_calibration(false)) {
            std::cout << "导纳参数标定：FAIL；修正提示后重新执行即可\n";
            return;
        }
        std::cout << "\n[2/3] 静态残差验证\n";
        if(!run_admittance_static_validation()) {
            std::cout << "导纳参数标定：FAIL；修正提示后重新执行即可\n";
            return;
        }
        std::cout << "\n[3/3] 摩擦参数标定\n";
        if(!run_admittance_friction_calibration(false)) {
            std::cout << "导纳参数标定：FAIL；修正提示后重新执行即可\n";
            return;
        }

        std::cout << "导纳参数标定：PASS，参数已应用到当前进程\n";
        std::cout << "\n可直接复制到 core.yaml 的当前标定参数\n";
        print_admittance_persistence_block();
    }

    YAML::Node model_calibration_task_params() const {
        YAML::Node params;
        if(!model_calibration_task_id_.empty()) params["task_id"] = model_calibration_task_id_;
        return params;
    }

    void print_model_calibration_status_terminal() const {
        const auto recorder = model_calibration_recorder_.status();
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        std::cout << "\n========== 重力模型校正状态 ==========\n";
        std::cout << "任务标识             : " << (model_calibration_task_id_.empty() ? "—" : model_calibration_task_id_) << '\n';
        std::cout << "任务阶段             : " << to_string(model_calibration_phase_.load()) << '\n';
        std::cout << "任务进度             : " << std::fixed << std::setprecision(1) << model_calibration_progress_ * 100.0
                  << "% (" << model_calibration_completed_units_ << '/' << model_calibration_total_units_ << ")\n";
        std::cout << "轨迹采样点           : " << (model_calibration_trajectory_ ? std::to_string(model_calibration_trajectory_->positions.size()) : "—") << '\n';
        std::cout << "有效静态样本         : " << model_calibration_valid_static_samples_ << '\n';
        std::cout << "原始帧 接收/写入/丢弃: " << recorder.accepted_frames << '/' << recorder.written_frames << '/' << recorder.dropped_frames << '\n';
        std::cout << "数据缺口             : " << (recorder.data_gap ? "是" : "否") << '\n';
        std::cout << "写入失败             : " << (recorder.write_failed ? "是" : "否") << '\n';
        std::cout << "候选已应用到当前进程 : " << (model_calibration_candidate_applied_ ? "是" : "否") << '\n';
        if(model_calibration_estimated_duration_s_ > 0.0) {
            std::cout << "预计自动阶段         : " << model_calibration_estimated_duration_s_ << " s\n";
        }
        if(!model_calibration_directory_.empty()) std::cout << "任务目录             : " << model_calibration_directory_ << '\n';
        if(!model_calibration_error_.empty()) std::cout << "失败原因             : " << model_calibration_error_ << '\n';
        if(model_calibration_result_) {
            std::cout << "静态验证             : " << (model_calibration_result_->static_pass ? "通过" : "未通过") << '\n';
            std::cout << "数值秩               : " << model_calibration_result_->numerical_rank << '\n';
            std::cout << "摩擦验证             : " << (model_calibration_friction_pass_ ? "通过" : "未通过") << '\n';
            const auto& current = model_calibration_result_->validation_original.rms;
            const auto& candidate = model_calibration_result_->validation_candidate.rms;
            if(current.size() == cfg_.joint_names.size() && candidate.size() == cfg_.joint_names.size()) {
                std::cout << "\n留出姿态 RMS (Nm)\n";
                std::cout << std::left << std::setw(16) << "关节" << std::setw(18) << "当前 URDF" << std::setw(18) << "候选重力" << '\n';
                for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                    std::cout << std::left << std::setw(16) << cfg_.joint_names[i]
                              << std::setw(18) << current[i] << std::setw(18) << candidate[i] << '\n';
                }
            }
        }
    }

    void configure_model_calibration_options_terminal() {
        while(true) {
            std::cout << "\n------------ 重力模型校正参数 ------------\n";
            std::cout << " 1. 姿态数量上限              : " << model_calibration_options_.pose_budget << '\n';
            std::cout << " 2. 留出验证比例              : " << model_calibration_options_.validation_fraction << '\n';
            std::cout << " 3. 正则化强度                : " << model_calibration_options_.regularization << '\n';
            std::cout << " 4. SVD 相对阈值              : " << model_calibration_options_.svd_relative_threshold << '\n';
            std::cout << " 5. 最小信息量                : " << model_calibration_options_.minimum_information_score << '\n';

            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择需要修改的参数: ");
            if(!selection || *selection == 0) return;
            if(*selection == 1) {
                const auto value = read_int("姿态数量上限 [5, 16]: ");
                if(value && *value >= 5 && *value <= 16) model_calibration_options_.pose_budget = static_cast<std::size_t>(*value);
                else std::cout << "输入无效\n";
                continue;
            }
            const auto value = read_double("输入新值: ");
            if(!value) { std::cout << "输入无效\n"; continue; }
            switch(*selection) {
                case 2: if(*value >= 0.15 && *value <= 0.45) model_calibration_options_.validation_fraction = *value; else std::cout << "范围应为 [0.15, 0.45]\n"; break;
                case 3: if(*value >= 1.0e-8 && *value <= 10.0) model_calibration_options_.regularization = *value; else std::cout << "范围应为 [1e-8, 10]\n"; break;
                case 4: if(*value >= 1.0e-8 && *value <= 0.2) model_calibration_options_.svd_relative_threshold = *value; else std::cout << "范围应为 [1e-8, 0.2]\n"; break;
                case 5: if(*value >= 1.0e-8 && *value <= 1.0e3) model_calibration_options_.minimum_information_score = *value; else std::cout << "范围应为 [1e-8, 1000]\n"; break;

                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    void print_model_calibration_offline_commands() const {
        if(model_calibration_directory_.empty()) {
            std::cout << "当前没有任务目录\n";
            return;
        }
        std::cout << "\n任务目录: " << model_calibration_directory_ << '\n';
        std::cout << "断开硬件后可在仓库根目录继续处理同一份记录\n";
        std::cout << "  python3 tools/model_calibration_cli.py inspect --directory \"" << model_calibration_directory_ << "\"\n";
        std::cout << "  python3 tools/model_calibration_cli.py recompute --config \"" << config_path_ << "\" --directory \"" << model_calibration_directory_ << "\"\n";
        std::cout << "  python3 tools/model_calibration_cli.py preview-save --config \"" << config_path_ << "\" --directory \"" << model_calibration_directory_ << "\"\n";
        std::cout << "  python3 tools/model_calibration_cli.py export-urdf --config \"" << config_path_ << "\" --directory \"" << model_calibration_directory_ << "\"\n";
        std::cout << "  python3 tools/model_calibration_cli.py save --config \"" << config_path_ << "\" --directory \"" << model_calibration_directory_ << "\" --confirm\n";
        std::cout << "  python3 tools/model_calibration_cli.py restore-config --config \"" << config_path_ << "\" --directory \"" << model_calibration_directory_ << "\" --confirm\n";
    }

    void run_model_calibration_menu() {
        while(true) {
            std::cout << "\n------------ 重力模型校正 ------------\n";
            std::cout << "一次拖动示教后，沿同一路径自动完成双向静态采样、重力校正和摩擦验证\n";
            std::cout << " 1. 设置校正参数\n";
            std::cout << " 2. 开始拖动示教\n";
            std::cout << " 3. 结束示教并检查路径\n";
            std::cout << " 4. 确认松手并开始自动采集\n";
            std::cout << " 5. 查看任务状态\n";
            std::cout << " 6. 暂停自动任务\n";
            std::cout << " 7. 继续自动任务\n";
            std::cout << " 8. 取消任务并保持当前位置\n";
            std::cout << " 9. 应用候选重力到当前进程（仅 INACTIVE）\n";
            std::cout << "10. 恢复校正前运行配置（仅 INACTIVE）\n";
            std::cout << "11. 查看非 GUI 离线重算、保存、导出与恢复入口\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            try {
                switch(*selection) {
                    case 1:
                        configure_model_calibration_options_terminal();
                        break;
                    case 2: {
                        YAML::Node params;
                        params["pose_budget"] = model_calibration_options_.pose_budget;
                        params["validation_fraction"] = model_calibration_options_.validation_fraction;
                        params["max_com_offset_m"] = model_calibration_options_.max_com_offset_m;
                        params["regularization"] = model_calibration_options_.regularization;
                        params["svd_relative_threshold"] = model_calibration_options_.svd_relative_threshold;
                        params["minimum_information_score"] = model_calibration_options_.minimum_information_score;

                        machine_model_calibration_teach_begin(params);
                        std::cout << "已进入 COMPLIANT_DRAG，请沿安全范围拖动机械臂覆盖需要校正的姿态\n";
                        break;
                    }
                    case 3:
                        machine_model_calibration_teach_stop(model_calibration_task_params());
                        print_model_calibration_status_terminal();
                        std::cout << "路径已冻结，请检查机械臂、负载、线缆和回放区域\n";
                        break;
                    case 4: {
                        if(model_calibration_phase_.load() != ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION) {
                            std::cout << "当前任务尚未等待回放确认\n";
                            break;
                        }
                        std::string line;
                        if(!read_line("确认已经完全松手机械臂并允许自动回放，输入 YES 继续: ", line) || line != "YES") {
                            std::cout << "已取消本次启动，任务仍停留在回放确认阶段\n";
                            break;
                        }
                        auto params = model_calibration_task_params(); params["released"] = true;
                        machine_model_calibration_start(params);
                        std::cout << "自动采集任务已启动，可随时返回本菜单查看状态、暂停或取消\n";
                        break;
                    }
                    case 5:
                        print_model_calibration_status_terminal();
                        break;
                    case 6:
                        machine_model_calibration_pause(model_calibration_task_params());
                        std::cout << "已请求暂停，后台会进入当前位置保持\n";
                        break;
                    case 7: {
                        auto params = model_calibration_task_params(); params["confirmed"] = false;
                        try {
                            machine_model_calibration_resume(params);
                        }
                        catch(const std::exception& error) {
                            if(std::string(error.what()).find("explicit resume confirmation") == std::string::npos) throw;
                            std::string line;
                            if(!read_line("暂停期间机械臂位置发生变化，确认当前位置安全一致后输入 YES 继续: ", line) || line != "YES") {
                                std::cout << "保持暂停\n";
                                break;
                            }
                            params["confirmed"] = true;
                            machine_model_calibration_resume(params);
                        }
                        std::cout << "已继续任务\n";
                        break;
                    }
                    case 8:
                        machine_model_calibration_cancel_request(model_calibration_task_params());
                        std::cout << "已取消任务，已采集数据会保留在任务目录\n";
                        break;
                    case 9:
                        machine_model_calibration_apply(model_calibration_task_params());
                        std::cout << "候选重力已应用到当前进程，不会自动修改配置文件\n";
                        break;
                    case 10:
                        machine_model_calibration_restore(model_calibration_task_params());
                        std::cout << "已恢复校正前运行配置\n";
                        break;
                    case 11:
                        print_model_calibration_offline_commands();
                        break;
                    default:
                        std::cout << "未知菜单编号\n";
                        break;
                }
            }
            catch(const std::exception& error) {
                std::cout << "操作失败: " << error.what() << '\n';
            }
        }
    }

    void run_admittance_calibration_menu() {
        while(true) {
            std::cout << "\n------------ 导纳参数标定 ------------\n";
            std::cout << " 1. 重力模型校正\n";
            std::cout << " 2. 一次性导纳参数标定\n";
            std::cout << " 3. 静态残差标定\n";
            std::cout << " 4. 静态残差验证\n";
            std::cout << " 5. 摩擦参数标定\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: run_model_calibration_menu(); break;
                case 2: run_admittance_parameter_calibration(); break;
                case 3: run_admittance_static_calibration(); break;
                case 4: run_admittance_static_validation(); break;
                case 5: run_admittance_friction_calibration(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    bool select_admittance_joints(std::vector<std::size_t>& indices) const {
        std::ostringstream joint_prompt; joint_prompt << "关节 1~" << cfg_.joint_names.size() << "，输入 0 表示全部: ";
        const auto joint = read_int(joint_prompt.str());
        if(!joint || *joint < 0 || static_cast<std::size_t>(*joint) > cfg_.joint_names.size()) {
            std::cout << "关节编号无效\n";
            return false;
        }
        indices.clear();
        if(*joint == 0) {
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) indices.push_back(i);
        }
        else {
            indices.push_back(static_cast<std::size_t>(*joint - 1));
        }
        return true;
    }

    void set_admittance_controller_vector(
        const char* label,
        const char* prompt,
        JointVector AdmittanceControllerCfg::* field,
        bool allow_zero)
    {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto value = read_double(prompt);
        if(!value || !std::isfinite(*value) || (allow_zero ? *value < 0.0 : *value <= 0.0)) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        auto& values = candidate.controller.*field;
        for(const auto i : indices) values[i] = *value;
        if(apply_runtime_admittance_cfg(candidate)) {
            std::cout << label << " 已应用到当前进程\n";
        }
    }

    void set_admittance_momentum_gain() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto value = read_double("MOMENTUM observer gain rad/s: ");
        if(!value || !std::isfinite(*value) || *value <= 0.0) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) candidate.observer.momentum_gain[i] = *value;
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "momentum_gain 已应用到当前进程\n";
    }

    static const char* observer_mode_name(AdmittanceObserverMode mode) {
        return mode == AdmittanceObserverMode::MOMENTUM ? "MOMENTUM" : "FULL_ID";
    }

    void print_admittance_mdk_guide() const {
        const auto& controller = cfg_.capability.admittance.controller;
        constexpr double kPi = 3.14159265358979323846;

        std::cout << "\n========== M / D / K 参数关系 ==========" << '\n';
        std::cout << "导纳方程: M*Δq_ddot + D*Δq_dot + K*Δq = tau_ext_hat" << '\n';
        std::cout << "虚拟机械阻抗: Z(s)=M*s^2 + D*s + K，导纳: G(s)=1/Z(s)" << '\n';
        std::cout << "自然频率: ωn=sqrt(K/M), fn=ωn/(2π)" << '\n';
        std::cout << "临界阻尼: Dcrit=2*sqrt(M*K), 阻尼比: ζ=D/Dcrit" << '\n';
        std::cout << "恒定外力稳态退让: Δq_ss=tau_ext_hat/K" << '\n';
        std::cout << "单位外力初始加速度: Δq_ddot(0)=tau_ext_hat/M" << "\n\n";
        std::cout << "调参原则" << '\n';
        std::cout << "  1 K 先定退让距离: K 越小，同样外力退让越远，回中更软" << '\n';
        std::cout << "  2 M 再定启动惯性: M 越小启动越快，但更敏感于外力估计波动" << '\n';
        std::cout << "  3 D 最后定阻尼比: 用 D=2*ζ*sqrt(M*K) 从目标 ζ 反算" << '\n';
        std::cout << "阻尼比参考" << '\n';
        std::cout << "  ζ<1  欠阻尼，更灵活/Q 弹但会有超调；0.6~0.8 可作为柔顺调参起点" << '\n';
        std::cout << "  ζ≈1 临界阻尼，回中快且理论上无超调" << '\n';
        std::cout << "  ζ>1 过阻尼，更稳但更黏、回中更拖" << '\n';
        std::cout << "组合规律" << '\n';
        std::cout << "  同倍率缩放 M/D/K 不改变 ωn 和 ζ；整体缩小会让同样外力产生更大退让" << '\n';
        std::cout << "  固定 K 降低 M 并按目标 ζ 重算 D，可加快启动和回中而不改变稳态退让" << '\n';
        std::cout << "  固定 M/K 只降低 D，会更 Q 弹但也更容易振荡" << "\n\n";
        std::cout << "当前逐轴指标" << '\n';
        std::cout << "  joint      M       D       K      ζ      ωn(rad/s)  fn(Hz)   Dcrit   1Nm退让(rad)" << '\n';
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            const auto metrics = compute_admittance_damping_metrics(
                controller.mass[i], controller.damping[i], controller.stiffness[i]);
            if(!metrics) {
                std::cout << "  " << cfg_.joint_names[i] << "  当前 M/D/K 无法计算二阶指标" << '\n';
                continue;
            }
            const double fn = metrics->natural_frequency / (2.0 * kPi);
            const double retreat_per_nm = 1.0 / controller.stiffness[i];
            std::cout << "  " << std::setw(6) << std::left << cfg_.joint_names[i] << std::right
                << std::setw(8) << std::fixed << std::setprecision(3) << controller.mass[i]
                << std::setw(8) << controller.damping[i]
                << std::setw(8) << controller.stiffness[i]
                << std::setw(7) << metrics->damping_ratio
                << std::setw(12) << metrics->natural_frequency
                << std::setw(9) << fn
                << std::setw(9) << metrics->critical_damping
                << std::setw(14) << retreat_per_nm << '\n';
        }
    }

    void print_admittance_controller_yaml() const {
        std::cout << "\n可直接复制并替换 core.yaml 中的 capability 区块\n";
        print_admittance_capability_yaml();
    }

    void run_admittance_controller_menu() {
        if(!ensure_admittance_tuning_active()) return;
        if(!cfg_.capability.admittance.enabled) {
            std::cout << "请先在 core.yaml 开启 admittance.enabled\n";
            return;
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) return;

        while(true) {
            std::cout << "\n------------ M / D / K 设置 ------------\n";
            std::cout << "公式: M*Δq_ddot + D*Δq_dot + K*Δq=tau_ext，ζ=D/(2*sqrt(M*K))，ωn=sqrt(K/M)\n";
            std::cout << " 1. 虚拟质量 M\n";
            std::cout << " 2. 虚拟阻尼 D\n";
            std::cout << " 3. 虚拟刚度 K\n";
            std::cout << " 4. 最大位置修正 max_delta_q\n";
            std::cout << " 5. 最大速度修正 max_delta_q_dot\n";
            std::cout << " 6. MOMENTUM momentum_gain\n";
            std::cout << " 7. 参数关系与调参说明\n";
            std::cout << " 8. 打印当前可写回参数\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1:
                    set_admittance_controller_vector(
                        "M", "虚拟质量 M (>0): ", &AdmittanceControllerCfg::mass, false);
                    break;
                case 2:
                    set_admittance_controller_vector(
                        "D", "虚拟阻尼 D (>0): ", &AdmittanceControllerCfg::damping, false);
                    break;
                case 3:
                    set_admittance_controller_vector(
                        "K", "虚拟刚度 K (>0): ", &AdmittanceControllerCfg::stiffness, false);
                    break;
                case 4:
                    set_admittance_controller_vector(
                        "max_delta_q", "最大位置修正 rad (>0): ", &AdmittanceControllerCfg::max_delta_q, false);
                    break;
                case 5:
                    set_admittance_controller_vector(
                        "max_delta_q_dot", "最大速度修正 rad/s (>0): ", &AdmittanceControllerCfg::max_delta_q_dot, false);
                    break;
                case 6: set_admittance_momentum_gain(); break;
                case 7: print_admittance_mdk_guide(); break;
                case 8: print_admittance_controller_yaml(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    void print_admittance_status_summary() const {
        const auto& a = cfg_.capability.admittance;
        std::cout << "\n导纳状态\n";
        std::cout << "  Observer     : " << observer_mode_name(a.observer.mode) << '\n';
        std::cout << "  摩擦 residual: " << (a.calibration.friction.enabled ? "ON" : "OFF") << '\n';
        print_vector("mass_M", a.controller.mass);
        print_vector("damping_D", a.controller.damping);
        print_vector("stiffness_K", a.controller.stiffness);
        print_vector("max_delta_q", a.controller.max_delta_q);
        print_vector("max_delta_q_dot", a.controller.max_delta_q_dot);
        if(last_static_validation_result_) {
            const bool pass = std::all_of(
                last_static_validation_result_->pass.begin(),
                last_static_validation_result_->pass.end(),
                [](std::uint8_t value) { return value != 0; });
            const bool max_warn = std::any_of(
                last_static_validation_result_->guarded_max_utilization.begin(),
                last_static_validation_result_->guarded_max_utilization.end(),
                [](double value) { return value > 1.0; });
            std::cout << "  本次静态验证   : " << (pass ? (max_warn ? "PASS + MAX_WARN" : "PASS") : "FAIL") << '\n';
        }
        if(last_friction_calibration_result_) {
            const bool pass = std::all_of(
                last_friction_calibration_result_->validation_pass.begin(),
                last_friction_calibration_result_->validation_pass.end(),
                [](std::uint8_t value) { return value != 0; });
            std::cout << "  本次摩擦交叉验证: " << (pass ? "PASS" : "FAIL") << '\n';
        }
    }

    void print_admittance_calibration_details() const {
        const auto& a = cfg_.capability.admittance;
        std::cout << "\n========== 标定详情 ==========\n";
        print_vector("gravity_scale", cfg_.dynamics.gravity_scale);
        print_vector("torque_bias", a.calibration.torque_bias);
        print_vector("torque_threshold", a.calibration.torque_threshold);
        std::cout << "Observer: " << observer_mode_name(a.observer.mode) << '\n';
        print_vector("momentum_gain", a.observer.momentum_gain);
        if(last_static_validation_result_) {
            print_static_validation_diagnostics(*last_static_validation_result_);
        }
        if(a.calibration.friction.enabled) {
            print_vector("friction_C+", a.calibration.friction.positive_coulomb);
            print_vector("friction_B+", a.calibration.friction.positive_viscous);
            print_vector("friction_C-", a.calibration.friction.negative_coulomb);
            print_vector("friction_B-", a.calibration.friction.negative_viscous);
        }
        if(last_friction_calibration_result_) {
            std::cout << "\n摩擦交叉验证（当前 Observer）\n";
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                const bool observable = i < last_friction_calibration_result_->observable.size() &&
                    last_friction_calibration_result_->observable[i] != 0;
                std::cout << "  " << cfg_.joint_names[i];
                if(!observable) {
                    std::cout << " UNOBSERVABLE FAIL\n";
                    continue;
                }
                std::cout << " RMS " << std::fixed << std::setprecision(3)
                    << last_friction_calibration_result_->cross_residual_rms_before[i]
                    << " -> " << last_friction_calibration_result_->cross_residual_rms_after[i]
                    << " P99=" << last_friction_calibration_result_->cross_residual_p99_after[i]
                    << (last_friction_calibration_result_->validation_pass[i] ? " PASS" : " FAIL") << '\n';
            }
        }
        if(a.observer.mode == AdmittanceObserverMode::MOMENTUM &&
            last_friction_calibration_result_ && last_full_id_friction_calibration_result_) {
            std::cout << "\nObserver A/B：MOMENTUM vs FULL_ID（交叉验证 P99）\n";
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                std::cout << "  " << cfg_.joint_names[i] << ' '
                    << last_friction_calibration_result_->cross_residual_p99_after[i] << " / "
                    << last_full_id_friction_calibration_result_->cross_residual_p99_after[i] << '\n';
            }
        }
        std::cout << "\ncore.yaml 写回参数\n";
        print_admittance_persistence_block();
    }

    void run_admittance_diagnostics_menu() {
        while(true) {
            std::cout << "\n------------ 状态与诊断 ------------\n";
            std::cout << " 1. 状态摘要\n";
            std::cout << " 2. 实时观测\n";
            std::cout << " 3. 标定与 Observer 详情\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: print_admittance_status_summary(); break;
                case 2: observe_admittance_realtime(); break;
                case 3: print_admittance_calibration_details(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    void observe_admittance_realtime() {
        std::atomic<bool> stop{ false };
        std::thread display([&]() {
            while(!stop.load()) {
                std::optional<RobotCycleOutput> output;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    output = last_output_;
                }
                if(output && output->admittance_active && output->tau_ext_hat.size() == cfg_.joint_names.size()) {
                    std::ostringstream flags;
                    bool first_flag = true;
                    for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                        std::string joint_flags;
                        if(i < output->torque_threshold_active.size() && output->torque_threshold_active[i]) joint_flags += "TH|";
                        if(i < output->delta_q_limited.size() && output->delta_q_limited[i]) joint_flags += "DQ|";
                        if(i < output->delta_q_dot_limited.size() && output->delta_q_dot_limited[i]) joint_flags += "DQV|";
                        if(i < output->safety_position_margin_active.size() && output->safety_position_margin_active[i]) joint_flags += "SP|";
                        if(i < output->safety_velocity_margin_active.size() && output->safety_velocity_margin_active[i]) joint_flags += "SV|";
                        if(!joint_flags.empty()) {
                            joint_flags.pop_back();
                            if(!first_flag) flags << ' ';
                            flags << cfg_.joint_names[i] << ':' << joint_flags;
                            first_flag = false;
                        }
                    }
                    auto print_inline = [](const char* label, const JointVector& values) {
                        std::cout << label;
                        for(double v : values) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    };
                    std::cout << '\n';
                    if(output->residual_raw.size() == cfg_.joint_names.size()) print_inline("raw ", output->residual_raw);
                    if(output->bias_compensated.size() == cfg_.joint_names.size()) print_inline("| bias ", output->bias_compensated);
                    if(output->friction_residual_hat.size() == cfg_.joint_names.size()) print_inline("| fric ", output->friction_residual_hat);
                    if(output->friction_compensated.size() == cfg_.joint_names.size()) print_inline("| ext_pre_db ", output->friction_compensated);
                    print_inline("| tau_ext ", output->tau_ext_hat);
                    if(output->full_id_residual_raw.size() == cfg_.joint_names.size()) print_inline("| full_id ", output->full_id_residual_raw);
                    print_inline("| dq ", output->delta_q);
                    print_inline("| dqdot ", output->delta_q_dot);
                    if(!first_flag) std::cout << "| " << flags.str();
                    std::cout << std::flush;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        });
        std::string line;
        read_line("\n实时观测中（raw→bias→fric→tau_ext→Δq，TH=deadband, DQ/DQV=导纳限幅, SP/SV=Safety）；按 Enter 停止: ", line);
        stop.store(true);
        display.join();
        std::cout << '\n';
    }

    void show_config_summary() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "ctrl_frequency_hz       : " << cfg_.runtime.ctrl_frequency_hz << '\n';
        std::cout << "joint_acc_filter_alpha  : " << cfg_.runtime.joint_acc_filter_alpha << '\n';
        std::cout << "write_enabled           : " << std::boolalpha << cfg_.runtime.write_enabled << '\n';
        std::cout << "model_feedforward_mode  : " << to_string(robot_.get_model_feedforward_mode()) << '\n';
        std::cout << "tracking_mode           : " << to_string(cfg_.runtime.tracking_impedance_mode) << '\n';
        std::cout << "admittance_enabled      : " << std::boolalpha << cfg_.capability.admittance.enabled << '\n';
        if(cfg_.capability.admittance.enabled || !cfg_.capability.admittance.joint_enabled.empty()) {
            const auto& a = cfg_.capability.admittance;
            std::cout << "admittance_observer     : " << observer_mode_name(a.observer.mode) << '\n';
            print_vector("momentum_gain", a.observer.momentum_gain);
            print_vector("admittance_M", a.controller.mass);
            print_vector("admittance_D", a.controller.damping);
            print_vector("admittance_K", a.controller.stiffness);
            print_vector("max_delta_q", a.controller.max_delta_q);
            print_vector("max_delta_q_dot", a.controller.max_delta_q_dot);
            print_vector("admittance_torque_bias", a.calibration.torque_bias);
            print_vector("admittance_threshold", a.calibration.torque_threshold);
            std::cout << "friction_compensation   : " << (a.calibration.friction.enabled ? "ENABLED" : "DISABLED") << '\n';
            if(a.calibration.friction.enabled) {
                std::cout << "friction_vel_transition: " << a.calibration.friction.velocity_transition << '\n';
                print_vector("friction_C_pos", a.calibration.friction.positive_coulomb);
                print_vector("friction_B_pos", a.calibration.friction.positive_viscous);
                print_vector("friction_C_neg", a.calibration.friction.negative_coulomb);
                print_vector("friction_B_neg", a.calibration.friction.negative_viscous);
            }
        }
        std::cout << "park_before_disable     : " << std::boolalpha << cfg_.shutdown.park_before_disable << '\n';
        print_vector("park_pos", cfg_.shutdown.park_pos);
        std::cout << "park_speed_scale        : " << cfg_.shutdown.speed_scale << '\n';
        std::cout << "park_position_tolerance : " << cfg_.shutdown.position_tolerance << '\n';
        std::cout << "park_velocity_tolerance : " << cfg_.shutdown.velocity_tolerance << '\n';
        std::cout << "park_settle_time_s      : " << cfg_.shutdown.settle_time_s << '\n';
        std::cout << "park_relaxed_ratio      : " << cfg_.shutdown.relaxed_tolerance_ratio << '\n';
        std::cout << "park_timeout_s          : " << cfg_.shutdown.timeout_s << '\n';
        std::cout << "cmd_timeout_s           : " << cfg_.safety.cmd_timeout_s << '\n';
        std::cout << "state_timeout_s         : " << cfg_.safety.state_timeout_s << '\n';
        std::cout << "max_dt_s                : " << cfg_.safety.max_dt_s << '\n';
        std::cout << "hardware_plugin         : " << hardware_plugin_ << '\n';
        std::cout << "hardware_config         : " << hardware_config_ << '\n';
        std::cout << "hardware_bus            : " << connection_summary_.bus << (hardware_overrides_.bus ? " (override)" : "") << '\n';
        std::cout << "hardware_serial_port    : " << connection_summary_.serial_port << (hardware_overrides_.serial_port ? " (override)" : "") << '\n';
        std::cout << "hardware_baudrate       : " << connection_summary_.baudrate << (hardware_overrides_.baudrate ? " (override)" : "") << '\n';
        std::cout << "urdf_path               : " << cfg_.dynamics.urdf_path << '\n';
        std::cout << "base_frame              : " << cfg_.dynamics.base_frame << '\n';
        std::cout << "tool_frame              : " << cfg_.dynamics.tool_frame << '\n';
        print_vector("gravity_scale", dynamics_.get_gravity_scale());
        print_vector("min_pos", cfg_.safety.limits.min_pos);
        print_vector("max_pos", cfg_.safety.limits.max_pos);
        print_vector("max_vel", cfg_.safety.limits.max_vel);
        print_vector("max_acc", cfg_.safety.limits.max_acc);
        print_vector("max_effort", cfg_.safety.limits.max_effort);
    }


    void show_frame_state() {
        std::string frame_name;
        if(!read_line("输入 Frame 名称: ", frame_name) || frame_name.empty()) {
            std::cout << "Frame 名称无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto pose = dynamics_.get_frame_pose(frame_name);
        if(!pose) {
            std::cout << "get_frame_pose() 失败: " << to_string(pose.error()) << '\n';
            return;
        }
        const auto jacobian = dynamics_.get_frame_jacobian(frame_name);
        if(!jacobian) {
            std::cout << "get_frame_jacobian() 失败: " << to_string(jacobian.error()) << '\n';
            return;
        }
        print_pose(frame_name + " pose", pose.value());
        print_matrix(frame_name + " Jacobian", jacobian.value());
    }

    bool safe_exit() {
        park_and_deactivate();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() == RobotState::FAULT) {
                const auto forced = robot_.force_deactivate();
                if(!forced) {
                    std::cout << "FAULT 安全退出失能失败：\n";
                    print_fault(forced.error());
                    return false;
                }
            }
            if(robot_.get_state() == RobotState::ACTIVE) {
                std::cout << "安全退出取消，机械臂仍处于 ACTIVE\n";
                return false;
            }
            quit_.store(true);
        }
        std::cout << "终端退出\n";
        return true;
    }

    void clear_command_sources() {
        stream_ = StreamState{};
    }

private:
    RobotCfg cfg_;
    std::string config_path_;
    std::string hardware_plugin_;
    std::string hardware_config_;
    HardwareConfigOverrides hardware_overrides_;
    HardwareConnectionSummary connection_summary_;
    std::string robot_profile_;
    Dynamics dynamics_;
    std::vector<GravityLinkParameterInfo> model_calibration_links_cache_;
    HardwareLoader hardware_loader_;
    Robot robot_;
    HardwareCapabilities actuator_info_;

    mutable std::mutex mutex_;
    std::condition_variable cycle_cv_;
    std::thread worker_;
    std::atomic<bool> worker_running_{ false };
    std::atomic<bool> quit_{ false };

    StreamState stream_;
    std::optional<RobotCycleOutput> last_output_;
    std::optional<AdmittanceStaticCalibrationResult> last_static_calibration_result_;
    std::optional<AdmittanceStaticValidationResult> last_static_validation_result_;
    std::optional<AdmittanceFrictionCalibrationResult> last_friction_calibration_result_;
    std::optional<AdmittanceFrictionCalibrationResult> last_full_id_friction_calibration_result_;
    std::uint64_t cycle_counter_{ 0 };
    bool background_fault_reported_{ false };
    std::optional<DynamicsErr> last_dynamics_err_;
    bool machine_mode_{ false };
    std::string park_last_error_;
    std::uint64_t feedback_time_ns_{ 0 };
    std::string machine_calibration_kind_{ "none" };
    std::string machine_calibration_phase_{ "idle" };
    std::size_t machine_calibration_expected_{ 0 };
    std::vector<AdmittanceStaticPoseSamples> machine_calibration_poses_;
    bool machine_calibration_context_valid_{ false };
    JointImpedanceMode machine_calibration_original_mode_{ JointImpedanceMode::RIGID_HOLD };
    AdmittanceCapabilityCfg machine_calibration_original_admittance_;
    JointVector machine_calibration_original_gravity_;
    bool machine_calibration_original_suspended_{ false };
    std::atomic<bool> machine_friction_stop_{ false };
    std::thread machine_friction_thread_;
    std::optional<FrictionCalibrationTrajectory> machine_friction_trajectory_;
    double machine_friction_replay_rate_{ 0.0 };
    std::string machine_calibration_error_;

    mutable std::mutex model_calibration_mutex_;
    std::condition_variable model_calibration_cv_;
    std::atomic<ModelCalibrationPhase> model_calibration_phase_{ ModelCalibrationPhase::IDLE };
    std::atomic<bool> model_calibration_cancel_{ false };
    std::atomic<bool> model_calibration_pause_{ false };
    std::atomic<bool> model_calibration_resume_confirmed_{ false };
    std::atomic<std::size_t> model_calibration_pose_group_{ 0 };
    std::atomic<bool> model_calibration_validation_{ false };
    std::atomic<int> model_calibration_direction_{ 0 };
    std::atomic<int> model_calibration_speed_tier_{ 0 };
    std::thread model_calibration_teach_thread_;
    std::thread model_calibration_worker_;
    std::atomic<bool> model_calibration_teach_stop_{ false };
    std::atomic<bool> model_calibration_fault_interrupted_{ false };
    ModelCalibrationRecorder model_calibration_recorder_{ 12000 };
    ModelCalibrationTaskOptions model_calibration_options_;
    std::string model_calibration_task_id_;
    std::atomic<bool> model_calibration_alignment_active_{false};
    std::atomic<bool> model_calibration_source_reversed_{false};
    std::string model_calibration_source_task_id_;
    std::string model_calibration_error_;
    std::string model_calibration_directory_;
    std::string model_calibration_core_fingerprint_;
    std::string model_calibration_urdf_fingerprint_;
    std::optional<FrictionCalibrationTrajectory> model_calibration_trajectory_;
    std::vector<ModelCalibrationPoseTarget> model_calibration_pose_targets_;
    double model_calibration_replay_rate_{ 0.0 };
    double model_calibration_estimated_duration_s_{ 0.0 };
    double model_calibration_original_duration_s_{ 0.0 };
    double model_calibration_path_length_rad_{ 0.0 };
    std::size_t model_calibration_geometric_waypoints_{ 0 };
    std::size_t model_calibration_original_samples_{ 0 };
    double model_calibration_teaching_wall_duration_s_{ 0.0 };
    std::optional<Robot::TimePoint> model_calibration_teaching_started_at_;
    std::optional<Robot::TimePoint> model_calibration_started_at_;
    double model_calibration_progress_{ 0.0 };
    std::size_t model_calibration_completed_units_{ 0 };
    std::size_t model_calibration_total_units_{ 0 };
    std::size_t model_calibration_valid_static_samples_{ 0 };
    JointVector model_calibration_pause_reference_;
    bool model_calibration_resume_requires_confirmation_{ false };
    JointImpedanceMode model_calibration_original_mode_{ JointImpedanceMode::RIGID_HOLD };
    ModelFeedforwardMode model_calibration_original_feedforward_{ ModelFeedforwardMode::NONE };
    AdmittanceCapabilityCfg model_calibration_original_admittance_;
    JointVector model_calibration_original_gravity_scale_;
    bool model_calibration_original_suspended_{ false };
    std::optional<GravityCalibrationResult> model_calibration_result_;
    std::optional<AdmittanceFrictionCalibrationResult> model_calibration_friction_result_;
    bool model_calibration_friction_pass_{ false };
    bool model_calibration_candidate_applied_{ false };
    JointVector model_calibration_applied_original_gravity_scale_;
    AdmittanceCapabilityCfg model_calibration_applied_original_admittance_;
    bool persistent_gravity_correction_loaded_{ false };
};

} // namespace

int main(int argc, char** argv) {
    CliOptions options;
    if(!parse_cli(argc, argv, options)) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    if(options.show_help) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }
    if(!options.robot_profile.empty()) {
        if(!options.config_path.empty() && options.config_path != SERIAL_ARM_DEFAULT_CONFIG_PATH) {
            std::cerr << "--robot-profile 不能与 --config 同时使用\n";
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        if(!options.hardware_plugin.empty() || !options.hardware_config.empty()) {
            std::cerr << "--robot-profile 不能与 --hardware-plugin/--hardware-config 同时使用\n";
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        RobotProfileLoadOptions profile_options;
        profile_options.profile_file = options.profiles_file;
        const auto profile = load_robot_profile_core(options.robot_profile, profile_options);
        if(!profile) {
            std::cerr << profile.error().message << '\n';
            return EXIT_FAILURE;
        }
        options.config_path = profile->core_config_path;
        options.hardware_plugin = profile->hardware_plugin;
        options.hardware_config = profile->hardware_config_path;
    }
    if(options.hardware_plugin.empty() || options.hardware_config.empty()) {
        std::cerr << "--hardware-plugin and --hardware-config are required\n";
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    if(options.compare_config) {
        HardwareLoader compare_loader;
        auto compare_bus = compare_loader.load(options.hardware_plugin, options.hardware_config);
        if(!compare_bus) {
            std::cerr << "HardwareLoader 失败\n";
            return EXIT_FAILURE;
        }
        const auto diffs = compare_robot_cfg(options.compare_lhs_path, options.compare_rhs_path, compare_bus.value()->capabilities());
        if(!diffs) {
            std::cerr << "配置比较失败: " << diffs.error().message << '\n';
            return EXIT_FAILURE;
        }
        std::cout << "Config compare: " << options.compare_lhs_path << " <-> " << options.compare_rhs_path << '\n';
        if(diffs->empty()) {
            std::cout << "未发现差异\n";
        }
        else {
            for(const auto& diff : *diffs) std::cout << "DIFF " << diff << '\n';
        }
        return EXIT_SUCCESS;
    }
    HardwareLoader config_loader;
    auto config_bus = config_loader.load(options.hardware_plugin, options.hardware_config, options.hardware_overrides);
    if(!config_bus) {
        std::cerr << "HardwareLoader 失败: " << to_string(config_bus.error()) << '\n';
        return EXIT_FAILURE;
    }
    const auto cfg_result = load_robot_cfg(options.config_path, config_bus.value()->capabilities());
    if(!cfg_result) {
        std::cerr << "配置加载失败: " << cfg_result.error().message << '\n';
        return EXIT_FAILURE;
    }

    const auto connection_summary = load_hardware_connection_summary(options.hardware_config, options.hardware_overrides);
    if(!connection_summary) {
        std::cerr << connection_summary.error() << '\n';
        return EXIT_FAILURE;
    }

    TerminalApp app(
        cfg_result.value(),
        options.config_path,
        options.hardware_plugin,
        options.hardware_config,
        options.hardware_overrides,
        connection_summary.value(),
        options.robot_profile,
        options.machine_mode);
    const auto init_result = app.initialize();
    if(!init_result) {
        std::cerr << init_result.error() << '\n';
        return EXIT_FAILURE;
    }
    return options.machine_mode ? app.run_machine() : app.run();
}
