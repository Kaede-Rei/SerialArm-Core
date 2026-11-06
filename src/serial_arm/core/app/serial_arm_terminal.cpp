#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/dynamics/gravity_correction.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include "serial_arm/dynamics/calibration_acquisition_plan.hpp"
#include "serial_arm/dynamics/calibration_retime.hpp"
#include "serial_arm/dynamics/calibration_measurement_quality.hpp"
#include "serial_arm/dynamics/calibration_auto_gain.hpp"
#include "serial_arm/dynamics/calibration_reference_watchdog.hpp"
#include "serial_arm/dynamics/calibration_replay_supervisor.hpp"
#include "serial_arm/dynamics/calibration_gain_adaptation.hpp"
#include "serial_arm/dynamics/servo_gain_tuning.hpp"
#include "serial_arm/dynamics/calibration_recovery_alignment.hpp"
#include "serial_arm/core/park_policy.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/torque_residual_observer.hpp"
#include "serial_arm/robot.hpp"
#include "serial_arm/runtime/control_runtime.hpp"
#include "serial_arm/runtime/joint_motion.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Cholesky>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <exception>
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
#include <poll.h>
#include <unistd.h>

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
    bool park_mode{ false };
    double speed_scale{ 0.3 };
    JointVector target_pos;
    JointVector ref_pos;
    JointVector ref_vel;
    Robot::TimePoint last_update_time{ };
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
    GAIN_ADAPTATION,
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



struct ModelCalibrationTaskOptions {
    std::string load_description{ "unspecified" };
    std::size_t pose_budget{ 12 };
    std::size_t minimum_training_groups{ 4 };
    double validation_fraction{ 0.30 };
    double static_hold_s{ 0.20 };
    double static_sample_s{ 0.45 };
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
        case ModelCalibrationPhase::GAIN_ADAPTATION: return "gain_adaptation";
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

std::string to_string(JointGainProfile value) {
    switch(value){case JointGainProfile::RIGID:return "RIGID";case JointGainProfile::COMPLIANT:return "COMPLIANT";
        case JointGainProfile::DRAG:return "DRAG";case JointGainProfile::SERVO_TRACKING:return "SERVO_TRACKING";}
    return "UNKNOWN";
}
std::string to_string(GainSource value) {
    switch(value){case GainSource::CONFIGURED:return "CONFIGURED";case GainSource::CALIBRATION_OVERRIDE:return "CALIBRATION_OVERRIDE";
        case GainSource::PARK_OVERRIDE:return "PARK_OVERRIDE";case GainSource::FULL_COMMAND:return "FULL_COMMAND";case GainSource::TRANSITION:return "TRANSITION";}
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
        case RobotErr::REFERENCE_GOVERNOR_FAILED: return "REFERENCE_GOVERNOR_FAILED";
        case RobotErr::MOTION_SOURCE_BUSY: return "MOTION_SOURCE_BUSY";
        case RobotErr::FAULT_RECOVERY_NOT_ALLOWED: return "FAULT_RECOVERY_NOT_ALLOWED";
    }
    return "UNKNOWN";
}

std::string to_string(ReferenceGovernorErr value) {
    return value==ReferenceGovernorErr::INFEASIBLE ? "INFEASIBLE" : "INVALID_INPUT";
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
        case HardwareLoaderErr::ABI_MISMATCH: return "ABI_MISMATCH: rebuild the Backend using the installed SDK";
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
    const std::vector<std::uint8_t>& values) {
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
        case RobotErr::REFERENCE_GOVERNOR_FAILED:
            std::cout << "  ReferenceGovernorErr: " << to_string(fault.reference_governor_err) << '\n';
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

// Synthetic bounds are used only by the write-disabled mock session. They
// are never accepted as evidence of physical actuator capabilities.
HardwareCapabilities simulation_capabilities(const std::vector<std::string>& names) {
    HardwareCapabilities result;
    for(const auto& name:names) {
        ActuatorCapability value;value.actuator_name=name;value.joint_name=name;
        value.min_pos=-1e6;value.max_pos=1e6;value.max_vel=1e12;value.max_effort=1e12;value.max_kp=1e12;value.max_kd=1e12;
        result.push_back(std::move(value));
    }
    return result;
}

class MockMotorBus final : public MotorBus {
public:
    explicit MockMotorBus(HardwareCapabilities capabilities) : capabilities_(std::move(capabilities)) {
        const auto size = capabilities_.size();
        state_.pos.assign(size, 0.0);
        state_.vel.assign(size, 0.0);
        state_.tor.assign(size, 0.0);
        state_.online.assign(size, 1);
        state_.enabled.assign(size, 1);
        state_.err_code.assign(size, 0);
    }

    tl::expected<void, MotorBusErr> configure(const std::string& config_path) override {
        static_cast<void>(config_path);
        return { };
    }

    tl::expected<void, MotorBusErr> connect() override {
        connected_ = true;
        return { };
    }

    tl::expected<ActuatorState, MotorBusErr> read() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        const auto stamp=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
        state_.feedback_received_ns.assign(size(),stamp);state_.feedback_timing_valid.assign(size(),1);
        return state_;
    }

    tl::expected<void, MotorBusErr> activate() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        active_ = true;
        return { };
    }

    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
        state_.pos = cmd.pos;
        state_.vel = cmd.vel;
        state_.tor = cmd.tor;
        return { };
    }

    tl::expected<void, MotorBusErr> stop() override { return { }; }
    tl::expected<void, MotorBusErr> deactivate() override {
        active_ = false;
        return { };
    }
    tl::expected<void, MotorBusErr> recover() override { return { }; }
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
        if(gravity_verify_active_ && robot_.get_state() == RobotState::INACTIVE) {
            try { gravity_verify_restore_locked(); } catch(...) { }
        }
    }

    tl::expected<void, std::string> initialize() {
        const auto capture=[](const std::string& path,std::string& text) {
            std::ifstream input(path,std::ios::binary);if(!input)return false;
            text.assign(std::istreambuf_iterator<char>(input),{});return !input.bad();
        };
        if(!capture(config_path_,core_source_config_) || !capture(cfg_.dynamics.urdf_path,urdf_source_config_))
            return tl::make_unexpected(std::string("failed to snapshot the loaded Core configuration or URDF"));
        if(!cfg_.dynamics.gravity_correction_path.empty() && !capture(cfg_.dynamics.gravity_correction_path,gravity_correction_source_config_))
            return tl::make_unexpected(std::string("failed to snapshot the loaded gravity correction"));
        RobotCfg robot_cfg = cfg_;
        const auto dynamics_result = configure_robot_dynamics(robot_cfg, dynamics_);
        if(!dynamics_result) return tl::make_unexpected("Dynamics configure() 失败: " + to_string(dynamics_result.error()));
        model_calibration_links_cache_.clear();
        if(const auto links = dynamics_.get_gravity_regression(JointVector(cfg_.joint_names.size(), 0.0)); links)
            model_calibration_links_cache_ = links->links;
        if(!cfg_.dynamics.gravity_correction_path.empty()) {
            model_calibration_applied_original_gravity_scale_ = cfg_.dynamics.gravity_scale;
            model_calibration_applied_original_admittance_ = cfg_.capability.admittance;
            cfg_ = robot_cfg;
            model_calibration_candidate_applied_ = true;
            persistent_gravity_correction_loaded_ = true;
        }

        std::unique_ptr<MotorBus> motor_bus;
        if(cfg_.runtime.write_enabled) {
            auto hardware_result=hardware_loader_.load(hardware_plugin_,hardware_config_,hardware_overrides_);
            if(!hardware_result)return tl::make_unexpected("HardwareLoader 失败: "+to_string(hardware_result.error()));
            auto hardware_bus=std::move(hardware_result.value());
            const auto ordered=capabilities_for_joint_order(hardware_bus->capabilities(),cfg_.joint_names);
            if(!ordered)return tl::make_unexpected(std::string("hardware joint identities do not match Core"));
            actuator_info_=*ordered;motor_bus=std::move(hardware_bus);
        } else {
            actuator_info_=simulation_capabilities(cfg_.joint_names);
            robot_cfg.runtime.write_enabled=true;
            motor_bus=std::make_unique<MockMotorBus>(actuator_info_);
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
            if(!dynamics_.update_state(state,JointVector(state.pos.size(),0.0))) {
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
        return { };
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
        std::atomic<bool> parking_running{ false };
        std::atomic<bool> parking_cancelled{ false };
        std::atomic<bool> shutdown_complete{ false };
        std::thread parking;

        auto emit_reply = [&](long long id, const std::string& result, const std::string& error = std::string{ }) {
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
                catch(...) { }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            });

        auto stop_telemetry = [&]() {
            telemetry_running.store(false);
            if(telemetry.joinable()) telemetry.join();
            };

        auto cancel_parking = [&]() {
            parking_cancelled.store(true);
            cycle_cv_.notify_all();
            if(parking.joinable()) parking.join();
            };
        auto start_parking = [&](long long id, bool shutdown) {
            if(parking.joinable()) parking.join();
            parking_cancelled.store(false);
            parking_running.store(true);
            try { parking = std::thread([&, id, shutdown]() {
                try {
                    park_and_deactivate(&parking_cancelled);
                    if(parking_cancelled.load()) throw std::runtime_error("park interrupted by operator");
                    if(shutdown && machine_robot_state() == RobotState::FAULT) machine_deactivate();
                    if(machine_robot_state() != RobotState::INACTIVE) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        throw std::runtime_error(park_last_error_.empty() ? "park did not reach INACTIVE" : park_last_error_);
                    }
                    emit_reply(id, "true");
                    if(shutdown) shutdown_complete.store(true);
                }
                catch(const std::exception& error) { emit_reply(id, "null", error.what()); }
                parking_running.store(false);
                }); }
            catch(...) { parking_running.store(false); throw; }
            };

        // Poll the pipe so successful asynchronous shutdown can end this loop
        // without waiting for another command or for the peer to close stdin.
        std::string buffered_input;
        bool input_eof = false;
        auto next_request = [&](std::string& line) {
            while(!shutdown_complete.load()) {
                const auto newline = buffered_input.find('\n');
                if(newline != std::string::npos) {
                    line = buffered_input.substr(0, newline);
                    buffered_input.erase(0, newline + 1);
                    return true;
                }
                if(input_eof) {
                    if(buffered_input.empty()) return false;
                    line.swap(buffered_input);
                    return true;
                }
                pollfd input{ STDIN_FILENO, POLLIN, 0 };
                const int ready = ::poll(&input, 1, 20);
                if(ready < 0) {
                    if(errno == EINTR) continue;
                    throw std::runtime_error("native request pipe polling failed");
                }
                if(!ready) continue;
                char chunk[8192];
                const auto count = ::read(STDIN_FILENO, chunk, sizeof(chunk));
                if(count < 0) {
                    if(errno == EINTR) continue;
                    throw std::runtime_error("native request pipe read failed");
                }
                if(count == 0) input_eof = true;
                else buffered_input.append(chunk, static_cast<std::size_t>(count));
                if(buffered_input.size() > 1024 * 1024 && buffered_input.find('\n') == std::string::npos)
                    throw std::runtime_error("request exceeds 1 MiB");
            }
            return false;
            };

        try {
            std::string line;
            while(next_request(line)) {
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
                    if(parking_running.load() && method != "status" && method != "get_admittance" &&
                       method != "candidate_overrides" && method != "model_calibration_status" &&
                       method != "calibration_status" && method != "teach_replay_status" &&
                       method != "deactivate" && method != "hold")
                        throw std::runtime_error("stop the active park operation before other commands");
                    if((teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3) &&
                       method != "status" && method != "teach_replay_status" &&
                       method != "teach_replay_record_stop" && method != "teach_replay_stop" &&
                       method != "deactivate" && method != "hold" && method != "shutdown")
                        throw std::runtime_error("stop the independent teaching or replay task before other commands");

                    if(method == "status") {
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "activate") {
                        machine_activate();
                        emit_reply(id, "true");
                    }
                    else if(method == "deactivate") {
                        cancel_parking();
                        machine_deactivate();
                        emit_reply(id, "true");
                    }
                    else if(method == "park") {
                        if(teach_replay_phase_.load()==1||teach_replay_phase_.load()==3)throw std::runtime_error("stop teaching/replay before automatic park");
                        if(gravity_verify_active_) throw std::runtime_error("automatic park uses tracking modes; use supported gravity verification stop");
                        // The demonstration is already stopped in this phase.
                        // Parking is an explicit operator decision to abandon
                        // pending replay, preserving its saved task data. Do
                        // not silently interrupt an active replay/recording.
                        if(model_calibration_phase_.load() == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)
                            machine_model_calibration_cancel();
                        machine_require_no_calibration("park");
                        start_parking(id, false);
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
                        machine_set_impedance_mode(machine_parse_impedance_mode(params["mode"].as<std::string>()),params["gain_profile"] ? std::optional<JointGainProfile>(machine_parse_gain_profile(params["gain_profile"].as<std::string>())) : std::nullopt);
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
                    else if(method == "gravity_verify_begin") {
                        machine_gravity_verify_begin();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_drag") {
                        machine_gravity_verify_drag();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_target") {
                        machine_gravity_verify_target(machine_node_vector(params["values"], "values"));
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_sample") {
                        machine_gravity_verify_sample();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_stop") {
                        if(!params || !params["supported"] || !params["supported"].as<bool>())
                            throw std::runtime_error("explicit physical support confirmation is required");
                        machine_gravity_verify_stop();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_finish") {
                        machine_gravity_verify_finish();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "gravity_verify_cancel") {
                        machine_gravity_verify_cancel();
                        emit_reply(id, machine_snapshot_json());
                    }
                    else if(method == "move_to") {
                        const double speed_scale = params && params["speed_scale"] ? params["speed_scale"].as<double>() : 0.3;
                        machine_move_to(machine_node_vector(params["positions"], "positions"), speed_scale, false);
                        emit_reply(id, machine_motion_json());
                    }
                    else if(method == "move_relative") {
                        const double speed_scale = params && params["speed_scale"] ? params["speed_scale"].as<double>() : 0.3;
                        machine_move_to(machine_node_vector(params["delta"], "delta"), speed_scale, true);
                        emit_reply(id, machine_motion_json());
                    }
                    else if(method == "get_admittance") {
                        emit_reply(id, machine_admittance_json());
                    }
                    else if(method == "set_reference_behavior") {
                        if(!params || !params["behavior"])throw std::runtime_error("reference behavior is required");
                        machine_set_reference_behavior(params["behavior"].as<std::string>());
                        emit_reply(id,machine_admittance_json());
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
                    else if(method == "teach_replay_status") { emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_record_begin") { machine_teach_record_begin(); emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_record_stop") { machine_teach_record_stop(); emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_save") { machine_teach_save(params["path"].as<std::string>()); emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_load") { machine_teach_load(params["path"].as<std::string>()); emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_play") { machine_teach_play(params["direction"].as<std::string>(), params["rate"].as<double>(), params["cycles"].as<int>(), params["confirmed"].as<bool>(), params["mode"].as<std::string>(), params["endpoint_dwell_s"] ? params["endpoint_dwell_s"].as<double>() : 0.0, params["reference_behavior"] ? params["reference_behavior"].as<std::string>() : "", params["gain_profile"] ? params["gain_profile"].as<std::string>() : ""); emit_reply(id, machine_teach_replay_json()); }
                    else if(method == "teach_replay_stop") { machine_teach_replay_stop(); emit_reply(id, machine_teach_replay_json()); }
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
                    else if(method == "model_calibration_finish_teaching") {
                        machine_model_calibration_finish_teaching(params);
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
                    else if(method == "candidate_gain_apply") {
                        machine_candidate_gain_apply(params);
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_gain_restore") {
                        machine_candidate_gain_restore();
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_gravity_apply") {
                        machine_candidate_gravity_apply(params);
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_gravity_restore") {
                        machine_candidate_gravity_restore();
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_overrides") {
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_dynamics_apply") {
                        machine_candidate_dynamics_apply(params);
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "candidate_dynamics_restore") {
                        machine_candidate_dynamics_restore();
                        emit_reply(id, machine_candidate_overrides_json());
                    }
                    else if(method == "hold") {
                        cancel_parking();
                        machine_hold();
                        emit_reply(id, "true");
                    }
                    else if(method == "shutdown") {
                        if(teach_replay_phase_.load()==3)machine_teach_replay_stop();
                        if(teach_replay_phase_.load()==1 || teach_replay_phase_.load()==4){teach_replay_record_stop_.store(true);teach_replay_stop_.store(true);cycle_cv_.notify_all();if(teach_replay_worker_.joinable())teach_replay_worker_.join();}
                        if(gravity_verify_active_ && machine_robot_state() != RobotState::INACTIVE)
                            throw std::runtime_error("gravity verification must be stopped with physical support before shutdown");
                        if(gravity_verify_active_) machine_gravity_verify_cancel();
                        if(machine_calibration_kind_ != "none") machine_calibration_cancel();
                        if(model_calibration_task_active()) { machine_model_calibration_cancel(); if(model_calibration_worker_.joinable()) model_calibration_worker_.join(); }
                        start_parking(id, true);
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
            cancel_parking();
            teach_replay_record_stop_.store(true);teach_replay_stop_.store(true);model_calibration_cancel_.store(true);cycle_cv_.notify_all();
            if(teach_replay_worker_.joinable())teach_replay_worker_.join();
            if(machine_friction_thread_.joinable()) {
                machine_friction_stop_.store(true); cycle_cv_.notify_all(); machine_friction_thread_.join();
            }
            stop_telemetry();
            throw;
        }

        cancel_parking();
        teach_replay_record_stop_.store(true);teach_replay_stop_.store(true);model_calibration_cancel_.store(true);cycle_cv_.notify_all();
        if(teach_replay_worker_.joinable())teach_replay_worker_.join();
        if(machine_calibration_kind_ != "none") machine_calibration_cancel();
        if(machine_friction_thread_.joinable()) {
            machine_friction_stop_.store(true); cycle_cv_.notify_all(); machine_friction_thread_.join();
        }
        stop_telemetry();
        if(machine_robot_state() == RobotState::ACTIVE) {
            if(gravity_verify_active_) {
                // The IPC peer disappeared while the arm used an unverified drag controller
                // Do not start an automatic trajectory with unvalidated tracking gains
                machine_deactivate();
            } else {
                park_and_deactivate();
            }
        }
        if(machine_robot_state() == RobotState::FAULT) machine_deactivate();
        if(gravity_verify_active_ && machine_robot_state() == RobotState::INACTIVE)
            machine_gravity_verify_cancel();
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
        std::string detail = std::string(action) + " failed: " + to_string(fault.code);
        if(fault.code == RobotErr::MOTOR_BUS_CONNECT_FAILED ||
           fault.code == RobotErr::MOTOR_BUS_ACTIVATE_FAILED ||
           fault.code == RobotErr::MOTOR_BUS_READ_FAILED ||
           fault.code == RobotErr::MOTOR_BUS_WRITE_FAILED ||
           fault.code == RobotErr::MOTOR_BUS_DEACTIVATE_FAILED)
            detail += " / " + to_string(fault.motor_bus_err);
        if(fault.code==RobotErr::SAFETY_FAILED) {
            detail += " / " + to_string(fault.safety_fault.code);
            if(fault.safety_fault.index!=kInvalidIndex) detail += " / joint_index=" + std::to_string(fault.safety_fault.index);
            detail += " / value=" + std::to_string(fault.safety_fault.value) + " / limit=" + std::to_string(fault.safety_fault.limit);
        }
        if(fault.code==RobotErr::REFERENCE_GOVERNOR_FAILED)
            detail += " / " + to_string(fault.reference_governor_err);
        return std::runtime_error(detail);
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
        if(teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3)
            throw std::runtime_error(std::string(action) + " is blocked by teaching/replay");
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) {
            throw std::runtime_error(std::string(action) + " is blocked while a calibration task is active");
        }
    }

    void machine_activate() {
        if(teach_replay_phase_.load()==4 && teach_replay_worker_.joinable())teach_replay_worker_.join();
        if((model_calibration_phase_.load() == ModelCalibrationPhase::FAILED ||
            model_calibration_phase_.load() == ModelCalibrationPhase::CANCELLED) &&
            model_calibration_teach_thread_.joinable()) machine_model_calibration_cancel();
        machine_require_no_calibration("activate");
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        if(robot_.get_state() != RobotState::ACTIVE) actuator_info_after_connect_ = false;
        const auto result = robot_.activate();
        cfg_.safety = robot_.get_config().safety;
        if(!result) throw machine_robot_error("activate", result.error());
        refresh_actuator_info_locked();
        if(teach_restore_gain_profile_) {
            const auto profile=robot_.set_execution_gain_profile(*teach_restore_gain_profile_);
            if(!profile)throw machine_robot_error("restore execution gain profile",profile.error());
            teach_restore_gain_profile_.reset();
        }
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_deactivate() {
        // Disable first. A recording/replay worker must not delay the motor
        // disable while it finishes sampling, restores parameters or joins.
        const bool stop_teaching = teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3;
        if(stop_teaching) {
            teach_replay_record_stop_.store(true);
            teach_replay_stop_.store(true);
        }
        const bool stop_legacy = machine_calibration_kind_ != "none";
        if(stop_legacy) machine_friction_stop_.store(true);
        const auto teaching_phase = model_calibration_phase_.load();
        const bool stopping_teaching = teaching_phase == ModelCalibrationPhase::TEACHING ||
            ((teaching_phase == ModelCalibrationPhase::FAILED || teaching_phase == ModelCalibrationPhase::CANCELLED) &&
                model_calibration_teach_thread_.joinable());
        if(model_calibration_task_active()) {
            model_calibration_cancel_.store(true);
            model_calibration_teach_stop_.store(true);
            model_calibration_pause_.store(false);
            model_calibration_cv_.notify_all();
            cycle_cv_.notify_all();
        }
        cycle_cv_.notify_all();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clear_command_sources();
            const auto result = robot_.force_deactivate();
            if(!result) throw machine_robot_error("deactivate", result.error());
            teach_restore_controls_locked();
            if(gravity_verify_active_) { gravity_verify_restore_locked(); gravity_verify_result_ = "cancelled"; }
            last_output_.reset();
            feedback_time_ns_ = 0;
            background_fault_reported_ = false;
        }
        if(stop_teaching && teach_replay_worker_.joinable()) teach_replay_worker_.join();
        if(stop_legacy) machine_calibration_cancel(false);
        // Explicit disable also closes an interrupted demonstration
        // Do not hold the robot mutex while joining its sampling worker
        if(stopping_teaching) machine_model_calibration_cancel();
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
        if(!result) {
            if(result.error().code == RobotErr::FAULT_RECOVERY_NOT_ALLOWED) {
                const auto& recovery = cfg_.safety.fault_recovery;
                if(!recovery.allow_compliant_recovery)
                    throw std::runtime_error("fault_compliant: fault_recovery.allow_compliant_recovery is false");
                if(!recovery.gravity_model_validated)
                    throw std::runtime_error("fault_compliant: fault_recovery.gravity_model_validated is false; verify the current robot gravity model and confirm it in Profile Safety before reconnecting");
                if(recovery.default_mode != FaultHoldMode::RIGID_HOLD || !recovery.require_operator_request)
                    throw std::runtime_error("fault_compliant: fault recovery policy does not permit operator compliant recovery");
                if(const auto& fault = robot_.get_last_fault()) {
                    if(fault->code == RobotErr::SAFETY_FAILED)
                        throw std::runtime_error("fault_compliant: fault type is not eligible for compliant recovery: " + to_string(fault->safety_fault.code));
                    throw std::runtime_error("fault_compliant: fault type is not eligible for compliant recovery: " + to_string(fault->code));
                }
            }
            throw machine_robot_error("fault_compliant", result.error());
        }
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

    static JointGainProfile machine_parse_gain_profile(const std::string& value) {
        if(value=="RIGID")return JointGainProfile::RIGID;
        if(value=="COMPLIANT")return JointGainProfile::COMPLIANT;
        if(value=="DRAG")return JointGainProfile::DRAG;
        if(value=="SERVO_TRACKING")return JointGainProfile::SERVO_TRACKING;
        throw std::runtime_error("unknown execution gain profile");
    }
    void machine_set_impedance_mode(JointImpedanceMode mode,std::optional<JointGainProfile> profile=std::nullopt) {
        if(teach_replay_phase_.load()==1||teach_replay_phase_.load()==3)throw std::runtime_error("teaching or replay owns the control mode");
        machine_require_no_calibration("set_impedance_mode");
        if(gravity_verify_active_) throw std::runtime_error("impedance mode is owned by gravity verification");
        std::unique_lock<std::mutex> lock(mutex_);
        if(profile) {
            const bool paired=*profile==JointGainProfile::RIGID ? (mode==JointImpedanceMode::RIGID_HOLD || mode==JointImpedanceMode::RIGID_TRACKING) :
                *profile==JointGainProfile::COMPLIANT ? (mode==JointImpedanceMode::COMPLIANT_HOLD || mode==JointImpedanceMode::COMPLIANT_TRACKING) :
                *profile==JointGainProfile::DRAG ? mode==JointImpedanceMode::COMPLIANT_DRAG :
                (mode==JointImpedanceMode::RIGID_HOLD || mode==JointImpedanceMode::RIGID_TRACKING);
            if(!paired)throw std::runtime_error("execution gain profile does not match its hold/tracking/drag task");
        }
        const auto& selected=robot_.get_admittance_cfg();
        if(profile && mode!=JointImpedanceMode::COMPLIANT_DRAG && selected.enabled &&
           selected.reference_behavior==ReferenceBehavior::MODEL_ADMITTANCE && *profile!=JointGainProfile::SERVO_TRACKING)
            throw std::runtime_error("MODEL_ADMITTANCE requires SERVO_TRACKING execution gains");
        if(mode==JointImpedanceMode::COMPLIANT_DRAG) require_drag_teaching_ready_locked(lock);
        clear_command_sources();
        if(mode==JointImpedanceMode::COMPLIANT_DRAG && !direct_drag_controls_) {
            teach_prepare_controls_locked(ReferenceBehavior::NATIVE_IMPEDANCE,true);direct_drag_controls_=true;
        }
        const auto result = runtime_.set_impedance_mode(robot_, mode);
        if(!result) {if(direct_drag_controls_){teach_restore_controls_locked();direct_drag_controls_=false;}throw machine_robot_error("set_impedance_mode", result.error());}
        if(mode!=JointImpedanceMode::COMPLIANT_DRAG && direct_drag_controls_) {teach_restore_controls_locked();direct_drag_controls_=false;}
        if(profile) {
            const auto selected_profile=robot_.set_execution_gain_profile(*profile);
            if(!selected_profile)throw machine_robot_error("set_impedance_mode gains",selected_profile.error());
        }
        last_output_.reset();
        feedback_time_ns_ = 0;
        background_fault_reported_ = false;
    }

    void machine_set_model_feedforward_mode(ModelFeedforwardMode mode) {
        machine_require_no_calibration("set_model_feedforward_mode");
        if(gravity_verify_active_) throw std::runtime_error("feedforward mode is owned by gravity verification");
        if(dynamics_.has_gravity_first_moment_override() && mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            throw std::runtime_error("FULL_INVERSE_DYNAMICS is unavailable while gravity correction is active");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result = robot_.set_model_feedforward_mode(mode);
        if(!result) throw machine_robot_error("set_model_feedforward_mode", result.error());
        cfg_.runtime.model_feedforward_mode = mode;
    }

    void gravity_verify_restore_locked() {
        if(!gravity_verify_active_) return;
        if(robot_.get_state() != RobotState::INACTIVE) throw std::runtime_error("gravity verification restore requires INACTIVE");
        const auto result = dynamics_.set_gravity_scale(gravity_verify_original_scale_);
        if(!result) throw std::runtime_error("gravity verification scale restore failed: " + to_string(result.error()));
        cfg_.dynamics.gravity_scale = gravity_verify_original_scale_;
        const auto feed = robot_.set_model_feedforward_mode(gravity_verify_original_feed_);
        if(!feed) throw machine_robot_error("gravity verification feedforward restore", feed.error());
        cfg_.runtime.model_feedforward_mode = gravity_verify_original_feed_;
        gravity_verify_active_ = false;
        gravity_verify_target_scale_.clear();
    }

    void machine_gravity_verify_begin() {
        machine_require_no_calibration("gravity_verify_begin");
        std::lock_guard<std::mutex> lock(mutex_);
        if(gravity_verify_active_) throw std::runtime_error("gravity verification is already active");
        if(robot_.get_state() != RobotState::INACTIVE || !cfg_.runtime.write_enabled)
            throw std::runtime_error("gravity verification requires a real INACTIVE hardware session");
        if(dynamics_.has_gravity_first_moment_override() || model_calibration_candidate_applied_ ||
           runtime_gravity_applied_ || runtime_gain_applied_ || runtime_dynamics_applied_)
            throw std::runtime_error("restore temporary calibration candidates before gravity verification");
        const auto& gains = cfg_.ctrller.compliant_drag_gains;
        if(gains.kd.size() != cfg_.joint_names.size() ||
           !std::all_of(gains.kd.begin(), gains.kd.end(), [](double d) { return std::isfinite(d) && d > 0.0; }))
            throw std::runtime_error("gravity verification requires positive COMPLIANT_DRAG damping on every joint");
        gravity_verify_original_scale_ = dynamics_.get_gravity_scale();
        if(!std::any_of(gravity_verify_original_scale_.begin(), gravity_verify_original_scale_.end(),
                        [](double value) { return value > 0.0; }))
            throw std::runtime_error("gravity verification requires a nonzero baseline gravity model");
        gravity_verify_original_feed_ = robot_.get_model_feedforward_mode();
        const JointVector zero(cfg_.joint_names.size(), 0.0);
        const auto reset = dynamics_.set_gravity_scale(zero);
        if(!reset) throw std::runtime_error("gravity verification initial scale failed");
        const auto feed = robot_.set_model_feedforward_mode(ModelFeedforwardMode::GRAVITY);
        if(!feed) {
            (void)dynamics_.set_gravity_scale(gravity_verify_original_scale_);
            throw machine_robot_error("gravity verification feedforward", feed.error());
        }
        cfg_.dynamics.gravity_scale = zero;
        cfg_.runtime.model_feedforward_mode = ModelFeedforwardMode::GRAVITY;
        gravity_verify_target_scale_ = gravity_verify_original_scale_;
        gravity_verify_samples_.clear();
        gravity_verify_result_ = "prepared";
        gravity_verify_active_ = true;
        gravity_verify_drag_entered_ = false;
        gravity_verify_stopped_safely_ = false;
    }

    void machine_gravity_verify_require_feedback_locked() const {
        if(robot_.get_state() != RobotState::ACTIVE || !last_output_)
            throw std::runtime_error("gravity verification requires valid ACTIVE feedback");
        const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            Robot::Clock::now().time_since_epoch()).count());
        if(!feedback_time_ns_ || now_ns < feedback_time_ns_ || now_ns - feedback_time_ns_ > 150000000)
            throw std::runtime_error("gravity verification feedback is stale");
        if(last_output_->joint_state.vel.size() != cfg_.joint_names.size() ||
           std::any_of(last_output_->joint_state.vel.begin(), last_output_->joint_state.vel.end(),
                       [](double value) { return !std::isfinite(value) || std::abs(value) > 0.35; }))
            throw std::runtime_error("gravity verification requires stationary joints");
    }

    void machine_gravity_verify_drag() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_ || gravity_verify_result_ != "prepared")
            throw std::runtime_error("start gravity verification before entering drag");
        machine_gravity_verify_require_feedback_locked();
        if(robot_.get_impedance_mode() != JointImpedanceMode::RIGID_HOLD ||
           robot_.get_model_feedforward_mode() != ModelFeedforwardMode::GRAVITY)
            throw std::runtime_error("gravity verification requires RIGID_HOLD with GRAVITY feedforward");
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i)
            if(std::abs(cfg_.dynamics.gravity_scale[i] - gravity_verify_target_scale_[i]) > 0.005)
                throw std::runtime_error("gravity compensation ramp has not settled");
        clear_command_sources();
        const auto result = runtime_.set_impedance_mode(robot_, JointImpedanceMode::COMPLIANT_DRAG);
        if(!result) throw machine_robot_error("gravity verification drag", result.error());
        gravity_verify_drag_entered_ = true;
        gravity_verify_result_ = "dragging";
    }

    void machine_gravity_verify_target(const JointVector& values) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_ || !gravity_verify_drag_entered_ || gravity_verify_result_ != "dragging" ||
           robot_.get_impedance_mode() != JointImpedanceMode::COMPLIANT_DRAG)
            throw std::runtime_error("gravity scale online adjustment requires active gravity verification drag");
        machine_gravity_verify_require_feedback_locked();
        if(dynamics_.has_gravity_first_moment_override() || model_calibration_task_active())
            throw std::runtime_error("gravity verification conflicts with calibration");
        if(values.size() != cfg_.joint_names.size() ||
           !std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x) && x >= 0.0 && x <= 2.0; }))
            throw std::runtime_error("gravity scale must contain finite values in [0,2] for all joints");
        if(!std::any_of(values.begin(), values.end(), [](double value) { return value > 0.0; }))
            throw std::runtime_error("all-zero gravity compensation is unsafe during drag");
        gravity_verify_target_scale_ = values;
    }

    void machine_gravity_verify_sample() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_ || gravity_verify_result_ != "dragging" ||
           robot_.get_impedance_mode() != JointImpedanceMode::COMPLIANT_DRAG)
            throw std::runtime_error("gravity verification sampling requires COMPLIANT_DRAG");
        machine_gravity_verify_require_feedback_locked();
        if(!gravity_verify_target_scale_.empty())
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i)
                if(std::abs(gravity_verify_target_scale_[i] - cfg_.dynamics.gravity_scale[i]) > 0.005)
                    throw std::runtime_error("wait until gravity scale is settled before sampling");
        const auto& position = last_output_->joint_state.pos;
        if(position.size() != cfg_.joint_names.size() ||
           !std::all_of(position.begin(), position.end(), [](double x) { return std::isfinite(x); }))
            throw std::runtime_error("invalid joint positions");
        for(const auto& prior : gravity_verify_samples_) {
            double max_shift = 0.0;
            for(std::size_t i = 0; i < position.size(); ++i)
                max_shift = std::max(max_shift, std::abs(position[i] - prior[i]));
            if(max_shift < 0.08) throw std::runtime_error("move to a distinct stable pose before the next sample");
        }
        gravity_verify_samples_.push_back(position);
    }

    void machine_gravity_verify_stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_ || robot_.get_state() != RobotState::ACTIVE)
            throw std::runtime_error("gravity verification controlled stop requires ACTIVE");
        machine_gravity_verify_require_feedback_locked();
        clear_command_sources();
        const auto hold = runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD);
        if(!hold) throw machine_robot_error("gravity verification hold", hold.error());
        const auto disabled = robot_.force_deactivate();
        if(!disabled) throw machine_robot_error("gravity verification supported disable", disabled.error());
        last_output_.reset();
        feedback_time_ns_ = 0;
        gravity_verify_stopped_safely_ = true;
        gravity_verify_result_ = "stopped";
    }

    void machine_gravity_verify_finish() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_ || !gravity_verify_drag_entered_ ||
           !gravity_verify_stopped_safely_ || gravity_verify_samples_.size() < 3)
            throw std::runtime_error("gravity verification requires three poses and a supported controlled stop");
        if(robot_.get_state() != RobotState::INACTIVE)
            throw std::runtime_error("supported disable is required before completing gravity verification");
        gravity_verify_restore_locked();
        gravity_verify_result_ = "completed";
    }

    void machine_gravity_verify_cancel() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!gravity_verify_active_) return;
        if(robot_.get_state() != RobotState::INACTIVE)
            throw std::runtime_error("deactivate under physical support before cancelling gravity verification");
        gravity_verify_restore_locked();
        gravity_verify_result_ = "cancelled";
    }

    void gravity_verify_cycle_update() {
        if(!gravity_verify_active_ || gravity_verify_target_scale_.size() != cfg_.joint_names.size() ||
           robot_.get_state() != RobotState::ACTIVE) return;
        // Updates use the same mutex and period as Robot::cycle, not the GUI event cadence
        const double dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        JointVector next = cfg_.dynamics.gravity_scale;
        const auto& gravity = dynamics_.get_state().gravity;
        bool changed = false;
        for(std::size_t i = 0; i < next.size(); ++i) {
            const double gravity_nm = i < gravity.size() && std::isfinite(gravity[i]) ? std::abs(gravity[i]) : 0.0;
            // Bound both the scale slew and expected gravity torque slew per control period
            const double step = std::min(0.20 * dt, 6.0 * dt / std::max(1.0, gravity_nm));
            const double goal = gravity_verify_target_scale_[i];
            const double value = next[i] + std::clamp(goal - next[i], -step, step);
            changed |= value != next[i];
            next[i] = value;
        }
        if(changed) {
            const auto result = dynamics_.set_gravity_scale(next);
            if(!result) {
                gravity_verify_result_ = "failed";
                gravity_verify_target_scale_ = cfg_.dynamics.gravity_scale;
                return;
            }
            cfg_.dynamics.gravity_scale = next;
        }
    }

    void machine_set_gravity_scale(const JointVector& values) {
        machine_require_no_calibration("set_gravity_scale");
        if(gravity_verify_active_) throw std::runtime_error("gravity scale is owned by the active verification task");
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
            const bool limited=cfg_.safety.limits.has_position_limit.empty()||cfg_.safety.limits.has_position_limit[i]!=0;
            if(limited&&(target[i] < cfg_.safety.limits.min_pos[i]+cfg_.safety.limits.pos_margin[i] ||
                target[i] > cfg_.safety.limits.max_pos[i]-cfg_.safety.limits.pos_margin[i])) {
                throw std::runtime_error("target exceeds configured joint position limit: " + cfg_.joint_names[i]);
            }
        }
    }

    void machine_move_to(const JointVector& values, double speed_scale, bool relative) {
        if(teach_replay_phase_.load()==1||teach_replay_phase_.load()==3)throw std::runtime_error("teaching or replay owns motion");
        machine_require_no_calibration("motion");
        if(gravity_verify_active_) throw std::runtime_error("tracking commands are blocked during gravity verification");
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("motion requires RobotState::ACTIVE");
        const auto previous_mode=robot_.get_impedance_mode();
        const auto purpose=robot_.get_gain_profile();
        if(previous_mode==JointImpedanceMode::COMPLIANT_DRAG || purpose==JointGainProfile::DRAG)
            throw std::runtime_error("motion is unavailable in drag mode; select a hold or tracking behavior");
        JointVector target = values;
        if(relative) {
            if(values.size() != cfg_.joint_names.size()) throw std::runtime_error("delta size does not match joint count");
            target = current_reference_pos();
            for(std::size_t i = 0; i < target.size(); ++i) target[i] += values[i];
        }
        machine_validate_target(target, speed_scale);
        JointVector first_pos=current_reference_pos(),first_vel=current_reference_vel();
        for(std::size_t i=0;i<first_pos.size();++i)if(!std::isfinite(first_pos[i]) || !std::isfinite(first_vel[i]))throw std::runtime_error("motion seed reference is invalid");
        (void)advance_joint_reference(first_pos,first_vel,target,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,
            speed_scale,1.0/cfg_.runtime.ctrl_frequency_hz);
        if(!is_tracking_mode(previous_mode)) {
            const auto tracking=purpose==JointGainProfile::COMPLIANT ? JointImpedanceMode::COMPLIANT_TRACKING : JointImpedanceMode::RIGID_TRACKING;
            const auto changed=runtime_.set_impedance_mode(robot_,tracking);
            if(!changed)throw machine_robot_error("motion tracking transition",changed.error());
            if(purpose==JointGainProfile::SERVO_TRACKING && !robot_.set_execution_gain_profile(purpose)) {
                (void)runtime_.set_impedance_mode(robot_,previous_mode);
                throw std::runtime_error("cannot preserve servo gain purpose for motion");
            }
        }
        try {
            begin_stream(target,speed_scale);
            const auto queued=runtime_.submit_reference(robot_,JointPosVelCmd{first_pos,first_vel},Robot::Clock::now(),MotionSource::STREAM);
            if(!queued)throw machine_robot_error("motion submission",queued.error());
            motion_.begin(target,dynamics_.get_info().joint_types);
        } catch(...) {
            clear_command_sources();(void)runtime_.set_impedance_mode(robot_,previous_mode);
            (void)robot_.set_execution_gain_profile(purpose);throw;
        }
    }

    void machine_hold() {
        const bool stop_teaching = teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3;
        if(stop_teaching) {
            teach_replay_record_stop_.store(true);
            teach_replay_stop_.store(true);
        }
        const bool stop_legacy = machine_calibration_kind_ != "none";
        if(stop_legacy) machine_friction_stop_.store(true);
        if(model_calibration_task_active()) model_calibration_pause_.store(true);
        cycle_cv_.notify_all();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clear_command_sources();
        }
        // Finish stopped workers before the final mode write, so their own
        // cleanup cannot replace the operator's current-position hold.
        if(stop_teaching && teach_replay_worker_.joinable()) teach_replay_worker_.join();
        if(stop_legacy) machine_calibration_cancel(false);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("hold requires RobotState::ACTIVE");
            clear_command_sources();
            if(stop_teaching || direct_drag_controls_) teach_restore_controls_locked();
            const auto purpose=robot_.get_gain_profile();
            const auto hold_mode=purpose==JointGainProfile::COMPLIANT ? JointImpedanceMode::COMPLIANT_HOLD : JointImpedanceMode::RIGID_HOLD;
            const auto result = runtime_.set_impedance_mode(robot_,hold_mode,Robot::Clock::now(),HoldTargetPolicy::CAPTURE_MEASURED);
            if(!result) throw machine_robot_error("hold", result.error());
            if(purpose==JointGainProfile::SERVO_TRACKING) {
                const auto gains=robot_.set_execution_gain_profile(purpose);
                if(!gains)throw machine_robot_error("hold servo gains",gains.error());
            }
            last_output_.reset();
            feedback_time_ns_ = 0;
            background_fault_reported_ = false;
        }
    }

    static const char* reference_behavior_name(ReferenceBehavior behavior) {
        return behavior==ReferenceBehavior::NATIVE_IMPEDANCE ? "NATIVE_IMPEDANCE" : "MODEL_ADMITTANCE";
    }

    void machine_set_reference_behavior(const std::string& value) {
        machine_require_no_calibration("set_reference_behavior");
        ReferenceBehavior behavior;
        if(value=="NATIVE_IMPEDANCE")behavior=ReferenceBehavior::NATIVE_IMPEDANCE;
        else if(value=="MODEL_ADMITTANCE")behavior=ReferenceBehavior::MODEL_ADMITTANCE;
        else throw std::runtime_error("unknown reference behavior");
        std::lock_guard<std::mutex> lock(mutex_);
        const auto result=robot_.set_reference_behavior(behavior);
        if(!result)throw machine_robot_error("set_reference_behavior",result.error());
        cfg_.capability.admittance.reference_behavior=behavior;
        runtime_.clear_commands();
    }

    std::string machine_admittance_json() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& a = cfg_.capability.admittance;
        std::ostringstream out;
        out << std::setprecision(17) << "{\"enabled\":" << (a.enabled ? "true" : "false");
        out << ",\"reference_behavior\":\"" << reference_behavior_name(a.reference_behavior) << "\"";
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
        out << "},\"execution_gains\":{";
        const auto& c=robot_.get_config().ctrller;
        const std::array<std::pair<const char*,const JointImpedanceGains*>,6> presets{{
            {"rigid_hold",&c.rigid_hold_gains},{"rigid_tracking",&c.rigid_tracking_gains},
            {"compliant_hold",&c.compliant_hold_gains},{"compliant_tracking",&c.compliant_tracking_gains},
            {"compliant_drag",&c.compliant_drag_gains},{"servo_tracking",&c.servo_tracking_gains}}};
        for(std::size_t i=0;i<presets.size();++i){if(i)out<<',';out<<'"'<<presets[i].first<<"\":{\"kp\":";machine_json_joint_vector(out,presets[i].second->kp);out<<",\"kd\":";machine_json_joint_vector(out,presets[i].second->kd);out<<'}';}
        const auto& h=robot_.get_hold_compensation_cfg();
        out<<"},\"hold_compensation\":{\"enabled\":"<<(h.enabled?"true":"false")<<",\"integral_gain\":";machine_json_joint_vector(out,h.integral_gain);
        out<<",\"max_torque\":";machine_json_joint_vector(out,h.max_torque);out<<",\"max_rate\":";machine_json_joint_vector(out,h.max_rate);
        out<<",\"contact_threshold\":";machine_json_joint_vector(out,h.contact_threshold);
        out<<",\"velocity_threshold\":"<<h.velocity_threshold<<",\"error_threshold\":"<<h.error_threshold<<"}}";
        return out.str();
    }

    void machine_set_admittance(const YAML::Node& params) {
        if(!params || !params.IsMap()) throw std::runtime_error("admittance params must be an object");
        AdmittanceCapabilityCfg candidate;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE && robot_.get_state()!=RobotState::INACTIVE)throw std::runtime_error("set_admittance requires an ACTIVE or INACTIVE session");
            candidate = cfg_.capability.admittance;
        }
        auto set_vector = [&](const char* name, JointVector& target, bool allow_zero=false) {
            if(!params[name]) return;
            JointVector values = machine_node_vector(params[name], name);
            if(values.size() != cfg_.joint_names.size()) throw std::runtime_error(std::string(name) + " size does not match joint count");
            for(double value : values) if(!std::isfinite(value) || (allow_zero ? value<0.0 : value<=0.0)) throw std::runtime_error(std::string(name) + " contains invalid value");
            target = std::move(values);
            };
        set_vector("mass", candidate.controller.mass);
        set_vector("damping", candidate.controller.damping,true);
        set_vector("stiffness", candidate.controller.stiffness,true);
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

    void machine_restore_calibration_runtime(bool restore_gravity, bool restore_mode = true) {
        if(!machine_calibration_context_valid_) return;
        if(restore_gravity) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = dynamics_.set_gravity_scale(machine_calibration_original_gravity_);
            if(result) cfg_.dynamics.gravity_scale = machine_calibration_original_gravity_;
        }
        if(restore_mode) set_tuning_impedance_mode(machine_calibration_original_mode_);
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
        metadata.hardware_plugin_path = hardware_plugin_;
        metadata.hardware_plugin_file_fingerprint = model_calibration_file_fingerprint(hardware_plugin_);
        metadata.hardware_config_path=hardware_config_;
        metadata.hardware_source_config=hardware_loader_.source_config();
        metadata.hardware_effective_config=hardware_loader_.effective_config();
        metadata.hardware_serial_port_override=hardware_overrides_.serial_port;
        metadata.hardware_baudrate_override=hardware_overrides_.baudrate;
        metadata.hardware_bus_override=hardware_overrides_.bus;
        metadata.gravity_correction_fingerprint=gravity_correction_source_config_.empty()?std::string{}:source_text_fingerprint(gravity_correction_source_config_);
        metadata.core_config_path = config_path_;
        metadata.core_fingerprint = model_calibration_core_fingerprint_.empty() ? model_calibration_file_fingerprint(config_path_) : model_calibration_core_fingerprint_;
        metadata.urdf_path = cfg_.dynamics.urdf_path;
        metadata.urdf_fingerprint = model_calibration_urdf_fingerprint_.empty() ? model_calibration_file_fingerprint(cfg_.dynamics.urdf_path) : model_calibration_urdf_fingerprint_;
        metadata.resource_paths = machine_resource_paths();
        metadata.joint_names = cfg_.joint_names;
        if(actuator_info_after_connect_) {
            metadata.actuator_capabilities = actuator_info_;
            metadata.actuator_capabilities_source = cfg_.runtime.write_enabled ?
                "hardware_backend_after_successful_activate" : "mock_backend_after_successful_activate";
        }
        metadata.base_frame = cfg_.dynamics.base_frame;
        metadata.tool_frame = cfg_.dynamics.tool_frame;
        metadata.gravity = cfg_.dynamics.gravity;
        metadata.original_gravity_scale = model_calibration_original_gravity_scale_;
        metadata.mapping_pos_ratio = cfg_.mapper.pos_ratio;
        metadata.mapping_tor_ratio = cfg_.mapper.tor_ratio;
        metadata.mapping_direction = cfg_.mapper.direction;
        metadata.mapping_joint_zero_offset = cfg_.mapper.joint_zero_offset;
        metadata.mapping_actuator_zero_offset = cfg_.mapper.actuator_zero_offset;
        metadata.load_description = model_calibration_options_.load_description;
        metadata.pose_budget = model_calibration_options_.pose_budget;
        metadata.validation_fraction = model_calibration_options_.validation_fraction;
        metadata.max_com_offset_m = model_calibration_options_.max_com_offset_m;
        metadata.regularization = model_calibration_options_.regularization;
        metadata.svd_relative_threshold = model_calibration_options_.svd_relative_threshold;
        metadata.minimum_information_score = model_calibration_options_.minimum_information_score;
        metadata.identification_mode = model_calibration_options_.identification_mode;
        metadata.locked_links = model_calibration_options_.locked_links;
        for(const auto& link : model_calibration_links_cache_) metadata.identifiable_links.push_back(link.link_name);
        // Runtime model-based/adaptive gains are pose-dependent. The actual
        // commanded Kp/Kd are recorded per-frame, not guessed at task start.
        metadata.measurement_hold_kp.clear();
        metadata.measurement_hold_kd.clear();
        metadata.measurement_tracking_kp.clear();
        metadata.measurement_tracking_kd.clear();
        metadata.max_task_duration_s = 0.0; // 0: no user-configurable task deadline
        return metadata;
    }

    static std::string source_text_fingerprint(const std::string& text) {
        std::uint64_t hash=1469598103934665603ULL;
        for(unsigned char value:text){hash^=value;hash*=1099511628211ULL;}
        std::ostringstream output;output<<std::hex<<std::setw(16)<<std::setfill('0')<<hash;return output.str();
    }

    void write_model_calibration_source_snapshot(const std::filesystem::path& source) const {
        std::filesystem::create_directories(source);
        const auto write=[&](const char* name,const std::string& text) {
            std::ofstream output(source/name,std::ios::binary|std::ios::trunc);output<<text;output.close();
            if(!output)throw std::runtime_error(std::string("failed to snapshot loaded source: ")+name);
        };
        write("core.yaml",core_source_config_);write("model.urdf",urdf_source_config_);
        if(!gravity_correction_source_config_.empty())write("gravity-correction.yaml",gravity_correction_source_config_);
    }

    void model_calibration_restore_runtime_hold() {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_.clear_calibration_hold_gains();
        robot_.clear_calibration_tracking_gains();
        model_calibration_active_tracking_gains_.reset();
        model_calibration_last_replay_gain_revision_.reset();
        robot_.clear_execution_gain_profile();robot_.clear_calibration_reference_behavior();robot_.clear_calibration_model_feedforward_mode();
        if(gain_trial_original_servo_ && robot_.set_servo_tracking_gains(*gain_trial_original_servo_))gain_trial_original_servo_.reset();
        if(gain_trial_original_hold_compensation_ && robot_.set_hold_compensation_cfg(*gain_trial_original_hold_compensation_))gain_trial_original_hold_compensation_.reset();
        robot_.set_admittance_suspended(model_calibration_original_suspended_);
        if(model_calibration_original_gain_profile_) {
            if(robot_.get_state()==RobotState::ACTIVE || robot_.get_state()==RobotState::INACTIVE)
                (void)robot_.set_execution_gain_profile(*model_calibration_original_gain_profile_);
            else teach_restore_gain_profile_=*model_calibration_original_gain_profile_;
            model_calibration_original_gain_profile_.reset();
        }
        last_output_.reset();
    }

    void model_calibration_reset_task_state() {
        model_calibration_fault_interrupted_.store(false);
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
        model_calibration_preview_error_.clear();
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
        model_calibration_rejected_static_groups_ = 0;
        model_calibration_estimated_duration_s_ = 0.0;
        model_calibration_original_duration_s_ = 0.0;
        model_calibration_path_length_rad_ = 0.0;
        model_calibration_geometric_waypoints_ = 0;
        model_calibration_original_samples_ = 0;
        model_calibration_replay_rate_ = 0.0;
        model_calibration_adaptation_passes_.store(0);
        model_calibration_adaptation_revisions_.store(0);
        model_calibration_adaptation_converged_.store(false);
        model_calibration_workflow_kind_ = "gravity";
        model_calibration_gain_mode_ = "identification";
        model_calibration_teaching_wall_duration_s_ = 0.0;
        model_calibration_teaching_started_at_.reset();
        model_calibration_started_at_.reset();
        {
            std::lock_guard<std::mutex> lock(model_calibration_mutex_);
            gain_probe_kind_.clear();gain_probe_joint_.clear();gain_probe_message_.clear();
            gain_candidate_index_=0;gain_probe_elapsed_s_=0;gain_probe_duration_s_=0;
            gain_probe_amplitude_rad_=0;gain_probe_kp_=0;gain_probe_kd_=0;gain_probe_reference_.clear();
            gain_probe_stage_.clear();gain_probe_diagnostic_json_="{}";gain_probe_frequency_hz_={};
            gain_noise_.clear();gain_plans_.clear();gain_native_plans_.clear();gain_probe_origin_.clear();
            gain_seed_source_.clear();gain_seed_={};
        }
        model_calibration_update_progress(0, 0);
    }

    // General-purpose teaching and replay is independent from the calibration task state
    std::string machine_teach_replay_json() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream out;
        out << "{\"phase\":\"" << (teach_replay_phase_ == 1 ? "recording" : teach_replay_phase_ == 2 ? "ready" : teach_replay_phase_ == 3 ? "playing" : teach_replay_phase_ == 4 ? "failed" : "idle") << '"';
        out << ",\"error\":\"" << machine_json_escape(teach_replay_error_) << '"';
        out << ",\"completed\":" << teach_replay_completed_.load();
        if(teach_replay_plan_) {
            const auto& plan=*teach_replay_plan_;
            out << ",\"plan\":{\"direction\":\"" << plan.direction << "\",\"cycles\":" << plan.cycles
                << ",\"total_passes\":" << plan.total_passes;
            if(teach_replay_trajectory_&&!teach_replay_trajectory_->positions.empty()) {
                const auto& positions=teach_replay_trajectory_->positions;
                out << ",\"start\":";machine_json_joint_vector(out,plan.first_reverse?positions.back():positions.front());
                const bool final_reverse=plan.first_reverse?plan.total_passes%2!=0:plan.total_passes%2==0;
                out << ",\"end\":";machine_json_joint_vector(out,final_reverse?positions.front():positions.back());
            }
            out << '}';
        }
        out << ",\"selected_reference_behavior\":\"" << reference_behavior_name(teach_replay_behavior_) << "\",\"reference_behavior\":\"" << reference_behavior_name(teach_replay_behavior_) << "\",\"gain_profile\":\"" << to_string(teach_replay_gain_profile_) << '\"';
        out << ",\"has_trajectory\":" << (teach_replay_trajectory_ && !teach_replay_trajectory_->positions.empty()?"true":"false")
            << ",\"fault_interrupted\":" << (teach_replay_fault_interrupted_?"true":"false");
        out << ",\"capabilities\":{\"contact_first_capture\":true,\"contact_cycle\":true,\"endpoint_dwell\":true}";
        if(teach_replay_trajectory_ && !teach_replay_trajectory_->positions.empty()) {
            const auto& t = *teach_replay_trajectory_;
            out << ",\"samples\":" << t.positions.size() << ",\"duration_s\":" << (t.positions.size()-1)*t.sample_dt;
            out << ",\"start\":";machine_json_joint_vector(out,t.positions.front());
            out << ",\"end\":";machine_json_joint_vector(out,t.positions.back());
        }
        out << '}';return out.str();
    }

    void teach_restore_controls_locked() {
        if(!teach_controls_active_)return;
        robot_.clear_execution_gain_profile();
        robot_.clear_calibration_reference_behavior();
        robot_.clear_calibration_model_feedforward_mode();
        robot_.set_admittance_suspended(teach_original_suspended_);
        if(robot_.get_state()==RobotState::ACTIVE || robot_.get_state()==RobotState::INACTIVE)
            (void)robot_.set_execution_gain_profile(teach_original_gain_profile_);
        else teach_restore_gain_profile_=teach_original_gain_profile_;
        teach_controls_active_=false;direct_drag_controls_=false;
        // Do not mix the restored current mode/strategy with a cached cycle
        // from the temporary DRAG/MDK executor in the next status response
        last_output_.reset();feedback_time_ns_=0;
    }
    void teach_prepare_controls_locked(ReferenceBehavior behavior,bool gravity_only) {
        if(teach_controls_active_)throw std::runtime_error("teaching controls are already owned");
        teach_original_suspended_=robot_.is_admittance_suspended();teach_original_gain_profile_=robot_.get_gain_profile()==JointGainProfile::DRAG?JointGainProfile::RIGID:robot_.get_gain_profile();teach_controls_active_=true;
        const auto selected=robot_.set_calibration_reference_behavior(behavior);
        if(!selected) {teach_restore_controls_locked();throw machine_robot_error("teaching reference behavior",selected.error());}
        if(gravity_only) {
            const auto feed=robot_.set_calibration_model_feedforward_mode(ModelFeedforwardMode::GRAVITY);
            if(!feed){teach_restore_controls_locked();throw machine_robot_error("teaching gravity feedforward",feed.error());}
        }
        robot_.set_admittance_suspended(false);
    }

    void require_drag_teaching_ready_locked(std::unique_lock<std::mutex>& lock) {
        if(robot_.get_state()!=RobotState::ACTIVE || !dynamics_.is_updated())
            throw std::runtime_error("drag teaching requires ACTIVE robot and valid dynamics feedback");
        const auto& scales=dynamics_.get_gravity_scale();
        if(!std::any_of(scales.begin(),scales.end(),[](double value){return std::isfinite(value) && value>0;}))
            throw std::runtime_error("drag teaching requires nonzero gravity compensation");
        const auto& gains=robot_.get_config().ctrller.compliant_drag_gains;
        if(gains.kd.size()!=cfg_.joint_names.size() ||
           std::any_of(gains.kd.begin(),gains.kd.end(),[](double value){return !std::isfinite(value) || value<=0;}))
            throw std::runtime_error("drag teaching requires positive damping");
        // Hold/mode acknowledgements invalidate the displayed cycle until the
        // worker publishes its next sample. Wait for that sample without making
        // the user repeat an otherwise valid operation.
        if(!last_output_ || !feedback_time_ns_)
            cycle_cv_.wait_for(lock,std::chrono::milliseconds(150),[this]{return robot_.get_state()!=RobotState::ACTIVE || (last_output_ && feedback_time_ns_);});
        const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
        if(robot_.get_state()!=RobotState::ACTIVE || !last_output_ || !feedback_time_ns_ ||
           now_ns<feedback_time_ns_ || now_ns-feedback_time_ns_>150000000)
            throw std::runtime_error("drag teaching requires fresh hardware feedback");
    }

    void machine_teach_record_begin() {
        machine_require_no_calibration("teach record");
        if(teach_replay_phase_.load() == 1 || teach_replay_phase_.load() == 3)
            throw std::runtime_error("a teaching or playback operation is active");
        if(teach_replay_worker_.joinable()) teach_replay_worker_.join();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            require_drag_teaching_ready_locked(lock);
            const auto previous_purpose=direct_drag_controls_?teach_original_gain_profile_:robot_.get_gain_profile();
            if(direct_drag_controls_){teach_restore_controls_locked();direct_drag_controls_=false;}
            teach_prepare_controls_locked(ReferenceBehavior::NATIVE_IMPEDANCE,true);
            teach_original_gain_profile_=previous_purpose==JointGainProfile::DRAG?JointGainProfile::RIGID:previous_purpose;
            const auto result=runtime_.set_impedance_mode(robot_, JointImpedanceMode::COMPLIANT_DRAG);
            if(!result){teach_restore_controls_locked();throw machine_robot_error("teach drag",result.error());}
            clear_command_sources();teach_replay_fault_interrupted_=false;
            teach_replay_behavior_=ReferenceBehavior::NATIVE_IMPEDANCE;teach_replay_gain_profile_=JointGainProfile::DRAG;
            teach_replay_plan_.reset();teach_replay_trajectory_.reset(); teach_replay_record_start_=last_output_->joint_state.pos; teach_replay_error_.clear();
            teach_replay_record_stop_.store(false);
            teach_replay_phase_.store(1);
            teach_replay_completed_.store(0);
        }
        teach_replay_worker_=std::thread([this] {
            auto recorded=collect_friction_drag_trajectory_until_stop(teach_replay_record_stop_);
            std::lock_guard<std::mutex> lock(mutex_);
            if(recorded && !teach_replay_record_start_.empty()) {
                // Preserve the physical pose at record_begin as the contact endpoint
                // The first scheduled recorder sample may arrive after dragging starts
                recorded->positions.insert(recorded->positions.begin(),teach_replay_record_start_);
            }
            teach_replay_record_start_.clear();
            const bool interrupted=teach_replay_fault_interrupted_ || robot_.get_state()!=RobotState::ACTIVE;
            if(recorded && friction_trajectory_inside_safe_replay_range(*recorded)) {
                teach_replay_trajectory_=std::move(*recorded);
                teach_replay_phase_.store(interrupted?4:2);
            } else {
                if(teach_replay_error_.empty())teach_replay_error_="trajectory is too short or exceeds URDF joint safety margins";
                teach_replay_phase_.store(4);
            }
            if(interrupted) {
                if(teach_replay_error_.empty())teach_replay_error_="teaching interrupted before completion";
                teach_restore_controls_locked();
            }
        });
    }

    void machine_teach_record_stop() {
        if(teach_replay_phase_.load()!=1 && teach_replay_phase_.load()!=4)
            throw std::runtime_error("not recording a trajectory");
        // Capture the latest physical joint position at the marked recording endpoint
        // before waiting for the recording worker to finish
        JointVector marked_endpoint;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(last_output_ && robot_.get_state()==RobotState::ACTIVE)
                marked_endpoint=last_output_->joint_state.pos;
        }
        teach_replay_record_stop_.store(true);cycle_cv_.notify_all();
        if(teach_replay_worker_.joinable())teach_replay_worker_.join();
        std::lock_guard<std::mutex> lock(mutex_);
        if(teach_replay_phase_.load()==2 && teach_replay_trajectory_ &&
                !marked_endpoint.empty() &&
                marked_endpoint.size()==teach_replay_trajectory_->positions.back().size()) {
            teach_replay_trajectory_->positions.push_back(std::move(marked_endpoint));
            if(!friction_trajectory_inside_safe_replay_range(*teach_replay_trajectory_)) {
                teach_replay_trajectory_.reset();
                teach_replay_error_="recorded endpoint violates joint safety margins";
                teach_replay_phase_.store(4);
            }
        }
        if(robot_.get_state()==RobotState::ACTIVE) {
            const auto hold=runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD,Robot::Clock::now(),HoldTargetPolicy::CAPTURE_MEASURED);
            if(!hold){teach_restore_controls_locked();throw machine_robot_error("teach recording hold",hold.error());}
        }
        teach_restore_controls_locked();
    }

    void machine_teach_save(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(teach_replay_phase_.load()!=2 || !teach_replay_trajectory_)
            throw std::runtime_error("no completed teaching trajectory to save");
        const std::filesystem::path path(filename);
        if(path.extension()!=".csv" || !std::filesystem::is_directory(path.parent_path()))
            throw std::runtime_error("invalid output path");
        std::ofstream out(path,std::ios::trunc);
        if(!out)throw std::runtime_error("unable to write trajectory");
        out << std::setprecision(17) << "sample_dt," << teach_replay_trajectory_->sample_dt << '\n';
        for(std::size_t j=0;j<cfg_.joint_names.size();++j){if(j)out << ',';out << cfg_.joint_names[j];}out << '\n';
        for(const auto& row:teach_replay_trajectory_->positions){
            for(std::size_t j=0;j<row.size();++j){if(j)out << ',';out << row[j];}out << '\n';
        }
        if(!out)throw std::runtime_error("failed to write complete trajectory");
    }

    void machine_teach_load(const std::string& filename) {
        if(teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3)
            throw std::runtime_error("stop active recording or replay before loading");
        if(teach_replay_worker_.joinable())teach_replay_worker_.join();
        std::ifstream file(filename);if(!file)throw std::runtime_error("trajectory file unavailable");
        std::string line;std::getline(file,line);
        if(line.rfind("sample_dt,",0)!=0)throw std::runtime_error("invalid trajectory sample interval header");
        FrictionCalibrationTrajectory trajectory;
        trajectory.sample_dt=std::stod(line.substr(10));
        if(!std::isfinite(trajectory.sample_dt)||trajectory.sample_dt<0.001||trajectory.sample_dt>1.0)
            throw std::runtime_error("trajectory sample interval is outside supported limits");
        std::getline(file,line);std::ostringstream header;
        for(std::size_t i=0;i<cfg_.joint_names.size();++i){if(i)header << ',';header<<cfg_.joint_names[i];}
        if(line!=header.str())throw std::runtime_error("trajectory joint names or order differ from the loaded Profile");
        while(std::getline(file,line)) {
            if(line.empty())continue;
            JointVector row;std::istringstream in(line);std::string item;
            while(std::getline(in,item,',')){
                std::size_t used=0;double value=std::stod(item,&used);
                if(used!=item.size()||!std::isfinite(value))throw std::runtime_error("invalid trajectory position");
                row.push_back(value);
            }
            if(row.size()!=cfg_.joint_names.size())throw std::runtime_error("trajectory joint column count mismatch");
            trajectory.positions.push_back(std::move(row));
            if(trajectory.positions.size()>300000)throw std::runtime_error("trajectory has too many samples");
        }
        if(trajectory.positions.size()<20 || !friction_trajectory_inside_safe_replay_range(trajectory))
            throw std::runtime_error("trajectory is too short or violates safety limits");
        std::lock_guard<std::mutex> lock(mutex_);
        teach_replay_plan_.reset();teach_replay_completed_.store(0);teach_replay_trajectory_=std::move(trajectory);teach_replay_phase_.store(2);teach_replay_error_.clear();teach_replay_fault_interrupted_=false;
    }

    void machine_teach_replay_stop() {
        if(teach_replay_phase_.load()!=3 && teach_replay_phase_.load()!=4)throw std::runtime_error("no replay task is active");
        teach_replay_stop_.store(true);model_calibration_cancel_.store(true);
        cycle_cv_.notify_all();
        if(teach_replay_worker_.joinable())teach_replay_worker_.join();
        std::lock_guard<std::mutex> lock(mutex_);
        if(teach_replay_phase_.load()==3)teach_replay_phase_.store(2);
        // The joined replay worker has already entered HOLD and restored the
        // original purpose. A second mode write here would clear that purpose
        clear_command_sources();
        teach_restore_controls_locked();
        model_calibration_active_tracking_gains_.reset();model_calibration_cancel_.store(false);
    }

    void machine_teach_play(const std::string& direction,double rate,int cycles,bool released,const std::string& tracking_mode,double endpoint_dwell_s=0.0,
                            const std::string& reference_behavior="",const std::string& gain_profile="") {
        if(tracking_mode!="RIGID_TRACKING" && tracking_mode!="COMPLIANT_TRACKING")
            throw std::runtime_error("replay mode must be RIGID_TRACKING or COMPLIANT_TRACKING");
        ReferenceBehavior behavior;
        if(reference_behavior.empty())behavior=cfg_.capability.admittance.enabled?cfg_.capability.admittance.reference_behavior:ReferenceBehavior::NATIVE_IMPEDANCE;
        else if(reference_behavior=="NATIVE_IMPEDANCE")behavior=ReferenceBehavior::NATIVE_IMPEDANCE;
        else if(reference_behavior=="MODEL_ADMITTANCE")behavior=ReferenceBehavior::MODEL_ADMITTANCE;
        else throw std::runtime_error("unknown replay reference behavior");
        const auto profile=gain_profile.empty()?(behavior==ReferenceBehavior::MODEL_ADMITTANCE?JointGainProfile::SERVO_TRACKING:
            tracking_mode=="COMPLIANT_TRACKING"?JointGainProfile::COMPLIANT:JointGainProfile::RIGID):machine_parse_gain_profile(gain_profile);
        if(profile==JointGainProfile::DRAG)throw std::runtime_error("DRAG gain profile cannot replay a trajectory");
        if(behavior==ReferenceBehavior::MODEL_ADMITTANCE && !cfg_.capability.admittance.enabled)
            throw std::runtime_error("MODEL_ADMITTANCE replay requires the enabled interaction capability");
        if(behavior==ReferenceBehavior::MODEL_ADMITTANCE && profile!=JointGainProfile::SERVO_TRACKING)
            throw std::runtime_error("MODEL_ADMITTANCE replay requires SERVO_TRACKING gains");
        const auto replay_mode=profile==JointGainProfile::COMPLIANT ? JointImpedanceMode::COMPLIANT_TRACKING : JointImpedanceMode::RIGID_TRACKING;
        if(!released)throw std::runtime_error("workspace and released-hand confirmation required");
        if(direction!="forward"&&direction!="reverse"&&direction!="cycle"&&direction!="return"&&direction!="contact_cycle")
            throw std::runtime_error("unknown replay direction");
        if(!std::isfinite(rate)||rate<0.05||rate>1.0||cycles<1||cycles>100||
           !std::isfinite(endpoint_dwell_s)||endpoint_dwell_s<0.0||endpoint_dwell_s>30.0)
            throw std::runtime_error("invalid replay speed or repeat count");
        if((teach_replay_phase_.load()!=2 && teach_replay_phase_.load()!=4) || !teach_replay_trajectory_)
            throw std::runtime_error("load or record a trajectory before replay");
        machine_require_no_calibration("teach replay");
        if(teach_replay_worker_.joinable())teach_replay_worker_.join();
        FrictionCalibrationTrajectory planned;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE || !last_output_ || !last_output_->joint_state.pos.size())
                throw std::runtime_error("replay requires valid ACTIVE feedback");
            if(robot_.get_impedance_mode()==JointImpedanceMode::COMPLIANT_DRAG)
                throw std::runtime_error("leave compliant drag before replay");
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            if(!feedback_time_ns_ || now_ns<feedback_time_ns_ || now_ns-feedback_time_ns_>150000000)
                throw std::runtime_error("replay requires fresh control feedback");
            planned=*teach_replay_trajectory_;
        }
        // Re-time both directions using the actual URDF/Safety limits
        auto retimed=serial_arm::calibration_retime::plan(planned.positions,
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,cfg_.runtime.ctrl_frequency_hz,0.003,{});
        planned.positions=std::move(retimed.positions);planned.sample_dt=retimed.sample_dt;
        if(!friction_trajectory_inside_safe_replay_range(planned))
            throw std::runtime_error("re-timed trajectory violates joint margins");
        FrictionCalibrationTrajectory interpolated;
        interpolated.sample_dt=planned.sample_dt/4.0;
        for(std::size_t k=0;k+1<planned.positions.size();++k)
            for(int quarter=0;quarter<4;++quarter)
                interpolated.positions.push_back(serial_arm::calibration_retime::interpolate(
                    planned.positions,static_cast<double>(k)+0.25*quarter,planned.sample_dt,1.0).position);
        interpolated.positions.push_back(planned.positions.back());
        if(!friction_trajectory_inside_safe_replay_range(interpolated))
            throw std::runtime_error("interpolated playback would exceed URDF joint safety margins");
        const bool reverse=(direction=="reverse"||direction=="return");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& expected=(reverse || direction=="contact_cycle")?planned.positions.back():planned.positions.front();
            const auto& state=last_output_->joint_state;
            if(state.pos.size()!=expected.size())throw std::runtime_error("joint feedback size mismatch");
            const auto& types=dynamics_.get_info().joint_types;
            for(std::size_t i=0;i<expected.size();++i) {
                const bool linear=i<types.size()&&types[i]=="prismatic";
                const double position_limit=linear?(direction=="contact_cycle"?0.0005:0.001):(direction=="contact_cycle"?0.03:0.08);
                const double speed_limit=linear?0.001:(direction=="contact_cycle"?0.03:0.05);
                if(!std::isfinite(state.pos[i])||!std::isfinite(state.vel[i])||
                    std::abs(state.pos[i]-expected[i])>position_limit||std::abs(state.vel[i])>speed_limit)
                    throw std::runtime_error("robot is not aligned with the selected trajectory endpoint at "+cfg_.joint_names[i]+
                        "; return to the corresponding safe start pose before playback");
            }
            teach_prepare_controls_locked(behavior,false);
            const auto mode=runtime_.set_impedance_mode(robot_, replay_mode);
            if(!mode){teach_restore_controls_locked();throw machine_robot_error("teach replay tracking",mode.error());}
            const auto gains=robot_.set_execution_gain_profile(profile);
            if(!gains){teach_restore_controls_locked();throw machine_robot_error("teach replay gains",gains.error());}
            clear_command_sources();teach_replay_behavior_=behavior;teach_replay_gain_profile_=profile;
            model_calibration_cancel_.store(false);teach_replay_stop_.store(false);
            model_calibration_error_.clear();
            model_calibration_active_tracking_gains_=profile==JointGainProfile::SERVO_TRACKING ? robot_.get_config().ctrller.servo_tracking_gains :
                profile==JointGainProfile::COMPLIANT ? cfg_.ctrller.compliant_tracking_gains : cfg_.ctrller.rigid_tracking_gains;
            teach_replay_plan_=teach_replay_plan(direction,cycles);
            teach_replay_phase_.store(3);teach_replay_error_.clear();teach_replay_completed_.store(0);teach_replay_fault_interrupted_=false;
        }
        teach_replay_worker_=std::thread([this,path=std::move(planned),direction,rate,cycles,profile,endpoint_dwell_s]() {
            bool failed=false;
            const auto plan=teach_replay_plan(direction,cycles);
            const int passes=plan.total_passes;
            const bool first_reverse=plan.first_reverse;
            for(int pass=0;pass<passes && !teach_replay_stop_.load();++pass){
                const bool reverse=first_reverse ? pass%2==0 : pass%2!=0;
                auto ordered=path.positions;
                if(reverse)std::reverse(ordered.begin(),ordered.end());
                const auto count=ordered.size();
                const double step=rate/(cfg_.runtime.ctrl_frequency_hz*path.sample_dt);
                if(!std::isfinite(step)||step<=0.0){failed=true;break;}
                double progress=0.0;
                while(progress<static_cast<double>(count-1) && !teach_replay_stop_.load()){
                    progress=std::min(progress+step,static_cast<double>(count-1));
                    const auto sample=serial_arm::calibration_retime::interpolate(ordered,progress,path.sample_dt,rate);
                    if(!model_calibration_send_reference(sample.position,sample.velocity,
                            false,nullptr,nullptr,true,profile==JointGainProfile::COMPLIANT)){
                        failed=!teach_replay_stop_.load();break;
                    }
                }
                if(failed)break;
                // The dedicated contact-cycle starts at the safe recorded endpoint, plays
                // backward to the recorded contact start, dwells, then withdraws forward
                // Legacy forward-first playback keeps its original endpoint dwell behavior
                const bool at_contact=(direction=="contact_cycle") ? reverse : !reverse;
                if(at_contact && endpoint_dwell_s>0.0 && !teach_replay_stop_.load()) {
                    const auto hold_end=std::chrono::steady_clock::now()+std::chrono::duration<double>(endpoint_dwell_s);
                    const JointVector target=ordered.back();
                    const JointVector zero_velocity(target.size(),0.0);
                    while(std::chrono::steady_clock::now()<hold_end && !teach_replay_stop_.load()) {
                        if(!model_calibration_send_reference(target,zero_velocity,
                                false,nullptr,nullptr,true,profile==JointGainProfile::COMPLIANT)) {
                            failed=!teach_replay_stop_.load();break;
                        }
                    }
                }
                if(failed)break;
                teach_replay_completed_.store(pass+1);
            }
            std::lock_guard<std::mutex> lock(mutex_);
            failed=failed || robot_.get_state()!=RobotState::ACTIVE || teach_replay_fault_interrupted_;
            if(robot_.get_state()==RobotState::ACTIVE){
                clear_command_sources();
                const auto held=runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD);
                if(!held && !failed){failed=true;teach_replay_error_="unable to enter safe hold after replay";}
            }
            teach_restore_controls_locked();
            if(failed){
                if(teach_replay_error_.empty())teach_replay_error_="trajectory replay stopped before completion";
                teach_replay_phase_.store(4);
            }else {teach_replay_phase_.store(2);teach_replay_fault_interrupted_=false;}
            model_calibration_active_tracking_gains_.reset();
        });
    }

    void machine_model_calibration_teach_begin(const YAML::Node& params) {
        if(teach_replay_phase_.load()==1||teach_replay_phase_.load()==3)throw std::runtime_error("finish teaching playback before calibration");
        if(machine_calibration_kind_ != "none" || model_calibration_task_active()) {
            throw std::runtime_error("another calibration task is active");
        }
        if(dynamics_.has_gravity_first_moment_override()) {
            throw std::runtime_error("restore the active gravity correction before starting a new model calibration task");
        }
        {
            std::unique_lock<std::mutex> lock(mutex_);
            require_drag_teaching_ready_locked(lock);
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
        if(params && params["mode"]) model_calibration_options_.identification_mode = params["mode"].as<std::string>();
        if(params && params["load_description"]) model_calibration_options_.load_description = params["load_description"].as<std::string>();
        if(params && params["locked_links"]) model_calibration_options_.locked_links = params["locked_links"].as<std::vector<std::string>>();
        if(model_calibration_options_.identification_mode != "global" && model_calibration_options_.identification_mode != "local") throw std::runtime_error("invalid identification mode");
        if(model_calibration_options_.identification_mode == "global") model_calibration_options_.locked_links.clear();
        if(model_calibration_links_cache_.empty()) throw std::runtime_error("no identifiable gravity links");
        std::set<std::string> choices;
        for(const auto& item : model_calibration_links_cache_) choices.insert(item.link_name);
        std::set<std::string> unique;
        for(const auto& name : model_calibration_options_.locked_links) {
            if(!choices.count(name) || !unique.insert(name).second) throw std::runtime_error("unknown or duplicate locked gravity link: " + name);
        }
        if(choices.size() <= unique.size()) throw std::runtime_error("at least one identifiable gravity link must remain unlocked");

        {
            std::unique_lock<std::mutex> lock(mutex_);
            require_drag_teaching_ready_locked(lock);
            if(direct_drag_controls_){teach_restore_controls_locked();direct_drag_controls_=false;}
            model_calibration_original_mode_ = robot_.get_impedance_mode();
            model_calibration_original_gain_profile_=robot_.get_gain_profile()==JointGainProfile::DRAG?JointGainProfile::RIGID:robot_.get_gain_profile();
            model_calibration_original_feedforward_ = robot_.get_model_feedforward_mode();
            model_calibration_original_admittance_ = cfg_.capability.admittance;
            model_calibration_original_gravity_scale_ = dynamics_.get_gravity_scale();
            model_calibration_original_suspended_ = robot_.is_admittance_suspended();
            const auto native=robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE);
            if(!native)throw machine_robot_error("model teaching native strategy",native.error());
            const auto gravity=robot_.set_calibration_model_feedforward_mode(ModelFeedforwardMode::GRAVITY);
            if(!gravity){robot_.clear_calibration_reference_behavior();throw machine_robot_error("model teaching gravity feedforward",gravity.error());}
            robot_.set_admittance_suspended(true);
            last_output_.reset();
        }
        model_calibration_task_id_ = machine_model_calibration_task_id();
        model_calibration_core_fingerprint_ = source_text_fingerprint(core_source_config_);
        model_calibration_urdf_fingerprint_ = source_text_fingerprint(urdf_source_config_);
        const char* storage = std::getenv("SERIAL_ARM_PROFILE_STORAGE");
        const std::filesystem::path root = storage && *storage ? std::filesystem::path(storage) / "model-calibration" : std::filesystem::current_path() / ".install" / "model-calibration";
        model_calibration_directory_ = (root / model_calibration_task_id_).string();
        const auto recorder = model_calibration_recorder_.start(model_calibration_directory_, model_calibration_metadata());
        if(!recorder) {
            model_calibration_error_ = recorder.error();
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw std::runtime_error(model_calibration_error_);
        }
        try {
            std::error_code source_error;
            const std::filesystem::path source_dir = std::filesystem::path(model_calibration_directory_) / "source";
            std::filesystem::create_directories(source_dir, source_error);
            if(source_error) throw std::runtime_error("failed to create model calibration source snapshot directory: " + source_error.message());
            write_model_calibration_source_snapshot(source_dir);
        }
        catch(...) {
            model_calibration_recorder_.stop();
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw;
        }
        model_calibration_set_phase(ModelCalibrationPhase::TEACHING);
        model_calibration_teaching_started_at_ = Robot::Clock::now();
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)) {
            model_calibration_recorder_.stop();
            model_calibration_restore_runtime_hold();
            model_calibration_error_ = "failed to enter COMPLIANT_DRAG";
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_write_result_file();
            throw std::runtime_error(model_calibration_error_);
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
            const auto robot_state = machine_robot_state();
            const bool interrupted = robot_state != RobotState::ACTIVE || model_calibration_fault_interrupted_.load();
            if(interrupted) {
                if(captured && captured->positions.size() >= 20) {
                    const auto saved = model_calibration_recorder_.write_trajectory(
                        captured->positions, captured->sample_dt);
                    if(!saved) std::cerr << "[示教故障] 保存中断轨迹失败: " << saved.error() << '\n';
                    else {
                        std::ofstream marker(std::filesystem::path(model_calibration_directory_) / "interrupted.txt");
                        if(marker) marker << "Teaching interrupted before normal stop. Saved data is a partial demonstration, not a calibrated model.\n";
                        std::cout << "[示教中断] 已保留中断前轨迹，必须重新安全检查后才能导入；不会自动回放\n";
                    }
                }
                model_calibration_recorder_.stop();
            }
            model_calibration_trajectory_ = std::move(captured);
            if(interrupted) {
                // A hardware-originated state change must also restore the
                // temporary recording controller settings without requiring
                // a second GUI request or ever clearing a latched FAULT.
                if(model_calibration_phase_.load() == ModelCalibrationPhase::TEACHING) {
                    if(robot_state != RobotState::FAULT && !model_calibration_cancel_.load() &&
                        !model_calibration_fault_interrupted_.load()) {
                        std::lock_guard<std::mutex> guard(model_calibration_mutex_);
                        if(model_calibration_error_.empty())
                            model_calibration_error_ = "teaching stopped because the robot is no longer ACTIVE";
                    }
                    model_calibration_set_phase(robot_state == RobotState::FAULT || model_calibration_fault_interrupted_.load() ?
                        ModelCalibrationPhase::FAILED : ModelCalibrationPhase::CANCELLED);
                }
                model_calibration_recorder_.stop();
                model_calibration_restore_runtime_hold();
                model_calibration_write_result_file();
            }
            });
    }

    std::vector<ModelCalibrationPoseTarget> model_calibration_remap_targets(
        const FrictionCalibrationTrajectory& retimed,
        const std::vector<JointVector>& stop_positions,
        const std::vector<ModelCalibrationPoseTarget>& targets) const {
        if(stop_positions.size() != targets.size()) throw std::runtime_error("invalid calibration stops");
        std::vector<ModelCalibrationPoseTarget> result = targets;
        std::size_t previous = 0;
        for(std::size_t i = 0; i < targets.size(); ++i) {
            std::size_t best = previous + 1;
            double best_distance = std::numeric_limits<double>::infinity();
            // Targets are ordered in their original demonstrated path direction
            // so do not mistake a later revisited configuration for this stop
            for(std::size_t k = previous + 1; k + 1 < retimed.positions.size(); ++k) {
                double distance = 0.0;
                for(std::size_t j = 0; j < stop_positions[i].size(); ++j)
                    distance = std::max(distance, std::abs(stop_positions[i][j] - retimed.positions[k][j]));
                if(distance < best_distance) { best_distance = distance; best = k; }
            }
            if(best_distance > 0.025 || best <= previous || best + 1 >= retimed.positions.size())
                throw std::runtime_error("unable to preserve calibration stop on retimed path");
            result[i].trajectory_index = best;
            previous = best;
        }
        return result;
    }

    std::vector<ModelCalibrationPoseTarget> model_calibration_select_pose_targets(const FrictionCalibrationTrajectory& trajectory) {
        struct Candidate { std::size_t index; JointVector q; Eigen::MatrixXd feature; double norm; };
        std::vector<Candidate> candidates;
        if(trajectory.positions.empty()) return { };
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
            if(model_calibration_options_.identification_mode == "local") {
                for(std::size_t link = 0; link < regression->links.size(); ++link)
                    if(std::find(model_calibration_options_.locked_links.begin(), model_calibration_options_.locked_links.end(), regression->links[link].link_name) != model_calibration_options_.locked_links.end())
                        scaled.block(0, static_cast<Eigen::Index>(3 * link), scaled.rows(), 3).setZero();
            }
            for(std::size_t link = 0; link < regression->links.size(); ++link) {
                const double parameter_scale = std::max(1.0e-4, regression->links[link].mass * 0.05); // Geometry reference for information scoring; NOT a COM limit
                scaled.block(0, static_cast<Eigen::Index>(3 * link), scaled.rows(), 3) *= parameter_scale;
            }
            Candidate item{ index, q, scaled, scaled.norm() };
            if(std::isfinite(item.norm) && item.norm >= model_calibration_options_.minimum_information_score) candidates.push_back(std::move(item));
        }
        if(candidates.size() < model_calibration_options_.minimum_training_groups + 1) return { };
        const std::size_t desired = std::min(model_calibration_options_.pose_budget, candidates.size());
        std::vector<std::size_t> chosen;
        chosen.reserve(desired);
        // Greedy D-optimal experimental design. A single stationary pose
        // contributes an entire multi-joint regression BLOCK, rather than one
        // flattened vector. Accumulating G(q)^T G(q) correctly captures the
        // information on every observable gravity-parameter combination.
        const Eigen::Index columns = candidates.front().feature.cols();
        double typical_energy = 0.0;
        for(const auto& pose : candidates) typical_energy += pose.feature.squaredNorm();
        typical_energy /= static_cast<double>(candidates.size()) * static_cast<double>(columns);
        const double regularizer = std::max(1e-12, 1e-2 * typical_energy);
        Eigen::MatrixXd information = regularizer * Eigen::MatrixXd::Identity(columns, columns);
        auto information_logdet = [](const Eigen::MatrixXd& gram) {
            Eigen::LLT<Eigen::MatrixXd> chol(gram);
            if(chol.info() != Eigen::Success) return -std::numeric_limits<double>::infinity();
            const Eigen::MatrixXd lower = chol.matrixL();
            double value = 0.0;
            for(Eigen::Index j = 0; j < lower.rows(); ++j) value += 2.0 * std::log(lower(j, j));
            return value;
            };
        double previous_logdet = information_logdet(information);
        while(chosen.size() < desired) {
            double best_gain = -std::numeric_limits<double>::infinity();
            std::size_t best_index = candidates.size();
            for(std::size_t i = 0; i < candidates.size(); ++i) {
                if(std::find(chosen.begin(), chosen.end(), i) != chosen.end()) continue;
                const Eigen::MatrixXd updated = information
                    + candidates[i].feature.transpose() * candidates[i].feature;
                const double gain = information_logdet(updated) - previous_logdet;
                if(std::isfinite(gain) && gain > best_gain) { best_gain = gain; best_index = i; }
            }
            if(best_index == candidates.size() || best_gain < 1e-7) break;
            chosen.push_back(best_index);
            information.noalias() += candidates[best_index].feature.transpose() * candidates[best_index].feature;
            previous_logdet += best_gain;
        }
        if(chosen.size() < model_calibration_options_.minimum_training_groups + 1) return { };
        std::sort(chosen.begin(), chosen.end(), [&](std::size_t a, std::size_t b) { return candidates[a].index < candidates[b].index; });
        const std::size_t validation_count = std::max<std::size_t>(1, static_cast<std::size_t>(std::round(chosen.size() * model_calibration_options_.validation_fraction)));
        std::vector<ModelCalibrationPoseTarget> result;
        result.reserve(chosen.size());
        for(std::size_t i = 0; i < chosen.size(); ++i) {
            const bool validation = validation_count > 0 && ((i + 1) * validation_count / chosen.size() > i * validation_count / chosen.size());
            result.push_back(ModelCalibrationPoseTarget{ candidates[chosen[i]].index, i + 1, validation, candidates[chosen[i]].norm });
        }
        if(static_cast<std::size_t>(std::count_if(result.begin(), result.end(), [](const auto& x) {return !x.validation; })) < model_calibration_options_.minimum_training_groups) {
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
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i)
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
        if(params["mode"]) model_calibration_options_.identification_mode = params["mode"].as<std::string>();
        if(params["load_description"]) model_calibration_options_.load_description = params["load_description"].as<std::string>();
        if(params["locked_links"]) model_calibration_options_.locked_links = params["locked_links"].as<std::vector<std::string>>();
        if(model_calibration_options_.identification_mode != "global" && model_calibration_options_.identification_mode != "local") throw std::runtime_error("invalid identification mode");
        if(model_calibration_options_.identification_mode == "global") model_calibration_options_.locked_links.clear();
        if(model_calibration_links_cache_.empty()) throw std::runtime_error("no identifiable gravity links");
        std::set<std::string> choices;
        for(const auto& item : model_calibration_links_cache_) choices.insert(item.link_name);
        std::set<std::string> unique;
        for(const auto& name : model_calibration_options_.locked_links) {
            if(!choices.count(name) || !unique.insert(name).second) throw std::runtime_error("unknown or duplicate locked gravity link: " + name);
        }
        if(choices.size() <= unique.size()) throw std::runtime_error("at least one identifiable gravity link must remain unlocked");
        // Protect even against metadata with manually edited joint calibration;
        // the entire Core fingerprint above includes the joint mapping.
        const auto trajectory_path = std::filesystem::is_regular_file(directory / "trajectory.csv") ?
            directory / "trajectory.csv" : directory / "trajectory.checkpoint.csv";
        // Checkpoints are partial, unverified teaching recordings. Apply all
        // fingerprint, joint-order, finite/range and retiming checks below.
        std::ifstream csv(trajectory_path);
        if(!csv) throw std::runtime_error("neither trajectory.csv nor trajectory.checkpoint.csv exists");
        std::string line, field;
        if(!std::getline(csv, line)) throw std::runtime_error("trajectory sample_dt header is missing");
        std::istringstream head(line);
        std::getline(head, field, ',');
        if(field != "sample_dt" || !std::getline(head, field) || field.empty())
            throw std::runtime_error("invalid trajectory sample_dt header");
        FrictionCalibrationTrajectory candidate;
        candidate.sample_dt = std::stod(field);
        if(!std::isfinite(candidate.sample_dt) || candidate.sample_dt <= 0 || candidate.sample_dt > 0.5)
            throw std::runtime_error("invalid trajectory sample interval");
        if(!std::getline(csv, line)) throw std::runtime_error("trajectory columns are missing");
        std::ostringstream expected_header;
        expected_header << "index";
        for(const auto& name : cfg_.joint_names) expected_header << ',' << name;
        if(line != expected_header.str()) throw std::runtime_error("trajectory joint names do not match recorded metadata");
        while(std::getline(csv, line)) {
            if(line.empty()) throw std::runtime_error("empty row in trajectory.csv");
            std::istringstream row(line);
            if(!std::getline(row, field, ',') || field != std::to_string(candidate.positions.size()))
                throw std::runtime_error("trajectory frame index is invalid");
            JointVector q;
            for(std::size_t j = 0; j < cfg_.joint_names.size(); ++j) {
                if(!std::getline(row, field, ',') || field.empty()) throw std::runtime_error("trajectory joint column is missing");
                std::size_t parsed = 0;
                const double value = std::stod(field, &parsed);
                if(parsed != field.size() || !std::isfinite(value)) throw std::runtime_error("non-finite or invalid trajectory joint position");
                q.push_back(value);
            }
            if(std::getline(row, field, ',')) throw std::runtime_error("trajectory row contains extra joint columns");
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
            if(last_output_ && robot_.get_state() == RobotState::ACTIVE && feedback_time_ns_ > 0) {
                const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                const double age_ms = now_ns >= feedback_time_ns_ ? static_cast<double>(now_ns - feedback_time_ns_) / 1.0e6 : std::numeric_limits<double>::infinity();
                endpoint_choice = serial_arm::calibration_recovery::select_endpoint(
                    last_output_->joint_state.pos, last_output_->joint_state.vel,
                    candidate.positions.front(), candidate.positions.back(), age_ms, dynamics_.get_info().joint_types);
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
        const double recorded_duration = candidate.sample_dt * static_cast<double>(candidate.positions.size() - 1);
        const auto selected = model_calibration_select_pose_targets(candidate);
        if(selected.size() < model_calibration_options_.minimum_training_groups + 1)
            throw std::runtime_error("imported trajectory does not provide enough independent gravity poses");
        std::vector<JointVector> stop_positions;
        for(const auto& target : selected) stop_positions.push_back(candidate.positions[target.trajectory_index]);
        const auto planned = serial_arm::calibration_retime::plan(candidate.positions,
            cfg_.safety.limits.max_vel, cfg_.safety.limits.max_acc, cfg_.runtime.ctrl_frequency_hz,
            0.003, stop_positions);
        candidate.positions = planned.positions;
        candidate.sample_dt = planned.sample_dt;
        if(!friction_trajectory_inside_safe_replay_range(candidate))
            throw std::runtime_error("retimed imported trajectory exceeds safe joint range");
        FrictionCalibrationTrajectory sampled;
        sampled.sample_dt = candidate.sample_dt / 4.0;
        for(std::size_t k = 0; k + 1 < candidate.positions.size(); ++k)
            for(int n = 0; n < 4; ++n)
                sampled.positions.push_back(serial_arm::calibration_retime::interpolate(
                    candidate.positions, static_cast<double>(k) + 0.25 * n, candidate.sample_dt, 1.0).position);
        sampled.positions.push_back(candidate.positions.back());
        if(!friction_trajectory_inside_safe_replay_range(sampled))
            throw std::runtime_error("interpolated trajectory violates joint position margins");
        const auto targets = model_calibration_remap_targets(candidate, stop_positions, selected);
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
            model_calibration_original_gain_profile_=robot_.get_gain_profile()==JointGainProfile::DRAG?JointGainProfile::RIGID:robot_.get_gain_profile();
            model_calibration_original_feedforward_ = robot_.get_model_feedforward_mode();
            model_calibration_original_admittance_ = cfg_.capability.admittance;
            model_calibration_original_gravity_scale_ = dynamics_.get_gravity_scale();
            model_calibration_original_suspended_ = robot_.is_admittance_suspended();
        }
        model_calibration_source_reversed_.store(endpoint_choice.reverse_recorded_path);
        model_calibration_source_task_id_ = source_task_id;
        model_calibration_task_id_ = machine_model_calibration_task_id();
        model_calibration_core_fingerprint_ = source_text_fingerprint(core_source_config_);
        model_calibration_urdf_fingerprint_ = source_text_fingerprint(urdf_source_config_);
        const char* storage = std::getenv("SERIAL_ARM_PROFILE_STORAGE");
        const std::filesystem::path calibration_root = storage && *storage ? std::filesystem::path(storage) / "model-calibration" : std::filesystem::current_path() / ".install" / "model-calibration";
        model_calibration_directory_ = (calibration_root / model_calibration_task_id_).string();
        const auto started = model_calibration_recorder_.start(model_calibration_directory_, model_calibration_metadata());
        if(!started) throw std::runtime_error(started.error());
        try {
            const auto source = std::filesystem::path(model_calibration_directory_) / "source";
            std::filesystem::create_directories(source);
            write_model_calibration_source_snapshot(source);
            std::ofstream origin(std::filesystem::path(model_calibration_directory_) / "imported-from.txt");
            origin << directory.string() << '\n';
            if(!origin) throw std::runtime_error("failed to write trajectory provenance");
            auto written = model_calibration_recorder_.write_trajectory(candidate.positions, candidate.sample_dt);
            if(!written) throw std::runtime_error(written.error());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                robot_.set_admittance_suspended(true);
                last_output_.reset();
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("cannot enter RIGID_HOLD before playback confirmation");
        }
        catch(...) {
            model_calibration_recorder_.stop();
            model_calibration_restore_runtime_hold();
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            throw;
        }
        model_calibration_trajectory_ = std::move(candidate);
        model_calibration_pose_targets_ = targets;
        model_calibration_replay_rate_ = 1.0;
        model_calibration_original_duration_s_ = recorded_duration;
        model_calibration_path_length_rad_ = planned.joint_path_length_rad;
        model_calibration_geometric_waypoints_ = planned.geometric_waypoints;
        model_calibration_original_samples_ = planned.original_samples;
        model_calibration_estimated_duration_s_ = serial_arm::calibration_acquisition::estimated_seconds(
            model_calibration_trajectory_->sample_dt * (model_calibration_trajectory_->positions.size() - 1),
            targets.size(), model_calibration_options_.static_hold_s + model_calibration_options_.static_sample_s + 0.5);
        model_calibration_update_progress(0, serial_arm::calibration_acquisition::progress_units(targets.size()));
        model_calibration_set_phase(ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION);
    }

    // Stopping teaching must remain safe after a FAULT or an explicit disable
    // The only effect in those cases is to finish file/worker cleanup
    void model_calibration_finish_aborted_teaching() {
        model_calibration_teach_stop_.store(true);
        cycle_cv_.notify_all();
        if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
        if(model_calibration_teaching_started_at_ && model_calibration_teaching_wall_duration_s_ <= 0.0)
            model_calibration_teaching_wall_duration_s_ = std::chrono::duration<double>(
                Robot::Clock::now() - *model_calibration_teaching_started_at_).count();
        model_calibration_teaching_started_at_.reset();
        model_calibration_recorder_.stop();
        model_calibration_restore_runtime_hold();
        model_calibration_write_result_file();
    }

    void machine_model_calibration_teach_stop(const YAML::Node& params) {
        model_calibration_require_task(params);
        const auto phase = model_calibration_phase_.load();
        if(phase == ModelCalibrationPhase::FAILED || phase == ModelCalibrationPhase::CANCELLED) {
            if(model_calibration_teach_thread_.joinable()) model_calibration_finish_aborted_teaching();
            return;
        }
        if(phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION || phase == ModelCalibrationPhase::COMPLETE)
            return;
        if(phase != ModelCalibrationPhase::TEACHING)
            throw std::runtime_error("cannot stop teaching while a calibration replay is active");
        if(machine_robot_state() != RobotState::ACTIVE) {
            const bool faulted = machine_robot_state() == RobotState::FAULT || model_calibration_fault_interrupted_.load();
            model_calibration_cancel_.store(true);
            model_calibration_set_phase(faulted ? ModelCalibrationPhase::FAILED : ModelCalibrationPhase::CANCELLED);
            model_calibration_finish_aborted_teaching();
            return;
        }
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
            const auto selected = model_calibration_select_pose_targets(*model_calibration_trajectory_);
            if(selected.size() < model_calibration_options_.minimum_training_groups + 1)
                throw std::runtime_error("demonstration does not contain enough independent gravity poses");
            std::vector<JointVector> stop_positions;
            for(const auto& target : selected)
                stop_positions.push_back(model_calibration_trajectory_->positions[target.trajectory_index]);
            const auto planned = serial_arm::calibration_retime::plan(
                model_calibration_trajectory_->positions,
                cfg_.safety.limits.max_vel,
                cfg_.safety.limits.max_acc,
                cfg_.runtime.ctrl_frequency_hz, 0.003, stop_positions);
            {
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                model_calibration_path_length_rad_ = planned.joint_path_length_rad;
                model_calibration_geometric_waypoints_ = planned.geometric_waypoints;
                model_calibration_original_samples_ = planned.original_samples;
            }
            model_calibration_trajectory_->positions = planned.positions;
            model_calibration_trajectory_->sample_dt = planned.sample_dt;
            model_calibration_pose_targets_ = model_calibration_remap_targets(
                *model_calibration_trajectory_, stop_positions, selected);
            // The newly time-parameterized trajectory is safe at rate 1.0.
            // The two slow friction passes use rate 0.5, with extra margin.
            model_calibration_replay_rate_ = 1.0;
            if(!friction_trajectory_inside_safe_replay_range(*model_calibration_trajectory_))
                throw std::runtime_error("retimed trajectory exceeds safe replay range");
            // Validate the actual spline at quarter intervals: a continuous
            // Hermite spline may overshoot positions even if its knots are safe.
            FrictionCalibrationTrajectory interpolated;
            interpolated.sample_dt = planned.sample_dt / 4.0;
            for(std::size_t k = 0; k + 1 < planned.positions.size(); ++k)
                for(int quarter = 0; quarter < 4; ++quarter)
                    interpolated.positions.push_back(serial_arm::calibration_retime::interpolate(
                        planned.positions, static_cast<double>(k) + 0.25 * quarter, planned.sample_dt, 1.0).position);
            interpolated.positions.push_back(planned.positions.back());
            if(!friction_trajectory_inside_safe_replay_range(interpolated))
                throw std::runtime_error("interpolated replay exceeds joint safety margin");
        }
        catch(const std::exception& error) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = std::string("cannot safely plan demonstration: ") + error.what();
            model_calibration_recorder_.stop();
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
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
        model_calibration_estimated_duration_s_ =
            serial_arm::calibration_acquisition::traversal_factor * planned_pass_s +
            2.0 * static_collection_s + 2.0;
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
        {std::lock_guard<std::mutex> lock(mutex_);robot_.clear_calibration_model_feedforward_mode();}
        model_calibration_set_phase(ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION);
        model_calibration_update_progress(0, serial_arm::calibration_acquisition::progress_units(model_calibration_pose_targets_.size()));
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
            const auto hold = runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD);
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

    // Governor is used for adaptive searching and model data acquisition
    // Frozen gain validation deliberately remains an ungoverned full-speed test
    // Each call observes the last completed feedback cycle, never a future pose
    double model_calibration_feedback_governor(double previous, double slew) {
        using serial_arm::calibration_replay_supervisor::evaluate;
        using serial_arm::calibration_replay_supervisor::search_progress_scale;
        std::lock_guard<std::mutex> lock(mutex_);
        double desired=1.0;
        if(last_output_) {
            const auto feedback=evaluate(last_output_->joint_cmd.pos,
                last_output_->joint_state.pos,last_output_->joint_state.vel,
                cfg_.safety.limits.max_vel);
            if(!feedback.valid || feedback.hard_violation) {
                model_calibration_error_="invalid or unsafe feedback during calibration replay";
                return 0.0;
            }
            desired=search_progress_scale(feedback);
        }
        return previous+std::clamp(desired-previous,-slew,slew);
    }

    bool model_calibration_send_reference(const JointVector& ref_pos, const JointVector& ref_vel,
                                          bool allow_gain_revision=false,
                                          serial_arm::calibration_gain_adaptation::PassMetrics* metrics=nullptr,
                                          const serial_arm::calibration_replay_supervisor::CompliantBounds* compliant_limits=nullptr,
                                          bool standalone_replay=false,
                                          bool compliant_replay=false) {
        // Keep the demonstrated path.  Supervise EACH replay point using live
        // feedback; when catching up, position reference is frozen AND target
        // velocity becomes ZERO.  Holding a nonzero target velocity was able to
        // drive a Damiao MIT motor beyond a frozen position reference.
        using serial_arm::calibration_reference_watchdog::Heartbeat;
        using serial_arm::calibration_replay_supervisor::evaluate;
        using serial_arm::calibration_replay_supervisor::State;
        Heartbeat heartbeat(Robot::Clock::now());
        // Search and gravity acquisition pause at the soft follow threshold.
        // A frozen gain verification has a DIFFERENT instant-error acceptance
        // criterion. Keep the pause threshold consistent with that criterion
        // while the independent hard following guard always remains active.
        const bool frozen_gain_verification=metrics && !allow_gain_revision;
        // Ordinary compliant playback is intentionally softer than frozen-gain
        // validation, while the independent hard-following limit stays unchanged.
        // A longer bounded catch-up lets compliant motors settle without
        // falsely reporting a calibration failure.
        const double pause_soft_ratio=standalone_replay && compliant_replay ? 1.5 :
            frozen_gain_verification ?
            (compliant_limits ? serial_arm::calibration_gain_adaptation::compliant_validation_max_error_ratio :
                                serial_arm::calibration_gain_adaptation::rigid_validation_max_error_ratio) : 1.0;
        std::string& follow_error=standalone_replay ? teach_replay_error_ : model_calibration_error_;
        const auto cancelled = [&]() {
            return model_calibration_cancel_.load() ||
                (standalone_replay && teach_replay_stop_.load());
        };
        std::unique_lock<std::mutex> lock(mutex_);
        if(robot_.get_state()!=RobotState::ACTIVE || cancelled() ||
           ref_pos.size()!=cfg_.joint_names.size() || ref_vel.size()!=ref_pos.size()) return false;
        std::uint64_t cursor=cycle_counter_;
        bool braking=false;
        const auto assess_replay=[&](const RobotCycleOutput& output) {
            if(standalone_replay && output.admittance_active)
                return serial_arm::calibration_replay_supervisor::evaluate_reference_layers(ref_pos,output.nominal_ref_pos,
                    output.accepted_ref_pos,output.joint_state.pos,output.joint_state.vel,cfg_.safety.limits.max_vel,pause_soft_ratio);
            return evaluate(ref_pos,output.joint_state.pos,output.joint_state.vel,cfg_.safety.limits.max_vel,pause_soft_ratio);
        };
        const auto issue_reference=[&](bool hold) {
            if(cancelled()) return false;
            const JointPosVelCmd cmd{ref_pos,
                serial_arm::calibration_replay_supervisor::command_velocity(ref_vel,hold)};
            const auto sent=runtime_.submit_reference(robot_,cmd,Robot::Clock::now(),standalone_replay ? MotionSource::REPLAY : MotionSource::CALIBRATION);
            if(!sent) {
                follow_error=(standalone_replay ? "trajectory replay reference command rejected" : "calibration replay reference command rejected");
                return false;
            }
            heartbeat.command_sent(Robot::Clock::now());
            return true;
        };
        // Inspect the initial offset before sending a nonzero velocity reference
        // An already lagging joint starts directly in stationary catch-up
        if(last_output_ && last_output_->joint_state.pos.size()==ref_pos.size() &&
           last_output_->joint_state.vel.size()==ref_pos.size()) {
            const auto first=assess_replay(*last_output_);
            if(!first.valid || first.hard_violation) {
                follow_error=(standalone_replay ? "trajectory replay initial reference exceeds hard following limit" : "calibration replay initial reference exceeds hard following limit");
                return false;
            }
            if(first.state==State::PAUSE_FOR_FEEDBACK) {
                braking=true;
                heartbeat.begin_catchup(Robot::Clock::now());
                if(metrics)++metrics->pauses;
            }
        }
        if(!issue_reference(braking))return false;
        while(!cancelled()) {
            const bool updated=cycle_cv_.wait_for(lock,Heartbeat::poll_interval,[&] {
                return cancelled() || cycle_counter_>cursor ||
                       robot_.get_state()!=RobotState::ACTIVE;
            });
            if(cancelled() || robot_.get_state()!=RobotState::ACTIVE)return false;
            const auto now=Robot::Clock::now();
            bool fresh_feedback=false;
            if(updated && cycle_counter_>cursor) {
                cursor=cycle_counter_;
                heartbeat.control_cycle_received(now);
                fresh_feedback=true;
            }
            if(heartbeat.feedback_stalled(now)) {
                follow_error=(standalone_replay ? "trajectory replay control feedback stalled" : "calibration replay control feedback stalled");
                return false;
            }
            if(fresh_feedback && last_output_ &&
               last_output_->joint_state.pos.size()==ref_pos.size() &&
               last_output_->joint_state.vel.size()==ref_pos.size()) {
                const auto& actual=last_output_->joint_state.pos;
                const auto& velocity=last_output_->joint_state.vel;
                const auto status=assess_replay(*last_output_);
                if(!status.valid) {
                    follow_error=(standalone_replay ? "trajectory replay feedback is invalid" : "calibration replay feedback is invalid");
                    return false;
                }
                if(metrics)metrics->observe(status.max_error / status.soft_limit[status.worst_joint]);
                // Soft limits govern path progress, hard limits always stop replay
                if(status.hard_violation) {
                    std::ostringstream diagnostic;
                    diagnostic<<std::fixed<<std::setprecision(4)
                        <<(standalone_replay ? "trajectory replay hard following limit reached: joint=" : "calibration replay hard following limit reached: joint=")
                        <<cfg_.joint_names[status.worst_joint]
                        <<" layer="<<(status.nominal_lag?"nominal_progress":"accepted_following")
                        <<" actual="<<(status.nominal_lag?last_output_->nominal_ref_pos[status.worst_joint]:actual[status.worst_joint])
                        <<" reference="<<(standalone_replay && last_output_->admittance_active && !status.nominal_lag?last_output_->accepted_ref_pos[status.worst_joint]:ref_pos[status.worst_joint])
                        <<" error="<<status.max_error
                        <<" hard_limit="<<status.hard_limit[status.worst_joint];
                    follow_error=diagnostic.str();
                    return false;
                }
                // Search revises gains on a measurable rising error BEFORE
                // reaching the pause threshold, not only after a full stop
                const double error_ratio=status.max_error/status.soft_limit[status.worst_joint];
                if(status.state!=State::FOLLOWING && !braking) {
                    braking=true;
                    heartbeat.begin_catchup(now);
                    if(metrics)++metrics->pauses;
                    // A paused trajectory MUST NOT keep the previous nonzero
                    // MIT velocity setpoint. Brake immediately at the SAME q.
                    if(!issue_reference(true))return false;
                }
                if(braking && status.state==State::FOLLOWING &&
                   status.peak_velocity<=status.recovery_velocity_limit) return true;
                if(!model_calibration_active_tracking_gains_) {
                    follow_error=(standalone_replay ? "trajectory replay tracking gains are unavailable" : "automatic calibration replay gains are unavailable");
                    return false;
                }
                // Only a real, fresh feedback response can trigger a revision.
                // Never use the user's configured rigidity as a calibration
                // target: the safety bounds are solely hard actuator ceilings.
                if(allow_gain_revision && model_calibration_last_replay_gain_revision_ &&
                   now-*model_calibration_last_replay_gain_revision_>=std::chrono::milliseconds(120) &&
                   (braking || status.state!=State::FOLLOWING || error_ratio>=0.65 ||
                    (compliant_limits && error_ratio<0.35))) {
                    auto revised=serial_arm::calibration_auto_gain::Gains{
                        model_calibration_active_tracking_gains_->kp,
                        model_calibration_active_tracking_gains_->kd};
                    const bool changed=compliant_limits ?
                        serial_arm::calibration_replay_supervisor::revise_compliant(
                            revised.kp,revised.kd,status,ref_pos,actual,velocity,*compliant_limits) :
                        serial_arm::calibration_replay_supervisor::revise_tracking(
                            revised.kp,revised.kd,status,ref_pos,actual,velocity,
                            cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd);
                    if(changed) {
                        JointImpedanceGains next{revised.kp,revised.kd};
                        if(!robot_.set_calibration_tracking_gains(next,
                            model_calibration_workflow_kind_=="gain" && model_calibration_gain_stage_.load()==1 ?
                            JointImpedanceMode::COMPLIANT_TRACKING : JointImpedanceMode::RIGID_TRACKING)) {
                            follow_error="calibration replay gain revision rejected by safety limits";
                            return false;
                        }
                        model_calibration_active_tracking_gains_=std::move(next);
                        if(metrics)++metrics->revisions;
                        model_calibration_adaptation_revisions_.fetch_add(1);
                    }
                    model_calibration_last_replay_gain_revision_=now;
                }
                if(status.state==State::FOLLOWING && !braking)return true;
                if(heartbeat.catchup_expired(now,allow_gain_revision || (standalone_replay && compliant_replay))) {
                    std::ostringstream diagnostic;
                    diagnostic<<std::fixed<<std::setprecision(4)
                        <<(frozen_gain_verification ?
                            "calibration frozen gain verification following limit exceeded: joint=" :
                            (standalone_replay ? "trajectory replay soft following recovery timed out: joint=" :
                            "calibration replay soft following recovery timed out: joint="))
                        <<cfg_.joint_names[status.worst_joint]
                        <<" actual="<<actual[status.worst_joint]
                        <<" reference="<<ref_pos[status.worst_joint]
                        <<" error="<<status.max_error
                        <<" soft_limit="<<status.soft_limit[status.worst_joint]
                        <<" hard_limit="<<status.hard_limit[status.worst_joint]
                        <<" kp="<<last_output_->joint_cmd.kp[status.worst_joint]
                        <<" kd="<<last_output_->joint_cmd.kd[status.worst_joint]
                        <<" adaptive="<<(allow_gain_revision?"true":"false")
                        <<" recovery_elapsed_s="<<heartbeat.catchup_elapsed_s(now)
                        <<" velocity="<<velocity[status.worst_joint]
                        <<" validation_pause_limit="<<std::min(
                            pause_soft_ratio*status.soft_limit[status.worst_joint],
                            0.80*status.hard_limit[status.worst_joint])
                        <<" error_ratio="<<error_ratio
                        <<" validation="<<(frozen_gain_verification?"true":"false")
                        <<" phase="<<(standalone_replay ? "teach_replay" :
                            model_calibration_workflow_kind_=="gain" ? "gain" : "gravity")
                        <<" tracking_mode="<<(standalone_replay ? (compliant_replay ? "COMPLIANT_TRACKING" : "RIGID_TRACKING") : "calibration");
                    follow_error=diagnostic.str();
                    return false;
                }
            } else if(heartbeat.catchup_expired(now,allow_gain_revision || (standalone_replay && compliant_replay))) {
                follow_error=(standalone_replay ? "trajectory replay did not receive fresh control feedback" : "calibration replay did not receive fresh control feedback");
                return false;
            }
            // The reference stays live for the native 100 ms CMD_TIMEOUT even
            // when path time is paused. A genuinely stalled cycle still faults.
            if(heartbeat.refresh_due(now) && !issue_reference(braking))return false;
        }
        return false;
    }

    // The URDF inertia and gravity gradient define physical gains in real
    // units, not a percentage of any user-supplied gain or safety ceiling.
    // Safety ceilings ONLY constrain commands.  The real robot response then
    // decides whether a pose is usable and whether the controller needs revision.
    JointImpedanceGains model_calibration_physical_gains(double frequency) const {
        const std::size_t n=cfg_.joint_names.size();
        if(!dynamics_.is_updated())throw std::runtime_error("calibration requires an updated dynamics model");
        const auto& M=dynamics_.get_mass_matrix();
        const auto& q=robot_.get_joint_state().pos;
        if(M.rows()!=static_cast<Eigen::Index>(n)||M.cols()!=static_cast<Eigen::Index>(n) || q.size()!=n)
            throw std::runtime_error("calibration inertia/state dimension mismatch");
        std::vector<double> inertia(n),gradient(n);
        for(std::size_t i=0;i<n;++i){
            inertia[i]=M(static_cast<Eigen::Index>(i),static_cast<Eigen::Index>(i));
            JointVector plus=q,minus=q;
            plus[i]+=0.005; minus[i]-=0.005;
            const auto a=dynamics_.get_gravity_regression(plus);
            const auto b=dynamics_.get_gravity_regression(minus);
            if(!a||!b||a->original_gravity.size()!=n||b->original_gravity.size()!=n)
                throw std::runtime_error("calibration cannot estimate gravity stiffness");
            gradient[i]=std::abs(a->original_gravity[i]-b->original_gravity[i])/0.01;
        }
        const auto computed=serial_arm::calibration_auto_gain::from_model(inertia,gradient,
            cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd,frequency);
        return JointImpedanceGains{computed.kp,computed.kd};
    }

    static double measured_feedback_age_ms(const RobotCycleOutput& output) {
        const auto& stamps = output.actuator_state.feedback_received_ns;
        if(stamps.empty() || stamps.size() != output.joint_state.pos.size())
            return std::numeric_limits<double>::quiet_NaN();
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
        const auto oldest = *std::min_element(stamps.begin(), stamps.end());
        const auto newest = *std::max_element(stamps.begin(), stamps.end());
        if(oldest == 0 || newest > static_cast<std::uint64_t>(now_ns))
            return std::numeric_limits<double>::quiet_NaN();
        return static_cast<double>(now_ns-oldest)/1.0e6;
    }

    // The background recorder keeps all cycles untagged. This snapshot is
    // copied ONLY into an in-memory candidate window, never accepted early.
    ModelCalibrationFrame model_calibration_static_frame(const RobotCycleOutput& output,
                                                          Robot::TimePoint now,
                                                          const ModelCalibrationPoseTarget& target) const {
        ModelCalibrationFrame f;
        f.monotonic_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        f.cycle=cycle_counter_;
        f.position=output.joint_state.pos;f.velocity=output.joint_state.vel;f.torque=output.joint_state.tor;
        f.feedback_received_ns=output.actuator_state.feedback_received_ns;
        f.feedback_timing_valid=output.actuator_state.feedback_timing_valid;
        f.acceleration=output.joint_acc;f.reference_position=output.joint_cmd.pos;
        f.reference_velocity=output.joint_cmd.vel;
        f.command_kp=output.joint_cmd.kp;f.command_kd=output.joint_cmd.kd;
        f.model_feedforward=output.model_feedforward;
        f.acceleration_source=output.joint_acc_valid ? "robot_feedback_velocity_filter" : "invalid_or_unknown_sample_timing";
        f.impedance_mode=to_string(robot_.get_impedance_mode());
        f.feedback_age_ms=measured_feedback_age_ms(output);
        f.phase=to_string(model_calibration_phase_.load());
        f.direction=model_calibration_direction_.load()<0?"reverse":"forward";
        f.speed_tier="fast";f.pose_group=target.pose_group;
        f.validation=target.validation;
        const auto good=[&](const JointVector& x){
            return x.size()==cfg_.joint_names.size() &&
                std::all_of(x.begin(),x.end(),[](double v){return std::isfinite(v);});
        };
        f.valid=good(f.position)&&good(f.velocity)&&good(f.torque)&&good(f.acceleration)&&
            good(f.reference_position)&&good(f.reference_velocity)&&good(f.command_kp)&&good(f.command_kd)&&
            good(f.model_feedforward) && std::isfinite(f.feedback_age_ms) && f.feedback_age_ms <= 30.0;
        return f;
    }

    bool model_calibration_collect_static(const ModelCalibrationPoseTarget& target, const std::string& direction) {
        using serial_arm::calibration_measurement::Observation;
        using serial_arm::calibration_measurement::StableWindow;
        model_calibration_pose_group_.store(0);
        model_calibration_validation_.store(false);
        model_calibration_direction_.store(direction=="reverse"?-1:1);
        JointImpedanceGains gains;
        std::vector<double> inertia;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            try {
                gains=robot_.get_config().ctrller.servo_tracking_gains;
                const auto& M=dynamics_.get_mass_matrix();
                for(std::size_t i=0;i<cfg_.joint_names.size();++i)
                    inertia.push_back(M(static_cast<Eigen::Index>(i),static_cast<Eigen::Index>(i)));
            } catch(const std::exception& e) {
                model_calibration_error_=e.what();return false;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(!robot_.begin_calibration_hold(gains)) {
                model_calibration_error_="could not safely switch to model-based calibration hold";
                return false;
            }
            last_output_.reset();
        }
        auto finish=[&](){
            if(robot_.get_state()!=RobotState::ACTIVE)return false;
            return set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING);
        };
        // Minimum initial ramp must complete before a sample is even considered.
        // No fixed Kp/Kd ratio and no assumption that a quiet encoder is enough.
        StableWindow window;
        std::vector<ModelCalibrationFrame> candidate;
        std::optional<Robot::TimePoint> good_started;
        auto adjustment_started=Robot::Clock::now();
        auto last_revision=adjustment_started;
        unsigned revisions=0;
        std::uint64_t cursor=0;
        {std::lock_guard<std::mutex> lock(mutex_);cursor=cycle_counter_;}
        const auto deadline=Robot::Clock::now()+std::chrono::duration_cast<Robot::Clock::duration>(
            std::chrono::duration<double>(model_calibration_options_.static_timeout_s+2.0));
        while(Robot::Clock::now()<deadline){
            if(model_calibration_cancel_.load()||!model_calibration_within_time_limit()) {
                (void)finish();return false;
            }
            const auto phase=model_calibration_phase_.load();
            if(model_calibration_pause_.load()){
                window.clear();candidate.clear();good_started.reset();
                adjustment_started=Robot::Clock::now();last_revision=adjustment_started;
            }
            if(!model_calibration_wait_if_paused(phase)){(void)finish();return false;}
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated=cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{
                return cycle_counter_>cursor||robot_.get_state()!=RobotState::ACTIVE||model_calibration_cancel_.load();
            });
            if(robot_.get_state()!=RobotState::ACTIVE||model_calibration_cancel_.load()){
                lock.unlock();(void)finish();return false;
            }
            if(!updated||cycle_counter_<=cursor||!last_output_)continue;
            cursor=cycle_counter_;
            const auto now=Robot::Clock::now();
            const auto frame=model_calibration_static_frame(*last_output_,now,target);
            Observation obs{std::chrono::duration<double>(now.time_since_epoch()).count(),
                frame.position,frame.velocity,frame.torque};
            lock.unlock();
            const double since_adjustment=std::chrono::duration<double>(now-adjustment_started).count();
            if(since_adjustment<0.43){window.clear();candidate.clear();good_started.reset();continue;}
            window.push(std::move(obs));
            const bool stable=window.stable(model_calibration_options_.static_max_velocity);
            if(stable && frame.valid){
                if(!good_started){good_started=now;candidate.clear();}
                candidate.push_back(frame);
                if(candidate.size()>=35 && std::chrono::duration<double>(now-*good_started).count()>=
                    model_calibration_options_.static_sample_s){
                    // Final check is across the WHOLE accepted window, not
                    // a handful of individually quiet 0.18 s slices.
                    StableWindow complete(model_calibration_options_.static_sample_s);
                    for(const auto& row:candidate)
                        complete.push(Observation{row.monotonic_ns*1e-9,row.position,row.velocity,row.torque});
                    if(complete.stable(model_calibration_options_.static_max_velocity)){
                        if(!model_calibration_recorder_.approve_static_group(candidate)){
                            model_calibration_error_="verified static group could not be committed";
                            (void)finish();return false;
                        }
                        model_calibration_valid_static_samples_+=candidate.size();
                        return finish();
                    }
                    candidate.clear();good_started.reset();
                }
            }else{
                good_started.reset();candidate.clear();
            }
            // Revision only after a genuinely observed response window.
            // Limit both count and cadence; all commands remain under the
            // existing motor safety limits. No user-provided tuning assumptions.
            if(!stable && window.full() && revisions<3 &&
               std::chrono::duration<double>(now-last_revision).count()>0.90){
                std::vector<double> std_tau,drift,peak;
                window.motion_metrics(std_tau,drift,peak);
                serial_arm::calibration_auto_gain::Gains proposal{gains.kp,gains.kd};
                if(serial_arm::calibration_auto_gain::revise(proposal,inertia,
                    cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd,std_tau,drift,peak)){
                    JointImpedanceGains next{proposal.kp,proposal.kd};
                    std::lock_guard<std::mutex> guard(mutex_);
                    if(!robot_.set_calibration_hold_gains(next)){
                        model_calibration_error_="automatic calibration gain revision rejected";
                        return false;
                    }
                    gains=std::move(next);adjustment_started=now;
                    window.clear();candidate.clear();good_started.reset();
                    ++revisions;
                }
                last_revision=now;
            }
        }
        ++model_calibration_rejected_static_groups_;
        // A failed pose is NEVER added to the online/offline dataset.
        return finish();
    }

    struct ServoTrial {
        std::vector<std::vector<serial_arm::servo_tuning::Sample>> rows;
        std::vector<serial_arm::servo_tuning::Metrics> metrics;
        std::vector<serial_arm::servo_tuning::ResponseModel> models;
        std::vector<serial_arm::servo_tuning::Requirements> plans;
        std::vector<serial_arm::servo_tuning::MotionObservation> motion;
        JointVector center,min_position,max_position;
        bool completed{false},passed{false};
    };
    struct GainQualificationFailure : std::runtime_error { using std::runtime_error::runtime_error; };

    void gain_probe_begin(const std::string& kind, const std::string& joint,
                          double duration, double amplitude, const std::string& message) {
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        gain_probe_kind_=kind;gain_probe_joint_=joint;gain_probe_duration_s_=duration;
        if(kind=="hold" || kind.find("frozen")!=std::string::npos || gain_probe_stage_=="noise")gain_candidate_index_=0;
        gain_probe_elapsed_s_=0;gain_probe_amplitude_rad_=amplitude;
        gain_probe_reference_.clear();gain_probe_message_=message;
        if(kind=="hold" || gain_probe_stage_=="drag") {
            std::ostringstream current;current<<"{\"completed\":false,\"stage\":\""<<machine_json_escape(gain_probe_stage_)
                <<"\",\"joint\":\""<<machine_json_escape(joint)<<"\",\"kp\":"<<gain_probe_kp_<<",\"kd\":"<<gain_probe_kd_
                <<",\"reason\":\""<<machine_json_escape(message)<<"\"}";
            gain_probe_diagnostic_json_=current.str();
        }
    }

    void gain_probe_update(double elapsed, const JointVector& reference, const std::string& message) {
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        gain_probe_elapsed_s_=std::clamp(elapsed,0.0,gain_probe_duration_s_);
        gain_probe_reference_=reference;gain_probe_message_=message;
        const double fraction=gain_probe_duration_s_>0 ? gain_probe_elapsed_s_/gain_probe_duration_s_ : 0;
        if(model_calibration_total_units_)
            model_calibration_progress_=std::max(model_calibration_progress_,
                std::min(0.99,(static_cast<double>(model_calibration_adaptation_passes_.load())+fraction)/
                    static_cast<double>(model_calibration_total_units_)));
    }

    void gain_probe_end(const std::string& message, bool completed=false) {
        const auto count=completed ? model_calibration_adaptation_passes_.fetch_add(1)+1 :
            model_calibration_adaptation_passes_.load();
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        gain_probe_reference_.clear();gain_probe_message_=message;
        if(completed) {
            gain_probe_elapsed_s_=gain_probe_duration_s_;
            model_calibration_completed_units_=count;
            if(model_calibration_total_units_)
                model_calibration_progress_=std::max(model_calibration_progress_,
                    std::min(0.99,static_cast<double>(count)/static_cast<double>(model_calibration_total_units_)));
        }
    }

    void gain_probe_diagnostic(std::size_t axis,const JointImpedanceGains& gains,
                               const serial_arm::servo_tuning::Requirements& req,const ServoTrial& trial,
                               const std::string& reason,const std::string& record,bool completed=true) {
        const auto& m=trial.metrics[axis];const auto& model=trial.models[axis];const auto& motion=trial.motion[axis];
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        std::ostringstream out;out<<std::setprecision(17);
        out<<"{\"completed\":"<<(completed?"true":"false")<<",\"stage\":\""<<machine_json_escape(gain_probe_stage_)<<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
            <<"\",\"kind\":\""<<machine_json_escape(gain_probe_kind_)<<"\",\"candidate_index\":"<<gain_candidate_index_
            <<",\"kp\":"<<gains.kp[axis]<<",\"kd\":"<<gains.kd[axis]<<",\"amplitude_rad\":"<<req.amplitude_rad
            <<",\"frequencies_hz\":["<<req.frequencies_hz[0]<<','<<req.frequencies_hz[1]<<','<<req.frequencies_hz[2]<<']'
            <<",\"samples\":"<<m.samples<<",\"measurement_samples\":"<<m.measurement_samples;
        if(m.samples) {
            out<<",\"reference_positive_span_rad\":"<<motion.reference_positive<<",\"reference_negative_span_rad\":"<<motion.reference_negative
                <<",\"actual_positive_span_rad\":"<<motion.actual_positive<<",\"actual_negative_span_rad\":"<<motion.actual_negative
                <<",\"minimum_motion_span_rad\":"<<motion.minimum_motion_span_rad
                <<",\"response_rms_ratio\":"<<m.response_rms_ratio<<",\"coherence\":"<<(req.motion_probe?motion.coherence:model.coherence)
                <<",\"continuous\":"<<(m.continuous?"true":"false")<<",\"limited_samples\":"<<m.limited_samples
                <<",\"observable\":"<<(m.observable?"true":"false")<<",\"metrics_passed\":"<<(m.passed?"true":"false")
                <<",\"follow_frequency_hz\":";
            if(completed && !req.motion_probe && !req.stationary_probe)out<<model.tested_follow_frequency_hz;else out<<"null";
            out<<",\"encoder_step_rad\":"<<std::max(req.encoder_step_rad,motion.encoder_step_rad)<<",\"noise_rms_rad\":"<<req.noise_rms_rad
                <<",\"metrics\":{\"rms_error_rad\":"<<m.rms_error<<",\"peak_error_rad\":"<<m.peak_error<<",\"velocity_rms_rad_s\":"<<m.velocity_rms
                <<",\"overshoot_rad\":"<<m.overshoot<<",\"settling_time_s\":"<<m.settling_s<<",\"tail_error_rad\":"<<m.tail_error_rad<<'}';
        }
        out<<",\"requirements\":{\"rms_error_rad\":"<<req.rms_error_rad<<",\"peak_error_rad\":"<<req.peak_error_rad
            <<",\"velocity_rms_rad_s\":"<<req.velocity_rms_rad_s<<",\"overshoot_rad\":"<<req.overshoot_rad<<",\"settling_time_s\":"<<req.settling_time_s
            <<",\"settling_error_rad\":"<<req.settling_error_rad<<'}'
            <<",\"reason\":\""<<machine_json_escape(reason)<<"\",\"record_path\":\""<<machine_json_escape(record)<<"\"}";
        gain_probe_diagnostic_json_=out.str();gain_probe_message_=reason;
    }

    ServoTrial gain_response_trial(const JointImpedanceGains& gains,JointGainProfile profile,
                                   bool admittance,bool configured,const std::string& label,bool /*searching*/=false,
                                   std::optional<std::size_t> selected_axis=std::nullopt,
                                   const std::vector<serial_arm::servo_tuning::Requirements>* plans=nullptr,
                                   const std::string& stage="dynamic_search") {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();ServoTrial trial;
        trial.rows.resize(n);trial.center.resize(n);trial.min_position.resize(n);trial.max_position.resize(n);
        trial.metrics.resize(n);trial.models.resize(n);trial.plans.resize(n);trial.motion.resize(n);
        const auto mode=profile==JointGainProfile::COMPLIANT ? JointImpedanceMode::COMPLIANT_TRACKING : JointImpedanceMode::RIGID_TRACKING;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("gain probe requires ACTIVE robot");
            clear_command_sources();
            if(!runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile))
                throw std::runtime_error("cannot select gain probe execution profile");
            if(!robot_.set_calibration_reference_behavior(admittance ? ReferenceBehavior::MODEL_ADMITTANCE : ReferenceBehavior::NATIVE_IMPEDANCE))
                throw std::runtime_error("cannot select gain probe reference behavior");
            robot_.set_admittance_suspended(!admittance);
            if(configured)robot_.clear_calibration_tracking_gains();
            else if(!robot_.set_calibration_tracking_gains(gains,mode))throw std::runtime_error("gain probe candidate rejected");
            trial.center=gain_probe_origin_.size()==n ? gain_probe_origin_ : robot_.get_joint_state().pos;trial.min_position=trial.max_position=trial.center;
        }
        const auto first_axis=selected_axis.value_or(0),end_axis=selected_axis ? first_axis+1 : n;
        for(std::size_t axis=first_axis;axis<end_axis;++axis) {
            Requirements req=plans ? plans->at(axis) : gain_requirements_;
            if(!plans) {
                if(profile==JointGainProfile::COMPLIANT)for(auto& hz:req.frequencies_hz)hz*=0.65;
                double margin=req.local_range_rad;
                if(cfg_.safety.limits.has_position_limit[axis])margin=std::min(margin,std::min(
                    trial.center[axis]-cfg_.safety.limits.min_pos[axis]-cfg_.safety.limits.pos_margin[axis],
                    cfg_.safety.limits.max_pos[axis]-cfg_.safety.limits.pos_margin[axis]-trial.center[axis]));
                req=bounded_plan(req,req.amplitude_rad,req.frequencies_hz[0],{margin,
                    .3*cfg_.safety.limits.max_vel[axis],.3*cfg_.safety.limits.max_acc[axis],
                    .5*cfg_.safety.limits.max_effort[axis],gains.kp[axis],gains.kd[axis]});
            }
            if(axis<gain_noise_.size()) {
                req.noise_rms_rad=gain_noise_[axis].rms_rad;req.encoder_step_rad=gain_noise_[axis].encoder_step_rad;
                req.velocity_noise_floor=gain_noise_[axis].velocity_peak+1e-6;
            }
            trial.plans[axis]=req;
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_=stage;
                gain_probe_frequency_hz_=req.frequencies_hz;gain_probe_kp_=gains.kp[axis];gain_probe_kd_=gains.kd[axis];}
            const std::string record="servo-probe-"+label+"-"+std::to_string(axis)+".csv";
            gain_probe_begin(label,cfg_.joint_names[axis],req.probe_duration_s+req.tail_s,req.amplitude_rad,"settling");
            gain_probe_diagnostic(axis,gains,req,trial,"measurement_pending",record,false);
            if(!req.stationary_probe && req.amplitude_rad<0.001) {
                gain_probe_diagnostic(axis,gains,req,trial,"safety_bound",record,false);
                gain_probe_end("safety_bound");
                throw GainQualificationFailure("insufficient joint margin or excitation budget for an observable bounded servo probe at "+cfg_.joint_names[axis]);
            }
            if(!req.stationary_probe) {
                double margin=req.local_range_rad;
                if(cfg_.safety.limits.has_position_limit[axis])margin=std::min(margin,std::min(
                    trial.center[axis]-cfg_.safety.limits.min_pos[axis]-cfg_.safety.limits.pos_margin[axis],
                    cfg_.safety.limits.max_pos[axis]-cfg_.safety.limits.pos_margin[axis]-trial.center[axis]));
                const auto bound=bounded_plan(req,req.amplitude_rad,req.frequencies_hz[0],{margin,
                    .3*cfg_.safety.limits.max_vel[axis],.3*cfg_.safety.limits.max_acc[axis],
                    .5*cfg_.safety.limits.max_effort[axis],gains.kp[axis],gains.kd[axis]});
                if(bound.amplitude_rad+1e-7<req.amplitude_rad) {
                    gain_probe_diagnostic(axis,gains,req,trial,"safety_bound",record,false);gain_probe_end("safety_bound");
                    throw GainQualificationFailure("gain probe cannot preserve its established amplitude within safety limits at "+cfg_.joint_names[axis]);
                }
            }
            std::ofstream csv(std::filesystem::path(model_calibration_directory_)/record);
            if(!csv)throw std::runtime_error("cannot record gain probe");
            csv << "time_s,reference,reference_velocity,actual,velocity,residual,residual_valid,limited,new_measurement\n" << std::setprecision(17);
            const auto warm_until=Robot::Clock::now()+std::chrono::milliseconds(400);
            Robot::TimePoint started{};std::uint64_t cursor=0,previous_sample=0;
            bool first=true;std::size_t fresh_samples=0;
            try {
            while(true) {
                std::unique_lock<std::mutex> lock(mutex_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("continuous gain probe interrupted; restart verification rather than pause-and-pass");
                const auto now=Robot::Clock::now();
                if(now<warm_until) {
                    const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{trial.center,JointVector(n,0.0)},now,MotionSource::CALIBRATION);
                    if(!sent)throw machine_robot_error("gain probe settling reference",sent.error());
                    gain_probe_update(0,trial.center,"settling");
                    cycle_cv_.wait_for(lock,std::chrono::milliseconds(5));continue;
                }
                if(first){started=now;cursor=cycle_counter_;first=false;}
                const double time=std::chrono::duration<double>(now-started).count();
                if(time>req.probe_duration_s+req.tail_s || (req.stationary_probe && time>=1.2 && fresh_samples>=100))break;
                auto command=trial.center;JointVector velocity(n,0.0);
                const auto offset=probe(time,req);command[axis]+=offset.first;velocity[axis]=offset.second;
                const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{command,velocity},now,MotionSource::CALIBRATION);
                if(!sent)throw machine_robot_error("continuous gain probe reference",sent.error());
                gain_probe_update(time,command,"measuring");
                const bool fresh=cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();});
                if(!fresh || !last_output_ || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("gain probe lost continuous feedback");
                cursor=cycle_counter_;const auto& out=*last_output_;
                if(out.joint_state.pos.size()!=n || out.joint_state.vel.size()!=n)throw std::runtime_error("gain probe feedback dimensions changed");
                const auto& stamps=out.actuator_state.feedback_received_ns;
                const auto stamp=stamps.size()==n ? stamps[axis] : 0;
                const bool new_sample=stamp>previous_sample && stamp>0;
                previous_sample=std::max(previous_sample,stamp);
                bool limited=std::any_of(out.reference_limited.begin(),out.reference_limited.end(),[](auto value){return value!=0;}) ||
                    std::any_of(out.torque_limited.begin(),out.torque_limited.end(),[](auto value){return value!=0;});
                Sample sample;
                sample.time_s=std::chrono::duration<double>(Robot::TimePoint(std::chrono::nanoseconds(last_cycle_monotonic_ns_))-started).count();
                if(sample.time_s<0)continue;
                const auto& reference=admittance ? out.virtual_ref_pos : out.nominal_ref_pos;
                const auto& ref_velocity=admittance ? out.virtual_ref_vel : out.nominal_ref_vel;
                sample.reference=reference.size()==n ? reference[axis] : command[axis];
                sample.reference_velocity=ref_velocity.size()==n ? ref_velocity[axis] : velocity[axis];
                sample.actual=out.joint_state.pos[axis];sample.velocity=out.joint_state.vel[axis];
                sample.limited=limited;sample.new_measurement=new_sample;if(new_sample)++fresh_samples;
                if(out.residual_raw.size()==n && out.observer_valid){sample.residual=out.residual_raw[axis];sample.residual_valid=true;}
                if(admittance && time>0.5 && (!out.observer_valid || !out.admittance_active))throw std::runtime_error("admittance validation requires an active MDK reference and valid force observer");
                for(std::size_t j=0;j<n;++j) {
                    trial.min_position[j]=std::min(trial.min_position[j],out.joint_state.pos[j]);
                    trial.max_position[j]=std::max(trial.max_position[j],out.joint_state.pos[j]);
                    if(std::abs(out.joint_state.pos[j]-trial.center[j])>req.local_range_rad)
                        throw std::runtime_error("gain probe exceeded its local motion envelope");
                }
                trial.rows[axis].push_back(sample);lock.unlock();
                csv << sample.time_s << ',' << sample.reference << ',' << sample.reference_velocity << ',' << sample.actual << ',' << sample.velocity << ','
                    << sample.residual << ',' << sample.residual_valid << ',' << sample.limited << ',' << sample.new_measurement << '\n';
            }
            csv.flush();if(!csv)throw std::runtime_error("gain probe recording failed");
            } catch(...) {
                trial.metrics[axis]=assess(trial.rows[axis],req,req.probe_duration_s);
                trial.motion[axis]=observe_motion(trial.rows[axis],req,axis<gain_noise_.size()?gain_noise_[axis]:NoiseEstimate{});
                gain_probe_diagnostic(axis,gains,req,trial,"measurement_interrupted",record,false);
                throw;
            }
            gain_probe_end("assessing",true);
            trial.metrics[axis]=assess(trial.rows[axis],req,req.probe_duration_s);
            trial.motion[axis]=observe_motion(trial.rows[axis],req,axis<gain_noise_.size()?gain_noise_[axis]:NoiseEstimate{});
            if(!req.stationary_probe && !req.motion_probe)trial.models[axis]=identify(trial.rows[axis],req);
            const bool qualified=req.stationary_probe ? trial.metrics[axis].continuous : req.motion_probe ? trial.motion[axis].qualified :
                trial.motion[axis].qualified && trial.metrics[axis].passed && trial.models[axis].tested_follow_frequency_hz>0;
            const std::string reason=req.stationary_probe ? "noise_measured" : !trial.motion[axis].qualified ? trial.motion[axis].reason :
                req.motion_probe ? "motion_established" : trial.models[axis].resonance_observed || trial.metrics[axis].oscillating ? "resonance" :
                trial.models[axis].coherence<.65 ? "low_coherence" : trial.models[axis].tested_follow_frequency_hz<=0 ?
                (trial.models[axis].points.empty() || trial.models[axis].points.front().gain<.8 ? "low_bandwidth" : "excessive_lag") :
                trial.metrics[axis].passed ? "qualified" : "tracking_not_qualified";
            gain_probe_diagnostic(axis,gains,req,trial,reason,record);
            gain_probe_end(reason);
            // Stop this candidate on its first rejected axis. Later axes have
            // no measurements yet and must not be revised speculatively.
            if(!req.stationary_probe && !qualified)break;
        }
        {std::lock_guard<std::mutex> lock(mutex_);robot_.set_admittance_suspended(true);
            if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD))throw std::runtime_error("cannot hold after servo probe");}
        trial.completed=std::all_of(trial.metrics.begin(),trial.metrics.end(),[](const Metrics& m){return m.samples>0;});
        trial.passed=trial.completed && std::all_of(trial.motion.begin(),trial.motion.end(),[](const auto& m){return m.qualified;}) && std::all_of(trial.metrics.begin(),trial.metrics.end(),[](const Metrics& m){return m.passed;}) &&
            std::all_of(trial.models.begin(),trial.models.end(),[](const auto& m){return m.tested_follow_frequency_hz>0;});
        return trial;
    }

    // Static qualification uses the Core's actual nominal HOLD target
    struct GainHoldResult {
        std::size_t samples{0},observer_samples{0},integral_samples{0};double worst_error{0},worst_velocity{0};
        double raw_worst_velocity{0},worst_span{0};
        JointVector target,mean_error;bool validated{false};
    };
    GainHoldResult gain_hold_trial(const JointImpedanceGains& gains,bool compliant,
                                  const HoldCompensationCfg& compensation) {
        GainHoldResult result;const auto n=cfg_.joint_names.size();result.mean_error.assign(n,0);
        std::vector<std::vector<serial_arm::servo_tuning::Sample>> position_windows(n);
        std::vector<std::uint64_t> previous_stamp(n,0);
        const auto mode=compliant?JointImpedanceMode::COMPLIANT_HOLD:JointImpedanceMode::RIGID_HOLD;
        {
            std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(!runtime_.set_impedance_mode(robot_,mode) ||
               !robot_.set_execution_gain_profile(compliant?JointGainProfile::COMPLIANT:JointGainProfile::RIGID) ||
               !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
               !robot_.set_calibration_tracking_gains(gains,mode) || !robot_.set_hold_compensation_cfg(compensation))
                throw std::runtime_error("cannot select hold verification controls");
            robot_.set_admittance_suspended(false);result.target=robot_.get_hold_target();
        }
        const auto started=Robot::Clock::now();std::uint64_t cursor=0;
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="hold";gain_candidate_index_=0;gain_probe_kp_=0;gain_probe_kd_=0;}
        gain_probe_begin("hold","",2.1,0,"hold_verification");
        while(std::chrono::duration<double>(Robot::Clock::now()-started).count()<2.1) {
            std::unique_lock<std::mutex> lock(mutex_);
            if(model_calibration_cancel_.load() || model_calibration_pause_.load())throw std::runtime_error("continuous hold verification interrupted");
            gain_probe_update(std::chrono::duration<double>(Robot::Clock::now()-started).count(),{},"hold_verification");
            if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE;}) ||
               robot_.get_state()!=RobotState::ACTIVE || !last_output_)throw std::runtime_error("hold verification lost feedback");
            cursor=cycle_counter_;if(std::chrono::duration<double>(Robot::Clock::now()-started).count()<0.8)continue;
            const auto& output=*last_output_;
            if(output.hold_target_pos.size()!=n || output.joint_state.pos.size()!=n || output.joint_state.vel.size()!=n)
                throw std::runtime_error("hold target feedback unavailable");
            if(std::any_of(output.torque_limited.begin(),output.torque_limited.end(),[](auto v){return v!=0;})) {
                gain_probe_end("safety_bound",true);return result;
            }
            for(std::size_t i=0;i<n;++i) {
                // The nominal HOLD target is authoritative, never a fresh actual-position baseline
                if(std::abs(result.target[i]-output.hold_target_pos[i])>1e-9)throw std::runtime_error("hold target changed during verification");
                const double error=output.joint_state.pos[i]-output.hold_target_pos[i];
                result.mean_error[i]+=error;result.worst_error=std::max(result.worst_error,std::abs(error));
                result.raw_worst_velocity=std::max(result.raw_worst_velocity,std::abs(output.joint_state.vel[i]));
                const auto& stamps=output.actuator_state.feedback_received_ns;
                const auto stamp=stamps.size()==n?stamps[i]:0;
                serial_arm::servo_tuning::Sample row;row.time_s=std::chrono::duration<double>(Robot::Clock::now()-started).count();
                row.actual=output.joint_state.pos[i];row.velocity=output.joint_state.vel[i];row.new_measurement=stamp>previous_stamp[i] && stamp>0;
                previous_stamp[i]=std::max(previous_stamp[i],stamp);position_windows[i].push_back(row);
                const auto noise=i<gain_noise_.size()?gain_noise_[i]:serial_arm::servo_tuning::NoiseEstimate{};
                double speed=std::abs(serial_arm::servo_tuning::position_window_velocity(position_windows[i],noise));
                if(std::abs(row.velocity)>std::max(gain_requirements_.hold_velocity_rad_s,noise.velocity_peak+1e-6))speed=std::max(speed,std::abs(row.velocity));
                result.worst_velocity=std::max(result.worst_velocity,speed);
            }
            ++result.samples;
            if(output.observer_valid)++result.observer_samples;
            if(std::any_of(output.hold_integral_torque.begin(),output.hold_integral_torque.end(),[](double v){return std::abs(v)>1e-6;}))++result.integral_samples;
        }
        if(result.samples)for(auto& e:result.mean_error)e/=result.samples;
        bool stationary=true;
        for(std::size_t axis=0;axis<n;++axis) {const auto& rows=position_windows[axis];if(rows.empty()){stationary=false;continue;}
            double lo=rows.front().actual,hi=lo;for(const auto& row:rows){lo=std::min(lo,row.actual);hi=std::max(hi,row.actual);}
            result.worst_span=std::max(result.worst_span,hi-lo);
            const auto noise=axis<gain_noise_.size()?gain_noise_[axis]:serial_arm::servo_tuning::NoiseEstimate{};
            stationary=stationary && hi-lo<=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
            if(std::count_if(rows.begin(),rows.end(),[](const auto& row){return row.new_measurement;})<30)result.samples=0;
        }
        gain_probe_end("assessing",true);
        result.validated=stationary && result.samples>=30 && result.worst_span<=gain_requirements_.hold_error_rad && result.worst_error<=gain_requirements_.hold_error_rad &&
            result.worst_velocity<=gain_requirements_.hold_velocity_rad_s &&
            (!compensation.enabled || (result.observer_samples>=30 && result.integral_samples>=30));
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);std::ostringstream summary;
            summary<<"{\"completed\":true,\"stage\":\"hold\",\"samples\":"<<result.samples
                <<",\"metrics\":{\"peak_error_rad\":"<<result.worst_error<<",\"tail_error_rad\":"<<result.worst_error
                <<",\"velocity_rms_rad_s\":"<<result.worst_velocity<<"},\"requirements\":{\"peak_error_rad\":"<<gain_requirements_.hold_error_rad
                <<",\"settling_error_rad\":"<<gain_requirements_.hold_error_rad<<",\"velocity_rms_rad_s\":"<<gain_requirements_.hold_velocity_rad_s
                <<"},\"metrics_passed\":"<<(result.validated?"true":"false")<<",\"reason\":\""<<(result.validated?"qualified":"hold_drift")<<"\"}";
            gain_probe_diagnostic_json_=summary.str();}
        return result;
    }

    struct DragTrial {JointVector decay_s,initial_velocity,peak_velocity;bool validated{true};std::size_t samples{0};};
    DragTrial gain_drag_trial(const JointImpedanceGains& gains,const JointImpedanceGains& excitation,
                              const std::string& label,std::optional<std::size_t> selected_axis=std::nullopt) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();DragTrial result;
        result.decay_s.assign(n,0);result.initial_velocity.assign(n,0);result.peak_velocity.assign(n,0);
        const auto first_axis=selected_axis.value_or(0),end_axis=selected_axis ? first_axis+1 : n;
        for(std::size_t axis=first_axis;axis<end_axis;++axis) {
            if(gain_native_plans_.size()!=n)throw GainQualificationFailure("drag excitation requires an independently qualified tracking probe");
            auto plan=gain_native_plans_[axis];plan.motion_probe=true;
            const double release_at=1.0/plan.frequencies_hz[0],duration=release_at+2.0;
            const auto noise=axis<gain_noise_.size()?gain_noise_[axis]:NoiseEstimate{};
            std::vector<Sample> position_window;std::uint64_t previous_stamp=0;
            const auto observed_speed=[&]() {
                if(position_window.size()<3)return 0.0;
                const double end=position_window.back().time_s;double lo=1e30,hi=-1e30;
                for(const auto& row:position_window)if(row.new_measurement && row.time_s>=end-.6){lo=std::min(lo,row.actual);hi=std::max(hi,row.actual);}
                if(hi-lo<=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002}))return 0.0;
                return std::max(std::abs(position_window_velocity(position_window,noise,.6)),(hi-lo)/.6);
            };
            {
                std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_TRACKING) ||
                   !robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
                   !robot_.set_calibration_tracking_gains(excitation,JointImpedanceMode::RIGID_TRACKING))
                    throw std::runtime_error("cannot select bounded drag damping probe");
                robot_.set_admittance_suspended(true);
            }
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="drag";gain_probe_frequency_hz_=plan.frequencies_hz;gain_probe_kp_=excitation.kp[axis];gain_probe_kd_=excitation.kd[axis];}
            gain_probe_begin(label,cfg_.joint_names[axis],duration,plan.amplitude_rad,"drag_excitation");
            const auto start=Robot::Clock::now();bool released=false;double release_s=0;std::uint64_t cursor=0;
            std::ofstream csv(std::filesystem::path(model_calibration_directory_)/("drag-decay-"+label+"-"+std::to_string(axis)+".csv"));
            if(!csv)throw std::runtime_error("cannot record drag validation");
            csv<<"time_s,reference,reference_velocity,actual,velocity,position_window_speed,released,new_measurement\n"<<std::setprecision(17);
            while(true) {
                std::unique_lock<std::mutex> lock(mutex_);const auto now=Robot::Clock::now();
                const double time=std::chrono::duration<double>(now-start).count();if(time>duration)break;
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("continuous drag verification interrupted");
                JointVector q=gain_probe_origin_,v(n,0);
                if(time<release_at) {
                    const auto offset=probe(time,plan);q[axis]+=offset.first;v[axis]=offset.second;
                    if(!runtime_.submit_reference(robot_,JointPosVelCmd{q,v},now,MotionSource::CALIBRATION))throw std::runtime_error("drag excitation rejected");
                    gain_probe_update(time,q,"drag_excitation");
                } else if(!released) {
                    result.initial_velocity[axis]=observed_speed();
                    if(result.initial_velocity[axis]<.003){gain_probe_end("no_motion");throw GainQualificationFailure("drag excitation produced no observable release velocity at "+cfg_.joint_names[axis]+"; damping search cannot qualify a stationary joint");}
                    if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::COMPLIANT_DRAG) ||
                       !robot_.set_execution_gain_profile(JointGainProfile::DRAG) ||
                       !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::COMPLIANT_DRAG))
                        throw std::runtime_error("drag release rejected");
                    clear_command_sources();released=true;release_s=time;
                    {std::lock_guard<std::mutex> probe_lock(model_calibration_mutex_);gain_probe_kp_=gains.kp[axis];gain_probe_kd_=gains.kd[axis];}
                }
                if(released)gain_probe_update(time,{},"drag_verification");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE;}) || !last_output_ || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("drag validation lost feedback");
                cursor=cycle_counter_;const auto& out=*last_output_;
                for(std::size_t j=0;j<n;++j)if(std::abs(out.joint_state.pos[j]-gain_probe_origin_[j])>gain_requirements_.local_range_rad)throw std::runtime_error("drag probe exceeded local motion envelope");
                const auto& stamps=out.actuator_state.feedback_received_ns;const auto stamp=stamps.size()==n?stamps[axis]:0;
                Sample row;row.time_s=time;row.actual=out.joint_state.pos[axis];row.velocity=out.joint_state.vel[axis];row.new_measurement=stamp>previous_stamp && stamp>0;
                previous_stamp=std::max(previous_stamp,stamp);position_window.push_back(row);
                double speed=observed_speed();
                if(std::abs(row.velocity)>std::max(gain_requirements_.hold_velocity_rad_s,noise.velocity_peak+1e-6))speed=std::max(speed,std::abs(row.velocity));
                if(released) {
                    ++result.samples;result.peak_velocity[axis]=std::max(result.peak_velocity[axis],speed);
                    if(speed>std::max(.003,.1*result.initial_velocity[axis]))result.decay_s[axis]=time-release_s;
                    if(std::any_of(out.torque_limited.begin(),out.torque_limited.end(),[](auto value){return value!=0;}))result.validated=false;
                }
                lock.unlock();csv<<time<<','<<q[axis]<<','<<v[axis]<<','<<row.actual<<','<<row.velocity<<','<<speed<<','<<released<<','<<row.new_measurement<<'\n';
            }
            csv.flush();if(!csv)throw std::runtime_error("drag validation recording failed");
            result.validated=result.validated && result.initial_velocity[axis]>=.003 && result.decay_s[axis]<=1.0 &&
                result.peak_velocity[axis]<=std::max(.025,1.4*result.initial_velocity[axis]);
            gain_probe_end("assessing",true);
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);std::ostringstream summary;summary<<std::setprecision(17);
                summary<<"{\"completed\":true,\"stage\":\"drag\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
                    <<"\",\"kp\":"<<gains.kp[axis]<<",\"kd\":"<<gains.kd[axis]<<",\"amplitude_rad\":"<<plan.amplitude_rad
                    <<",\"samples\":"<<result.samples<<",\"initial_velocity_rad_s\":"<<result.initial_velocity[axis]
                    <<",\"peak_velocity_rad_s\":"<<result.peak_velocity[axis]<<",\"metrics\":{\"settling_time_s\":"<<result.decay_s[axis]
                    <<"},\"requirements\":{\"settling_time_s\":1.0},\"metrics_passed\":"<<(result.validated?"true":"false")
                    <<",\"reason\":\""<<(result.validated?"qualified":"drag_decay_not_qualified")<<"\",\"record_path\":\"drag-decay-"<<machine_json_escape(label)<<'-'<<axis<<".csv\"}";
                gain_probe_diagnostic_json_=summary.str();}
            if(!result.validated)break;
        }
        {std::lock_guard<std::mutex> lock(mutex_);if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD))throw std::runtime_error("cannot hold after damping probe");}
        return result;
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
        const double governor_slew=serial_arm::calibration_replay_supervisor::search_governor_slew_limit(
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,cfg_.runtime.ctrl_frequency_hz);
        if(!std::isfinite(governor_slew) || governor_slew<=0.0)return false;
        double progress = 0.0;
        double governor=1.0;
        std::size_t next_target = 0;
        std::vector<std::pair<std::size_t, ModelCalibrationPoseTarget>> target_list(targets.begin(), targets.end());
        while(progress < static_cast<double>(ordered.size() - 1)) {
            const auto phase = reverse ? ModelCalibrationPhase::STATIC_REVERSE : ModelCalibrationPhase::STATIC_FORWARD;
            if(!model_calibration_wait_if_paused(phase)) return false;
            governor=model_calibration_feedback_governor(governor,governor_slew);
            if(governor<=0.0)return false;
            const double rate=model_calibration_replay_rate_*governor;
            const double step=rate*nominal_dt/trajectory.sample_dt;
            if(!std::isfinite(step) || step<=0.0)return false;
            double next_progress = std::min(static_cast<double>(ordered.size() - 1), progress + step);
            if(next_target < target_list.size())
                next_progress = std::min(next_progress, static_cast<double>(target_list[next_target].first));
            const auto reference = serial_arm::calibration_retime::interpolate(
                ordered, next_progress, trajectory.sample_dt, rate);
            if(!model_calibration_send_reference(reference.position, reference.velocity)) return false;
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
        const auto& trajectory = *model_calibration_trajectory_;
        std::vector<JointVector> ordered = trajectory.positions;
        if(reverse) std::reverse(ordered.begin(), ordered.end());
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_TRACKING)) return false;
        model_calibration_direction_.store(reverse ? -1 : 1);
        model_calibration_speed_tier_.store(speed_tier);
        const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        const double governor_slew=serial_arm::calibration_replay_supervisor::search_governor_slew_limit(
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,cfg_.runtime.ctrl_frequency_hz);
        if(!std::isfinite(governor_slew) || governor_slew<=0.0)return false;
        double governor=1.0;
        double progress = 0.0;
        while(progress < static_cast<double>(ordered.size() - 1)) {
            if(!model_calibration_wait_if_paused(phase))return false;
            governor=model_calibration_feedback_governor(governor,governor_slew);
            if(governor<=0.0)return false;
            const double rate=playback_rate*governor;
            const double step=rate*nominal_dt/trajectory.sample_dt;
            if(!std::isfinite(step) || step<=0.0)return false;
            const double next_progress = std::min(static_cast<double>(ordered.size() - 1), progress + step);
            const auto reference = serial_arm::calibration_retime::interpolate(
                ordered, next_progress, trajectory.sample_dt, rate);
            if(!model_calibration_send_reference(reference.position, reference.velocity))return false;
            progress = next_progress;
        }
        {
            std::lock_guard<std::mutex> lock(model_calibration_mutex_);
            ++model_calibration_completed_units_;
            model_calibration_progress_ = model_calibration_total_units_ ? static_cast<double>(model_calibration_completed_units_) / static_cast<double>(model_calibration_total_units_) : 0.0;
        }
        return true;
    }

    bool model_calibration_validate_friction_holdout(
        const AdmittanceFrictionCalibrationResult& fit,
        const std::vector<AdmittanceFrictionSample>& validation) const {
        if(validation.empty()) return false;
        const std::size_t n = cfg_.joint_names.size();
        std::vector<double> before(n, 0.0), after(n, 0.0); std::vector<std::size_t> counts(n, 0);
        for(const auto& sample : validation) {
            for(std::size_t i = 0; i < n; ++i) { const double v = sample.velocity[i]; if(std::abs(v) < 0.05 || std::abs(sample.acceleration[i]) > 1.5)continue; const double prediction = v >= 0.0 ? fit.positive_coulomb[i] + fit.positive_viscous[i] * std::abs(v) : fit.negative_coulomb[i] + fit.negative_viscous[i] * std::abs(v); before[i] += sample.residual_after_bias[i] * sample.residual_after_bias[i]; const double e = sample.residual_after_bias[i] - prediction; after[i] += e * e; ++counts[i]; }
        }
        bool pass = true;
        for(std::size_t i = 0; i < n; ++i) { if(counts[i] < 20) { pass = false; continue; }const double b = std::sqrt(before[i] / static_cast<double>(counts[i])); const double a = std::sqrt(after[i] / static_cast<double>(counts[i])); if(!(a <= 0.9 * b))pass = false; }
        return pass;
    }

    void model_calibration_run_gain() {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();JointVector inertia(n);JointImpedanceGains seed;
        HoldCompensationCfg disabled;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            gain_probe_origin_=robot_.get_joint_state().pos;
            gain_trial_original_servo_=robot_.get_config().ctrller.servo_tracking_gains;
            gain_trial_original_hold_compensation_=robot_.get_hold_compensation_cfg();
            disabled=*gain_trial_original_hold_compensation_;disabled.enabled=false;
            if(!robot_.set_hold_compensation_cfg(disabled))throw std::runtime_error("cannot isolate gain calibration from hold compensation");
            if(!dynamics_.is_updated())throw std::runtime_error("servo calibration needs a valid measured dynamics snapshot");
            const auto& matrix=dynamics_.get_mass_matrix();
            for(std::size_t i=0;i<n;++i){inertia[i]=matrix(i,i);if(!std::isfinite(inertia[i]) || inertia[i]<=0)throw std::runtime_error("invalid local effective inertia");}
            std::lock_guard<std::mutex> probe_lock(model_calibration_mutex_);
            const auto& configured=*gain_trial_original_servo_;
            const bool has_baseline=configured.kp.size()==n && configured.kd.size()==n &&
                std::all_of(configured.kp.begin(),configured.kp.end(),[](double value){return std::isfinite(value)&&value>0;}) &&
                std::all_of(configured.kd.begin(),configured.kd.end(),[](double value){return std::isfinite(value)&&value>0;});
            if(has_baseline) {
                for(std::size_t i=0;i<n;++i)
                    if(configured.kp[i]>cfg_.safety.limits.max_kp[i] || configured.kd[i]>cfg_.safety.limits.max_kd[i])
                        throw std::runtime_error("configured servo gain baseline exceeds joint safety limits at "+cfg_.joint_names[i]);
                // Reuse the operator's existing actuator baseline. Its legal
                // values are a starting point, not evidence of qualification.
                seed=configured;
                gain_seed_source_=runtime_gain_applied_ && (runtime_gain_category_=="all" || runtime_gain_category_=="servo_tracking") ?
                    "verified_runtime" : "configured_servo";
            } else {
                seed=model_calibration_physical_gains(4.0);
                gain_seed_source_="model_fallback";
            }
            gain_seed_=seed;
        }
        gain_probe_end(gain_seed_source_=="model_fallback" ? "model_seed" : "configured_baseline");
        // Measure the actual encoder/velocity codes while commanded stationary.
        // This is a local observation, not permission to raise Safety limits.
        gain_noise_.assign(n,NoiseEstimate{});gain_plans_.assign(n,gain_requirements_);
        std::vector<Requirements> noise_plans(n,gain_requirements_);
        for(auto& req:noise_plans){req.stationary_probe=true;req.probe_duration_s=6.0;req.tail_s=0;}
        for(std::size_t axis=0;axis<n;++axis) {
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_candidate_index_=0;}
            auto noise_trial=gain_response_trial(seed,JointGainProfile::SERVO_TRACKING,false,false,"noise-0",false,axis,&noise_plans,"noise");
            gain_noise_[axis]=estimate_noise(noise_trial.rows[axis]);
            noise_trial.plans[axis].encoder_step_rad=gain_noise_[axis].encoder_step_rad;
            noise_trial.plans[axis].noise_rms_rad=gain_noise_[axis].rms_rad;
            noise_trial.plans[axis].velocity_noise_floor=gain_noise_[axis].velocity_peak+1e-6;
            gain_probe_diagnostic(axis,seed,noise_trial.plans[axis],noise_trial,"noise_measured","servo-probe-noise-0-"+std::to_string(axis)+".csv");
            if(!gain_noise_[axis].stationary || !noise_trial.metrics[axis].continuous) {
                gain_probe_diagnostic(axis,seed,noise_trial.plans[axis],noise_trial,"unstable_noise","servo-probe-noise-0-"+std::to_string(axis)+".csv");
                throw GainQualificationFailure("gain stationary noise baseline is moving at "+cfg_.joint_names[axis]+"; inspect support and feedback before probing");
            }
        }
        const auto bounds_for=[&](std::size_t axis,const JointImpedanceGains& gains) {
            double margin=gain_requirements_.local_range_rad;
            if(cfg_.safety.limits.has_position_limit[axis])margin=std::min(margin,std::min(
                gain_probe_origin_[axis]-cfg_.safety.limits.min_pos[axis]-cfg_.safety.limits.pos_margin[axis],
                cfg_.safety.limits.max_pos[axis]-cfg_.safety.limits.pos_margin[axis]-gain_probe_origin_[axis]));
            return ProbeBounds{margin,.3*cfg_.safety.limits.max_vel[axis],.3*cfg_.safety.limits.max_acc[axis],
                .5*cfg_.safety.limits.max_effort[axis],gains.kp[axis],gains.kd[axis]};
        };
        const auto search_candidate=[&](JointImpedanceGains& gains,JointGainProfile profile,
                                        const JointVector& kp_limit,const JointVector& kd_limit,const std::string& label,
                                        ServoTrial* last_trial=nullptr) {
            // Motion discovery has a separate budget and never spends gain-search
            // attempts on an encoder which has not moved in both directions.
            for(std::size_t axis=0;axis<n;++axis) {
                bool established=false;double last_amplitude=0;std::string reason="no_motion";
                const double initial=std::max(gain_requirements_.amplitude_rad,
                    label=="soft"?gain_plans_[axis].amplitude_rad:gain_requirements_.amplitude_rad);
                for(std::size_t attempt=0;attempt<motion_attempt_limit;++attempt) {
                    auto plan=bounded_plan(gain_requirements_,std::min(gain_requirements_.local_range_rad,initial*std::pow(1.7,attempt)),profile==JointGainProfile::COMPLIANT?.13:.2,bounds_for(axis,gains));
                    if(attempt && plan.amplitude_rad<=last_amplitude+1e-7)break;
                    last_amplitude=plan.amplitude_rad;plan.motion_probe=true;
                    gain_plans_[axis]=plan;
                    {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_candidate_index_=attempt+1;}
                    const auto motion=gain_response_trial(gains,profile,false,false,label+"-motion-"+std::to_string(attempt),true,axis,&gain_plans_,"motion_discovery");
                    if(last_trial)*last_trial=motion;
                    reason=motion.motion[axis].reason;
                    if(motion.motion[axis].qualified) {
                        gain_noise_[axis].encoder_step_rad=std::max(gain_noise_[axis].encoder_step_rad,motion.motion[axis].encoder_step_rad);
                        gain_plans_[axis]=motion.plans[axis];gain_plans_[axis].motion_probe=false;
                        // The exact multifrequency input must also cross the
                        // deadband; a successful large sine alone is insufficient.
                        const auto spectrum=gain_response_trial(gains,profile,false,false,label+"-motion-spectrum-"+std::to_string(attempt),true,axis,&gain_plans_,"motion_discovery");
                        if(last_trial)*last_trial=spectrum;
                        reason=spectrum.motion[axis].reason;
                        if(spectrum.motion[axis].qualified && spectrum.models[axis].coherence>=.65){established=true;gain_plans_[axis]=spectrum.plans[axis];break;}
                        if(spectrum.motion[axis].qualified)reason="low_coherence";
                    }
                    if(reason=="limited" || reason=="insufficient_measurements")break;
                }
                if(!established)throw GainQualificationFailure("gain motion discovery failed at "+cfg_.joint_names[axis]+": "+reason+"; no qualified bidirectional motion within the confirmed local range");
                for(std::size_t attempt=0;attempt<gain_requirements_.max_candidates;++attempt) {
                    {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_candidate_index_=attempt+1;}
                    const auto trial=gain_response_trial(gains,profile,false,false,label+"-"+std::to_string(attempt),true,axis,&gain_plans_,"dynamic_search");
                    if(last_trial)*last_trial=trial;
                    if(trial.motion[axis].qualified && trial.metrics[axis].passed && trial.models[axis].tested_follow_frequency_hz>0)break;
                    if(trial.metrics[axis].measurement_samples<100 || !trial.metrics[axis].continuous)
                        throw std::runtime_error("gain probe has insufficient continuous measurements at "+cfg_.joint_names[axis]+"; check feedback before retrying");
                    {std::lock_guard<std::mutex> lock(model_calibration_mutex_);reason=gain_probe_message_;}
                    if(attempt+1>=gain_requirements_.max_candidates)
                        throw GainQualificationFailure("gain dynamic search failed at "+cfg_.joint_names[axis]+": "+reason+"; inspect the measured probe diagnostic");
                    // Low coherence and unilateral motion are not evidence that
                    // a larger Kp/Kd will help. They require a different probe.
                    if(!trial.motion[axis].qualified || trial.models[axis].coherence<.65)
                        throw GainQualificationFailure("gain dynamic search failed at "+cfg_.joint_names[axis]+": "+reason+"; inspect the measured probe diagnostic");
                    const auto candidate=paired_candidate({gains.kp,gains.kd},inertia,kp_limit,kd_limit,trial.metrics,trial.models,gain_requirements_);
                    if(std::abs(candidate.kp[axis]-gains.kp[axis])<=1e-9 && std::abs(candidate.kd[axis]-gains.kd[axis])<=1e-9) {
                        gain_probe_end("safety_bound");
                        throw GainQualificationFailure("gain dynamic search failed at "+cfg_.joint_names[axis]+": safety_bound; inspect the measured probe diagnostic");
                    }
                    gains={candidate.kp,candidate.kd};model_calibration_adaptation_revisions_.fetch_add(1);
                    gain_probe_end("candidate_updated");
                }
            }
        };
        model_calibration_gain_stage_.store(2);
        const bool standalone_compliant=model_calibration_gain_mode_=="compliant";
        // A standalone compliant candidate is qualified against the selected rigid
        // configuration, which its apply/export consumers also use as the bound
        JointImpedanceGains best=standalone_compliant?cfg_.ctrller.rigid_tracking_gains:seed;
        if(!standalone_compliant)
            search_candidate(best,JointGainProfile::SERVO_TRACKING,cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd,"base");
        // A frozen trial is independent from the samples used to select gains
        auto native=standalone_compliant?ServoTrial{}:gain_response_trial(best,JointGainProfile::SERVO_TRACKING,false,false,"native-frozen",false,std::nullopt,&gain_plans_,"frozen");
        if(!standalone_compliant && !native.passed)
            throw std::runtime_error("servo baseline failed independent frozen verification; later gain qualifications were not started");
        gain_native_plans_=native.plans;
        struct Stage {
            std::string category;JointImpedanceGains gains;ServoTrial native,admittance;
            GainHoldResult hold;std::optional<HoldCompensationCfg> compensation;std::optional<GainHoldResult> compensated_hold;
            std::optional<DragTrial> drag;bool validated{false};std::string error;
        };
        std::vector<Stage> stages;
        const bool all=model_calibration_gain_mode_=="all";
        const auto want=[&](const char* mode){return all || model_calibration_gain_mode_==mode;};
        std::exception_ptr interrupted;
        try {
        if(want("rigid")) {
            model_calibration_gain_stage_.store(0);Stage stage;stage.category="rigid_tracking";stage.gains=best;stage.native=native;stage.compensation=disabled;
            stage.hold=gain_hold_trial(best,false,disabled);
            // Only try bounded static trim where the baseline is stationary and has an observable offset
            if(stage.hold.samples>=30 && stage.hold.worst_velocity<=gain_requirements_.hold_velocity_rad_s && stage.hold.worst_error>0.0005) {
                auto trim=disabled;trim.enabled=true;trim.integral_gain.resize(n);trim.max_torque.resize(n);trim.max_rate.resize(n);trim.contact_threshold.resize(n);
                for(std::size_t i=0;i<n;++i) {
                    const double residual=std::abs(stage.hold.mean_error[i]*best.kp[i]);
                    trim.max_torque[i]=std::min(.05*cfg_.safety.limits.max_effort[i],std::max(.005*cfg_.safety.limits.max_effort[i],2*residual));
                    trim.max_rate[i]=trim.max_torque[i];trim.contact_threshold[i]=std::max(.02,.10*cfg_.safety.limits.max_effort[i]);
                    trim.integral_gain[i]=std::min(.25*best.kp[i],trim.max_torque[i]/std::max(.001,std::abs(stage.hold.mean_error[i])));
                }
                trim.velocity_threshold=gain_requirements_.hold_velocity_rad_s;trim.error_threshold=std::max(.01,2*gain_requirements_.hold_error_rad);
                auto checked=gain_hold_trial(best,false,trim);
                if(checked.validated && checked.worst_error<=.8*stage.hold.worst_error) {stage.compensation=trim;stage.compensated_hold=checked;}
                {std::lock_guard<std::mutex> lock(mutex_);(void)robot_.set_hold_compensation_cfg(disabled);}
            }
            stage.validated=stage.native.passed && (stage.hold.validated || (stage.compensated_hold && stage.compensated_hold->validated));
            stages.push_back(std::move(stage));
        }
        if(want("compliant")) {
            model_calibration_gain_stage_.store(1);Stage stage;stage.category="compliant_tracking";stage.gains=best;
            const auto bounds=serial_arm::calibration_replay_supervisor::compliant_bounds(best.kp,best.kd,cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd);
            if(!bounds.valid)throw std::runtime_error("cannot derive compliant limits from the measured rigid reference");
            const auto& configured_soft=cfg_.ctrller.compliant_tracking_gains;
            for(std::size_t i=0;i<n;++i) {
                const bool configured=configured_soft.kp.size()==n && configured_soft.kd.size()==n &&
                    std::isfinite(configured_soft.kp[i]) && configured_soft.kp[i]>0 &&
                    std::isfinite(configured_soft.kd[i]) && configured_soft.kd[i]>0;
                stage.gains.kp[i]=std::min(bounds.kp_ceiling[i],configured ? configured_soft.kp[i] : .4*best.kp[i]);
                stage.gains.kd[i]=std::min(bounds.kd_ceiling[i],configured ? configured_soft.kd[i] : 1.8*std::sqrt(inertia[i]*stage.gains.kp[i]));
            }
            try {
                search_candidate(stage.gains,JointGainProfile::COMPLIANT,bounds.kp_ceiling,bounds.kd_ceiling,"soft",&stage.native);
                stage.native=gain_response_trial(stage.gains,JointGainProfile::COMPLIANT,false,false,"soft-frozen",false,std::nullopt,&gain_plans_,"frozen");
                if(!stage.native.passed)throw GainQualificationFailure("compliant gains failed independent frozen verification");
                stage.hold=gain_hold_trial(stage.gains,true,disabled);
                stage.validated=stage.hold.validated && serial_arm::calibration_replay_supervisor::within_compliant_bounds(stage.gains.kp,stage.gains.kd,bounds);
            } catch(const GainQualificationFailure& error) {
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || machine_robot_state()!=RobotState::ACTIVE)throw;
                // A local soft-search failure does not discard independently
                // qualified rigid/servo uses in an all-purpose result.
                stage.error=error.what();stage.validated=false;
                if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))throw;
                gain_probe_end("failed");
            }
            stages.push_back(std::move(stage));
        }
        if(want("drag")) {
            model_calibration_gain_stage_.store(3);Stage stage;stage.category="compliant_drag";
            stage.gains.kp.assign(n,0);stage.gains.kd.resize(n);
            for(std::size_t i=0;i<n;++i) {
                const auto& configured=cfg_.ctrller.compliant_drag_gains.kd;
                const double damping=configured.size()==n && std::isfinite(configured[i]) && configured[i]>0 ?
                    configured[i] : std::max(.01,5.0*inertia[i]);
                stage.gains.kd[i]=std::min(cfg_.safety.limits.max_kd[i],damping);
            }
            try {
                for(std::size_t axis=0;axis<n;++axis) {
                    for(std::size_t attempt=0;attempt<gain_requirements_.max_candidates;++attempt) {
                        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_candidate_index_=attempt+1;}
                        stage.drag=gain_drag_trial(stage.gains,best,"drag-search-"+std::to_string(attempt),axis);
                        if(stage.drag->validated)break;
                        const double next=std::min(cfg_.safety.limits.max_kd[axis],stage.gains.kd[axis]*1.35);
                        if(attempt+1>=gain_requirements_.max_candidates || std::abs(next-stage.gains.kd[axis])<=1e-9)
                            throw GainQualificationFailure("drag damping did not qualify within its bounded search at "+cfg_.joint_names[axis]);
                        stage.gains.kd[axis]=next;model_calibration_adaptation_revisions_.fetch_add(1);
                    }
                }
                // Independent release windows use the selected damping only
                stage.drag=gain_drag_trial(stage.gains,best,"drag-frozen");
                stage.validated=stage.drag->validated;
            } catch(const GainQualificationFailure& error) {
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || machine_robot_state()!=RobotState::ACTIVE)throw;
                stage.error=error.what();stage.validated=false;
                if(!stage.drag){stage.drag=DragTrial{};stage.drag->decay_s.assign(n,0);stage.drag->initial_velocity.assign(n,0);}
                stage.drag->validated=false;
                if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))throw;
                gain_probe_end("failed");
            }
            stages.push_back(std::move(stage));
        }
        if(want("servo_tracking")) {
            model_calibration_gain_stage_.store(2);Stage stage;stage.category="servo_tracking";stage.gains=best;stage.native=native;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!robot_.set_servo_tracking_gains(best))throw std::runtime_error("cannot install session servo candidate for joint validation");
            }
            stage.admittance=gain_response_trial(best,JointGainProfile::SERVO_TRACKING,true,true,"admittance-frozen",false,std::nullopt,&native.plans,"frozen");
            stage.validated=stage.native.passed && stage.admittance.passed;stages.push_back(std::move(stage));
        }
        } catch(...) {
            interrupted=std::current_exception();
            if(stages.empty())std::rethrow_exception(interrupted);
            gain_probe_end(model_calibration_cancel_.load() ? "cancelled" : "failed");
            if(machine_robot_state()==RobotState::ACTIVE)(void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
        }
        const bool approved=!interrupted && !stages.empty() && std::all_of(stages.begin(),stages.end(),[](const auto& s){return s.validated;});
        try {
        std::ofstream out(std::filesystem::path(model_calibration_directory_)/"gain-candidate.json");
        if(!out)throw std::runtime_error("cannot save measured gain candidate");
        out<<std::setprecision(17);
        const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
        const auto gains_json=[&](const JointImpedanceGains& value){out<<"{\"kp\":";machine_json_joint_vector(out,value.kp);out<<",\"kd\":";machine_json_joint_vector(out,value.kd);out<<'}';};
        const auto hold_json=[&](const GainHoldResult& h){out<<"{\"validated\":"<<(h.validated?"true":"false")<<",\"samples\":"<<h.samples<<",\"worst_error_rad\":"<<h.worst_error<<",\"worst_drift_rad\":"<<h.worst_error<<",\"worst_velocity_rad_s\":"<<h.worst_velocity<<",\"raw_peak_velocity_rad_s\":"<<h.raw_worst_velocity<<",\"position_span_rad\":"<<h.worst_span<<",\"velocity_source\":\"position_window_with_stationary_velocity_codes\",\"target_source\":\"cycle.hold_target_pos\",\"target_pos\":";machine_json_joint_vector(out,h.target);out<<",\"disturbance_release\":\"pending\"}";};
        const auto validation_json=[&](const ServoTrial& trial) {
            out<<"{\"origin\":";machine_json_joint_vector(out,trial.center);out<<",\"probe_plans\":[";
            for(std::size_t i=0;i<trial.plans.size();++i){if(i)out<<',';const auto& plan=trial.plans[i];out<<"{\"amplitude_rad\":"<<plan.amplitude_rad<<",\"frequencies_hz\":["<<plan.frequencies_hz[0]<<','<<plan.frequencies_hz[1]<<','<<plan.frequencies_hz[2]<<"],\"duration_s\":"<<plan.probe_duration_s<<'}';}out<<"],\"validated\":"<<(trial.passed?"true":"false")<<",\"continuous\":"<<(std::all_of(trial.metrics.begin(),trial.metrics.end(),[](const auto& m){return m.continuous;})?"true":"false")<<",\"metrics\":[";
            for(std::size_t i=0;i<trial.metrics.size();++i){if(i)out<<',';const auto& m=trial.metrics[i];out<<"{\"samples\":"<<m.samples<<",\"rms_error_rad\":"<<m.rms_error<<",\"peak_error_rad\":"<<m.peak_error<<",\"velocity_rms_rad_s\":"<<m.velocity_rms<<",\"overshoot_rad\":"<<m.overshoot<<",\"settling_s\":"<<m.settling_s<<",\"tail_error_rad\":"<<m.tail_error_rad<<",\"limited_samples\":"<<m.limited_samples<<",\"oscillating\":"<<(m.oscillating?"true":"false")<<",\"observable\":"<<(m.observable?"true":"false")<<",\"reference_span_rad\":"<<m.reference_span<<",\"actual_span_rad\":"<<m.actual_span<<",\"response_rms_ratio\":"<<m.response_rms_ratio<<'}';}out<<"]}";
        };
        const auto write_stage=[&](const Stage& stage) {
            if(all)out<<"\"validated\":"<<(stage.validated?"true":"false")<<',';
            if(!stage.error.empty())out<<"\"error\":\""<<machine_json_escape(stage.error)<<"\",";
            out<<"\"kp\":";machine_json_joint_vector(out,stage.gains.kp);out<<",\"kd\":";machine_json_joint_vector(out,stage.gains.kd);
            out<<",\"qualification_scope\":{\"trajectory\":\"local_multifrequency\",\"external_tip\":\"pending\",\"disturbance_release\":\"pending\",\"contact\":\"not_tested\"}";
            if(stage.category=="compliant_drag") {
                out<<",\"drag_validation\":{\"validated\":"<<(stage.drag&&stage.drag->validated?"true":"false")<<",\"source\":\"bounded_velocity_release\",\"samples\":"<<(stage.drag?stage.drag->samples:0)<<",\"decay_s\":";machine_json_joint_vector(out,stage.drag->decay_s);out<<",\"initial_velocity_rad_s\":";machine_json_joint_vector(out,stage.drag->initial_velocity);out<<'}';return;
            }
            out<<",\"native_validation\":";validation_json(stage.native);
            out<<",\"observed_joint_min\":";machine_json_joint_vector(out,stage.native.min_position);out<<",\"observed_joint_max\":";machine_json_joint_vector(out,stage.native.max_position);
            if(stage.category=="rigid_tracking" || stage.category=="compliant_tracking") {
                out<<",\"hold_gains\":";gains_json(stage.gains);out<<",\"hold_verification\":";hold_json(stage.hold);
                if(stage.compensation) {
                    const auto& c=*stage.compensation;out<<",\"hold_compensation\":{\"enabled\":"<<(c.enabled?"true":"false")<<",\"integral_gain\":";machine_json_joint_vector(out,c.integral_gain);out<<",\"max_torque\":";machine_json_joint_vector(out,c.max_torque);out<<",\"max_rate\":";machine_json_joint_vector(out,c.max_rate);out<<",\"contact_threshold\":";machine_json_joint_vector(out,c.contact_threshold);out<<",\"velocity_threshold\":"<<c.velocity_threshold<<",\"error_threshold\":"<<c.error_threshold<<'}';if(stage.compensated_hold){out<<",\"hold_compensation_verification\":";hold_json(*stage.compensated_hold);}
                }
            }
            if(stage.category=="compliant_tracking") {
                out<<",\"tested_speed_fraction\":0.65,\"compliant_separation_validated\":true,\"rigid_reference_kp\":";machine_json_joint_vector(out,best.kp);out<<",\"rigid_reference_kd\":";machine_json_joint_vector(out,best.kd);
            }
            if(stage.category=="servo_tracking") {out<<",\"admittance_validation\":";validation_json(stage.admittance);}
            out<<",\"response_model\":[";
            for(std::size_t i=0;i<stage.native.models.size();++i) {
                if(i)out<<',';
                const auto& m=stage.native.models[i];out<<"{\"status\":\""<<(m.identifiable?"identified":m.points.empty()?"insufficient_data":"frequency_response_only")<<"\",\"natural_frequency_rad_s\":";
                if(m.identifiable)number(m.natural_frequency_rad_s);else out<<"null";
                out<<",\"damping_ratio\":";if(m.identifiable)number(m.damping_ratio);else out<<"null";
                out<<",\"effective_delay_s\":";if(m.identifiable)number(m.effective_delay_s);else out<<"null";
                out<<",\"relative_fit_error\":";number(m.relative_fit_error);out<<",\"heldout_fit_error\":";number(m.held_out_error);
                out<<",\"tested_follow_frequency_hz\":"<<m.tested_follow_frequency_hz<<",\"observed_cutoff_hz\":";number(m.observed_cutoff_hz);
                out<<",\"frequency_response\":[";for(std::size_t j=0;j<m.points.size();++j){if(j)out<<',';const auto& f=m.points[j];out<<"{\"hz\":"<<f.hz<<",\"gain\":"<<f.gain<<",\"phase_rad\":"<<f.phase_rad<<'}';}out<<"]}";
            }out<<']';
            const auto& residual=stage.category=="servo_tracking" ? stage.admittance : stage.native;
            out<<",\"observer_model\":[";
            for(std::size_t i=0;i<residual.metrics.size();++i){if(i)out<<',';const auto& m=residual.metrics[i];out<<"{\"bias_nm\":";if(m.residual_samples>=30)number(m.residual_bias);else out<<"null";out<<",\"noise_rms_nm\":";if(m.residual_samples>=30)number(m.residual_noise_rms);else out<<"null";out<<",\"samples\":"<<m.residual_samples<<",\"source\":\"no_contact_residual\"}";}out<<']';
            if(stage.category=="servo_tracking") {
                out<<",\"feasible_admittance\":[";
                for(std::size_t i=0;i<n;++i) {
                    if(i)out<<',';
                    const double f=stage.native.models[i].tested_follow_frequency_hz/5.0;
                    const auto& m=stage.admittance.metrics[i];const bool available=f>0 && m.residual_samples>=30;
                    out<<"{\"status\":\""<<(available?"local_no_contact_bound":"insufficient_data")<<"\",\"outer_frequency_max_hz\":";if(available)number(f);else out<<"null";
                    out<<",\"mass_min\":";if(available)number(std::max(cfg_.capability.admittance.controller.stiffness[i]/std::pow(2*pi*f,2),cfg_.capability.admittance.controller.damping[i]/(2*pi*f)));else out<<"null";
                    out<<",\"deadband_nm\":";if(available)number(3*m.residual_noise_rms);else out<<"null";
                    out<<",\"basis\":\"measured_follow_frequency_ratio_and_residual_noise\",\"contact_validation\":\"not_tested\"}";
                }out<<']';
            }
        };
        out<<"{\"schema\":\"serial_arm_gain_candidate\",\"task_id\":\""<<machine_json_escape(model_calibration_task_id_)<<"\",\"urdf_fingerprint\":\""<<machine_json_escape(model_calibration_urdf_fingerprint_)<<"\",\"validated\":"<<(approved?"true":"false")<<",\"category\":\""<<(all?"all":stages.front().category)<<"\",\"joint_names\":[";
        for(std::size_t i=0;i<n;++i){if(i)out<<',';out<<'"'<<machine_json_escape(cfg_.joint_names[i])<<'"';}out<<"],\"passes\":"<<model_calibration_adaptation_passes_.load()<<",\"revisions\":"<<model_calibration_adaptation_revisions_.load()<<",\"validation_scope\":\"frozen_continuous_local_multifrequency\",\"load_description\":\""<<machine_json_escape(model_calibration_options_.load_description)<<"\",\"seed_posture\":";machine_json_joint_vector(out,native.center.empty()?stages.front().native.center:native.center);
        const auto& r=gain_requirements_;
        out<<",\"gain_requirements\":{\"peak_error_rad\":"<<r.peak_error_rad<<",\"rms_error_rad\":"<<r.rms_error_rad<<",\"velocity_rms_rad_s\":"<<r.velocity_rms_rad_s<<",\"overshoot_rad\":"<<r.overshoot_rad<<",\"settling_error_rad\":"<<r.settling_error_rad<<",\"settling_time_s\":"<<r.settling_time_s<<",\"hold_error_rad\":"<<r.hold_error_rad<<",\"hold_velocity_rad_s\":"<<r.hold_velocity_rad_s<<",\"amplitude_rad\":"<<r.amplitude_rad<<",\"local_range_rad\":"<<r.local_range_rad<<"}";
        out<<",\"probe_origin\":";machine_json_joint_vector(out,gain_probe_origin_);
        out<<",\"established_probe_plans\":[";for(std::size_t i=0;i<gain_plans_.size();++i){if(i)out<<',';const auto& plan=gain_plans_[i];out<<"{\"amplitude_rad\":"<<plan.amplitude_rad<<",\"frequencies_hz\":["<<plan.frequencies_hz[0]<<','<<plan.frequencies_hz[1]<<','<<plan.frequencies_hz[2]<<"],\"duration_s\":"<<plan.probe_duration_s<<'}';}out<<']';
        out<<",\"seed_source\":\""<<machine_json_escape(gain_seed_source_)<<"\",\"seed_kp\":";machine_json_joint_vector(out,gain_seed_.kp);
        out<<",\"seed_kd\":";machine_json_joint_vector(out,gain_seed_.kd);
        out<<",\"partial\":"<<(interrupted?"true":"false");
        out<<",\"admittance_parameters\":{\"mass\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.mass);
        out<<",\"damping\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.damping);out<<",\"stiffness\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.stiffness);
        out<<",\"joint_enabled\":";machine_json_vector(out,cfg_.capability.admittance.joint_enabled);out<<",\"observer_mode\":\""<<observer_mode_name(cfg_.capability.admittance.observer.mode)<<"\"}";
        if(all){out<<",\"targets\":{";for(std::size_t i=0;i<stages.size();++i){if(i)out<<',';out<<'"'<<stages[i].category<<"\":{";write_stage(stages[i]);out<<'}';}out<<'}';}else{out<<',';write_stage(stages.front());}out<<'}';out.flush();if(!out)throw std::runtime_error("gain candidate writing failed");
        model_calibration_adaptation_converged_.store(approved);
        } catch(...) {
            // File I/O must never replace the original cancellation or control fault.
            if(interrupted)std::rethrow_exception(interrupted);
            throw;
        }
        if(interrupted)std::rethrow_exception(interrupted);
        if(!approved)throw std::runtime_error("gain candidate failed continuous measured qualification; inspect gain-candidate.json and probe CSV files");
    }

    void model_calibration_worker() {
        try {
            if(model_calibration_workflow_kind_=="gain") {
                model_calibration_run_gain();
                model_calibration_restore_runtime_hold();
                model_calibration_recorder_.stop();
                const auto recording=model_calibration_recorder_.status();
                if(recording.write_failed || recording.data_gap || recording.accepted_frames!=recording.written_frames)
                    throw std::runtime_error(recording.error.empty()?"gain recording did not finish writing":recording.error);
                gain_probe_end("complete");
                {
                    std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                    model_calibration_completed_units_=model_calibration_adaptation_passes_.load();
                    model_calibration_total_units_=model_calibration_completed_units_;
                    model_calibration_progress_=1.0;
                }
                model_calibration_set_phase(ModelCalibrationPhase::COMPLETE);
                model_calibration_write_result_file();return;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                model_calibration_active_tracking_gains_=robot_.get_config().ctrller.servo_tracking_gains;
                if(!robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING))
                    throw std::runtime_error("cannot select calibrated servo gains for identification");
            }
            // Formal identification: both static approach directions followed by
            // two independently tagged dynamic speed pairs on the checked path.
            model_calibration_set_phase(ModelCalibrationPhase::STATIC_REVERSE);
            if(!model_calibration_static_pass(true))
                throw std::runtime_error(model_calibration_error_.empty() ? "reverse static replay failed or cancelled" : model_calibration_error_);
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("failed to return to RIGID_HOLD");
            model_calibration_set_phase(ModelCalibrationPhase::STATIC_FORWARD);
            if(!model_calibration_static_pass(false))
                throw std::runtime_error(model_calibration_error_.empty() ? "forward static replay failed or cancelled" : model_calibration_error_);
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("failed to hold after forward static replay");
            model_calibration_set_phase(ModelCalibrationPhase::GRAVITY_FITTING);
            const auto recorder_status = model_calibration_recorder_.status();
            if(recorder_status.data_gap || recorder_status.write_failed)
                throw std::runtime_error(recorder_status.error.empty() ? "model calibration raw data contains a recording gap" : recorder_status.error);
            const auto groups = group_static_calibration_frames(model_calibration_recorder_.approved_static_frames(), 30);
            GravityCalibrationOptions options;
            options.minimum_training_groups = model_calibration_options_.minimum_training_groups;
            options.regularization = model_calibration_options_.regularization;
            options.svd_relative_threshold = model_calibration_options_.svd_relative_threshold;
            options.identification_mode = model_calibration_options_.identification_mode;
            options.locked_links = model_calibration_options_.locked_links;
            Dynamics preview_dynamics;
            const auto preview_configured=preview_dynamics.configure(cfg_.dynamics);
            const auto gravity = preview_configured ?
                fit_gravity_calibration(preview_dynamics, groups, model_calibration_original_gravity_scale_, options) :
                tl::expected<GravityCalibrationResult,std::string>(tl::make_unexpected(std::string("offline preview model configuration failed")));
            if(gravity)model_calibration_result_=gravity.value();
            else {
                // A failed quick static preview must not discard the remaining
                // formal dynamics observations. Hardware failures still abort.
                model_calibration_result_.reset();
                std::lock_guard<std::mutex> preview_lock(model_calibration_mutex_);
                model_calibration_preview_error_=gravity.error();
            }
            // Holdout validation is advisory for exported candidates. Never
            // abort data acquisition or suppress the candidate because of
            // an inaccurate initial URDF. Applying it to hardware is separate.
            if(!model_calibration_within_time_limit()) throw std::runtime_error(model_calibration_error_);
            {
                std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                ++model_calibration_completed_units_;
                model_calibration_progress_ = static_cast<double>(model_calibration_completed_units_) / static_cast<double>(model_calibration_total_units_);
            }
            for(const auto& pass : serial_arm::calibration_acquisition::dynamic_passes) {
                const auto phase = pass.speed_tier==1 ?
                    (pass.reverse ? ModelCalibrationPhase::FRICTION_REVERSE_SLOW : ModelCalibrationPhase::FRICTION_FORWARD_SLOW) :
                    (pass.reverse ? ModelCalibrationPhase::FRICTION_REVERSE_FAST : ModelCalibrationPhase::FRICTION_FORWARD_FAST);
                model_calibration_set_phase(phase);
                if(!model_calibration_dynamic_pass(pass.reverse,model_calibration_replay_rate_*pass.rate,pass.speed_tier,phase))
                    throw std::runtime_error(model_calibration_error_.empty() ? "dynamic direction/speed replay failed or cancelled" : model_calibration_error_);
            }
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
                throw std::runtime_error("failed to hold after replay");
            // Acquisition ends in the confirmed hold above. No full-history copy
            // or archived-state update of the LIVE dynamics object while ACTIVE.
            // Friction and full IDIM belong to isolated offline identification.
            model_calibration_friction_result_.reset();
            model_calibration_friction_pass_=false;
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
            model_calibration_recorder_.stop();
            const auto final_recording=model_calibration_recorder_.status();
            if(final_recording.write_failed || final_recording.data_gap ||
               final_recording.accepted_frames!=final_recording.written_frames)
                throw std::runtime_error(final_recording.error.empty() ? "calibration recording did not finish writing" : final_recording.error);
            model_calibration_set_phase(ModelCalibrationPhase::COMPLETE);
            model_calibration_write_result_file();
        }
        catch(const std::exception& error) {
            if(model_calibration_workflow_kind_=="gain")
                gain_probe_end(model_calibration_cancel_.load() ? "cancelled" : "failed");
            {
                std::lock_guard<std::mutex> error_lock(model_calibration_mutex_);
                if(!model_calibration_fault_interrupted_.load() || model_calibration_error_.empty())
                    model_calibration_error_ = error.what();
            }
            model_calibration_pose_group_.store(0);
            model_calibration_validation_.store(false);
            model_calibration_direction_.store(0);
            model_calibration_speed_tier_.store(0);
            if(machine_robot_state() == RobotState::ACTIVE)
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
        if(model_calibration_phase_.load() != ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
            model_calibration_source_task_id_.empty() || !model_calibration_trajectory_)
            throw std::runtime_error("alignment is only supported for a staged imported demonstration");
        if(model_calibration_alignment_active_.load()) throw std::runtime_error("manual endpoint alignment is already active");
        if(!params || !params["supported"] || !params["supported"].as<bool>())
            throw std::runtime_error("operator must confirm the arm is physically supported and the work area is clear");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE || !last_output_ || feedback_time_ns_ == 0)
                throw std::runtime_error("endpoint alignment requires ACTIVE robot and fresh joint feedback");
            const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            if(now_ns<feedback_time_ns_ || double(now_ns - feedback_time_ns_) / 1.0e6>serial_arm::calibration_recovery::kFeedbackMaxAgeMs)
                throw std::runtime_error("joint feedback is stale; manual alignment is not permitted");
            const auto& types=dynamics_.get_info().joint_types;
            for(std::size_t i=0;i<last_output_->joint_state.vel.size();++i) {
                const auto v=last_output_->joint_state.vel[i];
                if(!std::isfinite(v) || std::abs(v)>serial_arm::calibration_recovery::velocity_tolerance(types.at(i)))
                    throw std::runtime_error("arm must be stationary before enabling manual alignment at "+cfg_.joint_names.at(i));
            }
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG))
            throw std::runtime_error("could not enter compliant drag for manual alignment");
        model_calibration_alignment_active_.store(true);
    }
    void machine_model_calibration_alignment_finish(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load() != ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
            !model_calibration_alignment_active_.load())
            throw std::runtime_error("manual endpoint alignment is not active");
        if(machine_robot_state() != RobotState::ACTIVE)
            throw std::runtime_error("robot is not ACTIVE; cannot finish manual alignment");
        // Only engage the stronger hold after the operator has stopped moving;
        // the pose itself may still be far from the saved endpoint.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(!last_output_ || feedback_time_ns_ == 0)
                throw std::runtime_error("cannot stop manual alignment: live joint feedback is unavailable");
            const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            if(now_ns<feedback_time_ns_ || double(now_ns - feedback_time_ns_) / 1.0e6>serial_arm::calibration_recovery::kFeedbackMaxAgeMs)
                throw std::runtime_error("cannot stop manual alignment: joint feedback is stale");
            const auto& types=dynamics_.get_info().joint_types;
            for(std::size_t i=0;i<last_output_->joint_state.vel.size();++i) {
                const auto v=last_output_->joint_state.vel[i];
                if(!std::isfinite(v) || std::abs(v)>serial_arm::calibration_recovery::velocity_tolerance(types.at(i)))
                    throw std::runtime_error("stop moving and physically support the arm before switching to RIGID_HOLD at "+cfg_.joint_names.at(i));
            }
        }
        // Hold the ACTUAL current pose, regardless of alignment outcome.
        // A non-matching pose remains blocked from starting replay.
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD))
            throw std::runtime_error("failed to enter rigid hold after manual alignment");
        model_calibration_alignment_active_.store(false);
    }

    void machine_model_calibration_finish_teaching(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_phase_.load()!=ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)
            throw std::runtime_error("please stop demonstration and save its trajectory first");
        if(model_calibration_alignment_active_.load())
            throw std::runtime_error("finish manual alignment before saving demonstration");
        model_calibration_workflow_kind_="teaching";
        model_calibration_recorder_.stop();
        model_calibration_restore_runtime_hold();
        model_calibration_set_phase(ModelCalibrationPhase::COMPLETE);
        model_calibration_write_result_file();
    }

    std::size_t gain_planned_windows() const {
        const auto n=cfg_.joint_names.size(),attempts=gain_requirements_.max_candidates;
        std::size_t windows=(2*serial_arm::servo_tuning::motion_attempt_limit+attempts+2)*n;
        const bool all=model_calibration_gain_mode_=="all";
        if(all || model_calibration_gain_mode_=="compliant")windows+=(2*serial_arm::servo_tuning::motion_attempt_limit+attempts+1)*n+1;
        if(all || model_calibration_gain_mode_=="rigid")windows+=2;
        if(all || model_calibration_gain_mode_=="servo_tracking")windows+=n;
        if(all || model_calibration_gain_mode_=="drag")windows+=(attempts+1)*n;
        return windows;
    }

    void machine_model_calibration_start(const YAML::Node& params) {
        model_calibration_require_task(params);
        model_calibration_workflow_kind_ = params && params["workflow"] ? params["workflow"].as<std::string>() : "gravity";
        if(model_calibration_workflow_kind_!="gravity" && model_calibration_workflow_kind_!="gain")
            throw std::runtime_error("invalid calibration workflow target");
        if(model_calibration_workflow_kind_=="gain" && std::any_of(dynamics_.get_info().joint_types.begin(),dynamics_.get_info().joint_types.end(),[](const auto& type){return type=="prismatic";}))
            throw std::runtime_error("automatic actuator gain calibration currently supports rotary joints only; prismatic axes require per-axis linear probe and acceptance requirements");
        model_calibration_gain_mode_=params && params["gain_mode"] ? params["gain_mode"].as<std::string>() : "all";
        if(model_calibration_gain_mode_=="identification" || model_calibration_gain_mode_=="identification_tracking" || model_calibration_gain_mode_=="servo")model_calibration_gain_mode_="servo_tracking";
        if(model_calibration_gain_mode_=="compliant_drag")model_calibration_gain_mode_="drag";
        model_calibration_gain_stage_.store(model_calibration_gain_mode_=="compliant"?1:model_calibration_gain_mode_=="drag"?3:2);
        if(model_calibration_workflow_kind_=="gain" && model_calibration_gain_mode_!="rigid" && model_calibration_gain_mode_!="compliant" && model_calibration_gain_mode_!="drag" && model_calibration_gain_mode_!="servo_tracking" && model_calibration_gain_mode_!="all")throw std::runtime_error("unknown gain calibration purpose");
        gain_requirements_=serial_arm::servo_tuning::Requirements{};
        if(params["gain_requirements"]) {
            const auto r=params["gain_requirements"];
            const auto read=[&](const char* key,double& value,double maximum){if(!r[key])return;value=r[key].as<double>();if(!std::isfinite(value)||value<=0||value>maximum)throw std::runtime_error(std::string("invalid gain requirement: ")+key);};
            read("peak_error_rad",gain_requirements_.peak_error_rad,.04);read("rms_error_rad",gain_requirements_.rms_error_rad,.02);
            read("velocity_rms_rad_s",gain_requirements_.velocity_rms_rad_s,.20);read("overshoot_rad",gain_requirements_.overshoot_rad,.02);
            read("settling_error_rad",gain_requirements_.settling_error_rad,.02);read("settling_time_s",gain_requirements_.settling_time_s,2.0);
            read("hold_error_rad",gain_requirements_.hold_error_rad,.03);read("hold_velocity_rad_s",gain_requirements_.hold_velocity_rad_s,.10);
            read("amplitude_rad",gain_requirements_.amplitude_rad,.02);
            read("local_range_rad",gain_requirements_.local_range_rad,.04);
        }
        if(model_calibration_workflow_kind_=="gain" && (!params["gain_motion_range_confirmed"] || !params["gain_motion_range_confirmed"].as<bool>()))
            throw std::runtime_error("explicit gain local motion range confirmation is required");
        if(model_calibration_phase_.load() != ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION || !model_calibration_trajectory_)throw std::runtime_error("model calibration is not waiting for replay confirmation");
        if(model_calibration_alignment_active_.load())
            throw std::runtime_error("finish manual endpoint alignment and hold before confirming replay");
        if(!params["released"] || !params["released"].as<bool>())throw std::runtime_error("explicit released confirmation is required");
        if(!model_calibration_source_task_id_.empty()) {
            // The first pass traverses the path in reverse, starting at its last
            // sample. A parked arm can be far away: NEVER jump to that target.
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE || !last_output_ || feedback_time_ns_ == 0)
                throw std::runtime_error("cannot confirm imported replay without fresh ACTIVE robot feedback");
            const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            const double age_ms = now_ns >= feedback_time_ns_ ? double(now_ns - feedback_time_ns_) / 1.0e6 : std::numeric_limits<double>::infinity();
            const auto& start = model_calibration_trajectory_->positions.back();
            const auto& actual = last_output_->joint_state;
            const auto choice = serial_arm::calibration_recovery::select_endpoint(actual.pos, actual.vel, start, start, age_ms, dynamics_.get_info().joint_types);
            if(!choice.valid) throw std::runtime_error("replay blocked: joint feedback is stale or does not match the imported trajectory");
            if(!choice.at_endpoint) {
                std::ostringstream message;
                message << std::fixed << std::setprecision(4)
                    << "imported replay start pose mismatch (position tolerance ratio=" << choice.max_position_ratio
                    << "; velocity tolerance ratio=" << choice.max_velocity_ratio
                    << "; allowed per axis: rotary 0.08 rad / 0.05 rad/s, prismatic 0.001 m / 0.001 m/s). Use guided endpoint alignment, then reconfirm.";
                throw std::runtime_error(message.str());
            }
        }
        if(model_calibration_worker_.joinable())throw std::runtime_error("model calibration worker is already running");
        model_calibration_fault_interrupted_.store(false); model_calibration_cancel_.store(false); model_calibration_pause_.store(false); model_calibration_resume_confirmed_.store(false);
        if(model_calibration_workflow_kind_!="gain" && model_calibration_trajectory_ && model_calibration_trajectory_->positions.size()>=2) {
            const double one_way=model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1);
            const double dwell=2.0*static_cast<double>(model_calibration_pose_targets_.size())*
                (model_calibration_options_.static_hold_s+model_calibration_options_.static_sample_s+.5);
            model_calibration_estimated_duration_s_=serial_arm::calibration_acquisition::traversal_factor*one_way+dwell+2.0;
        }
        if(model_calibration_workflow_kind_=="gain") {
            model_calibration_adaptation_passes_.store(0);
            model_calibration_adaptation_revisions_.store(0);
            model_calibration_update_progress(0,gain_planned_windows());
            const double axes=static_cast<double>(cfg_.joint_names.size());
            const double probe=axes*(gain_requirements_.probe_duration_s+gain_requirements_.tail_s+.4);
            const double candidates=static_cast<double>(gain_requirements_.max_candidates);
            model_calibration_estimated_duration_s_=axes*6.4+(2*serial_arm::servo_tuning::motion_attempt_limit+candidates+1)*axes*13.3;
            if(model_calibration_gain_mode_=="all" || model_calibration_gain_mode_=="compliant")model_calibration_estimated_duration_s_+=(2*serial_arm::servo_tuning::motion_attempt_limit+candidates+1)*axes*18.7+2.1;
            if(model_calibration_gain_mode_=="all" || model_calibration_gain_mode_=="rigid")model_calibration_estimated_duration_s_+=4.2;
            if(model_calibration_gain_mode_=="all" || model_calibration_gain_mode_=="servo_tracking")model_calibration_estimated_duration_s_+=probe;
            if(model_calibration_gain_mode_=="all" || model_calibration_gain_mode_=="drag")model_calibration_estimated_duration_s_+=(candidates+1)*axes*7.0;
        }
        model_calibration_started_at_ = Robot::Clock::now();
        model_calibration_set_phase(ModelCalibrationPhase::GAIN_ADAPTATION);
        model_calibration_worker_=std::thread([this]() {model_calibration_worker(); });
    }

    void machine_model_calibration_pause(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(!model_calibration_task_active() || model_calibration_phase_.load() == ModelCalibrationPhase::TEACHING || model_calibration_phase_.load() == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION)throw std::runtime_error("model calibration task is not in a pausable stage");
        model_calibration_pause_.store(true); model_calibration_cv_.notify_all();
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
        double max_ratio=0;const auto& types=dynamics_.get_info().joint_types;
        for(std::size_t i=0;i<current.size();++i) {
            if(!std::isfinite(current[i]))throw std::runtime_error("non-finite joint feedback; resume is not allowed");
            max_ratio=std::max(max_ratio,std::abs(current[i]-model_calibration_pause_reference_[i])/serial_arm::calibration_recovery::resume_tolerance(types.at(i)));
        }
        if(max_ratio > 1.0 && (!params || !params["confirmed"] || !params["confirmed"].as<bool>())) { model_calibration_resume_requires_confirmation_ = true; throw std::runtime_error("robot position changed while paused; explicit resume confirmation is required"); }
        model_calibration_resume_requires_confirmation_ = false; model_calibration_resume_confirmed_.store(true); model_calibration_pause_.store(false); model_calibration_cv_.notify_all();
    }

    void machine_model_calibration_cancel_request(const YAML::Node& params) {
        model_calibration_require_task(params);
        machine_model_calibration_cancel();
    }

    void machine_model_calibration_cancel() {
        const auto phase = model_calibration_phase_.load();
        if(phase == ModelCalibrationPhase::FAILED || phase == ModelCalibrationPhase::CANCELLED) {
            if(model_calibration_teach_thread_.joinable()) model_calibration_finish_aborted_teaching();
            return;
        }
        if(phase == ModelCalibrationPhase::IDLE || phase == ModelCalibrationPhase::COMPLETE) return;
        model_calibration_cancel_.store(true);
        model_calibration_teach_stop_.store(true);
        model_calibration_pause_.store(false);
        model_calibration_cv_.notify_all();
        cycle_cv_.notify_all();
        if(phase == ModelCalibrationPhase::TEACHING || phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION) {
            if(model_calibration_teach_thread_.joinable()) model_calibration_teach_thread_.join();
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_set_phase(model_calibration_fault_interrupted_.load() ?
                ModelCalibrationPhase::FAILED : ModelCalibrationPhase::CANCELLED);
            model_calibration_finish_aborted_teaching();
        }
    }

    void model_calibration_write_result_file() const {
        if(model_calibration_directory_.empty()) return;
        std::ofstream out(std::filesystem::path(model_calibration_directory_) / "result.json");
        if(!out) return;
        out << machine_model_calibration_status_json() << '\n';
    }

    void machine_require_safe_candidate_state() const {
        if(model_calibration_task_active() || machine_calibration_kind_ != "none")
            throw std::runtime_error("finish calibration before switching runtime candidates");
        if(robot_.get_state()!=RobotState::INACTIVE)
            throw std::runtime_error("candidate switch requires INACTIVE robot: park safely first");
    }

    std::string machine_candidate_overrides_json() const {
        std::ostringstream out;
        out << "{\"gain_applied\":" << (runtime_gain_applied_?"true":"false")
            << ",\"gravity_applied\":" << (runtime_gravity_applied_?"true":"false")
            << ",\"dynamics_applied\":" << (runtime_dynamics_applied_?"true":"false")
            << ",\"gain_purposes\":[";
        for(std::size_t i=0;i<runtime_gain_purposes_.size();++i){if(i)out<<',';out<<'"'<<runtime_gain_purposes_[i]<<'"';}
        out << "]" << ",\"gain_candidate\":\"" << machine_json_escape(runtime_gain_candidate_) << "\""
            << ",\"gravity_candidate\":\"" << machine_json_escape(runtime_gravity_candidate_) << "\""
            << ",\"dynamics_candidate\":\"" << machine_json_escape(runtime_dynamics_candidate_) << "\"}";
        return out.str();
    }

    JointImpedanceGains machine_saved_mode_gains(JointImpedanceMode mode) const {
        switch(mode) {
            case JointImpedanceMode::RIGID_HOLD: return cfg_.ctrller.rigid_hold_gains;
            case JointImpedanceMode::RIGID_TRACKING: return cfg_.ctrller.rigid_tracking_gains;
            case JointImpedanceMode::COMPLIANT_HOLD: return cfg_.ctrller.compliant_hold_gains;
            case JointImpedanceMode::COMPLIANT_TRACKING: return cfg_.ctrller.compliant_tracking_gains;
            case JointImpedanceMode::COMPLIANT_DRAG: return cfg_.ctrller.compliant_drag_gains;
            default: throw std::runtime_error("unsupported calibrated gain mode");
        }
    }

    void machine_assign_mode_gains(JointImpedanceMode mode,const JointImpedanceGains& gains) {
        switch(mode) {
            case JointImpedanceMode::RIGID_HOLD: cfg_.ctrller.rigid_hold_gains=gains; break;
            case JointImpedanceMode::RIGID_TRACKING: cfg_.ctrller.rigid_tracking_gains=gains; break;
            case JointImpedanceMode::COMPLIANT_HOLD: cfg_.ctrller.compliant_hold_gains=gains; break;
            case JointImpedanceMode::COMPLIANT_TRACKING: cfg_.ctrller.compliant_tracking_gains=gains; break;
            case JointImpedanceMode::COMPLIANT_DRAG: cfg_.ctrller.compliant_drag_gains=gains; break;
            default: throw std::runtime_error("unsupported calibrated gain mode");
        }
    }

    void machine_candidate_gain_apply(const YAML::Node& params) {
        if(!params || !params["path"])throw std::runtime_error("gain candidate path is required");
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(runtime_gain_applied_)throw std::runtime_error("restore existing gain candidate first");
        const auto path=std::filesystem::canonical(params["path"].as<std::string>());
        const auto data=YAML::LoadFile(path.string());
        if(!data || !data["schema"] || data["schema"].as<std::string>()!="serial_arm_gain_candidate")
            throw std::runtime_error("invalid gain candidate schema");
        const bool explicit_selection=params["gain_purposes"].IsDefined();
        if(!explicit_selection && (!data["validated"] || !data["validated"].as<bool>()))
            throw std::runtime_error("gain candidate is partially qualified; explicitly select passed gain_purposes");
        if(!data["urdf_fingerprint"] ||
           data["urdf_fingerprint"].as<std::string>()!=model_calibration_file_fingerprint(cfg_.dynamics.urdf_path))
            throw std::runtime_error("gain candidate URDF fingerprint mismatch");
        if(!data["joint_names"] || data["joint_names"].as<std::vector<std::string>>()!=cfg_.joint_names)
            throw std::runtime_error("gain candidate joint mapping mismatch");
        if(!data["category"])throw std::runtime_error("gain candidate mode is missing");
        const auto category=data["category"].as<std::string>();
        if(category=="identification_tracking")throw std::runtime_error("legacy identification gain candidates are read-only; rerun native and admittance qualification");
        std::vector<std::pair<JointImpedanceMode,JointImpedanceGains>> updates;
        std::optional<JointImpedanceGains> identification;
        std::optional<HoldCompensationCfg> hold_compensation;
        std::optional<JointImpedanceGains> candidate_rigid;
        const auto collect=[&](const std::string& target,const YAML::Node& node) {
            if(!node || !node.IsMap() || !node["validated"] || !node["validated"].as<bool>())
                throw std::runtime_error("gain target was not validated");
            if((target=="rigid_tracking" || target=="compliant_tracking") && (!node["hold_gains"] || !node["hold_gains"].IsMap()))
                throw std::runtime_error("gain candidate requires independently verified hold gains");
            JointImpedanceGains gains{node["kp"].as<JointVector>(),node["kd"].as<JointVector>()};
            const auto count=cfg_.joint_names.size();
            if(gains.kp.size()!=count || gains.kd.size()!=count)
                throw std::runtime_error("gain candidate dimensions do not match robot");
            for(std::size_t i=0;i<count;++i)
                if(!std::isfinite(gains.kp[i]) || !std::isfinite(gains.kd[i]) ||
                   gains.kp[i]<0 || gains.kd[i]<0 || gains.kp[i]>cfg_.safety.limits.max_kp[i] ||
                   gains.kd[i]>cfg_.safety.limits.max_kd[i])
                    throw std::runtime_error("gain candidate violates motor safety limits");
            if(target=="compliant_tracking" &&
               (!node["compliant_separation_validated"] ||
                !node["compliant_separation_validated"].as<bool>() ||
                !node["hold_verification"] ||
                !node["hold_verification"]["validated"] ||
                !node["hold_verification"]["validated"].as<bool>() ||
                !node["tested_speed_fraction"] ||
                node["tested_speed_fraction"].as<double>()<=0.0))
                throw std::runtime_error("legacy or incomplete compliant gain candidate requires a new compliant calibration");
            if(target=="compliant_tracking") {
                const auto& rigid=candidate_rigid ? *candidate_rigid : cfg_.ctrller.rigid_tracking_gains;
                if(!node["rigid_reference_kp"] || !node["rigid_reference_kd"] ||
                   !node["rigid_reference_kp"].IsSequence() || !node["rigid_reference_kd"].IsSequence())
                    throw std::runtime_error("compliant candidate lacks rigid reference values");
                const auto ref_kp=node["rigid_reference_kp"].as<JointVector>();
                const auto ref_kd=node["rigid_reference_kd"].as<JointVector>();
                if(ref_kp.size()!=count || ref_kd.size()!=count)throw std::runtime_error("compliant rigid reference dimension mismatch");
                for(std::size_t i=0;i<count;++i) {
                    if(!std::isfinite(ref_kp[i]) || !std::isfinite(ref_kd[i]) ||
                       std::abs(rigid.kp[i]-ref_kp[i])>1e-5 ||
                       std::abs(rigid.kd[i]-ref_kd[i])>1e-5 ||
                       gains.kp[i]>0.65*ref_kp[i]+1e-6 || gains.kd[i]>0.85*ref_kd[i]+1e-6)
                        throw std::runtime_error("compliant candidate does not match the currently selected rigid gains");
                }
            }
            if(target=="rigid_tracking") {
                candidate_rigid=gains;
                const bool trim_verified=node["hold_compensation"] && node["hold_compensation_verification"] && node["hold_compensation_verification"]["validated"].as<bool>(false);
                const bool baseline_verified=node["hold_verification"] && node["hold_verification"]["validated"].as<bool>(false);
                if(!baseline_verified && !trim_verified)throw std::runtime_error("rigid gain candidate requires hold or compensated-hold qualification");
                if(node["hold_gains"] && (baseline_verified || trim_verified))
                    updates.emplace_back(JointImpedanceMode::RIGID_HOLD,JointImpedanceGains{node["hold_gains"]["kp"].as<JointVector>(),node["hold_gains"]["kd"].as<JointVector>()});
                if(node["hold_compensation"]) {
                    const auto c=node["hold_compensation"];auto trim=robot_.get_hold_compensation_cfg();
                    trim.enabled=c["enabled"].as<bool>();
                    if((trim.enabled && !trim_verified) || (!trim.enabled && !baseline_verified))throw std::runtime_error("hold compensation state does not match its qualified trial");
                    trim.integral_gain=c["integral_gain"].as<JointVector>();
                    trim.max_torque=c["max_torque"].as<JointVector>();trim.max_rate=c["max_rate"].as<JointVector>();
                    trim.contact_threshold=c["contact_threshold"].as<JointVector>();
                    trim.velocity_threshold=c["velocity_threshold"].as<double>();trim.error_threshold=c["error_threshold"].as<double>();
                    hold_compensation=trim;
                } else {
                    // Earlier baseline-only records were also measured with trim
                    // disabled; absence must never re-enable an unrelated old trim
                    auto trim=robot_.get_hold_compensation_cfg();trim.enabled=false;hold_compensation=trim;
                }
                updates.emplace_back(JointImpedanceMode::RIGID_TRACKING,gains);
            } else if(target=="compliant_tracking") {
                if(node["hold_gains"] && node["hold_verification"] && node["hold_verification"]["validated"].as<bool>(false))
                    updates.emplace_back(JointImpedanceMode::COMPLIANT_HOLD,JointImpedanceGains{node["hold_gains"]["kp"].as<JointVector>(),node["hold_gains"]["kd"].as<JointVector>()});
                updates.emplace_back(JointImpedanceMode::COMPLIANT_TRACKING,gains);
            } else if(target=="servo_tracking") {
                if(!node["native_validation"]["validated"].as<bool>(false) || !node["admittance_validation"]["validated"].as<bool>(false))throw std::runtime_error("servo candidate requires independent native and admittance validation");
                identification=gains;
            } else if(target=="compliant_drag") {
                if(!node["drag_validation"]["validated"].as<bool>(false) || std::any_of(gains.kp.begin(),gains.kp.end(),[](double v){return v!=0;}))throw std::runtime_error("drag candidate requires zero stiffness and measured release validation");
                updates.emplace_back(JointImpedanceMode::COMPLIANT_DRAG,gains);
            }
            else throw std::runtime_error("unknown gain calibration target");
        };
        std::vector<std::string> selected;
        const std::vector<std::string> order={"rigid_tracking","compliant_tracking","compliant_drag","servo_tracking"};
        if(category!="all" && std::find(order.begin(),order.end(),category)==order.end())throw std::runtime_error("unknown gain calibration target");
        if(explicit_selection) {
            if(!params["gain_purposes"].IsSequence() || params["gain_purposes"].size()==0)throw std::runtime_error("select at least one gain purpose");
            for(const auto& value:params["gain_purposes"]) {
                auto key=value.as<std::string>();
                if(key=="RIGID")key="rigid_tracking";else if(key=="COMPLIANT")key="compliant_tracking";
                else if(key=="DRAG")key="compliant_drag";else if(key=="SERVO_TRACKING" || key=="SERVO")key="servo_tracking";
                if(std::find(order.begin(),order.end(),key)==order.end())throw std::runtime_error("unknown selected gain purpose");
                if(std::find(selected.begin(),selected.end(),key)==selected.end())selected.push_back(key);
            }
        } else selected=category=="all"?order:std::vector<std::string>{category};
        // Apply rigid first, so a selected compliant purpose is checked against
        // either the selected rigid candidate or the currently deployed rigid set.
        for(const auto& purpose:order)if(std::find(selected.begin(),selected.end(),purpose)!=selected.end()) {
            if(category=="all") {
                if(!data["targets"] || !data["targets"].IsMap() || !data["targets"][purpose])throw std::runtime_error("selected gain purpose missing from candidate");
                collect(purpose,data["targets"][purpose]);
            } else {
                if(purpose!=category)throw std::runtime_error("selected gain purpose does not match candidate");
                collect(purpose,data);
            }
        }
        std::vector<std::pair<JointImpedanceMode,JointImpedanceGains>> original;
        for(const auto& item:updates)original.emplace_back(item.first,machine_saved_mode_gains(item.first));
        const auto previous_identification=robot_.get_config().ctrller.servo_tracking_gains;
        const auto previous_compensation=robot_.get_hold_compensation_cfg();
        std::size_t applied=0;bool servo_applied=false,trim_applied=false;
        try {
            for(const auto& item:updates) {
                if(!robot_.set_runtime_impedance_gains(item.first,item.second))throw std::runtime_error("cannot safely apply calibrated gains");
                machine_assign_mode_gains(item.first,item.second);++applied;
            }
            if(identification) {
                if(!robot_.set_servo_tracking_gains(*identification))throw std::runtime_error("cannot apply servo gain candidate");
                cfg_.ctrller.servo_tracking_gains=*identification;servo_applied=true;
            }
            if(hold_compensation) {
                if(!robot_.set_hold_compensation_cfg(*hold_compensation))throw std::runtime_error("cannot apply verified hold compensation");
                cfg_.hold_compensation=*hold_compensation;trim_applied=true;
            }
        } catch(...) {
            if(trim_applied){(void)robot_.set_hold_compensation_cfg(previous_compensation);cfg_.hold_compensation=previous_compensation;}
            if(servo_applied){(void)robot_.set_servo_tracking_gains(previous_identification);cfg_.ctrller.servo_tracking_gains=previous_identification;}
            while(applied>0){--applied;(void)robot_.set_runtime_impedance_gains(original[applied].first,original[applied].second);machine_assign_mode_gains(original[applied].first,original[applied].second);}
            throw;
        }
        runtime_gain_original_hold_compensation_=hold_compensation?std::optional<HoldCompensationCfg>(previous_compensation):std::nullopt;
        runtime_gain_original_modes_=std::move(original);
        runtime_gain_original_identification_=identification ?
            std::optional<JointImpedanceGains>(previous_identification) : std::nullopt;
        runtime_gain_category_=selected.size()==4?"all":std::find(selected.begin(),selected.end(),"servo_tracking")!=selected.end()?"servo_tracking":selected.front();
        runtime_gain_purposes_=selected;
        runtime_gain_applied_=true;
        runtime_gain_candidate_=path.string();
    }

    void machine_candidate_gain_restore() {
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(!runtime_gain_applied_)return;
        for(const auto& item:runtime_gain_original_modes_) {
            if(!robot_.set_runtime_impedance_gains(item.first,item.second))
                throw std::runtime_error("failed to restore original gain configuration");
            machine_assign_mode_gains(item.first,item.second);
        }
        if(runtime_gain_original_identification_) {
            if(!robot_.set_servo_tracking_gains(*runtime_gain_original_identification_))throw std::runtime_error("cannot restore servo gain candidate");
            cfg_.ctrller.servo_tracking_gains=*runtime_gain_original_identification_;
        }
        if(runtime_gain_original_hold_compensation_) {
            if(!robot_.set_hold_compensation_cfg(*runtime_gain_original_hold_compensation_))throw std::runtime_error("cannot restore hold compensation");
            cfg_.hold_compensation=*runtime_gain_original_hold_compensation_;
        }
        runtime_gain_original_modes_.clear();
        runtime_gain_original_identification_.reset();runtime_gain_original_hold_compensation_.reset();
        runtime_gain_category_.clear();runtime_gain_purposes_.clear();
        runtime_gain_applied_=false;
        runtime_gain_candidate_.clear();
    }

    void machine_candidate_gravity_apply(const YAML::Node& params) {
        if(!params || !params["path"])throw std::runtime_error("gravity correction path is required");
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(runtime_gravity_applied_ || runtime_dynamics_applied_ || model_calibration_candidate_applied_ ||
           dynamics_.has_gravity_first_moment_override())
            throw std::runtime_error("restore existing gravity correction before applying another");
        if(robot_.get_model_feedforward_mode()==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS ||
           cfg_.capability.admittance.observer.mode==AdmittanceObserverMode::FULL_ID)
            throw std::runtime_error("gravity-only candidate is incompatible with full dynamics controller");
        const auto path=std::filesystem::canonical(params["path"].as<std::string>());
        const auto package=load_gravity_correction_package(path.string(),cfg_.joint_names,cfg_.dynamics.urdf_path);
        const auto original=dynamics_.get_gravity_scale();
        JointVector unit(cfg_.joint_names.size(),1.0);
        if(!dynamics_.set_gravity_scale(unit))throw std::runtime_error("failed to use unit gravity scale");
        if(!dynamics_.set_gravity_first_moment_override(package.first_moments)) {
            (void)dynamics_.set_gravity_scale(original);
            throw std::runtime_error("gravity candidate failed model validation");
        }
        runtime_gravity_original_scale_=original;
        runtime_gravity_applied_=true;
        runtime_gravity_candidate_=path.string();
        cfg_.dynamics.gravity_scale=unit;
    }

    void machine_candidate_gravity_restore() {
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(!runtime_gravity_applied_)return;
        dynamics_.clear_gravity_first_moment_override();
        if(!dynamics_.set_gravity_scale(runtime_gravity_original_scale_))
            throw std::runtime_error("failed to restore original gravity scale");
        cfg_.dynamics.gravity_scale=runtime_gravity_original_scale_;
        runtime_gravity_applied_=false;
        runtime_gravity_candidate_.clear();
    }

    void machine_candidate_dynamics_apply(const YAML::Node& params) {
        if(!params || !params["report"])throw std::runtime_error("identification report is required");
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(runtime_dynamics_applied_ || runtime_gravity_applied_ ||
           dynamics_.has_gravity_first_moment_override())
            throw std::runtime_error("restore existing model override first");
        const auto report_path=std::filesystem::canonical(params["report"].as<std::string>());
        const auto report=YAML::LoadFile(report_path.string());
        if(!report || !report["schema"] || report["schema"].as<std::string>()!="serial_arm_advanced_identification" ||
           !report["status"] || report["status"].as<std::string>()!="review_passed_offline" ||
           !report["full_dynamics_physical_pass"] || !report["full_dynamics_physical_pass"].as<bool>())
            throw std::runtime_error("advanced candidate failed independent offline and physical checks");
        if(!report["source_urdf_fingerprint"] ||
           report["source_urdf_fingerprint"].as<std::string>()!=model_calibration_file_fingerprint(cfg_.dynamics.urdf_path))
            throw std::runtime_error("advanced model does not match current URDF");
        if(!report["candidate_urdf"] || !report["validation_scope"])
            throw std::runtime_error("advanced identification report is incomplete");
        // Full inertia is allowed only for an actual dynamic-data assessment.
        const auto scope=report["validation_scope"].as<std::string>();
        if(scope!="dynamic_inertia")
            throw std::runtime_error("static gravity candidate must use the gravity-only apply route");
        const std::filesystem::path recorded_candidate(report["candidate_urdf"].as<std::string>());
        if(recorded_candidate.empty()) throw std::runtime_error("candidate URDF path is empty");
        // Reports and candidates form one portable directory. An older report
        // may retain its former absolute path after the directory is copied.
        const auto local_candidate = recorded_candidate.is_absolute()
            ? report_path.parent_path() / recorded_candidate.filename()
            : report_path.parent_path() / recorded_candidate;
        const auto candidate_path=std::filesystem::canonical(local_candidate);
        if(candidate_path.parent_path()!=report_path.parent_path())
            throw std::runtime_error("candidate URDF must remain in the identification report directory");
        if(!std::filesystem::is_regular_file(candidate_path))
            throw std::runtime_error("candidate URDF is missing");
        if(!report["candidate_urdf_fingerprint"] ||
           report["candidate_urdf_fingerprint"].as<std::string>()!=model_calibration_file_fingerprint(candidate_path.string()))
            throw std::runtime_error("advanced candidate bytes changed since offline verification");
        auto new_cfg=cfg_.dynamics;
        new_cfg.urdf_path=candidate_path.string();
        // Avoid accidentally layering a stale gravity correction on a new model.
        new_cfg.gravity_correction_path.clear();
        Dynamics candidate;
        if(!candidate.configure(new_cfg))
            throw std::runtime_error("candidate URDF cannot configure robot dynamics");
        // Check the whole controlled kinematic tree, not only tool0 at q=0.
        const auto& expected_frames=dynamics_.get_info().frame_names;
        const auto& candidate_frames=candidate.get_info().frame_names;
        if(expected_frames!=candidate_frames)
            throw std::runtime_error("candidate URDF changed frame hierarchy");
        const std::size_t n=cfg_.joint_names.size();
        JointState sample;
        sample.pos.assign(n,0.0);sample.vel.assign(n,0.0);sample.tor.assign(n,0.0);
        JointVector zero(n,0.0);
        // Must not mutate live dynamics caches: work on a separate prior.
        Dynamics prior;
        if(!prior.configure(cfg_.dynamics))
            throw std::runtime_error("original model snapshot could not be validated");
        for(std::size_t index=0;index<=n;++index) {
            std::fill(sample.pos.begin(),sample.pos.end(),0.0);
            if(index<n) sample.pos[index]=0.35;
            if(!prior.update(sample,zero,zero) || !candidate.update(sample,zero,zero))
                throw std::runtime_error("candidate kinematics evaluation failed");
            for(const auto& frame:expected_frames) {
                const auto a=prior.get_frame_pose(frame);
                const auto b=candidate.get_frame_pose(frame);
                if(!a || !b ||
                   (a->translation()-b->translation()).norm()>1.0e-6 ||
                   (a->rotation()-b->rotation()).norm()>1.0e-6)
                    throw std::runtime_error("candidate changed geometry or joint axes: " + frame);
            }
        }
        runtime_original_dynamics_=std::make_unique<Dynamics>(std::move(dynamics_));
        dynamics_=std::move(candidate);
        runtime_original_dynamics_cfg_=cfg_.dynamics;
        cfg_.dynamics=new_cfg;
        runtime_dynamics_applied_=true;
        runtime_dynamics_candidate_=candidate_path.string();
    }

    void machine_candidate_dynamics_restore() {
        std::lock_guard<std::mutex> lock(mutex_);
        machine_require_safe_candidate_state();
        if(!runtime_dynamics_applied_)return;
        if(!runtime_original_dynamics_)
            throw std::runtime_error("original dynamics snapshot is unavailable");
        dynamics_=std::move(*runtime_original_dynamics_);
        runtime_original_dynamics_.reset();
        cfg_.dynamics=runtime_original_dynamics_cfg_;
        runtime_dynamics_applied_=false;
        runtime_dynamics_candidate_.clear();
    }

    void machine_model_calibration_apply(const YAML::Node& params) {
        model_calibration_require_task(params);
        if(model_calibration_task_active())throw std::runtime_error("model calibration task is still active");
        if(!model_calibration_result_ || !model_calibration_result_->static_pass)throw std::runtime_error("no validated gravity candidate is available");
        std::lock_guard<std::mutex> lock(mutex_);
        if(runtime_gravity_applied_ || runtime_dynamics_applied_)
            throw std::runtime_error("restore temporary runtime model override before legacy apply");
        if(robot_.get_state() != RobotState::INACTIVE)throw std::runtime_error("candidate application requires RobotState::INACTIVE");
        if(robot_.get_model_feedforward_mode() == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)throw std::runtime_error("candidate gravity is not compatible with FULL_INVERSE_DYNAMICS");
        if(cfg_.capability.admittance.observer.mode == AdmittanceObserverMode::FULL_ID)throw std::runtime_error("candidate gravity is not compatible with FULL_ID observer mode");
        const auto current_core = model_calibration_file_fingerprint(config_path_); const auto current_urdf = model_calibration_file_fingerprint(cfg_.dynamics.urdf_path);
        if(current_core != model_calibration_core_fingerprint_ || current_urdf != model_calibration_urdf_fingerprint_)throw std::runtime_error("configuration or URDF changed after calibration");
        model_calibration_applied_original_gravity_scale_ = dynamics_.get_gravity_scale(); model_calibration_applied_original_admittance_ = cfg_.capability.admittance;
        JointVector unit(cfg_.joint_names.size(), 1.0); const auto scale = dynamics_.set_gravity_scale(unit); if(!scale)throw std::runtime_error("failed to set unit gravity scale");
        const auto override_result = dynamics_.set_gravity_first_moment_override(model_calibration_result_->first_moments); if(!override_result) { (void)dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_); throw std::runtime_error("failed to apply gravity candidate"); }
        auto candidate = cfg_.capability.admittance; candidate.calibration.torque_bias = model_calibration_result_->torque_bias; candidate.calibration.torque_threshold.resize(cfg_.joint_names.size()); for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) { const double p99 = i < model_calibration_result_->validation_candidate.p99.size() ? model_calibration_result_->validation_candidate.p99[i] : 0.0; const double noise = i < model_calibration_result_->noise_rms.size() ? model_calibration_result_->noise_rms[i] : 0.0; candidate.calibration.torque_threshold[i] = std::max({ 0.02,1.2 * p99,3.0 * noise }); }
        if(model_calibration_friction_pass_ && model_calibration_friction_result_) { candidate.calibration.friction.enabled = true; candidate.calibration.friction.velocity_transition = 0.03; candidate.calibration.friction.positive_coulomb = model_calibration_friction_result_->positive_coulomb; candidate.calibration.friction.positive_viscous = model_calibration_friction_result_->positive_viscous; candidate.calibration.friction.negative_coulomb = model_calibration_friction_result_->negative_coulomb; candidate.calibration.friction.negative_viscous = model_calibration_friction_result_->negative_viscous; }
        else candidate.calibration.friction.enabled = false;
        const auto admittance = robot_.set_admittance_cfg(candidate); if(!admittance) { dynamics_.clear_gravity_first_moment_override(); (void)dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_); throw std::runtime_error("failed to apply candidate residual parameters"); }
        cfg_.dynamics.gravity_scale = unit; cfg_.capability.admittance = candidate; model_calibration_candidate_applied_ = true;
    }

    void machine_model_calibration_restore(const YAML::Node& params) {
        model_calibration_require_task(params);
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_.get_state() != RobotState::INACTIVE)throw std::runtime_error("candidate restore requires RobotState::INACTIVE");
        if(!model_calibration_candidate_applied_)return;
        const auto candidate_admittance = cfg_.capability.admittance;
        const auto candidate_scale = dynamics_.get_gravity_scale();
        const auto candidate_moments = dynamics_.get_gravity_first_moment_override();
        const auto admittance = robot_.set_admittance_cfg(model_calibration_applied_original_admittance_); if(!admittance)throw std::runtime_error("failed to restore original residual parameters");
        dynamics_.clear_gravity_first_moment_override();
        const auto scale = dynamics_.set_gravity_scale(model_calibration_applied_original_gravity_scale_);
        if(!scale) {
            (void)dynamics_.set_gravity_scale(candidate_scale);
            (void)dynamics_.set_gravity_first_moment_override(candidate_moments);
            (void)robot_.set_admittance_cfg(candidate_admittance);
            throw std::runtime_error("failed to restore original gravity scale");
        }
        cfg_.dynamics.gravity_scale = model_calibration_applied_original_gravity_scale_; cfg_.capability.admittance = model_calibration_applied_original_admittance_; model_calibration_candidate_applied_ = false;
    }

    std::string machine_model_calibration_status_json() const {
        std::ostringstream out; const auto recorder = model_calibration_recorder_.status();
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        out << std::setprecision(17) << "{\"task_id\":\"" << machine_json_escape(model_calibration_task_id_) << "\",\"phase\":\"" << to_string(model_calibration_phase_.load()) << "\",\"progress\":" << model_calibration_progress_ << ",\"completed\":" << model_calibration_completed_units_ << ",\"total\":" << model_calibration_total_units_ << ",\"valid_static_samples\":" << model_calibration_valid_static_samples_ << ",\"rejected_static_groups\":" << model_calibration_rejected_static_groups_ << ",\"directory\":\"" << machine_json_escape(model_calibration_directory_) << "\",\"candidate_applied\":" << (model_calibration_candidate_applied_ ? "true" : "false") << ",\"resume_confirmation_required\":" << (model_calibration_resume_requires_confirmation_ ? "true" : "false") << ",\"core_fingerprint\":\"" << machine_json_escape(model_calibration_core_fingerprint_) << "\",\"urdf_fingerprint\":\"" << machine_json_escape(model_calibration_urdf_fingerprint_) << "\",\"joint_names\":[";
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) { if(i)out << ','; out << '\"' << machine_json_escape(cfg_.joint_names[i]) << '\"'; }
        out << "],\"original_gravity_scale\":"; machine_json_joint_vector(out, model_calibration_original_gravity_scale_);
        out << ",\"recorder\":{\"accepted\":" << recorder.accepted_frames << ",\"written\":" << recorder.written_frames << ",\"dropped\":" << recorder.dropped_frames << ",\"data_gap\":" << (recorder.data_gap ? "true" : "false") << ",\"write_failed\":" << (recorder.write_failed ? "true" : "false") << "}";
        out << ",\"static_pair_quality\":{\"bidirectional_groups\":" << recorder.bidirectional_static_groups
            << ",\"position_warnings\":" << recorder.static_pair_position_warnings
            << ",\"max_position_delta_rad\":" << recorder.max_static_pair_position_delta_rad << '}';
        if(!model_calibration_preview_error_.empty())out << ",\"preview_error\":\"" << machine_json_escape(model_calibration_preview_error_) << "\"";
        if(!model_calibration_error_.empty())out << ",\"error\":\"" << machine_json_escape(model_calibration_error_) << "\"";
        if(model_calibration_trajectory_)out << ",\"trajectory_samples\":" << model_calibration_trajectory_->positions.size() << ",\"replay_rate\":" << model_calibration_replay_rate_;
        out << ",\"identification_mode\":\"" << machine_json_escape(model_calibration_options_.identification_mode) << "\",\"locked_links\":[";
        for(std::size_t i = 0; i < model_calibration_options_.locked_links.size(); ++i) { if(i)out << ','; out << '"' << machine_json_escape(model_calibration_options_.locked_links[i]) << '"'; }
        out << "],\"identifiable_links\":[";
        for(std::size_t i = 0; i < model_calibration_links_cache_.size(); ++i) {
            if(i) out << ',';
            const auto& info = model_calibration_links_cache_[i];
            out << "{\"name\":\"" << machine_json_escape(info.link_name) << "\",\"joint\":\"" << machine_json_escape(info.joint_name) << "\",\"mass\":" << info.mass << ",\"com\":[" << info.center_of_mass.x() << "," << info.center_of_mass.y() << "," << info.center_of_mass.z() << "]}";
        }
        out << ']';
        out << ",\"planner_id\":\"local_hermite\",\"calibration_strategy\":\"bidirectional_two_speed_v1\"";
        out << ",\"alignment_active\":" << (model_calibration_alignment_active_.load() ? "true" : "false");
        out << ",\"alignment_position_tolerance\":[";
        const auto& alignment_types=dynamics_.get_info().joint_types;
        for(std::size_t i=0;i<alignment_types.size();++i){if(i)out<<',';out<<serial_arm::calibration_recovery::position_tolerance(alignment_types[i]);}
        out << "],\"alignment_velocity_tolerance\":[";
        for(std::size_t i=0;i<alignment_types.size();++i){if(i)out<<',';out<<serial_arm::calibration_recovery::velocity_tolerance(alignment_types[i]);}
        out << "],\"joint_units\":[";
        for(std::size_t i=0;i<alignment_types.size();++i){if(i)out<<',';out<<'"'<<(alignment_types[i]=="prismatic"?"m":"rad")<<'"';}
        out << ']';
        if(!model_calibration_source_task_id_.empty()) out << ",\"original_start_selected\":" << (model_calibration_source_reversed_.load() ? "true" : "false");
        if(!model_calibration_source_task_id_.empty()) out << ",\"source_task_id\":\"" << machine_json_escape(model_calibration_source_task_id_) << "\"";
        if(model_calibration_trajectory_ && !model_calibration_trajectory_->positions.empty()) {
            out << ",\"replay_start_joint_positions\":";
            machine_json_joint_vector(out, model_calibration_trajectory_->positions.back());
        }
        out << ",\"joint_path_length_rad\":" << model_calibration_path_length_rad_
            << ",\"geometric_waypoints\":" << model_calibration_geometric_waypoints_
            << ",\"original_samples\":" << model_calibration_original_samples_;
        out << ",\"workflow_kind\":\"" << machine_json_escape(model_calibration_workflow_kind_) << "\"";
        out << ",\"gain_mode\":\"" << machine_json_escape(model_calibration_gain_mode_) << "\"";
        out << ",\"gain_active_target\":\"" << (model_calibration_gain_stage_.load()==1 ? "compliant" :
            model_calibration_gain_stage_.load()==2 ? "servo_tracking" : model_calibration_gain_stage_.load()==3 ? "drag" : "rigid") << "\"";
        out << ",\"gain_adaptation_max_passes\":" << gain_planned_windows();
        out << ",\"gain_adaptation_passes\":" << model_calibration_adaptation_passes_.load()
            << ",\"gain_adaptation_revisions\":" << model_calibration_adaptation_revisions_.load()
            << ",\"gain_adaptation_converged\":" << (model_calibration_adaptation_converged_.load()?"true":"false");
        out << ",\"gain_probe_kind\":\"" << machine_json_escape(gain_probe_kind_)
            << "\",\"gain_probe_joint\":\"" << machine_json_escape(gain_probe_joint_)
            << "\",\"gain_probe_message\":\"" << machine_json_escape(gain_probe_message_)
            << "\",\"gain_candidate_index\":" << gain_candidate_index_
            << ",\"gain_candidate_limit\":" << (gain_probe_stage_=="motion_discovery"?serial_arm::servo_tuning::motion_attempt_limit:gain_probe_stage_=="dynamic_search"?gain_requirements_.max_candidates:0)
            << ",\"gain_probe_stage\":\"" << machine_json_escape(gain_probe_stage_) << "\",\"gain_probe_frequency_hz\":["
            << gain_probe_frequency_hz_[0] << "," << gain_probe_frequency_hz_[1] << "," << gain_probe_frequency_hz_[2] << "]"
            << ",\"gain_probe_kp\":" << gain_probe_kp_ << ",\"gain_probe_kd\":" << gain_probe_kd_
            << ",\"gain_probe_diagnostic\":" << gain_probe_diagnostic_json_
            << ",\"gain_probe_elapsed_s\":" << gain_probe_elapsed_s_
            << ",\"gain_probe_duration_s\":" << gain_probe_duration_s_
            << ",\"gain_probe_amplitude_rad\":" << gain_probe_amplitude_rad_
            << ",\"gain_probe_reference_joint_positions\":";
        if(model_calibration_phase_.load()==ModelCalibrationPhase::GAIN_ADAPTATION &&
           !model_calibration_cancel_.load() && !model_calibration_pause_.load())machine_json_joint_vector(out,gain_probe_reference_);
        else out << "[]";
        out << ",\"gain_seed_source\":\"" << machine_json_escape(gain_seed_source_) << "\",\"gain_seed_kp\":";
        machine_json_joint_vector(out,gain_seed_.kp);out << ",\"gain_seed_kd\":";machine_json_joint_vector(out,gain_seed_.kd);
        out << ",\"fault_interrupted\":" << (model_calibration_fault_interrupted_.load()?"true":"false");
        out << ",\"teaching_wall_duration_s\":" << (model_calibration_phase_.load() == ModelCalibrationPhase::TEACHING && model_calibration_teaching_started_at_ ?
            std::chrono::duration<double>(Robot::Clock::now() - *model_calibration_teaching_started_at_).count() : model_calibration_teaching_wall_duration_s_)
            << ",\"teaching_cleanup_pending\":" << (model_calibration_teach_thread_.joinable() &&
                (model_calibration_phase_.load() == ModelCalibrationPhase::FAILED ||
                 model_calibration_phase_.load() == ModelCalibrationPhase::CANCELLED) ? "true" : "false")
            << ",\"recorded_duration_s\":" << model_calibration_original_duration_s_;
        if(model_calibration_trajectory_ && model_calibration_trajectory_->positions.size() >= 2)
            out << ",\"trajectory_duration_s\":" << model_calibration_trajectory_->sample_dt *
            static_cast<double>(model_calibration_trajectory_->positions.size() - 1)
            << ",\"trajectory_sample_dt_s\":" << model_calibration_trajectory_->sample_dt;
        out << ",\"single_pass_duration_s\":" << (model_calibration_trajectory_ && model_calibration_trajectory_->positions.size() >= 2 ?
            model_calibration_trajectory_->sample_dt * static_cast<double>(model_calibration_trajectory_->positions.size() - 1) : 0.0)
            << ",\"estimated_duration_s\":" << model_calibration_estimated_duration_s_
            << ",\"estimated_total_duration_s\":" << (model_calibration_teaching_wall_duration_s_ + model_calibration_estimated_duration_s_)
            << ",\"max_task_duration_s\":0"
            << ",\"minimum_information_score\":" << model_calibration_options_.minimum_information_score;
        out << ",\"pose_targets\":["; for(std::size_t i = 0; i < model_calibration_pose_targets_.size(); ++i) { if(i)out << ','; const auto& p = model_calibration_pose_targets_[i]; out << "{\"index\":" << p.trajectory_index << ",\"group\":" << p.pose_group << ",\"validation\":" << (p.validation ? "true" : "false") << ",\"score\":" << p.information_score << '}'; }out << ']';
        if(model_calibration_result_)out << ",\"gravity_result\":" << gravity_calibration_result_json(*model_calibration_result_);
        out << ",\"friction_pass\":" << (model_calibration_friction_pass_ ? "true" : "false");
        if(model_calibration_friction_result_) { out << ",\"friction\":{\"positive_coulomb\":"; machine_json_joint_vector(out, model_calibration_friction_result_->positive_coulomb); out << ",\"positive_viscous\":"; machine_json_joint_vector(out, model_calibration_friction_result_->positive_viscous); out << ",\"negative_coulomb\":"; machine_json_joint_vector(out, model_calibration_friction_result_->negative_coulomb); out << ",\"negative_viscous\":"; machine_json_joint_vector(out, model_calibration_friction_result_->negative_viscous); out << "}"; }
        out << '}'; return out.str();
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
        }
        else machine_calibration_phase_ = "ready_to_fit";
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
        }
        else {
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

    void machine_calibration_cancel(bool restore_mode = true) {
        if(machine_friction_thread_.joinable()) {
            machine_friction_stop_.store(true);
            cycle_cv_.notify_all();
            machine_friction_thread_.join();
        }
        if(machine_calibration_context_valid_) machine_restore_calibration_runtime(true, restore_mode);
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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            machine_require_friction_endpoint_locked(machine_friction_trajectory_->positions.back());
        }
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
            for(std::size_t i = 0; i < cfg_.joint_names.size() && valid; ++i) valid = result->observable[i] && result->validation_pass[i];
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
            out << ",\"static\":{\"gravity_scale\":"; machine_json_joint_vector(out, last_static_calibration_result_->gravity_scale);
            out << ",\"torque_bias\":"; machine_json_joint_vector(out, last_static_calibration_result_->torque_bias);
            out << ",\"torque_threshold\":"; machine_json_joint_vector(out, last_static_calibration_result_->torque_threshold);
            out << ",\"observable\":"; machine_json_vector(out, last_static_calibration_result_->gravity_scale_observable); out << '}';
        }
        if(last_static_validation_result_) {
            out << ",\"validation\":{\"residual_rms\":"; machine_json_joint_vector(out, last_static_validation_result_->residual_rms);
            out << ",\"residual_p99\":"; machine_json_joint_vector(out, last_static_validation_result_->residual_p99);
            out << ",\"residual_max\":"; machine_json_joint_vector(out, last_static_validation_result_->residual_max);
            out << ",\"pass\":"; machine_json_vector(out, last_static_validation_result_->pass); out << '}';
        }
        if(last_friction_calibration_result_) {
            out << ",\"friction\":{\"positive_coulomb\":"; machine_json_joint_vector(out, last_friction_calibration_result_->positive_coulomb);
            out << ",\"positive_viscous\":"; machine_json_joint_vector(out, last_friction_calibration_result_->positive_viscous);
            out << ",\"negative_coulomb\":"; machine_json_joint_vector(out, last_friction_calibration_result_->negative_coulomb);
            out << ",\"negative_viscous\":"; machine_json_joint_vector(out, last_friction_calibration_result_->negative_viscous);
            out << ",\"validation_pass\":"; machine_json_vector(out, last_friction_calibration_result_->validation_pass); out << '}';
        }
        out << '}'; return out.str();
    }

    std::string motion_json_locked() const {
        const auto& motion=motion_.status();
        std::ostringstream out;out << std::setprecision(17);
        out << "{\"id\":" << motion.id << ",\"state\":\"" << motion.state << "\",\"target\":";
        machine_json_joint_vector(out,motion.target);
        out << ",\"position_tolerance\":";machine_json_joint_vector(out,motion.position_tolerance);
        out << ",\"velocity_tolerance\":";machine_json_joint_vector(out,motion.velocity_tolerance);
        out << ",\"units\":[";
        for(std::size_t i=0;i<motion.units.size();++i){if(i)out<<',';machine_json_string(out,motion.units[i]);}
        out << "],\"tolerance_source\":\"joint_type_defaults\",\"reference_complete\":" << (motion.reference_complete?"true":"false")
            << ",\"settled_s\":" << motion.settled_s << ",\"reason\":\"" << machine_json_escape(motion.reason) << "\"}";
        return out.str();
    }
    std::string machine_motion_json() const {
        std::lock_guard<std::mutex> lock(mutex_);return motion_json_locked();
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
        out << ",\"reference_behavior\":";machine_json_string(out,reference_behavior_name(robot_.get_reference_behavior()));
        out << ",\"monotonic_ns\":" << last_cycle_monotonic_ns_;
        out << ",\"feedback_receive_ns\":" << feedback_time_ns_;
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
        out << ",\"gravity_verification\":{\"active\":" << (gravity_verify_active_ ? "true" : "false")
            << ",\"phase\":\"" << gravity_verify_result_ << "\""
            << ",\"sample_count\":" << gravity_verify_samples_.size()
            << ",\"ramp_settled\":";
        bool settled = gravity_verify_target_scale_.size() == cfg_.dynamics.gravity_scale.size();
        if(settled) for(std::size_t i = 0; i < gravity_verify_target_scale_.size(); ++i)
            if(std::abs(gravity_verify_target_scale_[i] - cfg_.dynamics.gravity_scale[i]) > 0.005) settled = false;
        out << (settled ? "true" : "false") << ",\"target\":";
        machine_json_joint_vector(out, gravity_verify_target_scale_);
        out << '}';
        out << ",\"motion\":" << motion_json_locked();
        out << ",\"calibration\":" << machine_calibration_status_json();
        out << ",\"model_calibration\":" << machine_model_calibration_status_json();
        out << ",\"teach_replay\":{\"phase\":\"" << (teach_replay_phase_ == 1 ? "recording" : teach_replay_phase_ == 2 ? "ready" : teach_replay_phase_ == 3 ? "playing" : teach_replay_phase_ == 4 ? "failed" : "idle") << '"';
        out << ",\"completed\":" << teach_replay_completed_.load();
        if(teach_replay_plan_) {
            const auto& plan=*teach_replay_plan_;
            out << ",\"plan\":{\"direction\":\"" << plan.direction << "\",\"cycles\":" << plan.cycles
                << ",\"total_passes\":" << plan.total_passes;
            if(teach_replay_trajectory_&&!teach_replay_trajectory_->positions.empty()) {
                const auto& positions=teach_replay_trajectory_->positions;
                out << ",\"start\":";machine_json_joint_vector(out,plan.first_reverse?positions.back():positions.front());
                const bool final_reverse=plan.first_reverse?plan.total_passes%2!=0:plan.total_passes%2==0;
                out << ",\"end\":";machine_json_joint_vector(out,final_reverse?positions.front():positions.back());
            }
            out << '}';
        }
        out << ",\"selected_reference_behavior\":\"" << reference_behavior_name(teach_replay_behavior_) << "\",\"reference_behavior\":\"" << reference_behavior_name(teach_replay_behavior_) << "\",\"gain_profile\":\"" << to_string(teach_replay_gain_profile_) << '\"';
        out << ",\"has_trajectory\":" << (teach_replay_trajectory_ && !teach_replay_trajectory_->positions.empty()?"true":"false")
            << ",\"fault_interrupted\":" << (teach_replay_fault_interrupted_?"true":"false");
        if(teach_replay_trajectory_ && !teach_replay_trajectory_->positions.empty()) {
            out << ",\"start\":";machine_json_joint_vector(out,teach_replay_trajectory_->positions.front());
            out << ",\"end\":";machine_json_joint_vector(out,teach_replay_trajectory_->positions.back());
        }
        if(!teach_replay_error_.empty())out << ",\"error\":\"" << machine_json_escape(teach_replay_error_) << '"';
        out << '}';

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
        out << ",\"actuator_info_source\":";
        machine_json_string(out, actuator_info_after_connect_ ?
            (cfg_.runtime.write_enabled ? "hardware_backend_after_successful_activate" : "mock_backend_after_successful_activate") :
            "unconfirmed_for_current_connection");
        if(const auto fault = robot_.get_last_fault()) {
            out << ",\"last_fault\":";
            machine_json_string(out, to_string(fault->code));
            if(robot_.get_state() == RobotState::FAULT) {
                out << ",\"fault\":{\"code\":";
                machine_json_string(out, to_string(fault->code));
                if(fault->code == RobotErr::MOTOR_BUS_CONNECT_FAILED ||
                   fault->code == RobotErr::MOTOR_BUS_ACTIVATE_FAILED ||
                   fault->code == RobotErr::MOTOR_BUS_READ_FAILED ||
                   fault->code == RobotErr::MOTOR_BUS_WRITE_FAILED ||
                   fault->code == RobotErr::MOTOR_BUS_DEACTIVATE_FAILED) {
                    out << ",\"motor_bus_error\":";
                    machine_json_string(out, to_string(fault->motor_bus_err));
                }
                if(fault->code==RobotErr::REFERENCE_GOVERNOR_FAILED) {
                    out << ",\"reference_governor_error\":";
                    machine_json_string(out,to_string(fault->reference_governor_err));
                }
                if(fault->code == RobotErr::SAFETY_FAILED) {
                    const auto& safety = fault->safety_fault;
                    out << ",\"safety_code\":";
                    machine_json_string(out, to_string(safety.code));
                    if(safety.index != kInvalidIndex && safety.index < cfg_.joint_names.size()) {
                        out << ",\"joint_index\":" << safety.index << ",\"joint_name\":";
                        machine_json_string(out, cfg_.joint_names[safety.index]);
                    }
                    if(safety.code==SafetyErr::STATE_TIMEOUT) {
                        out << ",\"feedback_age_source\":\"" << (safety.index<cfg_.joint_names.size()?"host_transport_receive":"legacy_state_read_interval") << "\"";
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
        out << ",\"min_pos\":";machine_json_joint_vector(out,cfg_.safety.limits.min_pos);
        out << ",\"max_pos\":";machine_json_joint_vector(out,cfg_.safety.limits.max_pos);
        out << ",\"pos_margin\":";machine_json_joint_vector(out,cfg_.safety.limits.pos_margin);
        out << ",\"has_position_limit\":";machine_json_vector(out,cfg_.safety.limits.has_position_limit);
        out << ",\"max_state_vel\":[";
        for(std::size_t i = 0; i < cfg_.safety.limits.max_vel.size(); ++i) {
            if(i) out << ',';
            out << cfg_.safety.limits.max_vel[i] * cfg_.safety.state_vel_fault_ratio;
        }
        out << "]}";

        if(last_output_) {
            const auto& output = *last_output_;
            out << ",\"parameter_update\":{\"state\":\"" <<
                (output.parameter_update_progress<1.0 ? "applying" : "applied") <<
                "\",\"progress\":" << output.parameter_update_progress << ",\"duration_s\":0.3}";
            out << ",\"gain_profile\":";machine_json_string(out,to_string(output.gain_profile));
            out << ",\"gain_source\":";machine_json_string(out,to_string(output.gain_source));
            out << ",\"effective_kp\":";machine_json_joint_vector(out,output.effective_kp);
            out << ",\"effective_kd\":";machine_json_joint_vector(out,output.effective_kd);
            out << ",\"hold_target_pos\":";machine_json_joint_vector(out,output.hold_target_pos);
            out << ",\"hold_integral_torque\":";machine_json_joint_vector(out,output.hold_integral_torque);
            out << ",\"admittance_active\":" << (output.admittance_active ? "true" : "false");
            out << ",\"observer_valid\":" << (output.observer_valid ? "true" : "false");
            out << ",\"new_measurement\":" << (output.new_measurement ? "true" : "false");
            out << ",\"measurement_dt\":" << output.measurement_dt;
            out << ",\"feedback_skew_ms\":";
            if(std::isfinite(output.feedback_skew_ms))out << output.feedback_skew_ms;else out << "null";
            out << ",\"reference_limited\":";machine_json_vector(out,output.reference_limited);
            out << ",\"torque_limited\":";machine_json_vector(out,output.torque_limited);
            out << ",\"joint\":{\"pos\":";
            machine_json_joint_vector(out, output.joint_state.pos);
            out << ",\"vel\":";
            machine_json_joint_vector(out, output.joint_state.vel);
            out << ",\"acc\":";
            if(output.joint_acc_valid)machine_json_joint_vector(out,output.joint_acc);else out << "null";
            out << ",\"acceleration_valid\":" << (output.joint_acc_valid ? "true" : "false");
            out << ",\"tor\":";
            machine_json_joint_vector(out, output.joint_state.tor);
            out << ",\"ref_pos\":";
            machine_json_joint_vector(out, output.joint_cmd.pos);
            out << ",\"ref_vel\":";
            machine_json_joint_vector(out, output.joint_cmd.vel);
            out << ",\"nominal_ref_pos\":";machine_json_joint_vector(out,output.nominal_ref_pos);
            out << ",\"nominal_ref_vel\":";machine_json_joint_vector(out,output.nominal_ref_vel);
            out << ",\"nominal_ref_acc\":";machine_json_joint_vector(out,output.nominal_ref_acc);
            out << ",\"virtual_ref_pos\":";machine_json_joint_vector(out,output.virtual_ref_pos);
            out << ",\"virtual_ref_vel\":";machine_json_joint_vector(out,output.virtual_ref_vel);
            out << ",\"virtual_ref_acc\":";machine_json_joint_vector(out,output.virtual_ref_acc);
            out << ",\"accepted_ref_pos\":";machine_json_joint_vector(out,output.accepted_ref_pos);
            out << ",\"accepted_ref_vel\":";machine_json_joint_vector(out,output.accepted_ref_vel);
            out << ",\"accepted_ref_acc\":";machine_json_joint_vector(out,output.accepted_ref_acc);
            out << ",\"predicted_total_torque\":";machine_json_joint_vector(out,output.predicted_total_torque);
            out << ",\"delta_q_ddot\":";machine_json_joint_vector(out,output.delta_q_ddot);
            JointVector pd_torque(output.joint_state.pos.size(),0.0);
            for(std::size_t i=0;i<pd_torque.size();++i)pd_torque[i]=
                output.joint_cmd.kp[i]*(output.joint_cmd.pos[i]-output.joint_state.pos[i])+
                output.joint_cmd.kd[i]*(output.joint_cmd.vel[i]-output.joint_state.vel[i]);
            out << ",\"pd_torque\":";machine_json_joint_vector(out,pd_torque);
            out << ",\"effort_limit\":";machine_json_joint_vector(out,cfg_.safety.limits.max_effort);
            out << ",\"kp\":";machine_json_joint_vector(out,output.joint_cmd.kp);
            out << ",\"kd\":";machine_json_joint_vector(out,output.joint_cmd.kd);
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
        RuntimeHooks hooks;
        hooks.running=[this]{return worker_running_.load();};
        hooks.serialization_mutex=&mutex_;
        hooks.before_cycle=[this](Robot::TimePoint now,double){
            gravity_verify_cycle_update();
            if(stream_.enabled) {
                const auto result=update_stream(now);
                if(!result){report_background_fault(result.error());return false;}
            }
            return true;
        };
        hooks.after_step=[this](const RuntimeSnapshot& step){
            if(step.cycle) {
                // Refresh diagnostic RNEA against the accepted reference; the
                // command itself was already computed from the cycle snapshot.
                if(dynamics_.is_updated())static_cast<void>(dynamics_.update_reference(step.cycle->joint_ref_acc));
                last_output_=*step.cycle;
                last_cycle_monotonic_ns_=step.monotonic_ns;
                const auto& stamps=last_output_->actuator_state.feedback_received_ns;
                feedback_time_ns_=std::isfinite(measured_feedback_age_ms(*last_output_)) ?
                    *std::min_element(stamps.begin(),stamps.end()) : 0;
                ++cycle_counter_;
                motion_.update(*last_output_,cfg_.safety.state_timeout_s*1000.0);
                record_model_calibration_cycle(*last_output_,Robot::TimePoint(std::chrono::nanoseconds(step.monotonic_ns)));
                cycle_cv_.notify_all();
            }
            if(step.fault)report_background_fault(*step.fault);
        };
        runtime_.run(robot_,cfg_.runtime.ctrl_frequency_hz,cfg_.safety.max_dt_s,hooks);
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
        frame.feedback_received_ns=output.actuator_state.feedback_received_ns;
        frame.feedback_timing_valid=output.actuator_state.feedback_timing_valid;
        frame.acceleration = output.joint_acc;
        frame.reference_position = output.joint_cmd.pos;
        frame.reference_velocity = output.joint_cmd.vel;
        frame.command_kp = output.joint_cmd.kp;
        frame.command_kd = output.joint_cmd.kd;
        frame.model_feedforward = output.model_feedforward;
        frame.acceleration_source = output.joint_acc_valid ? "robot_feedback_velocity_filter" : "invalid_or_unknown_sample_timing";
        frame.impedance_mode = to_string(robot_.get_impedance_mode());
        frame.feedback_age_ms = measured_feedback_age_ms(output);
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
            finite(frame.reference_position) && finite(frame.reference_velocity) &&
            std::isfinite(frame.feedback_age_ms) && frame.feedback_age_ms <= 30.0;
        model_calibration_recorder_.push(std::move(frame));
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
        const JointVector* measured=stream_.park_mode && last_output_ ? &last_output_->joint_state.pos : nullptr;
        const bool complete=advance_joint_reference(next_pos,next_vel,stream_.target_pos,
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,stream_.speed_scale,dt,measured);
        const auto result = runtime_.submit_reference(robot_,JointPosVelCmd{next_pos,next_vel},now,
            stream_.park_mode ? MotionSource::PARK : MotionSource::STREAM);
        if(!result)return result;

        stream_.ref_pos = std::move(next_pos);
        stream_.ref_vel = std::move(next_vel);
        stream_.last_update_time = now;
        stream_.has_last_update_time = true;
        if(!stream_.park_mode)motion_.reference_complete(complete);
        if(complete && !stream_.completion_reported) {
            stream_.completion_reported = true;
            std::cout << "\n[轨迹] 参考已到达目标，继续刷新并等待实测停放判据\n请输入菜单编号继续\n";
        }
        return { };
    }

    void report_background_fault(const RobotFault& fault) {
        motion_.finish("failed",machine_robot_error("motion",fault).what());
        clear_command_sources();
        if(background_fault_reported_) return;
        background_fault_reported_ = true;
        if(teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3) {
            teach_replay_fault_interrupted_=true;
            teach_replay_error_=machine_robot_error("teaching or replay stopped by safety fault",fault).what();
            teach_replay_record_stop_.store(true);teach_replay_stop_.store(true);teach_replay_phase_.store(4);
            cycle_cv_.notify_all();
        }
        if(direct_drag_controls_)teach_restore_controls_locked();
        // A FAULT must interrupt replay/teaching even while a task is paused.
        // Leave the robot in its latched safe FAULT; do not auto-clear or resume.
        const auto calibration_phase = model_calibration_phase_.load();
        if(calibration_phase == ModelCalibrationPhase::TEACHING ||
            calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION ||
            calibration_phase == ModelCalibrationPhase::PAUSED ||
            calibration_phase == ModelCalibrationPhase::GAIN_ADAPTATION ||
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
            {
                std::lock_guard<std::mutex> error_lock(model_calibration_mutex_);
                model_calibration_error_=machine_robot_error("model calibration stopped by safety fault",fault).what();
            }
            // Teaching and waiting-for-confirmation have no replay worker to
            // transition them into FAILED, so latch that phase here.
            if(calibration_phase == ModelCalibrationPhase::TEACHING ||
                calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION) {
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
        std::cout << " 3. 控制方式与执行增益\n";
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
        std::cout << "\n------------ 控制方式与执行增益 ------------\n";
        show_control_context();
        std::cout << " 1. 选择控制方式（失能后切换）\n";
        std::cout << " 2. 选择增益用途 / 进入拖拽\n";
        std::cout << " 3. 当前位置保持\n";
        std::cout << " 4. 选择模型前馈（失能后切换）\n";
        std::cout << " 5. 设置重力补偿比例（失能后设置）\n";
        std::cout << " 0. 返回主菜单\n";
        const auto selection = read_int("请选择: ");
        if(!selection || *selection == 0) return;
        try {
            switch(*selection) {
                case 1: select_reference_behavior(); break;
                case 2: set_impedance_mode(); break;
                case 3: cancel_and_hold(); break;
                case 4: set_model_feedforward_mode(); break;
                case 5: set_gravity_scale(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        } catch(const std::exception& error) {std::cout << "操作未完成: " << error.what() << '\n';}
    }

    void handle_motion_menu() {
        std::cout << "\n------------ 运动与命令 ------------\n";
        std::cout << " 1. 移动到关节目标\n";
        std::cout << " 2. 按关节增量移动\n";
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
            std::cout << " 1. 外力观测参数标定\n";
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

    void refresh_actuator_info_locked() {
        const auto snapshot = robot_.hardware_capabilities();
        const auto ordered = capabilities_for_joint_order(snapshot, cfg_.joint_names);
        if(ordered) {
            actuator_info_ = *ordered;
            actuator_info_after_connect_ = true;
        }
    }

    void activate() {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_command_sources();
        if(robot_.get_state() != RobotState::ACTIVE) actuator_info_after_connect_ = false;
        const auto result = robot_.activate();
        cfg_.safety = robot_.get_config().safety;
        if(!result) {
            std::cout << "activate() 失败：\n";
            print_fault(result.error());
            return;
        }
        refresh_actuator_info_locked();
        last_output_.reset();
        background_fault_reported_ = false;
        std::cout << "activate() 成功，后台 cycle() 自动运行\n";
        if(robot_.get_model_feedforward_mode() == ModelFeedforwardMode::GRAVITY && is_zero_vector(dynamics_.get_gravity_scale())) {
            std::cout << "[提示] 当前已选择 GRAVITY，但 gravity_scale 全为 0，实际重力补偿力矩仍为 0\n";
        }
    }

    void park_and_deactivate(const std::atomic<bool>* cancelled = nullptr) {
        std::unique_lock<std::mutex> lock(mutex_);
        park_last_error_.clear();
        if(cancelled && cancelled->load()) { park_last_error_ = "park interrupted by operator"; return; }
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

        // Stop an unfinished calibration from leaking temporary tracking gains
        // into a normal shutdown. Robot's ordinary configured gains stay intact.
        robot_.clear_calibration_hold_gains();
        robot_.clear_calibration_tracking_gains();
        // 停放使用低刚度 COMPLIANT_TRACKING 降低碰撞风险，同时暂停导纳避免 park reference 被外力偏移
        robot_.set_admittance_suspended(true);
        const auto resume_admittance = [this]() {
            robot_.set_admittance_suspended(false);
            };
        const auto interrupted = [&]() {
            if(!cancelled || !cancelled->load()) return false;
            clear_command_sources();
            if(robot_.get_state() == RobotState::ACTIVE)
                (void)runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD, Robot::Clock::now());
            park_last_error_ = "park interrupted by operator";
            resume_admittance();
            return true;
            };

        const auto mode_result = runtime_.set_impedance_mode(robot_, JointImpedanceMode::COMPLIANT_TRACKING, Robot::Clock::now());
        if(!mode_result) {
            park_last_error_ = "park failed: cannot enter COMPLIANT_TRACKING; robot was not automatically disabled";
            std::cout << "切换 COMPLIANT_TRACKING 失败：\n";
            print_fault(mode_result.error());
            resume_admittance();
            return;
        }

        stream_.enabled = true;
        stream_.completion_reported = false;
        stream_.park_mode = true;
        stream_.speed_scale = cfg_.shutdown.speed_scale;
        stream_.target_pos = cfg_.shutdown.park_pos;
        stream_.ref_pos = robot_.get_joint_state().pos;
        stream_.ref_vel.assign(cfg_.joint_names.size(), 0.0);
        stream_.last_update_time = Robot::Clock::now();
        stream_.has_last_update_time = true;
        last_output_.reset();
        background_fault_reported_ = false;

        std::cout << "开始回到配置的停放姿态\n";
        std::cout << "[停放] 前 " << cfg_.shutdown.compliant_phase_ratio * 100.0
            << "% 时间柔性快速回位，后 " << (1.0 - cfg_.shutdown.compliant_phase_ratio) * 100.0
            << "% 时间刚性精调，切换时重新以实测关节位置建立参考\n";
        print_vector("park_pos", cfg_.shutdown.park_pos);

        const Robot::TimePoint started_at = Robot::Clock::now();
        Robot::TimePoint last_progress_at = started_at;
        std::optional<Robot::TimePoint> settled_at;
        bool rigid_finish_mode = false;
        while(robot_.get_state() == RobotState::ACTIVE) {
            if(interrupted()) return;
            const std::uint64_t previous_cycle = cycle_counter_;
            cycle_cv_.wait_for(lock, std::chrono::milliseconds(20), [this, previous_cycle, cancelled]() {
                return cycle_counter_ != previous_cycle || robot_.get_state() != RobotState::ACTIVE || (cancelled && cancelled->load());
                });
            if(interrupted()) return;
            const Robot::TimePoint now = Robot::Clock::now();
            // Do not validate or deactivate based on an old frozen feedback frame.
            if(cycle_counter_ == previous_cycle || !last_output_) {
                if(std::chrono::duration<double>(now - started_at).count() >= cfg_.shutdown.timeout_s) {
                    clear_command_sources();
                    (void)runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD, now);
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

            // The nominal 75/25 division is temporal. Decelerate the compliant
            // reference shortly before the transition, so measured velocity can
            // pass the handover gate without requiring proximity to park_pos.
            const double elapsed_s = std::chrono::duration<double>(now - started_at).count();
            const double soft_end_s=cfg_.shutdown.timeout_s*cfg_.shutdown.compliant_phase_ratio;
            if(!rigid_finish_mode && elapsed_s>=std::max(0.0,soft_end_s-0.35))
                stream_.speed_scale=0.0;
            if(!rigid_finish_mode && serial_arm::park_time_to_rigid(
                elapsed_s, cfg_.shutdown.timeout_s, cfg_.shutdown.compliant_phase_ratio) &&
                serial_arm::park_safe_for_timed_rigid(
                    last_output_->joint_state.pos, last_output_->joint_state.vel,
                    cfg_.shutdown.park_pos, cfg_.shutdown.velocity_tolerance,
                    cfg_.shutdown.relaxed_tolerance_ratio)) {
                // Robot::set_impedance_mode resets its command history to actual
                // feedback, then the park streamer starts from this same state.
                // High rigidity never inherits the compliant reference backlog.
                const JointVector actual_pos = last_output_->joint_state.pos;
                const auto rigid_result = runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_TRACKING, now);
                if(!rigid_result) {
                    clear_command_sources();
                    (void)runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD, now);
                    park_last_error_ = "park failed: cannot switch to RIGID_TRACKING; robot remains enabled in RIGID_HOLD";
                    std::cout << "切换精确收敛模式失败，保持使能并取消自动失能\n";
                    resume_admittance();
                    return;
                }
                // The command starts at measured q, NOT at the old compliant
                // reference, and the gains are blended over 0.65 s instead of
                // jumping directly from Gray compliant Kp to rigid Kp.
                if(!robot_.begin_park_rigid_gain_ramp(0.65,now)) {
                    clear_command_sources();
                    (void)runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD,now);
                    park_last_error_="park failed: cannot initialize smooth rigid handover; robot remains enabled in RIGID_HOLD";
                    resume_admittance();
                    return;
                }
                stream_.ref_pos = actual_pos;
                stream_.ref_vel.assign(cfg_.joint_names.size(), 0.0);
                stream_.last_update_time = now;
                stream_.completion_reported = false;
                // Keep the configured safe speed limit during final approach.
                // The old 0.16 clamp made a 2.5 s rigid phase unnecessarily slow.
                stream_.speed_scale = cfg_.shutdown.speed_scale;
                rigid_finish_mode = true;
                settled_at.reset();
                std::cout << "[停放] 柔性阶段结束，实测位置续接并渐进增加刚度\n";
            }

            const bool reached = max_position_error <= cfg_.shutdown.position_tolerance &&
                max_velocity <= cfg_.shutdown.velocity_tolerance;
            if(rigid_finish_mode && reached) {
                if(!settled_at) settled_at = now;
                if(std::chrono::duration<double>(now - *settled_at).count() >= cfg_.shutdown.settle_time_s) break;
            }
            else settled_at.reset();

            if(std::chrono::duration<double>(now - last_progress_at).count() >= 1.0) {
                std::cout << "[停放] 最大位置误差=" << max_position_error << " rad (" << cfg_.joint_names[position_index] << ")，最大速度="
                    << max_velocity << " rad/s (" << cfg_.joint_names[velocity_index] << ")\n";
                last_progress_at = now;
            }

            if(elapsed_s > cfg_.shutdown.timeout_s) {
                clear_command_sources();
                (void)runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD, now);
                std::ostringstream detail;
                detail << "park timeout: position error=" << std::fixed << std::setprecision(6)
                    << max_position_error << " rad (" << cfg_.joint_names[position_index] << ", limit="
                    << cfg_.shutdown.position_tolerance << " rad); speed=" << max_velocity << " rad/s ("
                    << cfg_.joint_names[velocity_index] << ", limit=" << cfg_.shutdown.velocity_tolerance
                    << " rad/s); timeout=" << cfg_.shutdown.timeout_s
                    << "s; stage=" << (rigid_finish_mode ? "rigid_final" : "compliant_approach")
                    << ". Robot remains enabled in RIGID_HOLD; no automatic disable. "
                    << (rigid_finish_mode ? "Rigid finishing did not settle in time." :
                        "Timed rigid switch was delayed because joint motion was still too fast or feedback invalid; target distance is not a switching gate.");
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

        if(interrupted()) return;
        if(robot_.get_state() != RobotState::ACTIVE) {
            park_last_error_ = "park interrupted: robot left ACTIVE during approach (" + to_string(robot_.get_state()) + ")";
            std::cout << "停放过程中 Robot 离开 ACTIVE：" << to_string(robot_.get_state()) << '\n';
            resume_admittance();
            return;
        }

        clear_command_sources();
        const auto hold_result = runtime_.set_impedance_mode(robot_, JointImpedanceMode::RIGID_HOLD, Robot::Clock::now());
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
            if(interrupted()) return;
        }

        if(interrupted()) return;
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
        show_control_context();
        std::cout << "\n1 刚性\n2 柔性\n3 拖拽\n4 伺服跟踪（辨识与导纳共用）\n";
        const auto selection = read_int("增益用途: ");
        if(!selection || *selection < 1 || *selection > 4) {
            std::cout << "增益用途输入无效\n";
            return;
        }
        const JointGainProfile profile=*selection==1 ? JointGainProfile::RIGID :
            *selection==2 ? JointGainProfile::COMPLIANT : *selection==3 ? JointGainProfile::DRAG : JointGainProfile::SERVO_TRACKING;
        const auto mode=profile==JointGainProfile::DRAG ? JointImpedanceMode::COMPLIANT_DRAG :
            profile==JointGainProfile::COMPLIANT ? JointImpedanceMode::COMPLIANT_HOLD : JointImpedanceMode::RIGID_HOLD;
        machine_set_impedance_mode(mode,profile);
        show_control_context();
    }

    void show_control_context() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "控制方式: " << reference_behavior_name(robot_.get_reference_behavior())
            << " | 增益用途: " << to_string(robot_.get_gain_profile())
            << " | 任务状态: " << to_string(robot_.get_impedance_mode()) << '\n';
        if(direct_drag_controls_ || teach_controls_active_)
            std::cout << "当前为临时拖拽控制，结束后恢复会话选择\n";
    }

    void select_reference_behavior() {
        std::cout << "\n1 原生阻抗\n2 模型导纳\n";
        const auto selection=read_int("控制方式: ");
        if(!selection || (*selection!=1 && *selection!=2)) {
            std::cout << "控制方式输入无效\n";return;
        }
        machine_set_reference_behavior(*selection==1 ? "NATIVE_IMPEDANCE" : "MODEL_ADMITTANCE");
        show_control_context();
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
            std::cout << "gravity_scale 全为 0；失能后可在“控制方式与执行增益”中设置重力补偿比例\n";
        }
    }

    void start_absolute_stream() {
        const auto target = read_vector("输入目标位置，旋转关节 rad、直线关节 m，以空格分隔: ", cfg_.joint_names.size());
        const auto speed_scale = read_double("速度比例 (0, 1]，建议 0.1~0.5: ");
        if(!target || !speed_scale || *speed_scale <= 0.0 || *speed_scale > 1.0) {
            std::cout << "目标位置或速度比例输入无效\n";
            return;
        }

        try {machine_move_to(*target,*speed_scale,false);}
        catch(const std::exception& error) {std::cout << "移动未开始: " << error.what() << '\n';}
    }

    void start_relative_stream() {
        const auto delta = read_vector("输入相对位移 rad，以空格分隔: ", cfg_.joint_names.size());
        const auto speed_scale = read_double("速度比例 (0, 1]，建议 0.1~0.5: ");
        if(!delta || !speed_scale || *speed_scale <= 0.0 || *speed_scale > 1.0) {
            std::cout << "相对位移或速度比例输入无效\n";
            return;
        }

        try {machine_move_to(*delta,*speed_scale,true);}
        catch(const std::exception& error) {std::cout << "移动未开始: " << error.what() << '\n';}
    }

    void begin_stream(const JointVector& target, double speed_scale) {
        clear_command_sources();
        stream_.enabled = true;
        stream_.completion_reported = false;
        stream_.park_mode = false;
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
        try {machine_hold();std::cout << "已停止移动并保持当前位置\n";show_control_context();}
        catch(const std::exception& error) {std::cout << "保持未完成: " << error.what() << '\n';}
    }

    void show_robot_summary() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "RobotState             : " << to_string(robot_.get_state()) << '\n';
        std::cout << "JointImpedanceMode      : " << to_string(robot_.get_impedance_mode()) << '\n';
        std::cout << "ReferenceBehavior       : " << reference_behavior_name(robot_.get_reference_behavior()) << '\n';
        std::cout << "JointGainProfile        : " << to_string(robot_.get_gain_profile()) << '\n';
        std::cout << "ModelFeedforwardMode    : " << to_string(robot_.get_model_feedforward_mode()) << '\n';
        std::cout << "tracking_gain_profile   : " << to_string(cfg_.runtime.native_tracking_gain_profile()) << '\n';
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
        std::cout << (actuator_info_after_connect_ ?
            "Backend limits after successful activation (actuator/protocol units; not joint safety limits)\n" :
            "Configured defaults or last known limits; not confirmed for the current connection\n");
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
        const bool finish_drag=robot_.get_impedance_mode()==JointImpedanceMode::COMPLIANT_DRAG &&
            (mode==JointImpedanceMode::RIGID_HOLD || mode==JointImpedanceMode::COMPLIANT_HOLD);
        clear_command_sources();
        const auto result = runtime_.set_impedance_mode(robot_, mode,Robot::Clock::now(),finish_drag?HoldTargetPolicy::CAPTURE_MEASURED:HoldTargetPolicy::PRESERVE_NOMINAL);
        if(!result) {
            std::cout << "切换阻抗模式失败：\n";
            print_fault(result.error());
            return false;
        }
        if(model_calibration_task_active() && model_calibration_workflow_kind_!="gain" &&
           (mode==JointImpedanceMode::RIGID_TRACKING || mode==JointImpedanceMode::RIGID_HOLD)) {
            if(!robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING))return false;
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
        const std::function<void(const FrictionCalibrationTrajectory&)>& checkpoint = { }) {
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
        const auto& limits = cfg_.safety.limits;
        const auto& types=dynamics_.get_info().joint_types;
        for(const auto& q : trajectory.positions) {
            if(q.size() != cfg_.joint_names.size()) return false;
            for(std::size_t i = 0; i < q.size(); ++i) {
                if(!std::isfinite(q[i])) return false;
                if(i < limits.has_position_limit.size() && limits.has_position_limit[i] != 0) {
                    const double margin = std::max(types.at(i)=="prismatic"?.001:.05, limits.pos_margin[i]);
                    const double lower = limits.min_pos[i] + margin;
                    const double upper = limits.max_pos[i] - margin;
                    if(lower >= upper || q[i] < lower || q[i] > upper) {
                        std::cout << "[拒绝回放] " << cfg_.joint_names[i]
                            << " 示教轨迹进入距硬限位 " << margin
                            << (types.at(i)=="prismatic"?" m":" rad") << " 的标定保护区；请重新示教更保守的轨迹\n";
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

    void machine_require_friction_endpoint_locked(const JointVector& expected) const {
        if(robot_.get_state()!=RobotState::ACTIVE || !last_output_ || !feedback_time_ns_)
            throw std::runtime_error("friction replay requires fresh ACTIVE feedback; align the endpoint manually before replay");
        const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
        const double age_ms=now_ns>=feedback_time_ns_ ? double(now_ns-feedback_time_ns_)/1.0e6 :
            std::numeric_limits<double>::infinity();
        const auto& actual=last_output_->joint_state;
        const auto choice=serial_arm::calibration_recovery::select_endpoint(actual.pos,actual.vel,
            expected,expected,age_ms,dynamics_.get_info().joint_types);
        if(age_ms>150.0 || !choice.valid || !choice.at_endpoint)
            throw std::runtime_error("friction replay endpoint is not aligned or stationary; align the endpoint manually and reconfirm playback");
    }

    bool wait_for_friction_endpoint(const JointVector& expected, bool finish_pass) {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto deadline=Robot::Clock::now()+std::chrono::milliseconds(finish_pass ? 2000 : 150);
        std::uint64_t zero_velocity_cursor=0;
        bool zero_velocity_sent=false;
        while(Robot::Clock::now()<deadline) {
            if(machine_friction_stop_.load() || robot_.get_state()!=RobotState::ACTIVE) return false;
            if(last_output_ && feedback_time_ns_) {
                const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                const double age_ms=now_ns>=feedback_time_ns_ ? double(now_ns-feedback_time_ns_)/1.0e6 :
                    std::numeric_limits<double>::infinity();
                const auto& actual=last_output_->joint_state;
                const auto choice=serial_arm::calibration_recovery::select_endpoint(actual.pos,actual.vel,
                    expected,expected,age_ms,dynamics_.get_info().joint_types);
                if(age_ms>150.0 || !choice.valid || choice.max_position_ratio>1.0)
                    throw std::runtime_error("friction replay endpoint feedback is stale or misaligned; align the endpoint manually before replay");
                if(!finish_pass || (zero_velocity_sent && cycle_counter_>zero_velocity_cursor && choice.at_endpoint)) {
                    machine_require_friction_endpoint_locked(expected);
                    return true;
                }
                // Finish the path at its already-commanded endpoint with zero
                // velocity. This never creates a separate alignment movement.
                zero_velocity_cursor=cycle_counter_;
                if(machine_friction_stop_.load()) return false;
                if(!runtime_.submit_reference(robot_,JointPosVelCmd{expected,JointVector(expected.size(),0.0)},
                        Robot::Clock::now(),MotionSource::CALIBRATION)) return false;
                zero_velocity_sent=true;
            }
            const auto cursor=cycle_counter_;
            cycle_cv_.wait_until(lock,std::min(deadline,Robot::Clock::now()+std::chrono::milliseconds(100)),[&]() {
                return machine_friction_stop_.load() || robot_.get_state()!=RobotState::ACTIVE || cycle_counter_>cursor;
            });
        }
        throw std::runtime_error("friction replay endpoint did not become stationary; hold and align it manually before retrying");
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
        if(!wait_for_friction_endpoint(ordered.front(),false)) return false;

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
                if(machine_friction_stop_.load() || robot_.get_state() != RobotState::ACTIVE) return false;
                cursor = cycle_counter_;
                const auto command = runtime_.submit_reference(robot_,JointPosVelCmd{ ref_pos, ref_vel }, Robot::Clock::now(),MotionSource::CALIBRATION);
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
        return wait_for_friction_endpoint(ordered.back(),true);
    }

    void print_friction_calibration_yaml(
        const FrictionResidualModelCfg& friction,
        const std::string& indent) const {
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
        std::cout << "    reference_behavior: " << reference_behavior_name(a.reference_behavior) << '\n';
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
        std::cout << "\n========== 外力观测参数标定 ==========\n";
        std::cout << "按顺序完成静态残差标定、静态残差验证和摩擦参数标定\n";
        std::string line;
        if(!read_line("按 Enter 开始，输入 q 取消: ", line) || line == "q" || line == "Q") return;

        last_static_calibration_result_.reset();
        last_static_validation_result_.reset();
        last_friction_calibration_result_.reset();
        last_full_id_friction_calibration_result_.reset();

        std::cout << "\n[1/3] 静态残差标定\n";
        if(!run_admittance_static_calibration(false)) {
            std::cout << "外力观测参数标定失败，请按提示检查后重试\n";
            return;
        }
        std::cout << "\n[2/3] 静态残差验证\n";
        if(!run_admittance_static_validation()) {
            std::cout << "外力观测参数标定失败，请按提示检查后重试\n";
            return;
        }
        std::cout << "\n[3/3] 摩擦参数标定\n";
        if(!run_admittance_friction_calibration(false)) {
            std::cout << "外力观测参数标定失败，请按提示检查后重试\n";
            return;
        }

        std::cout << "外力观测参数标定通过，参数已应用到当前会话\n";
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
            std::cout << "\n------------ 外力观测参数标定 ------------\n";
            std::cout << " 1. 重力模型校正\n";
            std::cout << " 2. 一次性外力观测参数标定\n";
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
        bool allow_zero) {
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
        if(!cfg_.capability.admittance.enabled) {
            std::cout << "请先在 core.yaml 开启 admittance.enabled\n";
            return;
        }
        show_control_context();
        bool native=false,inactive=false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            native=robot_.get_reference_behavior()!=ReferenceBehavior::MODEL_ADMITTANCE;
            inactive=robot_.get_state()==RobotState::INACTIVE;
        }
        if(native) {
            if(!inactive) {
                std::cout << "当前为原生阻抗，M/D/K 不参与控制；请先停放失能，再选择模型导纳\n";
                return;
            }
            const auto selection=read_int("当前为原生阻抗；输入 1 切换模型导纳，0 返回: ");
            if(!selection || *selection!=1)return;
            try {machine_set_reference_behavior("MODEL_ADMITTANCE");}
            catch(const std::exception& error) {std::cout << "切换未完成: " << error.what() << '\n';return;}
        }
        if(!ensure_admittance_tuning_active()) return;
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
        std::cout << "tracking_gain_profile   : " << to_string(cfg_.runtime.native_tracking_gain_profile()) << '\n';
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
        std::cout << "park_compliant_ratio    : " << cfg_.shutdown.compliant_phase_ratio << '\n';
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
        motion_.finish("interrupted","motion replaced or stopped");
        runtime_.clear_commands();
        stream_ = StreamState{ };
    }

private:
    RobotCfg cfg_;
    std::string config_path_;
    std::string hardware_plugin_;
    std::string hardware_config_;
    std::string core_source_config_;
    std::string urdf_source_config_;
    std::string gravity_correction_source_config_;
    HardwareConfigOverrides hardware_overrides_;
    HardwareConnectionSummary connection_summary_;
    std::string robot_profile_;
    Dynamics dynamics_;
    std::vector<GravityLinkParameterInfo> model_calibration_links_cache_;
    HardwareLoader hardware_loader_;
    Robot robot_;
    ControlRuntime runtime_;
    JointMotionTracker motion_;
    HardwareCapabilities actuator_info_;
    bool actuator_info_after_connect_{ false };

    mutable std::mutex mutex_;
    std::condition_variable cycle_cv_;
    std::thread worker_;
    std::atomic<bool> worker_running_{ false };
    std::atomic<bool> quit_{ false };

    StreamState stream_;
    std::optional<RobotCycleOutput> last_output_;
    std::uint64_t last_cycle_monotonic_ns_{0};
    std::optional<AdmittanceStaticCalibrationResult> last_static_calibration_result_;
    std::optional<AdmittanceStaticValidationResult> last_static_validation_result_;
    std::optional<AdmittanceFrictionCalibrationResult> last_friction_calibration_result_;
    std::optional<AdmittanceFrictionCalibrationResult> last_full_id_friction_calibration_result_;
    bool gravity_verify_active_{ false };
    bool gravity_verify_drag_entered_{ false };
    bool gravity_verify_stopped_safely_{ false };
    std::string gravity_verify_result_{ "idle" };
    JointVector gravity_verify_original_scale_;
    JointVector gravity_verify_target_scale_;
    ModelFeedforwardMode gravity_verify_original_feed_{ ModelFeedforwardMode::NONE };
    std::vector<JointVector> gravity_verify_samples_;
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

    std::optional<TeachReplayPlan> teach_replay_plan_;
    std::atomic<int> teach_replay_phase_{0};
    std::atomic<bool> teach_replay_record_stop_{false};
    std::atomic<bool> teach_replay_stop_{false};
    std::atomic<int> teach_replay_completed_{0};
    std::thread teach_replay_worker_;
    std::optional<FrictionCalibrationTrajectory> teach_replay_trajectory_;
    JointVector teach_replay_record_start_;
    std::string teach_replay_error_;
    bool teach_replay_fault_interrupted_{false};
    bool teach_controls_active_{false};
    bool direct_drag_controls_{false};
    bool teach_original_suspended_{false};
    JointGainProfile teach_original_gain_profile_{JointGainProfile::RIGID};
    std::optional<JointGainProfile> teach_restore_gain_profile_;
    ReferenceBehavior teach_replay_behavior_{ReferenceBehavior::NATIVE_IMPEDANCE};
    JointGainProfile teach_replay_gain_profile_{JointGainProfile::RIGID};

    mutable std::mutex model_calibration_mutex_;
    std::condition_variable model_calibration_cv_;
    std::atomic<ModelCalibrationPhase> model_calibration_phase_{ ModelCalibrationPhase::IDLE };
    std::atomic<bool> model_calibration_cancel_{ false };
    std::atomic<bool> model_calibration_pause_{ false };
    std::atomic<bool> model_calibration_resume_confirmed_{ false };
    std::atomic<std::size_t> model_calibration_pose_group_{ 0 };
    std::atomic<std::size_t> model_calibration_rejected_static_groups_{ 0 };
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
    std::atomic<bool> model_calibration_alignment_active_{ false };
    std::atomic<bool> model_calibration_source_reversed_{ false };
    std::string model_calibration_source_task_id_;
    std::string model_calibration_error_;
    std::string model_calibration_preview_error_;
    std::string model_calibration_directory_;
    std::string model_calibration_core_fingerprint_;
    std::string model_calibration_urdf_fingerprint_;
    std::optional<FrictionCalibrationTrajectory> model_calibration_trajectory_;
    std::vector<ModelCalibrationPoseTarget> model_calibration_pose_targets_;
    double model_calibration_replay_rate_{ 0.0 };
    std::atomic<std::size_t> model_calibration_adaptation_passes_{0};
    std::atomic<std::size_t> model_calibration_adaptation_revisions_{0};
    std::atomic<bool> model_calibration_adaptation_converged_{false};
    std::string model_calibration_workflow_kind_{ "gravity" };
    std::string model_calibration_gain_mode_{ "all" };
    serial_arm::servo_tuning::Requirements gain_requirements_;
    std::optional<JointImpedanceGains> gain_trial_original_servo_;
    std::optional<HoldCompensationCfg> gain_trial_original_hold_compensation_;
    std::atomic<int> model_calibration_gain_stage_{2};
    std::string gain_probe_kind_,gain_probe_joint_,gain_probe_message_,gain_seed_source_,gain_probe_stage_,gain_probe_diagnostic_json_{"{}"};
    std::array<double,3> gain_probe_frequency_hz_{};
    std::vector<serial_arm::servo_tuning::NoiseEstimate> gain_noise_;
    std::vector<serial_arm::servo_tuning::Requirements> gain_plans_,gain_native_plans_;
    JointVector gain_probe_origin_;
    std::size_t gain_candidate_index_{0};
    double gain_probe_elapsed_s_{0},gain_probe_duration_s_{0},gain_probe_amplitude_rad_{0},gain_probe_kp_{0},gain_probe_kd_{0};
    JointVector gain_probe_reference_;
    JointImpedanceGains gain_seed_;
    bool runtime_gain_applied_{false};
    bool runtime_gravity_applied_{false};
    bool runtime_dynamics_applied_{false};
    std::unique_ptr<Dynamics> runtime_original_dynamics_;
    DynamicsCfg runtime_original_dynamics_cfg_;
    std::string runtime_dynamics_candidate_;
    std::vector<std::pair<JointImpedanceMode, JointImpedanceGains>> runtime_gain_original_modes_;
    std::optional<JointImpedanceGains> runtime_gain_original_identification_;
    std::optional<HoldCompensationCfg> runtime_gain_original_hold_compensation_;
    std::string runtime_gain_category_;
    std::vector<std::string> runtime_gain_purposes_;
    std::optional<serial_arm::calibration_gain_adaptation::PassMetrics> model_calibration_gain_validation_metrics_;
    JointVector runtime_gravity_original_scale_;
    std::string runtime_gain_candidate_;
    std::string runtime_gravity_candidate_;
    std::optional<JointImpedanceGains> model_calibration_active_tracking_gains_;
    std::optional<Robot::TimePoint> model_calibration_last_replay_gain_revision_;
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
    std::optional<JointGainProfile> model_calibration_original_gain_profile_;
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
    bool write_enabled=true;
    try {
        const auto root=YAML::LoadFile(options.config_path);
        const auto runtime=root["control"]?root["control"]["runtime"]:root["runtime"];
        if(runtime && runtime["write_enabled"])write_enabled=runtime["write_enabled"].as<bool>();
    } catch(const YAML::Exception& error) {std::cerr<<"配置加载失败: "<<error.what()<<'\n';return EXIT_FAILURE;}
    if((write_enabled || options.compare_config) && (options.hardware_plugin.empty() || options.hardware_config.empty())) {
        std::cerr << "--hardware-plugin and --hardware-config are required for hardware writing or connected comparison\n";
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
    HardwareCapabilities capabilities;
    HardwareConnectionSummary connection_summary;
    if(write_enabled) {
        HardwareLoader config_loader;
        auto config_bus=config_loader.load(options.hardware_plugin,options.hardware_config,options.hardware_overrides);
        if(!config_bus){std::cerr<<"HardwareLoader 失败: "<<to_string(config_bus.error())<<'\n';return EXIT_FAILURE;}
        capabilities=config_bus.value()->capabilities();
        const auto summary=load_hardware_connection_summary(options.hardware_config,options.hardware_overrides);
        if(!summary){std::cerr<<summary.error()<<'\n';return EXIT_FAILURE;}
        connection_summary=*summary;
    } else {
        const auto model=load_dynamics_cfg(options.config_path);
        if(!model){std::cerr<<"模型配置加载失败: "<<model.error().message<<'\n';return EXIT_FAILURE;}
        capabilities=simulation_capabilities(model->joint_names);
        connection_summary.bus="mock";connection_summary.serial_port="simulation";
    }
    const auto cfg_result=load_robot_cfg(options.config_path,capabilities);
    if(!cfg_result){std::cerr<<"配置加载失败: "<<cfg_result.error().message<<'\n';return EXIT_FAILURE;}

    TerminalApp app(
        cfg_result.value(),
        options.config_path,
        options.hardware_plugin,
        options.hardware_config,
        options.hardware_overrides,
        connection_summary,
        options.robot_profile,
        options.machine_mode);
    const auto init_result = app.initialize();
    if(!init_result) {
        std::cerr << init_result.error() << '\n';
        return EXIT_FAILURE;
    }
    return options.machine_mode ? app.run_machine() : app.run();
}
