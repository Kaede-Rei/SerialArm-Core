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
#include "serial_arm/dynamics/gain_trajectory_tuning.hpp"
#include "serial_arm/dynamics/gain_online_trajectory_tuning.hpp"
#include "serial_arm/dynamics/gain_candidate_admission.hpp"
#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include "serial_arm/dynamics/frequency_response_gain_tuning.hpp"
#include "serial_arm/dynamics/gain_measurement_center.hpp"
#include "serial_arm/dynamics/gain_taught_measurement_schedule.hpp"
#include "serial_arm/dynamics/gain_taught_anchor_visit.hpp"
#include "serial_arm/dynamics/gain_rigid_tuning.hpp"
#include "serial_arm/dynamics/mit_closed_loop_autotune.hpp"
#include "serial_arm/dynamics/urdf_nominal_gain_design.hpp"
#include "serial_arm/dynamics/gain_taught_path_following.hpp"
#include "serial_arm/dynamics/gain_drag_tuning.hpp"
#include "serial_arm/dynamics/gain_compliant_tuning.hpp"
#include "serial_arm/dynamics/gain_purpose_measurement.hpp"
#include "serial_arm/dynamics/gain_taught_local_correction.hpp"
#include "serial_arm/dynamics/gain_disturbance_plan.hpp"
#include "serial_arm/dynamics/calibration_recovery_alignment.hpp"
#include "serial_arm/core/park_policy.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/hardware/gain_context.hpp"
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
        case RobotErr::CALIBRATION_RESPONSE_UNSTABLE: return "CALIBRATION_RESPONSE_UNSTABLE";
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
// are never accepted as evidence of physical actuator capabilities
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
        // without waiting for another command or for the peer to close stdin
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
                       method != "candidate_overrides" && method != "model_calibration_status" && method != "model_calibration_gain_plan" &&
                       method != "calibration_status" && method != "teach_replay_status" &&
                       method != "deactivate" && method != "hold")
                        throw std::runtime_error("stop the active park operation before other commands");
                    if((teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3) &&
                       method != "status" && method != "teach_replay_status" && method != "model_calibration_gain_plan" &&
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
                        // not silently interrupt an active replay/recording
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
                    else if(method == "model_calibration_gain_plan") {
                        emit_reply(id,machine_model_calibration_gain_plan(params));
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
                        machine_hold(params && params["rigid"] && params["rigid"].as<bool>());
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
        // Join the local telemetry thread before fallible EOF cleanup. A
        // vanished device must produce a failed exit, not an exception escaping
        // run_machine() or destruction of a still-joinable local thread.
        stop_telemetry();
        std::string shutdown_error;
        try {
            if(machine_calibration_kind_ != "none") machine_calibration_cancel();
            if(machine_friction_thread_.joinable()) {
                machine_friction_stop_.store(true); cycle_cv_.notify_all(); machine_friction_thread_.join();
            }
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
            if(machine_robot_state() == RobotState::ACTIVE || machine_robot_state() == RobotState::FAULT)
                throw std::runtime_error("shutdown did not confirm INACTIVE; motor disable remains unconfirmed");
            if(gravity_verify_active_ && machine_robot_state() == RobotState::INACTIVE)
                machine_gravity_verify_cancel();
        }
        catch(const std::exception& error) { shutdown_error = error.what(); }
        catch(...) { shutdown_error = "unknown shutdown cleanup failure; motor disable remains unconfirmed"; }
        quit_.store(true);
        if(!shutdown_error.empty()) {
            std::cerr << "[退出清理失败] " << shutdown_error << "；失能尚未确认，不会自动清除故障或重新使能\n";
            std::lock_guard<std::mutex> output_lock(protocol_mutex);
            protocol << "{\"event\":\"shutdown_cleanup_failed\",\"disable_confirmed\":false,\"error\":\""
                     << machine_json_escape(shutdown_error) << "\"}\n" << std::flush;
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
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
        // disable while it finishes sampling, restores parameters or joins
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

    void machine_hold(bool force_rigid=false) {
        const bool stop_teaching = teach_replay_phase_.load()==1 || teach_replay_phase_.load()==3;
        if(stop_teaching) {
            teach_replay_record_stop_.store(true);
            teach_replay_stop_.store(true);
        }
        const bool stop_legacy = machine_calibration_kind_ != "none";
        if(stop_legacy) machine_friction_stop_.store(true);
        const bool stop_model=model_calibration_task_active();
        if(stop_model)model_calibration_pause_.store(true);
        cycle_cv_.notify_all();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clear_command_sources();
        }
        // Finish stopped workers before the final mode write, so their own
        // cleanup cannot replace the operator's current-position hold
        if(stop_teaching && teach_replay_worker_.joinable()) teach_replay_worker_.join();
        if(stop_legacy) machine_calibration_cancel(false);
        if(stop_model && model_calibration_worker_.joinable())model_calibration_worker_.join();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE) throw std::runtime_error("hold requires RobotState::ACTIVE");
            clear_command_sources();
            if(stop_teaching || direct_drag_controls_) teach_restore_controls_locked();
            const auto& strategy=robot_.get_admittance_cfg();
            const auto purpose=force_rigid ? (strategy.enabled && strategy.reference_behavior==ReferenceBehavior::MODEL_ADMITTANCE ? JointGainProfile::SERVO_TRACKING : JointGainProfile::RIGID) : robot_.get_gain_profile();
            const auto hold_mode=!force_rigid && purpose==JointGainProfile::COMPLIANT?JointImpedanceMode::COMPLIANT_HOLD:JointImpedanceMode::RIGID_HOLD;
            const auto result = runtime_.set_impedance_mode(robot_,hold_mode,Robot::Clock::now(),HoldTargetPolicy::CAPTURE_MEASURED);
            if(!result) throw machine_robot_error("hold", result.error());
            if(force_rigid || purpose==JointGainProfile::SERVO_TRACKING) {
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
        metadata.gain_budget_source_duration_s=gain_budget_source_.duration_s;
        metadata.gain_budget_source=gain_budget_source_.source;
        metadata.gain_budget_source_task_id=gain_budget_source_.task_id;
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
        // commanded Kp/Kd are recorded per-frame, not guessed at task start
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
        gain_cycle_reference_={};gain_reference_sample_ns_=0;
        robot_.clear_calibration_hold_gains();
        robot_.clear_calibration_tracking_gains();
        robot_.clear_calibration_disturbance();gain_capture_active_=false;
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
        gain_control_context_.clear();
        model_calibration_trajectory_.reset();
        gain_demonstrated_path_lower_.clear();gain_demonstrated_path_upper_.clear();
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
            gain_uses_demonstration_=false;gain_next_reverse_=true;gain_admittance_required_=false;gain_schedule_={};gain_budget_source_={};gain_deadline_.reset();gain_finished_at_.reset();gain_budget_exhausted_.store(false);gain_path_pass_current_=0;gain_effective_pass_duration_s_=0;gain_available_=false;gain_warnings_.clear();
            gain_seed_source_.clear();gain_seed_={};
            gain_termination_kind_="running";gain_termination_reason_.clear();gain_termination_blocked_joints_.clear();
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
        // the user repeat an otherwise valid operation
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
            if(recorded && friction_trajectory_inside_safe_replay_range(*recorded,false)) {
                teach_replay_trajectory_=std::move(*recorded);
                teach_replay_phase_.store(interrupted?4:2);
            } else {
                if(teach_replay_error_.empty())teach_replay_error_=recorded?friction_trajectory_range_error(*recorded,false):"trajectory is too short";
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
            if(!friction_trajectory_inside_safe_replay_range(*teach_replay_trajectory_,false)) {
                teach_replay_error_=friction_trajectory_range_error(*teach_replay_trajectory_,false);
                teach_replay_trajectory_.reset();
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
        if(trajectory.positions.size()<20 || !friction_trajectory_inside_safe_replay_range(trajectory,false))
            throw std::runtime_error(trajectory.positions.size()<20?"trajectory is too short":friction_trajectory_range_error(trajectory,false));
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
            throw std::runtime_error(friction_trajectory_range_error(planned));
        FrictionCalibrationTrajectory interpolated;
        interpolated.sample_dt=planned.sample_dt/4.0;
        for(std::size_t k=0;k+1<planned.positions.size();++k)
            for(int quarter=0;quarter<4;++quarter)
                interpolated.positions.push_back(serial_arm::calibration_retime::interpolate(
                    planned.positions,static_cast<double>(k)+0.25*quarter,planned.sample_dt,1.0).position);
        interpolated.positions.push_back(planned.positions.back());
        if(!friction_trajectory_inside_safe_replay_range(interpolated))
            throw std::runtime_error(friction_trajectory_range_error(interpolated));
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
            // command or fault clear is ever issued by this thread
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
                // a second GUI request or ever clearing a latched FAULT
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
        const auto indices = serial_arm::calibration_retime::ordered_stop_indices(
            retimed.positions, stop_positions, 0.025);
        std::vector<ModelCalibrationPoseTarget> result = targets;
        for(std::size_t i = 0; i < result.size(); ++i)
            result[i].trajectory_index = indices[i];
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
        // information on every observable gravity-parameter combination
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
    // confirm model_calibration_start in the usual safety dialog
    struct GainBudgetSource {double duration_s{0};std::string source,task_id;};
    static GainBudgetSource recorded_gain_budget_source(const std::filesystem::path& directory,double fallback_duration) {
        GainBudgetSource fallback{fallback_duration,"imported_trajectory_timestamps",directory.filename().string()};
        std::set<std::string> visited;auto current=directory;
        while(std::filesystem::is_directory(current) && visited.insert(current.lexically_normal().string()).second) {
            YAML::Node result;
            for(const auto* filename:{"metadata.json","result.json"}) {
                const auto path=current/filename;if(!std::filesystem::is_regular_file(path))continue;
                const auto saved=YAML::LoadFile(path.string());
                if(std::string(filename)=="result.json")result=saved;
                if(saved && saved["gain_budget_source_duration_s"]) {
                    const double duration=saved["gain_budget_source_duration_s"].as<double>();
                    if(std::isfinite(duration) && duration>0)return {duration,
                        saved["gain_budget_source"]?saved["gain_budget_source"].as<std::string>():"original_recorded_samples",
                        saved["gain_budget_source_task_id"]?saved["gain_budget_source_task_id"].as<std::string>():current.filename().string()};
                }
            }
            if(result && result["recorded_duration_s"]) {
                const double duration=result["recorded_duration_s"].as<double>();
                if(std::isfinite(duration) && duration>0)fallback={duration,result["source_task_id"]?"legacy_recorded_duration":"original_recorded_samples",result["task_id"]?result["task_id"].as<std::string>():current.filename().string()};
            }
            std::ifstream origin(current/"imported-from.txt");std::string path;
            if(!origin || !std::getline(origin,path) || path.empty())return fallback;
            auto previous=std::filesystem::path(path);if(previous.is_relative())previous=current/previous;
            if(!std::filesystem::is_directory(previous))return fallback;
            current=previous;
        }
        return fallback;
    }

    double gain_time_budget_s() const {
        return gain_budget_source_.duration_s>0 ? (model_calibration_gain_mode_=="all"?24.0:15.0)*gain_budget_source_.duration_s : 0;
    }

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
        // trajectory; apply the same bounded options as normal teaching
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
        // the entire Core fingerprint above includes the joint mapping
        const auto trajectory_path = std::filesystem::is_regular_file(directory / "trajectory.csv") ?
            directory / "trajectory.csv" : directory / "trajectory.checkpoint.csv";
        // Checkpoints are partial, unverified teaching recordings. Apply all
        // fingerprint, joint-order, finite/range and retiming checks below
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
        if(candidate.positions.size() < 2 || !friction_trajectory_inside_safe_replay_range(candidate,false))
            throw std::runtime_error(candidate.positions.size()<2?"imported trajectory is too short":friction_trajectory_range_error(candidate,false));
        // Prefer whichever recorded endpoint is already close to the robot.
        // Either direction traverses the exact same demonstrated geometric path;
        // this does NOT generate an unvalidated path from park to an endpoint
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
        // Do not silently assume the original endpoint when feedback is missing
        if(!endpoint_choice.valid)
            throw std::runtime_error("fresh robot feedback is required before importing a replay trajectory");
        if(endpoint_choice.reverse_recorded_path)
            std::reverse(candidate.positions.begin(),candidate.positions.end());
        // Always retime with *current* safety limits; never trust timestamps in
        // an imported CSV as a guarantee of velocity/acceleration feasibility
        const double recorded_duration = candidate.sample_dt * static_cast<double>(candidate.positions.size() - 1);
        const auto budget_source=recorded_gain_budget_source(directory,recorded_duration);
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
            throw std::runtime_error(friction_trajectory_range_error(candidate));
        FrictionCalibrationTrajectory sampled;
        sampled.sample_dt = candidate.sample_dt / 4.0;
        for(std::size_t k = 0; k + 1 < candidate.positions.size(); ++k)
            for(int n = 0; n < 4; ++n)
                sampled.positions.push_back(serial_arm::calibration_retime::interpolate(
                    candidate.positions, static_cast<double>(k) + 0.25 * n, candidate.sample_dt, 1.0).position);
        sampled.positions.push_back(candidate.positions.back());
        if(!friction_trajectory_inside_safe_replay_range(sampled))
            throw std::runtime_error(friction_trajectory_range_error(sampled));
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
        gain_budget_source_=budget_source;
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
            if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD,true))
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
            // Never try to override a latched robot FAULT
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        *model_calibration_trajectory_ = smooth_friction_trajectory(*model_calibration_trajectory_);
        if(!friction_trajectory_inside_safe_replay_range(*model_calibration_trajectory_,false)) {
            model_calibration_set_phase(ModelCalibrationPhase::FAILED);
            model_calibration_error_ = friction_trajectory_range_error(*model_calibration_trajectory_,false);
            model_calibration_recorder_.stop();
            // Teaching runs in COMPLIANT_DRAG. A rejected trajectory must not
            // leave that mode active merely because the GUI request failed.
            // Never try to override a latched robot FAULT
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        model_calibration_original_duration_s_ = model_calibration_trajectory_->sample_dt *
            static_cast<double>(model_calibration_trajectory_->positions.size() - 1);
        gain_budget_source_={model_calibration_original_duration_s_,"original_recorded_samples",model_calibration_task_id_};
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
            // The two slow friction passes use rate 0.5, with extra margin
            model_calibration_replay_rate_ = 1.0;
            if(!friction_trajectory_inside_safe_replay_range(*model_calibration_trajectory_))
                throw std::runtime_error(friction_trajectory_range_error(*model_calibration_trajectory_));
            // Validate the actual spline at quarter intervals: a continuous
            // Hermite spline may overshoot positions even if its knots are safe
            FrictionCalibrationTrajectory interpolated;
            interpolated.sample_dt = planned.sample_dt / 4.0;
            for(std::size_t k = 0; k + 1 < planned.positions.size(); ++k)
                for(int quarter = 0; quarter < 4; ++quarter)
                    interpolated.positions.push_back(serial_arm::calibration_retime::interpolate(
                        planned.positions, static_cast<double>(k) + 0.25 * quarter, planned.sample_dt, 1.0).position);
            interpolated.positions.push_back(planned.positions.back());
            if(!friction_trajectory_inside_safe_replay_range(interpolated))
                throw std::runtime_error(friction_trajectory_range_error(interpolated));
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
            // Never try to override a latched robot FAULT
            if(machine_robot_state() == RobotState::ACTIVE)
                (void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
            model_calibration_restore_runtime_hold();
            throw std::runtime_error(model_calibration_error_);
        }
        // Combined experimental calibration: one reverse replay with static
        // pauses and one forward dynamic replay. Both directions are logged for
        // friction estimation; the fitter must still pass observability checks.
        // This is a two-pass budget, NOT a guarantee of identified inertias.
        // Teaching wall time is independent of the replay clock
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
        // Joint limits, range checks, fault handling and manual cancellation remain mandatory
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
        // safety, command cycle watchdogs, and per-pose stability checks remain
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
        // drive a Damiao MIT motor beyond a frozen position reference
        using serial_arm::calibration_reference_watchdog::Heartbeat;
        using serial_arm::calibration_replay_supervisor::evaluate;
        using serial_arm::calibration_replay_supervisor::State;
        Heartbeat heartbeat(Robot::Clock::now());
        // Search and gravity acquisition pause at the soft follow threshold.
        // A frozen gain verification has a DIFFERENT instant-error acceptance
        // criterion. Keep the pause threshold consistent with that criterion
        // while the independent hard following guard always remains active
        const bool frozen_gain_verification=metrics && !allow_gain_revision;
        // Ordinary compliant playback is intentionally softer than frozen-gain
        // validation, while the independent hard-following limit stays unchanged.
        // A longer bounded catch-up lets compliant motors settle without
        // falsely reporting a calibration failure
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
                    // MIT velocity setpoint. Brake immediately at the SAME q
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
                // target: the safety bounds are solely hard actuator ceilings
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
            // when path time is paused. A genuinely stalled cycle still faults
            if(heartbeat.refresh_due(now) && !issue_reference(braking))return false;
        }
        return false;
    }

    // The URDF inertia and gravity gradient define physical gains in real
    // units, not a percentage of any user-supplied gain or safety ceiling.
    // Safety ceilings ONLY constrain commands.  The real robot response then
    // decides whether a pose is usable and whether the controller needs revision
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

    void record_gain_encoding(ModelCalibrationFrame& frame,const RobotCycleOutput& output) const {
        const auto n=cfg_.joint_names.size();frame.requested_kp=output.requested_kp;frame.requested_kd=output.requested_kd;
        frame.command_kp_code.assign(n,-1);frame.command_kd_code.assign(n,-1);
        for(std::size_t i=0;i<n;++i) {
            if(cfg_.safety.limits.kp_resolution.size()==n && cfg_.safety.limits.kp_resolution[i]>0 && output.joint_cmd.kp.size()==n)
                frame.command_kp_code[i]=std::llround(output.joint_cmd.kp[i]/cfg_.safety.limits.kp_resolution[i]);
            if(cfg_.safety.limits.kd_resolution.size()==n && cfg_.safety.limits.kd_resolution[i]>0 && output.joint_cmd.kd.size()==n)
                frame.command_kd_code[i]=std::llround(output.joint_cmd.kd[i]/cfg_.safety.limits.kd_resolution[i]);
        }
    }

    // The background recorder keeps all cycles untagged. This snapshot is
    // copied ONLY into an in-memory candidate window, never accepted early
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
        f.command_kp=output.joint_cmd.kp;f.command_kd=output.joint_cmd.kd;record_gain_encoding(f,output);
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
        // No fixed Kp/Kd ratio and no assumption that a quiet encoder is enough
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
                    // a handful of individually quiet 0.18 s slices
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
            // existing motor safety limits. No user-provided tuning assumptions
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
        // A failed pose is NEVER added to the online/offline dataset
        return finish();
    }

    struct ServoTrial {
        std::vector<std::vector<serial_arm::servo_tuning::Sample>> rows;
        std::vector<serial_arm::servo_tuning::Metrics> metrics;
        std::vector<serial_arm::servo_tuning::ResponseModel> models;
        std::vector<serial_arm::servo_tuning::Requirements> plans;
        std::vector<serial_arm::servo_tuning::MotionObservation> motion;
        JointVector center,entry_actual,min_position,max_position,disturbance_torque_amplitude;
        std::vector<serial_arm::servo_tuning::TrajectoryAssessment> trajectory_assessments;
        std::vector<serial_arm::servo_tuning::DisturbanceAssessment> disturbance_assessments;
        std::vector<serial_arm::servo_tuning::CompliantUseAssessment> compliant_assessments;
        std::vector<serial_arm::servo_tuning::PurposeInputPrior> input_priors;
        std::vector<serial_arm::servo_tuning::PurposePulsePlan> purpose_plans;
        std::vector<serial_arm::servo_tuning::PurposePulseAssessment> purpose_assessments;
        bool purpose_pulse{false};
        std::string record_path;
        bool trajectory_replay{false},reverse{false},online_adaptation{false};double playback_rate{1},duration_s{0};
        bool completed{false},available{false},passed{false};
    };
    static bool gain_direct_response_complete(const ServoTrial& trial) {
        return !trial.purpose_assessments.empty() && std::all_of(trial.purpose_assessments.begin(),trial.purpose_assessments.end(),
            [](const auto& axis){return axis.direct_response_complete && axis.compliant.measurement_complete;});
    }
    void gain_compliant_axes_json(std::ostream& out,const ServoTrial& trial) const {
        out<<'[';for(std::size_t i=0;i<trial.compliant_assessments.size();++i){if(i)out<<',';const auto& a=trial.compliant_assessments[i];
            out<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"measurement_complete\":"<<(a.measurement_complete?"true":"false")
                <<",\"input_measurement_complete\":"<<(a.input_measurement_complete?"true":"false")<<",\"passed\":"<<(a.passed?"true":"false")
                <<",\"yield_observed\":"<<(a.yield_observed?"true":"false")<<",\"stable_recovery\":"<<(a.stable_recovery?"true":"false")
                <<",\"unresolved_input_direction_mask\":"<<a.unresolved_input_direction_mask
                <<",\"unresolved_both_input_directions_at_ceiling\":"<<(a.unresolved_both_input_directions_at_ceiling?"true":"false")
                <<",\"peak_displacement_rad\":"<<a.peak_displacement_rad<<",\"displacement_limit_rad\":"<<a.displacement_limit_rad
                <<",\"initial_reference_error_rad\":"<<a.initial_reference_error_rad<<",\"final_equilibrium_error_rad\":"<<a.final_equilibrium_error_rad
                <<",\"release_recovery_s\":"<<a.release_recovery_s<<",\"final_drift_rad\":"<<gain_finite_json(a.final_drift_rad)
                <<",\"reason\":\""<<a.reason<<"\",\"validation_scope\":\""<<a.validation_scope<<"\"";
            if(i<trial.purpose_assessments.size()) {const auto& p=trial.purpose_assessments[i];
                out<<",\"direct_response_complete\":"<<(p.direct_response_complete?"true":"false")<<",\"model_identified\":false"
                    <<",\"hold_settled\":"<<(p.hold_settled?"true":"false")<<",\"hold_signed_displacement_rad\":"<<gain_finite_json(p.hold_signed_displacement_rad)
                    <<",\"effective_compliance_rad_nm\":"<<gain_finite_json(p.effective_compliance_rad_nm);}
            out<<'}';}
        out<<']';
    }
    struct GainQualificationFailure : std::runtime_error { using std::runtime_error::runtime_error; };
    struct GainBudgetReached : std::runtime_error { using std::runtime_error::runtime_error; };
    struct GainEndpointNotReady : std::runtime_error { using std::runtime_error::runtime_error; };
    void gain_add_warning(const std::string& warning) {
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);
        if(std::find(gain_warnings_.begin(),gain_warnings_.end(),warning)==gain_warnings_.end())gain_warnings_.push_back(warning);
    }
    static void gain_json_warnings(std::ostream& out,const std::vector<std::string>& warnings) {
        out<<'[';for(std::size_t i=0;i<warnings.size();++i){if(i)out<<',';machine_json_string(out,warnings[i]);}out<<']';
    }
    std::string gain_termination_json_locked() const {
        std::ostringstream out;
        out<<"{\"kind\":\""<<machine_json_escape(gain_termination_kind_)<<"\",\"reason\":\""<<machine_json_escape(gain_termination_reason_)
            <<"\",\"blocked_joint_names\":";
        gain_json_warnings(out,gain_termination_blocked_joints_);
        out<<",\"fault_interrupted\":"<<(model_calibration_fault_interrupted_.load()?"true":"false")<<'}';
        return out.str();
    }
    static void gain_endpoint_metrics_json(std::ostream& out,const serial_arm::servo_tuning::Metrics& m) {
        out<<",\"endpoint_window_available\":"<<(m.endpoint_window_available?"true":"false")
            <<",\"endpoint_passed\":false"
            <<",\"endpoint_peak_error_rad\":"<<m.endpoint_peak_error_rad
            <<",\"endpoint_rms_error_rad\":"<<m.endpoint_rms_error_rad
            <<",\"endpoint_bias_error_rad\":"<<m.endpoint_bias_error_rad
            <<",\"endpoint_window_start_s\":"<<m.endpoint_window_start_s
            <<",\"endpoint_window_end_s\":"<<m.endpoint_window_end_s
            <<",\"endpoint_measurement_samples\":"<<m.endpoint_measurement_samples
            <<",\"endpoint_reason\":\""<<(m.endpoint_window_available?"final_target_window_measured_reference_only":m.endpoint_reason)<<"\"";
    }

    static void gain_tuning_json(std::ostream& out,const serial_arm::servo_tuning::TrajectoryTuningEvidence& evidence) {
        out<<"{\"valid\":"<<(evidence.valid?"true":"false")<<",\"samples\":"<<evidence.samples
            <<",\"position_floor_rad\":"<<evidence.position_floor_rad<<",\"bias_rad\":"<<evidence.bias_rad
            <<",\"centered_rms_rad\":"<<evidence.centered_rms_rad<<",\"reference_rms_rad\":"<<evidence.reference_rms_rad
            <<",\"in_phase_gain\":"<<evidence.in_phase_gain<<",\"lag_s\":"<<evidence.lag_s
            <<",\"amplitude_error_rms_rad\":"<<evidence.amplitude_error_rms_rad<<",\"velocity_error_rms_rad\":"<<evidence.velocity_error_rms_rad<<'}';
    }
    static void gain_path_motion_json(std::ostream& out,const serial_arm::servo_tuning::TrajectoryAssessment& evidence) {
        const auto& m=evidence.residual_motion;
        out<<",\"local_reversal_overshoot_rad\":"<<evidence.local_reversal_overshoot_rad
            <<",\"motion_residual\":{\"valid\":"<<(m.valid?"true":"false")<<",\"sustained_uncommanded\":"<<(m.sustained_uncommanded?"true":"false")
            <<",\"reference_component_present\":"<<(m.reference_component_present?"true":"false")<<",\"mechanical_resonance_identified\":false"
            <<",\"examined_windows\":"<<m.examined_windows<<",\"repeated_windows\":"<<m.repeated_windows<<",\"position_floor_rad\":"<<m.position_floor_rad
            <<",\"frequency_hz\":"<<m.frequency_hz<<",\"amplitude_rad\":"<<m.amplitude_rad<<",\"reference_amplitude_rad\":"<<m.reference_amplitude_rad
            <<",\"explained_fraction\":"<<m.explained_fraction<<",\"duration_s\":"<<m.duration_s<<'}';
    }

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
        out<<"{\"completed\":"<<(completed?"true":"false")<<",\"available\":"<<(trial.available?"true":"false")<<",\"stage\":\""<<machine_json_escape(gain_probe_stage_)<<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
            <<"\",\"kind\":\""<<machine_json_escape(gain_probe_kind_)<<"\",\"candidate_index\":"<<gain_candidate_index_
            <<",\"kp\":"<<gains.kp[axis]<<",\"kd\":"<<gains.kd[axis]<<",\"protocol_kp_max\":"<<cfg_.safety.limits.max_kp[axis]<<",\"protocol_kd_max\":"<<cfg_.safety.limits.max_kd[axis]<<",\"amplitude_rad\":"<<req.amplitude_rad
            ;
        out<<",\"frequencies_hz\":";
        if(trial.trajectory_replay)out<<"null";else out<<'['<<req.frequencies_hz[0]<<','<<req.frequencies_hz[1]<<','<<req.frequencies_hz[2]<<']';
        out<<",\"samples\":"<<m.samples<<",\"measurement_samples\":"<<m.measurement_samples;
        if(m.samples) {
            out<<",\"motion_observation_basis\":\"reference_aligned_travel_v1\",\"motion_qualified\":"<<(motion.qualified?"true":"false")
                <<",\"bidirectional\":"<<(motion.bidirectional?"true":"false")<<",\"initial_error_rad\":"<<motion.initial_error_rad
                <<",\"actual_positive_travel_rad\":"<<motion.positive_travel_rad<<",\"actual_negative_travel_rad\":"<<motion.negative_travel_rad
                <<",\"motion_coherence\":"<<motion.coherence<<",\"motion_alignment_delay_s\":"<<motion.alignment_delay_s
                <<",\"reference_positive_span_rad\":"<<motion.reference_positive<<",\"reference_negative_span_rad\":"<<motion.reference_negative
                <<",\"actual_positive_span_rad\":"<<motion.actual_positive<<",\"actual_negative_span_rad\":"<<motion.actual_negative
                <<",\"minimum_motion_span_rad\":"<<motion.minimum_motion_span_rad
                <<",\"response_rms_ratio\":"<<m.response_rms_ratio<<",\"coherence\":";
            if(trial.trajectory_replay)out<<"null";else out<<(req.motion_probe?motion.coherence:model.coherence);
            out
                <<",\"continuous\":"<<(m.continuous?"true":"false")<<",\"limited_samples\":"<<m.limited_samples
                <<",\"observable\":"<<(m.observable?"true":"false")<<",\"metrics_passed\":false"
                <<",\"follow_frequency_hz\":";
            if(completed && !trial.trajectory_replay && !req.motion_probe && !req.stationary_probe)out<<model.tested_follow_frequency_hz;else out<<"null";
            out<<",\"encoder_step_rad\":"<<std::max(req.encoder_step_rad,motion.encoder_step_rad)<<",\"noise_rms_rad\":"<<req.noise_rms_rad
                <<",\"metrics\":{\"rms_error_rad\":"<<m.rms_error<<",\"peak_error_rad\":"<<m.peak_error<<",\"velocity_rms_rad_s\":"<<m.velocity_rms
                <<",\"overshoot_rad\":"<<m.overshoot<<",\"settling_time_s\":"<<m.settling_s<<",\"tail_error_rad\":"<<m.tail_error_rad;
            if(trial.trajectory_replay)gain_endpoint_metrics_json(out,m);
            out<<'}';
            if(trial.trajectory_replay){out<<",\"oscillation_source\":\"fresh_position_residual_and_tail_hysteresis\",\"tail_position_reversals\":"<<trial.trajectory_assessments[axis].tail_position_reversals<<",\"tuning\":";gain_tuning_json(out,trial.trajectory_assessments[axis].tuning);gain_path_motion_json(out,trial.trajectory_assessments[axis]);}
            out<<",\"measurement_notes\":[";bool first=true;
            const auto note=[&](bool condition,const char* key){if(condition){if(!first)out<<',';out<<'"'<<key<<'"';first=false;}};
            note(m.measurement_samples<100,"measurement_samples_incomplete");
            note(!m.continuous,"discontinuous_feedback");note(m.limited_samples>0,"limited_samples_recorded");
            note(trial.trajectory_replay && !m.endpoint_window_available,"final_target_window_incomplete");
            out<<']';
        }
        out<<",\"acceptance_scope\":\"measured_response_reference_only_v1\",\"reference_only\":true,\"accuracy_thresholds_enabled\":false"
            <<",\"reason\":\""<<machine_json_escape(reason)<<"\",\"record_path\":\""<<machine_json_escape(record)<<"\"}";
        gain_probe_diagnostic_json_=out.str();gain_probe_message_=reason;
    }

    // Captured by the control loop under mutex_: UI/worker wakeups never
    // determine peak acceptance or hide a skipped feedback cycle
    void gain_capture_begin(const std::string& label,Robot::TimePoint start,bool accepted=false,
        const std::vector<serial_arm::servo_tuning::Requirements>* actual_plans=nullptr,
        bool independent_input=false,const std::vector<bool>* input_eligible=nullptr) {
        gain_capture_label_=label;gain_capture_start_=start;gain_capture_accepted_=accepted;
        gain_capture_integral_.clear();gain_capture_observer_.clear();gain_capture_rows_.assign(cfg_.joint_names.size(),{});gain_capture_stamps_.assign(cfg_.joint_names.size(),0);
        gain_motion_guards_.clear();
        for(std::size_t axis=0;axis<cfg_.joint_names.size();++axis) {
            const auto& plan=actual_plans?actual_plans->at(axis):gain_requirements_;
            gain_motion_guards_.emplace_back(plan,axis<gain_noise_.size()?gain_noise_[axis]:serial_arm::servo_tuning::NoiseEstimate{});
            if(independent_input && actual_plans && plan.amplitude_rad>0 && (!input_eligible || input_eligible->at(axis)))
                gain_motion_guards_.back().set_independent_input_frequencies(plan.frequencies_hz);
        }
        gain_capture_active_=true;
    }
    void gain_capture_cycle(const RobotCycleOutput& output,Robot::TimePoint now) {
        if(!gain_capture_active_ || now<gain_capture_start_)return;
        const auto n=cfg_.joint_names.size();
        if(output.joint_state.pos.size()!=n || output.joint_cmd.pos.size()!=n || output.actuator_state.feedback_received_ns.size()!=n)return;
        const double time=std::chrono::duration<double>((output.command_written_ns?Robot::TimePoint(std::chrono::nanoseconds(output.command_written_ns)):now)-gain_capture_start_).count();
        gain_capture_integral_.push_back({output.hold_integral_torque,output.hold_integrator_freeze_reasons});gain_capture_observer_.push_back(output.observer_valid);
        std::optional<std::pair<std::size_t,serial_arm::servo_tuning::MotionGuardResult>> unstable;
        for(std::size_t axis=0;axis<n;++axis) {
            serial_arm::servo_tuning::Sample row;row.time_s=time;
            row.reference=gain_capture_accepted_?output.joint_cmd.pos[axis]:
                (output.reference_behavior==ReferenceBehavior::MODEL_ADMITTANCE && output.virtual_ref_pos.size()==n?output.virtual_ref_pos[axis]:output.nominal_ref_pos.size()==n?output.nominal_ref_pos[axis]:output.joint_cmd.pos[axis]);
            row.reference_velocity=gain_capture_accepted_?output.joint_cmd.vel[axis]:
                (output.reference_behavior==ReferenceBehavior::MODEL_ADMITTANCE && output.virtual_ref_vel.size()==n?output.virtual_ref_vel[axis]:output.nominal_ref_vel.size()==n?output.nominal_ref_vel[axis]:output.joint_cmd.vel[axis]);
            row.actual=output.joint_state.pos[axis];row.velocity=output.joint_state.vel[axis];
            if(output.joint_state.tor.size()==n)row.measured_torque_nm=output.joint_state.tor[axis];
            if(output.joint_cmd.kp.size()==n && output.joint_cmd.kd.size()==n){row.actual_kp=output.joint_cmd.kp[axis];row.actual_kd=output.joint_cmd.kd[axis];row.actual_gains_known=true;}
            const auto stamp=output.actuator_state.feedback_received_ns[axis];row.feedback_sequence=stamp;
            row.new_measurement=stamp>gain_capture_stamps_[axis] && stamp>0;gain_capture_stamps_[axis]=std::max(stamp,gain_capture_stamps_[axis]);
            row.feedback_time_s=std::chrono::duration<double>(Robot::TimePoint(std::chrono::nanoseconds(stamp))-gain_capture_start_).count();row.command_time_s=time;
            if(gain_cycle_reference_ && gain_reference_sample_ns_) {
                row.reference_sample_time_s=std::chrono::duration<double>(Robot::TimePoint(std::chrono::nanoseconds(gain_reference_sample_ns_))-gain_capture_start_).count();
                row.reference_sample_time_known=true;
            }
            row.limited=(axis<output.reference_limited.size() && output.reference_limited[axis]) || (axis<output.torque_limited.size() && output.torque_limited[axis]);
            if(output.observer_valid && output.residual_raw.size()==n){row.residual=output.residual_raw[axis];row.residual_valid=true;}
            row.independent_disturbance_nm=output.calibration_disturbance.size()==n?output.calibration_disturbance[axis]:0;
            if(gain_online_input_plan_.size()==n){row.planned_independent_disturbance_nm=gain_online_input_plan_[axis];row.independent_input_planned_known=true;}
            // Accepted non-PD torque includes model/hold/handover additions;
            // independent D is separated exactly once for the MIT numerator
            if(output.joint_cmd.tor.size()==n){row.applied_feedforward_nm=output.joint_cmd.tor[axis]-row.independent_disturbance_nm;row.feedforward_known=true;}
            row.independent_disturbance_known=true;gain_capture_rows_[axis].push_back(row);
            if(axis<gain_motion_guards_.size()) {
                if(axis<gain_noise_.size())gain_motion_guards_[axis].update_noise(gain_noise_[axis]);
                auto result=gain_motion_guards_[axis].observe(row);
                // Learn only repeated raw position-code spacing, never the
                // RMS of a moving response. Retain it across capture windows
                // so endpoint evidence is available to the first full path.
                if(result.encoder_step_known && axis<gain_noise_.size()) {
                    auto& step=gain_noise_[axis].encoder_step_rad;
                    step=step>0?std::min(step,result.effective_encoder_step_rad):result.effective_encoder_step_rad;
                }
                // Reject a trial controller at the first repeated excursion;
                // the existing sustained-motion guard remains armed throughout
                // rollback and still faults if the response does not recover.
                if(gain_discovery_rollback_ && !gain_discovery_reverted_ && result.position_reversals>=2 &&
                   result.position_reversals>=result.reference_reversals+2 &&
                   result.actual_travel_rad>2.5*result.reference_travel_rad+
                       6*result.hysteresis_rad && result.actual_peak_to_peak_rad>2*result.hysteresis_rad) {
                    if(robot_.set_calibration_tracking_gains(*gain_discovery_rollback_,robot_.get_impedance_mode()))
                        {gain_discovery_reverted_=true;gain_discovery_rollback_axis_=axis;}
                }
                if(result.triggered && !unstable)unstable=std::make_pair(axis,std::move(result));
            }
        }
        if(unstable) {
            const auto axis=unstable->first;const auto& evidence=unstable->second;
            gain_capture_active_=false;
            // Do not spend the recording/assessment interval holding the
            // rejected high-gain controller. Use the established fault exit.
            const auto stopped=robot_.stop_calibration_response();
            std::ostringstream record;record<<std::setprecision(17)<<"{\"kind\":\"calibration_response_guard\",\"stage\":\""<<machine_json_escape(gain_capture_label_)
                <<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"time_s\":"<<time<<",\"reason\":\""<<evidence.reason
                <<"\",\"position_reversals\":"<<evidence.position_reversals<<",\"reference_reversals\":"<<evidence.reference_reversals
                <<",\"independent_input_reversals\":"<<evidence.independent_input_reversals<<",\"requested_motion_reversals\":"<<evidence.requested_motion_reversals
                <<",\"independent_input_known\":"<<(evidence.independent_input_known?"true":"false")<<",\"independent_input_span_nm\":"<<evidence.independent_input_span_nm
                <<",\"independent_input_frequencies_known\":"<<(evidence.independent_input_frequencies_known?"true":"false")
                <<",\"input_tone_response_fit_available\":"<<(evidence.input_tone_response_fit_available?"true":"false")
                <<",\"input_tone_response_explained\":"<<(evidence.input_tone_response_explained?"true":"false")
                <<",\"input_tone_unexplained_oscillation\":"<<(evidence.input_tone_unexplained_oscillation?"true":"false")
                <<",\"input_tone_response_r_squared\":"<<evidence.input_tone_response_r_squared
                <<",\"input_tone_response_condition_number\":"<<evidence.input_tone_response_condition_number
                <<",\"input_tone_residual_peak_to_peak_rad\":"<<evidence.input_tone_residual_peak_to_peak_rad
                <<",\"input_tone_explanation_reason\":\""<<machine_json_escape(evidence.input_tone_explanation_reason)<<'"'
                <<",\"actual_travel_rad\":"<<evidence.actual_travel_rad<<",\"reference_travel_rad\":"<<evidence.reference_travel_rad
                <<",\"actual_peak_to_peak_rad\":"<<evidence.actual_peak_to_peak_rad<<",\"reference_peak_to_peak_rad\":"<<evidence.reference_peak_to_peak_rad
                <<",\"completed_alternating_travel_rad\":"<<evidence.completed_alternating_travel_rad
                <<",\"repeated_leg_min_rad\":"<<evidence.repeated_leg_min_rad<<",\"repeated_leg_max_rad\":"<<evidence.repeated_leg_max_rad
                <<",\"repeated_comparable_excursions\":"<<evidence.repeated_comparable_excursions
                <<",\"residual_peak_to_peak_rad\":"<<evidence.residual_peak_to_peak_rad<<",\"window_s\":"<<evidence.window_s
                <<",\"effective_encoder_step_rad\":"<<evidence.effective_encoder_step_rad<<",\"hysteresis_rad\":"<<evidence.hysteresis_rad
                <<",\"encoder_step_known\":"<<(evidence.encoder_step_known?"true":"false")<<",\"fresh_position_codes\":"<<evidence.fresh_position_codes
                <<",\"encoder_step_evidence_pairs_capped_at_two\":"<<evidence.encoder_step_evidence_pairs
                <<",\"actual_kp\":"<<output.joint_cmd.kp[axis]<<",\"actual_kd\":"<<output.joint_cmd.kd[axis]
                <<",\"reaction\":\"latched_existing_stop_hold\",\"mechanical_resonance_identified\":false}";
            gain_guard_records_.push_back(record.str());
            if(stopped && robot_.get_last_fault())report_background_fault(*robot_.get_last_fault());
        }
    }
    bool gain_trial_fits(double duration,double reserve=2.0) const {
        return !gain_deadline_ || std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count()>=duration+reserve;
    }
    void gain_require_trial(double duration) {
        if(model_calibration_cancel_.load() || model_calibration_pause_.load())throw std::runtime_error("gain calibration interrupted before the next trial");
        gain_check_budget();
        if(!gain_trial_fits(duration)) {gain_budget_exhausted_.store(true);throw GainBudgetReached("gain calibration remaining budget cannot fit the next complete measurement and exit");}
    }

    void gain_check_budget() {
        if(gain_deadline_ && Robot::Clock::now()+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(std::min(2.0,.1*gain_budget_source_.duration_s)))>=*gain_deadline_) {
            gain_budget_exhausted_.store(true);
            throw GainBudgetReached("gain calibration recorded-duration wall-time budget exhausted; completed purposes and recordings were preserved");
        }
    }

    // Called under mutex_ at the end of a measured motion, before copying data,
    // fitting models or writing CSV. HOLD owns the final nominal reference;
    // background analysis must not leave an expired streaming reference alive.
    void gain_hold_for_assessment_locked(const JointImpedanceGains& gains,bool capture_measured=false) {
        gain_cycle_reference_={};
        gain_reference_sample_ns_=0;
        gain_capture_active_=false;
        robot_.clear_calibration_disturbance();
        if(robot_.get_state()!=RobotState::ACTIVE ||
           !runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD,Robot::Clock::now(),capture_measured?HoldTargetPolicy::CAPTURE_MEASURED:HoldTargetPolicy::PRESERVE_NOMINAL) ||
           !robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING) ||
           !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::RIGID_HOLD))
            throw std::runtime_error("cannot enter hold before gain assessment");
        robot_.set_admittance_suspended(true);
        gain_capture_label_+="-assessment";
    }

    // Called by the existing serialized control executor. Reference position,
    // velocity and independent input share its monotonic timeline; analysis/UI
    // wakeups do not sample or advance an executable trajectory.
    void gain_update_cycle_reference(Robot::TimePoint now) {
        if(!gain_cycle_reference_)return;
        try {
            gain_check_budget();
            if(model_calibration_cancel_.load() || model_calibration_pause_.load()) {
                gain_cycle_reference_={};gain_reference_sample_ns_=0;robot_.clear_calibration_disturbance();
                (void)runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD,now,HoldTargetPolicy::CAPTURE_MEASURED);return;
            }
            if(now>gain_cycle_reference_until_)throw GainQualificationFailure("calibration_reference_timeline_expired");
            gain_cycle_reference_(now);
            gain_reference_sample_ns_=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        } catch(...) {
            gain_cycle_reference_error_=std::current_exception();gain_cycle_reference_={};
            robot_.clear_calibration_disturbance();
            (void)runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD,now,HoldTargetPolicy::CAPTURE_MEASURED);
        }
    }

    JointVector gain_nominal_damping_prior() const {
        JointVector prior(cfg_.joint_names.size(),0);
        for(std::size_t i=0;i<prior.size();++i)if(i<gain_seed_.kp.size() && gain_seed_.kp[i]>0 && i<gain_model_initialization_.inertia_max.size())
            prior[i]=gain_seed_.kd[i]/(2*std::sqrt(gain_model_initialization_.inertia_max[i]*gain_seed_.kp[i]));
        return prior;
    }

    void gain_write_response_csv(const std::string& record,const std::vector<std::vector<serial_arm::servo_tuning::Sample>>& rows,
        const std::function<double(double)>& source_clock={}) const {
        std::ofstream csv(std::filesystem::path(model_calibration_directory_)/record);
        if(!csv)throw std::runtime_error("cannot write gain response evidence");
        csv<<"time_s,joint,reference,reference_velocity,actual,velocity,residual,residual_valid,limited,new_measurement,feedforward_nm,independent_disturbance_nm,command_time_s,feedback_time_s,feedback_sequence,actual_kp,actual_kd,actual_gains_known,reference_sample_time_s,reference_sample_time_known,measured_torque_nm,planned_independent_disturbance_nm,independent_input_planned_known";if(source_clock)csv<<",trajectory_source_time_s";csv<<'\n'<<std::setprecision(17);
        for(std::size_t axis=0;axis<rows.size();++axis)for(const auto& row:rows[axis]) {
            csv<<row.time_s<<','<<cfg_.joint_names[axis]<<','<<row.reference<<','<<row.reference_velocity<<','<<row.actual<<','<<row.velocity<<','<<row.residual<<','<<row.residual_valid<<','<<row.limited<<','<<row.new_measurement<<','<<row.applied_feedforward_nm<<','<<row.independent_disturbance_nm<<','<<row.command_time_s<<','<<row.feedback_time_s<<','<<row.feedback_sequence<<','<<row.actual_kp<<','<<row.actual_kd<<','<<row.actual_gains_known<<','<<row.reference_sample_time_s<<','<<row.reference_sample_time_known<<','<<row.measured_torque_nm<<','<<row.planned_independent_disturbance_nm<<','<<row.independent_input_planned_known;if(source_clock)csv<<','<<source_clock(row.reference_sample_time_known?row.reference_sample_time_s:row.time_s);csv<<'\n';}
        csv.flush();if(!csv)throw std::runtime_error("gain response evidence writing failed");
    }

    void gain_remember_completed_endpoint_locked(const JointVector& endpoint,const std::string& record) {
        gain_completed_endpoint_reference_.clear();gain_completed_endpoint_record_.clear();
        if(last_output_ && serial_arm::calibration_recovery::emitted_endpoint_matches(last_output_->joint_cmd.pos,endpoint)) {
            gain_completed_endpoint_reference_=endpoint;gain_completed_endpoint_record_=record;
        }
    }
    struct GainEndpointObservation {
        JointVector position,velocity,feedback_age_ms,reference_origin;
        double age_ms{0},elapsed_s{0};bool reference_continuation{false};std::string continuation_record;
    };
    GainEndpointObservation gain_endpoint_observation(bool budgeted,const JointVector& target) {
        using namespace serial_arm::servo_tuning;
        constexpr double timeout_s=2.0,quiet_s=.15;
        const auto n=cfg_.joint_names.size();std::vector<std::vector<Sample>> rows(n);std::vector<std::uint64_t> stamps(n,0);JointVector fresh_at(n,0);
        if(target.size()!=n || !std::all_of(target.begin(),target.end(),[](double value){return std::isfinite(value);}))throw std::runtime_error("gain endpoint target is invalid");
        std::uint64_t cursor=0;
        {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(robot_.get_state()!=RobotState::ACTIVE ||
               (robot_.get_impedance_mode()!=JointImpedanceMode::RIGID_HOLD && !runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD)))throw std::runtime_error("gain endpoint observation requires an ACTIVE hold");
            cursor=cycle_counter_;}
        const auto started=Robot::Clock::now();double quiet_since=-1;GainEndpointObservation observation;observation.position.resize(n);observation.velocity.resize(n);observation.feedback_age_ms.resize(n);
        while(true) {
            if(budgeted)gain_check_budget();
            std::unique_lock<std::mutex> lock(mutex_);
            if(robot_.get_state()!=RobotState::ACTIVE || (budgeted && (model_calibration_cancel_.load() || model_calibration_pause_.load())))throw std::runtime_error("gain endpoint observation interrupted");
            if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE;}) || !last_output_ || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("gain endpoint observation lost feedback");
            cursor=cycle_counter_;const auto& out=*last_output_;const double time=std::chrono::duration<double>(Robot::Clock::now()-started).count();
            if(out.joint_state.pos.size()!=n || out.actuator_state.feedback_received_ns.size()!=n)throw std::runtime_error("gain endpoint feedback dimensions changed");
            for(std::size_t axis=0;axis<n;++axis) {
                Sample row;row.time_s=time;row.actual=out.joint_state.pos[axis];const auto stamp=out.actuator_state.feedback_received_ns[axis];
                if(!std::isfinite(row.actual))throw std::runtime_error("gain endpoint has non-finite feedback");
                row.new_measurement=stamp>stamps[axis] && stamp>0;
                if(row.new_measurement){if(time-fresh_at[axis]>.06)throw std::runtime_error("gain endpoint observation lost per-joint feedback");fresh_at[axis]=time;stamps[axis]=stamp;}
                if(time-fresh_at[axis]>.06)throw std::runtime_error("gain endpoint observation lost per-joint feedback");
                rows[axis].push_back(row);observation.position[axis]=row.actual;
            }
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            observation.age_ms=0;observation.elapsed_s=time;bool ready=time>=.3;
            const auto& types=dynamics_.get_info().joint_types;
            for(std::size_t axis=0;axis<n;++axis) {
                if(!stamps[axis] || stamps[axis]>now_ns)throw std::runtime_error("gain endpoint observation has invalid feedback timestamps");
                observation.feedback_age_ms[axis]=static_cast<double>(now_ns-stamps[axis])/1e6;
                observation.age_ms=std::max(observation.age_ms,observation.feedback_age_ms[axis]);
                auto noise=axis<gain_noise_.size()?gain_noise_[axis]:NoiseEstimate{};
                // A handover displacement is not an encoder-resolution
                // measurement. Retain the independent baseline/path estimate
                observation.velocity[axis]=std::max(drag_position_speed(rows[axis],noise,.25),drag_position_speed(rows[axis],noise,.1));
                ready=ready && std::abs(observation.position[axis]-target[axis])<=serial_arm::calibration_recovery::position_tolerance(types.at(axis)) &&
                    observation.velocity[axis]<=serial_arm::calibration_recovery::velocity_tolerance(types.at(axis));
            }
            const auto continuation=budgeted?serial_arm::calibration_recovery::automatic_endpoint_continuation(
                observation.position,out.joint_cmd.pos,gain_completed_endpoint_reference_,target,
                observation.feedback_age_ms,robot_.get_state()==RobotState::ACTIVE):serial_arm::calibration_recovery::AutomaticEndpointContinuation{};
            observation.reference_continuation=continuation.valid;
            observation.reference_origin=continuation.valid?continuation.reference_origin:observation.position;
            observation.continuation_record=continuation.valid?gain_completed_endpoint_record_:std::string{};
            if(ready){if(quiet_since<0)quiet_since=time;}else quiet_since=-1;
            const bool settled=quiet_since>=0 && time-quiet_since>=quiet_s;
            const auto reason=continuation.valid?"completed_endpoint_reference_continuation":settled?"endpoint_ready":time>=timeout_s?"endpoint_alignment_required":"endpoint_settling";
            lock.unlock();
            {std::lock_guard<std::mutex> probe_lock(model_calibration_mutex_);std::ostringstream summary;summary<<std::setprecision(17);
                summary<<"{\"completed\":false,\"stage\":\"trajectory_endpoint\",\"reason\":\""<<reason<<"\",\"elapsed_s\":"<<time<<",\"timeout_s\":"<<timeout_s<<",\"age_ms\":"<<observation.age_ms
                    <<",\"reference_continuation\":"<<(continuation.valid?"true":"false")<<",\"continuation_record_path\":\""<<machine_json_escape(observation.continuation_record)<<"\",\"reference_origin_rad\":";
                machine_json_joint_vector(summary,observation.reference_origin);summary<<",\"axes\":[";
                for(std::size_t axis=0;axis<n;++axis){if(axis)summary<<',';const double position_tolerance=serial_arm::calibration_recovery::position_tolerance(types.at(axis)),velocity_tolerance=serial_arm::calibration_recovery::velocity_tolerance(types.at(axis));
                    summary<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"actual\":"<<observation.position[axis]<<",\"target\":"<<target[axis]<<",\"error\":"<<observation.position[axis]-target[axis]<<",\"velocity\":"<<observation.velocity[axis]<<",\"position_tolerance\":"<<position_tolerance<<",\"velocity_tolerance\":"<<velocity_tolerance<<",\"feedback_age_ms\":"<<observation.feedback_age_ms[axis]<<",\"ready\":"<<((time>=.3 && std::abs(observation.position[axis]-target[axis])<=position_tolerance && observation.velocity[axis]<=velocity_tolerance)?"true":"false")<<'}';}
                summary<<"]}";gain_probe_stage_="trajectory_endpoint";gain_probe_message_=reason;gain_probe_reference_=target;
                gain_probe_elapsed_s_=time;gain_probe_duration_s_=timeout_s;gain_probe_diagnostic_json_=summary.str();}
            if(continuation.valid || settled)return observation;
            if(time>=timeout_s)throw GainEndpointNotReady("demonstrated trajectory endpoint still needs alignment after bounded settling; measured results were preserved");
        }
    }

    void gain_noise_trial_all(const JointImpedanceGains& gains,const std::string& label="trajectory-noise") {
        using namespace serial_arm::servo_tuning;
        gain_require_trial(6.0);
        const auto n=cfg_.joint_names.size();std::vector<std::vector<Sample>> rows(n);
        std::vector<std::size_t> counts(n,0);std::vector<std::uint64_t> stamps(n,0);std::vector<double> last_fresh(n,0);
        std::vector<NoiseEstimate> measured(n);
        {
            std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_TRACKING) ||
               !robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING) ||
               !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
               !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::RIGID_TRACKING))
                throw std::runtime_error("cannot select simultaneous gain noise measurement");
            robot_.set_admittance_suspended(true);gain_probe_origin_=robot_.get_joint_state().pos;
        }
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="trajectory_noise";gain_probe_frequency_hz_={};gain_candidate_index_=0;gain_probe_diagnostic_json_="{}";}
        gain_probe_begin(label,"all",6.0,0,"noise_settling");
        const auto started=Robot::Clock::now();std::uint64_t cursor=0;
        const std::string record_path="gain-"+label+".csv";
        std::ofstream csv(std::filesystem::path(model_calibration_directory_)/record_path);
        if(!csv)throw std::runtime_error("cannot record simultaneous gain noise measurement");
        csv<<"time_s,joint,actual,velocity,new_measurement\n"<<std::setprecision(17);
        while(true) {
            gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
            if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                throw std::runtime_error("simultaneous gain noise measurement interrupted");
            const auto now=Robot::Clock::now();const double time=std::chrono::duration<double>(now-started).count();
            if(time>=1.6) {
                bool stable=true,complete=true;
                for(std::size_t axis=0;axis<n;++axis){measured[axis]=estimate_noise(rolling_noise_window(rows[axis]));stable=stable && measured[axis].stationary;complete=complete && measured[axis].samples>0;}
                if(complete && (stable || time>=6.0))break;
            }
            if(time>=6.0)throw std::runtime_error("simultaneous gain noise measurement has insufficient fresh feedback");
            if(!runtime_.submit_reference(robot_,JointPosVelCmd{gain_probe_origin_,JointVector(n,0)},now,MotionSource::CALIBRATION))
                throw std::runtime_error("simultaneous gain noise hold reference rejected");
            gain_probe_update(time,gain_probe_origin_,time<.4?"noise_settling":"noise_measurement");
            if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}) || !last_output_ || robot_.get_state()!=RobotState::ACTIVE)
                throw std::runtime_error("simultaneous gain noise measurement lost feedback");
            cursor=cycle_counter_;const auto& out=*last_output_;
            if(out.joint_state.pos.size()!=n || out.joint_state.vel.size()!=n || out.actuator_state.feedback_received_ns.size()!=n)
                throw std::runtime_error("simultaneous gain noise feedback dimensions changed");
            for(std::size_t axis=0;axis<n;++axis) {
                Sample row;row.time_s=std::chrono::duration<double>(Robot::Clock::now()-started).count();
                row.reference=gain_probe_origin_[axis];row.actual=out.joint_state.pos[axis];row.velocity=out.joint_state.vel[axis];
                if(!std::isfinite(row.actual) || !std::isfinite(row.velocity))throw std::runtime_error("simultaneous gain noise has non-finite feedback");
                const auto stamp=out.actuator_state.feedback_received_ns[axis];row.new_measurement=stamp>stamps[axis] && stamp>0;
                if(row.new_measurement){if(counts[axis] && row.time_s-last_fresh[axis]>.06)throw std::runtime_error("simultaneous gain noise lost per-joint continuous feedback");last_fresh[axis]=row.time_s;}
                if(row.time_s-last_fresh[axis]>.06)throw std::runtime_error("simultaneous gain noise lost per-joint continuous feedback");
                stamps[axis]=std::max(stamps[axis],stamp);counts[axis]+=row.new_measurement;rows[axis].push_back(row);

            }
        }
        {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains);}
        for(std::size_t axis=0;axis<n;++axis)for(const auto& row:rows[axis])csv<<row.time_s<<','<<cfg_.joint_names[axis]<<','<<row.actual<<','<<row.velocity<<','<<row.new_measurement<<'\n';
        csv.flush();if(!csv)throw std::runtime_error("simultaneous gain noise recording failed");
        gain_noise_.resize(n);bool stable=true;
        for(std::size_t axis=0;axis<n;++axis) {
            // A quiet window can measure zero jitter without observing even
            // one encoder code transition. Keep earlier raw-position step
            // evidence; do not replace it with an assertion of exact position.
            const double previous_step=gain_noise_[axis].encoder_step_rad;
            gain_noise_[axis]=usable_noise_estimate(measured[axis]);
            if(previous_step>0)gain_noise_[axis].encoder_step_rad=gain_noise_[axis].encoder_step_rad>0?
                std::min(previous_step,gain_noise_[axis].encoder_step_rad):previous_step;
            if(!measured[axis].stationary){stable=false;gain_add_warning("noise_not_stationary:"+cfg_.joint_names[axis]);}
        }
        gain_probe_end(stable?"noise_measured":"noise_reference_warning",true);
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);std::ostringstream out;out<<std::setprecision(17);
            out<<"{\"completed\":true,\"available\":true,\"stage\":\"trajectory_noise\",\"joint\":\"all\",\"reason\":\""<<(stable?"noise_measured":"noise_reference_warning")<<"\",\"record_path\":\""<<record_path<<"\",\"axes\":[";
            for(std::size_t axis=0;axis<n;++axis){if(axis)out<<',';out<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"samples\":"<<measured[axis].samples<<",\"stationary\":"<<(measured[axis].stationary?"true":"false")<<",\"encoder_step_rad\":"<<measured[axis].encoder_step_rad<<",\"measured_span_rad\":"<<measured[axis].span_rad<<",\"measured_drift_rad\":"<<measured[axis].drift_rad<<",\"measured_rms_rad\":"<<measured[axis].rms_rad<<",\"noise_rms_rad\":"<<gain_noise_[axis].rms_rad<<",\"effective_encoder_step_rad\":"<<gain_noise_[axis].encoder_step_rad<<",\"encoder_step_observed\":"<<(gain_noise_[axis].encoder_step_rad>0?"true":"false")<<'}';}
            out<<"]}";gain_probe_diagnostic_json_=out.str();gain_experiment_records_.push_back(out.str());}
    }

    double gain_endpoint_transition_seconds(const JointVector& origin,const JointVector& target) const {
        // Cubic zero-speed endpoints: max |qdot|=1.5*distance/T,
        // max |qddot|=6*distance/T^2. This is a reference cost, not an
        // assumption about the measured response or final accuracy.
        double duration=.4;
        for(std::size_t i=0;i<origin.size();++i) {
            const double distance=std::abs(target[i]-origin[i]);
            const double velocity=std::min(.05,.15*cfg_.safety.limits.max_vel[i]);
            const double acceleration=std::min(.2,.15*cfg_.safety.limits.max_acc[i]);
            duration=std::max({duration,1.5*distance/velocity,std::sqrt(6*distance/acceleration)});
        }
        return duration+.15;
    }

    void gain_transition_to_endpoint(const JointImpedanceGains& gains,const JointVector& origin,const JointVector& target,
        const std::string& label,double following_trial_cost,std::vector<std::vector<serial_arm::servo_tuning::Sample>>& evidence,
        bool reference_continuation=false,const char* continuation_source="accepted_reference_after_completed_demonstration_endpoint",
        JointGainProfile profile=JointGainProfile::SERVO_TRACKING) {
        const auto n=cfg_.joint_names.size();JointVector reference_origin=origin,actual=origin,initial_actual=origin,position=origin;
        const std::string record_path="gain-transition-"+label+".csv";
        auto started=Robot::Clock::now();double duration=0;bool capturing=false,arrived=false;std::size_t limited_cycles=0;
        const auto diagnostic=[&](bool complete,const std::string& reason) {
            std::ostringstream out;out<<std::setprecision(17)<<"{\"kind\":\"feedback_bounded_endpoint_transition\",\"reference_schedule\":\"control_cycle_time_planned_cubic\",\"stage\":\"trajectory_endpoint_transition\",\"joint\":\"all\",\"record_path\":\""<<record_path
                <<"\",\"completed\":"<<(complete?"true":"false")<<",\"available\":"<<(complete?"true":"false")<<",\"reason\":\""<<machine_json_escape(reason)
                <<"\",\"validation_scope\":\"reference_preparation_only_not_tracking_or_purpose_qualification\",\"counted_in_budget\":true,\"elapsed_s\":"<<std::chrono::duration<double>(Robot::Clock::now()-started).count()
                <<",\"planned_duration_s\":"<<duration<<",\"reference_arrived\":"<<(arrived?"true":"false")<<",\"limited_cycles\":"<<limited_cycles
                <<",\"initial_reference_source\":\""<<(reference_continuation?continuation_source:"fresh_measured_feedback")
                <<"\",\"reference_continuation\":"<<(reference_continuation?"true":"false")<<",\"initial_reference_rad\":";machine_json_joint_vector(out,reference_origin);
            out<<",\"initial_actual_rad\":";machine_json_joint_vector(out,initial_actual);
            out<<",\"target_rad\":";machine_json_joint_vector(out,target);out<<",\"reference_rad\":";machine_json_joint_vector(out,position);
            out<<",\"actual_rad\":";machine_json_joint_vector(out,actual);out<<",\"kp\":";machine_json_joint_vector(out,gains.kp);out<<",\"kd\":";machine_json_joint_vector(out,gains.kd);out<<'}';return out.str();
        };
        const auto publish=[&](bool complete,const std::string& reason) {
            const auto value=diagnostic(complete,reason);std::lock_guard<std::mutex> status(model_calibration_mutex_);
            gain_probe_stage_="trajectory_endpoint_transition";gain_probe_joint_="all";gain_probe_diagnostic_json_=value;
        };
        try {
            {std::lock_guard<std::mutex> lock(mutex_);
                if(!last_output_ || last_output_->joint_state.pos.size()!=n)throw std::runtime_error("gain_endpoint_transition_feedback_unavailable");
                initial_actual=actual=last_output_->joint_state.pos;
                const auto& types=dynamics_.get_info().joint_types;
                if(reference_continuation) {
                    JointVector ages(n,std::numeric_limits<double>::infinity());
                    const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                    if(last_output_->actuator_state.feedback_received_ns.size()==n)for(std::size_t i=0;i<n;++i) {
                        const auto stamp=last_output_->actuator_state.feedback_received_ns[i];
                        if(stamp && stamp<=now_ns)ages[i]=static_cast<double>(now_ns-stamp)/1e6;
                    }
                    const auto continuation=serial_arm::calibration_recovery::automatic_endpoint_continuation(actual,
                        last_output_->joint_cmd.pos,gain_completed_endpoint_reference_,target,ages,robot_.get_state()==RobotState::ACTIVE);
                    if(!continuation.valid)throw GainEndpointNotReady("completed endpoint reference continuity or fresh feedback was lost");
                    reference_origin=continuation.reference_origin;
                } else {
                    reference_origin=actual;
                    for(std::size_t i=0;i<n;++i)if(std::abs(reference_origin[i]-target[i])>serial_arm::calibration_recovery::position_tolerance(i<types.size()?types[i]:"revolute"))
                        throw GainEndpointNotReady("endpoint preparation requires the nearby demonstrated endpoint; return along the demonstrated path from an interior pose");
                }
                position=reference_origin;
                duration=gain_endpoint_transition_seconds(reference_origin,target);gain_require_trial(duration+following_trial_cost+.2);
                // Measurements finish in HOLD (or DRAG). Restore the actual
                // streaming controller before submitting the first bridge
                // reference, including a bridge whose endpoints differ only
                // because of measured tracking error. Arrival is not a gate.
                clear_command_sources();
                const auto mode=profile==JointGainProfile::COMPLIANT?JointImpedanceMode::COMPLIANT_TRACKING:JointImpedanceMode::RIGID_TRACKING;
                if(!runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
                   !robot_.set_calibration_tracking_gains(gains,mode))
                    throw std::runtime_error("cannot_restore_tracking_controller_before_reference_transition");
                robot_.set_admittance_suspended(true);robot_.clear_calibration_disturbance();
                started=Robot::Clock::now();gain_capture_begin(label+"-endpoint-transition",started,true);capturing=true;
                gain_cycle_reference_error_=nullptr;
                gain_cycle_reference_until_=started+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(duration+.5));
                gain_cycle_reference_=[&,this](Robot::TimePoint now) {
                    const double motion_duration=duration-.15;
                    const double u=std::clamp(std::chrono::duration<double>(now-started).count()/motion_duration,0.0,1.0);
                    const double rate=6*u*(1-u)/motion_duration;
                    JointVector velocity(n,0);
                    position=serial_arm::calibration_recovery::endpoint_bridge_reference(reference_origin,target,u);
                    for(std::size_t i=0;i<n;++i)velocity[i]=rate*(target[i]-reference_origin[i]);
                    arrived=u>=1;
                    const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{position,velocity},now,MotionSource::CALIBRATION);
                    if(!sent)throw machine_robot_error("gain endpoint transition reference",sent.error());
                };
            }
            gain_probe_begin(label+"-endpoint-transition","all",duration,0,"bounded_endpoint_transition");
            double reported=-1;
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);const auto now=Robot::Clock::now();
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("gain_endpoint_transition_interrupted");
                if(!last_output_ || last_output_->joint_state.pos.size()!=n || last_output_->actuator_state.feedback_received_ns.size()!=n)
                    throw std::runtime_error("gain_endpoint_transition_feedback_unavailable");
                actual=last_output_->joint_state.pos;
                const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
                for(std::size_t i=0;i<n;++i){const auto stamp=last_output_->actuator_state.feedback_received_ns[i];
                    if(!stamp || stamp>now_ns || now_ns-stamp>60000000 || !std::isfinite(actual[i]))throw std::runtime_error("gain_endpoint_transition_feedback_stale");}
                limited_cycles+=std::any_of(last_output_->reference_limited.begin(),last_output_->reference_limited.end(),[](auto x){return x!=0;}) ||
                    std::any_of(last_output_->torque_limited.begin(),last_output_->torque_limited.end(),[](auto x){return x!=0;});
                const double elapsed=std::chrono::duration<double>(now-started).count();
                gain_probe_update(elapsed,position,arrived?"endpoint_reference_dwell":"bounded_endpoint_transition");
                if(elapsed-reported>=.1){reported=elapsed;publish(false,arrived?"endpoint_reference_dwell":"bounded_endpoint_transition");}
                if(elapsed>=duration && arrived){gain_cycle_reference_={};gain_capture_active_=false;evidence=std::move(gain_capture_rows_);break;}
                const auto cursor=cycle_counter_;
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || model_calibration_cancel_.load() || robot_.get_state()!=RobotState::ACTIVE;}))
                    throw std::runtime_error("gain_endpoint_transition_feedback_unavailable");
            }
            gain_experiment_records_.push_back(diagnostic(true,"endpoint_reference_ready"));publish(true,"endpoint_reference_ready");
        } catch(...) {
            const auto failure=std::current_exception();std::string reason="gain_endpoint_transition_interrupted";
            try{std::rethrow_exception(failure);}catch(const std::exception& error){reason=error.what();}catch(...){}
            if(capturing){std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_={};gain_capture_active_=false;evidence=std::move(gain_capture_rows_);}
            gain_experiment_records_.push_back(diagnostic(false,reason));publish(false,reason);std::rethrow_exception(failure);
        }
    }

    // Admit the whole proposed controller before installing it. The currently
    // accepted reference can still differ from actual position after a complete
    // taught traversal; that is not an arrival failure, but it matters to the
    // torque demanded by a gain change. Never substitute a desired final error.
    bool gain_admit_candidate(const JointImpedanceGains& incumbent,const JointImpedanceGains& proposal,
        const JointVector& measured_error,const JointVector& measured_velocity_error,const std::string& label) {
        if(incumbent.kp==proposal.kp && incumbent.kd==proposal.kd)return true;
        const auto n=cfg_.joint_names.size();const double unknown=std::numeric_limits<double>::quiet_NaN();
        JointVector current_error(n,unknown),current_velocity_error(n,unknown),actual(n,unknown),accepted(n,unknown);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(last_output_ && robot_.get_state()==RobotState::ACTIVE && last_output_->joint_cmd.pos.size()==n &&
               last_output_->joint_cmd.vel.size()==n && last_output_->joint_state.pos.size()==n &&
               last_output_->joint_state.vel.size()==n && last_output_->actuator_state.feedback_received_ns.size()==n) {
                const auto& out=*last_output_;
                const auto now=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                for(std::size_t i=0;i<n;++i) {
                    const auto stamp=out.actuator_state.feedback_received_ns[i];
                    if(!stamp || stamp>now || now-stamp>60000000)continue;
                    actual[i]=out.joint_state.pos[i];accepted[i]=out.joint_cmd.pos[i];
                    current_error[i]=accepted[i]-actual[i];current_velocity_error[i]=out.joint_cmd.vel[i]-out.joint_state.vel[i];
                }
            }
        }
        const auto admission=serial_arm::servo_tuning::admit_complete_gain_candidate(
            {incumbent.kp,incumbent.kd},{proposal.kp,proposal.kd},gain_model_limits_,
            measured_error,measured_velocity_error,current_error,current_velocity_error);
        std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"whole_group_candidate_admission\",\"phase\":\""<<machine_json_escape(label)
            <<"\",\"allowed\":"<<(admission.allowed?"true":"false")<<",\"accuracy_is_continuation_gate\":false"
            <<",\"rejected_group_clipped_or_applied\":false,\"source\":\"measured_error_envelope_and_fresh_accepted_reference_minus_actual_feedback\"";
        const auto field=[&](const char* name,const JointVector& value) {
            note<<",\""<<name<<"\":[";for(std::size_t i=0;i<value.size();++i){if(i)note<<',';note<<gain_finite_json(value[i]);}note<<']';
        };
        field("incumbent_kp",incumbent.kp);field("incumbent_kd",incumbent.kd);field("proposed_kp",proposal.kp);field("proposed_kd",proposal.kd);
        field("fresh_actual_position_rad",actual);field("fresh_accepted_position_rad",accepted);
        field("error_envelope_rad",admission.error_envelope_rad);field("velocity_error_envelope_rad_s",admission.velocity_error_envelope_rad_s);
        field("incumbent_PD_envelope_nm",admission.incumbent_PD_envelope_nm);field("proposal_PD_envelope_nm",admission.proposal_PD_envelope_nm);
        field("increase_PD_envelope_nm",admission.increase_PD_envelope_nm);field("feedback_effort_budget_nm",admission.feedback_effort_budget_nm);
        note<<",\"axis_reasons\":";gain_json_warnings(note,admission.axis_reasons);note<<'}';gain_candidate_records_.push_back(note.str());
        if(!admission.allowed)gain_add_warning("candidate_not_admitted_retained_complete_measured_controller:"+label);
        return admission.allowed;
    }

    bool gain_admit_candidate(const JointImpedanceGains& incumbent,const JointImpedanceGains& proposal,
        const std::vector<std::vector<serial_arm::servo_tuning::Sample>>& measured,const std::string& label) {
        const auto n=cfg_.joint_names.size();JointVector error(n,std::numeric_limits<double>::quiet_NaN()),velocity=error;
        if(measured.size()==n)for(std::size_t i=0;i<n;++i) {
            std::size_t count=0;double E=1e-9,V=0;
            for(const auto& row:measured[i])if(row.new_measurement && row.actual_gains_known &&
                std::abs(row.actual_kp-incumbent.kp[i])<1e-6 && std::abs(row.actual_kd-incumbent.kd[i])<1e-6 &&
                std::isfinite(row.reference) && std::isfinite(row.actual) && std::isfinite(row.reference_velocity) && std::isfinite(row.velocity)) {
                ++count;E=std::max(E,std::abs(row.reference-row.actual));V=std::max(V,std::abs(row.reference_velocity-row.velocity));
            }
            if(count>=30){error[i]=E;velocity[i]=V;}
        }
        return gain_admit_candidate(incumbent,proposal,error,velocity,label);
    }

    // Indirect closed-loop frequency identification. The returned motor
    // position is measured against a known independent torque command under
    // four actual embedded-PD controllers. Never reconstruct an unreported
    // internal motor torque/velocity channel from delayed host feedback.
    void gain_identify_interior_pd(JointImpedanceGains& gains,
        const std::vector<serial_arm::servo_tuning::ModelAxisLimits>& limits,
        double reserved_s,const std::string& label,
        const std::vector<std::vector<serial_arm::servo_tuning::Sample>>& path_rows) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();
        std::vector<std::size_t> priority(n);std::vector<double> deficit(n,0);
        for(std::size_t axis=0;axis<n;++axis) {
            priority[axis]=axis;
            if(axis<path_rows.size())for(const auto& row:path_rows[axis])if(row.new_measurement && std::isfinite(row.actual) && std::isfinite(row.reference))
                deficit[axis]=std::max(deficit[axis],std::abs(row.reference-row.actual));
        }
        std::stable_sort(priority.begin(),priority.end(),[&](auto a,auto b){return deficit[a]>deficit[b];});
        for(std::size_t slot=0;slot<n;++slot) {
            const auto axis=priority[slot];
            const auto incumbent=gains;
            try {
                auto center=gain_fresh_short_center();auto bound=limits[axis];
                gain_apply_demonstrated_probe_range(bound,axis,center[axis]);
                const double floor=std::max({12*gain_noise_[axis].encoder_step_rad,8*gain_noise_[axis].rms_rad,.002});
                const double room=std::min(-bound.reference_min_offset_rad,bound.reference_max_offset_rad);
                if(room<3*floor) {gain_add_warning("frequency_identification_continue_path_insufficient_actual_room:"+cfg_.joint_names[axis]);continue;}
                std::array<JointImpedanceGains,4> controllers{incumbent,incumbent,incumbent,incumbent};
                // Separate the identification controllers enough to resolve
                // inverse-response differences above encoder/noise floors.
                // These are experiment settings, not iterative gain tuning.
                controllers[1].kp[axis]=.5*incumbent.kp[axis];
                // Resolve the inverse-response coefficients using reductions
                // of the already measured feedback channels. Identification
                // is not permission for an unmeasured threefold D increase.
                controllers[2].kd[axis]=.5*incumbent.kd[axis];
                controllers[1]=gain_protocol_gains(controllers[1]);controllers[2]=gain_protocol_gains(controllers[2]);
                controllers[3].kp[axis]=controllers[1].kp[axis];controllers[3].kd[axis]=controllers[2].kd[axis];
                if(controllers[1].kp[axis]<=0 || controllers[1].kp[axis]==incumbent.kp[axis] ||
                   controllers[2].kd[axis]==incumbent.kd[axis]) {gain_add_warning("frequency_identification_protocol_variations_unresolved:"+cfg_.joint_names[axis]);continue;}
                bool experiments_admitted=true;
                for(std::size_t k=1;k<controllers.size();++k)
                    experiments_admitted=gain_admit_candidate(incumbent,controllers[k],path_rows,
                        label+"-identification-"+cfg_.joint_names[axis]+"-controller-"+std::to_string(k)) && experiments_admitted;
                if(!experiments_admitted)continue;
                Requirements req=gain_requirements_;req.reference_bias_rad=0;req.motion_probe=false;req.stationary_probe=false;
                req.encoder_step_rad=gain_noise_[axis].encoder_step_rad;req.noise_rms_rad=gain_noise_[axis].rms_rad;
                req.amplitude_rad=std::min(.2*room,std::max(floor,.006));
                MitFrequencyPlanRequest frequency_request;
                {std::lock_guard<std::mutex> lock(mutex_);
                    const auto& mass=dynamics_.get_mass_matrix();
                    frequency_request.inertia_kg_m2=mass(static_cast<Eigen::Index>(axis),static_cast<Eigen::Index>(axis));}
                frequency_request.actual_kp=incumbent.kp[axis];frequency_request.actual_kd=incumbent.kd[axis];
                std::vector<double> feedback_periods;double previous_feedback=-1;
                for(const auto& row:path_rows[axis])if(row.new_measurement && std::isfinite(row.feedback_time_s)) {
                    if(previous_feedback>=0 && row.feedback_time_s>previous_feedback)
                        feedback_periods.push_back(row.feedback_time_s-previous_feedback);
                    previous_feedback=row.feedback_time_s;
                    frequency_request.feedback_age_s=std::max(frequency_request.feedback_age_s,row.command_time_s-row.feedback_time_s);
                }
                if(!feedback_periods.empty()) {
                    std::sort(feedback_periods.begin(),feedback_periods.end());
                    frequency_request.feedback_period_s=feedback_periods[feedback_periods.size()/2];
                }
                const double remaining=gain_deadline_?std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():0;
                frequency_request.measurement_budget_s=std::max(0.0,remaining-reserved_s-2)/static_cast<double>(n-slot);
                frequency_request.per_controller_overhead_s=.25+req.tail_s+.65;
                MitFrequencyCoDesignRequest experiment_request;experiment_request.frequency=frequency_request;
                auto& input_request=experiment_request.torque;input_request.inertia_kg_m2=frequency_request.inertia_kg_m2;
                for(std::size_t k=0;k<controllers.size();++k){input_request.actual_kp[k]=controllers[k].kp[axis];input_request.actual_kd[k]=controllers[k].kd[axis];}
                input_request.encoder_step_rad=req.encoder_step_rad;input_request.noise_rms_rad=req.noise_rms_rad;
                input_request.negative_room_rad=-bound.reference_min_offset_rad;input_request.positive_room_rad=bound.reference_max_offset_rad;
                input_request.effort_allowance_nm=.25*bound.effort_budget;
                // The source capability is actuator-side; position ratio and
                // direction do not scale a positive joint torque quantum.
                if(actuator_info_.size()==n && cfg_.mapper.tor_ratio.size()==n)
                    input_request.torque_quantum_nm=actuator_info_[axis].effort_resolution*cfg_.mapper.tor_ratio[axis];
                const auto experiment_plan=plan_mit_frequency_experiment_co_design(experiment_request);
                const auto& frequency_plan=experiment_plan.frequency;const auto& input_plan=experiment_plan.torque;
                std::ostringstream planned;planned<<std::setprecision(17)<<"{\"kind\":\"mit_frequency_experiment_plan\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
                    <<"\",\"source\":\""<<frequency_plan.source<<"\",\"eligible\":"<<(experiment_plan.eligible?"true":"false")
                    <<",\"reason\":\""<<experiment_plan.reason<<"\",\"urdf_local_inertia_kg_m2\":"<<frequency_request.inertia_kg_m2
                    <<",\"actual_kp\":"<<incumbent.kp[axis]<<",\"actual_kd\":"<<incumbent.kd[axis]
                    <<",\"measured_feedback_period_s\":"<<frequency_request.feedback_period_s<<",\"measured_feedback_age_s\":"<<frequency_request.feedback_age_s
                    <<",\"allocated_measurement_budget_s\":"<<frequency_request.measurement_budget_s<<",\"rigid_body_probe_prior_hz\":"<<frequency_plan.rigid_body_crossover_hz
                    <<",\"selected_center_hz\":"<<frequency_plan.selected_center_hz<<",\"prior_crossover_covered\":"<<(frequency_plan.prior_crossover_covered?"true":"false")
                    <<",\"probe_duration_s\":"<<frequency_plan.probe_duration_s<<",\"complete_controller_set_cost_s\":"<<frequency_plan.total_cost_s
                    <<",\"numeric_bands_examined\":"<<experiment_plan.numeric_bands_examined<<",\"initial_torque_amplitude_nm\":"<<input_plan.initial_amplitude_nm
                    <<",\"known_joint_torque_command_quantum_nm\":"<<input_request.torque_quantum_nm
                    <<",\"input_value_scope\":\"independent_joint_torque_command_before_protocol_encoding_not_delivered_motor_torque\""
                    <<",\"maximum_torque_amplitude_nm\":"<<input_plan.maximum_amplitude_nm<<",\"position_signal_target_rad\":"<<input_plan.position_target_rad
                    <<",\"predicted_min_tone_position_rad\":"<<input_plan.predicted_min_tone_position_amplitude_rad<<",\"input_design_source\":\""<<input_plan.source<<"\",\"missing_evidence\":";
                gain_json_warnings(planned,experiment_plan.missing_evidence);planned<<'}';gain_experiment_records_.push_back(planned.str());
                if(!experiment_plan.eligible) {gain_add_warning("frequency_plan_incomplete_continue_path:"+cfg_.joint_names[axis]+":"+experiment_plan.reason);continue;}
                const double target=frequency_plan.selected_center_hz;
                req.frequencies_hz=frequency_plan.frequencies_hz;
                req.probe_duration_s=frequency_plan.probe_duration_s;
                const double one_cost=req.probe_duration_s+.25+req.tail_s+.65;
                // Admit an entire identifiable experiment, not three quarters
                // of one which consumes the reserved full-path validation.
                gain_require_trial(4*one_cost+reserved_s+2);
                std::vector<Requirements> plans(n,req);JointVector torque(n,0);
                torque[axis]=input_plan.initial_amplitude_nm;
                std::vector<MitClosedLoopTrialEstimate> measured;
                const std::string condition=label+"-axis-"+std::to_string(axis);
                for(std::size_t k=0;k<controllers.size();++k) {
                    if(!gain_admit_candidate(incumbent,controllers[k],path_rows,condition+"-before-controller-"+std::to_string(k)))break;
                    auto trial=gain_short_trial(controllers[k],true,condition+"-controller-"+std::to_string(k),
                        &plans,false,false,&center,"fresh_measured_interior_of_demonstrated_path",&torque,reserved_s,false,axis,true);
                    MitAutotuneOperatingPoint operating;operating.actual_gains={controllers[k].kp,controllers[k].kd};
                    operating.fixed_reference_rad=center;operating.excited_axis=axis;operating.condition_id=condition;
                    operating.isolated_axis_excitation=true;operating.maximum_operating_excursion_rad=std::min(room,bound.position_margin_rad);
                    auto estimated=estimate_mit_closed_loop_trial(trial.rows[axis],req,operating,.25);
                    if(k==0 && !estimated.valid &&
                       (estimated.reason=="mit_independent_position_tone_below_noise" || estimated.reason=="mit_independent_tone_fit_quality_insufficient")) {
                        const double stronger=std::min(2*torque[axis],input_plan.maximum_amplitude_nm);
                        // A legal final 3-19% increase was previously never
                        // attempted because of a fixed 20% threshold. Resolve
                        // by actual protocol steps (or a numerical tolerance
                        // for unknown/continuous inputs), never by that ratio.
                        const double input_step=std::max(input_request.torque_quantum_nm,1e-9);
                        if(stronger>=torque[axis]+input_step && gain_trial_fits(4*one_cost+reserved_s+2)) {
                            torque[axis]=stronger;
                            trial=gain_short_trial(controllers[k],true,condition+"-controller-0-input-supplement",&plans,false,false,&center,
                                "same_interior_center_known_input_amplitude_supplement",&torque,reserved_s,false,axis,true);
                            estimated=estimate_mit_closed_loop_trial(trial.rows[axis],req,operating,.25);
                        }
                    }
                    for(std::size_t held=0;held<n;++held)if(held!=axis)
                        for(const auto& row:trial.rows[held])if(row.time_s>=.95 && row.time_s<req.probe_duration_s-.45 &&
                           (!row.actual_gains_known || std::abs(row.actual_kp-controllers[k].kp[held])>1e-6 || std::abs(row.actual_kd-controllers[k].kd[held])>1e-6)) {
                            estimated.valid=false;estimated.reason="mit_held_other_actual_gains_not_constant";
                            estimated.missing_evidence.push_back("recorded_other_axes_actual_PD_settled_for_the_entire_frequency_window");break;
                        }
                    measured.push_back(std::move(estimated));
                    const auto& response=measured.back();
                    std::ostringstream out;out<<std::setprecision(17)<<"{\"kind\":\"mit_closed_loop_controller_experiment\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
                        <<"\",\"controller_index\":"<<k<<",\"record_path\":\""<<trial.record_path<<"\",\"valid\":"<<(response.valid?"true":"false")
                        <<",\"reason\":\""<<machine_json_escape(response.reason)<<"\",\"input_definition\":\""<<response.input_definition
                        <<"\",\"time_definition\":\""<<response.time_definition<<"\",\"fresh_samples\":"<<response.fresh_samples<<",\"center_rad\":";
                    machine_json_joint_vector(out,center);out<<",\"kp\":";machine_json_joint_vector(out,controllers[k].kp);out<<",\"kd\":";machine_json_joint_vector(out,controllers[k].kd);
                    out<<",\"missing_evidence\":";gain_json_warnings(out,response.missing_evidence);out<<",\"tones\":[";
                    for(std::size_t j=0;j<response.points.size();++j){if(j)out<<',';const auto& p=response.points[j];
                        out<<"{\"hz\":"<<p.hz<<",\"H_real\":"<<gain_finite_json(p.closed_loop_response.real())<<",\"H_imag\":"<<gain_finite_json(p.closed_loop_response.imag())
                            <<",\"input_amplitude_nm\":"<<std::abs(p.input_tone_nm)<<",\"output_amplitude_rad\":"<<std::abs(p.output_tone_rad)
                            <<",\"input_tone_fit_r_squared\":"<<p.input_tone_fit_r_squared<<",\"output_tone_fit_r_squared\":"<<p.output_tone_fit_r_squared<<",\"half_relative_error\":"<<p.half_relative_error<<'}';}
                    out<<"]}";gain_experiment_records_.push_back(out.str());
                    {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();gain_probe_frequency_hz_=req.frequencies_hz;}
                    // Do not repeat the whole sequence at a stuck posture.
                    if(!response.valid)break;
                }
                const auto identified=combine_mit_closed_loop_trials(measured);
                PdLoopShapingRequest request;request.target_crossover_hz=target;
                request.error_bound_rad=std::min(room,.5*gain_requirements_.local_range_rad);
                request.velocity_error_bound_rad=std::min(bound.max_velocity,2*pi*target*request.error_bound_rad);
                SampledPdGainProposal proposal;
                if(identified.valid && identified.gain_holdout_verified)
                    proposal=shape_mit_pd(incumbent.kp[axis],incumbent.kd[axis],identified.response,bound,request);
                auto next=incumbent;
                if(proposal.valid){next.kp[axis]=proposal.kp;next.kd[axis]=proposal.kd;next=gain_protocol_gains(next);}
                std::ostringstream out;out<<std::setprecision(17)<<"{\"kind\":\"mit_indirect_frequency_autotune\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])
                    <<"\",\"identified\":"<<(identified.valid?"true":"false")<<",\"independent_gain_holdout_verified\":"<<(identified.gain_holdout_verified?"true":"false")
                    <<",\"candidate_generated\":"<<(proposal.valid?"true":"false")<<",\"requires_complete_frozen_path\":true,\"validated\":false,\"reason\":\""
                    <<machine_json_escape(identified.valid?proposal.reason:identified.reason)<<"\",\"model_definition\":\""<<identified.model_definition
                    <<"\",\"plant_definition\":\""<<identified.plant_definition<<"\",\"velocity_definition\":\""<<identified.velocity_definition<<"\",\"frequency_response\":";
                gain_empirical_frequency_json(out,identified.response);out<<",\"measured_loop\":";gain_sampled_loop_json(out,proposal.measured_loop);
                out<<",\"candidate_loop\":";gain_sampled_loop_json(out,proposal.candidate_loop);out<<",\"kp\":";machine_json_joint_vector(out,next.kp);
                out<<",\"kd\":";machine_json_joint_vector(out,next.kd);out<<",\"identification_points\":[";
                for(std::size_t j=0;j<identified.points.size();++j){if(j)out<<',';const auto& p=identified.points[j];
                    out<<"{\"hz\":"<<p.hz<<",\"design_condition_number\":"<<gain_finite_json(p.design_condition_number)
                        <<",\"half_plant_relative_error\":"<<gain_finite_json(p.half_plant_relative_error)<<",\"half_velocity_relative_error\":"<<gain_finite_json(p.half_velocity_relative_error)
                        <<",\"gain_holdout_relative_error\":"<<gain_finite_json(p.gain_holdout_relative_error)<<",\"a\":["<<gain_finite_json(p.a.real())<<','<<gain_finite_json(p.a.imag())
                        <<"],\"b\":["<<gain_finite_json(p.b.real())<<','<<gain_finite_json(p.b.imag())<<"],\"c\":["<<gain_finite_json(p.c.real())<<','<<gain_finite_json(p.c.imag())<<"]}";}
                out<<"],\"missing_evidence\":";gain_json_warnings(out,identified.valid?proposal.missing_evidence:identified.missing_evidence);out<<'}';gain_candidate_records_.push_back(out.str());
                {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();}
                const bool admitted=!proposal.valid || gain_admit_candidate(incumbent,next,path_rows,
                    label+"-identified-"+cfg_.joint_names[axis]);
                gains=admitted?next:incumbent;
                {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains,true);}
                if(!proposal.valid)gain_add_warning("frequency_identification_incomplete_keep_measured_controller:"+cfg_.joint_names[axis]+":"+identified.reason);
                // Freeze and verify this complete controller on the path
                // before deriving another controller around it.
                if(admitted && proposal.valid && (gains.kp!=incumbent.kp || gains.kd!=incumbent.kd))break;
            } catch(const std::exception& error) {
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || model_calibration_fault_interrupted_.load() || machine_robot_state()!=RobotState::ACTIVE)throw;
                const bool out_of_time=gain_budget_exhausted_.exchange(false);
                gains=incumbent;{std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains,true);}
                gain_add_warning("frequency_identification_warning:"+cfg_.joint_names[axis]+":"+error.what());
                if(out_of_time)break;
            }
        }
    }

    // Discovery follows the already checked demonstration continuously. Its
    // changing gains are evidence for candidates, never final tracking
    // acceptance; the next traversal freezes one complete controller.
    void gain_taught_path_discovery(JointImpedanceGains& gains,JointGainProfile profile,
        const std::vector<serial_arm::servo_tuning::ModelAxisLimits>& limits,double reserved_s,const std::string& label,
        const std::vector<std::vector<serial_arm::servo_tuning::Sample>>& baseline_rows) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();const auto& path=*model_calibration_trajectory_;
        auto ordered=path.positions;if(gain_next_reverse_)std::reverse(ordered.begin(),ordered.end());
        const auto endpoint=gain_endpoint_observation(true,ordered.front());
        const double bridge_s=gain_endpoint_transition_seconds(endpoint.reference_origin,ordered.front());
        const auto bridge_count=std::max<std::size_t>(2,static_cast<std::size_t>(std::ceil(bridge_s/path.sample_dt)));
        std::vector<JointVector> timeline;timeline.reserve(bridge_count+ordered.size());
        for(std::size_t k=0;k<bridge_count;++k) {
            const double u=static_cast<double>(k)/bridge_count;
            auto q=serial_arm::calibration_recovery::endpoint_bridge_reference(endpoint.reference_origin,ordered.front(),u);
            timeline.push_back(std::move(q));
        }
        timeline.insert(timeline.end(),ordered.begin(),ordered.end());
        const double source_duration=path.sample_dt*(timeline.size()-1);
        gain_require_trial(source_duration+reserved_s+2);
        TaughtMeasurementRoomRequest room_request;
        for(std::size_t i=0;i<n;++i){room_request.required_room_rad.push_back(std::max(.006,12*gain_noise_[i].encoder_step_rad));
            room_request.observed_position_step_rad.push_back(gain_noise_[i].encoder_step_rad);room_request.stationary_noise_rms_rad.push_back(gain_noise_[i].rms_rad);}
        for(const auto& target:model_calibration_pose_targets_) {
            const auto index=gain_next_reverse_?ordered.size()-1-target.trajectory_index:target.trajectory_index;
            if(index<ordered.size())room_request.preferred_source_indices.push_back(bridge_count+index);
        }
        const auto schedule=plan_taught_measurement_schedule(timeline,room_request);
        const double remaining=gain_deadline_?std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():1000;
        const double available_dwell=std::max(0.0,remaining-source_duration-reserved_s-4);
        auto warp=schedule.has_anchor && available_dwell>=32?
            make_taught_measurement_time_warp(source_duration,schedule.common_anchor.source_index*path.sample_dt,available_dwell,1):TaughtMeasurementTimeWarp{};
        bool measured=!warp.valid,completed=false;double source_time=0,inserted_s=0;
        JointVector reference=endpoint.reference_origin,reference_velocity(n,0),resume_origin;double resume_wall=-1;
        const auto mode=profile==JointGainProfile::COMPLIANT?JointImpedanceMode::COMPLIANT_TRACKING:JointImpedanceMode::RIGID_TRACKING;
        std::vector<std::vector<Sample>> rows(n);const auto started=Robot::Clock::now();
        const std::string record="gain-trajectory-"+label+"-discovery.csv";
        const auto install_reference=[&] {
            gain_cycle_reference_error_=nullptr;
            gain_cycle_reference_until_=gain_deadline_.value_or(started+std::chrono::seconds(1000));
            gain_cycle_reference_=[&,this](Robot::TimePoint now) {
                const double elapsed=std::max(0.0,std::chrono::duration<double>(now-started).count());
                const auto clock=warp.valid?sample_taught_measurement_time_warp(warp,elapsed):TaughtMeasurementClock{};
                source_time=warp.valid?clock.source_time_s:std::min(elapsed,source_duration);
                completed=warp.valid?clock.complete:source_time>=source_duration;
                const double rate=completed?0:warp.valid?clock.rate:1;
                const auto next=serial_arm::calibration_retime::interpolate(timeline,source_time/path.sample_dt,path.sample_dt,rate);
                reference=next.position;reference_velocity=completed?JointVector(n,0):next.velocity;
                // Resume from the measured equilibrium with a convex smooth
                // blend. No exact arrival gate and no reference step back to
                // the planned anchor after the fixed-reference experiments.
                if(resume_wall>=0 && elapsed<resume_wall+2) {
                    const double u=std::clamp((elapsed-resume_wall)/2,0.0,1.0),w=u*u*(3-2*u),dw=3*u*(1-u);
                    for(std::size_t i=0;i<n;++i){reference_velocity[i]=w*reference_velocity[i]+dw*(reference[i]-resume_origin[i]);reference[i]=(1-w)*resume_origin[i]+w*reference[i];}
                }
                const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{reference,reference_velocity},now,MotionSource::CALIBRATION);
                if(!sent)throw machine_robot_error("taught path frequency identification reference",sent.error());
            };
        };
        const auto finish=[&](bool complete,const std::string& reason) {
            {std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_={};gain_capture_active_=false;
                if(gain_capture_rows_.size()==n)for(std::size_t i=0;i<n;++i){const auto& tail=gain_capture_rows_[i];rows[i].insert(rows[i].end(),tail.begin(),tail.end());}
                gain_capture_rows_.clear();
                if(complete)gain_remember_completed_endpoint_locked(ordered.back(),record);
                else {gain_completed_endpoint_reference_.clear();gain_completed_endpoint_record_.clear();}
                if(robot_.get_state()==RobotState::ACTIVE)gain_hold_for_assessment_locked(gains,!complete);}
            gain_write_response_csv(record,rows);
            std::ostringstream out;out<<std::setprecision(17)<<"{\"kind\":\"taught_path_discovery\",\"stage\":\"trajectory_discovery\",\"completed\":"<<(complete?"true":"false")
                <<",\"validated\":false,\"method\":\"indirect_closed_loop_multicontroller_frequency_response_and_sampled_PD_loop_shaping\",\"reference_schedule\":\"control_cycle_monotonic_with_budgeted_interior_measurement_dwell\",\"playback_rate\":1,\"validation_scope\":\"variable_gain_response_discovery_requires_frozen_complete_path\",\"record_path\":\""<<record
                <<"\",\"source_time_s\":"<<source_time<<",\"source_duration_s\":"<<source_duration<<",\"elapsed_s\":"<<std::chrono::duration<double>(Robot::Clock::now()-started).count()
                <<",\"measurement_dwell_s\":"<<inserted_s<<",\"endpoint_identification_required\":false,\"exact_anchor_arrival_required\":false"
                <<",\"endpoint_reference_continuation\":"<<(endpoint.reference_continuation?"true":"false")
                <<",\"continuation_record_path\":\""<<machine_json_escape(endpoint.continuation_record)<<"\",\"initial_reference_rad\":";
            machine_json_joint_vector(out,endpoint.reference_origin);out<<",\"initial_actual_rad\":";machine_json_joint_vector(out,endpoint.position);
            out<<",\"reason\":\""<<machine_json_escape(reason)<<"\",\"kp\":";
            machine_json_joint_vector(out,gains.kp);out<<",\"kd\":";machine_json_joint_vector(out,gains.kd);out<<'}';gain_experiment_records_.push_back(out.str());
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();}
        };
        try {
            {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) || !robot_.set_calibration_tracking_gains(gains,mode))
                    throw std::runtime_error("taught_path_discovery_control_mode_rejected");
                robot_.set_admittance_suspended(true);robot_.clear_calibration_disturbance();
                gain_capture_begin(label+"-discovery",started,true);install_reference();}
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="trajectory_discovery";gain_probe_frequency_hz_={};}
            gain_probe_begin(label+"-discovery","all",source_duration,0,"taught_path_original_speed_with_interior_identification");
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);const auto now=Robot::Clock::now();
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("taught_path_discovery_interrupted");
                const double elapsed=std::chrono::duration<double>(now-started).count();
                gain_probe_update(source_time,reference,"taught_path_original_speed_with_interior_identification");
                if(completed)break;
                if(!measured && sample_taught_measurement_time_warp(warp,elapsed).at_anchor) {
                    measured=true;gain_cycle_reference_={};gain_capture_active_=false;
                    for(std::size_t i=0;i<n;++i){auto& part=gain_capture_rows_[i];rows[i].insert(rows[i].end(),part.begin(),part.end());}gain_capture_rows_.clear();
                    gain_hold_for_assessment_locked(gains,true);lock.unlock();
                    const double remaining_path=source_duration-warp.anchor_source_s+2;
                    gain_identify_interior_pd(gains,limits,reserved_s+remaining_path,label,baseline_rows);
                    lock.lock();const double elapsed_after=std::chrono::duration<double>(Robot::Clock::now()-started).count();
                    inserted_s=std::max(0.0,elapsed_after-warp.dwell_wall_start_s)+.05;
                    warp=make_taught_measurement_time_warp(source_duration,warp.anchor_source_s,inserted_s,1);
                    resume_origin=last_output_->joint_state.pos;resume_wall=warp.resume_wall_start_s;
                    if(!runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile) || !robot_.set_calibration_tracking_gains(gains,mode))
                        throw std::runtime_error("cannot_resume_demonstrated_path_after_frequency_identification");
                    robot_.set_admittance_suspended(true);gain_capture_begin(label+"-discovery-resume",started,true);install_reference();
                    {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="trajectory_discovery";}
                }
                const auto cursor=cycle_counter_;if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))
                    throw std::runtime_error("taught_path_discovery_feedback_unavailable");
            }
            finish(true,"taught_path_discovery_complete");gain_next_reverse_=!gain_next_reverse_;
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);++gain_path_pass_current_;}
        } catch(...) {const auto failure=std::current_exception();finish(false,"taught_path_discovery_interrupted_or_partial");std::rethrow_exception(failure);}
    }

    bool gain_passing_purpose_measurement(JointImpedanceGains& gains,JointGainProfile profile,
        double reserved_s,double measurement_cost_s,const std::string& label,
        const std::function<void(std::size_t,double)>& measure,const ServoTrial* trajectory_evidence=nullptr) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();const auto& path=*model_calibration_trajectory_;
        auto ordered=path.positions;if(gain_next_reverse_)std::reverse(ordered.begin(),ordered.end());
        const auto endpoint=gain_endpoint_observation(true,ordered.front());
        const auto& timeline=ordered;
        const double source_duration=path.sample_dt*(timeline.size()-1);
        TaughtMeasurementRoomRequest room_request;
        for(std::size_t i=0;i<n;++i){room_request.required_room_rad.push_back(std::max({1e-9,6*gain_noise_[i].rms_rad,3*gain_noise_[i].encoder_step_rad}));
            room_request.observed_position_step_rad.push_back(gain_noise_[i].encoder_step_rad);room_request.stationary_noise_rms_rad.push_back(gain_noise_[i].rms_rad);}
        for(const auto& target:model_calibration_pose_targets_) {
            const auto index=gain_next_reverse_?ordered.size()-1-target.trajectory_index:target.trajectory_index;
            if(index<ordered.size())room_request.preferred_source_indices.push_back(index);
        }
        JointVector response_weights(n,0);
        if(trajectory_evidence && trajectory_evidence->rows.size()==n)for(std::size_t i=0;i<n;++i) {
            double square=0;std::size_t count=0;
            for(const auto& row:trajectory_evidence->rows[i])if(row.new_measurement && !row.limited && std::isfinite(row.actual) && std::isfinite(row.reference)) {
                square+=std::pow(row.reference-row.actual,2);++count;
            }
            if(count>=30)response_weights[i]=std::sqrt(square/count)/std::max({3*gain_noise_[i].encoder_step_rad,6*gain_noise_[i].rms_rad,1e-5});
        }
        const auto schedule=plan_taught_response_measurement_schedule(timeline,room_request,path.sample_dt,response_weights,4.0);
        const double remaining=gain_deadline_?std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():1000;
        const double available_dwell=std::max(0.0,remaining-source_duration-reserved_s-4);
        auto warp=schedule.has_anchor && available_dwell>=measurement_cost_s?
            make_taught_measurement_time_warp(source_duration,schedule.common_anchor.source_index*path.sample_dt,available_dwell,1):TaughtMeasurementTimeWarp{};
        if(!warp.valid)return false;
        // Check the actual q_ss*rate^2 + q_s*rate_acc source clock before
        // scheduling a stop; keep the original Safety velocity/acceleration.
        bool smooth=true;JointVector previous_velocity;double previous_time=0;
        for(double t=0;t<=warp.wall_duration_s+.005;t+=.005) {
            const auto clock=sample_taught_measurement_time_warp(warp,std::min(t,warp.wall_duration_s));
            const auto ref=serial_arm::calibration_retime::interpolate(timeline,clock.source_time_s/path.sample_dt,path.sample_dt,clock.rate);
            for(std::size_t i=0;i<n;++i) {
                smooth=smooth && std::abs(ref.velocity[i])<=cfg_.safety.limits.max_vel[i]+1e-6;
                if(!previous_velocity.empty())smooth=smooth && std::abs(ref.velocity[i]-previous_velocity[i])/(t-previous_time)<=cfg_.safety.limits.max_acc[i]+1e-5;
            }
            previous_velocity=ref.velocity;previous_time=t;
        }
        if(!smooth){gain_add_warning("passing_purpose_smooth_clock_unavailable");return false;}
        gain_require_trial(source_duration+1+measurement_cost_s+reserved_s+2);
        bool measured=false,completed=false,resumed=false;double source_time=0,inserted_s=0,pause_begin=0,pause_end=0;
        JointVector reference=endpoint.reference_origin,reference_velocity(n,0);
        const auto mode=profile==JointGainProfile::COMPLIANT?JointImpedanceMode::COMPLIANT_TRACKING:JointImpedanceMode::RIGID_TRACKING;
        std::vector<std::vector<Sample>> rows(n);const auto started=Robot::Clock::now();
        const std::string record="gain-trajectory-"+label+"-passing-purpose.csv";
        const auto install_reference=[&] {
            gain_cycle_reference_error_=nullptr;
            gain_cycle_reference_until_=gain_deadline_.value_or(started+std::chrono::seconds(1000));
            gain_cycle_reference_=[&,this](Robot::TimePoint now) {
                const double elapsed=std::max(0.0,std::chrono::duration<double>(now-started).count());
                const auto clock=warp.valid?sample_taught_measurement_time_warp(warp,elapsed):TaughtMeasurementClock{};
                source_time=warp.valid?clock.source_time_s:std::min(elapsed,source_duration);
                completed=warp.valid?clock.complete:source_time>=source_duration;
                const double rate=completed?0:warp.valid?clock.rate:1;
                const auto next=serial_arm::calibration_retime::interpolate(timeline,source_time/path.sample_dt,path.sample_dt,rate);
                reference=next.position;reference_velocity=completed?JointVector(n,0):next.velocity;
                const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{reference,reference_velocity},now,MotionSource::CALIBRATION);
                if(!sent)throw machine_robot_error("passing-purpose taught reference",sent.error());
            };
        };
        const auto finish=[&](bool complete,const std::string& reason) {
            {std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_={};gain_capture_active_=false;
                if((!measured || resumed) && gain_capture_rows_.size()==n)for(std::size_t i=0;i<n;++i){const auto& tail=gain_capture_rows_[i];rows[i].insert(rows[i].end(),tail.begin(),tail.end());}
                gain_capture_rows_.clear();
                if(complete)gain_remember_completed_endpoint_locked(ordered.back(),record);
                else {gain_completed_endpoint_reference_.clear();gain_completed_endpoint_record_.clear();}
                if(robot_.get_state()==RobotState::ACTIVE)gain_hold_for_assessment_locked(gains,!complete);}
            gain_write_response_csv(record,rows,[&](double time){return sample_taught_measurement_time_warp(warp,time).source_time_s;});
            std::ostringstream out;out<<std::setprecision(17)<<"{\"kind\":\"taught_path_purpose_dwell\",\"stage\":\"trajectory_purpose_pass\",\"completed\":"<<(complete?"true":"false")
                <<",\"validated\":false,\"method\":\"purpose_measurement_during_normal_taught_path_traversal\",\"reference_schedule\":\"control_cycle_monotonic_with_budgeted_interior_measurement_dwell\",\"playback_rate\":1,\"validation_scope\":\"variable_gain_response_discovery_requires_frozen_complete_path\",\"record_path\":\""<<record
                <<"\",\"source_time_s\":"<<source_time<<",\"source_duration_s\":"<<source_duration<<",\"elapsed_s\":"<<std::chrono::duration<double>(Robot::Clock::now()-started).count()
                <<",\"source_index\":"<<(gain_next_reverse_?path.positions.size()-1-schedule.common_anchor.source_index:schedule.common_anchor.source_index)
                <<",\"reverse\":"<<(gain_next_reverse_?"true":"false")<<",\"pause_begin_s\":"<<pause_begin<<",\"pause_end_s\":"<<pause_end
                <<",\"resumed\":"<<(resumed?"true":"false")<<",\"arrival_error_gate\":false,\"reference_rad\":";
            machine_json_joint_vector(out,schedule.common_anchor.position);out<<",\"measurement_dwell_s\":"<<inserted_s
                <<",\"anchor_selection\":\"measured_axis_tracking_need_and_local_demonstrated_motion_with_legal_room\",\"nearby_response_motion_score\":"<<schedule.common_anchor.nearby_response_motion_score
                <<",\"nearby_reference_span_rad\":";machine_json_joint_vector(out,schedule.common_anchor.nearby_reference_span_rad);
            out<<",\"axis_response_weights\":";
            machine_json_joint_vector(out,response_weights);out<<",\"endpoint_identification_required\":false,\"exact_anchor_arrival_required\":false"
                <<",\"endpoint_reference_continuation\":"<<(endpoint.reference_continuation?"true":"false")
                <<",\"continuation_record_path\":\""<<machine_json_escape(endpoint.continuation_record)<<"\",\"initial_reference_rad\":";
            machine_json_joint_vector(out,endpoint.reference_origin);out<<",\"initial_actual_rad\":";machine_json_joint_vector(out,endpoint.position);
            out<<",\"reason\":\""<<machine_json_escape(reason)<<"\",\"kp\":";
            machine_json_joint_vector(out,gains.kp);out<<",\"kd\":";machine_json_joint_vector(out,gains.kd);out<<'}';gain_experiment_records_.push_back(out.str());
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();}
        };
        try {
            {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) || !robot_.set_calibration_tracking_gains(gains,mode))
                    throw std::runtime_error("taught_path_purpose_dwell_control_mode_rejected");
                robot_.set_admittance_suspended(true);robot_.clear_calibration_disturbance();
                gain_capture_begin(label+"-passing",started,true);install_reference();}
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="trajectory_purpose_pass";gain_probe_frequency_hz_={};}
            gain_probe_begin(label+"-passing","all",source_duration,0,"normal_taught_path_with_purpose_dwell");
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);const auto now=Robot::Clock::now();
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("taught_path_purpose_dwell_interrupted");
                const double elapsed=std::chrono::duration<double>(now-started).count();
                gain_probe_update(source_time,reference,"normal_taught_path_with_purpose_dwell");
                if(completed)break;
                if(!measured && sample_taught_measurement_time_warp(warp,elapsed).at_anchor) {
                    measured=true;gain_cycle_reference_={};gain_capture_active_=false;
                    for(std::size_t i=0;i<n;++i){auto& part=gain_capture_rows_[i];rows[i].insert(rows[i].end(),part.begin(),part.end());}gain_capture_rows_.clear();
                    gain_remember_completed_endpoint_locked(schedule.common_anchor.position,record);
                    gain_hold_for_assessment_locked(gains);pause_begin=elapsed;lock.unlock();
                    const double remaining_path=source_duration-warp.anchor_source_s+2;
                    const auto original_index=gain_next_reverse_?path.positions.size()-1-schedule.common_anchor.source_index:schedule.common_anchor.source_index;
                    measure(original_index,remaining_path);
                    JointVector accepted;
                    {std::lock_guard<std::mutex> guard(mutex_);accepted=last_output_->joint_cmd.pos;}
                    std::vector<std::vector<Sample>> handoff;
                    if(!serial_arm::calibration_recovery::emitted_endpoint_matches(accepted,schedule.common_anchor.position)) {
                        gain_transition_to_endpoint(gains,accepted,schedule.common_anchor.position,label+"-passing-resume",reserved_s+remaining_path,handoff,true,"completed_passing_purpose_reference",profile);
                        gain_write_response_csv("gain-transition-"+label+"-passing-resume.csv",handoff);
                    }
                    lock.lock();const double elapsed_after=std::chrono::duration<double>(Robot::Clock::now()-started).count();
                    inserted_s=std::max(0.0,elapsed_after-warp.dwell_wall_start_s)+.05;
                    warp=make_taught_measurement_time_warp(source_duration,warp.anchor_source_s,inserted_s,1);
                    pause_end=elapsed_after;resumed=true;
                    if(!runtime_.set_impedance_mode(robot_,mode) || !robot_.set_execution_gain_profile(profile) || !robot_.set_calibration_tracking_gains(gains,mode))
                        throw std::runtime_error("cannot_resume_demonstrated_path_after_purpose_measurement");
                    robot_.set_admittance_suspended(true);gain_capture_begin(label+"-passing-resume",started,true);install_reference();
                    model_calibration_gain_stage_.store(profile==JointGainProfile::COMPLIANT?1:profile==JointGainProfile::RIGID?0:2);
                    {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="trajectory_purpose_pass";}
                    gain_probe_begin(label+"-passing","all",source_duration,0,"normal_taught_path_with_purpose_dwell");
                }
                const auto cursor=cycle_counter_;if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))
                    throw std::runtime_error("taught_path_purpose_dwell_feedback_unavailable");
            }
            finish(true,"taught_path_purpose_dwell_complete");gain_next_reverse_=!gain_next_reverse_;
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);++gain_path_pass_current_;}return true;
        } catch(...) {const auto failure=std::current_exception();finish(false,"taught_path_purpose_dwell_interrupted_or_partial");std::rethrow_exception(failure);}
    }

    // Use the checked demonstration itself to visit a measurement posture.
    // Feedback is recorded as read; proximity to a nominal anchor is never an
    // arrival threshold. The short experiment rechecks actual available room.
    void gain_visit_measurement_anchor(const JointImpedanceGains& gains,std::size_t from,std::size_t to,
        double reserved_s,const std::string& label) {
        using namespace serial_arm::servo_tuning;
        const auto& path=*model_calibration_trajectory_;
        auto plan=std::make_shared<TaughtAnchorVisitPlan>(plan_taught_anchor_visit(path.positions,path.sample_dt,from,to,
            cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc));
        if(!plan->valid)throw GainQualificationFailure("demonstrated_measurement_visit_unavailable:"+plan->reason);
        gain_require_trial(plan->duration_s+reserved_s+.5);
        auto started=Robot::Clock::now();
        const std::string record="gain-anchor-"+label+".csv";
        std::vector<std::vector<Sample>> rows;bool capturing=false;
        const auto finish=[&](bool completed) {
            {std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_={};gain_capture_active_=false;
                if(capturing)rows=std::move(gain_capture_rows_);else rows.resize(cfg_.joint_names.size());
                if(completed)gain_remember_completed_endpoint_locked(path.positions[to],record);
                if(robot_.get_state()==RobotState::ACTIVE)gain_hold_for_assessment_locked(gains,!completed);}
            gain_write_response_csv(record,rows);
            std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"taught_measurement_anchor_visit\",\"completed\":"<<(completed?"true":"false")
                <<",\"reference_only\":true,\"source_index_from\":"<<from<<",\"source_index_to\":"<<to
                <<",\"planned_cost_s\":"<<plan->duration_s<<",\"elapsed_s\":"<<std::chrono::duration<double>(Robot::Clock::now()-started).count()
                <<",\"source\":\"checked_demonstration_Hermite_with_rest_to_rest_source_clock\",\"arrival_error_gate\":false,\"record_path\":\""<<record<<"\",\"reference_rad\":";
            machine_json_joint_vector(note,path.positions[to]);note<<'}';gain_experiment_records_.push_back(note.str());
        };
        try {
            {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_TRACKING) ||
                   !robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
                   !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::RIGID_TRACKING))
                    throw std::runtime_error("cannot select demonstrated measurement-anchor travel");
                robot_.set_admittance_suspended(true);robot_.clear_calibration_disturbance();}
            JointVector entry;bool reference_continuation=false;
            {std::lock_guard<std::mutex> lock(mutex_);
                if(!last_output_)throw std::runtime_error("measurement-anchor handoff requires fresh feedback");
                entry=last_output_->joint_cmd.pos;
                // A short test may preserve a measured equilibrium rather
                // than the nominal source knot. Join that accepted reference
                // smoothly, retaining the previously emitted source knot as
                // the continuation credential. A measured equilibrium is not
                // a newly completed path knot or an arrival requirement.
                reference_continuation=serial_arm::calibration_recovery::emitted_endpoint_matches(
                    gain_completed_endpoint_reference_,path.positions[from]);
            }
            if(!reference_continuation || !serial_arm::calibration_recovery::emitted_endpoint_matches(entry,path.positions[from])) {
                std::vector<std::vector<Sample>> handoff;
                gain_transition_to_endpoint(gains,entry,path.positions[from],label+"-handoff",plan->duration_s+reserved_s+.5,handoff,reference_continuation,
                    "accepted_reference_after_completed_interior_measurement");
                gain_write_response_csv("gain-anchor-"+label+"-handoff.csv",handoff);
            }
            started=Robot::Clock::now();
            {std::lock_guard<std::mutex> lock(mutex_);gain_capture_begin(label,started,true);capturing=true;
                gain_cycle_reference_error_=nullptr;
                gain_cycle_reference_until_=started+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(plan->duration_s+.2));
                gain_cycle_reference_=[this,plan,started](Robot::TimePoint now) {
                    const auto p=sample_taught_anchor_visit(*plan,std::max(0.0,std::chrono::duration<double>(now-started).count()));
                    if(!p.valid)throw std::runtime_error("invalid demonstrated measurement-anchor clock");
                    const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{p.position,p.velocity},now,MotionSource::CALIBRATION);
                    if(!sent)throw machine_robot_error("demonstrated measurement-anchor reference",sent.error());
                };}
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="trajectory_measurement_visit";}
            gain_probe_begin(label,"all",plan->duration_s,0,"visiting_demonstrated_interior_measurement_anchor");
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("measurement-anchor visit interrupted");
                const auto elapsed=std::chrono::duration<double>(Robot::Clock::now()-started).count();
                gain_probe_update(elapsed,{},"visiting_demonstrated_interior_measurement_anchor");
                if(elapsed>=plan->duration_s)break;
                const auto cursor=cycle_counter_;
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))
                    throw std::runtime_error("measurement-anchor visit lost feedback");
            }
            finish(true);gain_probe_end("demonstrated_measurement_anchor_reached",true);
        } catch(...) {const auto error=std::current_exception();finish(false);std::rethrow_exception(error);}
    }

    void gain_record_online_update(const serial_arm::servo_tuning::OnlineTrajectoryReport& report,
        const JointImpedanceGains& before,const JointImpedanceGains& proposal,double time,
        const std::string& record,bool admitted,bool installed) {
        std::ostringstream out;out<<std::setprecision(17)
            <<"{\"kind\":\"online_trajectory_PD_update\",\"stage\":\"trajectory_adapting\",\"method\":\"indirect_local_IV_ARX_projected_PD\""
            <<",\"reference_only\":true,\"accuracy_thresholds_enabled\":false,\"completed\":false,\"time_s\":"<<time
            <<",\"source_record_path\":\""<<machine_json_escape(record)<<"\",\"record_path\":\""<<machine_json_escape(record)<<'"'
            <<",\"future_feedforward_assumption\":\"last_executed_non_PD_torque_held_over_local_prediction_horizon\""
            <<",\"candidate_generated\":"<<(report.changed?"true":"false")<<",\"admission_allowed\":"<<(admitted?"true":"false")
            <<",\"installed\":"<<(installed?"true":"false");
        const auto vector=[&](const char* name,const JointVector& value){out<<",\""<<name<<"\":";machine_json_joint_vector(out,value);};
        vector("previous_kp",before.kp);vector("previous_kd",before.kd);vector("proposed_kp",proposal.kp);vector("proposed_kd",proposal.kd);
        out<<",\"models\":[";
        for(std::size_t i=0;i<report.models.size();++i){if(i)out<<',';const auto& m=report.models[i];
            out<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"valid\":"<<(m.valid?"true":"false")
                <<",\"reason\":\""<<machine_json_escape(m.reason)<<"\",\"interpretation\":\""<<machine_json_escape(m.interpretation)<<'"'
                <<",\"structure\":\""<<machine_json_escape(m.structure)<<"\",\"parameter_count\":"<<m.parameter_count
                <<",\"dt_s\":"<<m.dt_s<<",\"delay_s\":"<<m.delay_s<<",\"iv_rank\":"<<m.iv_rank<<",\"iv_condition\":"<<gain_finite_json(m.iv_condition)
                <<",\"training_samples\":"<<m.training_samples<<",\"validation_samples\":"<<m.validation_samples
                <<",\"duplicate_feedback\":"<<m.duplicate_feedback<<",\"rejected_samples\":"<<m.rejected_samples
                <<",\"fresh_samples\":"<<m.samples<<",\"command_samples\":"<<m.command_samples<<",\"resampled_samples\":"<<m.resampled_samples
                <<",\"feedback_interval_min_s\":"<<m.feedback_interval_min_s<<",\"feedback_interval_max_s\":"<<m.feedback_interval_max_s
                <<",\"observed_encoder_step_rad\":"<<m.observed_encoder_step_rad
                <<",\"response_establishment_trial\":"<<(m.response_establishment_trial?"true":"false")
                <<",\"measured_error_bias_rad\":"<<gain_finite_json(m.measured_error_bias_rad)
                <<",\"unresolved_trial_origin_kp\":"<<gain_finite_json(m.unresolved_trial_origin_kp)
                <<",\"unresolved_trial_kp_ceiling\":"<<gain_finite_json(m.unresolved_trial_kp_ceiling)
                <<",\"fit_attempts\":"<<m.fit_attempts<<",\"rank_rejections\":"<<m.rank_rejections
                <<",\"plausibility_rejections\":"<<m.plausibility_rejections<<",\"prediction_rejections\":"<<m.prediction_rejections
                <<",\"input_rms_nm\":"<<m.independent_input_rms_nm<<",\"position_noise_rad\":"<<m.position_noise_rad
                <<",\"training_span_rad\":"<<m.training_span_rad<<",\"validation_error_limit_rad\":"<<m.validation_error_limit_rad
                <<",\"training_rms_rad\":"<<gain_finite_json(m.training_rms_rad)<<",\"validation_rms_rad\":"<<gain_finite_json(m.validation_rms_rad)
                <<",\"prior_validation_rms_rad\":"<<gain_finite_json(m.prior_validation_rms_rad)
                <<",\"arx\":["<<m.arx_a1<<','<<m.arx_a2<<','<<m.arx_b0<<','<<m.arx_b1<<','<<m.bias<<']'
                <<",\"predicted_cost_before\":"<<gain_finite_json(m.predicted_cost_before)<<",\"predicted_cost_after\":"<<gain_finite_json(m.predicted_cost_after)
                <<",\"step_fraction\":"<<m.applied_step_fraction<<'}';
        }
        out<<"],\"reasons\":";gain_json_warnings(out,report.reasons);out<<'}';
        gain_candidate_records_.push_back(out.str());
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();
            gain_probe_message_=installed?"online_PD_updated_during_taught_motion":"online_measurement_continues";
            gain_probe_kp_=installed?proposal.kp.front():before.kp.front();gain_probe_kd_=installed?proposal.kd.front():before.kd.front();}
    }

    ServoTrial gain_trajectory_trial(JointImpedanceGains gains,JointGainProfile profile,
                                    bool admittance,bool configured,const std::string& label,
                                    const std::string& stage="trajectory_search",JointImpedanceGains* online_gains=nullptr,
                                    const std::vector<serial_arm::servo_tuning::ModelAxisLimits>* online_limits=nullptr,
                                    const FrictionCalibrationTrajectory* replay_path=nullptr,bool local_reverse=false) {
        using namespace serial_arm::servo_tuning;
        gains=gain_protocol_gains(gains);
        if(!model_calibration_trajectory_ || model_calibration_trajectory_->positions.size()<2)
            throw std::runtime_error("gain calibration requires a checked demonstration trajectory");
        const auto n=cfg_.joint_names.size();const auto& path=replay_path?*replay_path:*model_calibration_trajectory_;
        ServoTrial trial;trial.trajectory_replay=true;trial.reverse=replay_path?local_reverse:gain_next_reverse_;
        trial.online_adaptation=online_gains && online_limits && !admittance && !configured;
        if(trial.online_adaptation && online_limits->size()!=n)throw std::invalid_argument("online gain mapping mismatch");
        model_calibration_direction_.store(trial.reverse?-1:1);model_calibration_speed_tier_.store(2);
        trial.playback_rate=1.0;
        trial.duration_s=path.sample_dt*static_cast<double>(path.positions.size()-1)/trial.playback_rate;
        trial.rows.resize(n);trial.metrics.resize(n);trial.models.resize(n);trial.motion.resize(n);trial.plans.assign(n,gain_requirements_);trial.trajectory_assessments.resize(n);
        std::vector<JointVector> ordered=path.positions;if(trial.reverse)std::reverse(ordered.begin(),ordered.end());
        trial.center=ordered.front();trial.min_position=trial.max_position=trial.center;
        JointVector taught_low=ordered.front(),taught_high=taught_low;
        for(const auto& q:ordered)for(std::size_t i=0;i<n;++i){taught_low[i]=std::min(taught_low[i],q[i]);taught_high[i]=std::max(taught_high[i],q[i]);}
        const auto mode=profile==JointGainProfile::COMPLIANT?JointImpedanceMode::COMPLIANT_TRACKING:JointImpedanceMode::RIGID_TRACKING;
        {
            std::lock_guard<std::mutex> lock(model_calibration_mutex_);
            gain_probe_stage_=stage;gain_probe_kind_=label;gain_probe_joint_="all";gain_probe_reference_.clear();
            std::ostringstream pending;pending<<"{\"completed\":false,\"stage\":\""<<machine_json_escape(stage)<<"\",\"kind\":\""<<machine_json_escape(label)
                <<"\",\"reason\":\"trajectory_endpoint_check\"}";gain_probe_diagnostic_json_=pending.str();
        }
        gain_require_trial(trial.duration_s+gain_requirements_.tail_s+2.5);
        const auto endpoint_observation=gain_endpoint_observation(true,ordered.front());
        {
            std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(!last_output_ || last_output_->actuator_state.feedback_received_ns.size()!=n)
                throw std::runtime_error("demonstrated trajectory replay requires fresh endpoint feedback");
            const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            double age_ms=0;
            for(const auto stamp:last_output_->actuator_state.feedback_received_ns) {
                if(!stamp || stamp>now_ns)throw std::runtime_error("demonstrated trajectory replay has invalid endpoint feedback timestamps");
                age_ms=std::max(age_ms,static_cast<double>(now_ns-stamp)/1.0e6);
            }
            const auto endpoint=serial_arm::calibration_recovery::select_endpoint(endpoint_observation.position,endpoint_observation.velocity,ordered.front(),ordered.front(),std::max(age_ms,endpoint_observation.age_ms),dynamics_.get_info().joint_types);
            if(!endpoint.valid || (!endpoint_observation.reference_continuation && !endpoint.at_endpoint))
                throw GainQualificationFailure("demonstrated trajectory endpoint is stale or not aligned; hold and align manually before replay");
            if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,mode) ||
               !robot_.set_execution_gain_profile(profile) ||
               !robot_.set_calibration_reference_behavior(admittance?ReferenceBehavior::MODEL_ADMITTANCE:ReferenceBehavior::NATIVE_IMPEDANCE))
                throw std::runtime_error("cannot select demonstrated-trajectory gain replay");
            robot_.set_admittance_suspended(!admittance);
            if(configured)robot_.clear_calibration_tracking_gains();
            else if(!robot_.set_calibration_tracking_gains(gains,mode))throw std::runtime_error("trajectory gain candidate rejected");
        }
        for(std::size_t axis=0;axis<n;++axis) {
            auto& req=trial.plans[axis];req.probe_duration_s=trial.duration_s;req.frequencies_hz={};
            double lo=ordered.front()[axis],hi=lo;for(const auto& q:ordered){lo=std::min(lo,q[axis]);hi=std::max(hi,q[axis]);}
            req.amplitude_rad=.5*(hi-lo);
            if(axis<gain_noise_.size()){req.noise_rms_rad=gain_noise_[axis].rms_rad;req.encoder_step_rad=gain_noise_[axis].encoder_step_rad;req.velocity_noise_floor=gain_noise_[axis].velocity_peak+1e-6;}
        }
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_frequency_hz_={};
            gain_probe_kp_=gains.kp.front();gain_probe_kd_=gains.kd.front();}
        const std::string record="gain-trajectory-"+label+".csv";
        trial.record_path=record;
        std::ofstream csv(std::filesystem::path(model_calibration_directory_)/record);
        if(!csv)throw std::runtime_error("cannot record demonstrated-trajectory gain replay");
        csv<<"time_s,joint,reference,reference_velocity,actual,velocity,residual,residual_valid,limited,new_measurement\n"<<std::setprecision(17);
        std::vector<std::vector<Sample>> transition_rows;
        Robot::TimePoint started{};bool beginning=true,path_counted=replay_path!=nullptr;std::uint64_t cursor=0;std::vector<std::uint64_t> stamps(n,0);JointVector last_fresh(n,0);
        double next_online_update=4;
        const auto input_limits=trial.online_adaptation?*online_limits:std::vector<ModelAxisLimits>{};
        const auto planned_gains=std::make_shared<JointImpedanceGains>(gains);
        try {
            gain_transition_to_endpoint(gains,endpoint_observation.reference_origin,ordered.front(),label,
                trial.duration_s+gain_requirements_.tail_s+.3,transition_rows,endpoint_observation.reference_continuation,
                "accepted_reference_after_completed_demonstration_endpoint",profile);
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_=stage;gain_effective_pass_duration_s_=trial.duration_s;}
            gain_probe_begin(label,"all",trial.duration_s+gain_requirements_.tail_s,0,"trajectory_replay");
            gain_probe_diagnostic(0,gains,trial.plans[0],trial,"measurement_pending",record,false);
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("continuous demonstrated-trajectory gain replay interrupted");
                const auto now=Robot::Clock::now();
                if(beginning){
                    started=now;beginning=false;gain_capture_begin(label,started,admittance);
                    cursor=cycle_counter_;
                    gain_cycle_reference_error_=nullptr;
                    gain_cycle_reference_until_=started+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(trial.duration_s+gain_requirements_.tail_s+.2));
                    const auto timeline=std::make_shared<const std::vector<JointVector>>(ordered);
                    const double sample_dt=path.sample_dt,rate=trial.playback_rate,duration=trial.duration_s;
                    const bool adapting=trial.online_adaptation;
                    gain_cycle_reference_=[this,timeline,sample_dt,rate,duration,started,n,adapting,input_limits,taught_low,taught_high,planned_gains](Robot::TimePoint cycle_time) {
                        const double elapsed=std::max(0.0,std::chrono::duration<double>(cycle_time-started).count());
                        const double progress=std::min(static_cast<double>(timeline->size()-1),elapsed*rate/sample_dt);
                        const auto reference=serial_arm::calibration_retime::interpolate(*timeline,progress,sample_dt,rate);
                        const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{reference.position,elapsed<duration?reference.velocity:JointVector(n,0)},cycle_time,MotionSource::CALIBRATION);
                        if(!sent)throw machine_robot_error("cycle sampled trajectory reference",sent.error());
                        if(adapting) {
                            JointVector input(n,0);
                            gain_online_input_plan_.assign(n,0);
                            if(last_output_ && elapsed<duration-.8 && last_output_->joint_cmd.kp.size()==n && last_output_->joint_cmd.kd.size()==n) {
                                const auto& out=*last_output_;
                                for(std::size_t i=0;i<n;++i) {
                                    const double P=out.joint_cmd.kp[i],D=out.joint_cmd.kd[i],q=out.joint_state.pos[i];
                                    const double pd=P*(reference.position[i]-q)+D*(reference.velocity[i]-out.joint_state.vel[i]);
                                    const double prior_input=out.calibration_disturbance.size()==n?out.calibration_disturbance[i]:0;
                                    const double ff=out.joint_cmd.tor.size()==n?out.joint_cmd.tor[i]-prior_input:0;
                                    const double headroom=std::max(0.0,std::min(input_limits[i].effort_budget-std::abs(pd),
                                        cfg_.safety.limits.max_effort[i]-std::abs(ff+pd)));
                                    double planned=online_trajectory_injection(elapsed,i,gain_model_initialization_.inertia_max[i],
                                        planned_gains->kp[i],planned_gains->kd[i],input_limits[i].effort_budget,
                                        std::max(0.0,reference.position[i]-taught_low[i]),std::max(0.0,taught_high[i]-reference.position[i]),
                                        1.0/cfg_.runtime.ctrl_frequency_hz,duration);
                                    if(i<actuator_info_.size()) {
                                        const double quantum=std::abs(cfg_.mapper.tor_ratio[i])*actuator_info_[i].effort_resolution;
                                        if(quantum>0)planned=std::round(planned/quantum)*quantum;
                                    }
                                    gain_online_input_plan_[i]=planned;
                                    input[i]=std::clamp(planned,-headroom,headroom);
                                    if((input[i]<0 && q<=taught_low[i]) || (input[i]>0 && q>=taught_high[i]))input[i]=0;
                                }
                            }
                            const auto applied=robot_.set_calibration_disturbance(input,cycle_time);
                            if(!applied)throw machine_robot_error("online trajectory identification input",applied.error());
                        }
                    };
                }
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                const double time=std::chrono::duration<double>(now-started).count();
                if(time>trial.duration_s+gain_requirements_.tail_s)break;
                if(trial.online_adaptation && time>=next_online_update && time<trial.duration_s-.8) {
                    next_online_update=time+1;
                    std::vector<std::vector<Sample>> measured(n);
                    for(std::size_t i=0;i<n;++i)for(const auto& row:gain_capture_rows_[i])if(row.time_s>=time-8)measured[i].push_back(row);
                    const auto noise=gain_noise_;const auto incumbent=gains;
                    JointVector ff(n,0);
                    for(std::size_t i=0;i<n;++i)if(!measured[i].empty())ff[i]=measured[i].back().applied_feedforward_nm;
                    std::vector<OnlineTrajectoryReference> future;
                    for(double dt=-.06;dt<=.8+1e-9;dt+=.02) {
                        const auto reference=serial_arm::calibration_retime::interpolate(ordered,
                            std::min(trial.duration_s,time+dt)/path.sample_dt,path.sample_dt,1);
                        future.push_back({time+dt,reference.position,time+dt<trial.duration_s?reference.velocity:JointVector(n,0),ff});
                    }
                    // Matrix fitting and prediction run on the task worker;
                    // the control owner keeps sending the same 1x path while
                    // this lock is released. No UI wakeup times the reference.
                    lock.unlock();
                    OnlineTrajectoryOptions options;
                    if(profile==JointGainProfile::COMPLIANT){options.use=OnlineTrajectoryUse::Compliant;options.compliant_displacement_rad=gain_requirements_.local_range_rad;}
                    const auto report=online_trajectory_candidate({incumbent.kp,incumbent.kd},measured,future,
                        gain_model_initialization_.inertia_max,input_limits,noise,options,
                        &gain_online_states_.at(profile==JointGainProfile::COMPLIANT?1:0));
                    const JointImpedanceGains proposal{report.gains.kp,report.gains.kd};
                    const bool admitted=report.changed && gain_admit_candidate(incumbent,proposal,measured,label+"-online-update");
                    lock.lock();
                    bool installed=false;
                    if(admitted && robot_.get_state()==RobotState::ACTIVE && !model_calibration_cancel_.load()) {
                        if(!robot_.set_calibration_tracking_gains(proposal,mode))throw std::runtime_error("online trajectory gain update rejected");
                        gains=proposal;*online_gains=gains;*planned_gains=gains;model_calibration_adaptation_revisions_.fetch_add(1);installed=true;
                    }
                    lock.unlock();gain_record_online_update(report,incumbent,proposal,time,trial.record_path,admitted,installed);lock.lock();
                }
                const double progress=std::min(static_cast<double>(ordered.size()-1),time*trial.playback_rate/path.sample_dt);
                const auto ref=serial_arm::calibration_retime::interpolate(ordered,progress,path.sample_dt,trial.playback_rate);
                JointVector velocity=time<trial.duration_s?ref.velocity:JointVector(n,0);
                gain_probe_update(time,ref.position,time<trial.duration_s?"trajectory_replay":"trajectory_settling");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}) || !last_output_ || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("demonstrated-trajectory gain replay lost continuous feedback");
                cursor=cycle_counter_;const auto& out=*last_output_;
                if(out.joint_state.pos.size()!=n || out.joint_state.vel.size()!=n || out.actuator_state.feedback_received_ns.size()!=n)
                    throw std::runtime_error("demonstrated-trajectory gain feedback dimensions changed");
                if(!path_counted) {std::lock_guard<std::mutex> status(model_calibration_mutex_);++gain_path_pass_current_;path_counted=true;}
                if(admittance && time>.5 && (!out.observer_valid || !out.admittance_active))throw std::runtime_error("requested admittance trajectory validation needs active MDK and valid observer feedback");
                const auto& positions=admittance?out.virtual_ref_pos:out.nominal_ref_pos;
                const auto& velocities=admittance?out.virtual_ref_vel:out.nominal_ref_vel;
                const bool limited=std::any_of(out.reference_limited.begin(),out.reference_limited.end(),[](auto v){return v!=0;}) || std::any_of(out.torque_limited.begin(),out.torque_limited.end(),[](auto v){return v!=0;});
                const auto following=serial_arm::calibration_replay_supervisor::evaluate(ref.position,out.joint_state.pos,out.joint_state.vel,cfg_.safety.limits.max_vel);
                if(!following.valid)throw std::runtime_error("demonstrated-trajectory gain replay has invalid following feedback");
                // Tracking quality is assessed from the complete captured
                // traversal. A diagnostic following-error threshold is not a
                // new Safety limit and must not truncate an otherwise valid
                // demonstrated path before the operator can inspect it.
                const double measured_time=std::chrono::duration<double>(Robot::TimePoint(std::chrono::nanoseconds(last_cycle_monotonic_ns_))-started).count();
                if(measured_time<0)continue;
                for(std::size_t axis=0;axis<n;++axis) {
                    Sample row;row.time_s=measured_time;row.reference=positions.size()==n?positions[axis]:ref.position[axis];row.reference_velocity=velocities.size()==n?velocities[axis]:velocity[axis];
                    row.actual=out.joint_state.pos[axis];row.velocity=out.joint_state.vel[axis];row.limited=limited;
                    const auto stamp=out.actuator_state.feedback_received_ns[axis];row.new_measurement=stamp>stamps[axis] && stamp>0;stamps[axis]=std::max(stamps[axis],stamp);
                    if(row.new_measurement){if(measured_time-last_fresh[axis]>.06)throw std::runtime_error("demonstrated-trajectory gain replay lost per-joint continuous feedback");last_fresh[axis]=measured_time;}
                    if(measured_time-last_fresh[axis]>.06)throw std::runtime_error("demonstrated-trajectory gain replay lost per-joint continuous feedback");
                    if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual) || !std::isfinite(row.velocity))throw std::runtime_error("demonstrated-trajectory gain replay has non-finite feedback");
                    if(out.observer_valid && out.residual_raw.size()==n){row.residual=out.residual_raw[axis];row.residual_valid=true;}
                    trial.min_position[axis]=std::min(trial.min_position[axis],row.actual);trial.max_position[axis]=std::max(trial.max_position[axis],row.actual);
                    trial.rows[axis].push_back(row);

                }
            }
            {std::lock_guard<std::mutex> lock(mutex_);robot_.clear_calibration_disturbance();gain_online_input_plan_.clear();gain_remember_completed_endpoint_locked(ordered.back(),record);
                gain_hold_for_assessment_locked(gains);trial.rows=std::move(gain_capture_rows_);}
            csv.flush();if(!csv)throw std::runtime_error("demonstrated-trajectory gain recording failed");
        } catch(...) {
            const auto failure=std::current_exception();
            {std::lock_guard<std::mutex> lock(mutex_);gain_capture_active_=false;robot_.clear_calibration_disturbance();gain_online_input_plan_.clear();
                gain_completed_endpoint_reference_.clear();gain_completed_endpoint_record_.clear();
                if(robot_.get_state()==RobotState::ACTIVE)try{gain_hold_for_assessment_locked(gains,true);}catch(...){}
                if(!gain_capture_rows_.empty())trial.rows=std::move(gain_capture_rows_);}
            csv.close();
            try{gain_write_response_csv(record,trial.rows);gain_write_response_csv("gain-transition-"+label+".csv",transition_rows);}catch(const std::exception& error){gain_add_warning(std::string("interrupted_evidence_write_failed:")+error.what());}
            for(std::size_t axis=0;axis<n;++axis){const auto a=assess_trajectory(trial.rows[axis],trial.plans[axis],gain_noise_[axis],trial.duration_s);trial.metrics[axis]=a.metrics;trial.motion[axis]=a.motion;trial.trajectory_assessments[axis]=a;}
            // Endpoint preparation has its own real evidence. Do not replace
            // it with an empty full-trajectory diagnostic if replay never ran.
            if(!beginning)gain_probe_diagnostic(0,gains,trial.plans[0],trial,"measurement_interrupted",record,false);
            std::rethrow_exception(failure);
        }
        csv.close();
        gain_write_response_csv(record,trial.rows);
        gain_write_response_csv("gain-transition-"+label+".csv",transition_rows);
        if(!replay_path)gain_next_reverse_=!gain_next_reverse_;
        gain_probe_end("assessing",true);
        std::size_t worst=0;double severity=-1;trial.completed=true;trial.available=true;trial.passed=true;
        for(std::size_t axis=0;axis<n;++axis) {
            std::vector<double> measured_positions;for(const auto& row:trial.rows[axis])if(row.new_measurement)measured_positions.push_back(row.actual);
            const double observed_step=observed_encoder_step(std::move(measured_positions));
            if(observed_step>0)gain_noise_[axis].encoder_step_rad=gain_noise_[axis].encoder_step_rad>0?
                std::min(gain_noise_[axis].encoder_step_rad,observed_step):observed_step;
            trial.plans[axis].encoder_step_rad=gain_noise_[axis].encoder_step_rad;
            const auto a=assess_trajectory(trial.rows[axis],trial.plans[axis],gain_noise_[axis],trial.duration_s);
            trial.trajectory_assessments[axis]=a;trial.metrics[axis]=a.metrics;trial.motion[axis]=a.motion;
            trial.completed=trial.completed && trajectory_measurement_complete(trial.rows[axis],trial.duration_s,gain_requirements_.tail_s);
            trial.available=trial.available && trajectory_measurement_complete(trial.rows[axis],trial.duration_s,gain_requirements_.tail_s);
            // Flexible tracking records the same raw accuracy but its use is
            // qualified by bounded yielding/recovery, not rigid precision.
            trial.passed=trial.passed && (profile==JointGainProfile::COMPLIANT?!a.metrics.oscillating:a.passed);
            const double value=(a.passed?0:10)+a.metrics.rms_error/gain_requirements_.rms_error_rad;
            if(value>severity){severity=value;worst=axis;}
        }
        gain_probe_diagnostic(worst,gains,trial.plans[worst],trial,
            trial.available?"trajectory_measured_reference_only":"trajectory_measurement_incomplete",record);
        return trial;
    }

    void gain_refine_taught_segment(JointImpedanceGains& gains,JointGainProfile profile,
        const ServoTrial& trajectory,double reserved_s,const std::string& label,std::optional<std::size_t> passing_anchor={}) {
        using namespace serial_arm::servo_tuning;
        if(!model_calibration_trajectory_ || !trajectory.completed || trajectory.rows.empty())return;
        const auto& original=*model_calibration_trajectory_;
        TaughtLocalOptions options;options.tail_s=gain_requirements_.tail_s;
        if(passing_anchor){options.nearby_source_index=passing_anchor;options.maximum_source_distance_s=2*options.segment_s;}
        const double remaining=gain_deadline_?std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():1000;
        const auto plan=plan_taught_local_correction(trajectory.rows,gain_noise_,original.positions.size(),original.sample_dt,
            trajectory.reverse,trajectory.playback_rate,std::max(0.0,remaining-reserved_s-3),options);
        if(!plan.valid || plan.score<=1)return;
        const std::size_t endpoint=passing_anchor.value_or(gain_next_reverse_?original.positions.size()-1:0);
        const auto travel=[&](std::size_t from,std::size_t to) {
            return plan_taught_anchor_visit(original.positions,original.sample_dt,from,to,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc);
        };
        const auto entry=travel(endpoint,plan.start_index),back=travel(plan.end_index,plan.start_index),exit=travel(plan.end_index,endpoint);
        if(!entry.valid || !back.valid || !exit.valid)return;
        const double measured_cost=plan.duration_s+options.tail_s+2.5;
        const double comparison_cost=2*measured_cost+back.duration_s+entry.duration_s+exit.duration_s+3;
        if(!gain_trial_fits(comparison_cost,reserved_s+2))return;
        FrictionCalibrationTrajectory segment=original;
        const auto lo=std::min(plan.start_index,plan.end_index),hi=std::max(plan.start_index,plan.end_index);
        segment.positions.assign(original.positions.begin()+lo,original.positions.begin()+hi+1);
        TaughtLocalConditions conditions;conditions.start_index=plan.start_index;conditions.end_index=plan.end_index;
        conditions.reverse=plan.reverse;conditions.playback_rate=1;conditions.source_dt_s=original.sample_dt;
        conditions.tail_s=options.tail_s;conditions.feedforward_scope=to_string(cfg_.runtime.model_feedforward_mode);
        conditions.load_scope=model_calibration_options_.load_description.empty()?"declared_unchanged_task_load":model_calibration_options_.load_description;
        const auto write_axes=[&](std::ostream& out,const TaughtLocalAssessment& a) {
            out<<'[';for(std::size_t i=0;i<a.axes.size();++i){if(i)out<<',';const auto& x=a.axes[i];
                out<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"complete\":"<<(x.complete?"true":"false")
                    <<",\"rms_error_rad\":"<<x.rms_error_rad<<",\"peak_error_rad\":"<<x.peak_error_rad
                    <<",\"static_measured\":"<<(x.static_measured?"true":"false")<<",\"static_error_rad\":"<<x.static_error_rad
                    <<",\"same_direction_lag_s\":"<<x.same_direction_lag_s<<",\"dynamic_overshoot_rad\":"<<x.dynamic_overshoot_rad
                    <<",\"sustained_oscillation\":"<<(x.sustained_oscillation?"true":"false")
                    <<",\"comparison_uncertainty_rad\":"<<x.comparison_uncertainty_rad<<'}';}out<<']';
        };
        {std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"taught_local_correction_plan\",\"purpose\":\""<<label
            <<"\",\"source_index_from\":"<<plan.start_index<<",\"source_index_to\":"<<plan.end_index<<",\"reverse\":"<<(plan.reverse?"true":"false")
            <<",\"playback_rate\":1,\"source_dt_s\":"<<original.sample_dt<<",\"duration_s\":"<<plan.duration_s<<",\"complete_comparison_cost_s\":"<<comparison_cost
            <<",\"focus_joint\":\""<<machine_json_escape(cfg_.joint_names.at(plan.focus_axis))<<"\",\"focus_reference_span_rad\":"<<plan.focus_reference_span_rad
            <<",\"model_identified\":false,\"selection_source_record_path\":\""<<trajectory.record_path<<"\",\"reason\":\""<<plan.reason<<"\"}";
            gain_experiment_records_.push_back(note.str());}
        gain_visit_measurement_anchor(gains,endpoint,plan.start_index,comparison_cost-entry.duration_s+reserved_s,label+"-local-entry");
        auto baseline_trial=gain_trajectory_trial(gains,profile,false,false,"local-"+label+"-baseline","trajectory_local_comparison",nullptr,nullptr,&segment,plan.reverse);
        auto baseline=assess_taught_local_segment(baseline_trial.rows,gain_noise_,{gains.kp,gains.kd},conditions);
        std::size_t coordinate_poll=0;
        for(std::size_t attempt=0;attempt<(gain_tuning_mode_=="fine"?3u:2u);++attempt) {
            auto limits=gain_model_limits_;
            const auto proposal=taught_local_candidate({gains.kp,gains.kd},limits,gain_noise_,baseline,{},coordinate_poll,options);
            if(!proposal.changed){++coordinate_poll;continue;}
            const JointImpedanceGains next{proposal.gains.kp,proposal.gains.kd};
            if(!gain_trial_fits(back.duration_s+measured_cost+exit.duration_s+3,reserved_s+2) ||
               !gain_admit_candidate(gains,next,baseline_trial.rows,"taught-local-complete-group-before-comparison"))break;
            const auto incumbent=gains;
            gain_visit_measurement_anchor(incumbent,plan.end_index,plan.start_index,measured_cost+exit.duration_s+reserved_s+2,
                label+"-local-reset-"+std::to_string(attempt));
            auto candidate_trial=gain_trajectory_trial(next,profile,false,false,"local-"+label+"-candidate-"+std::to_string(attempt),
                "trajectory_local_comparison",nullptr,nullptr,&segment,plan.reverse);
            auto candidate=assess_taught_local_segment(candidate_trial.rows,gain_noise_,{next.kp,next.kd},conditions);
            auto comparison=taught_local_comparison_report(candidate,baseline,{next.kp,next.kd},{incumbent.kp,incumbent.kd});
            if(comparison.retry_recommended && gain_trial_fits(2*(back.duration_s+measured_cost)+exit.duration_s+3,reserved_s+2)) {
                std::ostringstream retry;retry<<"{\"kind\":\"taught_local_comparison_repeat\",\"reason\":\""<<comparison.reason
                    <<"\",\"original_baseline_record_path\":\""<<baseline_trial.record_path<<"\",\"original_candidate_record_path\":\""<<candidate_trial.record_path
                    <<"\",\"bounded_repeats\":1,\"raw_metrics_preserved\":true}";gain_candidate_records_.push_back(retry.str());
                gain_visit_measurement_anchor(incumbent,plan.end_index,plan.start_index,2*measured_cost+back.duration_s+exit.duration_s+reserved_s+2,label+"-local-repeat-baseline-reset-"+std::to_string(attempt));
                baseline_trial=gain_trajectory_trial(incumbent,profile,false,false,"local-"+label+"-repeat-baseline-"+std::to_string(attempt),"trajectory_local_comparison",nullptr,nullptr,&segment,plan.reverse);
                baseline=assess_taught_local_segment(baseline_trial.rows,gain_noise_,{incumbent.kp,incumbent.kd},conditions);
                gain_visit_measurement_anchor(incumbent,plan.end_index,plan.start_index,measured_cost+exit.duration_s+reserved_s+2,label+"-local-repeat-candidate-reset-"+std::to_string(attempt));
                candidate_trial=gain_trajectory_trial(next,profile,false,false,"local-"+label+"-repeat-candidate-"+std::to_string(attempt),"trajectory_local_comparison",nullptr,nullptr,&segment,plan.reverse);
                candidate=assess_taught_local_segment(candidate_trial.rows,gain_noise_,{next.kp,next.kd},conditions);
                comparison=taught_local_comparison_report(candidate,baseline,{next.kp,next.kd},{incumbent.kp,incumbent.kd});
            }
            const bool kept=comparison.preferred;
            std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"taught_local_complete_group_comparison\",\"purpose\":\""<<label
                <<"\",\"reference_only\":true,\"model_identified\":false,\"kept\":"<<(kept?"true":"false")
                <<",\"comparison_reason\":\""<<machine_json_escape(comparison.reason)<<"\",\"source_index_from\":"<<conditions.start_index<<",\"source_index_to\":"<<conditions.end_index
                <<",\"reverse\":"<<(conditions.reverse?"true":"false")<<",\"playback_rate\":1,\"source_dt_s\":"<<conditions.source_dt_s<<",\"tail_s\":"<<conditions.tail_s
                <<",\"feedforward_scope\":\""<<machine_json_escape(conditions.feedforward_scope)<<"\",\"load_scope\":\""<<machine_json_escape(conditions.load_scope)
                <<"\",\"changed_joint\":\""<<machine_json_escape(cfg_.joint_names[proposal.axis])<<"\",\"coordinate\":\""<<proposal.coordinate
                <<"\",\"reason\":\""<<proposal.reason<<"\",\"baseline_record_path\":\""<<baseline_trial.record_path<<"\",\"record_path\":\""<<candidate_trial.record_path
                <<"\",\"previous_kp\":";machine_json_joint_vector(note,incumbent.kp);note<<",\"previous_kd\":";machine_json_joint_vector(note,incumbent.kd);
            note<<",\"proposed_kp\":";machine_json_joint_vector(note,next.kp);note<<",\"proposed_kd\":";machine_json_joint_vector(note,next.kd);
            note<<",\"baseline_axes\":";write_axes(note,baseline);note<<",\"candidate_axes\":";write_axes(note,candidate);note<<'}';gain_candidate_records_.push_back(note.str());
            if(kept){gains=next;baseline=std::move(candidate);baseline_trial=std::move(candidate_trial);coordinate_poll=0;model_calibration_adaptation_revisions_.fetch_add(1);}
            else ++coordinate_poll;
            {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains);}
        }
        gain_visit_measurement_anchor(gains,plan.end_index,endpoint,reserved_s,label+"-local-exit");
    }

    // Final-window static error, position span and transient motion are separate
    struct GainHoldResult {
        std::size_t samples{0},observer_samples{0},integral_samples{0};double worst_error{0},worst_velocity{0};
        double raw_worst_velocity{0},worst_span{0},transient_peak_error{0},transient_excursion{0},duration_s{0},settling_window_s{0};
        JointVector target,mean_error,entry_actual;std::vector<std::size_t> fresh_samples;bool validated{false},completed{false},available{false},limited{false};
        bool controls_known{false},reference_fixed{false},independent_input_zero{false};
        JointImpedanceGains actual_gains;HoldCompensationCfg actual_compensation;
        std::string record_path,reason{"not_started"},target_source{"cycle.hold_target_pos"};
    };
    GainHoldResult gain_hold_trial(JointImpedanceGains gains,bool compliant,const HoldCompensationCfg& compensation) {
        using namespace serial_arm::servo_tuning;
        gains=gain_protocol_gains(gains);
        GainHoldResult result;const auto n=cfg_.joint_names.size();result.mean_error.assign(n,0);result.fresh_samples.assign(n,0);result.actual_gains=gains;result.actual_compensation=compensation;
        result.settling_window_s=.8;
        if(compensation.enabled)for(std::size_t i=0;i<n;++i) {
            if(compensation.integral_gain[i]<=0)throw GainQualificationFailure("hold integral trial needs positive per-axis integration gain");
            result.settling_window_s=std::max(result.settling_window_s,.4+3.5*gains.kp[i]/compensation.integral_gain[i]);
        }
        result.duration_s=result.settling_window_s+1.3;gain_require_trial(result.duration_s+.5);
        const auto label=std::string(compliant?"compliant":"rigid")+(compensation.enabled?"-integral-":"-baseline-")+std::to_string(++gain_hold_serial_);
        result.record_path="gain-hold-"+label+".csv";
        const auto mode=compliant?JointImpedanceMode::COMPLIANT_HOLD:JointImpedanceMode::RIGID_HOLD;
        Robot::TimePoint started;
        {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            result.entry_actual=robot_.get_joint_state().pos;
            if(!runtime_.set_impedance_mode(robot_,mode,Robot::Clock::now(),HoldTargetPolicy::PRESERVE_NOMINAL) || !robot_.set_execution_gain_profile(compliant?JointGainProfile::COMPLIANT:JointGainProfile::RIGID) ||
               !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) || !robot_.set_calibration_tracking_gains(gains,mode) || !robot_.set_hold_compensation_cfg(compensation))throw std::runtime_error("cannot select hold verification controls");
            robot_.set_admittance_suspended(false);result.target=robot_.get_hold_target();
            if(serial_arm::calibration_recovery::emitted_endpoint_matches(result.target,gain_completed_endpoint_reference_))
                result.target_source="completed_taught_anchor_nominal_reference_not_actual_rezero";
            started=Robot::Clock::now();gain_capture_begin(label,started,true);
        }
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="hold";gain_candidate_index_=0;gain_probe_kp_=gains.kp.front();gain_probe_kd_=gains.kd.front();}
        gain_probe_begin(label,"all",result.duration_s,0,compensation.enabled?"hold_integral_convergence":"hold_verification");
        {std::ostringstream actual;actual<<std::setprecision(17)<<"{\"kind\":\"hold_parameters\",\"record_path\":\""<<result.record_path<<"\",\"full_cycle_evidence\":\"frames.csv\",\"label\":\""<<label<<"\",\"duration_s\":"<<result.duration_s<<",\"settling_window_s\":"<<result.settling_window_s<<",\"integral_enabled\":"<<(compensation.enabled?"true":"false")<<",\"kp\":";machine_json_joint_vector(actual,gains.kp);actual<<",\"kd\":";machine_json_joint_vector(actual,gains.kd);actual<<",\"integral_gain\":";machine_json_joint_vector(actual,compensation.integral_gain);actual<<",\"max_torque\":";machine_json_joint_vector(actual,compensation.max_torque);actual<<",\"max_rate\":";machine_json_joint_vector(actual,compensation.max_rate);actual<<",\"contact_threshold\":";machine_json_joint_vector(actual,compensation.contact_threshold);actual<<",\"velocity_threshold\":"<<compensation.velocity_threshold<<",\"error_threshold\":"<<compensation.error_threshold<<",\"target_source\":\""<<result.target_source<<"\",\"target_pos\":";machine_json_joint_vector(actual,result.target);
            actual<<",\"entry_actual_pos\":";machine_json_joint_vector(actual,result.entry_actual);actual<<'}';gain_experiment_records_.push_back(actual.str());}

        std::uint64_t cursor=0;std::string blocked_integral;
        try {
            while(std::chrono::duration<double>(Robot::Clock::now()-started).count()<result.duration_s) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("continuous hold verification interrupted");
                const double time=std::chrono::duration<double>(Robot::Clock::now()-started).count();gain_probe_update(time,{},time<result.settling_window_s?"hold_convergence":"hold_final_window");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))throw std::runtime_error("hold verification lost feedback");
                cursor=cycle_counter_;
                if(compensation.enabled && time>=1.5 && gain_capture_integral_.size()>=60) {
                    bool all_blocked=true;std::string reason;
                    for(auto it=gain_capture_integral_.end()-60;it!=gain_capture_integral_.end();++it) {
                        if(it->second.size()!=n){all_blocked=false;break;}
                        for(std::size_t i=0;i<n;++i) {
                            const auto& why=it->second[i];
                            if(why=="integrating" || why=="moving" || why=="torque_or_rate_limit" || std::abs(it->first[i])>1e-6)all_blocked=false;
                            else if(reason.empty())reason=why;
                        }
                    }
                    if(all_blocked){blocked_integral="hold_integral_blocked:"+reason;result.duration_s=time;break;}
                }
            }
        } catch(...) {std::lock_guard<std::mutex> lock(mutex_);gain_capture_active_=false;throw;}
        std::vector<std::vector<Sample>> rows;std::vector<std::pair<JointVector,std::vector<std::string>>> integral;std::vector<bool> observer;
        {std::lock_guard<std::mutex> lock(mutex_);gain_capture_active_=false;rows=gain_capture_rows_;integral=gain_capture_integral_;observer=gain_capture_observer_;}
        std::ofstream csv(std::filesystem::path(model_calibration_directory_)/result.record_path);
        if(!csv)throw std::runtime_error("cannot record hold verification");
        csv<<"time_s,joint,reference,actual,velocity,final_window,kp,kd,integral_enabled,integral_gain,max_torque,max_rate,integral_torque,freeze_reason,limited,new_measurement,actual_gains_known\n"<<std::setprecision(17);
        bool complete=true,stationary=true;std::size_t windows=0;
        result.controls_known=result.reference_fixed=result.independent_input_zero=true;
        for(std::size_t axis=0;axis<n;++axis) {
            std::vector<Sample> final;double last_fresh=-1;double lo=1e30,hi=-1e30;const double initial_error=rows[axis].empty()?0:rows[axis].front().actual-result.target[axis];
            for(std::size_t k=0;k<rows[axis].size();++k) {
                const auto& row=rows[axis][k];const double error=row.actual-result.target[axis];result.limited=result.limited || row.limited;
                result.controls_known=result.controls_known && row.actual_gains_known && (row.time_s<result.settling_window_s ||
                    (std::abs(row.actual_kp-gains.kp[axis])<1e-6 && std::abs(row.actual_kd-gains.kd[axis])<1e-6));
                result.reference_fixed=result.reference_fixed && std::abs(row.reference-result.target[axis])<1e-8 && std::abs(row.reference_velocity)<1e-8;
                result.independent_input_zero=result.independent_input_zero && row.independent_disturbance_known && std::abs(row.independent_disturbance_nm)<1e-10;
                result.transient_peak_error=std::max(result.transient_peak_error,std::abs(error));
                result.transient_excursion=std::max(result.transient_excursion,initial_error>=0?std::max(0.0,-error):std::max(0.0,error));
                if(row.new_measurement){if(last_fresh>=0 && row.time_s-last_fresh>.06)complete=false;last_fresh=row.time_s;}
                const bool final_window=row.time_s>=result.settling_window_s;
                const double torque=k<integral.size() && integral[k].first.size()==n?integral[k].first[axis]:0;
                const auto reason=k<integral.size() && integral[k].second.size()==n?integral[k].second[axis]:"unavailable";
                csv<<row.time_s<<','<<cfg_.joint_names[axis]<<','<<result.target[axis]<<','<<row.actual<<','<<row.velocity<<','<<final_window<<','<<row.actual_kp<<','<<row.actual_kd<<','<<compensation.enabled<<','<<(compensation.integral_gain.size()==n?compensation.integral_gain[axis]:0)<<','<<(compensation.max_torque.size()==n?compensation.max_torque[axis]:0)<<','<<(compensation.max_rate.size()==n?compensation.max_rate[axis]:0)<<','<<torque<<','<<reason<<','<<row.limited<<','<<row.new_measurement<<','<<row.actual_gains_known<<'\n';
                if(!final_window)continue;
                final.push_back(row);result.worst_error=std::max(result.worst_error,std::abs(error));
                result.raw_worst_velocity=std::max(result.raw_worst_velocity,std::abs(row.velocity));
                if(row.new_measurement){++result.fresh_samples[axis];result.mean_error[axis]+=error;lo=std::min(lo,row.actual);hi=std::max(hi,row.actual);}
                if(axis==0){++windows;if(k<observer.size() && observer[k])++result.observer_samples;}
            }
            complete=complete && last_fresh>=result.duration_s-.08 && result.fresh_samples[axis]>=30;
            if(result.fresh_samples[axis])result.mean_error[axis]/=result.fresh_samples[axis];
            const double span=final.empty()?0:hi-lo;result.worst_span=std::max(result.worst_span,span);
            const auto noise=axis<gain_noise_.size()?gain_noise_[axis]:NoiseEstimate{};
            const double speed=drag_position_speed(final,noise);result.worst_velocity=std::max(result.worst_velocity,speed);
            stationary=stationary && span<=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,compliant?std::min(.004,.1*gain_requirements_.local_range_rad):.0002});
        }
        for(std::size_t k=0;k<integral.size();++k)if(k<rows.front().size() && rows.front()[k].time_s>=result.settling_window_s && std::any_of(integral[k].first.begin(),integral[k].first.end(),[](double x){return std::abs(x)>1e-6;}))++result.integral_samples;
        result.samples=windows;result.completed=complete && result.controls_known && result.reference_fixed && result.independent_input_zero;result.available=result.completed;
        const double static_error_limit=compliant?gain_requirements_.local_range_rad:gain_requirements_.hold_error_rad;
        const double transient_limit=compliant?gain_requirements_.local_range_rad:gain_requirements_.overshoot_rad;
        result.validated=complete && !result.limited && stationary && result.worst_error<=static_error_limit && result.worst_velocity<=gain_requirements_.hold_velocity_rad_s && result.transient_excursion<=transient_limit && (!compensation.enabled || result.observer_samples>=30);
        result.reason=result.validated?"qualified":!complete?"insufficient_measurements":result.limited?"limited":compensation.enabled && result.observer_samples<30?"observer_unavailable":result.transient_excursion>transient_limit?"hold_dynamic_overshoot":result.worst_error>static_error_limit?"hold_position_error":result.worst_velocity>gain_requirements_.hold_velocity_rad_s?"hold_velocity_exceeded":"hold_not_stationary";
        if(!blocked_integral.empty()){result.reason=blocked_integral;result.completed=false;result.available=false;result.validated=false;}
        csv.flush();if(!csv)throw std::runtime_error("hold recording failed");
        std::ostringstream record;record<<std::setprecision(17)<<"{\"kind\":\"hold\",\"validation_scope\":\""<<(compliant?"compliant_unforced_equilibrium_stability":"rigid_position_hold")<<"\",\"static_error_limit_rad\":"<<static_error_limit<<",\"transient_limit_rad\":"<<transient_limit<<",\"record_path\":\""<<result.record_path<<"\",\"reason\":\""<<result.reason<<"\",\"integral_enabled\":"<<(compensation.enabled?"true":"false")<<",\"settling_window_s\":"<<result.settling_window_s<<",\"final_window_s\":1.3,\"peak_static_error_rad\":"<<result.worst_error<<",\"position_drift_rad\":"<<result.worst_span<<",\"transient_excursion_rad\":"<<result.transient_excursion<<",\"kp\":";machine_json_joint_vector(record,gains.kp);record<<",\"kd\":";machine_json_joint_vector(record,gains.kd);record<<",\"integral_gain\":";machine_json_joint_vector(record,compensation.integral_gain);record<<",\"max_torque\":";machine_json_joint_vector(record,compensation.max_torque);record<<",\"max_rate\":";machine_json_joint_vector(record,compensation.max_rate);record<<'}';gain_experiment_records_.push_back(record.str());
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=record.str();}
        gain_probe_end(result.reason,true);return result;
    }

    std::vector<serial_arm::servo_tuning::RigidUseEvidence> gain_rigid_evidence(const JointImpedanceGains& gains,
        const ServoTrial& disturbance,const GainHoldResult* hold,const std::string& input_signature) const {
        using namespace serial_arm::servo_tuning;const auto n=cfg_.joint_names.size();std::vector<RigidUseEvidence> out(n);
        for(std::size_t i=0;i<n;++i) {
            auto& e=out[i];if(disturbance.compliant_assessments.size()==n)e.disturbance=disturbance.compliant_assessments[i];
            e.measured_kp=gains.kp[i];e.measured_kd=gains.kd[i];e.fixed_gain_measurement_complete=disturbance.completed;
            e.input_signature=input_signature;e.execution_scope="native_MIT_fixed_interior_reference_same_declared_load_and_model_FF_integral_disabled";
            if(disturbance.rows.size()==n)for(const auto& row:disturbance.rows[i])if(row.new_measurement && row.actual_gains_known &&
                std::abs(row.actual_kp-gains.kp[i])<1e-6 && std::abs(row.actual_kd-gains.kd[i])<1e-6 &&
                std::isfinite(row.reference-row.actual) && std::isfinite(row.reference_velocity-row.velocity)) {
                const double error=row.reference-row.actual,velocity_error=row.reference_velocity-row.velocity;
                e.error_envelope_rad=std::max(std::isfinite(e.error_envelope_rad)?e.error_envelope_rad:0,std::abs(error));
                e.velocity_error_envelope_rad_s=std::max(std::isfinite(e.velocity_error_envelope_rad_s)?e.velocity_error_envelope_rad_s:0,std::abs(velocity_error));
                e.fresh_error_rad=error;e.fresh_velocity_error_rad_s=velocity_error;
            }
            if(hold) {
                auto& h=e.hold;h.complete=hold->completed;h.gains_constant=hold->controls_known;
                h.reference_fixed=hold->reference_fixed;h.input_known_zero=hold->independent_input_zero;h.limited=hold->limited;
                h.static_error_rad=hold->mean_error.size()==n?std::abs(hold->mean_error[i]):std::numeric_limits<double>::quiet_NaN();
                h.reference_position_rad=hold->target.size()==n?hold->target[i]:std::numeric_limits<double>::quiet_NaN();
                h.drift_rad=hold->worst_span;h.speed_rad_s=hold->worst_velocity;
                h.motion_floor_rad=std::max({3*gain_noise_[i].encoder_step_rad,6*gain_noise_[i].rms_rad,1e-6});
            }
        }
        return out;
    }

    struct DragPulseWindow {
        std::size_t axis{0};serial_arm::servo_tuning::PurposePulsePlan plan;
        double start_s{0},end_s{0};bool domain_stopped{false};
    };
    struct DragTrial {
        std::vector<DragPulseWindow> pulse_windows;
        JointVector center,decay_s,initial_velocity,peak_velocity,raw_peak_velocity,final_velocity;
        std::vector<serial_arm::servo_tuning::PurposePulsePlan> pulse_plans;
        JointVector pulse_start_s,pulse_release_s,pulse_end_s;
        std::vector<bool> pulse_executed,pulse_domain_stopped;
        std::vector<std::vector<serial_arm::servo_tuning::Sample>> rows;
        double duration_s{0},actual_zero_stiffness_ready_s{std::numeric_limits<double>::quiet_NaN()};
        double quiet_start_s{std::numeric_limits<double>::quiet_NaN()},quiet_end_s{std::numeric_limits<double>::quiet_NaN()};
        std::vector<std::size_t> measurements;
        std::vector<bool> continuous,limited,axis_passed,initial_measured;
        std::vector<serial_arm::servo_tuning::DragStationaryAssessment> stationary;
        std::vector<serial_arm::servo_tuning::DragStationaryAssessment> final_stationary;
        std::vector<serial_arm::servo_tuning::DragReleaseAssessment> release;
        std::vector<std::string> release_record_paths;
        std::optional<std::size_t> demonstrated_release_index;
        std::vector<std::string> reasons;
        bool validated{false},completed{false},available{false},released{false};std::size_t samples{0};
        std::string record_path,error;
        explicit DragTrial(std::size_t n=0):decay_s(n,0),initial_velocity(n,0),peak_velocity(n,0),raw_peak_velocity(n,0),final_velocity(n,0),
            pulse_plans(n),pulse_start_s(n,std::numeric_limits<double>::quiet_NaN()),pulse_release_s(n,std::numeric_limits<double>::quiet_NaN()),
            pulse_end_s(n,std::numeric_limits<double>::quiet_NaN()),pulse_executed(n,false),pulse_domain_stopped(n,false),rows(n),
            measurements(n,0),continuous(n,true),limited(n,false),axis_passed(n,false),initial_measured(n,false),stationary(n),final_stationary(n),release(n),release_record_paths(n),reasons(n,"not_started"){}
    };
    void gain_drag_axes_json(std::ostream& out,const DragTrial& trial) const {
        out<<'[';
        for(std::size_t axis=0;axis<trial.decay_s.size();++axis) {
            if(axis)out<<',';
            out<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"initial_velocity_rad_s\":";
            if(trial.initial_measured[axis])out<<trial.initial_velocity[axis];else out<<"null";
            out<<",\"peak_velocity_rad_s\":";if(trial.measurements[axis])out<<trial.peak_velocity[axis];else out<<"null";
            out<<",\"raw_peak_velocity_rad_s\":";if(trial.measurements[axis])out<<trial.raw_peak_velocity[axis];else out<<"null";
            out<<",\"final_velocity_rad_s\":";if(trial.measurements[axis])out<<trial.final_velocity[axis];else out<<"null";
            out<<",\"decay_s\":";if(trial.release[axis].measurement_complete)out<<trial.decay_s[axis];else out<<"null";
            out<<",\"samples\":"<<trial.measurements[axis]<<",\"continuous\":"<<(trial.continuous[axis]?"true":"false")
                <<",\"limited\":"<<(trial.limited[axis]?"true":"false")<<",\"passed\":"<<(trial.axis_passed[axis]?"true":"false")
                <<",\"quiet_stability_measured\":"<<(trial.stationary[axis].measurement_complete?"true":"false")<<",\"quiet_stability_passed\":"<<(trial.stationary[axis].passed?"true":"false")
                <<",\"feedback_clock_aligned\":"<<(trial.release[axis].feedback_clock_aligned?"true":"false")
                <<",\"release_timing_known\":"<<(trial.release[axis].release_timing_known?"true":"false")
                <<",\"speed_comparison_valid\":"<<(trial.release[axis].speed_comparison_valid?"true":"false")
                <<",\"release_speed_window_s\":"<<trial.release[axis].release_speed_window_s
                <<",\"release_decay_measured\":"<<(trial.release[axis].measurement_complete?"true":"false")<<",\"release_decay_passed\":"<<(trial.release[axis].passed?"true":"false")
                <<",\"release_record_path\":\""<<machine_json_escape(trial.release_record_paths[axis].empty()?trial.record_path:trial.release_record_paths[axis])<<"\""
                <<",\"demonstrated_motion_preparation\":"<<(trial.release[axis].demonstrated_motion_preparation?"true":"false")
                <<",\"preparation_motion_observed\":"<<(trial.release[axis].preparation_motion_observed?"true":"false")
                <<",\"release_motion_observed\":"<<(trial.release[axis].release_motion_observed?"true":"false")
                <<",\"fast_stop_observed\":"<<(trial.release[axis].fast_stop_observed?"true":"false")
                <<",\"decay_time_is_upper_bound\":"<<(trial.release[axis].decay_time_is_upper_bound?"true":"false")
                <<",\"decay_time_upper_bound_s\":"<<(std::isfinite(trial.release[axis].decay_time_upper_bound_s)?std::to_string(trial.release[axis].decay_time_upper_bound_s):"null")
                <<",\"quiet_position_span_rad\":"<<trial.stationary[axis].position_span_rad<<",\"quiet_speed_rad_s\":"<<trial.stationary[axis].speed_rad_s
                <<",\"validation_scope\":\""<<(trial.release[axis].measurement_complete?"zero_stiffness_quiet_stability_and_motion_release_decay":"zero_stiffness_quiet_stability_only_release_unmeasured")<<"\""
                <<",\"peak_velocity_limit_rad_s\":"<<std::max(.025,1.4*trial.initial_velocity[axis])
                <<",\"reason\":\""<<machine_json_escape(trial.reasons[axis])<<"\"}";
        }
        out<<']';
    }
    void gain_drag_diagnostic(const DragTrial& trial,const JointImpedanceGains& gains) {
        if(trial.decay_s.empty())return;
        std::size_t worst=0;bool located=trial.completed;
        for(std::size_t axis=0;axis<trial.decay_s.size();++axis) {
            if(!trial.continuous[axis] || trial.limited[axis] || trial.reasons[axis]=="no_motion"){worst=axis;located=true;break;}
            if(trial.completed && !trial.axis_passed[axis]){worst=axis;break;}
            if(trial.decay_s[axis]>trial.decay_s[worst])worst=axis;
        }
        std::ostringstream out;out<<std::setprecision(17);
        out<<"{\"completed\":"<<(trial.completed?"true":"false")<<",\"available\":"<<(trial.available?"true":"false")<<",\"stage\":\"drag\",\"joint\":\""<<(located?machine_json_escape(cfg_.joint_names[worst]):"all")
            <<"\",\"kp\":"<<gains.kp[worst]<<",\"kd\":"<<gains.kd[worst]<<",\"metrics_passed\":"<<(trial.validated?"true":"false")
            <<",\"velocity_source\":\"position_window_0.6s\",\"raw_velocity_role\":\"diagnostic_only_core_safety_unchanged\",\"record_path\":\""<<machine_json_escape(trial.record_path)
            <<"\",\"reason\":\""<<(trial.completed?(trial.validated?"qualified":trial.reasons[worst]):trial.error.empty()?"measurement_pending":"measurement_interrupted")<<"\"";
        if(!trial.error.empty())out<<",\"error\":\""<<machine_json_escape(trial.error)<<'"';
        out<<",\"metrics\":{\"initial_velocity_rad_s\":";
        if(trial.initial_measured[worst])out<<trial.initial_velocity[worst];else out<<"null";
        out<<",\"peak_velocity_rad_s\":";if(trial.measurements[worst])out<<trial.peak_velocity[worst];else out<<"null";
        out<<",\"raw_peak_velocity_rad_s\":";if(trial.measurements[worst])out<<trial.raw_peak_velocity[worst];else out<<"null";
        out<<",\"final_velocity_rad_s\":";if(trial.measurements[worst])out<<trial.final_velocity[worst];else out<<"null";
        out<<",\"decay_s\":";if(trial.measurements[worst])out<<trial.decay_s[worst];else out<<"null";
        out<<"},\"requirements\":{\"initial_velocity_rad_s\":0.003,\"peak_velocity_rad_s\":"<<std::max(.025,1.4*trial.initial_velocity[worst])
            <<",\"final_velocity_rad_s\":0.003,\"decay_s\":1,\"minimum_samples\":30,\"maximum_feedback_gap_s\":0.06},\"axes\":";
        gain_drag_axes_json(out,trial);out<<'}';
        std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=out.str();
    }
    serial_arm::servo_tuning::PurposeConditionComparison gain_drag_comparison_condition(
        const DragTrial& trial,const JointVector& actual) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();Dynamics model;
        if(!model.configure(cfg_.dynamics))return {false,0,0,"purpose_comparison_model_unavailable"};
        JointVector zero(n,0),before_mass,after_mass,before_gravity(n,0),after_gravity(n,0),amplitude(n),quantum(n);
        ModelFeedforwardMode ff;{std::lock_guard<std::mutex> lock(mutex_);ff=robot_.get_model_feedforward_mode();}
        const auto sample=[&](const JointVector& pose,JointVector& mass,JointVector& gravity) {
            if(!model.update(JointState{pose,zero,zero},zero,zero))return false;
            const auto& M=model.get_mass_matrix();if(M.rows()!=static_cast<Eigen::Index>(n))return false;
            for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)mass.push_back(M(i,j));
            if(ff!=ModelFeedforwardMode::NONE)gravity=model.get_gravity_compensation();
            return true;
        };
        if(!sample(trial.center,before_mass,before_gravity) || !sample(actual,after_mass,after_gravity))
            return {false,0,0,"purpose_comparison_model_unavailable"};
        for(std::size_t i=0;i<n;++i) {amplitude[i]=trial.pulse_plans[i].amplitude_nm;
            if(i<actuator_info_.size())quantum[i]=actuator_info_[i].effort_resolution*std::abs(cfg_.mapper.tor_ratio[i]);}
        return purpose_comparison_condition(before_mass,after_mass,before_gravity,after_gravity,amplitude,quantum);
    }

    // Reuse demonstrated motion when a bounded P=0 pulse cannot establish
    // motion against static resistance. The preparation is a known PD path;
    // decay starts only at the first actually written P=0, constant-D command.
    // No pulse cap or joint Safety limit is enlarged to overcome stiction.
    void gain_drag_taught_release(DragTrial& trial,const JointImpedanceGains& gains,
        const JointImpedanceGains& preparation_gains,double reserved_s,const std::string& label,
        const DragTrial* fixed_preparation=nullptr) {
        using namespace serial_arm::servo_tuning;
        const auto& path=*model_calibration_trajectory_;const auto n=cfg_.joint_names.size();
        const auto operation_started=Robot::Clock::now();
        const auto needs_measurement=[&](std::size_t axis){return fixed_preparation?
            !fixed_preparation->release_record_paths[axis].empty():!trial.release[axis].measurement_complete;};
        bool needed=false;for(std::size_t i=0;i<n;++i)needed=needed || needs_measurement(i);
        if(!needed)return;
        const auto fixed_release_index=fixed_preparation?fixed_preparation->demonstrated_release_index.value_or(path.positions.size()):path.positions.size();
        const auto nominal=gain_purpose_nominal_reference();
        std::size_t anchor=0;double distance=std::numeric_limits<double>::infinity();
        for(std::size_t k=0;k<path.positions.size();++k){double d=0;for(std::size_t i=0;i<n;++i)d=std::max(d,std::abs(path.positions[k][i]-nominal[i]));if(d<distance){distance=d;anchor=k;}}
        if(distance>1e-6)return;
        const auto radius=std::max<std::size_t>(2,static_cast<std::size_t>(2/path.sample_dt));
        const auto low=anchor>radius?anchor-radius:0,high=std::min(path.positions.size()-2,anchor+radius);
        std::size_t release=anchor;double score=0;
        for(std::size_t k=std::max<std::size_t>(1,low);k<=high;++k) {
            if(fixed_release_index<path.positions.size() && k!=fixed_release_index)continue;
            const auto ref=serial_arm::calibration_retime::interpolate(path.positions,k,path.sample_dt,1);
            double value=0;for(std::size_t i=0;i<n;++i)if(needs_measurement(i))
                value+=std::abs(ref.velocity[i])/std::max({.003,6*gain_noise_[i].rms_rad,3*gain_noise_[i].encoder_step_rad});
            if(value>score){score=value;release=k;}
        }
        if(score<=1 || release==0)return;
        const auto from=release>radius?release-radius:0;
        std::vector<JointVector> segment(path.positions.begin()+from,path.positions.begin()+release+2);
        const double source_s=(release-from)*path.sample_dt,ramp_s=std::min(.5,source_s);
        const double prepare_s=source_s+.5*ramp_s,release_window=DragUseRequirements{}.release_window_s+.15;
        const auto entry=plan_taught_anchor_visit(path.positions,path.sample_dt,anchor,from,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc);
        const auto exit=plan_taught_anchor_visit(path.positions,path.sample_dt,release,anchor,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc);
        if(!entry.valid || !exit.valid || !gain_trial_fits(entry.duration_s+prepare_s+1.3+release_window+exit.duration_s,reserved_s+2))return;
        const auto reference_at=[&](double t) {
            const double a=std::clamp(t/ramp_s,0.0,1.0);
            const double rate=t<ramp_s?.5*(1-std::cos(3.141592653589793*a)):1;
            const double source=t<ramp_s?.5*(t-ramp_s/3.141592653589793*std::sin(3.141592653589793*a)):t-.5*ramp_s;
            return serial_arm::calibration_retime::interpolate(segment,std::min(source,source_s)/path.sample_dt,path.sample_dt,rate);
        };
        JointVector previous;for(double t=0;t<prepare_s;t+=.005) {
            const auto ref=reference_at(t);for(std::size_t i=0;i<n;++i)
                if(std::abs(ref.velocity[i])>cfg_.safety.limits.max_vel[i] || (!previous.empty() && std::abs(ref.velocity[i]-previous[i])/.005>cfg_.safety.limits.max_acc[i]))return;
            previous=ref.velocity;
        }
        gain_visit_measurement_anchor(preparation_gains,anchor,from,prepare_s+release_window+exit.duration_s+reserved_s+2,label+"-release-entry");
        const auto started=Robot::Clock::now();const auto elapsed=[&]{return std::chrono::duration<double>(Robot::Clock::now()-started).count();};
        const std::string record="drag-taught-release-"+label+".csv";
        std::vector<std::vector<Sample>> rows;bool requested=false,domain_stopped=false,final_reference_sent=false;double ready=std::numeric_limits<double>::quiet_NaN();
        const auto finish=[&] {
            std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_={};gain_capture_active_=false;rows=std::move(gain_capture_rows_);
            robot_.clear_calibration_disturbance();gain_online_input_plan_.clear();
            if(robot_.get_state()==RobotState::ACTIVE){gain_remember_completed_endpoint_locked(path.positions[release],record);gain_hold_for_assessment_locked(preparation_gains,true);}
        };
        try {
            {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_TRACKING) || !robot_.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING) ||
                   !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) || !robot_.set_calibration_tracking_gains(preparation_gains,JointImpedanceMode::RIGID_TRACKING))
                    throw std::runtime_error("cannot_prepare_demonstrated_drag_release");
                robot_.clear_calibration_disturbance();robot_.set_admittance_suspended(true);gain_online_input_plan_.assign(n,0);
                gain_capture_begin(label+"-taught-release",started,true);gain_cycle_reference_error_=nullptr;
                gain_cycle_reference_until_=gain_deadline_.value_or(started+std::chrono::seconds(1000));
                gain_cycle_reference_=[&,this](Robot::TimePoint now) {
                    const double t=std::chrono::duration<double>(now-started).count();
                    if(t<prepare_s || !final_reference_sent) {
                        const auto ref=reference_at(std::min(t,prepare_s));const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{ref.position,ref.velocity},now,MotionSource::CALIBRATION);
                        if(!sent)throw machine_robot_error("demonstrated drag preparation",sent.error());
                        // Commit the final taught knot for one actual control
                        // cycle before changing mode clears pending commands.
                        // The remembered continuation is an emitted reference,
                        // never a future knot that was only planned.
                        if(t>=prepare_s)final_reference_sent=true;
                    } else if(!requested) {
                        bool room=last_output_ && last_output_->joint_state.pos.size()==n && last_output_->joint_state.vel.size()==n;
                        for(std::size_t i=0;room && i<n;++i) {
                            const double q=last_output_->joint_state.pos[i],v=last_output_->joint_state.vel[i];
                            const double floor=std::max({3*gain_noise_[i].encoder_step_rad,6*gain_noise_[i].rms_rad,1e-5});
                            const double available=v>=0?gain_demonstrated_path_upper_[i]-q:q-gain_demonstrated_path_lower_[i];
                            const double coast=std::abs(v)*(gains.kd[i]>0?std::min(release_window,gain_model_initialization_.inertia_max[i]/gains.kd[i]):release_window);
                            room=std::isfinite(coast) && available>=coast+floor;
                        }
                        if(!room) {
                            domain_stopped=true;requested=true;
                            if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_HOLD,now,HoldTargetPolicy::CAPTURE_MEASURED))
                                throw std::runtime_error("cannot_hold_demonstrated_release_without_coasting_room");
                            return;
                        }
                        if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::COMPLIANT_DRAG) || !robot_.set_execution_gain_profile(JointGainProfile::DRAG) ||
                           !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::COMPLIANT_DRAG))throw std::runtime_error("cannot_release_demonstrated_motion_to_actual_drag");
                        robot_.clear_calibration_disturbance();requested=true;
                    }
                };
            }
            {std::lock_guard<std::mutex> status(model_calibration_mutex_);gain_probe_stage_="drag";}
            gain_probe_begin(label+"-taught-release","all",prepare_s+1.3+release_window,0,"drag_demonstrated_motion_release");
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("demonstrated_drag_release_interrupted");
                const auto t=elapsed();
                if(requested && last_output_)for(std::size_t i=0;i<n;++i) {
                    const double tolerance=std::max({3*gain_noise_[i].encoder_step_rad,6*gain_noise_[i].rms_rad,1e-5});
                    const auto q=last_output_->joint_state.pos[i];
                    if(q<gain_demonstrated_path_lower_[i]-tolerance || q>gain_demonstrated_path_upper_[i]+tolerance)domain_stopped=true;
                }
                if(requested && !std::isfinite(ready) && last_output_) {
                    bool exact=last_output_->joint_cmd.kp.size()==n && last_output_->joint_cmd.kd.size()==n;
                    for(std::size_t i=0;exact && i<n;++i)exact=std::abs(last_output_->joint_cmd.kp[i])<1e-10 && std::abs(last_output_->joint_cmd.kd[i]-gains.kd[i])<1e-6;
                    if(exact)ready=t;
                }
                gain_probe_update(t,{},requested?"drag_release":"drag_demonstrated_preparation");
                if(domain_stopped || (std::isfinite(ready) && t>=ready+release_window) || t>prepare_s+1.3+release_window)break;
                const auto cursor=cycle_counter_;if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))throw std::runtime_error("demonstrated_drag_release_feedback_unavailable");
            }
            finish();gain_write_response_csv(record,rows);
        } catch(...) {const auto failure=std::current_exception();finish();gain_write_response_csv(record,rows);std::rethrow_exception(failure);}
        std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"drag_demonstrated_motion_release\",\"record_path\":\""<<record<<"\",\"source_index_from\":"<<from<<",\"source_index_release\":"<<release
            <<",\"preparation_duration_s\":"<<prepare_s<<",\"actual_zero_stiffness_s\":"<<gain_finite_json(ready)<<",\"elapsed_s\":"<<elapsed()<<",\"domain_stopped\":"<<(domain_stopped?"true":"false")<<",\"reference_only\":true,\"axes\":[";
        trial.demonstrated_release_index=release;
        for(std::size_t i=0;i<n;++i) {
            double actual_release=std::numeric_limits<double>::quiet_NaN();
            for(const auto& row:rows[i])if(row.time_s>=prepare_s && row.actual_gains_known && std::abs(row.actual_kp)<1e-10 && std::abs(row.actual_kd-gains.kd[i])<1e-6 &&
                std::abs(row.reference_velocity)<1e-8 && row.independent_disturbance_known && std::abs(row.independent_disturbance_nm)<1e-10){actual_release=row.time_s;break;}
            auto measured=assess_drag_release(rows[i],gain_noise_[i],actual_release,{},true);
            if(domain_stopped){measured.measurement_complete=false;measured.passed=false;measured.reason="drag_demonstrated_release_stopped_at_measured_domain";}
            if(needs_measurement(i) && (measured.measurement_complete || fixed_preparation)) {
                trial.release[i]=measured;trial.release_record_paths[i]=record;trial.initial_measured[i]=std::isfinite(actual_release);
                trial.initial_velocity[i]=measured.initial_speed_rad_s;trial.peak_velocity[i]=measured.peak_speed_rad_s;trial.decay_s[i]=measured.decay_s;
                trial.raw_peak_velocity[i]=std::max(trial.raw_peak_velocity[i],measured.raw_peak_velocity_rad_s);trial.reasons[i]=measured.reason;
                trial.axis_passed[i]=trial.stationary[i].passed && measured.passed;
            }
            if(i)note<<',';
            note<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"measurement_complete\":"<<(measured.measurement_complete?"true":"false")<<",\"reason\":\""<<machine_json_escape(measured.reason)<<"\"}";
        }
        note<<"]}";gain_experiment_records_.push_back(note.str());
        gain_visit_measurement_anchor(preparation_gains,release,anchor,reserved_s+2,label+"-release-return");
        trial.duration_s+=std::chrono::duration<double>(Robot::Clock::now()-operation_started).count();
        trial.released=trial.released || std::any_of(trial.release.begin(),trial.release.end(),[](const auto& axis){return axis.measurement_complete;});
        trial.validated=std::all_of(trial.axis_passed.begin(),trial.axis_passed.end(),[](bool passed){return passed;});
    }

    DragTrial gain_drag_trial(JointImpedanceGains gains,JointImpedanceGains excitation,
                              const std::string& label,DragTrial* observation=nullptr,double reserved_s=0,
                              const std::vector<serial_arm::servo_tuning::PurposePulsePlan>* fixed_plans=nullptr,
                              const ServoTrial* trajectory_evidence=nullptr) {
        using namespace serial_arm::servo_tuning;
        gains=gain_protocol_gains(gains);excitation=gain_protocol_gains(excitation);
        const auto n=cfg_.joint_names.size();DragTrial result(n);
        result.record_path=(std::filesystem::path(model_calibration_directory_)/("drag-decay-"+label+".csv")).string();
        if(fixed_plans && fixed_plans->size()!=n)throw std::invalid_argument("drag pulse plan mapping mismatch");
        if(std::any_of(gains.kp.begin(),gains.kp.end(),[](double p){return p!=0;}))throw std::invalid_argument("drag trial requires an actual zero-stiffness group");
        JointVector center,lower(n),upper(n);std::optional<std::size_t> active_axis;
        double active_start=0;bool capture_started=false;
        const auto started=Robot::Clock::now();
        const auto elapsed=[&]{return std::chrono::duration<double>(Robot::Clock::now()-started).count();};
        const auto remaining=[&]{return gain_deadline_?std::max(0.0,std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count()):1000.0;};
        const auto write_evidence=[&] {
            std::ofstream csv(result.record_path);if(!csv)throw std::runtime_error("cannot record zero-stiffness purpose evidence");
            csv<<"time_s,joint,reference,reference_velocity,actual,velocity,residual,residual_valid,limited,new_measurement,feedforward_nm,independent_disturbance_nm,command_time_s,feedback_time_s,feedback_sequence,actual_kp,actual_kd,actual_gains_known,reference_sample_time_s,reference_sample_time_known,measured_torque_nm,planned_independent_disturbance_nm,independent_input_planned_known,phase,active_axis,released,actual_zero_stiffness\n"<<std::setprecision(17);
            for(std::size_t axis=0;axis<n;++axis)for(const auto& row:result.rows[axis]) {
                std::string phase="handover";std::size_t selected=n;bool released=false;
                const double schedule_time=row.reference_sample_time_known?row.reference_sample_time_s:row.command_time_s;
                if(std::isfinite(result.actual_zero_stiffness_ready_s) && row.time_s>=result.actual_zero_stiffness_ready_s)phase="quiet";
                for(const auto& window:result.pulse_windows)if(schedule_time>=window.start_s && schedule_time<=window.end_s+.01) {
                    const auto k=window.axis;selected=k;const auto& plan=window.plan;const double t=schedule_time-window.start_s;
                    phase=t<plan.baseline_s?"quiet":t<plan.release_s?"input":"release";
                    released=axis==k && t>=plan.release_s && row.independent_disturbance_known && std::abs(row.independent_disturbance_nm)<1e-10;break;
                }
                csv<<row.time_s<<','<<cfg_.joint_names[axis]<<','<<row.reference<<','<<row.reference_velocity<<','<<row.actual<<','<<row.velocity<<','<<row.residual<<','<<row.residual_valid<<','<<row.limited<<','<<row.new_measurement<<','<<row.applied_feedforward_nm<<','<<row.independent_disturbance_nm<<','<<row.command_time_s<<','<<row.feedback_time_s<<','<<row.feedback_sequence<<','<<row.actual_kp<<','<<row.actual_kd<<','<<row.actual_gains_known<<','<<row.reference_sample_time_s<<','<<row.reference_sample_time_known<<','<<row.measured_torque_nm<<','<<row.planned_independent_disturbance_nm<<','<<row.independent_input_planned_known<<','<<phase<<',';
                if(selected<n)csv<<cfg_.joint_names[selected];
                csv<<','<<released<<','<<(row.actual_gains_known && std::abs(row.actual_kp)<1e-10 && std::abs(row.actual_kd-gains.kd[axis])<1e-6)<<'\n';
            }
            csv.flush();if(!csv)throw std::runtime_error("zero-stiffness purpose evidence writing failed");
        };
        const auto finish_capture=[&] {
            std::lock_guard<std::mutex> lock(mutex_);active_axis.reset();gain_cycle_reference_={};gain_capture_active_=false;
            robot_.clear_calibration_disturbance();gain_online_input_plan_.clear();
            if(capture_started && gain_capture_rows_.size()==n){result.rows=std::move(gain_capture_rows_);capture_started=false;}
            result.duration_s=elapsed();
            if(robot_.get_state()==RobotState::ACTIVE)gain_hold_for_assessment_locked(excitation,true);
        };
        const auto assess_evidence=[&] {
            result.available=true;result.validated=true;result.samples=0;
            for(std::size_t axis=0;axis<n;++axis) {
                const auto& rows=result.rows[axis];std::vector<Sample> quiet;
                if(std::isfinite(result.quiet_start_s) && std::isfinite(result.quiet_end_s)) {
                    for(const auto& row:rows)if(row.time_s>=result.quiet_start_s && row.time_s<=result.quiet_end_s+.01)quiet.push_back(row);
                }
                result.stationary[axis]=assess_drag_stationarity(quiet,gain_noise_[axis]);
                std::vector<Sample> final_quiet;
                if(!rows.empty() && std::isfinite(result.actual_zero_stiffness_ready_s))for(const auto& row:rows)
                    if(row.time_s>=std::max(result.actual_zero_stiffness_ready_s,rows.back().time_s-.7))final_quiet.push_back(row);
                result.final_stationary[axis]=assess_drag_stationarity(final_quiet,gain_noise_[axis]);
                auto& release=result.release[axis];
                if(result.pulse_executed[axis]) {
                    const auto& plan=result.pulse_plans[axis];std::vector<Sample> measured;
                    for(const auto& original:rows)if(original.time_s>=result.pulse_start_s[axis] && original.time_s<=result.pulse_end_s[axis]+.01) {
                        auto row=original;row.time_s-=result.pulse_start_s[axis];row.command_time_s-=result.pulse_start_s[axis];row.feedback_time_s-=result.pulse_start_s[axis];row.reference_sample_time_s-=result.pulse_start_s[axis];measured.push_back(row);
                    }
                    double actual_release=std::numeric_limits<double>::quiet_NaN();
                    for(const auto& row:measured)if((row.reference_sample_time_known?row.reference_sample_time_s:row.command_time_s)>=plan.release_s && row.actual_gains_known &&
                        std::abs(row.actual_kp)<1e-10 && std::abs(row.actual_kd-gains.kd[axis])<1e-6 &&
                        row.independent_disturbance_known && std::abs(row.independent_disturbance_nm)<1e-10){actual_release=row.time_s;break;}
                    if(std::isfinite(actual_release)) {
                        result.pulse_release_s[axis]=result.pulse_start_s[axis]+actual_release;
                        release=assess_drag_release(measured,gain_noise_[axis],actual_release);
                        if(result.pulse_domain_stopped[axis]){release.measurement_complete=false;release.passed=false;release.reason="drag_pulse_stopped_at_measured_domain_margin";}
                    } else release.reason="drag_actual_zero_input_release_not_measured";
                    result.initial_velocity[axis]=release.initial_speed_rad_s;result.initial_measured[axis]=std::isfinite(actual_release);
                    result.peak_velocity[axis]=release.peak_speed_rad_s;result.decay_s[axis]=release.decay_s;
                } else release.reason=result.reasons[axis]=="not_started"?"drag_release_not_requested_without_complete_quiet_window":result.reasons[axis];
                const auto& still=result.stationary[axis];const auto& final=result.final_stationary[axis];
                result.final_velocity[axis]=final.measurement_complete?final.speed_rad_s:still.speed_rad_s;
                result.raw_peak_velocity[axis]=std::max(still.raw_peak_velocity_rad_s,release.raw_peak_velocity_rad_s);
                result.measurements[axis]=still.fresh_samples+release.fresh_samples;result.samples=std::max(result.samples,result.measurements[axis]);
                result.continuous[axis]=still.measurement_complete;result.limited[axis]=still.limited || release.limited;
                result.axis_passed[axis]=still.passed && (!release.motion_observed || release.passed) &&
                    (!final.measurement_complete || final.passed);
                result.reasons[axis]=!still.passed?still.reason:release.measurement_complete?release.reason:
                    release.reason;
                if(final.measurement_complete && !final.passed)result.reasons[axis]=final.reason;
                result.available=result.available && still.measurement_complete && !still.limited;
                result.validated=result.validated && result.axis_passed[axis];
            }
        };
        try {
            gain_require_trial(2.0+reserved_s);
            {
                std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,JointImpedanceMode::COMPLIANT_DRAG) ||
                   !robot_.set_execution_gain_profile(JointGainProfile::DRAG) || !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
                   !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::COMPLIANT_DRAG))throw std::runtime_error("cannot select actual zero-stiffness drag purpose");
                robot_.set_admittance_suspended(true);center=robot_.get_joint_state().pos;result.center=center;
                for(std::size_t axis=0;axis<n;++axis) {
                    auto bound=gain_model_limits_[axis];gain_apply_purpose_range(bound,axis,center[axis],center[axis]);
                    lower[axis]=center[axis]+bound.reference_min_offset_rad;upper[axis]=center[axis]+bound.reference_max_offset_rad;
                }
                gain_capture_begin("drag-"+label,started,true);capture_started=true;gain_online_input_plan_.assign(n,0);
                gain_cycle_reference_error_=nullptr;gain_cycle_reference_until_=gain_deadline_.value_or(started+std::chrono::seconds(1000));
                gain_cycle_reference_=[&,this](Robot::TimePoint now) {
                    JointVector torque(n,0);const double time=std::chrono::duration<double>(now-started).count();
                    if(active_axis) {
                        const auto axis=*active_axis;bool room=true;
                        if(last_output_ && last_output_->joint_state.pos.size()==n)for(std::size_t k=0;k<n;++k) {
                            const double floor=std::max({3*gain_noise_[k].encoder_step_rad,6*gain_noise_[k].rms_rad,1e-5});
                            const double q=last_output_->joint_state.pos[k];
                            const bool outside=q<lower[k]-floor || q>upper[k]+floor;
                            const bool toward_edge=k==axis && (result.pulse_plans[axis].direction>0?q>upper[k]-floor:q<lower[k]+floor);
                            if(outside || toward_edge)room=false;
                        }
                        if(!room)result.pulse_domain_stopped[axis]=true;
                        if(!result.pulse_domain_stopped[axis])torque[axis]=purpose_pulse_input(time-active_start,result.pulse_plans[axis]);
                    }
                    gain_online_input_plan_=torque;const auto set=robot_.set_calibration_disturbance(torque,now);
                    if(!set)throw machine_robot_error("zero-stiffness cycle sampled pulse",set.error());
                };
            }
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="drag";gain_probe_frequency_hz_={0,0,0};gain_probe_kp_=0;gain_probe_kd_=gains.kd.front();}
            gain_probe_begin(label,"all",2.0,0,"drag_actual_zero_stiffness_handover");
            std::uint64_t cursor=0;
            const auto wait_cycle=[&](std::unique_lock<std::mutex>& lock) {
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("zero-stiffness drag purpose interrupted");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))throw std::runtime_error("zero-stiffness drag purpose lost control feedback");
                cursor=cycle_counter_;
            };
            // The existing Core handover is 0.3 s. Observe the emitted codes
            // instead of calling its still-stiff moving response a P=0 release.
            while(elapsed()<1.3) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);wait_cycle(lock);
                bool ready=last_output_ && last_output_->joint_cmd.kp.size()==n && last_output_->joint_cmd.kd.size()==n;
                if(ready)for(std::size_t axis=0;axis<n;++axis)ready=ready && std::abs(last_output_->joint_cmd.kp[axis])<1e-10 && std::abs(last_output_->joint_cmd.kd[axis]-gains.kd[axis])<1e-6;
                if(ready){result.actual_zero_stiffness_ready_s=elapsed();break;}
            }
            if(std::isfinite(result.actual_zero_stiffness_ready_s)) {
                const double quiet_end=result.actual_zero_stiffness_ready_s+.7;
                result.quiet_start_s=result.actual_zero_stiffness_ready_s;result.quiet_end_s=quiet_end;
                while(elapsed()<quiet_end){gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);gain_probe_update(elapsed(),{},"drag_quiet");wait_cycle(lock);}
                std::vector<std::vector<Sample>> quiet(n);
                {std::lock_guard<std::mutex> lock(mutex_);for(std::size_t axis=0;axis<n;++axis)for(const auto& row:gain_capture_rows_[axis])if(row.time_s>=result.actual_zero_stiffness_ready_s)quiet[axis].push_back(row);}
                bool quiet_stable=true;for(std::size_t axis=0;axis<n;++axis)quiet_stable=quiet_stable && assess_drag_stationarity(quiet[axis],gain_noise_[axis]).passed;
                std::vector<PurposeInputPrior> priors(n);
                if(trajectory_evidence && trajectory_evidence->rows.size()==n)
                    for(std::size_t axis=0;axis<n;++axis)priors[axis]=estimate_purpose_input_prior(trajectory_evidence->rows[axis],gain_noise_[axis],trajectory_evidence->record_path);
                const auto plan_axis=[&](std::size_t axis,double available,double previous_input=0.0) {
                    ModelAxisLimits bound;double inertia=0;PurposePulseOptions options;options.zero_stiffness=true;
                    options.available_s=available;options.tail_s=DragUseRequirements{}.release_window_s;
                    options.feedback_interval_s=1/std::min(200.0,cfg_.runtime.ctrl_frequency_hz);options.input_prior=priors[axis];options.previous_input_peak_nm=previous_input;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);bound=gain_model_limits_[axis];
                        const double actual=robot_.get_joint_state().pos[axis];gain_apply_purpose_range(bound,axis,actual,actual);
                        const auto& mass=dynamics_.get_mass_matrix();if(mass.rows()==static_cast<Eigen::Index>(n))inertia=mass(axis,axis);
                        if(axis<actuator_info_.size())options.torque_quantum_nm=actuator_info_[axis].effort_resolution*std::abs(cfg_.mapper.tor_ratio[axis]);
                    }
                    return std::make_pair(plan_purpose_pulse(bound,gain_noise_[axis],inertia,0,gains.kd[axis],options),options.torque_quantum_nm);
                };
                std::vector<double> axis_cost(n,0);
                for(std::size_t axis=0;axis<n;++axis) {
                    const auto planned=plan_axis(axis,std::max(0.0,remaining()-reserved_s-2));
                    axis_cost[axis]=planned.first.valid?planned.first.duration_s+.1:0;
                }
                for(std::size_t axis=0;axis<n;++axis) {
                    if(!quiet_stable){result.reasons[axis]="drag_pulse_not_requested_while_zero_stiffness_quiet_goal_unresolved";continue;}
                    double later_s=0;for(std::size_t k=axis+1;k<n;++k)later_s+=axis_cost[k];
                    const auto bounded=plan_axis(axis,std::max(0.0,remaining()-reserved_s-later_s-2));
                    auto plan=fixed_plans?fixed_plans->at(axis):bounded.first;
                    const auto record_plan=[&](const char* kind,const std::string& reason,bool scheduled,double next_input) {
                        std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\""<<kind<<"\",\"record_path\":\""<<machine_json_escape(result.record_path)
                            <<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"reason\":\""<<machine_json_escape(reason)
                            <<"\",\"scheduled\":"<<(scheduled?"true":"false")<<",\"input_nm\":"<<plan.amplitude_nm<<",\"next_input_nm\":"<<next_input
                            <<",\"input_ceiling_nm\":"<<plan.input_ceiling_nm<<",\"torque_quantum_nm\":"<<bounded.second
                            <<",\"prior_valid\":"<<(priors[axis].valid?"true":"false")<<",\"prior_input_nm\":"<<priors[axis].seed_nm
                            <<",\"prior_source\":\""<<machine_json_escape(priors[axis].source)<<"\",\"duration_s\":"<<plan.duration_s<<'}';gain_experiment_records_.push_back(note.str());
                    };
                    bool executable=plan.valid && plan.amplitude_nm<=bounded.first.input_ceiling_nm+1e-12 && plan.amplitude_nm>0 &&
                        gain_trial_fits(plan.duration_s,reserved_s+later_s+2);
                    if(!executable) {
                        result.reasons[axis]=!plan.valid?plan.reason:"drag_pulse_not_started_without_complete_window_and_reserved_exit";
                        result.pulse_plans[axis]=plan;record_plan("drag_input_plan",result.reasons[axis],false,0);continue;
                    }
                    record_plan("drag_input_plan",plan.reason,true,0);
                    for(std::size_t attempt=0;;++attempt) {
                        {
                            std::lock_guard<std::mutex> lock(mutex_);result.pulse_plans[axis]=plan;active_start=elapsed();active_axis=axis;
                            result.pulse_start_s[axis]=active_start;result.pulse_end_s[axis]=active_start+plan.duration_s;result.pulse_executed[axis]=true;
                            result.pulse_domain_stopped[axis]=false;result.pulse_windows.push_back({axis,plan,active_start,active_start+plan.duration_s,false});
                        }
                        gain_probe_end("quiet_drag_window_complete",false);
                        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_kp_=0;gain_probe_kd_=gains.kd[axis];}
                        gain_probe_begin(label+"-pulse-"+std::to_string(axis)+"-"+std::to_string(attempt),cfg_.joint_names[axis],plan.duration_s,0,"drag_input");
                        while(elapsed()<result.pulse_end_s[axis]) {
                            gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);
                            const double local=elapsed()-active_start;gain_probe_update(local,{},local<plan.release_s?"drag_input":"drag_release");wait_cycle(lock);
                        }
                        std::vector<Sample> measured;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);active_axis.reset();robot_.clear_calibration_disturbance();gain_online_input_plan_.assign(n,0);
                            result.pulse_windows.back().domain_stopped=result.pulse_domain_stopped[axis];
                            measured=gain_pulse_rows(gain_capture_rows_[axis],active_start,result.pulse_end_s[axis]);
                        }
                        record_plan("drag_input_window",result.pulse_domain_stopped[axis]?"drag_pulse_stopped_at_measured_domain_margin":"drag_input_and_release_window_completed",true,0);
                        if(fixed_plans || result.pulse_domain_stopped[axis])break;
                        double actual_release=std::numeric_limits<double>::quiet_NaN();
                        for(const auto& row:measured)if((row.reference_sample_time_known?row.reference_sample_time_s:row.command_time_s)>=plan.release_s &&
                            row.actual_gains_known && std::abs(row.actual_kp)<1e-10 && std::abs(row.actual_kd-gains.kd[axis])<1e-6 &&
                            row.independent_disturbance_known && std::abs(row.independent_disturbance_nm)<1e-10){actual_release=row.time_s;break;}
                        if(!std::isfinite(actual_release))break;
                        const auto assessment=assess_drag_release(measured,gain_noise_[axis],actual_release);
                        const auto retry=next_drag_release_input(assessment,plan.amplitude_nm,plan.maximum_sampled_input_ceiling_nm,bounded.second);
                        const auto fresh=plan_axis(axis,std::max(0.0,remaining()-reserved_s-later_s-2),retry.next_input_nm);
                        const bool fits=retry.retry && fresh.first.valid && fresh.first.amplitude_nm>plan.amplitude_nm+.5*bounded.second &&
                            gain_trial_fits(fresh.first.duration_s,reserved_s+later_s+2);
                        record_plan("drag_input_supplement_decision",!retry.retry?retry.reason:!fresh.first.valid?fresh.first.reason:
                            fresh.first.amplitude_nm<=plan.amplitude_nm+.5*bounded.second?"drag_input_at_impulse_and_protocol_domain_ceiling":retry.reason,fits,fresh.first.amplitude_nm);
                        if(!fits)break;
                        plan=fresh.first;
                    }
                }
            } else for(auto& reason:result.reasons)reason="drag_actual_zero_stiffness_not_observed_before_bounded_handover_window";
            finish_capture();result.completed=true;assess_evidence();write_evidence();
            result.released=std::any_of(result.pulse_release_s.begin(),result.pulse_release_s.end(),[](double x){return std::isfinite(x);});
            std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"drag_excitation_parameters\",\"label\":\""<<machine_json_escape(label)
                <<"\",\"source\":\"actual_zero_stiffness_then_quiet_then_isolated_known_torque_release\",\"record_path\":\""<<result.record_path<<"\",\"duration_s\":"<<result.duration_s
                <<",\"actual_gains_ready_s\":";
            const auto number=[&](double x){if(std::isfinite(x))note<<x;else note<<"null";};
            number(result.actual_zero_stiffness_ready_s);note<<",\"quiet_start_s\":";number(result.quiet_start_s);note<<",\"quiet_end_s\":";number(result.quiet_end_s);
            note<<",\"center_rad\":";machine_json_joint_vector(note,center);
            note<<",\"kp\":";machine_json_joint_vector(note,gains.kp);note<<",\"kd\":";machine_json_joint_vector(note,gains.kd);note<<",\"pulse_axes\":[";
            for(std::size_t axis=0;axis<n;++axis) {
                if(axis)note<<',';
                const auto& p=result.pulse_plans[axis];note<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[axis])<<"\",\"executed\":"<<(result.pulse_executed[axis]?"true":"false")
                    <<",\"direction\":"<<p.direction<<",\"amplitude_nm\":"<<p.amplitude_nm<<",\"baseline_s\":"<<p.baseline_s<<",\"rise_s\":"<<p.rise_s<<",\"hold_s\":"<<p.hold_s<<",\"fall_s\":"<<p.fall_s
                    <<",\"release_s\":"<<p.release_s<<",\"duration_s\":"<<p.duration_s<<",\"start_s\":";number(result.pulse_start_s[axis]);
                note<<",\"end_s\":";number(result.pulse_end_s[axis]);note<<",\"actual_release_s\":";number(result.pulse_release_s[axis]);
                note<<",\"domain_stopped\":"<<(result.pulse_domain_stopped[axis]?"true":"false")<<'}';
            }
            note<<"]}";gain_experiment_records_.push_back(note.str());
            gain_probe_end(result.validated?"quiet_drag_goal_measured":"drag_measurement_available_for_review",true);gain_drag_diagnostic(result,gains);
            if(observation)*observation=result;
            return result;
        } catch(const std::exception& error) {
            result.completed=false;result.error=error.what();
            try{finish_capture();}catch(...){}
            try{assess_evidence();write_evidence();}catch(...){}
            result.validated=false;gain_probe_end("measurement_interrupted");gain_drag_diagnostic(result,gains);if(observation)*observation=result;throw;
        }
    }

    bool model_calibration_static_pass(bool reverse) {
        if(!model_calibration_trajectory_) return false;
        // Capture usable moving samples in the same traversal as gravity poses
        model_calibration_speed_tier_.store(2);
        model_calibration_direction_.store(reverse ? -1 : 1);
        const auto& trajectory = *model_calibration_trajectory_;
        std::vector<JointVector> ordered = trajectory.positions;
        if(reverse) std::reverse(ordered.begin(), ordered.end());
        std::vector<std::pair<std::size_t, ModelCalibrationPoseTarget>> targets;
        for(const auto& target : model_calibration_pose_targets_) {
            const std::size_t ordered_index = reverse ? trajectory.positions.size() - 1 - target.trajectory_index : target.trajectory_index;
            targets.emplace_back(ordered_index, target);
        }
        std::stable_sort(targets.begin(),targets.end(),[](const auto& a,const auto& b){return a.first<b.first;});
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

    JointImpedanceGains gain_protocol_gains(const JointImpedanceGains& requested) const {
        const auto encoded=serial_arm::servo_tuning::quantize_gains({requested.kp,requested.kd},gain_model_limits_);
        return {encoded.kp,encoded.kd};
    }

    JointImpedanceGains gain_initialize_from_path() {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();Dynamics model,gravity_model;
        if(!model.configure(cfg_.dynamics) || !gravity_model.configure(cfg_.dynamics))throw std::runtime_error("gain initialization needs a valid URDF, controlled joint mapping and inertia");
        if(!model_calibration_trajectory_ || model_calibration_trajectory_->positions.empty())throw std::runtime_error("gain initialization needs the checked demonstration");
        const auto& trajectory=*model_calibration_trajectory_;const auto& path=trajectory.positions;
        gain_demonstrated_path_lower_.assign(n,std::numeric_limits<double>::infinity());
        gain_demonstrated_path_upper_.assign(n,-std::numeric_limits<double>::infinity());
        for(const auto& q:path) {
            if(q.size()!=n)throw std::runtime_error("demonstration joint count does not match gain model");
            for(std::size_t i=0;i<n;++i) {
                if(!std::isfinite(q[i]))throw std::runtime_error("demonstration contains nonfinite joint position");
                gain_demonstrated_path_lower_[i]=std::min(gain_demonstrated_path_lower_[i],q[i]);
                gain_demonstrated_path_upper_[i]=std::max(gain_demonstrated_path_upper_[i],q[i]);
            }
        }
        auto demand=path_response_demand(path,trajectory.sample_dt,gain_requirements_.peak_error_rad);
        demand.settling_time_s=gain_requirements_.settling_time_s;
        demand.observation_interval_s=1.0/cfg_.runtime.ctrl_frequency_hz;
        {std::lock_guard<std::mutex> lock(mutex_);
            if(last_output_ && last_output_->actuator_state.feedback_received_ns.size()==n) {
                const auto ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
                for(const auto stamp:last_output_->actuator_state.feedback_received_ns)
                    if(stamp>0 && stamp<=ns)demand.feedback_age_s=std::max(demand.feedback_age_s,(ns-stamp)*1e-9);
            }
        }
        demand.coupled_acceleration_torque.assign(n,0);
        JointVector gradient(n,0),feedforward_peak(n,0),origin;
        ModelFeedforwardMode feedforward;
        {std::lock_guard<std::mutex> lock(mutex_);origin=robot_.get_joint_state().pos;feedforward=robot_.get_model_feedforward_mode();}
        std::vector<double> inertia_sample_times;
        const auto stride=std::max<std::size_t>(1,path.size()/128);
        for(std::size_t index=0;;index=std::min(path.size()-1,index+stride)) {
            gain_check_budget();if(model_calibration_cancel_.load() || model_calibration_pause_.load())throw std::runtime_error("gain model initialization interrupted");
            const auto& q=path[index];JointVector velocity(n,0),acceleration(n,0);
            if(index>0 && index+1<path.size())for(std::size_t i=0;i<n;++i){velocity[i]=(path[index+1][i]-path[index-1][i])/(2*trajectory.sample_dt);acceleration[i]=(path[index+1][i]-2*q[i]+path[index-1][i])/(trajectory.sample_dt*trajectory.sample_dt);}
            if(q.size()!=n || !model.update(JointState{q,velocity,JointVector(n,0)},acceleration,acceleration))throw std::runtime_error("URDF mass matrix cannot be evaluated on demonstration");
            const auto& M=model.get_mass_matrix();
            std::vector<JointVector> matrix(n,JointVector(n));
            for(std::size_t axis=0;axis<n;++axis) {
                double coupled=0;
                for(std::size_t j=0;j<n;++j){matrix[axis][j]=M(axis,j);coupled+=std::abs(M(axis,j))*demand.acceleration_peak[j];}
                demand.coupled_acceleration_torque[axis]=std::max(demand.coupled_acceleration_torque[axis],coupled);
                const double ff=feedforward==ModelFeedforwardMode::NONE?0:feedforward==ModelFeedforwardMode::GRAVITY?model.get_gravity_compensation()[axis]:model.get_inverse_dynamics()[axis];
                feedforward_peak[axis]=std::max(feedforward_peak[axis],std::abs(ff));
                JointVector plus=q,minus=q;plus[axis]+=.001;minus[axis]-=.001;
                const auto a=gravity_model.get_gravity_regression(plus),b=gravity_model.get_gravity_regression(minus);
                if(!a || !b)throw std::runtime_error("URDF gravity gradient unavailable");
                double residual=(a->original_gravity[axis]-b->original_gravity[axis])/.002;
                // Use the configured compensation, including scale/overrides;
                // compensated gravity is not counted again as feedback stiffness.
                if(feedforward!=ModelFeedforwardMode::NONE) {
                    const JointVector zero(n,0);
                    if(!gravity_model.update(JointState{plus,zero,zero},zero,zero))throw std::runtime_error("gravity compensation gradient unavailable");
                    const double positive=gravity_model.get_gravity_compensation()[axis];
                    if(!gravity_model.update(JointState{minus,zero,zero},zero,zero))throw std::runtime_error("gravity compensation gradient unavailable");
                    residual-=(positive-gravity_model.get_gravity_compensation()[axis])/.002;
                }
                gradient[axis]=std::max(gradient[axis],std::abs(residual));
            }
            inertia_sample_times.push_back(index*trajectory.sample_dt);demand.mass_matrix_samples.push_back(std::move(matrix));
            if(index==path.size()-1)break;
        }
        gain_model_limits_.resize(n);
        for(std::size_t axis=0;axis<n;++axis) {
            const double effort=std::min(.25*cfg_.safety.limits.max_effort[axis],.8*cfg_.safety.limits.max_effort[axis]-feedforward_peak[axis]);
            auto& bounds=gain_model_limits_[axis];bounds={gain_requirements_.local_range_rad,.3*cfg_.safety.limits.max_vel[axis],.3*cfg_.safety.limits.max_acc[axis],effort,cfg_.safety.limits.max_kp[axis],cfg_.safety.limits.max_kd[axis],{}};
            // A legal endpoint has zero room on one side, not zero usable
            // path. Keep its observed asymmetric interval for inward entry;
            // actual bidirectional evidence is required only after positioning.
            gain_apply_demonstrated_probe_range(bounds,axis,origin[axis]);
            bounds.grid={cfg_.safety.limits.kp_resolution.size()==n?cfg_.safety.limits.kp_resolution[axis]:0,
                         cfg_.safety.limits.kd_resolution.size()==n?cfg_.safety.limits.kd_resolution[axis]:0};
        }
        // Use one explicitly time-weighted M reference for this fixed path
        // controller, retaining the separate full inertia envelope. Maxima
        // from unrelated local K/D designs cannot be combined into a DD pair.
        // First import uses the same local design at its declared pose.
        gain_model_initialization_={};auto& initialized=gain_model_initialization_;
        const auto nominal=nominal_urdf_path_gain_design(demand.mass_matrix_samples,inertia_sample_times);
        initialized.source=nominal.source;initialized.inertia_min=nominal.inertia_min;initialized.inertia_max=nominal.inertia_max;
        initialized.unconstrained_kp=nominal.unconstrained_kp;initialized.unconstrained_kd=nominal.unconstrained_kd;initialized.constraints.resize(n);
        for(std::size_t i=0;i<n;++i) {
            const auto& bound=gain_model_limits_[i];
            const auto executable=constrain_candidate(initialized.unconstrained_kp[i],0,bound,
                .5*gain_requirements_.local_range_rad,std::min(.1,bound.max_velocity));
            double executable_p=executable.first;
            if(executable_p<=0 && initialized.unconstrained_kp[i]>0) {
                executable_p=quantize_gain(initialized.unconstrained_kp[i],bound.grid.kp_step,bound.kp_max,true);
                if(executable_p*.5*gain_requirements_.local_range_rad>bound.effort_budget)
                    throw GainQualificationFailure("URDF_nominal_positive_protocol_gain_code_exceeds_execution_effort:"+cfg_.joint_names[i]);
            }
            initialized.gains.kp.push_back(executable_p);
            initialized.frequency_hz.push_back(2);initialized.task_frequency_hz.push_back(2);
            if(executable.first<initialized.unconstrained_kp[i])initialized.constraints[i].push_back("nominal_P_execution_protocol_or_feedback_effort_constraint");
        }
        const auto damping=nominal_urdf_damping_design(nominal.mass_matrix_basis,initialized.gains.kp);
        for(std::size_t i=0;i<n;++i) {
            const auto& bound=gain_model_limits_[i];
            const double remaining=std::max(0.0,bound.effort_budget-initialized.gains.kp[i]*.5*gain_requirements_.local_range_rad);
            const double ceiling=std::min(bound.kd_max,remaining/std::min(.1,bound.max_velocity));
            const double d=quantize_gain(std::min(damping.unconstrained_kd[i],ceiling),bound.grid.kd_step,bound.kd_max);
            initialized.gains.kd.push_back(d);
            if(d<damping.unconstrained_kd[i])initialized.constraints[i].push_back("nominal_D_execution_protocol_or_feedback_effort_constraint_not_physical_inertia");
        }
        const auto raw_modes=nominal.modal_diagnostics;
        const auto executable_modes=ideal_urdf_modal_diagnostics(nominal.mass_matrix_basis,initialized.gains.kp,initialized.gains.kd);
        initialized.nominal_mode_frequency_hz=executable_modes.nominal_mode_frequency_hz;
        initialized.nominal_mode_damping_ratio=executable_modes.nominal_mode_damping_ratio;
        gain_task_model_error_.clear();gain_task_model_initialization_=initialized;
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_seed_source_=gain_model_initialization_.source;}
        gain_experiment_records_.clear();gain_candidate_records_.clear();
        std::ostringstream initial;initial<<"{\"kind\":\"model_initialization_conditions\",\"physical_prior\":\"urdf_geometry_mass_com_inertia\",\"response_demand_source\":\"explicit_nominal_2Hz_damping_0.7_design_not_a_URDF_measurement\",\"constraint_role\":\"execution_guards_not_mechanical_identification\",\"feedforward_mode\":\""<<to_string(feedforward)<<"\",\"gravity_gradient_source\":\"nominal_minus_configured_executed_compensation\",\"measured\":false,\"feedforward_envelope_nm\":";
        machine_json_joint_vector(initial,feedforward_peak);initial<<",\"coupled_acceleration_torque_nm\":";machine_json_joint_vector(initial,demand.coupled_acceleration_torque);initial<<",\"residual_gravity_gradient\":";machine_json_joint_vector(initial,gradient);
        initial<<",\"design_parameters\":{\"frequency_hz\":2,\"damping_ratio\":0.7,\"role\":\"explicit_engineering_design_constants_not_URDF_measurements\"}"
            <<",\"damping_design_method\":\""<<damping.damping_design_method<<"\",\"damping_interface_projection\":\""<<damping.damping_interface_projection
            <<"\",\"damping_posture_aggregation\":\""<<nominal.basis<<"\",\"damping_stiffness_basis_kp\":";machine_json_joint_vector(initial,damping.stiffness_basis);
        initial<<",\"damping_before_protocol_kd\":";machine_json_joint_vector(initial,damping.unconstrained_kd);
        initial<<",\"inertia_sample_times_s\":";machine_json_joint_vector(initial,nominal.sample_times_s);
        initial<<",\"inertia_time_weights\":";machine_json_joint_vector(initial,nominal.normalized_time_weights);
        initial<<",\"ideal_modal_model\":{\"measured\":false,\"posture_rad\":null,\"mass_matrix_basis\":\""<<nominal.basis<<"\",\"mass_matrix_kg_m2\":[";
        for(std::size_t i=0;i<n;++i){if(i)initial<<',';machine_json_joint_vector(initial,nominal.mass_matrix_basis[i]);}initial<<']';
        initial<<",\"local_design_kp\":";machine_json_joint_vector(initial,damping.stiffness_basis);
        initial<<",\"local_full_damping_matrix\":[";for(std::size_t i=0;i<n;++i){if(i)initial<<',';machine_json_joint_vector(initial,damping.full_damping_matrix[i]);}initial<<']';
        const auto modal_json=[&](const char* key,const auto& modes) {
            initial<<",\""<<key<<"\":{\"nominal_mode_frequency_hz\":";machine_json_joint_vector(initial,modes.nominal_mode_frequency_hz);
            initial<<",\"nominal_mode_damping_ratio\":";machine_json_joint_vector(initial,modes.nominal_mode_damping_ratio);
            initial<<",\"damping_projection_is_exact\":"<<(modes.damping_projection_is_exact?"true":"false")<<'}';
        };
        modal_json("unconstrained_group",raw_modes);modal_json("executable_group",executable_modes);initial<<'}';
        initial<<",\"position_domain_source\":\"checked_demonstration_and_fresh_feedback_intersect_explicit_safety\",\"fresh_origin_rad\":";machine_json_joint_vector(initial,origin);
        initial<<",\"initial_reference_offsets_rad\":[";for(std::size_t i=0;i<n;++i){if(i)initial<<',';initial<<'['<<gain_model_limits_[i].reference_min_offset_rad<<','<<gain_model_limits_[i].reference_max_offset_rad<<']';}
        initial<<"]}";gain_experiment_records_.push_back(initial.str());
        return {gain_model_initialization_.gains.kp,gain_model_initialization_.gains.kd};
    }

    static std::string gain_finite_json(double value) {
        if(!std::isfinite(value))return "null";
        std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
    }

    // This is the already demonstrated space in which a new experiment may
    // operate, not an estimate of physical hard stops. Enlarging a URDF limit
    // must never authorize a short probe into previously unobserved space.
    void gain_apply_demonstrated_probe_range(serial_arm::servo_tuning::ModelAxisLimits& bounds,
                                             std::size_t axis,double center) const {
        if(gain_demonstrated_path_lower_.size()!=cfg_.joint_names.size() || !std::isfinite(center))
            throw GainQualificationFailure("demonstration probe range unavailable");
        double lower=std::max(center-gain_requirements_.local_range_rad,std::min(center,gain_demonstrated_path_lower_[axis]));
        double upper=std::min(center+gain_requirements_.local_range_rad,std::max(center,gain_demonstrated_path_upper_[axis]));
        if(cfg_.safety.limits.has_position_limit[axis]) {
            lower=std::max(lower,cfg_.safety.limits.min_pos[axis]+cfg_.safety.limits.pos_margin[axis]);
            upper=std::min(upper,cfg_.safety.limits.max_pos[axis]-cfg_.safety.limits.pos_margin[axis]);
        }
        if(lower>center || upper<center || upper<=lower)
            throw GainQualificationFailure("demonstration_probe_range_insufficient:"+cfg_.joint_names[axis]);
        bounds.reference_min_offset_rad=lower-center;
        bounds.reference_max_offset_rad=upper-center;
        bounds.position_margin_rad=.5*(upper-lower);
    }

    double gain_disturbance_ceiling(std::size_t axis,const serial_arm::servo_tuning::ModelAxisLimits& bounds,double kp) const {
        const double noise_room=std::max({3*gain_noise_[axis].encoder_step_rad,6*gain_noise_[axis].rms_rad,1e-5});
        const double room=std::max(0.0,std::max(-bounds.reference_min_offset_rad,bounds.reference_max_offset_rad)-noise_room);
        // A calibration input allowance is bounded by the actual demonstrated
        // reachable working room. Direction is selected separately and this
        // allowance is not an identified hardware torque cap.
        return std::min({.05*cfg_.safety.limits.max_effort[axis],bounds.effort_budget,.5*std::max(0.0,kp)*room});
    }

    void gain_response_json(std::ostream& out,const serial_arm::servo_tuning::ResponseModel& m) const {
        const auto num=[&](double v){if(std::isfinite(v))out<<v;else out<<"null";};
        out<<"{\"identifiable\":"<<(m.identifiable?"true":"false")<<",\"tracking_valid\":"<<(m.tracking_valid?"true":"false")
            <<",\"disturbance_valid\":"<<(m.disturbance_valid?"true":"false")<<",\"disturbance_goal_evaluated\":"<<(m.disturbance_goal_evaluated?"true":"false")<<",\"disturbance_goal_met\":"<<(m.disturbance_goal_met?"true":"false")<<",\"model_definition\":\""<<machine_json_escape(m.model_definition)<<"\",\"reason\":\""<<machine_json_escape(m.reason)<<"\",\"fresh_samples\":"<<m.fresh_samples
            <<",\"coherence\":"<<m.coherence<<",\"relative_fit_error\":";num(m.relative_fit_error);out<<",\"held_out_error\":";num(m.held_out_error);
        out<<",\"effective_delay_s\":";num(m.effective_delay_s);out<<",\"damping_ratio\":";num(m.damping_ratio);
        out<<",\"inertia_estimate\":";num(m.inertia_estimate);out<<",\"effective_stiffness_nm_rad\":";num(m.effective_stiffness_nm_rad);
        out<<",\"physical_stiffness_nm_rad\":";num(m.physical_stiffness_nm_rad);out<<",\"nominal_physical_stiffness_nm_rad\":";num(m.nominal_physical_stiffness_nm_rad);
        out<<",\"compliance_rad_nm\":";num(m.low_frequency_compliance_rad_nm);out<<",\"reference_lag_s\":"<<m.reference_lag_s<<",\"direction_evidence_valid\":"<<(m.direction_evidence_valid?"true":"false")<<",\"direction_bias_rad\":"<<gain_finite_json(m.direction_bias_rad)<<",\"velocity_error_rad_per_rad_s\":"<<gain_finite_json(m.velocity_error_rad_per_rad_s)
            <<",\"tracking_peak_error_rad\":"<<m.tracking_peak_error_rad<<",\"dynamic_overshoot\":"<<(m.dynamic_overshoot?"true":"false")
            <<",\"sustained_oscillation\":"<<(m.sustained_oscillation?"true":"false")<<",\"mechanical_resonance_identified\":false,\"missing_evidence\":";
        gain_json_warnings(out,m.missing_evidence);out<<'}';
    }

    void gain_sampled_loop_json(std::ostream& out,const serial_arm::servo_tuning::SampledPdLoopReport& loop) const {
        out<<"{\"valid\":"<<(loop.valid?"true":"false")<<",\"crossover_covered\":"<<(loop.crossover_covered?"true":"false")
            <<",\"scope\":\""<<machine_json_escape(loop.scope)<<"\",\"reason\":\""<<machine_json_escape(loop.reason)
            <<"\",\"crossover_hz\":"<<(loop.crossover_covered?gain_finite_json(loop.crossover_hz):"null")
            <<",\"phase_margin_deg\":"<<(loop.crossover_covered?gain_finite_json(loop.phase_margin_deg):"null")<<",\"missing_evidence\":";
        gain_json_warnings(out,loop.missing_evidence);out<<",\"points\":[";
        for(std::size_t i=0;i<loop.points.size();++i){if(i)out<<',';const auto& p=loop.points[i];
            out<<"{\"hz\":"<<gain_finite_json(p.hz)<<",\"magnitude\":"<<gain_finite_json(p.magnitude)<<",\"phase_deg\":"<<gain_finite_json(p.phase_deg)<<'}';}
        out<<"]}";
    }

    void gain_empirical_frequency_json(std::ostream& out,const serial_arm::servo_tuning::EmpiricalMitFrequencyResponse& response) const {
        out<<"{\"valid\":"<<(response.valid?"true":"false")<<",\"reason\":\""<<machine_json_escape(response.reason)
            <<"\",\"isolated_axis_excitation\":"<<(response.isolated_axis_excitation?"true":"false")
            <<",\"tracking_response_valid\":"<<(response.tracking_response_valid?"true":"false")
            <<",\"disturbance_response_valid\":"<<(response.disturbance_response_valid?"true":"false")
            <<",\"input_definition\":\""<<machine_json_escape(response.input_definition)<<"\",\"time_definition\":\""<<machine_json_escape(response.time_definition)
            <<"\",\"plant_scope\":\""<<machine_json_escape(response.plant_scope)<<"\",\"loop_plant_definition\":\""<<machine_json_escape(response.loop_plant_definition)
            <<"\",\"fresh_samples\":"<<response.fresh_samples<<",\"duplicate_feedback\":"<<response.duplicate_feedback<<",\"missing_evidence\":";
        gain_json_warnings(out,response.missing_evidence);out<<",\"points\":[";
        const auto complex=[&](const std::complex<double>& v){out<<"["<<gain_finite_json(v.real())<<','<<gain_finite_json(v.imag())<<']';};
        for(std::size_t i=0;i<response.points.size();++i){if(i)out<<',';const auto& p=response.points[i];out<<"{\"hz\":"<<gain_finite_json(p.hz)<<",\"plant\":";complex(p.plant);
            out<<",\"physical_plant\":";complex(p.physical_plant);out<<",\"feedback_velocity_per_position\":";complex(p.feedback_velocity_per_position);
            out<<",\"executed_feedforward_per_position\":";complex(p.executed_feedforward_per_position);
            out<<",\"input_amplitude_nm\":"<<gain_finite_json(p.input_amplitude_nm)<<",\"loop_input_amplitude_nm\":"<<gain_finite_json(p.loop_input_amplitude_nm)
                <<",\"output_amplitude_rad\":"<<gain_finite_json(p.output_amplitude_rad)<<",\"coherence\":"<<gain_finite_json(p.coherence)
                <<",\"half_relative_error\":"<<gain_finite_json(p.half_relative_error)<<",\"velocity_half_relative_error\":"<<gain_finite_json(p.velocity_half_relative_error)
                <<",\"input_noise_floor_nm\":"<<gain_finite_json(p.input_noise_floor_nm)<<",\"loop_input_noise_floor_nm\":"<<gain_finite_json(p.loop_input_noise_floor_nm)
                <<",\"output_noise_floor_rad\":"<<gain_finite_json(p.output_noise_floor_rad)<<'}';}
        out<<"]}";
    }

    JointVector gain_fresh_short_center() {
        const auto n=cfg_.joint_names.size();gain_check_budget();
        std::unique_lock<std::mutex> lock(mutex_);
        const auto started=Robot::Clock::now();std::uint64_t cursor=cycle_counter_;
        const auto before=last_output_?last_output_->actuator_state.feedback_received_ns:std::vector<std::uint64_t>{};
        while(Robot::Clock::now()-started<std::chrono::milliseconds(150)) {
            if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                throw std::runtime_error("motion establishment center acquisition interrupted");
            if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE;}))break;
            cursor=cycle_counter_;if(!last_output_)continue;
            const auto& out=*last_output_;const auto& stamps=out.actuator_state.feedback_received_ns;
            if(out.joint_state.pos.size()!=n || stamps.size()!=n)throw std::runtime_error("motion establishment center feedback mapping invalid");
            const auto now=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            const auto center=serial_arm::servo_tuning::measured_probe_center(out.joint_state.pos,stamps,now,before);
            if(center)return *center;
        }
        throw std::runtime_error("motion establishment center needs fresh per-joint measured feedback");
    }

    std::pair<JointVector,JointVector> gain_short_gravity_terms(const JointVector& center) {
        const auto n=cfg_.joint_names.size();Dynamics model;
        if(!model.configure(cfg_.dynamics))throw std::runtime_error("short experiment gravity model unavailable");
        ModelFeedforwardMode feedforward;{std::lock_guard<std::mutex> lock(mutex_);feedforward=robot_.get_model_feedforward_mode();}
        JointVector nominal(n,0),executed(n,0),zero(n,0);
        for(std::size_t i=0;i<n;++i) {
            gain_check_budget();auto plus=center,minus=center;plus[i]+=.001;minus[i]-=.001;
            const auto a=model.get_gravity_regression(plus),b=model.get_gravity_regression(minus);
            if(!a || !b)throw std::runtime_error("short experiment nominal gravity gradient unavailable");
            nominal[i]=(a->original_gravity[i]-b->original_gravity[i])/.002;
            if(feedforward!=ModelFeedforwardMode::NONE) {
                if(!model.update(JointState{plus,zero,zero},zero,zero))throw std::runtime_error("short experiment feedforward gradient unavailable");
                const double positive=model.get_gravity_compensation()[i];
                if(!model.update(JointState{minus,zero,zero},zero,zero))throw std::runtime_error("short experiment feedforward gradient unavailable");
                executed[i]=(positive-model.get_gravity_compensation()[i])/.002;
            }
        }
        return {nominal,executed};
    }

    // One known joint torque at a time; all references and the complete PD
    // group remain fixed. This measures yield/recovery, not dynamic poles.
    static std::vector<serial_arm::servo_tuning::Sample> gain_pulse_rows(
        const std::vector<serial_arm::servo_tuning::Sample>& rows,double start,double end) {
        std::vector<serial_arm::servo_tuning::Sample> out;
        for(auto row:rows)if(row.time_s>=start && row.time_s<=end+.02) {
            row.time_s-=start;row.command_time_s-=start;row.feedback_time_s-=start;
            row.reference_sample_time_s-=start;out.push_back(row);
        }
        return out;
    }
    JointVector gain_purpose_nominal_reference() {
        std::lock_guard<std::mutex> lock(mutex_);
        // A completed visit certifies the emitted taught knot, not arrival of
        // the mechanism. Preserve that target so static error remains visible.
        if(gain_completed_endpoint_reference_.size()!=cfg_.joint_names.size())
            throw GainQualificationFailure("purpose_measurement_needs_completed_taught_reference");
        return gain_completed_endpoint_reference_;
    }
    void gain_apply_purpose_range(serial_arm::servo_tuning::ModelAxisLimits& bounds,
        std::size_t axis,double reference,double actual) const {
        // Match Robot::set_calibration_disturbance's existing injection
        // allowance; it is an execution cap, not a mechanical/friction prior.
        bounds.effort_budget=std::min(bounds.effort_budget,.05*cfg_.safety.limits.max_effort[axis]);
        double lower=std::min(gain_demonstrated_path_lower_.at(axis),actual);
        double upper=std::max(gain_demonstrated_path_upper_.at(axis),actual);
        if(cfg_.safety.limits.has_position_limit[axis]) {
            lower=std::max(lower,cfg_.safety.limits.min_pos[axis]+cfg_.safety.limits.pos_margin[axis]);
            upper=std::min(upper,cfg_.safety.limits.max_pos[axis]-cfg_.safety.limits.pos_margin[axis]);
        }
        if(!std::isfinite(actual) || !std::isfinite(reference) || lower>reference || upper<reference || upper<=lower)
            throw GainQualificationFailure("purpose_measurement_demonstrated_domain_unavailable:"+cfg_.joint_names[axis]);
        bounds.reference_min_offset_rad=lower-reference;bounds.reference_max_offset_rad=upper-reference;
        bounds.position_margin_rad=.5*(upper-lower);
    }
    serial_arm::servo_tuning::ModelAxisLimits gain_purpose_actual_domain(
        serial_arm::servo_tuning::ModelAxisLimits bounds,double reference,double actual) const {
        bounds.reference_min_offset_rad+=reference-actual;bounds.reference_max_offset_rad+=reference-actual;
        return bounds;
    }
    double gain_purpose_cost(const JointImpedanceGains& gains) {
        using namespace serial_arm::servo_tuning;
        const auto center=gain_fresh_short_center();double cost=.6;
        for(std::size_t i=0;i<cfg_.joint_names.size();++i) {
            auto bound=gain_model_limits_[i];gain_apply_purpose_range(bound,i,center[i],center[i]);
            PurposePulseOptions option;option.tail_s=gain_requirements_.tail_s;
            option.feedback_interval_s=1/std::min(200.0,cfg_.runtime.ctrl_frequency_hz);
            option.torque_quantum_nm=actuator_info_[i].effort_resolution*std::abs(cfg_.mapper.tor_ratio[i]);
            const auto plan=plan_purpose_pulse(bound,gain_noise_[i],gain_model_initialization_.inertia_max[i],gains.kp[i],gains.kd[i],option);
            if(plan.valid)cost+=plan.duration_s+.05;
        }
        return cost;
    }
    void gain_prepare_purpose_approach(const JointImpedanceGains& approach_gains,const JointVector& center,
        const std::string& label,double reserve) {
        const auto& path=*model_calibration_trajectory_;
        std::size_t anchor=0;double distance=std::numeric_limits<double>::infinity();
        for(std::size_t k=0;k<path.positions.size();++k) {
            double d=0;for(std::size_t i=0;i<center.size();++i)d=std::max(d,std::abs(center[i]-path.positions[k][i]));
            if(d<distance){distance=d;anchor=k;}
        }
        if(distance>1e-6)return; // Legacy measured equilibria are not new source knots.
        const auto offset=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(.5/path.sample_dt)));
        const auto from=gain_next_reverse_?std::min(path.positions.size()-1,anchor+offset):(anchor>offset?anchor-offset:0);
        if(from==anchor)return;
        const auto plan=serial_arm::servo_tuning::plan_taught_anchor_visit(path.positions,path.sample_dt,anchor,from,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc);
        if(!plan.valid || !gain_trial_fits(plan.round_trip_cost_s,reserve+2))return;
        gain_visit_measurement_anchor(approach_gains,anchor,from,plan.duration_s+reserve,label+"-approach-out");
        gain_visit_measurement_anchor(approach_gains,from,anchor,reserve,label+"-approach-in");
        std::ostringstream note;note<<"{\"kind\":\"purpose_matched_approach\",\"label\":\""<<label<<"\",\"source_index\":"<<anchor
            <<",\"approach_source_index\":"<<from<<",\"arrival_error_gate\":false,\"reserved_s\":"<<reserve<<'}';gain_experiment_records_.push_back(note.str());
    }

    ServoTrial gain_purpose_trial(JointImpedanceGains gains,const std::string& label,
        double reserved_s=0,const ServoTrial* fixed=nullptr,bool compliant=false,const ServoTrial* trajectory_evidence=nullptr) {
        using namespace serial_arm::servo_tuning;
        gains=gain_protocol_gains(gains);const auto n=cfg_.joint_names.size();ServoTrial trial;
        trial.purpose_pulse=true;trial.rows.resize(n);trial.models.resize(n);trial.metrics.resize(n);
        trial.motion.resize(n);trial.plans.resize(n);trial.trajectory_assessments.resize(n);
        trial.disturbance_assessments.resize(n);trial.compliant_assessments.resize(n);
        trial.purpose_plans.resize(n);trial.purpose_assessments.resize(n);trial.input_priors.resize(n);trial.disturbance_torque_amplitude.assign(n,0);
        trial.center=fixed?fixed->center:gain_purpose_nominal_reference();
        auto approach_gains=gains;
        if(fixed && fixed->rows.size()==n)for(std::size_t i=0;i<n;++i)
            for(auto row=fixed->rows[i].rbegin();row!=fixed->rows[i].rend();++row)if(row->actual_gains_known){approach_gains.kp[i]=row->actual_kp;approach_gains.kd[i]=row->actual_kd;break;}
        gain_prepare_purpose_approach(approach_gains,trial.center,label,reserved_s+gain_purpose_cost(gains));
        trial.entry_actual=gain_fresh_short_center();auto bounds=gain_model_limits_;
        if(trial.center.size()!=n)throw std::runtime_error("purpose pulse center mapping invalid");
        for(std::size_t i=0;i<n;++i) {
            gain_apply_purpose_range(bounds[i],i,trial.center[i],trial.entry_actual[i]);
            PurposePulseOptions option;option.tail_s=gain_requirements_.tail_s;
            if(fixed && fixed->input_priors.size()==n)trial.input_priors[i]=fixed->input_priors[i];
            else if(trajectory_evidence && trajectory_evidence->rows.size()==n)
                trial.input_priors[i]=estimate_purpose_input_prior(trajectory_evidence->rows[i],gain_noise_[i],trajectory_evidence->record_path);
            option.input_prior=trial.input_priors[i];
            option.feedback_interval_s=1/std::min(200.0,cfg_.runtime.ctrl_frequency_hz);
            option.torque_quantum_nm=actuator_info_[i].effort_resolution*std::abs(cfg_.mapper.tor_ratio[i]);
            const auto actual_domain=gain_purpose_actual_domain(bounds[i],trial.center[i],trial.entry_actual[i]);
            const auto bounded=plan_purpose_pulse(actual_domain,gain_noise_[i],
                gain_model_initialization_.inertia_max[i],gains.kp[i],gains.kd[i],option);
            trial.purpose_plans[i]=fixed?fixed->purpose_plans.at(i):bounded;
            const auto& p=trial.purpose_plans[i];
            if(p.valid && (p.amplitude_nm>bounded.input_ceiling_nm ||
                p.direction*p.target_displacement_rad<actual_domain.reference_min_offset_rad ||
                p.direction*p.target_displacement_rad>actual_domain.reference_max_offset_rad))
                throw GainQualificationFailure("common purpose input exceeds measured domain or effort allowance");
        }
        trial.record_path="gain-purpose-"+label+".csv";
        {std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"purpose_input_prior\",\"record_path\":\""<<trial.record_path<<"\",\"input_allowance_source\":\"existing_Robot_calibration_disturbance_5_percent_of_explicit_Safety_effort\",\"input_priors\":[";
            for(std::size_t i=0;i<n;++i){if(i)note<<',';const auto& p=trial.input_priors[i];
                note<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"valid\":"<<(p.valid?"true":"false")
                    <<",\"source\":\""<<machine_json_escape(p.source)<<"\",\"seed_nm\":"<<p.seed_nm
                    <<",\"independent_peak_nm\":"<<p.independent_peak_nm<<",\"low_speed_pd_level_nm\":"<<p.low_speed_pd_level_nm
                    <<",\"independent_samples\":"<<p.independent_samples<<",\"low_speed_pd_samples\":"<<p.low_speed_pd_samples
                    <<",\"rejected_samples\":"<<p.rejected_samples<<",\"low_speed_cutoff_rad_s\":"<<p.low_speed_cutoff_rad_s
                    <<",\"planned_input_nm\":"<<trial.purpose_plans[i].amplitude_nm<<",\"current_domain_input_ceiling_nm\":"<<trial.purpose_plans[i].input_ceiling_nm
                    <<",\"interpretation\":\""<<p.interpretation<<"\",\"reason\":\""<<p.reason<<"\"}";}
            note<<"]}";gain_experiment_records_.push_back(note.str());}
        struct ExecutedPulse {std::size_t axis,attempt;double start;PurposePulsePlan plan;PurposePulseAssessment assessment;};
        std::vector<ExecutedPulse> executed;
        std::optional<std::size_t> active_axis;double window_start=0;PurposePulsePlan active_plan;
        const auto start=Robot::Clock::now()+std::chrono::milliseconds(500);
        {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(robot_.get_state()!=RobotState::ACTIVE || !runtime_.set_impedance_mode(robot_,JointImpedanceMode::RIGID_TRACKING) ||
               !robot_.set_execution_gain_profile(compliant?JointGainProfile::COMPLIANT:JointGainProfile::SERVO_TRACKING) ||
               !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
               !robot_.set_calibration_tracking_gains(gains,JointImpedanceMode::RIGID_TRACKING))
                throw std::runtime_error("cannot select isolated purpose pulse measurement");
            robot_.set_admittance_suspended(true);gain_capture_begin(label,start,true);
            gain_online_input_plan_.assign(n,0);
            gain_cycle_reference_error_=nullptr;
            gain_cycle_reference_until_=gain_deadline_.value_or(start+std::chrono::seconds(3600));
            gain_cycle_reference_=[this,start,center=trial.center,bounds,n,&active_axis,&window_start,&active_plan](Robot::TimePoint now) {
                JointVector tau(n,0);const double time=std::chrono::duration<double>(now-start).count();
                if(active_axis)tau[*active_axis]=purpose_pulse_input(time-window_start,active_plan);
                for(std::size_t i=0;i<n;++i) {
                    const double tolerance=std::max({3*gain_noise_[i].encoder_step_rad,6*gain_noise_[i].rms_rad,1e-5});
                    if(last_output_ && (last_output_->joint_state.pos[i]<center[i]+bounds[i].reference_min_offset_rad-tolerance ||
                        last_output_->joint_state.pos[i]>center[i]+bounds[i].reference_max_offset_rad+tolerance))
                        throw GainQualificationFailure("demonstration_probe_feedback_outside_observed_range:"+cfg_.joint_names[i]);
                }
                gain_online_input_plan_=tau;
                const auto input=robot_.set_calibration_disturbance(tau,now);if(!input)throw machine_robot_error("purpose pulse input",input.error());
                const auto reference=runtime_.submit_reference(robot_,JointPosVelCmd{center,JointVector(n,0)},now,MotionSource::CALIBRATION);
                if(!reference)throw machine_robot_error("purpose pulse fixed reference",reference.error());
            };
        }
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_="disturbance_identification";gain_probe_frequency_hz_={0,0,0};}
        double planned_duration=.6;for(const auto& plan:trial.purpose_plans)if(plan.valid)planned_duration+=plan.duration_s+.05;
        gain_probe_begin(label,"all",planned_duration,0,"isolated_purpose_pulse");
        const auto wait_until=[&](double until,std::size_t axis) {
            std::unique_lock<std::mutex> lock(mutex_);auto cursor=cycle_counter_;
            while(std::chrono::duration<double>(Robot::Clock::now()-start).count()<until) {
                gain_check_budget();if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("purpose pulse measurement interrupted");
                const double time=std::chrono::duration<double>(Robot::Clock::now()-start).count();
                gain_probe_update(std::max(0.0,time),trial.center,"isolated_purpose_pulse");
                {std::lock_guard<std::mutex> probe_lock(model_calibration_mutex_);gain_probe_joint_=cfg_.joint_names[axis];gain_probe_kp_=gains.kp[axis];gain_probe_kd_=gains.kd[axis];}
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))
                    throw std::runtime_error("purpose pulse lost control feedback");
                cursor=cycle_counter_;
            }
        };
        try {
            wait_until(0,0);
            for(std::size_t i=0;i<n;++i) {
                auto plan=trial.purpose_plans[i];double later_axes_s=0;
                for(std::size_t later=i+1;later<n;++later)if(trial.purpose_plans[later].valid)later_axes_s+=trial.purpose_plans[later].duration_s+.05;
                for(std::size_t attempt=0;;++attempt) {
                    if(!plan.valid)break;
                    if(!gain_trial_fits(plan.duration_s+.1,reserved_s+later_axes_s+2)) {
                        if(attempt==0){trial.purpose_assessments[i].reason="purpose_pulse_window_does_not_fit_remaining_budget";
                            trial.compliant_assessments[i].reason=trial.purpose_assessments[i].reason;}
                        break;
                    }
                    double beginning;
                    {std::lock_guard<std::mutex> lock(mutex_);beginning=std::max(0.0,std::chrono::duration<double>(Robot::Clock::now()-start).count());
                        active_axis=i;window_start=beginning;active_plan=plan;}
                    {std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"purpose_pulse_window_parameters\",\"record_path\":\""<<trial.record_path
                        <<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"start_s\":"<<beginning
                        <<",\"release_s\":"<<beginning+plan.release_s<<",\"end_s\":"<<beginning+plan.duration_s
                        <<",\"amplitude_nm\":"<<plan.amplitude_nm<<",\"direction\":"<<plan.direction<<",\"kp\":";
                        machine_json_joint_vector(note,gains.kp);note<<",\"kd\":";machine_json_joint_vector(note,gains.kd);note<<'}';gain_experiment_records_.push_back(note.str());}
                    wait_until(beginning+plan.duration_s,i);
                    std::vector<Sample> local;
                    {std::lock_guard<std::mutex> lock(mutex_);active_axis.reset();local=gain_pulse_rows(gain_capture_rows_[i],beginning,beginning+plan.duration_s);}
                    auto req=compliant_use_requirements(gain_requirements_.local_range_rad,plan.target_displacement_rad,gain_noise_[i]);
                    req.tail_s=gain_requirements_.tail_s;
                    const auto assessment=assess_purpose_pulse(local,gain_noise_[i],plan,req);
                    executed.push_back({i,attempt,beginning,plan,assessment});
                    // Keep the last complete direct measurement, rather than
                    // replacing useful evidence with an incomplete supplement.
                    if(attempt==0 || assessment.direct_response_complete || !trial.purpose_assessments[i].direct_response_complete) {
                        trial.purpose_plans[i]=plan;trial.purpose_assessments[i]=assessment;
                        trial.compliant_assessments[i]=assessment.compliant;
                        trial.disturbance_torque_amplitude[i]=plan.amplitude_nm;
                    }
                    if(fixed)break;
                    const auto next=next_purpose_pulse(plan,assessment);
                    const bool next_fits=next.valid && gain_trial_fits(next.duration_s+.1,reserved_s+later_axes_s+2);
                    {std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"purpose_input_supplement_decision\",\"record_path\":\""<<trial.record_path
                        <<"\",\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])<<"\",\"attempt\":"<<attempt
                        <<",\"previous_duration_s\":"<<plan.duration_s<<",\"next_duration_s\":"<<next.duration_s<<",\"supplement_action\":\""<<next.supplement_action<<"\",\"previous_input_nm\":"<<plan.amplitude_nm<<",\"next_input_nm\":"<<next.amplitude_nm
                        <<",\"previous_direction\":"<<plan.direction<<",\"next_direction\":"<<next.direction
                        <<",\"reason\":\""<<(next.valid && !next_fits?"purpose_pulse_window_does_not_fit_remaining_budget":next.reason)<<"\",\"next_window_planned\":"<<(next_fits?"true":"false")<<'}';gain_experiment_records_.push_back(note.str());}
                    if(!next_fits || (next.direction==plan.direction && next.amplitude_nm<=plan.amplitude_nm+1e-10 && next.duration_s<=plan.duration_s+1e-10))break;
                    plan=next;
                }
            }
            {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains);trial.rows=std::move(gain_capture_rows_);}
        } catch(...) {
            const auto failure=std::current_exception();
            {std::lock_guard<std::mutex> lock(mutex_);gain_cycle_reference_=nullptr;gain_capture_active_=false;gain_online_input_plan_.clear();robot_.clear_calibration_disturbance();
                if(robot_.get_state()==RobotState::ACTIVE)try{gain_hold_for_assessment_locked(gains,true);}catch(...){}
                trial.rows=std::move(gain_capture_rows_);}
            try{gain_write_response_csv(trial.record_path,trial.rows);}catch(const std::exception& e){gain_add_warning(std::string("interrupted_evidence_write_failed:")+e.what());}
            std::rethrow_exception(failure);
        }
        gain_write_response_csv(trial.record_path,trial.rows);trial.completed=true;trial.available=!executed.empty();trial.passed=false;
        for(std::size_t i=0;i<n;++i) {
            auto& a=trial.purpose_assessments[i];auto& c=trial.compliant_assessments[i];
            if(c.reason=="not_measured" || !trial.purpose_plans[i].valid){a.reason=trial.purpose_plans[i].reason;c.reason=a.reason;}
            auto& d=trial.disturbance_assessments[i];d.measurement_complete=a.direct_response_complete;
            d.static_measurement_complete=a.input_measurement_complete;d.motion_observed=a.yield_observed;d.limited=c.limited;
            d.sustained_oscillation=c.sustained_oscillation;d.input_peak_nm=c.input_peak_nm;
            d.peak_displacement_rad=c.peak_displacement_rad;d.peak_limit_rad=c.displacement_limit_rad;
            d.release_recovery_s=c.release_recovery_s;d.final_error_rad=c.final_equilibrium_error_rad;d.final_drift_rad=c.final_drift_rad;d.reason=a.reason;
            trial.models[i].reason="isolated_pulse_direct_measurement_not_dynamic_model_fit";
            trial.models[i].missing_evidence.push_back("dynamic_frequency_model_not_measured_by_purpose_pulse");
            trial.available=trial.available && a.input_measurement_complete;
            for(const auto& row:trial.rows[i])trial.duration_s=std::max(trial.duration_s,row.time_s);
        }
        std::ostringstream params;params<<std::setprecision(17)<<"{\"kind\":\"short_experiment_parameters\",\"record_path\":\""<<trial.record_path
            <<"\",\"label\":\""<<label<<"\",\"independent_disturbance\":true,\"waveform\":\"isolated_smooth_pulse\",\"isolation\":\"sequential_single_axis\",\"other_axes_execution\":\"fixed_reference_same_complete_gains\",\"center_source\":\"completed_taught_anchor_nominal_reference_not_actual_rezero\",\"hold_path\":false,\"duration_s\":"<<trial.duration_s<<",\"kp\":";
        machine_json_joint_vector(params,gains.kp);params<<",\"kd\":";machine_json_joint_vector(params,gains.kd);params<<",\"center_rad\":";machine_json_joint_vector(params,trial.center);params<<",\"entry_actual_rad\":";machine_json_joint_vector(params,trial.entry_actual);
        params<<",\"pulse_axes\":[";
        for(std::size_t k=0;k<executed.size();++k) {if(k)params<<',';const auto& w=executed[k];const auto& p=w.plan;
            params<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[w.axis])<<"\",\"axis_index\":"<<w.axis<<",\"scheduled\":true,\"attempt\":"<<w.attempt
                <<",\"start_s\":"<<w.start<<",\"release_s\":"<<w.start+p.release_s<<",\"end_s\":"<<w.start+p.duration_s
                <<",\"baseline_s\":"<<p.baseline_s<<",\"rise_s\":"<<p.rise_s<<",\"hold_s\":"<<p.hold_s<<",\"fall_s\":"<<p.fall_s
                <<",\"amplitude_nm\":"<<p.amplitude_nm<<",\"direction\":"<<p.direction<<",\"reason\":\""<<w.assessment.reason<<"\"}";}
        params<<"]}";gain_experiment_records_.push_back(params.str());
        std::ostringstream result;result<<std::setprecision(17)<<"{\"kind\":\"independent_torque_disturbance\",\"method\":\"isolated_smooth_torque_pulse\",\"model_identified\":false,\"record_path\":\""<<trial.record_path<<"\",\"axes\":";
        gain_compliant_axes_json(result,trial);result<<'}';gain_experiment_records_.push_back(result.str());
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=result.str();}
        gain_probe_end(trial.available?"purpose_direct_input_measured":"purpose_direct_measurement_partial",true);return trial;
    }

    ServoTrial gain_short_trial(JointImpedanceGains gains,bool disturbance,const std::string& label,
                                const std::vector<serial_arm::servo_tuning::Requirements>* fixed_plans=nullptr,bool hold_path=false,bool compliant_hold=false,
                                const JointVector* fixed_center=nullptr,const char* center_source="fresh_measured_center",
                                const JointVector* fixed_disturbance=nullptr,double disturbance_reserve_s=0,bool supplement_disturbance=true,
                                std::optional<std::size_t> isolated_axis=std::nullopt,bool closed_loop_frequency_trial=false,std::size_t supplemental_attempt=0) {
        using namespace serial_arm::servo_tuning;
        gains=gain_protocol_gains(gains);
        const auto n=cfg_.joint_names.size();ServoTrial trial;trial.rows.resize(n);trial.models.resize(n);trial.metrics.resize(n);trial.motion.resize(n);trial.plans.resize(n);trial.trajectory_assessments.resize(n);trial.disturbance_assessments.resize(n);
        JointVector torque_amplitude(n),center;auto local_limits=gain_model_limits_;
        std::vector<bool> disturbance_eligible(n,true);std::vector<IndependentDisturbanceDirectionPlan> disturbance_directions(n);
        if(isolated_axis && (*isolated_axis>=n || hold_path))throw std::invalid_argument("isolated frequency-response axis mapping invalid");
        center=fixed_center?*fixed_center:gain_fresh_short_center();
        const auto experiment_mode=hold_path?(compliant_hold?JointImpedanceMode::COMPLIANT_HOLD:JointImpedanceMode::RIGID_HOLD):JointImpedanceMode::RIGID_TRACKING;
        if(hold_path) {
            std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            const auto held=runtime_.set_impedance_mode(robot_,experiment_mode);
            if(!held)throw machine_robot_error("hold disturbance mode",held.error());
            center=robot_.get_hold_target();
            center_source="preserved_nominal_hold_target";
        }
        if(center.size()!=n)throw std::runtime_error("short experiment center mapping invalid");
        const auto gravity_terms=gain_short_gravity_terms(center);
        for(std::size_t axis=0;axis<n;++axis) {
            if(isolated_axis && axis!=*isolated_axis) {
                trial.plans[axis]=fixed_plans?fixed_plans->at(axis):gain_requirements_;
                trial.plans[axis].amplitude_rad=0;trial.plans[axis].reference_bias_rad=0;trial.plans[axis].motion_probe=false;trial.plans[axis].probe_duration_s=0;
                torque_amplitude[axis]=0;disturbance_eligible[axis]=false;continue;
            }
            gain_apply_demonstrated_probe_range(local_limits[axis],axis,center[axis]);
            if(closed_loop_frequency_trial) {
                if(!disturbance || !isolated_axis || !fixed_plans || !fixed_disturbance)
                    throw std::invalid_argument("closed-loop frequency experiment needs an isolated known torque and a common frozen plan");
                trial.plans[axis]=fixed_plans->at(axis);
                torque_amplitude[axis]=fixed_disturbance->at(axis);
                if(!std::isfinite(torque_amplitude[axis]) || torque_amplitude[axis]<=0 || torque_amplitude[axis]>local_limits[axis].effort_budget)
                    throw std::invalid_argument("frequency experiment exceeds the existing joint effort allowance");
                // A zero-mean AC input is planned against all four dynamic
                // responses and measured room. The static 2*tau/P recovery
                // bound belongs to ordinary hold-disturbance qualification.
                const double measured_room_floor=std::max({3*gain_noise_[axis].encoder_step_rad,6*gain_noise_[axis].rms_rad,1e-5});
                if(-local_limits[axis].reference_min_offset_rad<measured_room_floor || local_limits[axis].reference_max_offset_rad<measured_room_floor)
                    throw GainQualificationFailure("interior_frequency_measurement_needs_measured_two_sided_room_continue_taught_path");
                auto& direction=disturbance_directions[axis];direction.eligible=direction.bidirectional=true;
                direction.required_room_rad=measured_room_floor;direction.requested_torque_nm=torque_amplitude[axis];
                direction.validation_scope="bidirectional_zero_mean_frequency_input_with_dynamic_model_plan_and_actual_domain_monitor";
                direction.reason="known_AC_input_not_static_recovery_displacement_bound";
                continue;
            }
            const double remaining=gain_deadline_?std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():1000;
            const auto plan=fixed_plans && fixed_plans->at(axis).motion_probe?
                symmetric_motion_probe_plan(gain_requirements_,local_limits[axis],gains.kp[axis],gains.kd[axis],gain_noise_[axis],remaining,fixed_plans->at(axis)):
                short_probe_plan(gain_requirements_,local_limits[axis],gains.kp[axis],gains.kd[axis],gain_noise_[axis],gain_tuning_mode_=="fine",remaining,
                    fixed_plans?fixed_plans->at(axis).amplitude_rad:.015,fixed_plans?fixed_plans->at(axis).frequencies_hz.front():0);
            if(!plan.valid)throw GainQualificationFailure("short response identification insufficient at "+cfg_.joint_names[axis]+": "+plan.reason);
            trial.plans[axis]=fixed_plans?fixed_plans->at(axis):plan.requirements;
            if(fixed_plans && trial.plans[axis].amplitude_rad>plan.requirements.amplitude_rad*(1+1e-6))throw GainQualificationFailure("common candidate plan exceeds bounded excitation capability at "+cfg_.joint_names[axis]);
            const double disturbance_ceiling=gain_disturbance_ceiling(axis,local_limits[axis],gains.kp[axis]);
            torque_amplitude[axis]=disturbance?(fixed_disturbance?fixed_disturbance->at(axis):std::min(disturbance_ceiling,plan.torque_amplitude_nm)):0;
            if(disturbance && (!std::isfinite(torque_amplitude[axis]) || torque_amplitude[axis]<0 || torque_amplitude[axis]>disturbance_ceiling*(1+1e-9)))
                throw GainQualificationFailure("independent disturbance plan exceeds current calibration effort/Safety allowance");
            if(disturbance) {
                disturbance_directions[axis]=independent_disturbance_direction_plan(
                    std::max(0.0,-local_limits[axis].reference_min_offset_rad),std::max(0.0,local_limits[axis].reference_max_offset_rad),
                    gain_noise_[axis],torque_amplitude[axis],gains.kp[axis]);
                disturbance_eligible[axis]=disturbance_directions[axis].eligible;
                if(!disturbance_eligible[axis])torque_amplitude[axis]=0;
                trial.plans[axis].reference_bias_rad=0;
            }
        }
        for(const auto& plan:trial.plans)trial.duration_s=std::max(trial.duration_s,plan.probe_duration_s);
        for(auto& plan:trial.plans)plan.probe_duration_s=trial.duration_s;
        // Capture an unloaded baseline before injecting an independent torque.
        // The raw input and all time definitions retain this explicit offset.
        const double disturbance_baseline_s=disturbance?.25:0;
        trial.duration_s+=disturbance_baseline_s;
        const bool motion_only=std::any_of(trial.plans.begin(),trial.plans.end(),[](const Requirements& r){return r.motion_probe;});
        if(motion_only && disturbance)throw GainQualificationFailure("motion establishment excitation cannot replace independent disturbance identification");
        trial.center=center;trial.disturbance_torque_amplitude=torque_amplitude;trial.record_path="servo-probe-"+label+".csv";
        if(disturbance && std::none_of(disturbance_eligible.begin(),disturbance_eligible.end(),[](bool value){return value;})) {
            for(auto& model:trial.models){model.reason="independent_disturbance_not_requested_without_demonstrated_room";model.missing_evidence.push_back(model.reason);}
            std::ostringstream skipped;skipped<<"{\"kind\":\"independent_disturbance_not_run\",\"label\":\""<<machine_json_escape(label)
                <<"\",\"completed\":false,\"available\":false,\"reason\":\"independent_disturbance_not_requested_without_demonstrated_room\",\"planned_kp\":";
            machine_json_joint_vector(skipped,gains.kp);skipped<<",\"planned_kd\":";machine_json_joint_vector(skipped,gains.kd);
            skipped<<",\"center_rad\":";machine_json_joint_vector(skipped,center);skipped<<",\"axes\":[";
            for(std::size_t i=0;i<n;++i){if(i)skipped<<',';skipped<<"{\"joint\":\""<<machine_json_escape(cfg_.joint_names[i])
                <<"\",\"available_negative_rad\":"<<-local_limits[i].reference_min_offset_rad<<",\"available_positive_rad\":"<<local_limits[i].reference_max_offset_rad<<'}';}
            skipped<<"]}";gain_experiment_records_.push_back(skipped.str());
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=skipped.str();}
            gain_add_warning(label+":independent_disturbance_not_requested_without_demonstrated_room");
            return trial;
        }
        gain_require_trial(trial.duration_s+gain_requirements_.tail_s+.6);
        const auto start=Robot::Clock::now()+std::chrono::milliseconds(500);
        {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
            if(robot_.get_state()!=RobotState::ACTIVE || (!hold_path && !runtime_.set_impedance_mode(robot_,experiment_mode)) ||
               !robot_.set_execution_gain_profile(hold_path?(compliant_hold?JointGainProfile::COMPLIANT:JointGainProfile::RIGID):JointGainProfile::SERVO_TRACKING) || !robot_.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE) ||
               !robot_.set_calibration_tracking_gains(gains,experiment_mode))throw std::runtime_error("cannot select bounded short response experiment");
            robot_.set_admittance_suspended(!hold_path);
            gain_capture_begin(label,start,true,&trial.plans,disturbance,&disturbance_eligible);
        }
        model_calibration_direction_.store(0);model_calibration_speed_tier_.store(0);
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_stage_=isolated_axis?"frequency_response_identification":disturbance?"disturbance_identification":motion_only?"motion_establishment":"short_identification";}
        gain_probe_begin(label,isolated_axis?cfg_.joint_names[*isolated_axis]:"all",trial.duration_s+gain_requirements_.tail_s,trial.plans[isolated_axis.value_or(0)].amplitude_rad,
            isolated_axis?"isolated_frequency_response":disturbance?"known_torque_disturbance":motion_only?"bounded_symmetric_motion_establishment":"bounded_reference_identification");
        {std::ostringstream actual;actual<<std::setprecision(17)<<"{\"kind\":\"short_experiment_parameters\",\"record_path\":\""<<trial.record_path<<"\",\"label\":\""<<label<<"\",\"full_cycle_evidence\":\"frames.csv\",\"independent_disturbance\":"<<(disturbance?"true":"false")<<",\"hold_path\":"<<(hold_path?"true":"false")<<",\"reference_schedule\":\"control_cycle_monotonic\",\"reference_execution\":\""<<(hold_path?"fixed_hold_target":"streamed_tracking_reference")<<"\",\"duration_s\":"<<trial.duration_s<<",\"kp\":";machine_json_joint_vector(actual,gains.kp);actual<<",\"kd\":";machine_json_joint_vector(actual,gains.kd);actual<<",\"torque_amplitude_nm\":";machine_json_joint_vector(actual,torque_amplitude);actual<<",\"center_rad\":";machine_json_joint_vector(actual,center);actual<<",\"independent_input_baseline_s\":"<<disturbance_baseline_s<<",\"center_source\":\""<<center_source<<"\",\"position_domain_source\":\"checked_demonstration_and_fresh_feedback_intersect_explicit_safety\",\"demonstrated_path_lower_rad\":";machine_json_joint_vector(actual,gain_demonstrated_path_lower_);actual<<",\"demonstrated_path_upper_rad\":";machine_json_joint_vector(actual,gain_demonstrated_path_upper_);actual<<",\"disturbance_axis_eligible\":[";for(std::size_t i=0;i<n;++i){if(i)actual<<',';actual<<(disturbance && disturbance_eligible[i]?"true":"false");}actual<<"],\"independent_input_axes\":[";for(std::size_t i=0;i<n;++i){if(i)actual<<',';const auto& d=disturbance_directions[i];actual<<"{\"direction\":"<<d.direction<<",\"bidirectional\":"<<(d.bidirectional?"true":"false")<<",\"required_room_rad\":"<<d.required_room_rad<<",\"validation_scope\":\""<<d.validation_scope<<"\",\"reason\":\""<<d.reason<<"\"}";}actual<<"],\"isolated_axis_index\":";if(isolated_axis)actual<<*isolated_axis;else actual<<"null";actual<<",\"other_axes_execution\":\""<<(isolated_axis?"fixed_reference_same_complete_gains":"not_isolated")<<"\",\"plans\":[";for(std::size_t i=0;i<n;++i){if(i)actual<<',';const auto& r=trial.plans[i];actual<<"{\"amplitude_rad\":"<<r.amplitude_rad<<",\"reference_bias_rad\":"<<r.reference_bias_rad<<",\"frequencies_hz\":["<<r.frequencies_hz[0]<<','<<r.frequencies_hz[1]<<','<<r.frequencies_hz[2]<<"],\"motion_probe\":"<<(r.motion_probe?"true":"false")<<",\"duration_s\":"<<r.probe_duration_s<<",\"encoder_step_rad\":"<<r.encoder_step_rad<<",\"noise_rms_rad\":"<<r.noise_rms_rad<<"}";}actual<<"],\"nominal_gravity_gradient\":";machine_json_joint_vector(actual,gravity_terms.first);actual<<",\"executed_state_feedforward_gradient\":";machine_json_joint_vector(actual,gravity_terms.second);actual<<",\"working_domain\":[";
            for(std::size_t i=0;i<n;++i){if(i)actual<<',';const auto& r=trial.plans[i];const double w=2*pi*(r.motion_probe?r.frequencies_hz.front():r.frequencies_hz.back());
                const double error_bound=std::max(1e-6,2*r.amplitude_rad),velocity_bound=std::min(local_limits[i].max_velocity,2*r.amplitude_rad*(w+3));
                const auto domain=gain_working_domain(local_limits[i],error_bound,velocity_bound);
                actual<<"{\"protocol_kp_max\":"<<local_limits[i].kp_max<<",\"protocol_kd_max\":"<<local_limits[i].kd_max
                    <<",\"protocol_kp_quantum\":"<<local_limits[i].grid.kp_step<<",\"protocol_kd_quantum\":"<<local_limits[i].grid.kd_step
                    <<",\"working_kp_max\":"<<domain.kp_max<<",\"working_kd_max\":"<<domain.kd_max
                    <<",\"error_bound_rad\":"<<domain.reference_error_bound_rad<<",\"velocity_error_bound_rad\":"<<domain.velocity_error_bound_rad<<",\"feedback_effort_budget_nm\":"<<domain.effort_budget<<",\"reference_lower_rad\":"<<center[i]+local_limits[i].reference_min_offset_rad<<",\"reference_upper_rad\":"<<center[i]+local_limits[i].reference_max_offset_rad<<'}';}
            actual<<"]}";gain_experiment_records_.push_back(actual.str());}

        std::uint64_t cursor=0;
        try {
            {std::lock_guard<std::mutex> lock(mutex_);
                cursor=cycle_counter_;
                gain_cycle_reference_error_=nullptr;
                gain_cycle_reference_until_=start+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(trial.duration_s+gain_requirements_.tail_s+.2));
                gain_cycle_reference_=[this,start,center,plans=trial.plans,local_limits,torque_amplitude,disturbance_directions,isolated_axis,disturbance,disturbance_baseline_s,hold_path,n](Robot::TimePoint cycle_time) {
                    const double elapsed=std::chrono::duration<double>(cycle_time-start).count();
                    JointVector q=center,v(n,0),tau(n,0);
                    for(std::size_t axis=0;axis<n;++axis) {
                        if(!isolated_axis || axis==*isolated_axis) {
                            const auto offset=serial_arm::servo_tuning::probe(elapsed-disturbance_baseline_s,plans[axis]);
                            if(disturbance)tau[axis]=serial_arm::servo_tuning::independent_disturbance_input(disturbance_directions[axis],elapsed-disturbance_baseline_s,plans[axis]);
                            else {q[axis]+=offset.first;v[axis]=offset.second;}
                        }
                        if(isolated_axis && axis!=*isolated_axis)continue;
                        const auto& bound=local_limits[axis];
                        if(!disturbance && (q[axis]<center[axis]+bound.reference_min_offset_rad-1e-12 || q[axis]>center[axis]+bound.reference_max_offset_rad+1e-12))
                            throw GainQualificationFailure("demonstration_probe_reference_outside_observed_range:"+cfg_.joint_names[axis]);
                        const double tolerance=std::max({3*gain_noise_[axis].encoder_step_rad,6*gain_noise_[axis].rms_rad,.25*gain_requirements_.peak_error_rad,1e-5});
                        if(last_output_ && (last_output_->joint_state.pos[axis]<center[axis]+bound.reference_min_offset_rad-tolerance ||
                           last_output_->joint_state.pos[axis]>center[axis]+bound.reference_max_offset_rad+tolerance))
                            throw GainQualificationFailure("demonstration_probe_feedback_outside_observed_range:"+cfg_.joint_names[axis]);
                    }
                    const auto input=robot_.set_calibration_disturbance(tau,cycle_time);
                    if(!input)throw machine_robot_error("cycle sampled disturbance",input.error());
                    if(!hold_path) {
                        const auto sent=runtime_.submit_reference(robot_,JointPosVelCmd{q,v},cycle_time,MotionSource::CALIBRATION);
                        if(!sent)throw machine_robot_error("cycle sampled short reference",sent.error());
                    }
                };
            }
            while(true) {
                gain_check_budget();std::unique_lock<std::mutex> lock(mutex_);const auto now=Robot::Clock::now();const double t=std::chrono::duration<double>(now-start).count();
                if(gain_cycle_reference_error_)std::rethrow_exception(gain_cycle_reference_error_);
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("short response experiment interrupted");
                if(t>trial.duration_s+gain_requirements_.tail_s)break;
                JointVector q=center,v(n,0),tau(n,0);
                for(std::size_t axis=0;axis<n;++axis){
                    if(!isolated_axis || axis==*isolated_axis) {
                        const auto offset=probe(t,trial.plans[axis]);
                        if(disturbance)tau[axis]=offset.first*torque_amplitude[axis]/trial.plans[axis].amplitude_rad;
                        else {q[axis]+=offset.first;v[axis]=offset.second;}
                    }
                    if(isolated_axis && axis!=*isolated_axis)continue;
                    const auto& bound=local_limits[axis];
                    if(!disturbance && (q[axis]<center[axis]+bound.reference_min_offset_rad-1e-12 || q[axis]>center[axis]+bound.reference_max_offset_rad+1e-12))
                        throw GainQualificationFailure("demonstration_probe_reference_outside_observed_range:"+cfg_.joint_names[axis]);
                    const double tolerance=std::max({3*gain_noise_[axis].encoder_step_rad,6*gain_noise_[axis].rms_rad,.25*gain_requirements_.peak_error_rad,1e-5});
                    if(last_output_ && (last_output_->joint_state.pos[axis]<center[axis]+bound.reference_min_offset_rad-tolerance ||
                       last_output_->joint_state.pos[axis]>center[axis]+bound.reference_max_offset_rad+tolerance))
                        throw GainQualificationFailure("demonstration_probe_feedback_outside_observed_range:"+cfg_.joint_names[axis]);
                }
                gain_probe_update(std::max(0.0,t),q,isolated_axis?"isolated_frequency_response":disturbance?"known_torque_disturbance":"bounded_reference_identification");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE || model_calibration_cancel_.load();}))throw std::runtime_error("short response lost control feedback");
                cursor=cycle_counter_;
            }
            {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains);trial.rows=std::move(gain_capture_rows_);}
        } catch(...) {
            const auto failure=std::current_exception();
            {std::lock_guard<std::mutex> lock(mutex_);gain_capture_active_=false;robot_.clear_calibration_disturbance();
                if(robot_.get_state()==RobotState::ACTIVE)try{gain_hold_for_assessment_locked(gains,true);}catch(...){}
                trial.rows=std::move(gain_capture_rows_);}
            try{gain_write_response_csv(trial.record_path,trial.rows);}catch(const std::exception& error){gain_add_warning(std::string("interrupted_evidence_write_failed:")+error.what());}
            std::rethrow_exception(failure);
        }
        gain_write_response_csv(trial.record_path,trial.rows);
        trial.completed=true;trial.available=true;trial.passed=true;trial.compliant_assessments.resize(n);
        std::ostringstream experiment;experiment<<std::setprecision(17)<<"{\"record_path\":\""<<trial.record_path<<"\",\"kind\":\""<<(disturbance?"independent_torque_disturbance":motion_only?"motion_establishment_probe":"reference_tracking")<<"\",\"time_definition\":\"host_steady_command_and_receive_times\",\"reference_velocity_term\":true,\"feedforward\":\"executed_model_feedforward\",\"kp\":";machine_json_joint_vector(experiment,gains.kp);experiment<<",\"kd\":";machine_json_joint_vector(experiment,gains.kd);experiment<<",\"axes\":[";
        for(std::size_t axis=0;axis<n;++axis) {
            auto& m=trial.models[axis];
            if(isolated_axis && axis!=*isolated_axis) {
                m.reason="held_axis_not_excited";
                if(axis)experiment<<',';
                gain_response_json(experiment,m);
                continue;
            }
            else if(closed_loop_frequency_trial)m.reason="known_external_torque_closed_loop_response_requires_multi_controller_identification";
            else if(trial.plans[axis].motion_probe)m=motion_probe_evidence(trial.rows[axis],trial.plans[axis],gains.kp[axis],gains.kd[axis]);
            else m=identify_mit(trial.rows[axis],trial.plans[axis],{gains.kp[axis],gains.kd[axis],gain_model_initialization_.inertia_max[axis],gravity_terms.first[axis],disturbance?ExcitationKind::IndependentTorqueDisturbance:ExcitationKind::ReferenceTracking,gravity_terms.second[axis]});
            if(disturbance && !disturbance_eligible[axis]) {
                m.disturbance_valid=false;m.identifiable=false;m.reason="independent_disturbance_not_requested_without_demonstrated_room";
                m.missing_evidence.push_back(m.reason);
            } else if(disturbance && !disturbance_directions[axis].bidirectional)
                m.missing_evidence.push_back("opposite_direction_disturbance_not_measured");
            auto a=assess_trajectory(trial.rows[axis],trial.plans[axis],gain_noise_[axis],trial.duration_s);trial.metrics[axis]=a.metrics;trial.metrics[axis].applied_input_peak_nm=applied_pd_input_peak(trial.rows[axis],gains.kp[axis],gains.kd[axis]);trial.motion[axis]=a.motion;trial.trajectory_assessments[axis]=a;
            m.sustained_oscillation=m.sustained_oscillation || a.metrics.oscillating;m.dynamic_overshoot=!disturbance && a.dynamic_overshoot_rad>gain_requirements_.overshoot_rad;
            const bool valid=disturbance?m.disturbance_valid:m.tracking_valid;
            bool response_pass=valid && !m.sustained_oscillation && !m.dynamic_overshoot;
            if(disturbance) {
                auto& d=trial.disturbance_assessments[axis];
                const bool compliant_use=compliant_hold || label.find("compliant")!=std::string::npos;
                auto response_req=compliant_use_requirements(gain_requirements_.local_range_rad,
                    std::max(1e-6,torque_amplitude[axis]/std::max(gains.kp[axis],1e-9)),gain_noise_[axis]);
                response_req.tail_s=gain_requirements_.tail_s;
                trial.compliant_assessments[axis]=assess_compliant_response(trial.rows[axis],gain_noise_[axis],trial.duration_s,response_req);
                if(!disturbance_eligible[axis]){auto& c=trial.compliant_assessments[axis];c.measurement_complete=false;c.input_measurement_complete=false;c.passed=false;c.reason=m.reason;}
                if(compliant_use) {
                    auto& c=trial.compliant_assessments[axis];
                    if(!disturbance_eligible[axis]){c.measurement_complete=false;c.passed=false;c.reason=m.reason;}
                    d.measurement_complete=c.measurement_complete;d.static_measurement_complete=c.input_measurement_complete;
                    d.passed=c.passed;d.limited=c.limited;d.motion_observed=c.yield_observed;d.sustained_oscillation=c.sustained_oscillation;
                    d.input_peak_nm=c.input_peak_nm;d.peak_displacement_rad=c.peak_displacement_rad;d.peak_limit_rad=c.displacement_limit_rad;
                    d.release_recovery_s=c.release_recovery_s;d.final_error_rad=c.final_equilibrium_error_rad;d.final_drift_rad=c.final_drift_rad;d.reason=c.reason;
                    // A valid finite input/yield/recovery observation is useful
                    // even when a full closed-loop parameter fit is unresolved.
                    response_pass=c.passed;trial.available=trial.available && c.measurement_complete;
                } else {
                    d=assess_disturbance(trial.rows[axis],trial.plans[axis],gain_noise_[axis],trial.duration_s,gain_requirements_.hold_error_rad);
                    if(!disturbance_eligible[axis]){d.measurement_complete=false;d.static_measurement_complete=false;d.passed=false;d.reason=m.reason;}
                    trial.available=trial.available && d.measurement_complete && valid;response_pass=response_pass && d.passed;
                }
                m.disturbance_goal_evaluated=d.measurement_complete;m.disturbance_goal_met=d.passed;
            } else trial.available=trial.available && valid;
            if(closed_loop_frequency_trial){trial.available=true;response_pass=false;}
            trial.passed=trial.passed && response_pass;
            if(axis)experiment<<',';
            gain_response_json(experiment,m);
            if(disturbance){const auto& d=trial.disturbance_assessments[axis];experiment<<",{\"kind\":\"disturbance_recovery\",\"measurement_complete\":"<<(d.measurement_complete?"true":"false")<<",\"validated\":"<<(d.passed?"true":"false")<<",\"static_measurement_complete\":"<<(d.static_measurement_complete?"true":"false")<<",\"motion_observed\":"<<(d.motion_observed?"true":"false")<<",\"input_peak_nm\":"<<d.input_peak_nm<<",\"encoder_step_rad\":"<<d.encoder_step_rad<<",\"motion_floor_rad\":"<<d.motion_floor_rad<<",\"peak_displacement_rad\":"<<d.peak_displacement_rad<<",\"peak_limit_rad\":"<<d.peak_limit_rad<<",\"release_recovery_s\":"<<d.release_recovery_s<<",\"final_error_rad\":"<<d.final_error_rad<<",\"final_drift_rad\":"<<gain_finite_json(d.final_drift_rad)<<",\"reason\":\""<<d.reason<<"\"}";}
        }
        experiment<<"],\"compliant_use_axes\":";gain_compliant_axes_json(experiment,trial);experiment<<'}';gain_experiment_records_.push_back(experiment.str());
        gain_probe_end(trial.available?"response_measured":"identification_insufficient",true);
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_probe_diagnostic_json_=experiment.str();}
        if(disturbance && supplement_disturbance && !closed_loop_frequency_trial && supplemental_attempt<(gain_tuning_mode_=="fine"?3u:1u)) {
            JointVector ceiling(n);for(std::size_t i=0;i<n;++i)ceiling[i]=disturbance_eligible[i]?gain_disturbance_ceiling(i,local_limits[i],gains.kp[i]):0;
            const double remaining=gain_deadline_?std::max(0.0,std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count()):1000.0;
            const auto revision=revise_disturbance_input(torque_amplitude,ceiling,trial.models,trial.metrics,remaining,
                short_trial_cost(trial.plans,gain_requirements_.tail_s),disturbance_reserve_s+2.0);
            std::ostringstream note;note<<"{\"kind\":\"independent_disturbance_supplement\",\"record_path\":\""<<trial.record_path
                <<"\",\"retry\":"<<(revision.retry?"true":"false")<<",\"reasons\":";gain_json_warnings(note,revision.reasons);
            note<<",\"next_torque_amplitude_nm\":";machine_json_joint_vector(note,revision.amplitudes);note<<'}';gain_candidate_records_.push_back(note.str());
            if(revision.feedback_fault)throw GainQualificationFailure("independent disturbance experiment has unhealthy or inconsistent feedback evidence");
            if(revision.retry)return gain_short_trial(gains,true,label+"-supplement",&trial.plans,hold_path,compliant_hold,&center,center_source,
                &revision.amplitudes,disturbance_reserve_s,true,isolated_axis,false,supplemental_attempt+1);
        }
        return trial;
    }

    void gain_observe_static_input_prior(const JointImpedanceGains& candidate) {
        using namespace serial_arm::servo_tuning;
        const auto n=cfg_.joint_names.size();const auto started=Robot::Clock::now();
        std::vector<JointVector> residual(n);JointVector lo(n,std::numeric_limits<double>::infinity()),hi(n,-std::numeric_limits<double>::infinity()),vmax(n,0);
        std::vector<std::uint64_t> stamps(n,0);std::vector<std::vector<Sample>> rows;
        gain_require_trial(1.0);
        {std::lock_guard<std::mutex> lock(mutex_);gain_capture_begin("bootstrap-static-input",started,true);}
        try {
            std::unique_lock<std::mutex> lock(mutex_);std::uint64_t cursor=cycle_counter_;
            while(std::chrono::duration<double>(Robot::Clock::now()-started).count()<.45) {
                gain_check_budget();
                if(model_calibration_cancel_.load() || robot_.get_state()!=RobotState::ACTIVE)throw std::runtime_error("static_input_prior_interrupted");
                if(!cycle_cv_.wait_for(lock,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor || robot_.get_state()!=RobotState::ACTIVE;}))
                    throw std::runtime_error("static_input_prior_feedback_unavailable");
                cursor=cycle_counter_;if(!last_output_)continue;const auto& out=*last_output_;
                if(out.joint_state.tor.size()!=n || out.joint_cmd.tor.size()!=n || out.actuator_state.feedback_received_ns.size()!=n)
                    break;
                for(std::size_t i=0;i<n;++i) {
                    const auto stamp=out.actuator_state.feedback_received_ns[i];if(!stamp || stamp<=stamps[i])continue;stamps[i]=stamp;
                    const double difference=out.joint_state.tor[i]-out.joint_cmd.tor[i];
                    if(!std::isfinite(difference))continue;
                    residual[i].push_back(difference);lo[i]=std::min(lo[i],out.joint_state.pos[i]);hi[i]=std::max(hi[i],out.joint_state.pos[i]);
                    vmax[i]=std::max(vmax[i],std::abs(out.joint_state.vel[i]));
                }
            }
            gain_capture_active_=false;rows=std::move(gain_capture_rows_);
        } catch(...) {
            {std::lock_guard<std::mutex> lock(mutex_);gain_capture_active_=false;rows=std::move(gain_capture_rows_);}
            gain_write_response_csv("gain-bootstrap-static-input.csv",rows);throw;
        }
        gain_write_response_csv("gain-bootstrap-static-input.csv",rows);
        Gains requested{candidate.kp,candidate.kd};JointVector errors(n,gain_requirements_.local_range_rad),velocities(n,.01),bias(n,std::numeric_limits<double>::quiet_NaN());
        std::vector<std::string> prior_reasons(n);
        for(std::size_t i=0;i<n;++i) {
            const double step=observed_encoder_step([&]{std::vector<double> q;for(const auto& row:rows[i])if(row.new_measurement)q.push_back(row.actual);return q;}());
            if(step>0)gain_noise_[i].encoder_step_rad=gain_noise_[i].encoder_step_rad>0?std::min(gain_noise_[i].encoder_step_rad,step):step;
            if(residual[i].size()<40 || hi[i]-lo[i]>std::max(.002,6*gain_noise_[i].encoder_step_rad)) {
                prior_reasons[i]="insufficient_quiet_position_or_actual_torque_keep_model_candidate";
                gain_add_warning("static_input_prior_missing:"+cfg_.joint_names[i]);continue;
            }
            prior_reasons[i]="quiet_actual_torque_residual_prior_measured";
            auto& samples=residual[i];std::sort(samples.begin(),samples.end());bias[i]=samples[samples.size()/2];
            // A measured torque residual is a local load/feedforward prior,
            // never a configured gain floor or a global gravity correction.
            // Withhold half the legal local following room for this uncertainty.
            requested.kp[i]=std::max(candidate.kp[i],2*std::abs(bias[i])/gain_requirements_.local_range_rad);
            velocities[i]=std::max(.01,1.5*vmax[i]);
        }
        const auto bounded=constrain_measured_gain_step({candidate.kp,candidate.kd},requested,gain_model_limits_,errors,velocities);
        const bool any_prior=std::any_of(bias.begin(),bias.end(),[](double value){return std::isfinite(value);});
        std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"measured_static_input_prior\",\"record_path\":\"gain-bootstrap-static-input.csv\",\"measured\":"<<(any_prior?"true":"false")<<",\"candidate_validated\":false,\"source\":\"actual_quiet_torque_minus_executed_non_pd_torque\",\"scope\":\"local_uncertainty_prior_not_gravity_or_friction_identification\",\"torque_residual_nm\":";
        note<<'[';for(std::size_t i=0;i<n;++i){if(i)note<<',';note<<gain_finite_json(bias[i]);}note<<']';
        note<<",\"candidate_modified\":false,\"use\":\"diagnostic_only_first_controller_is_URDF_nominal\",\"axis_reasons\":";gain_json_warnings(note,prior_reasons);note<<",\"kp\":";machine_json_joint_vector(note,candidate.kp);note<<",\"kd\":";machine_json_joint_vector(note,candidate.kd);note<<",\"constraints\":";gain_json_warnings(note,bounded.reasons);note<<'}';gain_experiment_records_.push_back(note.str());
    }

    void model_calibration_run_gain() {
        using namespace serial_arm::servo_tuning;
        // Unresolved response trials retain their trust region across all
        // traversals of this purpose, and start afresh only for a new task.
        gain_online_states_.assign(2,OnlineTrajectoryState{});
        const auto n=cfg_.joint_names.size();JointVector inertia(n);JointImpedanceGains seed;
        HoldCompensationCfg disabled;
        {std::lock_guard<std::mutex> lock(mutex_);gain_probe_origin_=robot_.get_joint_state().pos;
            gain_guard_records_.clear();
            gain_completed_endpoint_reference_.clear();gain_completed_endpoint_record_.clear();
            gain_trial_original_servo_=robot_.get_config().ctrller.servo_tracking_gains;
            gain_trial_original_hold_compensation_=robot_.get_hold_compensation_cfg();disabled=*gain_trial_original_hold_compensation_;disabled.enabled=false;
            if(!robot_.set_hold_compensation_cfg(disabled))throw std::runtime_error("cannot isolate gain calibration from hold compensation");}
        seed=gain_initialize_from_path();{std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_seed_=seed;}inertia=gain_model_initialization_.inertia_max;
        gain_probe_end("model_seed_unmeasured");
        gain_noise_.resize(n);
        const bool all=model_calibration_gain_mode_=="all";
        const auto want=[&](const char* key){return all || model_calibration_gain_mode_==key;};
        JointImpedanceGains best=seed;
        ServoTrial native,native_disturbance;std::vector<ResponseModel> native_models;JointImpedanceGains compliant_rigid_reference=seed;
        std::vector<std::pair<JointImpedanceGains,ServoTrial>> measured_native_disturbances;
        struct Stage {
            std::string category;JointImpedanceGains gains;ServoTrial native,admittance,disturbance,hold_disturbance;std::vector<ResponseModel> identification_models;
            JointImpedanceGains hold_gains;GainHoldResult hold;std::optional<HoldCompensationCfg> compensation;std::optional<GainHoldResult> compensated_hold;std::optional<HoldCompensationCfg> attempted_compensation;
            std::optional<DragTrial> drag;bool validated{false},available{false},qualification_complete{false};std::string error{"not_started"};std::vector<std::string> warnings;
        };
        std::vector<Stage> stages;
        const auto append=[&](const char* key){Stage stage;stage.category=key;stage.gains=best;if(stage.category=="rigid_tracking" || stage.category=="compliant_tracking"){stage.hold_gains=seed;const double factor=stage.category=="rigid_tracking"?1.25:.35;for(std::size_t i=0;i<n;++i){stage.hold_gains.kp[i]=std::min(cfg_.safety.limits.max_kp[i],factor*seed.kp[i]);stage.hold_gains.kd[i]=std::min(cfg_.safety.limits.max_kd[i],2*.9*std::sqrt(inertia[i]*stage.hold_gains.kp[i]));}}if(stage.category=="compliant_drag")stage.drag=DragTrial(n);stages.push_back(std::move(stage));};
        if(want("rigid"))append("rigid_tracking");
        if(want("compliant"))append("compliant_tracking");
        if(want("drag"))append("compliant_drag");
        if(want("servo_tracking"))append("servo_tracking");
        const auto stage_for=[&](const char* key)->Stage&{return *std::find_if(stages.begin(),stages.end(),[&](const Stage& stage){return stage.category==key;});};
        bool compliant_path_scheduled=false,compliant_training_started=false,compliant_anchor_started=false,compliant_comparison_started=false,drag_attempted=false;
        bool native_path_scheduled=!(want("rigid") || want("servo_tracking"));
        double pending_anchor_return_s=0;
        // Optional native comparisons must not consume another purpose's
        // first learning pass, interior visit or one measured comparison.
        const auto interior_round_trip_cost=[&]() {
            return model_calibration_trajectory_->sample_dt*(model_calibration_trajectory_->positions.size()-1)+1.0;
        };
        const auto purpose_disturbance_cost=[&](const JointImpedanceGains& gains){return gain_purpose_cost(gains);};
        const auto verification_reserve=[&](const std::string& charged_measurement="") {
            const double path=model_calibration_trajectory_->sample_dt*(model_calibration_trajectory_->positions.size()-1);
            double pending=3.0+pending_anchor_return_s;
            if(!native_path_scheduled)pending+=path+gain_requirements_.tail_s+3;
            if((want("rigid") || want("servo_tracking")) && !native_disturbance.completed && charged_measurement!="native-disturbance")pending+=purpose_disturbance_cost(best);
            for(const auto* purpose:{"rigid_tracking","compliant_tracking"}) {
                if(!want(std::string(purpose)=="rigid_tracking"?"rigid":"compliant"))continue;
                const auto& stage=stage_for(purpose);
                if(!stage.hold.completed)pending+=3.0;
                if(stage.category=="compliant_tracking") {
                    if(!compliant_path_scheduled)pending+=path+gain_requirements_.tail_s+5.0;
                    if(!compliant_training_started)pending+=gain_schedule_.compliant_search_passes*(path+gain_requirements_.tail_s+3.0);
                    if(!compliant_anchor_started)pending+=interior_round_trip_cost();
                    if(!compliant_comparison_started && charged_measurement!="compliant-comparison")pending+=purpose_disturbance_cost(stage.gains)+.6;
                    if(!stage.disturbance.completed && charged_measurement!="compliant-disturbance")pending+=purpose_disturbance_cost(stage.gains);
                }
            }
            if(want("drag") && !drag_attempted && charged_measurement!="drag")pending+=30.0;
            if(gain_admittance_required_ && !stage_for("servo_tracking").admittance.available)pending+=path+gain_requirements_.tail_s+5.0;
            return pending;
        };
        struct AnchorVisit {bool ready{false};std::size_t from{0},to{0};double return_cost_s{0};};
        const auto enter_anchor=[&](const JointImpedanceGains& gains,const std::string& label,double measurement_s) {
            AnchorVisit visit;const auto& path=*model_calibration_trajectory_;
            TaughtMeasurementRoomRequest request;
            for(std::size_t i=0;i<n;++i){request.required_room_rad.push_back(std::max(6*gain_noise_[i].rms_rad,3*gain_noise_[i].encoder_step_rad));
                request.observed_position_step_rad.push_back(gain_noise_[i].encoder_step_rad);request.stationary_noise_rms_rad.push_back(gain_noise_[i].rms_rad);}
            // A positive experiment scale is needed even if the stationary
            // window observed no encoder transitions. The path itself supplies
            // that scale; it never changes an imported URDF or Safety bound.
            for(std::size_t i=0;i<n;++i)if(request.required_room_rad[i]<=0) {
                double lo=path.positions.front()[i],hi=lo;for(const auto& q:path.positions){lo=std::min(lo,q[i]);hi=std::max(hi,q[i]);}
                request.required_room_rad[i]=std::max(1e-9,.01*(hi-lo));
            }
            const auto schedule=plan_taught_measurement_schedule(path.positions,request);
            visit.from=gain_next_reverse_?path.positions.size()-1:0;
            visit.to=schedule.common_anchor.source_index;
            const auto plan=schedule.has_anchor?plan_taught_anchor_visit(path.positions,path.sample_dt,visit.from,visit.to,
                cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc):TaughtAnchorVisitPlan{};
            // The current measurement and full visit round trip are charged
            // here; reserve only the stages that will actually follow them.
            const double reserve=verification_reserve(label=="drag"?"drag":label+"-disturbance");
            if(label!="drag")measurement_s=std::max(measurement_s,purpose_disturbance_cost(gains));
            if(!schedule.has_anchor || !plan.valid || !gain_trial_fits(plan.round_trip_cost_s+measurement_s+reserve)) {
                std::ostringstream note;note<<"{\"kind\":\"interior_purpose_measurement_skipped\",\"label\":\""<<label
                    <<"\",\"completed\":false,\"endpoint_fallback\":false,\"reason\":\""
                    <<(!schedule.has_anchor?"no_measurable_demonstrated_interior":!plan.valid?"demonstrated_anchor_visit_invalid":"budget_reserves_frozen_group_and_other_purposes")<<"\"}";
                gain_experiment_records_.push_back(note.str());return visit;
            }
            gain_visit_measurement_anchor(gains,visit.from,visit.to,plan.duration_s+measurement_s+reserve,label+"-purpose-visit");
            visit.ready=true;visit.return_cost_s=plan.duration_s;pending_anchor_return_s=plan.duration_s;return visit;
        };
        const auto leave_anchor=[&](const JointImpedanceGains& gains,AnchorVisit& visit,const std::string& label) {
            const double following_path=(label=="native" || label=="compliant")?
                model_calibration_trajectory_->sample_dt*(model_calibration_trajectory_->positions.size()-1)+gain_requirements_.tail_s:0;
            if(visit.ready)gain_visit_measurement_anchor(gains,visit.to,visit.from,
                following_path+3,label+"-purpose-return");
            visit.ready=false;pending_anchor_return_s=0;
        };
        AnchorVisit native_anchor,compliant_anchor;
        std::size_t nonfatal_warnings=0;bool budget_prevented_measurement=false;
        const auto run_measurement=[&](const std::string& phase,const JointImpedanceGains& gains,const auto& measure) {
            try {
                gain_check_budget();
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || machine_robot_state()!=RobotState::ACTIVE)
                    throw std::runtime_error("gain calibration interrupted before measurement");
                measure();return true;
            }
            catch(const std::exception& error) {
                const bool deadline=gain_deadline_ && Robot::Clock::now()+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(std::min(2.0,.1*gain_budget_source_.duration_s)))>=*gain_deadline_;
                if(model_calibration_cancel_.load() || model_calibration_pause_.load() || model_calibration_fault_interrupted_.load() || machine_robot_state()!=RobotState::ACTIVE || deadline)throw;
                // An optional measurement may not fit while other purposes
                // still do. Only the original wall deadline ends the run.
                budget_prevented_measurement=gain_budget_exhausted_.exchange(false) || budget_prevented_measurement;
                {std::lock_guard<std::mutex> lock(mutex_);clear_command_sources();
                    if(!robot_.set_hold_compensation_cfg(disabled))throw std::runtime_error("cannot restore hold compensation after measurement warning");
                    gain_hold_for_assessment_locked(gains,true);}
                ++nonfatal_warnings;std::vector<std::string> blocked;
                {std::lock_guard<std::mutex> status(model_calibration_mutex_);blocked=gain_termination_blocked_joints_;gain_termination_blocked_joints_.clear();}
                const std::string warning=phase+":"+error.what();gain_add_warning("measurement_warning:"+warning);
                for(auto& stage:stages)if(phase.rfind(stage.category,0)==0 ||
                    (phase.rfind("native-",0)==0 && (stage.category=="rigid_tracking" || stage.category=="servo_tracking")) ||
                    (phase.rfind("compliant-",0)==0 && stage.category=="compliant_tracking") ||
                    (phase.rfind("drag-",0)==0 && stage.category=="compliant_drag")){stage.warnings.push_back(warning);stage.error=error.what();}
                std::ostringstream note;note<<"{\"kind\":\"measurement_warning\",\"phase\":\""<<machine_json_escape(phase)
                    <<"\",\"reason\":\""<<machine_json_escape(error.what())<<"\",\"continued\":true,\"completed\":false,\"validated\":false,\"exit_reference\":\"fresh_measured_position_hold\",\"kp\":";
                machine_json_joint_vector(note,gains.kp);note<<",\"kd\":";machine_json_joint_vector(note,gains.kd);
                note<<",\"blocked_joint_names\":";gain_json_warnings(note,blocked);note<<'}';gain_experiment_records_.push_back(note.str());
                return false;
            }
        };
        const auto train=[&](JointImpedanceGains& gains,JointGainProfile profile,const JointVector& kp_limit,const JointVector& kd_limit,std::size_t learning_passes,const std::string& label) {
            gains=gain_protocol_gains(gains);auto limits=gain_model_limits_;
            for(std::size_t i=0;i<n;++i){limits[i].kp_max=kp_limit[i];limits[i].kd_max=kd_limit[i];}
            const double path_cost=model_calibration_trajectory_->sample_dt*(model_calibration_trajectory_->positions.size()-1)+gain_requirements_.tail_s+3;
            ServoTrial measured;
            // The demonstration is the running experiment: fresh samples feed
            // the local estimator and its bounded PD updates during motion.
            // No constant-controller return traversal is consumed just to get
            // back to a cold baseline. Each optional pass also learns.
            for(std::size_t pass=0;pass<learning_passes;++pass) {
                if(!gain_trial_fits(path_cost+verification_reserve()))break;
                const auto phase=label+"-online-"+std::to_string(pass);
                const bool complete=run_measurement(phase,gains,[&]{
                    measured=gain_trajectory_trial(gains,profile,false,false,phase,"trajectory_adapting",&gains,&limits);
                });
                if(!complete)break;
            }
            measured.available=false;return measured;
        };
        const auto tune_hold=[&](Stage& stage,bool compliant) {
            stage.hold_gains=gain_protocol_gains(stage.gains);
            run_measurement(stage.category+"-hold",stage.hold_gains,[&] {
            stage.hold=gain_hold_trial(stage.hold_gains,compliant,disabled);
            // Rigid/compliant each expose one actual PD group. The legacy
            // hold_gains key mirrors that group; holding does not silently
            // create a separate, untracked feedback controller.
            });
            // Fixed-reference native disturbance and hold use the same MIT
            // execution, disabled integral, complete gains and interior target.
            // Keep the source evidence rather than injecting it twice.
            stage.hold_disturbance=stage.disturbance;
            std::ostringstream note;note<<"{\"kind\":\"shared_disturbance_evidence\",\"purpose\":\""<<stage.category
                <<"\",\"record_path\":\""<<stage.disturbance.record_path
                <<"\",\"reason\":\"same_complete_PD_fixed_interior_reference_input_feedforward_and_load\",\"reference_only\":true}";
            gain_experiment_records_.push_back(note.str());
        };
        const auto tune_rigid=[&](JointImpedanceGains& gains,ServoTrial& disturbance,GainHoldResult* hold) {
            if(disturbance.rows.size()!=n || disturbance.center.size()!=n || disturbance.compliant_assessments.size()!=n)return;
            const auto signature="fixed_input_from:"+disturbance.record_path;
            const auto before=gain_rigid_evidence(gains,disturbance,hold,signature);
            std::vector<GainCommandGrid> grids;for(const auto& axis:gain_model_limits_)grids.push_back(axis.grid);
            const auto proposal=rigid_candidate_report({gains.kp,gains.kd},cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd,before,grids,gain_model_limits_);
            const JointImpedanceGains next{proposal.gains.kp,proposal.gains.kd};
            const auto incumbent=gains;
            const double cost=disturbance.duration_s+.6+(hold?3:0)+verification_reserve();
            if(!proposal.changed || !gain_trial_fits(cost) || !gain_admit_candidate(gains,next,disturbance.rows,"rigid-before-purpose-comparison"))return;
            run_measurement("rigid-purpose-local-comparison",gains,[&] {
                auto checked=gain_purpose_trial(next,"rigid-purpose-local-comparison",verification_reserve(),&disturbance);
                std::optional<GainHoldResult> checked_hold;if(hold)checked_hold=gain_hold_trial(next,false,disabled);
                const auto after=gain_rigid_evidence(next,checked,checked_hold?&*checked_hold:nullptr,signature);
                const bool kept=rigid_use_preferred({next.kp,next.kd},after,{incumbent.kp,incumbent.kd},before);
                std::ostringstream note;note<<std::setprecision(17)<<"{\"kind\":\"rigid_purpose_local_comparison\",\"kept\":"<<(kept?"true":"false")
                    <<",\"reference_only\":true,\"accuracy_is_retention_gate\":false,\"requires_frozen_group_measurement\":true,\"record_path\":\""<<checked.record_path
                    <<"\",\"baseline_record_path\":\""<<disturbance.record_path<<"\",\"previous_kp\":";
                machine_json_joint_vector(note,incumbent.kp);note<<",\"previous_kd\":";machine_json_joint_vector(note,incumbent.kd);
                note<<",\"proposed_kp\":";machine_json_joint_vector(note,next.kp);note<<",\"proposed_kd\":";machine_json_joint_vector(note,next.kd);
                note<<",\"reasons\":";gain_json_warnings(note,proposal.reasons);note<<'}';gain_candidate_records_.push_back(note.str());
                if(kept){gains=next;disturbance=std::move(checked);if(hold && checked_hold)*hold=std::move(*checked_hold);model_calibration_adaptation_revisions_.fetch_add(1);}
                {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(gains);}
            });
        };
        const auto run_drag=[&] {
            if(!want("drag") || drag_attempted)return;
            drag_attempted=true;
                model_calibration_gain_stage_.store(3);auto& stage=stage_for("compliant_drag");
                run_measurement(stage.category,best,[&] {
                stage.error="measurement_in_progress";stage.gains.kp.assign(n,0);stage.gains.kd.resize(n);
                for(std::size_t axis=0;axis<n;++axis){const double viscous=axis<gain_response_models_.size() && gain_response_models_[axis].identifiable?gain_response_models_[axis].viscous_damping_nm_s_rad:0;stage.gains.kd[axis]=std::clamp(5*inertia[axis]-viscous,0.0,cfg_.safety.limits.max_kd[axis]);}
                stage.gains=gain_protocol_gains(stage.gains);
                const ServoTrial* drag_prior=native.completed?&native:
                    want("compliant") && stage_for("compliant_tracking").native.completed?&stage_for("compliant_tracking").native:nullptr;
                stage.drag=gain_drag_trial(stage.gains,best,"drag-train",&*stage.drag,verification_reserve("drag"),nullptr,drag_prior);
                gain_drag_taught_release(*stage.drag,stage.gains,best,verification_reserve("drag"),"drag-train");
                stage.available=stage.drag->available;stage.validated=stage.drag->validated;
                const auto proposal=serial_arm::servo_tuning::drag_damping_candidate_report({stage.gains.kp,stage.gains.kd},
                    cfg_.safety.limits.max_kd,stage.drag->stationary,stage.drag->release);
                auto next=gain_protocol_gains({proposal.gains.kp,proposal.gains.kd});
                JointVector drag_velocity(n,0);
                for(std::size_t i=0;i<n;++i)drag_velocity[i]=stage.drag->raw_peak_velocity[i];
                bool comparable_pose=false;
                if(proposal.changed) {
                    const auto actual=gain_fresh_short_center();
                    // The measured resting pose is the dynamics-comparison
                    // basis, not an emitted taught knot. Resume toward the
                    // real nominal knot, then compare M/gravity at the fresh
                    // resulting pose; never invent an endpoint credential.
                    const auto reset_target=gain_purpose_nominal_reference();
                    const double reset_cost=gain_endpoint_transition_seconds(actual,reset_target);
                    if(gain_trial_fits(reset_cost+stage.drag->duration_s+.6,verification_reserve("drag")+2)) {
                        std::vector<std::vector<Sample>> reset_rows;
                        gain_transition_to_endpoint(best,actual,reset_target,"drag-comparison-reset",
                            stage.drag->duration_s+verification_reserve("drag")+.6,reset_rows,true,"fresh_measured_drag_release_pose");
                        gain_write_response_csv("gain-transition-drag-comparison-reset.csv",reset_rows);
                        const auto conditions=gain_drag_comparison_condition(*stage.drag,gain_fresh_short_center());
                        comparable_pose=conditions.comparable;
                        std::ostringstream note;note<<"{\"kind\":\"drag_damping_comparison_conditions\",\"comparable\":"<<(comparable_pose?"true":"false")
                            <<",\"reason\":\""<<conditions.reason<<"\",\"relative_M_change\":"<<gain_finite_json(conditions.relative_M_change)
                            <<",\"max_gravity_change_nm\":"<<gain_finite_json(conditions.max_gravity_change)<<",\"arrival_error_gate\":false}";gain_candidate_records_.push_back(note.str());
                    } else {std::ostringstream note;note<<"{\"kind\":\"drag_damping_comparison_skipped\",\"reason\":\"complete_reset_comparison_and_exit_do_not_fit_remaining_budget\"}";gain_candidate_records_.push_back(note.str());}
                }
                if(proposal.changed && comparable_pose && gain_trial_fits(stage.drag->duration_s+.6+verification_reserve("drag")) &&
                   gain_admit_candidate(stage.gains,next,JointVector(n,0),drag_velocity,"drag-before-damping-comparison")) {
                    auto frozen=gain_drag_trial(next,best,"drag-frozen",nullptr,verification_reserve("drag"),&stage.drag->pulse_plans,drag_prior);
                    gain_drag_taught_release(frozen,next,best,verification_reserve("drag"),"drag-frozen",&*stage.drag);
                    // Retain a whole measured damping group. A failed optional
                    // comparison does not erase an available quiet result.
                    if(frozen.available && serial_arm::servo_tuning::drag_use_preferred(frozen.stationary,frozen.release,stage.drag->stationary,stage.drag->release)) {
                        stage.gains=next;stage.drag=std::move(frozen);
                    }
                    std::ostringstream note;note<<"{\"kind\":\"drag_damping_only_comparison\",\"measured\":true,\"reasons\":";
                    gain_json_warnings(note,proposal.reasons);note<<",\"kp\":";machine_json_joint_vector(note,next.kp);note<<",\"kd\":";machine_json_joint_vector(note,next.kd);note<<'}';gain_candidate_records_.push_back(note.str());
                }
                stage.validated=stage.drag->validated;stage.available=stage.drag->available;
                for(std::size_t axis=0;axis<n;++axis)if(!stage.drag->release[axis].measurement_complete)
                    gain_add_warning("drag_release_decay_unmeasured:"+cfg_.joint_names[axis]);
                stage.error=stage.available?"":"drag_measurement_incomplete";
                });
        };
        const auto native_purpose_work=[&] {
            run_measurement("native-disturbance",best,[&] {
                native_disturbance=gain_purpose_trial(best,"native-disturbance",verification_reserve("native-disturbance"),nullptr,false,&native);
            });
            if(want("rigid")) {
                model_calibration_gain_stage_.store(0);auto& stage=stage_for("rigid_tracking");
                run_measurement(stage.category,stage.gains,[&] {
                stage.error="measurement_in_progress";stage.gains=best;stage.native=native;stage.disturbance=native_disturbance;stage.identification_models=native_models;stage.compensation=disabled;
                tune_hold(stage,false);
                tune_rigid(best,native_disturbance,&stage.hold);
                stage.gains=stage.hold_gains=best;stage.disturbance=stage.hold_disturbance=native_disturbance;
                // Preserve the independently measured baseline before an optional
                // compensation trial can exhaust the budget or lose feedback
                stage.available=stage.native.available && stage.disturbance.available && stage.hold_disturbance.available && stage.hold.available;
                stage.validated=stage.native.passed && stage.disturbance.passed && stage.hold_disturbance.passed && stage.hold.validated;
                stage.error=stage.available?"":"rigid_measurement_incomplete";
                if(!stage.hold.validated && stage.hold.samples>=30 && stage.hold.worst_velocity<=gain_requirements_.hold_velocity_rad_s && stage.hold.worst_error>0.0005) {
                    auto trim=disabled;trim.enabled=true;trim.integral_gain.resize(n);trim.max_torque.resize(n);trim.max_rate.resize(n);trim.contact_threshold.resize(n);
                    for(std::size_t axis=0;axis<n;++axis) {
                        const double residual=std::abs(stage.hold.mean_error[axis]*stage.hold_gains.kp[axis]);
                        trim.max_torque[axis]=std::min(.05*cfg_.safety.limits.max_effort[axis],std::max(.005*cfg_.safety.limits.max_effort[axis],2*residual));
                        trim.max_rate[axis]=trim.max_torque[axis];trim.contact_threshold[axis]=std::max(.02,.10*cfg_.safety.limits.max_effort[axis]);
                        trim.integral_gain[axis]=std::min(.25*stage.hold_gains.kp[axis],trim.max_torque[axis]/std::max(.001,std::abs(stage.hold.mean_error[axis])));
                    }
                    trim.velocity_threshold=gain_requirements_.hold_velocity_rad_s;trim.error_threshold=std::max({.01,2*gain_requirements_.hold_error_rad,1.25*stage.hold.worst_error});
                    stage.attempted_compensation=trim;
                    double integral_cost=2.1;for(std::size_t i=0;i<n;++i)integral_cost=std::max(integral_cost,1.7+3.5*stage.hold_gains.kp[i]/trim.integral_gain[i]);
                    if(gain_trial_fits(integral_cost+verification_reserve()))try {
                        auto checked=gain_hold_trial(stage.hold_gains,false,trim);
                        stage.compensated_hold=checked;
                        if(checked.available && !checked.limited && checked.worst_error<stage.hold.worst_error && checked.worst_span<=stage.hold.worst_span+std::max(.0002,3*gain_noise_.front().encoder_step_rad))stage.compensation=trim;
                    } catch(...) {
                        stage.warnings.push_back("hold_compensation_measurement_interrupted");
                        std::lock_guard<std::mutex> lock(mutex_);
                        if(!robot_.set_hold_compensation_cfg(disabled))
                            throw std::runtime_error("cannot disable interrupted hold compensation trial");
                        throw;
                    }
                    {std::lock_guard<std::mutex> lock(mutex_);if(!robot_.set_hold_compensation_cfg(disabled))throw std::runtime_error("cannot disable hold compensation trial");}
                }
                stage.validated=stage.native.passed && stage.disturbance.passed && stage.hold_disturbance.passed && (stage.hold.validated ||
                    (stage.compensation && stage.compensation->enabled && stage.compensated_hold && stage.compensated_hold->validated));
                stage.available=stage.native.available && stage.disturbance.available && stage.hold_disturbance.available && stage.hold.available;
                stage.error=stage.available?"":"rigid_measurement_incomplete";
                if(!stage.validated && stage.compensated_hold && !stage.compensated_hold->validated)stage.error+="; hold compensation: "+stage.compensated_hold->reason;
                });
            }
            if(!want("rigid"))tune_rigid(best,native_disturbance,nullptr);
            if(!want("compliant"))run_drag();
        };
        std::exception_ptr interrupted;
        try {
            gain_observe_static_input_prior(seed);best=seed;compliant_rigid_reference=seed;
            run_measurement("native-noise",seed,[&]{gain_noise_trial_all(seed);});gain_noise_.resize(n);gain_plans_.clear();
            if(want("rigid") || want("servo_tracking")) {
                model_calibration_gain_stage_.store(2);
                run_measurement("native-training",best,[&]{native=train(best,JointGainProfile::SERVO_TRACKING,cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd,gain_schedule_.native_search_passes,"native");});
                native_models=gain_response_models_;
                bool passing=false;
                run_measurement("native-passing-purpose",best,[&] {
                    passing=gain_passing_purpose_measurement(best,JointGainProfile::SERVO_TRACKING,
                        verification_reserve("native-disturbance"),purpose_disturbance_cost(best),"native",[&](std::size_t anchor,double remaining_path) {
                            pending_anchor_return_s=remaining_path;
                            run_measurement("native-local-correction",best,[&]{gain_refine_taught_segment(best,JointGainProfile::SERVO_TRACKING,native,verification_reserve(),"native",anchor);});
                            native_purpose_work();pending_anchor_return_s=0;
                        },&native);
                });
                if(!passing) {
                    run_measurement("native-purpose-visit",best,[&]{native_anchor=enter_anchor(best,"native",purpose_disturbance_cost(best));});
                    if(native_anchor.ready)native_purpose_work();
                }
            }
            if(want("rigid") || want("servo_tracking")) {
                run_measurement("native-purpose-return",best,[&]{leave_anchor(best,native_anchor,"native");});
                if(!native_anchor.ready)run_measurement("native-frozen",best,[&]{native=gain_trajectory_trial(best,JointGainProfile::SERVO_TRACKING,false,false,"native-frozen","trajectory_frozen");native_path_scheduled=native.completed;});
                for(auto& stage:stages)if(stage.category=="rigid_tracking" || stage.category=="servo_tracking") {
                    stage.gains=best;stage.native=native;stage.disturbance=native_disturbance;stage.identification_models=native_models;
                    if(stage.category=="rigid_tracking")stage.hold_gains=best;
                }
            }
            if(want("compliant")) {
                model_calibration_gain_stage_.store(1);auto& stage=stage_for("compliant_tracking");
                run_measurement(stage.category,stage.gains,[&] {
                stage.error="measurement_in_progress";
                compliant_rigid_reference=best;
                stage.gains=compliant_rigid_reference;
                auto bounds=serial_arm::calibration_replay_supervisor::compliant_bounds(compliant_rigid_reference.kp,compliant_rigid_reference.kd,cfg_.safety.limits.max_kp,cfg_.safety.limits.max_kd);
                if(!bounds.valid)throw std::runtime_error("cannot derive compliant limits from rigid reference");
                for(std::size_t axis=0;axis<n;++axis){stage.gains.kp[axis]=std::min(bounds.kp_ceiling[axis],.5*compliant_rigid_reference.kp[axis]);stage.gains.kd[axis]=std::min(bounds.kd_ceiling[axis],std::sqrt(.5)*compliant_rigid_reference.kd[axis]);}
                Gains encoded_soft{stage.gains.kp,stage.gains.kd};
                if(!encode_compliant_candidate(encoded_soft,bounds.kp_ceiling,bounds.kd_ceiling,gain_model_limits_))
                    throw GainQualificationFailure("compliant restoring response is below executable protocol resolution");
                stage.gains={encoded_soft.kp,encoded_soft.kd};
                {std::ostringstream shared;shared<<"{\"kind\":\"compliant_shared_response_candidate\",\"measured\":false,\"source\":\"native_controller_candidate_and_urdf_nominal_frequency_ratio\",\"candidate_requires_purpose_measurement\":true,\"kp\":";
                    machine_json_joint_vector(shared,stage.gains.kp);shared<<",\"kd\":";machine_json_joint_vector(shared,stage.gains.kd);shared<<'}';gain_experiment_records_.push_back(shared.str());}
                compliant_training_started=true;
                {std::ostringstream note;note<<"{\"kind\":\"gain_stage_budget_reservation\",\"purpose\":\"compliant\",\"future_measurements_reserved_s\":"<<verification_reserve()
                    <<",\"scope\":\"remaining_purposes_and_frozen_groups\"}";gain_experiment_records_.push_back(note.str());}
                if(!run_measurement(stage.category+"-training",stage.gains,[&]{stage.native=train(stage.gains,JointGainProfile::COMPLIANT,bounds.kp_ceiling,bounds.kd_ceiling,gain_schedule_.compliant_search_passes,"compliant");}))
                    stage.warnings.push_back("compliant_online_measurement_incomplete");
                stage.identification_models=gain_response_models_;
                compliant_anchor_started=true;
                const auto soft_purpose_work=[&] {
                run_measurement(stage.category+"-disturbance",stage.gains,[&]{
                    stage.disturbance=gain_purpose_trial(stage.gains,"compliant-disturbance",verification_reserve("compliant-disturbance"),nullptr,true,&stage.native);
                });
                compliant_comparison_started=true;
                if(stage.disturbance.compliant_assessments.size()==n) {
                    std::vector<GainCommandGrid> grids;for(const auto& axis:gain_model_limits_)grids.push_back(axis.grid);
                    const auto proposal=compliant_candidate_report({stage.gains.kp,stage.gains.kd},bounds.kp_ceiling,bounds.kd_ceiling,stage.disturbance.compliant_assessments,grids);
                    auto encoded_next=proposal.gains;
                    auto pmax=bounds.kp_ceiling,dmax=bounds.kd_ceiling;
                    const bool executable=encode_compliant_candidate(encoded_next,pmax,dmax,gain_model_limits_);
                    const JointImpedanceGains next{encoded_next.kp,encoded_next.kd};
                    if(executable && proposal.changed && gain_trial_fits(stage.disturbance.duration_s+.6+verification_reserve()) &&
                       gain_admit_candidate(stage.gains,next,stage.native.rows,"compliant-before-purpose-comparison"))run_measurement("compliant-local-correction",stage.gains,[&] {
                        auto checked=gain_purpose_trial(next,"compliant-local-correction",verification_reserve(),&stage.disturbance,true);
                        const bool purpose_improved=checked.completed && compliant_use_preferred(checked.compliant_assessments,stage.disturbance.compliant_assessments,
                            Gains{next.kp,next.kd},Gains{stage.gains.kp,stage.gains.kd},checked.completed,stage.disturbance.completed);
                        const bool kept=purpose_improved;
                        std::ostringstream note;note<<"{\"kind\":\"compliant_purpose_local_comparison\",\"kept\":"<<(kept?"true":"false")
                            <<",\"purpose_improved\":"<<(purpose_improved?"true":"false")<<",\"requires_frozen_group_measurement\":true"
                            <<",\"accuracy_is_retention_gate\":false,\"record_path\":\""<<checked.record_path<<"\",\"reasons\":";
                        gain_json_warnings(note,proposal.reasons);note<<'}';gain_candidate_records_.push_back(note.str());
                        if(kept){stage.gains=next;stage.disturbance=std::move(checked);}
                        {std::lock_guard<std::mutex> lock(mutex_);gain_hold_for_assessment_locked(stage.gains);}
                    });
                }
                tune_hold(stage,true);run_drag();
                };
                bool passing=false;
                run_measurement("compliant-passing-purpose",stage.gains,[&] {
                    passing=gain_passing_purpose_measurement(stage.gains,JointGainProfile::COMPLIANT,
                        verification_reserve("compliant-disturbance"),purpose_disturbance_cost(stage.gains),"compliant",[&](std::size_t anchor,double remaining_path) {
                            pending_anchor_return_s=remaining_path;
                            run_measurement("compliant-local-correction",stage.gains,[&]{gain_refine_taught_segment(stage.gains,JointGainProfile::COMPLIANT,stage.native,verification_reserve(),"compliant",anchor);});
                            soft_purpose_work();pending_anchor_return_s=0;
                        },&stage.native);
                });
                if(!passing) {
                    run_measurement("compliant-purpose-visit",stage.gains,[&]{compliant_anchor=enter_anchor(stage.gains,"compliant",purpose_disturbance_cost(stage.gains));});
                    if(compliant_anchor.ready)soft_purpose_work();
                }
                run_measurement("compliant-purpose-return",stage.gains,[&]{leave_anchor(stage.gains,compliant_anchor,"compliant");});
                if(!compliant_anchor.ready)run_measurement(stage.category+"-frozen",stage.gains,[&]{
                    stage.native=gain_trajectory_trial(stage.gains,JointGainProfile::COMPLIANT,false,false,"compliant-frozen","trajectory_frozen");
                    compliant_path_scheduled=stage.native.completed;
                });
                stage.validated=stage.native.passed && stage.disturbance.passed && stage.hold_disturbance.passed && stage.hold.validated && serial_arm::calibration_replay_supervisor::within_compliant_bounds(stage.gains.kp,stage.gains.kd,bounds);
                stage.available=stage.native.available && stage.disturbance.available && stage.hold_disturbance.available && stage.hold.available;
                stage.error=stage.available?"":"compliant_measurement_incomplete";
                });
            }
            if(want("servo_tracking")) {
                model_calibration_gain_stage_.store(2);auto& stage=stage_for("servo_tracking");
                run_measurement(stage.category,stage.gains,[&] {
                stage.error="measurement_in_progress";stage.gains=best;stage.native=native;stage.disturbance=native_disturbance;stage.identification_models=native_models;
                if(gain_admittance_required_) {
                    {std::lock_guard<std::mutex> lock(mutex_);if(!robot_.set_servo_tracking_gains(best))throw std::runtime_error("cannot install session servo candidate");}
                    stage.admittance=gain_trajectory_trial(best,JointGainProfile::SERVO_TRACKING,true,true,"admittance-frozen","trajectory_frozen");
                }
                stage.validated=stage.native.passed && stage.disturbance.passed && (!gain_admittance_required_ || stage.admittance.passed);
                stage.available=stage.native.available && stage.disturbance.available && (!gain_admittance_required_ || stage.admittance.available);
                stage.error=stage.available?"":"servo_measurement_incomplete";
                });
            }
            if(want("drag") && !drag_attempted) {
                auto drag_anchor=enter_anchor(best,"drag",30);
                if(drag_anchor.ready){run_drag();leave_anchor(best,drag_anchor,"drag");}
            }
        } catch(...) {
            interrupted=std::current_exception();
            bool budget_stop=false,endpoint_stop=false,evidence_stop=false;
            try {std::rethrow_exception(interrupted);}catch(const GainBudgetReached&){budget_stop=true;}catch(const GainEndpointNotReady&){endpoint_stop=true;}catch(const GainQualificationFailure& error){evidence_stop=true;gain_add_warning(std::string("identification_insufficient:")+error.what());}catch(const std::exception& error){for(auto& stage:stages)if(stage.error=="measurement_in_progress")stage.error=error.what();}catch(...){}
            budget_stop=budget_stop && machine_robot_state()==RobotState::ACTIVE && !model_calibration_cancel_.load() && !model_calibration_pause_.load() && !model_calibration_fault_interrupted_.load();
            endpoint_stop=endpoint_stop && machine_robot_state()==RobotState::ACTIVE && !model_calibration_cancel_.load() && !model_calibration_pause_.load() && !model_calibration_fault_interrupted_.load();
            evidence_stop=evidence_stop && machine_robot_state()==RobotState::ACTIVE && !model_calibration_cancel_.load() && !model_calibration_pause_.load() && !model_calibration_fault_interrupted_.load();
            if(budget_stop){gain_add_warning("time_budget_reached");interrupted=nullptr;for(auto& stage:stages)if(stage.error=="measurement_in_progress")stage.error="time_budget_reached";}
            if(evidence_stop){interrupted=nullptr;for(auto& stage:stages)if(stage.error=="measurement_in_progress")stage.error="identification_insufficient";}
            if(endpoint_stop){gain_add_warning("endpoint_alignment_required");interrupted=nullptr;for(auto& stage:stages)if(stage.error=="measurement_in_progress")stage.error="endpoint_alignment_required";}
            gain_probe_end(budget_stop?"time_budget_reached":endpoint_stop?"endpoint_alignment_required":model_calibration_cancel_.load()?"cancelled":"failed");
            if(machine_robot_state()==RobotState::ACTIVE)(void)set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD);
        }
        for(auto& stage:stages) {
            // Availability describes the parameters actually measured in the
            // contexts this purpose applies. Missing independent disturbance
            // evidence prevents qualification; it does not erase a completed
            // tracking/hold candidate after a later stage is interrupted.
            if(stage.category=="compliant_drag") {
                stage.available=stage.drag && stage.drag->available;
                stage.qualification_complete=stage.available;
            } else {
                const bool holding=stage.category=="rigid_tracking" || stage.category=="compliant_tracking";
                const bool trimmed=stage.compensation && stage.compensation->enabled;
                const bool hold_available=trimmed?(stage.compensated_hold && stage.compensated_hold->available):stage.hold.available;
                stage.available=stage.native.available && (!holding || hold_available) &&
                    (stage.category!="servo_tracking" || !gain_admittance_required_ || stage.admittance.available);
                stage.qualification_complete=stage.available && gain_direct_response_complete(stage.disturbance) && (!holding || gain_direct_response_complete(stage.hold_disturbance));
            }
            stage.validated=false; // Compatibility field; new measurements are reference-only.
            if(!stage.disturbance.available && stage.category!="compliant_drag")stage.warnings.push_back("disturbance_measurement_incomplete");
            if((stage.category=="rigid_tracking" || stage.category=="compliant_tracking") && !stage.hold_disturbance.available)
                stage.warnings.push_back("hold_disturbance_measurement_incomplete");
            if(!stage.available && !interrupted)stage.warnings.push_back(stage.error.empty()?"measurement_incomplete":stage.error);
            if(!interrupted || stage.available)stage.error.clear();
            for(const auto& warning:stage.warnings)gain_add_warning(stage.category+":"+warning);
        }
        const bool all_qualification_complete=!stages.empty() && std::all_of(stages.begin(),stages.end(),[](const auto& stage){return stage.qualification_complete;});
        const bool all_parameter_measurements_complete=!stages.empty() && std::all_of(stages.begin(),stages.end(),[](const auto& stage){return stage.available;});
        if(!interrupted && !all_parameter_measurements_complete && budget_prevented_measurement)gain_budget_exhausted_.store(true);
        const bool review_ready=std::any_of(stages.begin(),stages.end(),[](const auto& stage){return stage.available;});
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_available_=review_ready;}
        const bool approved=!interrupted && !stages.empty() && std::all_of(stages.begin(),stages.end(),[](const auto& s){return s.validated;});
        {
            std::lock_guard<std::mutex> lock(model_calibration_mutex_);
            gain_termination_kind_=model_calibration_fault_interrupted_.load()?"execution_fault":
                model_calibration_cancel_.load()?"cancelled":gain_budget_exhausted_.load()?"budget_exhausted":
                interrupted?"execution_fault":all_parameter_measurements_complete?"measured_reference_complete":"insufficient_measurement";
            if(gain_termination_kind_=="cancelled")gain_termination_reason_="stop_requested";
            else if(interrupted) {
                try{std::rethrow_exception(interrupted);}catch(const std::exception& error){gain_termination_reason_=error.what();}catch(...){gain_termination_reason_="unknown_execution_error";}
                if(model_calibration_fault_interrupted_.load() && !model_calibration_error_.empty())gain_termination_reason_=model_calibration_error_;
            } else {
                gain_termination_reason_=all_parameter_measurements_complete?"reference_analysis_available":gain_warnings_.empty()?"measurement_incomplete":gain_warnings_.front();
                if(!approved) {
                    // An actual stopping condition is more specific than a
                    // planning warning emitted before measurements began.
                    const auto evidence=std::find_if(gain_warnings_.rbegin(),gain_warnings_.rend(),[](const std::string& value){return value.rfind("identification_insufficient:",0)==0;});
                    if(evidence!=gain_warnings_.rend())gain_termination_reason_=*evidence;
                    else if(gain_budget_exhausted_.load())gain_termination_reason_="time_budget_reached";
                    else if(std::find(gain_warnings_.begin(),gain_warnings_.end(),"endpoint_alignment_required")!=gain_warnings_.end())gain_termination_reason_="endpoint_alignment_required";
                }
            }
        }
        std::string termination_kind,termination_json;
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);termination_kind=gain_termination_kind_;termination_json=gain_termination_json_locked();}
        try {
        std::ofstream out(std::filesystem::path(model_calibration_directory_)/"gain-candidate.json");
        if(!out)throw std::runtime_error("cannot save measured gain candidate");
        out<<std::setprecision(17);
        const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
        const auto gains_json=[&](const JointImpedanceGains& value){out<<"{\"kp\":";machine_json_joint_vector(out,value.kp);out<<",\"kd\":";machine_json_joint_vector(out,value.kd);out<<'}';};
        const auto hold_json=[&](const GainHoldResult& h){out<<"{\"validated\":"<<"false"<<",\"available\":"<<(h.available?"true":"false")<<",\"completed\":"<<(h.completed?"true":"false")<<",\"fresh_samples\":";machine_json_vector(out,h.fresh_samples);out<<",\"reason\":\""<<machine_json_escape(h.reason)<<"\",\"observer_samples\":"<<h.observer_samples<<",\"integral_samples\":"<<h.integral_samples<<",\"samples\":"<<h.samples<<",\"limited\":"<<(h.limited?"true":"false")<<",\"worst_error_rad\":"<<h.worst_error<<",\"worst_drift_rad\":"<<h.worst_span<<",\"worst_velocity_rad_s\":"<<h.worst_velocity<<",\"raw_peak_velocity_rad_s\":"<<h.raw_worst_velocity<<",\"position_span_rad\":"<<h.worst_span<<",\"velocity_source\":\"position_window_0.6s\",\"target_source\":\""<<h.target_source<<"\",\"target_pos\":";machine_json_joint_vector(out,h.target);out<<",\"hold_evaluation_scope\":\"fixed_interior_target_error_and_drift_not_trajectory_endpoint_arrival\",\"controls_known\":"<<(h.controls_known?"true":"false")<<",\"reference_fixed\":"<<(h.reference_fixed?"true":"false")<<",\"independent_input_zero\":"<<(h.independent_input_zero?"true":"false")<<",\"entry_actual_pos\":";machine_json_joint_vector(out,h.entry_actual);out<<",\"transient_peak_error_rad\":"<<h.transient_peak_error<<",\"transient_excursion_rad\":"<<h.transient_excursion<<",\"settling_window_s\":"<<h.settling_window_s<<",\"duration_s\":"<<h.duration_s<<",\"record_path\":\""<<machine_json_escape(h.record_path)<<"\",\"actual_integral_enabled\":"<<(h.actual_compensation.enabled?"true":"false")<<",\"actual_integral_gain\":";machine_json_joint_vector(out,h.actual_compensation.integral_gain);out<<"}";};
        const auto validation_json=[&](const ServoTrial& trial) {
            out<<"{\"acceptance_scope\":\"measured_response_reference_only_v1\",\"reference_only\":true,\"accuracy_thresholds_enabled\":false,\"dynamic_error_role\":\"tuning_and_diagnostics_only\",\"method\":\"demonstrated_trajectory_tracking\",\"reference_schedule\":\"control_cycle_monotonic\",\"record_path\":\""<<machine_json_escape(trial.record_path)<<"\",\"required\":true,\"completed\":"<<(trial.completed?"true":"false")<<",\"available\":"<<(trial.available?"true":"false")<<",\"status\":\""<<(trial.available?"measured_reference":"not_measured")<<"\",\"independent\":"<<(trial.trajectory_replay?"true":"false")<<",\"reverse\":"<<(trial.reverse?"true":"false")<<",\"playback_rate\":"<<trial.playback_rate<<",\"duration_s\":"<<trial.duration_s<<",\"measured_duration_s\":"<<(trial.rows.empty() || trial.rows[0].empty()?0:trial.rows[0].back().time_s)<<",\"origin\":";machine_json_joint_vector(out,trial.center);out<<",\"probe_plans\":[";
            for(std::size_t i=0;i<trial.plans.size();++i){if(i)out<<',';const auto& plan=trial.plans[i];out<<"{\"amplitude_rad\":"<<plan.amplitude_rad<<",\"reference_bias_rad\":"<<plan.reference_bias_rad<<",\"frequencies_hz\":null,\"duration_s\":"<<plan.probe_duration_s<<'}';}out<<"],\"validated\":"<<"false"<<",\"online_adaptation\":"<<(trial.online_adaptation?"true":"false")<<",\"continuous\":"<<(std::all_of(trial.metrics.begin(),trial.metrics.end(),[](const auto& m){return m.continuous;})?"true":"false")<<",\"metrics\":[";
            for(std::size_t i=0;i<trial.metrics.size();++i){if(i)out<<',';const auto& m=trial.metrics[i];out<<"{\"samples\":"<<m.samples<<",\"measurement_samples\":"<<m.measurement_samples<<",\"rms_error_rad\":"<<m.rms_error<<",\"peak_error_rad\":"<<m.peak_error<<",\"velocity_rms_rad_s\":"<<m.velocity_rms<<",\"overshoot_rad\":"<<m.overshoot<<",\"settling_s\":"<<m.settling_s<<",\"tail_error_rad\":"<<m.tail_error_rad<<",\"limited_samples\":"<<m.limited_samples<<",\"oscillating\":"<<(m.oscillating?"true":"false")<<",\"observable\":"<<(m.observable?"true":"false")<<",\"reference_span_rad\":"<<m.reference_span<<",\"actual_span_rad\":"<<m.actual_span<<",\"response_rms_ratio\":"<<m.response_rms_ratio;
                gain_endpoint_metrics_json(out,m);
                const auto& motion=trial.motion[i];out<<",\"motion_observation_basis\":\"reference_aligned_travel_v1\",\"motion_qualified\":"<<(motion.qualified?"true":"false")
                    <<",\"bidirectional\":"<<(motion.bidirectional?"true":"false")<<",\"initial_error_rad\":"<<motion.initial_error_rad
                    <<",\"actual_positive_travel_rad\":"<<motion.positive_travel_rad<<",\"actual_negative_travel_rad\":"<<motion.negative_travel_rad
                    <<",\"motion_coherence\":"<<motion.coherence<<",\"minimum_motion_span_rad\":"<<motion.minimum_motion_span_rad
                    <<",\"oscillation_source\":\"fresh_position_residual_and_tail_hysteresis\",\"tail_position_reversals\":"<<trial.trajectory_assessments[i].tail_position_reversals<<",\"tuning\":";gain_tuning_json(out,trial.trajectory_assessments[i].tuning);gain_path_motion_json(out,trial.trajectory_assessments[i]);out<<'}';}out<<"]}";
        };
        const auto write_stage=[&](const Stage& stage) {
            if(all)out<<"\"validated\":"<<"false"<<',';
            if(all){out<<"\"available\":"<<(stage.available?"true":"false")<<",\"warnings\":";gain_json_warnings(out,stage.warnings);out<<',';}
            out<<"\"qualification_complete\":"<<(stage.qualification_complete?"true":"false")
                <<",\"parameter_measurement_complete\":"<<(stage.available?"true":"false")
                <<",\"input_measurement_complete\":"<<((stage.category=="compliant_drag"?(stage.drag && stage.drag->available):stage.disturbance.available)?"true":"false")
                <<",\"response_measurement_complete\":"<<((stage.category=="compliant_drag"?(stage.drag && std::all_of(stage.drag->release.begin(),stage.drag->release.end(),[](const auto& axis){return axis.measurement_complete;})):gain_direct_response_complete(stage.disturbance))?"true":"false")<<',';
            if(!stage.error.empty())out<<"\"error\":\""<<machine_json_escape(stage.error)<<"\",";
            out<<"\"kp\":";machine_json_joint_vector(out,stage.gains.kp);out<<",\"kd\":";machine_json_joint_vector(out,stage.gains.kd);
            out<<",\"qualification_scope\":{\"trajectory\":\"demonstration_trajectory\",\"external_tip\":\"pending\",\"disturbance_release\":\"independent_joint_torque_short_test\",\"contact\":\"not_tested\"}";
            if(stage.category=="compliant_drag") {
                const auto& drag=*stage.drag;
                out<<",\"drag_validation\":{\"validated\":"<<"false"<<",\"available\":"<<(drag.available?"true":"false")<<",\"completed\":"<<(drag.completed?"true":"false")
                    <<",\"source\":\"actual_zero_stiffness_quiet_then_isolated_torque_release\",\"velocity_source\":\"position_window_0.6s\",\"raw_velocity_role\":\"diagnostic_only_core_safety_unchanged\",\"samples\":"<<drag.samples
                    <<",\"record_path\":\""<<machine_json_escape(drag.record_path)<<"\",\"decay_s\":";machine_json_joint_vector(out,drag.decay_s);
                out<<",\"initial_velocity_rad_s\":";machine_json_joint_vector(out,drag.initial_velocity);
                out<<",\"peak_velocity_rad_s\":";machine_json_joint_vector(out,drag.peak_velocity);
                out<<",\"raw_peak_velocity_rad_s\":";machine_json_joint_vector(out,drag.raw_peak_velocity);
                out<<",\"final_velocity_rad_s\":";machine_json_joint_vector(out,drag.final_velocity);
                out<<",\"axes\":";gain_drag_axes_json(out,drag);out<<'}';return;
            }
            out<<",\"native_validation\":";validation_json(stage.native);
            out<<",\"observed_joint_min\":";machine_json_joint_vector(out,stage.native.min_position);out<<",\"observed_joint_max\":";machine_json_joint_vector(out,stage.native.max_position);
            if(stage.category=="rigid_tracking" || stage.category=="compliant_tracking") {
                out<<",\"hold_gains\":";gains_json(stage.hold_gains);out<<",\"hold_verification\":";hold_json(stage.hold);
                if(stage.compensation) {
                    const auto& c=*stage.compensation;out<<",\"hold_compensation\":{\"enabled\":"<<(c.enabled?"true":"false")<<",\"integral_gain\":";machine_json_joint_vector(out,c.integral_gain);out<<",\"max_torque\":";machine_json_joint_vector(out,c.max_torque);out<<",\"max_rate\":";machine_json_joint_vector(out,c.max_rate);out<<",\"contact_threshold\":";machine_json_joint_vector(out,c.contact_threshold);out<<",\"velocity_threshold\":"<<c.velocity_threshold<<",\"error_threshold\":"<<c.error_threshold<<'}';if(stage.compensated_hold){out<<",\"hold_compensation_verification\":";hold_json(*stage.compensated_hold);}
                }
            }
            if(stage.category=="compliant_tracking") {
                out<<",\"tested_speed_fraction\":1.0,\"compliant_separation_validated\":false,\"compliant_design_scope\":\"measured_yield_recovery_not_fixed_rigid_gain_ratio\",\"rigid_reference_kp\":";machine_json_joint_vector(out,compliant_rigid_reference.kp);out<<",\"rigid_reference_kd\":";machine_json_joint_vector(out,compliant_rigid_reference.kd);
            }
            if(stage.category=="servo_tracking") {out<<",\"admittance_required\":"<<(gain_admittance_required_?"true":"false")<<",\"admittance_validation\":";if(gain_admittance_required_)validation_json(stage.admittance);else out<<"{\"required\":false,\"status\":\"not_requested\",\"validated\":false}";}
            if(stage.category=="compliant_tracking") {out<<",\"compliant_use_verification\":{\"record_path\":\""<<stage.disturbance.record_path<<"\",\"axes\":";gain_compliant_axes_json(out,stage.disturbance);out<<'}';}
            out<<",\"response_model\":[";for(std::size_t i=0;i<stage.identification_models.size();++i){if(i)out<<',';gain_response_json(out,stage.identification_models[i]);}out<<"],\"response_model_status\":\"short_mit_experiments\",\"disturbance_verification\":{\"method\":\"isolated_smooth_torque_pulse\",\"model_identified\":false,\"direct_measurement_complete\":"<<(gain_direct_response_complete(stage.disturbance)?"true":"false")<<",\"axes\":";gain_compliant_axes_json(out,stage.disturbance);out<<",\"completed\":"<<(stage.disturbance.completed?"true":"false")<<",\"available\":"<<(stage.disturbance.available?"true":"false")<<",\"validated\":"<<"false"<<",\"record_path\":\""<<stage.disturbance.record_path<<"\"},\"hold_disturbance_verification\":{\"shared_with\":\"disturbance_verification\",\"method\":\"isolated_smooth_torque_pulse\",\"model_identified\":false,\"available\":"<<(stage.hold_disturbance.available?"true":"false")<<",\"validated\":"<<"false"<<",\"record_path\":\""<<stage.hold_disturbance.record_path<<"\"}";
            out<<",\"observer_model\":[";
            const auto& measured=stage.category=="servo_tracking" && gain_admittance_required_?stage.admittance:stage.native;
            for(std::size_t axis=0;axis<measured.metrics.size();++axis){if(axis)out<<',';const auto& m=measured.metrics[axis];out<<"{\"bias_nm\":";if(m.residual_samples>=30)number(m.residual_bias);else out<<"null";out<<",\"noise_rms_nm\":";if(m.residual_samples>=30)number(m.residual_noise_rms);else out<<"null";out<<",\"samples\":"<<m.residual_samples<<",\"source\":\"demonstration_no_contact_residual\"}";}out<<']';
        };
        out<<"{\"reference_only\":true,\"accuracy_thresholds_enabled\":false,\"schema\":\"serial_arm_gain_candidate\",\"task_id\":\""<<machine_json_escape(model_calibration_task_id_)<<"\",\"urdf_fingerprint\":\""<<machine_json_escape(model_calibration_urdf_fingerprint_)<<"\",\"validated\":"<<"false"<<",\"category\":\""<<(all?"all":stages.front().category)<<"\",\"joint_names\":[";
        for(std::size_t i=0;i<n;++i){if(i)out<<',';out<<'"'<<machine_json_escape(cfg_.joint_names[i])<<'"';}out<<"],\"passes\":"<<model_calibration_adaptation_passes_.load()<<",\"revisions\":"<<model_calibration_adaptation_revisions_.load()<<",\"validation_scope\":\"independent_demonstration_trajectory\",\"load_description\":\""<<machine_json_escape(model_calibration_options_.load_description)<<"\",\"seed_posture\":";machine_json_joint_vector(out,native.center.empty()?stages.front().native.center:native.center);
        if(!gain_control_context_.empty())out<<",\"control_context\":"<<gain_control_context_;
        const auto& r=gain_requirements_;
        out<<",\"gain_requirements\":{\"acceptance_scope\":\"measured_response_reference_only_v1\",\"accuracy_thresholds_enabled\":false,\"legacy_accuracy_fields_ignored\":true,\"tail_s\":"<<r.tail_s<<",\"amplitude_rad\":"<<r.amplitude_rad<<",\"local_range_rad\":"<<r.local_range_rad<<"}";
        out<<",\"probe_origin\":";machine_json_joint_vector(out,gain_probe_origin_);
        out<<",\"established_probe_plans\":[";for(std::size_t i=0;i<gain_plans_.size();++i){if(i)out<<',';const auto& plan=gain_plans_[i];out<<"{\"amplitude_rad\":"<<plan.amplitude_rad<<",\"reference_bias_rad\":"<<plan.reference_bias_rad<<",\"frequencies_hz\":["<<plan.frequencies_hz[0]<<','<<plan.frequencies_hz[1]<<','<<plan.frequencies_hz[2]<<"],\"duration_s\":"<<plan.probe_duration_s<<'}';}out<<']';
        out<<",\"model_initialization\":{\"reference_only\":true,\"accuracy_thresholds_enabled\":false,\"source\":\""<<gain_model_initialization_.source<<"\",\"measured\":false,\"inertia_min\":";machine_json_joint_vector(out,gain_model_initialization_.inertia_min);out<<",\"inertia_max\":";machine_json_joint_vector(out,gain_model_initialization_.inertia_max);out<<",\"frequency_hz\":";machine_json_joint_vector(out,gain_model_initialization_.frequency_hz);out<<",\"kp\":";machine_json_joint_vector(out,gain_seed_.kp);out<<",\"kd\":";machine_json_joint_vector(out,gain_seed_.kd);out<<",\"constraints\":[";for(std::size_t i=0;i<gain_model_initialization_.constraints.size();++i){if(i)out<<',';gain_json_warnings(out,gain_model_initialization_.constraints[i]);}out<<']';
        const auto model_field=[&](const char* key,const JointVector& value){out<<",\""<<key<<"\":";machine_json_joint_vector(out,value);};
        model_field("unconstrained_kp",gain_model_initialization_.unconstrained_kp);model_field("unconstrained_kd",gain_model_initialization_.unconstrained_kd);
        model_field("pre_modal_kp",gain_model_initialization_.pre_modal_kp);model_field("pre_modal_kd",gain_model_initialization_.pre_modal_kd);
        model_field("working_kp_max",gain_model_initialization_.working_kp_max);model_field("working_kd_max",gain_model_initialization_.working_kd_max);
        model_field("reference_error_bound_rad",gain_model_initialization_.reference_error_bound_rad);model_field("velocity_error_bound_rad",gain_model_initialization_.velocity_error_bound_rad);
        model_field("task_frequency_hz",gain_model_initialization_.task_frequency_hz);model_field("nominal_mode_frequency_hz",gain_model_initialization_.nominal_mode_frequency_hz);model_field("nominal_mode_damping_ratio",gain_model_initialization_.nominal_mode_damping_ratio);
        model_field("bootstrap_damping_inertia_min",gain_model_initialization_.bootstrap_damping_inertia_min);
        out<<",\"bootstrap_observation_interval_s\":"<<gain_model_initialization_.bootstrap_observation_interval_s
            <<",\"bootstrap_feedback_age_s\":"<<gain_model_initialization_.bootstrap_feedback_age_s
            <<",\"modeled_fastest_damping_rate_s_inv\":"<<gain_model_initialization_.modeled_fastest_damping_rate_s_inv;
        model_field("protocol_kp_max",cfg_.safety.limits.max_kp);model_field("protocol_kd_max",cfg_.safety.limits.max_kd);model_field("protocol_kp_quantum",cfg_.safety.limits.kp_resolution);model_field("protocol_kd_quantum",cfg_.safety.limits.kd_resolution);
        out<<",\"modal_adjustments\":";gain_json_warnings(out,gain_model_initialization_.modal_adjustments);
        out<<",\"physical_prior\":\"urdf_geometry_mass_com_inertia\",\"response_demand_source\":\"explicit_nominal_2Hz_damping_0.7_design_and_URDF_mass_matrix\",\"frequency_role\":\"unconstrained_design_lower_bound_not_measured_joint_bandwidth\",\"nominal_mode_basis\":\"time_weighted_mean_URDF_path_mass_matrix_with_executable_complete_gain_group\",\"damping_design_method\":\"double_diagonalization_diagonal_projection\",\"nominal_damping_role\":\"projection_of_independent_MIT_derivative_channels_not_exact_full_matrix_modal_damping\",\"initialization_strategy\":\"URDF_double_diagonalization_start_then_online_taught_path_local_IV_ARX_PD_updates_and_frozen_reference_measurement\",\"constraint_role\":\"execution_guards_not_mechanical_identification\",\"working_domain_scope\":\"initial_experiment_joint_effort_envelope_not_global_stability_limit\",\"legacy_user_pd_limits_ignored\":true}";
        out<<",\"task_model_demand\":{\"executed\":false,\"measured\":false,\"valid\":"<<(gain_task_model_error_.empty()?"true":"false")<<",\"scope\":\"offline_path_performance_demand_not_starting_gains\"";
        model_field("unconstrained_kp",gain_task_model_initialization_.unconstrained_kp);model_field("unconstrained_kd",gain_task_model_initialization_.unconstrained_kd);
        model_field("constrained_design_kp",gain_task_model_initialization_.gains.kp);model_field("constrained_design_kd",gain_task_model_initialization_.gains.kd);
        model_field("task_frequency_hz",gain_task_model_initialization_.task_frequency_hz);
        out<<",\"reason\":\""<<machine_json_escape(gain_task_model_error_)<<"\"}";
        std::vector<std::string> guards;{std::lock_guard<std::mutex> lock(mutex_);guards=gain_guard_records_;}
        out<<",\"response_guard_events\":[";for(std::size_t i=0;i<guards.size();++i){if(i)out<<',';out<<guards[i];}out<<']';
        out<<",\"experiments\":[";for(std::size_t i=0;i<gain_experiment_records_.size();++i){if(i)out<<',';out<<gain_experiment_records_[i];}out<<"],\"candidate_comparisons\":[";for(std::size_t i=0;i<gain_candidate_records_.size();++i){if(i)out<<',';out<<gain_candidate_records_[i];}out<<']';
        out<<",\"seed_source\":\""<<machine_json_escape(gain_seed_source_)<<"\",\"seed_kp\":";machine_json_joint_vector(out,gain_seed_.kp);
        out<<",\"seed_kd\":";machine_json_joint_vector(out,gain_seed_.kd);
        out<<",\"measurement_complete\":"<<(all_parameter_measurements_complete?"true":"false")<<",\"review_ready\":"<<(review_ready?"true":"false")
            <<",\"review_required\":"<<(review_ready?"true":"false")<<",\"nonfatal_stage_warnings\":"<<nonfatal_warnings
            <<",\"execution_policy\":\"continue_available_measurements_until_stop_fault_or_original_budget\"";
        out<<",\"available\":"<<(review_ready?"true":"false")<<",\"partial\":"<<(!all_qualification_complete?"true":"false")<<",\"outcome\":\""<<termination_kind<<"\",\"warnings\":";gain_json_warnings(out,gain_warnings_);
        if(all)out<<",\"qualification_complete\":"<<(all_qualification_complete?"true":"false");
        out<<",\"gain_tuning_mode\":\""<<gain_tuning_mode_<<"\",\"target_budget_s\":"<<(model_calibration_gain_mode_=="all"?18:12)*gain_budget_source_.duration_s<<",\"actual_elapsed_s\":"<<(gain_deadline_?gain_time_budget_s()-std::chrono::duration<double>(*gain_deadline_-Robot::Clock::now()).count():0)<<",\"gain_protocol\":\"taught_trajectory\",\"time_budget_s\":"<<gain_time_budget_s()
            <<",\"budget_source_duration_s\":"<<gain_budget_source_.duration_s<<",\"budget_source\":\""<<machine_json_escape(gain_budget_source_.source)
            <<"\",\"budget_source_task_id\":\""<<machine_json_escape(gain_budget_source_.task_id)<<"\",\"budget_multiplier\":"<<(model_calibration_gain_mode_=="all"?24:15)
            <<",\"path_pass_limit\":"<<gain_schedule_.full_pass_limit<<",\"path_passes\":"<<gain_path_pass_current_<<",\"budget_exhausted\":"<<(gain_budget_exhausted_.load()?"true":"false");
        out<<",\"admittance_parameters\":{\"mass\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.mass);
        out<<",\"damping\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.damping);out<<",\"stiffness\":";machine_json_joint_vector(out,cfg_.capability.admittance.controller.stiffness);
        out<<",\"joint_enabled\":";machine_json_vector(out,cfg_.capability.admittance.joint_enabled);out<<",\"observer_mode\":\""<<observer_mode_name(cfg_.capability.admittance.observer.mode)<<"\"}";
        if(all){out<<",\"targets\":{";for(std::size_t i=0;i<stages.size();++i){if(i)out<<',';out<<'"'<<stages[i].category<<"\":{";write_stage(stages[i]);out<<'}';}out<<'}';}else{out<<',';write_stage(stages.front());}
        out<<",\"termination\":"<<termination_json<<'}';out.flush();if(!out)throw std::runtime_error("gain candidate writing failed");
        model_calibration_adaptation_converged_.store(approved);
        } catch(...) {
            // File I/O must never replace the original cancellation or control fault
            if(interrupted)std::rethrow_exception(interrupted);
            throw;
        }
        if(interrupted)std::rethrow_exception(interrupted);
    }

    void gain_finalize_candidate_elapsed() const {
        const auto path=std::filesystem::path(model_calibration_directory_)/"gain-candidate.json";
        std::ifstream in(path);if(!in)return;std::string body((std::istreambuf_iterator<char>(in)),{});
        const std::string key="\"actual_elapsed_s\":";const auto begin=body.find(key);if(begin==std::string::npos)return;
        const auto value=begin+key.size(),end=body.find_first_of(",}",value);if(end==std::string::npos)return;
        const double elapsed=gain_deadline_?gain_time_budget_s()-std::chrono::duration<double>(*gain_deadline_-gain_finished_at_.value_or(Robot::Clock::now())).count():0;
        std::ostringstream number;number<<std::setprecision(17)<<elapsed;body.replace(value,end-value,number.str());
        std::string kind,termination;
        {std::lock_guard<std::mutex> lock(model_calibration_mutex_);kind=gain_termination_kind_;termination=gain_termination_json_locked();}
        const std::string outcome_key="\"outcome\":\"";const auto outcome=body.find(outcome_key);
        if(outcome!=std::string::npos) {
            const auto first=outcome+outcome_key.size(),last=body.find('"',first);
            if(last!=std::string::npos)body.replace(first,last-first,kind);
        }
        if(kind=="execution_fault") {
            const std::string validated_key="\"validated\":";
            const auto validated=body.find(validated_key);
            if(validated!=std::string::npos && body.compare(validated+validated_key.size(),4,"true")==0)
                body.replace(validated+validated_key.size(),4,"false");
        }
        if(kind!="qualified" && kind!="measured_not_qualified" && kind!="measured_reference_complete" &&
           body.find("\"validation_scope\":\"independent_demonstration_trajectory\"")==std::string::npos) {
            for(const auto* field:{"review_ready","review_required"}) {
                const auto marker=std::string("\"")+field+"\":";
                const auto position=body.find(marker);
                if(position!=std::string::npos && body.compare(position+marker.size(),4,"true")==0)
                    body.replace(position+marker.size(),4,"false");
            }
        }
        // The task writer owns this final top-level field. Refresh it after
        // cleanup too, so a late recording/exit fault cannot retain success.
        const std::string termination_key=",\"termination\":";
        const auto previous=body.rfind(termination_key);
        if(previous!=std::string::npos)body.resize(previous);
        else {const auto close=body.find_last_of('}');if(close==std::string::npos)throw std::runtime_error("invalid gain candidate during finalization");body.resize(close);}
        body+=termination_key+termination+'}';
        std::ofstream out(path);out<<body;if(!out)throw std::runtime_error("cannot finalize gain elapsed time");
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
                bool complete=false;
                {std::lock_guard<std::mutex> lock(model_calibration_mutex_);complete=gain_available_;}
                gain_probe_end(complete?"complete":"measurement_incomplete");
                {
                    std::lock_guard<std::mutex> lock(model_calibration_mutex_);
                    model_calibration_completed_units_=model_calibration_adaptation_passes_.load();
                    model_calibration_total_units_=model_calibration_completed_units_+(complete?0:1);
                    model_calibration_progress_=complete?1.0:double(model_calibration_completed_units_)/model_calibration_total_units_;
                }
                {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_finished_at_=Robot::Clock::now();}
                gain_finalize_candidate_elapsed();
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
            // two independently tagged dynamic speed pairs on the checked path
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
                // formal dynamics observations. Hardware failures still abort
                model_calibration_result_.reset();
                std::lock_guard<std::mutex> preview_lock(model_calibration_mutex_);
                model_calibration_preview_error_=gravity.error();
            }
            // Holdout validation is advisory for exported candidates. Never
            // abort data acquisition or suppress the candidate because of
            // an inaccurate initial URDF. Applying it to hardware is separate
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
            // Friction and full IDIM belong to isolated offline identification
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
                if(model_calibration_workflow_kind_=="gain") {
                    gain_available_=false;
                    model_calibration_adaptation_converged_.store(false);
                    gain_termination_kind_=model_calibration_fault_interrupted_.load()?"execution_fault":model_calibration_cancel_.load()?"cancelled":"execution_fault";
                    gain_termination_reason_=gain_termination_kind_=="cancelled"?"stop_requested":model_calibration_error_;
                }
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
            if(model_calibration_workflow_kind_=="gain"){
                {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_finished_at_=Robot::Clock::now();}
                try{gain_finalize_candidate_elapsed();}catch(const std::exception&){gain_add_warning("final_elapsed_record_write_failed");}
            }
            model_calibration_write_result_file();
        }
    }

    // A supervised, manual alignment mode: no generated autonomous path is
    // permitted here because the standalone Core has no live planning scene
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
        // the pose itself may still be far from the saved endpoint
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
        // A non-matching pose remains blocked from starting replay
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

    serial_arm::servo_tuning::TrajectorySchedule gain_automatic_schedule(double duration,const std::string& purpose,const std::string& tuning,double hard,bool admittance) const {
        serial_arm::servo_tuning::TrajectorySchedule p;p.deadline_s=hard;
        const bool all=purpose=="all",native=purpose!="compliant" && purpose!="drag",soft=all || purpose=="compliant";
        p.native_pass_duration_s=duration+gain_requirements_.tail_s+2.5;p.compliant_pass_duration_s=duration+gain_requirements_.tail_s+2.5;
        p.native_search_passes=native?(tuning=="fine"?2:1):0;p.compliant_search_passes=soft?(tuning=="fine"?2:1):0;
        p.full_pass_limit=(native?2+p.native_search_passes:0)+(soft?2+p.compliant_search_passes:0)+(admittance?1:0);
        const double short_cost=tuning=="fine"?10:8;
        p.estimated_total_s=3+6+(native?2*short_cost+(2+p.native_search_passes)*p.native_pass_duration_s:0)+(soft?2*short_cost+(2+p.compliant_search_passes)*p.compliant_pass_duration_s:0)+
            (all || purpose=="rigid"?24:0)+(soft?24:0)+(all || purpose=="drag"?30:0)+(admittance?p.native_pass_duration_s:0);
        p.fits=p.estimated_total_s<=hard;return p;
    }

    std::size_t gain_planned_windows() const {
        const bool all=model_calibration_gain_mode_=="all";
        return gain_schedule_.full_pass_limit+1+(all || model_calibration_gain_mode_=="rigid"?2:0)+(all || model_calibration_gain_mode_=="compliant"?1:0)+(all || model_calibration_gain_mode_=="drag"?2:0);
    }

    std::string machine_model_calibration_gain_plan(const YAML::Node& params) const {
        const auto mode=params && params["gain_mode"]?params["gain_mode"].as<std::string>():"all";
        const bool requested=cfg_.capability.admittance.enabled && (mode=="all" || mode=="servo_tracking");
        const double duration=model_calibration_trajectory_ && model_calibration_trajectory_->positions.size()>1?model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1):0;
        std::ostringstream out;out<<std::setprecision(17);
        const auto tuning=params && params["gain_tuning_mode"]?params["gain_tuning_mode"].as<std::string>():(mode=="all"?"quick":"fine");
        if((tuning!="quick" && tuning!="fine") || (tuning=="quick" && mode!="all") || (tuning=="fine" && mode=="all"))throw std::runtime_error("invalid automatic gain mode and purpose combination");
        const double target=(mode=="all"?18:12)*gain_budget_source_.duration_s;
        const double budget=(mode=="all"?24:15)*gain_budget_source_.duration_s;
        out<<"{\"gain_tuning_mode\":\""<<tuning<<"\",\"target_budget_s\":"<<target<<",\"gain_target_budget_s\":"<<target<<",\"hard_limit_s\":"<<budget<<",\"budget_s\":"<<budget<<",\"budget_source_duration_s\":"<<gain_budget_source_.duration_s
            <<",\"budget_source\":\""<<machine_json_escape(gain_budget_source_.source)<<"\",\"budget_source_task_id\":\""<<machine_json_escape(gain_budget_source_.task_id)
            <<"\",\"budget_multiplier\":"<<(mode=="all"?24:15);
        if(duration<=0 || budget<=0) {
            out<<",\"valid\":false,\"reason\":\"gain calibration requires a checked demonstration trajectory and original recorded duration\",\"estimated_duration_s\":0,\"trajectory_duration_s\":0,\"max_path_passes\":0,\"admittance_required\":false}";
            return out.str();
        }
        const auto plan=gain_automatic_schedule(duration,mode,tuning,budget,requested);
        out<<",\"valid\":"<<(plan.fits?"true":"false")<<",\"fits\":"<<(plan.fits?"true":"false")<<",\"reason\":\""<<(plan.fits?"":"time_budget_insufficient_for_complete_measurement")<<"\",\"warnings\":";
        gain_json_warnings(out,plan.fits?std::vector<std::string>{}:std::vector<std::string>{"time_budget_may_limit_measurements"});
        out<<",\"estimated_duration_s\":"<<plan.estimated_total_s<<",\"trajectory_duration_s\":"<<duration<<",\"max_path_passes\":"<<plan.full_pass_limit<<",\"admittance_required\":"<<(requested?"true":"false")<<'}';
        return out.str();
    }

    void machine_model_calibration_start(const YAML::Node& params) {
        const auto request_started=Robot::Clock::now();
        model_calibration_require_task(params);
        if(model_calibration_phase_.load() != ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION || !model_calibration_trajectory_)
            throw std::runtime_error("model calibration is not waiting for replay confirmation");
        if(model_calibration_alignment_active_.load())
            throw std::runtime_error("finish manual endpoint alignment and hold before confirming replay");
        if(model_calibration_worker_.joinable())throw std::runtime_error("model calibration worker is already running");
        auto workflow = params && params["workflow"] ? params["workflow"].as<std::string>() : "gravity";
        if(workflow!="gravity" && workflow!="gain")
            throw std::runtime_error("invalid calibration workflow target");
        if(workflow=="gain" && std::any_of(dynamics_.get_info().joint_types.begin(),dynamics_.get_info().joint_types.end(),[](const auto& type){return type=="prismatic";}))
            throw std::runtime_error("automatic actuator gain calibration currently supports rotary joints only; prismatic axes require per-axis linear probe and acceptance requirements");
        auto gain_mode=params && params["gain_mode"] ? params["gain_mode"].as<std::string>() : "all";
        if(gain_mode=="identification" || gain_mode=="identification_tracking" || gain_mode=="servo")gain_mode="servo_tracking";
        if(gain_mode=="compliant_drag")gain_mode="drag";
        if(workflow=="gain" && gain_mode!="rigid" && gain_mode!="compliant" && gain_mode!="drag" && gain_mode!="servo_tracking" && gain_mode!="all")throw std::runtime_error("unknown gain calibration purpose");
        const auto tuning_mode=params && params["gain_tuning_mode"]?params["gain_tuning_mode"].as<std::string>():(gain_mode=="all"?"quick":"fine");
        if(workflow=="gain" && (tuning_mode!="quick" && tuning_mode!="fine"))throw std::runtime_error("unknown gain tuning mode");
        if(workflow=="gain" && ((tuning_mode=="quick" && gain_mode!="all") || (tuning_mode=="fine" && gain_mode=="all")))throw std::runtime_error("quick calibration requires four uses; fine calibration requires one selected use");
        auto requirements=serial_arm::servo_tuning::Requirements{};
        if(params["gain_requirements"]) {
            const auto r=params["gain_requirements"];
            const auto read=[&](const char* key,double& value,double maximum){if(!r[key])return;value=r[key].as<double>();if(!std::isfinite(value)||value<=0||value>maximum)throw std::runtime_error(std::string("invalid gain requirement: ")+key);};
            // Legacy accuracy/RMS/settling targets remain readable in old
            // tasks, but a new task does not use them as scoring standards.
            read("amplitude_rad",requirements.amplitude_rad,.02);
            read("local_range_rad",requirements.local_range_rad,.04);
        }
        if(workflow=="gain" && (!params["gain_trajectory_confirmed"] || !params["gain_trajectory_confirmed"].as<bool>()))
            throw std::runtime_error("explicit demonstrated trajectory replay confirmation is required");
        if(workflow=="gain" && (!params["gain_motion_range_confirmed"] || !params["gain_motion_range_confirmed"].as<bool>()))
            throw std::runtime_error("explicit gain local motion range confirmation is required");
        if(!params["released"] || !params["released"].as<bool>())throw std::runtime_error("explicit released confirmation is required");
        std::optional<GainEndpointObservation> gain_endpoint;
        bool gain_initial_reverse=true;
        if(workflow=="gain") {
            {std::lock_guard<std::mutex> lock(mutex_);
                if(last_output_) {
                    const auto choice=serial_arm::calibration_recovery::select_endpoint(last_output_->joint_state.pos,last_output_->joint_state.vel,
                        model_calibration_trajectory_->positions.front(),model_calibration_trajectory_->positions.back(),0,dynamics_.get_info().joint_types);
                    if(choice.valid)gain_initial_reverse=!choice.reverse_recorded_path;
                }
            }
            gain_endpoint=gain_endpoint_observation(false,gain_initial_reverse?model_calibration_trajectory_->positions.back():model_calibration_trajectory_->positions.front());
        }
        if(!model_calibration_source_task_id_.empty() || gain_endpoint) {
            // The first pass traverses the path in reverse, starting at its last
            // sample. A parked arm can be far away: NEVER jump to that target
            std::lock_guard<std::mutex> lock(mutex_);
            if(robot_.get_state() != RobotState::ACTIVE || !last_output_ || feedback_time_ns_ == 0)
                throw std::runtime_error("cannot confirm imported replay without fresh ACTIVE robot feedback");
            const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
            const double age_ms = now_ns >= feedback_time_ns_ ? double(now_ns - feedback_time_ns_) / 1.0e6 : std::numeric_limits<double>::infinity();
            const auto& start = gain_endpoint && !gain_initial_reverse?model_calibration_trajectory_->positions.front():model_calibration_trajectory_->positions.back();
            const auto& actual = last_output_->joint_state;
            const auto choice = serial_arm::calibration_recovery::select_endpoint(gain_endpoint?gain_endpoint->position:actual.pos,
                gain_endpoint?gain_endpoint->velocity:actual.vel,start,start,gain_endpoint?std::max(age_ms,gain_endpoint->age_ms):age_ms,dynamics_.get_info().joint_types);
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
        if(workflow=="gain") {
            const double duration=model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1);
            const double budget=(gain_mode=="all"?24:15)*gain_budget_source_.duration_s;
            const bool admittance=cfg_.capability.admittance.enabled && (gain_mode=="all" || gain_mode=="servo_tracking");
            if(!gain_automatic_schedule(duration,gain_mode,tuning_mode,budget,admittance).fits)
                throw std::runtime_error("time_budget_insufficient_for_complete_measurement");
        }
        // Publish a complete accepted request only after all stage, parameter
        // and endpoint checks pass. A duplicate or rejected request is read-only
        model_calibration_workflow_kind_=std::move(workflow);
        model_calibration_gain_mode_=std::move(gain_mode);
        gain_requirements_=requirements;gain_tuning_mode_=tuning_mode;
        {std::lock_guard<std::mutex> lock(mutex_);gain_control_context_=gain_current_control_context_json();}
        model_calibration_gain_stage_.store(model_calibration_gain_mode_=="compliant"?1:model_calibration_gain_mode_=="drag"?3:2);
        model_calibration_fault_interrupted_.store(false); model_calibration_cancel_.store(false); model_calibration_pause_.store(false); model_calibration_resume_confirmed_.store(false);
        if(model_calibration_workflow_kind_!="gain" && model_calibration_trajectory_ && model_calibration_trajectory_->positions.size()>=2) {
            const double one_way=model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1);
            const double dwell=2.0*static_cast<double>(model_calibration_pose_targets_.size())*
                (model_calibration_options_.static_hold_s+model_calibration_options_.static_sample_s+.5);
            model_calibration_estimated_duration_s_=serial_arm::calibration_acquisition::traversal_factor*one_way+dwell+2.0;
        }
        if(model_calibration_workflow_kind_=="gain") {
            {std::lock_guard<std::mutex> lock(model_calibration_mutex_);gain_available_=false;gain_warnings_.clear();
                gain_termination_kind_="running";gain_termination_reason_.clear();gain_termination_blocked_joints_.clear();}
            gain_uses_demonstration_=true;gain_next_reverse_=gain_initial_reverse;gain_admittance_required_=cfg_.capability.admittance.enabled && (model_calibration_gain_mode_=="all" || model_calibration_gain_mode_=="servo_tracking");
            const double duration=model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1);
            const double budget=gain_time_budget_s();
            gain_schedule_=gain_automatic_schedule(duration,model_calibration_gain_mode_,gain_tuning_mode_,budget,gain_admittance_required_);
            if(!gain_schedule_.fits)gain_add_warning("time_budget_may_limit_measurements");
            gain_path_pass_current_=0;gain_budget_exhausted_.store(false);gain_finished_at_.reset();
            gain_deadline_=request_started+std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(budget));
            model_calibration_adaptation_passes_.store(0);model_calibration_adaptation_revisions_.store(0);
            model_calibration_update_progress(0,gain_planned_windows());model_calibration_estimated_duration_s_=gain_schedule_.estimated_total_s;
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

    static bool gain_measurement_available(const YAML::Node& node,std::size_t count,const std::string& kind) {
        if(!node || !node.IsMap() || (node["available"] && !node["available"].as<bool>()) ||
           (node["completed"] && !node["completed"].as<bool>()))return false;
        const auto finite=[&](const char* key){return node[key] && std::isfinite(node[key].as<double>());};
        if(kind=="trajectory") {
            if(!node["independent"].as<bool>(false) || !finite("duration_s") || node["duration_s"].as<double>()<=0 ||
               !node["metrics"] || !node["metrics"].IsSequence() || node["metrics"].size()!=count)return false;
            if(node["measured_duration_s"] && (!std::isfinite(node["measured_duration_s"].as<double>()) || node["measured_duration_s"].as<double>()<node["duration_s"].as<double>()-.06))return false;
            for(const auto& metric:node["metrics"]) {
                if(!metric["samples"] || !std::isfinite(metric["samples"].as<double>()) || metric["samples"].as<double>()<=0 ||
                   (metric["measurement_samples"] && (!std::isfinite(metric["measurement_samples"].as<double>()) || metric["measurement_samples"].as<double>()<=0)))return false;
                for(const auto* key:{"rms_error_rad","peak_error_rad","velocity_rms_rad_s","settling_s","tail_error_rad"})
                    if(!metric[key] || !std::isfinite(metric[key].as<double>()) || metric[key].as<double>()<0)return false;
            }
            return true;
        }
        if(!finite("samples") || node["samples"].as<double>()<=0)return false;
        if(kind=="hold") {
            if(node["reason"].as<std::string>("")=="not_started" || node["reason"].as<std::string>("")=="measurement_in_progress")return false;
            for(const auto* key:{"worst_error_rad","worst_velocity_rad_s"})if(!finite(key) || node[key].as<double>()<0)return false;
            if(node["fresh_samples"]) {
                if(!node["fresh_samples"].IsSequence() || node["fresh_samples"].size()!=count)return false;
                for(const auto& sample:node["fresh_samples"])if(!std::isfinite(sample.as<double>()) || sample.as<double>()<=0)return false;
            }
            return true;
        }
        if(kind=="drag") {
            if(!node["completed"].as<bool>(false))return false;
            if(node["axes"]) {
                if(!node["axes"].IsSequence() || node["axes"].size()!=count)return false;
                for(const auto& axis:node["axes"]) {
                    if(!axis["samples"] || !std::isfinite(axis["samples"].as<double>()) || axis["samples"].as<double>()<=0)return false;
                    if(axis["quiet_stability_measured"]) {
                        if(!axis["quiet_stability_measured"].as<bool>(false))return false;
                        for(const auto* key:{"quiet_position_span_rad","quiet_speed_rad_s"})
                            if(!axis[key] || !std::isfinite(axis[key].as<double>()) || axis[key].as<double>()<0)return false;
                        if(!axis["release_decay_measured"].as<bool>(false))continue;
                    }
                    for(const auto* key:{"initial_velocity_rad_s","peak_velocity_rad_s","final_velocity_rad_s","decay_s"})
                        if(!axis[key] || !std::isfinite(axis[key].as<double>()) || axis[key].as<double>()<0)return false;
                }
            } else {
                for(const auto* key:{"decay_s","initial_velocity_rad_s"}) {
                    if(!node[key] || !node[key].IsSequence() || node[key].size()!=count)return false;
                    for(const auto& value:node[key])if(!std::isfinite(value.as<double>()) || value.as<double>()<0)return false;
                }
            }
            return true;
        }
        return false;
    }
    static bool gain_target_available(const YAML::Node& data,const YAML::Node& node,const std::string& target,std::size_t count) {
        if(!node || !node.IsMap())return false;
        if(node["available"] && !node["available"].as<bool>() &&
           data["validation_scope"].as<std::string>("")!="independent_demonstration_trajectory")return false;
        if(node["error"].as<std::string>("")=="not_started" || node["error"].as<std::string>("")=="measurement_in_progress")return false;
        if(!node["available"] && data["validation_scope"].as<std::string>("")!="independent_demonstration_trajectory") {
            if(!node["validated"].as<bool>(false))return false;
            if(target=="compliant_drag")return node["drag_validation"]["validated"].as<bool>(false);
            if(target=="servo_tracking")return node["native_validation"]["validated"].as<bool>(false) && node["admittance_validation"]["validated"].as<bool>(false);
            if(target=="rigid_tracking" || target=="compliant_tracking") {
                const bool trimmed=target=="rigid_tracking" && node["hold_compensation"] && node["hold_compensation"]["enabled"].as<bool>(false);
                return node[trimmed?"hold_compensation_verification":"hold_verification"]["validated"].as<bool>(false);
            }
            return false;
        }
        if(target=="compliant_drag")return gain_measurement_available(node["drag_validation"],count,"drag");
        if(!gain_measurement_available(node["native_validation"],count,"trajectory"))return false;
        if(target=="rigid_tracking") {
            const bool trimmed=node["hold_compensation"] && node["hold_compensation"]["enabled"].as<bool>(false);
            return gain_measurement_available(node[trimmed?"hold_compensation_verification":"hold_verification"],count,"hold");
        }
        if(target=="compliant_tracking")return gain_measurement_available(node["hold_verification"],count,"hold");
        if(target=="servo_tracking") {
            const bool native_only=data["validation_scope"].as<std::string>("")=="independent_demonstration_trajectory" &&
                node["admittance_required"].IsDefined() && !node["admittance_required"].as<bool>() &&
                node["admittance_validation"]["required"].IsDefined() && !node["admittance_validation"]["required"].as<bool>() &&
                node["admittance_validation"]["status"].as<std::string>("")=="not_requested";
            return native_only || gain_measurement_available(node["admittance_validation"],count,"trajectory");
        }
        return false;
    }

    // Compare decoded values, so formatting, key ordering and YAML anchors
    // never invalidate a compatible recorded control context
    static bool gain_context_equal(const YAML::Node& a,const YAML::Node& b) {
        if(!a || a.IsNull())return !b || b.IsNull();
        if(!b || b.IsNull() || a.Type()!=b.Type())return false;
        if(a.IsMap()) {
            if(a.size()!=b.size())return false;
            for(const auto& item:a)
                if(!gain_context_equal(item.second,b[item.first.as<std::string>()]))return false;
            return true;
        }
        if(a.IsSequence()) {
            if(a.size()!=b.size())return false;
            for(std::size_t i=0;i<a.size();++i)if(!gain_context_equal(a[i],b[i]))return false;
            return true;
        }
        if(a.Scalar()==b.Scalar())return true;
        try {const double x=a.as<double>(),y=b.as<double>();return std::isfinite(x) && std::isfinite(y) && x==y;}
        catch(const YAML::Exception&){return false;}
    }

    std::string gain_current_control_context_json() const {
        std::ostringstream out;out<<std::setprecision(17);
        out<<"{\"runtime\":{\"ctrl_frequency_hz\":"<<cfg_.runtime.ctrl_frequency_hz
            <<",\"joint_acc_filter_alpha\":"<<cfg_.runtime.joint_acc_filter_alpha
            <<",\"write_enabled\":"<<(cfg_.runtime.write_enabled?"true":"false")
            <<",\"model_feedforward_mode\":\""<<to_string(robot_.get_model_feedforward_mode())<<"\"},\"admittance\":{";
        const auto& a=cfg_.capability.admittance;
        out<<"\"enabled\":"<<(a.enabled?"true":"false")<<",\"joint_enabled\":";machine_json_vector(out,a.joint_enabled);
        out<<",\"observer\":{\"mode\":\""<<observer_mode_name(a.observer.mode)<<"\",\"momentum_gain\":";machine_json_joint_vector(out,a.observer.momentum_gain);
        out<<"},\"calibration\":{\"torque_bias\":";machine_json_joint_vector(out,a.calibration.torque_bias);
        out<<",\"torque_threshold\":";machine_json_joint_vector(out,a.calibration.torque_threshold);
        const auto& f=a.calibration.friction;
        out<<",\"friction\":{\"enabled\":"<<(f.enabled?"true":"false")<<",\"velocity_transition\":"<<f.velocity_transition;
        out<<",\"positive_coulomb\":";machine_json_joint_vector(out,f.positive_coulomb);
        out<<",\"positive_viscous\":";machine_json_joint_vector(out,f.positive_viscous);
        out<<",\"negative_coulomb\":";machine_json_joint_vector(out,f.negative_coulomb);
        out<<",\"negative_viscous\":";machine_json_joint_vector(out,f.negative_viscous);
        out<<"}},\"controller\":{\"mass\":";machine_json_joint_vector(out,a.controller.mass);
        out<<",\"damping\":";machine_json_joint_vector(out,a.controller.damping);
        out<<",\"stiffness\":";machine_json_joint_vector(out,a.controller.stiffness);
        out<<",\"max_delta_q\":";machine_json_joint_vector(out,a.controller.max_delta_q);
        out<<",\"max_delta_q_dot\":";machine_json_joint_vector(out,a.controller.max_delta_q_dot);
        out<<"}}}";return out.str();
    }

    void gain_require_matching_context(const std::filesystem::path& folder,const YAML::Node& data,
                                       const std::vector<std::string>& selected) const {
        const auto fail=[](const std::string& field){throw std::runtime_error("gain candidate control context mismatch: "+field+"; recalibrate with the current configuration");};
        const auto meta=YAML::LoadFile((folder/"metadata.json").string());
        const auto trusted=[&](const char* file,const char* key) {
            const auto path=folder/"source"/file;
            if(!meta[key] || meta[key].as<std::string>().empty() ||
               meta[key].as<std::string>()!=model_calibration_file_fingerprint(path.string()))
                fail(std::string("missing or changed task snapshot ")+file);
            return YAML::LoadFile(path.string());
        };
        const auto original=trusted("core.yaml","core_fingerprint");
        if(!meta["task_id"] || !data["task_id"] || meta["task_id"].as<std::string>()!=data["task_id"].as<std::string>())fail("task_id");
        if(!robot_profile_.empty() && meta["profile"].as<std::string>("")!=robot_profile_)fail("profile");
        const auto vector_equal=[&](const YAML::Node& node,const auto& values) {
            if(!node || !node.IsSequence() || node.size()!=values.size())return false;
            for(std::size_t i=0;i<values.size();++i)
                if(!std::isfinite(node[i].template as<double>()) || node[i].template as<double>()!=static_cast<double>(values[i]))return false;
            return true;
        };
        if(!meta["joint_names"] || meta["joint_names"].as<std::vector<std::string>>()!=cfg_.joint_names)fail("joint_names");
        const auto mapping=meta["mapping"];
        if(!vector_equal(mapping["pos_ratio"],cfg_.mapper.pos_ratio) || !vector_equal(mapping["tor_ratio"],cfg_.mapper.tor_ratio) ||
           !vector_equal(mapping["direction"],cfg_.mapper.direction) || !vector_equal(mapping["joint_zero_offset"],cfg_.mapper.joint_zero_offset) ||
           !vector_equal(mapping["actuator_zero_offset"],cfg_.mapper.actuator_zero_offset))fail("calibration.joints");
        if(meta["base_frame"].as<std::string>("")!=cfg_.dynamics.base_frame || meta["tool_frame"].as<std::string>("")!=cfg_.dynamics.tool_frame ||
           !vector_equal(meta["gravity"],cfg_.dynamics.gravity) || !vector_equal(meta["original_gravity_scale"],dynamics_.get_gravity_scale()))fail("model frames, gravity or gravity_scale");
        const auto current=YAML::Load(gain_current_control_context_json());
        const auto runtime=original["control"]?original["control"]["runtime"]:original["runtime"];
        const auto before=data["control_context"];
        if(before) {
            if(!gain_context_equal(before["runtime"],current["runtime"]))fail("control.runtime");
        } else {
            if(!runtime || runtime["ctrl_frequency_hz"].as<double>()!=cfg_.runtime.ctrl_frequency_hz ||
               runtime["joint_acc_filter_alpha"].as<double>(.2)!=cfg_.runtime.joint_acc_filter_alpha ||
               runtime["write_enabled"].as<bool>()!=cfg_.runtime.write_enabled ||
               runtime["model_feedforward_mode"].as<std::string>()!=to_string(robot_.get_model_feedforward_mode()))fail("control.runtime");
        }
        const bool servo=std::find(selected.begin(),selected.end(),"servo_tracking")!=selected.end();
        const auto rigid=data["category"].as<std::string>("")=="all"?data["targets"]["rigid_tracking"]:data;
        const bool trim=std::find(selected.begin(),selected.end(),"rigid_tracking")!=selected.end() &&
            rigid["hold_compensation"] && rigid["hold_compensation"]["enabled"].as<bool>(false);
        if(servo || trim) {
            if(before) {
                const auto expected=before["admittance"],actual=current["admittance"];
                if(servo && !gain_context_equal(expected["enabled"],actual["enabled"]))fail("capability.admittance.enabled");
                const bool active=actual["enabled"].as<bool>(false);
                if(trim || (servo && active))for(const auto* key:{"observer","calibration"})
                    if(!gain_context_equal(expected[key],actual[key]))fail(std::string("capability.admittance.")+key);
                if(servo && active)for(const auto* key:{"joint_enabled","controller"})
                    if(!gain_context_equal(expected[key],actual[key]))fail(std::string("capability.admittance.")+key);
            } else {
                const auto loaded=YAML::Load(core_source_config_);
                const auto expected=original["capability"]?original["capability"]["admittance"]:YAML::Node{};
                const auto actual=loaded["capability"]?loaded["capability"]["admittance"]:YAML::Node{};
                const bool expected_enabled=expected && expected["enabled"].as<bool>(false);
                const bool actual_enabled=actual && actual["enabled"].as<bool>(false);
                if(servo && expected_enabled!=actual_enabled)fail("capability.admittance.enabled");
                if(trim || (servo && actual_enabled))for(const auto* key:{"observer","calibration"})
                    if(!gain_context_equal(expected?expected[key]:YAML::Node{},actual?actual[key]:YAML::Node{}))fail(std::string("capability.admittance.")+key);
                if(servo && actual_enabled)for(const auto* key:{"joint_enabled","controller"})
                    if(!gain_context_equal(expected[key],actual[key]))fail(std::string("capability.admittance.")+key);
                if(servo && actual_enabled && data["admittance_parameters"]) {
                    const auto measured=data["admittance_parameters"];const auto& a=cfg_.capability.admittance;
                    if(!vector_equal(measured["mass"],a.controller.mass) || !vector_equal(measured["damping"],a.controller.damping) ||
                       !vector_equal(measured["stiffness"],a.controller.stiffness) || !vector_equal(measured["joint_enabled"],a.joint_enabled) ||
                       measured["observer_mode"].as<std::string>("")!=observer_mode_name(a.observer.mode))fail("measured admittance parameters");
                }
            }
        }
        const auto model=original["model"]?original["model"]:original["dynamics"];
        const bool correction=model["gravity_correction_path"] && !model["gravity_correction_path"].as<std::string>().empty();
        if(correction!=!gravity_correction_source_config_.empty())fail("gravity_correction_path");
        if(correction) {
            (void)trusted("gravity-correction.yaml","gravity_correction_fingerprint");
            if(meta["gravity_correction_fingerprint"].as<std::string>()!=source_text_fingerprint(gravity_correction_source_config_))fail("gravity correction");
        }
        if(runtime_gravity_applied_ || runtime_dynamics_applied_ || (model_calibration_candidate_applied_ && !persistent_gravity_correction_loaded_))fail("temporary model override");
        // Offline recordings remain usable offline, but never qualify a hardware
        // session; a real session must match its effective connection snapshot
        if(runtime["write_enabled"].as<bool>()!=cfg_.runtime.write_enabled)fail("hardware/mock backend");
        if(cfg_.runtime.write_enabled) {
            (void)trusted("hardware.yaml","hardware_fingerprint");
            const auto effective=trusted("hardware-effective.yaml","hardware_effective_fingerprint");
            const auto recorded=meta["runtime_provenance"]["hardware_plugin_file_path"].as<std::string>("");
            if(recorded.empty() || gain_hardware_driver_identity(recorded)!=gain_hardware_driver_identity(hardware_plugin_))fail("hardware driver");
            if(!gain_context_equal(gain_hardware_context(effective,recorded),
                                   gain_hardware_context(YAML::Load(hardware_loader_.effective_config()),hardware_plugin_)))
                fail("hardware configuration or connection overrides");
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

        if(!data["urdf_fingerprint"] ||
           data["urdf_fingerprint"].as<std::string>()!=model_calibration_file_fingerprint(cfg_.dynamics.urdf_path))
            throw std::runtime_error("gain candidate URDF fingerprint mismatch");
        if(!data["joint_names"] || data["joint_names"].as<std::vector<std::string>>()!=cfg_.joint_names)
            throw std::runtime_error("gain candidate joint mapping mismatch");
        if(!data["category"])throw std::runtime_error("gain candidate mode is missing");
        const auto category=data["category"].as<std::string>();
        if(category=="identification_tracking")throw std::runtime_error("legacy identification gain candidates are read-only; rerun native and admittance qualification");
        const bool measured_trajectory_scope=data["validation_scope"].as<std::string>("")=="independent_demonstration_trajectory";
        if(!measured_trajectory_scope && ((data["review_ready"].IsDefined() && !data["review_ready"].as<bool>()) ||
           (data["qualification_complete"].IsDefined() && !data["qualification_complete"].as<bool>()) ||
           (data["measurement_complete"].IsDefined() && !data["measurement_complete"].as<bool>())))
            throw std::runtime_error("gain calibration measurements are incomplete; review is not ready");
        if(measured_trajectory_scope) {
            bool any=false;
            if(category=="all")for(const auto* purpose:{"rigid_tracking","compliant_tracking","compliant_drag","servo_tracking"})
                any=gain_target_available(data,data["targets"][purpose],purpose,cfg_.joint_names.size()) || any;
            else any=gain_target_available(data,data,category,cfg_.joint_names.size());
            if(!any)throw std::runtime_error("gain calibration has no complete measured parameter group");
        }
        std::vector<std::pair<JointImpedanceMode,JointImpedanceGains>> updates;
        std::optional<JointImpedanceGains> identification;
        std::optional<HoldCompensationCfg> hold_compensation;
        const auto collect=[&](const std::string& target,const YAML::Node& node) {
            if(!gain_target_available(data,node,target,cfg_.joint_names.size()))
                throw std::runtime_error("gain target has no complete measured candidate");
            const bool legacy=!node["available"] && data["validation_scope"].as<std::string>("")!="independent_demonstration_trajectory";
            const auto measured=[&](const YAML::Node& validation,const std::string& kind){return legacy?validation["validated"].as<bool>(false):gain_measurement_available(validation,cfg_.joint_names.size(),kind);};
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
            if(target=="rigid_tracking") {
                const bool trim_verified=node["hold_compensation"] && node["hold_compensation"]["enabled"].as<bool>(false) &&
                    measured(node["hold_compensation_verification"],"hold");
                const bool baseline_verified=measured(node["hold_verification"],"hold");
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
                if(node["hold_gains"] && measured(node["hold_verification"],"hold"))
                    updates.emplace_back(JointImpedanceMode::COMPLIANT_HOLD,JointImpedanceGains{node["hold_gains"]["kp"].as<JointVector>(),node["hold_gains"]["kd"].as<JointVector>()});
                updates.emplace_back(JointImpedanceMode::COMPLIANT_TRACKING,gains);
            } else if(target=="servo_tracking") {
                const bool trajectory_scope=data["validation_scope"].as<std::string>("")=="independent_demonstration_trajectory";
                const bool native_only=trajectory_scope && node["admittance_required"].IsDefined() && !node["admittance_required"].as<bool>() &&
                    node["admittance_validation"]["required"].IsDefined() && !node["admittance_validation"]["required"].as<bool>() && node["admittance_validation"]["status"].as<std::string>("")=="not_requested";
                if(!measured(node["native_validation"],"trajectory") || (!native_only && !measured(node["admittance_validation"],"trajectory")))throw std::runtime_error("servo candidate requires complete native and requested admittance measurements");
                identification=gains;
            } else if(target=="compliant_drag") {
                if(!measured(node["drag_validation"],"drag") || std::any_of(gains.kp.begin(),gains.kp.end(),[](double v){return v!=0;}))throw std::runtime_error("drag candidate requires zero stiffness and measured release validation");
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
        } else if(category=="all") {
            for(const auto& target:order)if(gain_target_available(data,data["targets"][target],target,cfg_.joint_names.size()))selected.push_back(target);
            if(selected.empty())throw std::runtime_error("gain candidate has no complete measured purpose");
        } else selected={category};
        gain_require_matching_context(path.parent_path(),data,selected);
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
        // Full inertia is allowed only for an actual dynamic-data assessment
        const auto scope=report["validation_scope"].as<std::string>();
        if(scope!="dynamic_inertia")
            throw std::runtime_error("static gravity candidate must use the gravity-only apply route");
        const std::filesystem::path recorded_candidate(report["candidate_urdf"].as<std::string>());
        if(recorded_candidate.empty()) throw std::runtime_error("candidate URDF path is empty");
        // Reports and candidates form one portable directory. An older report
        // may retain its former absolute path after the directory is copied
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
        // Avoid accidentally layering a stale gravity correction on a new model
        new_cfg.gravity_correction_path.clear();
        Dynamics candidate;
        if(!candidate.configure(new_cfg))
            throw std::runtime_error("candidate URDF cannot configure robot dynamics");
        // Check the whole controlled kinematic tree, not only tool0 at q=0
        const auto& expected_frames=dynamics_.get_info().frame_names;
        const auto& candidate_frames=candidate.get_info().frame_names;
        if(expected_frames!=candidate_frames)
            throw std::runtime_error("candidate URDF changed frame hierarchy");
        const std::size_t n=cfg_.joint_names.size();
        JointState sample;
        sample.pos.assign(n,0.0);sample.vel.assign(n,0.0);sample.tor.assign(n,0.0);
        JointVector zero(n,0.0);
        // Must not mutate live dynamics caches: work on a separate prior
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
        if(model_calibration_workflow_kind_=="gain")out<<",\"reference_only\":true,\"accuracy_thresholds_enabled\":false,\"acceptance_scope\":\"measured_response_reference_only_v1\"";
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
        const double gain_budget=gain_time_budget_s();
        const double gain_elapsed=gain_deadline_?std::max(0.0,gain_budget-std::chrono::duration<double>(*gain_deadline_-gain_finished_at_.value_or(Robot::Clock::now())).count()):0;
        out<<",\"gain_tuning_mode\":\""<<gain_tuning_mode_<<"\",\"gain_target_budget_s\":"<<(model_calibration_gain_mode_=="all"?18:12)*gain_budget_source_.duration_s<<",\"gain_protocol\":\""<<(gain_uses_demonstration_?"taught_trajectory":"local_probe_legacy")<<"\",\"gain_time_budget_s\":"<<gain_budget<<",\"gain_elapsed_s\":"<<gain_elapsed
            <<",\"gain_budget_source_duration_s\":"<<gain_budget_source_.duration_s<<",\"gain_budget_source\":\""<<machine_json_escape(gain_budget_source_.source)
            <<"\",\"gain_budget_source_task_id\":\""<<machine_json_escape(gain_budget_source_.task_id)<<"\",\"gain_budget_multiplier\":"<<(model_calibration_gain_mode_=="all"?24:15)
            <<",\"gain_remaining_s\":"<<std::max(0.0,gain_budget-gain_elapsed)<<",\"gain_budget_exhausted\":"<<(gain_budget_exhausted_.load()?"true":"false")
            <<",\"gain_path_pass_limit\":"<<gain_schedule_.full_pass_limit<<",\"gain_path_pass_current\":"<<gain_path_pass_current_
            <<",\"gain_trajectory_duration_s\":"<<(model_calibration_trajectory_ && model_calibration_trajectory_->positions.size()>1?model_calibration_trajectory_->sample_dt*static_cast<double>(model_calibration_trajectory_->positions.size()-1):0)
            <<",\"gain_effective_pass_duration_s\":"<<gain_effective_pass_duration_s_<<",\"gain_admittance_required\":"<<(gain_admittance_required_?"true":"false");
        out<<",\"gain_available\":"<<(gain_available_?"true":"false")<<",\"gain_warnings\":";gain_json_warnings(out,gain_warnings_);
        if(model_calibration_workflow_kind_=="gain")out<<",\"termination\":"<<gain_termination_json_locked();

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
            << ",\"gain_candidate_limit\":" << (gain_probe_stage_=="trajectory_search"?(model_calibration_gain_stage_.load()==1?gain_schedule_.compliant_search_passes:gain_schedule_.native_search_passes):gain_probe_stage_=="motion_discovery"?serial_arm::servo_tuning::motion_attempt_limit:gain_probe_stage_=="dynamic_search"?gain_requirements_.max_candidates:0)
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
            << ",\"max_task_duration_s\":" << (gain_uses_demonstration_?gain_budget:0)
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
            << ",\"settled_s\":" << motion.settled_s << ",\"target_reached\":" << (motion.target_reached?"true":"false")
            << ",\"position_error\":";machine_json_joint_vector(out,motion.position_error);
        out << ",\"settling_velocity_source\":\"fresh_position_window\",\"reason\":\"" << machine_json_escape(motion.reason) << "\"}";
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
                << ",\"max_kd\":" << item.max_kd
                << ",\"kp_resolution\":"<<item.kp_resolution<<",\"kd_resolution\":"<<item.kd_resolution<<",\"effort_resolution\":"<<item.effort_resolution<<'}';
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
        // Report effective runtime limits, not merely the URDF source limits
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
            gain_update_cycle_reference(now);
            if(stream_.enabled) {
                const auto result=update_stream(now);
                if(!result){report_background_fault(result.error());return false;}
            }
            return true;
        };
        hooks.after_step=[this](const RuntimeSnapshot& step){
            if(step.cycle) {
                // Refresh diagnostic RNEA against the accepted reference; the
                // command itself was already computed from the cycle snapshot
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
        frame.command_kd = output.joint_cmd.kd;record_gain_encoding(frame,output);
        frame.model_feedforward = output.model_feedforward;
        frame.nominal_position=output.nominal_ref_pos;frame.nominal_velocity=output.nominal_ref_vel;
        frame.virtual_position=output.virtual_ref_pos;frame.virtual_velocity=output.virtual_ref_vel;
        frame.accepted_position=output.accepted_ref_pos;frame.accepted_velocity=output.accepted_ref_vel;
        frame.command_torque=output.joint_cmd.tor;frame.hold_integral_torque=output.hold_integral_torque;
        frame.calibration_disturbance=output.calibration_disturbance;
        frame.reference_limited=output.reference_limited;frame.torque_limited=output.torque_limited;
        frame.hold_integrator_freeze_reasons=output.hold_integrator_freeze_reasons;
        if(model_calibration_workflow_kind_=="gain") {
            frame.gain_stage=gain_capture_label_;
        }
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
        // Freeze this successful cycle's execution metadata before a response
        // guard can switch the Robot into its fault hold reaction.
        if(model_calibration_workflow_kind_=="gain")gain_capture_cycle(output,now);
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
        // Leave the robot in its latched safe FAULT; do not auto-clear or resume
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
            // transition them into FAILED, so latch that phase here
            if(calibration_phase == ModelCalibrationPhase::TEACHING ||
                calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION) {
                model_calibration_set_phase(ModelCalibrationPhase::FAILED);
                // WAITING has no worker to clean up its recorder; the already
                // saved trajectory.csv remains available for later inspection
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
        // into a normal shutdown. Robot's ordinary configured gains stay intact
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
            // Do not validate or deactivate based on an old frozen feedback frame
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
            // pass the handover gate without requiring proximity to park_pos
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
                // High rigidity never inherits the compliant reference backlog
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
                // jumping directly from Gray compliant Kp to rigid Kp
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
                // The old 0.16 clamp made a 2.5 s rigid phase unnecessarily slow
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

    bool set_tuning_impedance_mode(JointImpedanceMode mode,bool capture_measured=false) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool finish_drag=robot_.get_impedance_mode()==JointImpedanceMode::COMPLIANT_DRAG &&
            (mode==JointImpedanceMode::RIGID_HOLD || mode==JointImpedanceMode::COMPLIANT_HOLD);
        clear_command_sources();
        const auto result = runtime_.set_impedance_mode(robot_, mode,Robot::Clock::now(),(finish_drag||capture_measured)?HoldTargetPolicy::CAPTURE_MEASURED:HoldTargetPolicy::PRESERVE_NOMINAL);
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
            // but must not discard samples from earlier valid control cycles
            if(robot_.get_state() != RobotState::ACTIVE) break;
            if(stop_recording.load()) break;
            if(!updated || cycle_counter_ <= cursor || !last_output_) continue;
            cursor = cycle_counter_;
            if((received_cycles++ % kRecordStride) == 0) {
                trajectory.positions.push_back(last_output_->joint_state.pos);
                if(checkpoint && trajectory.positions.size() >= last_checkpoint_samples + checkpoint_interval) {
                    // Sampling has its own worker, but never write a file while
                    // holding the mutex that guards the real-time cycle state
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

    std::string friction_trajectory_range_error(const FrictionCalibrationTrajectory& trajectory,bool command_reference=true) const {
        const auto& limits=cfg_.safety.limits;
        const auto& types=dynamics_.get_info().joint_types;
        for(std::size_t k=0;k<trajectory.positions.size();++k) {
            const auto& q=trajectory.positions[k];
            if(q.size()!=cfg_.joint_names.size())return "trajectory joint count does not match active robot";
            for(std::size_t i=0;i<q.size();++i) {
                if(!std::isfinite(q[i]))return "trajectory contains nonfinite position at "+cfg_.joint_names[i];
                if(i>=limits.has_position_limit.size() || !limits.has_position_limit[i])continue;
                // Use the same explicit command range as ordinary motion. A
                // hidden extra replay margin made valid teaching look invalid
                // and encouraged operators to enlarge the URDF instead.
                const double margin=command_reference?limits.pos_margin[i]:0.0,lower=limits.min_pos[i]+margin,upper=limits.max_pos[i]-margin;
                if(lower>=upper || q[i]<lower || q[i]>upper) {
                    std::ostringstream out;out<<std::setprecision(17)<<"trajectory_range_violation:joint="<<cfg_.joint_names[i]
                        <<";sample="<<k<<";value="<<q[i]<<";lower="<<lower<<";upper="<<upper<<";margin="<<margin
                        <<";unit="<<(types.at(i)=="prismatic"?"m":"rad")<<";scope="<<(command_reference?"command_reference":"recorded_position");return out.str();
                }
            }
        }
        return {};
    }

    bool friction_trajectory_inside_safe_replay_range(const FrictionCalibrationTrajectory& trajectory,bool command_reference=true) const {
        const auto error=friction_trajectory_range_error(trajectory,command_reference);
        if(!error.empty())std::cout<<"[拒绝回放] "<<error<<'\n';
        return error.empty();
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
                // velocity. This never creates a separate alignment movement
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
        gain_cycle_reference_={};
        gain_reference_sample_ns_=0;
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
    std::function<void(Robot::TimePoint)> gain_cycle_reference_;
    std::optional<JointImpedanceGains> gain_discovery_rollback_;
    bool gain_discovery_reverted_{false};
    std::optional<std::size_t> gain_discovery_rollback_axis_;
    Robot::TimePoint gain_cycle_reference_until_{};
    std::exception_ptr gain_cycle_reference_error_;
    std::uint64_t gain_reference_sample_ns_{0};
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
    std::string gain_control_context_;
    std::optional<FrictionCalibrationTrajectory> model_calibration_trajectory_;
    std::vector<ModelCalibrationPoseTarget> model_calibration_pose_targets_;
    double model_calibration_replay_rate_{ 0.0 };
    std::atomic<std::size_t> model_calibration_adaptation_passes_{0};
    std::atomic<std::size_t> model_calibration_adaptation_revisions_{0};
    std::atomic<bool> model_calibration_adaptation_converged_{false};
    std::string model_calibration_workflow_kind_{ "gravity" };
    std::string model_calibration_gain_mode_{ "all" };
    serial_arm::servo_tuning::Requirements gain_requirements_;
    serial_arm::servo_tuning::TrajectorySchedule gain_schedule_;
    std::string gain_tuning_mode_{"quick"};
    serial_arm::servo_tuning::ModelInitialization gain_model_initialization_,gain_task_model_initialization_;
    std::string gain_task_model_error_;
    std::vector<serial_arm::servo_tuning::ModelAxisLimits> gain_model_limits_;
    std::vector<serial_arm::servo_tuning::ResponseModel> gain_response_models_;
    std::vector<serial_arm::servo_tuning::OnlineTrajectoryState> gain_online_states_;
    bool gain_capture_active_{false},gain_capture_accepted_{false};
    Robot::TimePoint gain_capture_start_{};
    std::string gain_capture_label_;
    std::vector<std::vector<serial_arm::servo_tuning::Sample>> gain_capture_rows_;
    JointVector gain_online_input_plan_;
    std::vector<serial_arm::servo_tuning::CommandConditionedMotionGuard> gain_motion_guards_;
    std::vector<std::string> gain_guard_records_;
    std::vector<std::uint64_t> gain_capture_stamps_;
    std::vector<std::pair<JointVector,std::vector<std::string>>> gain_capture_integral_;
    std::vector<bool> gain_capture_observer_;
    std::size_t gain_hold_serial_{0};
    std::vector<std::string> gain_experiment_records_,gain_candidate_records_;
    GainBudgetSource gain_budget_source_;
    bool gain_uses_demonstration_{false},gain_next_reverse_{true},gain_admittance_required_{false};
    JointVector gain_completed_endpoint_reference_;
    std::string gain_completed_endpoint_record_;
    bool gain_available_{false};std::vector<std::string> gain_warnings_;
    std::string gain_termination_kind_{"running"},gain_termination_reason_;
    std::vector<std::string> gain_termination_blocked_joints_;
    std::optional<Robot::TimePoint> gain_deadline_,gain_finished_at_;
    std::atomic<bool> gain_budget_exhausted_{false};
    std::size_t gain_path_pass_current_{0};
    double gain_effective_pass_duration_s_{0};
    std::optional<JointImpedanceGains> gain_trial_original_servo_;
    std::optional<HoldCompensationCfg> gain_trial_original_hold_compensation_;
    std::atomic<int> model_calibration_gain_stage_{2};
    std::string gain_probe_kind_,gain_probe_joint_,gain_probe_message_,gain_seed_source_,gain_probe_stage_,gain_probe_diagnostic_json_{"{}"};
    std::array<double,3> gain_probe_frequency_hz_{};
    std::vector<serial_arm::servo_tuning::NoiseEstimate> gain_noise_;
    std::vector<serial_arm::servo_tuning::Requirements> gain_plans_,gain_native_plans_;
    JointVector gain_probe_origin_,gain_demonstrated_path_lower_,gain_demonstrated_path_upper_;
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
