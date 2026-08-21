#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"
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
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
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
        std::string robot_profile)
        : cfg_(std::move(cfg)),
        config_path_(std::move(config_path)),
        hardware_plugin_(std::move(hardware_plugin)),
        hardware_config_(std::move(hardware_config)),
        hardware_overrides_(std::move(hardware_overrides)),
        connection_summary_(std::move(connection_summary)),
        robot_profile_(std::move(robot_profile)) {

    }

    ~TerminalApp() {
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

        auto hardware_result = hardware_loader_.load(hardware_plugin_, hardware_config_, hardware_overrides_);
        if(!hardware_result) return tl::make_unexpected("HardwareLoader 失败: " + to_string(hardware_result.error()));
        std::unique_ptr<MotorBus> hardware_bus = std::move(hardware_result.value());
        actuator_info_ = hardware_bus->capabilities();

        std::unique_ptr<MotorBus> motor_bus;
        RobotCfg robot_cfg = cfg_;
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

private:
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
        ++cycle_counter_;
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
            std::cout << " 2. 手感设置\n";
            std::cout << " 3. 状态与诊断\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: run_admittance_calibration_menu(); break;
                case 2: run_admittance_feel_menu(); break;
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
        clear_command_sources();
        if(robot_.get_state() == RobotState::INACTIVE) {
            std::cout << "Robot 已经处于 INACTIVE\n";
            return;
        }
        if(robot_.get_state() == RobotState::FAULT) {
            std::cout << "Robot 当前处于 FAULT，不能执行停放轨迹，请先处理故障或使用立即失能\n";
            return;
        }
        if(!cfg_.shutdown.park_before_disable) {
            std::cout << "shutdown.park_before_disable=false，将直接执行立即失能\n";
            immediate_deactivate_locked();
            return;
        }

        // 停放是确定性的安全轨迹任务，不允许导纳改变 park reference
        robot_.set_admittance_suspended(true);
        const auto resume_admittance = [this]() {
            robot_.set_admittance_suspended(false);
            };

        const auto mode_result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING, Robot::Clock::now());
        if(!mode_result) {
            std::cout << "切换 RIGID_TRACKING 失败：\n";
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
        print_vector("park_pos", cfg_.shutdown.park_pos);

        const Robot::TimePoint started_at = Robot::Clock::now();
        Robot::TimePoint last_progress_at = started_at;
        std::optional<Robot::TimePoint> settled_at;
        while(robot_.get_state() == RobotState::ACTIVE) {
            cycle_cv_.wait_for(lock, std::chrono::milliseconds(20));
            const Robot::TimePoint now = Robot::Clock::now();
            if(!last_output_) continue;

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

            if(std::chrono::duration<double>(now - last_progress_at).count() >= 1.0) {
                std::cout << "[停放] 最大位置误差=" << max_position_error << " rad (" << cfg_.joint_names[position_index] << ")，最大速度="
                    << max_velocity << " rad/s (" << cfg_.joint_names[velocity_index] << ")\n";
                last_progress_at = now;
            }

            if(std::chrono::duration<double>(now - started_at).count() > cfg_.shutdown.timeout_s) {
                const double relaxed_position_tolerance = cfg_.shutdown.position_tolerance * cfg_.shutdown.relaxed_tolerance_ratio;
                const double relaxed_velocity_tolerance = cfg_.shutdown.velocity_tolerance * cfg_.shutdown.relaxed_tolerance_ratio;
                if(max_position_error <= relaxed_position_tolerance && max_velocity <= relaxed_velocity_tolerance) {
                    std::cout << "[停放] 严格判据超时，但实测状态已满足宽松判据，将继续保持并失能\n";
                    break;
                }

                clear_command_sources();
                (void)robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, now);
                std::cout << "停放流程超时，已切换 RIGID_HOLD 并取消失能\n";
                std::cout << "最差位置误差=" << max_position_error << " rad (" << cfg_.joint_names[position_index] << ")，允许="
                    << cfg_.shutdown.position_tolerance << " rad，宽松允许=" << relaxed_position_tolerance << " rad\n";
                std::cout << "最差速度=" << max_velocity << " rad/s (" << cfg_.joint_names[velocity_index] << ")，允许="
                    << cfg_.shutdown.velocity_tolerance << " rad/s，宽松允许=" << relaxed_velocity_tolerance << " rad/s\n";
                resume_admittance();
                return;
            }
        }

        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "停放过程中 Robot 离开 ACTIVE：" << to_string(robot_.get_state()) << '\n';
            resume_admittance();
            return;
        }

        clear_command_sources();
        const auto hold_result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, Robot::Clock::now());
        if(!hold_result) {
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
        const auto scale = read_vector("输入重力补偿比例 [0, 1]，以空格分隔: ", cfg_.joint_names.size());
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
                std::cout << "重力补偿比例必须位于 [0, 1]，本次输入整组未生效\n";
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
        const std::atomic<bool>& stop_recording) {
        constexpr std::size_t kRecordStride = 4; // 200 Hz control -> about 50 Hz recorded path
        FrictionCalibrationTrajectory trajectory;
        trajectory.sample_dt = static_cast<double>(kRecordStride) / cfg_.runtime.ctrl_frequency_hz;

        std::uint64_t cursor = 0;
        std::size_t received_cycles = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cursor = cycle_counter_;
        }

        while(!stop_recording.load()) {
            std::unique_lock<std::mutex> lock(mutex_);
            const bool updated = cycle_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
                return stop_recording.load() || cycle_counter_ > cursor || robot_.get_state() != RobotState::ACTIVE;
                });
            if(robot_.get_state() != RobotState::ACTIVE) return std::nullopt;
            if(stop_recording.load()) break;
            if(!updated || cycle_counter_ <= cursor || !last_output_) continue;
            cursor = cycle_counter_;
            if((received_cycles++ % kRecordStride) == 0) {
                trajectory.positions.push_back(last_output_->joint_state.pos);
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
        TorqueResidualObserver momentum_filter;
        TorqueResidualObserver full_id_filter;
        TorqueResidualObserverCfg full_id_filter_cfg;
        full_id_filter_cfg.joints_count = cfg_.joint_names.size();
        full_id_filter_cfg.filter_alpha = admittance_cfg.observer.filter_alpha;
        full_id_filter_cfg.initial_filtered_residual = admittance_cfg.calibration.torque_bias;
        if(!full_id_filter.configure(full_id_filter_cfg)) return false;
        if(admittance_cfg.observer.mode == AdmittanceObserverMode::MOMENTUM) {
            GeneralizedMomentumObserverCfg momentum_cfg;
            momentum_cfg.joints_count = cfg_.joint_names.size();
            momentum_cfg.gain = admittance_cfg.observer.momentum_gain;
            momentum_cfg.initial_residual = admittance_cfg.calibration.torque_bias;
            if(!momentum_observer.configure(momentum_cfg)) return false;

            TorqueResidualObserverCfg filter_cfg;
            filter_cfg.joints_count = cfg_.joint_names.size();
            filter_cfg.filter_alpha = admittance_cfg.observer.filter_alpha;
            filter_cfg.initial_filtered_residual = admittance_cfg.calibration.torque_bias;
            if(!momentum_filter.configure(filter_cfg)) return false;
        }

        std::vector<JointVector> ordered = trajectory.positions;
        if(reverse) std::reverse(ordered.begin(), ordered.end());

        const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
        const double index_step = playback_rate * nominal_dt / trajectory.sample_dt;
        if(!std::isfinite(index_step) || index_step <= 0.0) return false;

        JointVector previous_ref = ordered.front();
        double progress = 0.0;
        while(progress < static_cast<double>(ordered.size() - 1)) {
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
                // residual must come from the same observer family used at runtime.
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
            const auto full_id_filtered = full_id_filter.update(output.joint_state.tor, model_torque);
            if(!full_id_filtered) return false;
            JointVector full_id_after_bias(n, 0.0);
            for(std::size_t i = 0; i < n; ++i) {
                full_id_after_bias[i] = full_id_filtered->residual_filtered[i] -
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
                const auto filtered = momentum_filter.update(
                    JointVector(n, 0.0), momentum->tau_ext_hat);
                if(!filtered) return false;
                selected_after_bias.resize(n);
                for(std::size_t i = 0; i < n; ++i) {
                    selected_after_bias[i] = filtered->residual_filtered[i] -
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

    void print_friction_calibration_yaml(const FrictionResidualModelCfg& friction) const {
        std::cout << "  friction:\n";
        std::cout << "    enabled: " << (friction.enabled ? "true" : "false") << '\n';
        std::cout << "    velocity_transition: " << std::fixed << std::setprecision(6)
            << friction.velocity_transition << '\n';
        std::cout << "    zero_velocity_adaptation_s: " << friction.zero_velocity_adaptation_s << '\n';
        std::cout << "    kinetic_feedforward_scale: " << friction.kinetic_feedforward_scale << '\n';
        std::cout << "    "; print_joint_yaml_map("positive_coulomb", cfg_.joint_names, friction.positive_coulomb);
        std::cout << "    "; print_joint_yaml_map("positive_viscous", cfg_.joint_names, friction.positive_viscous);
        std::cout << "    "; print_joint_yaml_map("negative_coulomb", cfg_.joint_names, friction.negative_coulomb);
        std::cout << "    "; print_joint_yaml_map("negative_viscous", cfg_.joint_names, friction.negative_viscous);
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
        const bool pass = std::all_of(result->pass.begin(), result->pass.end(), [](std::uint8_t value) {
            return value != 0;
            });
        std::cout << (pass ? "静态残差验证：PASS\n" : "静态残差验证：FAIL（请重新执行导纳参数标定）\n");
        return pass;
    }

    void print_admittance_persistence_block() const {
        std::cout << "model.gravity_scale:\n  ";
        print_joint_yaml_map("gravity_scale", cfg_.joint_names, cfg_.dynamics.gravity_scale);
        std::cout << "capability.admittance.calibration:\n";
        std::cout << "  ";
        print_joint_yaml_map("torque_bias", cfg_.joint_names, cfg_.capability.admittance.calibration.torque_bias);
        std::cout << "  ";
        print_joint_yaml_map("torque_threshold", cfg_.joint_names, cfg_.capability.admittance.calibration.torque_threshold);
        print_friction_calibration_yaml(cfg_.capability.admittance.calibration.friction);
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

    void run_admittance_calibration_menu() {
        while(true) {
            std::cout << "\n------------ 导纳参数标定 ------------\n";
            std::cout << " 1. 一次性标定\n";
            std::cout << " 2. 静态残差标定\n";
            std::cout << " 3. 静态残差验证\n";
            std::cout << " 4. 摩擦参数标定\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: run_admittance_parameter_calibration(); break;
                case 2: run_admittance_static_calibration(); break;
                case 3: run_admittance_static_validation(); break;
                case 4: run_admittance_friction_calibration(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    bool select_admittance_joints(std::vector<std::size_t>& indices) const {
        const auto joint = read_int("关节 1~6，输入 0 表示全部: ");
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

    void set_follow_resistance() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto torque = read_double("舒适推力矩 Nm: ");
        const auto speed = read_double("该力矩对应的跟随速度 rad/s: ");
        if(!torque || !speed || *torque <= 0.0 || *speed <= 0.0) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) {
            candidate.feel.comfortable_torque[i] = *torque;
            candidate.feel.follow_speed[i] = *speed;
        }
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "跟手阻力已应用\n";
    }

    void set_start_response() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto t95 = read_double("起步达到约 95% 跟随速度的时间 s: ");
        if(!t95 || *t95 <= 0.02 || *t95 > 3.0) {
            std::cout << "建议范围 (0.02, 3.0] s\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) candidate.feel.start_response_s[i] = *t95;
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "起步响应已应用\n";
    }

    void set_soft_velocity() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto speed = read_double("Q弹开始速度 rad/s: ");
        if(!speed || *speed <= 0.0) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) {
            if(*speed >= candidate.feel.max_correction_speed[i]) {
                std::cout << "Q弹开始速度必须小于最大导纳修正速度\n";
                return;
            }
            candidate.feel.q_elastic_start_speed[i] = *speed;
        }
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "Q弹起始速度已应用\n";
    }

    void set_return_response() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto t95 = read_double("松手后约 95% 回中的时间 s: ");
        if(!t95 || *t95 <= 0.05 || *t95 > 5.0) {
            std::cout << "建议范围 (0.05, 5.0] s\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) candidate.feel.return_time_s[i] = *t95;
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "回中速度已应用\n";
    }

    void set_max_retreat() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto value = read_double("最大退让 rad: ");
        if(!value || *value <= 0.0) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) candidate.feel.max_retreat[i] = *value;
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "最大退让已应用\n";
    }

    void run_admittance_feel_menu() {
        if(!ensure_admittance_tuning_active()) return;
        if(!cfg_.capability.admittance.enabled) {
            std::cout << "请先在 core.yaml 开启 admittance.enabled\n";
            return;
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) return;
        while(true) {
            std::cout << "\n------------ 手感设置 ------------\n";
            std::cout << " 1. 跟手阻力\n";
            std::cout << " 2. 起步响应\n";
            std::cout << " 3. Q弹起始速度\n";
            std::cout << " 4. 回中速度\n";
            std::cout << " 5. 最大退让\n";
            std::cout << " 6. 高级参数\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1: set_follow_resistance(); break;
                case 2: set_start_response(); break;
                case 3: set_soft_velocity(); break;
                case 4: set_return_response(); break;
                case 5: set_max_retreat(); break;
                case 6: run_admittance_live_tuning(); break;
                default: std::cout << "未知菜单编号\n"; break;
            }
        }
    }

    static const char* observer_mode_name(AdmittanceObserverMode mode) {
        return mode == AdmittanceObserverMode::MOMENTUM ? "MOMENTUM" : "FULL_ID";
    }

    void print_admittance_status_summary() const {
        const auto& a = cfg_.capability.admittance;
        const auto derived = derive_admittance_controller_cfg(a);
        std::cout << "\n导纳状态\n";
        std::cout << "  Observer       : " << observer_mode_name(a.observer.mode) << '\n';
        std::cout << "  摩擦补偿       : " << (a.calibration.friction.enabled ? "ON" : "OFF") << '\n';
        std::cout << "  摩擦主动助力   : " << std::fixed << std::setprecision(2)
            << a.calibration.friction.kinetic_feedforward_scale << '\n';
        std::cout << "  连续可变导纳   : ON\n";
        print_vector("derived_M", derived.mass);
        print_vector("derived_D_follow", derived.damping);
        print_vector("derived_K_return", derived.stiffness);
        if(last_static_validation_result_) {
            const bool pass = std::all_of(
                last_static_validation_result_->pass.begin(),
                last_static_validation_result_->pass.end(),
                [](std::uint8_t value) { return value != 0; });
            std::cout << "  本次静态验证   : " << (pass ? "PASS" : "FAIL") << '\n';
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
        if(a.calibration.friction.enabled) {
            print_vector("friction_C+", a.calibration.friction.positive_coulomb);
            print_vector("friction_B+", a.calibration.friction.positive_viscous);
            print_vector("friction_C-", a.calibration.friction.negative_coulomb);
            print_vector("friction_B-", a.calibration.friction.negative_viscous);
        }
        if(last_friction_calibration_result_) {
            std::cout << "\n摩擦交叉验证（当前 Observer）\n";
            for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
                std::cout << "  " << cfg_.joint_names[i]
                    << " RMS " << std::fixed << std::setprecision(3)
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
                    std::cout << "\ntau ";
                    for(double v : output->tau_ext_hat) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    if(output->full_id_residual_raw.size() == cfg_.joint_names.size()) {
                        std::cout << "| full_id ";
                        for(double v : output->full_id_residual_raw) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    }
                    if(output->contact_confidence.size() == cfg_.joint_names.size()) {
                        std::cout << "| conf ";
                        for(double v : output->contact_confidence) std::cout << std::fixed << std::setprecision(2) << v << ' ';
                    }
                    if(cfg_.capability.admittance.calibration.friction.enabled &&
                        output->friction_residual_hat.size() == cfg_.joint_names.size() &&
                        output->friction_compensated.size() == cfg_.joint_names.size()) {
                        std::cout << "| fric ";
                        for(double v : output->friction_residual_hat) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                        std::cout << "| fric_comp ";
                        for(double v : output->friction_compensated) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    }
                    std::cout << "| dq ";
                    for(double v : output->delta_q) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    std::cout << "| dqdot ";
                    for(double v : output->delta_q_dot) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    if(output->effective_damping.size() == cfg_.joint_names.size()) {
                        std::cout << "| D_eff ";
                        for(double v : output->effective_damping) std::cout << std::fixed << std::setprecision(2) << v << ' ';
                    }
                    if(output->effective_stiffness.size() == cfg_.joint_names.size()) {
                        std::cout << "| K_eff ";
                        for(double v : output->effective_stiffness) std::cout << std::fixed << std::setprecision(2) << v << ' ';
                    }
                    if(output->friction_feedforward.size() == cfg_.joint_names.size()) {
                        std::cout << "| fric_ff ";
                        for(double v : output->friction_feedforward) std::cout << std::fixed << std::setprecision(3) << v << ' ';
                    }
                    if(!first_flag) std::cout << "| " << flags.str();
                    std::cout << std::flush;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            });
        std::string line;
        read_line("\n实时观测中（TH=threshold, DQ=位置限幅, DQV=速度限幅, SP/SV=Safety 剩余空间）；按 Enter 停止: ", line);
        stop.store(true);
        display.join();
        std::cout << '\n';
    }

    void print_derived_admittance_parameters() const {
        const auto derived = derive_admittance_controller_cfg(cfg_.capability.admittance);
        std::cout << "\n派生控制参数（只读）\n";
        print_vector("derived_M", derived.mass);
        print_vector("derived_D_follow", derived.damping);
        print_vector("derived_K_return", derived.stiffness);
        JointVector return_damping(derived.joints_count, 0.0);
        for(std::size_t i = 0; i < derived.joints_count; ++i) {
            return_damping[i] = 2.0 * std::sqrt(derived.mass[i] * derived.stiffness[i]);
        }
        print_vector("derived_D_return", return_damping);
    }

    void set_max_correction_speed_advanced() {
        std::vector<std::size_t> indices;
        if(!select_admittance_joints(indices)) return;
        const auto value = read_double("最大导纳修正速度 rad/s: ");
        if(!value || *value <= 0.0) {
            std::cout << "输入无效\n";
            return;
        }
        auto candidate = cfg_.capability.admittance;
        for(const auto i : indices) {
            if(*value <= candidate.feel.q_elastic_start_speed[i]) {
                std::cout << "最大导纳修正速度必须大于 Q弹开始速度\n";
                return;
            }
            candidate.feel.max_correction_speed[i] = *value;
        }
        if(apply_runtime_admittance_cfg(candidate)) std::cout << "最大导纳修正速度已应用\n";
    }

    void print_live_tuning_yaml() const {
        const auto& a = cfg_.capability.admittance;
        std::cout << "\n可写回 core.yaml 的手感/高级项：\n";
        std::cout << "observer:\n";
        std::cout << "  filter_alpha: " << std::fixed << std::setprecision(6) << a.observer.filter_alpha << '\n';
        std::cout << "feel:\n";
        std::cout << "  "; print_joint_yaml_map("comfortable_torque", cfg_.joint_names, a.feel.comfortable_torque);
        std::cout << "  "; print_joint_yaml_map("follow_speed", cfg_.joint_names, a.feel.follow_speed);
        std::cout << "  "; print_joint_yaml_map("start_response_s", cfg_.joint_names, a.feel.start_response_s);
        std::cout << "  "; print_joint_yaml_map("q_elastic_start_speed", cfg_.joint_names, a.feel.q_elastic_start_speed);
        std::cout << "  "; print_joint_yaml_map("return_time_s", cfg_.joint_names, a.feel.return_time_s);
        std::cout << "  "; print_joint_yaml_map("max_retreat", cfg_.joint_names, a.feel.max_retreat);
        std::cout << "  "; print_joint_yaml_map("max_correction_speed", cfg_.joint_names, a.feel.max_correction_speed);
        std::cout << "  q_elastic_max_resistance_ratio: " << a.feel.q_elastic_max_resistance_ratio << '\n';
        std::cout << "calibration.friction.kinetic_feedforward_scale: "
            << a.calibration.friction.kinetic_feedforward_scale << '\n';
    }

    void run_admittance_live_tuning() {
        if(!ensure_admittance_tuning_active()) return;
        if(!cfg_.capability.admittance.enabled) {
            std::cout << "请先在 core.yaml 开启 admittance.enabled\n";
            return;
        }
        if(!set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)) return;

        while(true) {
            std::cout << "\n------------ 高级参数 ------------\n";
            std::cout << " 1. 查看派生 M / D / K（只读）\n";
            std::cout << " 2. Observer filter_alpha\n";
            std::cout << " 3. 最大导纳修正速度\n";
            std::cout << " 4. Q弹最大阻力倍率\n";
            std::cout << " 5. 摩擦主动助力比例\n";
            std::cout << " 6. 打印当前可写回参数\n";
            std::cout << " 0. 返回\n";
            const auto selection = read_int("请选择: ");
            if(!selection || *selection == 0) return;
            switch(*selection) {
                case 1:
                    print_derived_admittance_parameters();
                    break;
                case 2: {
                    const auto value = read_double("filter_alpha (0,1]: ");
                    if(!value || *value <= 0.0 || *value > 1.0) {
                        std::cout << "输入无效\n";
                        break;
                    }
                    auto candidate = cfg_.capability.admittance;
                    candidate.observer.filter_alpha = *value;
                    if(apply_runtime_admittance_cfg(candidate)) std::cout << "已应用\n";
                    break;
                }
                case 3:
                    set_max_correction_speed_advanced();
                    break;
                case 4: {
                    const auto value = read_double("Q弹最大阻力倍率 [1,10]: ");
                    if(!value || *value < 1.0 || *value > 10.0) {
                        std::cout << "输入无效\n";
                        break;
                    }
                    auto candidate = cfg_.capability.admittance;
                    candidate.feel.q_elastic_max_resistance_ratio = *value;
                    if(apply_runtime_admittance_cfg(candidate)) std::cout << "已应用\n";
                    break;
                }
                case 5: {
                    const auto value = read_double("摩擦主动助力比例 [0,0.7]，0=关闭: ");
                    if(!value || *value < 0.0 || *value > 0.7) {
                        std::cout << "输入无效\n";
                        break;
                    }
                    auto candidate = cfg_.capability.admittance;
                    candidate.calibration.friction.kinetic_feedforward_scale = *value;
                    if(apply_runtime_admittance_cfg(candidate)) std::cout << "已应用\n";
                    break;
                }
                case 6:
                    print_live_tuning_yaml();
                    break;
                default:
                    std::cout << "未知菜单编号\n";
                    break;
            }
        }
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
            const auto derived = derive_admittance_controller_cfg(a);
            std::cout << "admittance_observer     : " << observer_mode_name(a.observer.mode) << '\n';
            std::cout << "admittance_filter_alpha : " << a.observer.filter_alpha << '\n';
            print_vector("momentum_gain", a.observer.momentum_gain);
            print_vector("comfortable_torque", a.feel.comfortable_torque);
            print_vector("follow_speed", a.feel.follow_speed);
            print_vector("start_response_s", a.feel.start_response_s);
            print_vector("q_elastic_start_speed", a.feel.q_elastic_start_speed);
            print_vector("return_time_s", a.feel.return_time_s);
            print_vector("max_retreat", a.feel.max_retreat);
            print_vector("max_correction_speed", a.feel.max_correction_speed);
            print_vector("derived_M", derived.mass);
            print_vector("derived_D_follow", derived.damping);
            print_vector("derived_K_return", derived.stiffness);
            print_vector("admittance_torque_bias", a.calibration.torque_bias);
            print_vector("admittance_threshold", a.calibration.torque_threshold);
            std::cout << "friction_compensation   : " << (a.calibration.friction.enabled ? "ENABLED" : "DISABLED") << '\n';
            if(a.calibration.friction.enabled) {
                std::cout << "friction_vel_transition: " << a.calibration.friction.velocity_transition << '\n';
                std::cout << "friction_zero_adapt_s  : " << a.calibration.friction.zero_velocity_adaptation_s << '\n';
                std::cout << "friction_ff_scale      : " << a.calibration.friction.kinetic_feedforward_scale << '\n';
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
        options.robot_profile);
    const auto init_result = app.initialize();
    if(!init_result) {
        std::cerr << init_result.error() << '\n';
        return EXIT_FAILURE;
    }
    return app.run();
}
