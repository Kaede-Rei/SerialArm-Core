#include "robot_session.hpp"
#include "serial_arm/dynamics/gravity_correction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace serial_arm {

namespace {

/**
 * @brief 将 RobotFault 转换为 Python 会话错误文本
 * @param action 当前失败操作名称
 * @param fault Robot 故障详情
 * @return 包含顶层错误和有效子错误的文本
 */
std::string make_robot_error(const char* action, const RobotFault& fault) {
    std::ostringstream stream;
    stream << action << " failed; RobotErr=" << static_cast<int>(fault.code);

    const bool is_start_error = fault.code == RobotErr::MOTOR_BUS_CONNECT_FAILED || fault.code == RobotErr::MOTOR_BUS_ACTIVATE_FAILED;
    const bool is_cycle_error = fault.code == RobotErr::MOTOR_BUS_READ_FAILED || fault.code == RobotErr::MOTOR_BUS_WRITE_FAILED;
    const bool is_end_error = fault.code == RobotErr::MOTOR_BUS_DEACTIVATE_FAILED || fault.code == RobotErr::MOTOR_BUS_RECOVER_FAILED;
    const bool is_motor_bus_error = is_start_error || is_cycle_error || is_end_error;
    if(is_motor_bus_error) {
        stream << "; MotorBusErr=" << static_cast<int>(fault.motor_bus_err);
    }
    if(fault.code == RobotErr::MAPPER_FAILED) {
        stream << "; JointActuatorMapErr=" << static_cast<int>(fault.mapper_err);
    }
    if(fault.code == RobotErr::CTRLLER_FAILED) {
        stream << "; JointCtrllerErr=" << static_cast<int>(fault.ctrller_err);
    }
    if(fault.code == RobotErr::SAFETY_FAILED) {
        stream << "; SafetyErr=" << static_cast<int>(fault.safety_fault.code) << "; index=" << fault.safety_fault.index
            << "; value=" << fault.safety_fault.value << "; limit=" << fault.safety_fault.limit;
    }
    if(fault.code == RobotErr::MODEL_FEEDFORWARD_FAILED || fault.code == RobotErr::INVALID_MODEL_FEEDFORWARD) {
        stream << "; ModelFeedforwardErr=" << static_cast<int>(fault.model_feedforward_err);
    }
    if(fault.code == RobotErr::INTERACTION_FAILED) {
        stream << "; InteractionControllerErr=" << static_cast<int>(fault.interaction_err);
    }

    if(fault.code==RobotErr::REFERENCE_GOVERNOR_FAILED)
        stream << "; REFERENCE_GOVERNOR_FAILED / " << (fault.reference_governor_err==ReferenceGovernorErr::INFEASIBLE ? "INFEASIBLE" : "INVALID_INPUT");
    if(fault.code==RobotErr::MOTION_SOURCE_BUSY)
        stream << "; MOTION_SOURCE_BUSY: another motion task owns the reference";
    return stream.str();
}

/**
 * @brief 判断阻抗模式是否需要连续外部参考
 * @param mode 阻抗模式
 * @return 需要连续参考时返回 true，否则返回 false
 */
bool is_tracking_mode(JointImpedanceMode mode) {
    return mode == JointImpedanceMode::RIGID_TRACKING || mode == JointImpedanceMode::COMPLIANT_TRACKING;
}

/**
 * @brief 离线时的模拟电机 Bus
 */
class MockMotorBus final : public MotorBus {
public:
    explicit MockMotorBus(HardwareCapabilities capabilities) : capabilities_(std::move(capabilities)) {
        const auto size=capabilities_.size();
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

} // namespace

// ! ========================= 构 造 / 析 构 方 法 实 现 ========================= ! //

PyRobotSession::~PyRobotSession() {
    try {
        stop();
    }
    catch(...) {
    }
}

// ! ========================= 配 置 / 生 命 周 期 方 法 实 现 ========================= ! //

void PyRobotSession::configure(
    const std::string& config_file,
    const std::string& hardware_plugin,
    const std::string& hardware_config,
    const HardwareConfigOverrides& hardware_overrides) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(configured_) {
        throw SerialArmPythonError("RobotSession is already configured");
    }

    auto hardware_result = hardware_loader_.load(hardware_plugin, hardware_config, hardware_overrides);
    if(!hardware_result) {
        throw SerialArmPythonError("HardwareLoader failed; HardwareLoaderErr=" + std::to_string(static_cast<int>(hardware_result.error())));
    }
    std::unique_ptr<MotorBus> hardware_bus = std::move(hardware_result.value());

    const auto cfg_result = load_robot_cfg(config_file, hardware_bus->capabilities());
    if(!cfg_result) {
        throw SerialArmPythonError(cfg_result.error().message);
    }

    RobotCfg cfg = cfg_result.value();
    auto dynamics = std::make_unique<Dynamics>();
    const auto dynamics_result = configure_robot_dynamics(cfg, *dynamics);
    if(!dynamics_result) {
        throw SerialArmPythonError("Dynamics configure failed; DynamicsErr=" + std::to_string(static_cast<int>(dynamics_result.error())));
    }

    const auto ordered_capabilities = capabilities_for_joint_order(hardware_bus->capabilities(), cfg.joint_names);
    if(!ordered_capabilities) throw SerialArmPythonError("hardware joint identities do not match Core");
    std::vector<RobotSessionActuatorInfo> actuator_info;
    actuator_info.reserve(ordered_capabilities->size());
    for(std::size_t i = 0; i < ordered_capabilities->size(); ++i) {
        const auto& item = (*ordered_capabilities)[i];
        const std::string joint_name = i < cfg.joint_names.size() ? cfg.joint_names[i] : std::string{ };
        actuator_info.push_back(RobotSessionActuatorInfo{
            item.actuator_name,
            joint_name,
            item.min_pos,
            item.max_pos,
            item.max_vel,
            item.max_effort,
            item.max_kp,
            item.max_kd,
            });
    }

    RobotCfg robot_cfg = cfg;
    std::unique_ptr<MotorBus> bus;
    if(cfg.runtime.write_enabled) {
        bus = std::move(hardware_bus);
    }
    else {
        robot_cfg.runtime.write_enabled = true;
        bus = std::make_unique<MockMotorBus>(*ordered_capabilities);
    }

    cfg_ = std::move(cfg);
    dynamics_ = std::move(dynamics);
    actuator_info_ = std::move(actuator_info);

    auto robot = std::make_unique<Robot>();
    const auto robot_result = robot->configure(
        robot_cfg, std::move(bus), make_model_feedforward(), make_interaction_model_state());
    if(!robot_result) {
        dynamics_.reset();
        actuator_info_.clear();
        throw SerialArmPythonError(make_robot_error("Robot configure", robot_result.error()));
    }

    std::lock_guard<std::mutex> snapshot_lock(mutex_);
    robot_ = std::move(robot);
    requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    requested_gravity_scale_ = cfg_.dynamics.gravity_scale;
    goal_pos_.assign(cfg_.joint_names.size(), 0.0);
    ref_pos_.assign(cfg_.joint_names.size(), 0.0);
    ref_vel_.assign(cfg_.joint_names.size(), 0.0);
    snapshot_.robot_state = RobotState::INACTIVE;
    snapshot_.last_error.clear();
    snapshot_.valid = false;
    requested_fault_recovery_ = FaultRecoveryRequest::NONE;
    fault_recovery_sequence_ = 0;
    completed_fault_recovery_sequence_ = 0;
    fault_recovery_error_.clear();
    fault_hold_mode_cache_ = FaultHoldMode::RIGID_HOLD;
    pending_execution_update_={};execution_sequence_=completed_execution_sequence_=0;execution_error_.clear();
    pending_admittance_.reset();admittance_sequence_=completed_admittance_sequence_=0;admittance_error_.clear();
    runtime_.clear_commands();
    configured_ = true;
}

void PyRobotSession::start() {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!configured_ || !robot_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }
    if(running_.load()) {
        throw SerialArmPythonError("RobotSession is already running");
    }
    if(worker_.joinable()) {
        worker_.join();
    }
    if(robot_->get_state() == RobotState::FAULT) {
        throw SerialArmPythonError("Robot is in FAULT without a running session; call stop() to force INACTIVE before start()");
    }

    const auto activate_result = robot_->activate();
    if(!activate_result) {
        if(robot_->get_state() == RobotState::FAULT && robot_->is_fault_holding()) {
            static_cast<void>(robot_->force_deactivate());
        }
        throw SerialArmPythonError(make_robot_error("Robot activate", activate_result.error()));
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg_.safety=robot_->get_config().safety;
        const auto capabilities=robot_->hardware_capabilities();
        for(std::size_t i=0;i<capabilities.size();++i) {
            const auto& item=capabilities[i];
            actuator_info_[i]={item.actuator_name,cfg_.joint_names[i],item.min_pos,item.max_pos,
                item.max_vel,item.max_effort,item.max_kp,item.max_kd};
        }
        ref_pos_ = robot_->get_joint_state().pos;
        ref_vel_.assign(ref_pos_.size(), 0.0);
        goal_pos_ = ref_pos_;
        requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
        requested_gravity_scale_ = dynamics_->get_gravity_scale();
        impedance_sequence_ = 0;
        gravity_sequence_ = 0;
        speed_scale_ = 0.3;
        requested_execution_profile_.reset();
        pending_execution_update_={};completed_execution_sequence_=execution_sequence_;
        pending_admittance_.reset();completed_admittance_sequence_=admittance_sequence_;
        has_goal_ = false;
        requested_fault_recovery_ = FaultRecoveryRequest::NONE;
        fault_recovery_sequence_ = 0;
        completed_fault_recovery_sequence_ = 0;
        fault_recovery_error_.clear();
        fault_hold_mode_cache_ = FaultHoldMode::RIGID_HOLD;
        snapshot_.robot_state = RobotState::ACTIVE;
        snapshot_.last_error.clear();
        snapshot_.valid = false;
    }

    running_.store(true);
    try {
        worker_ = std::thread(&PyRobotSession::loop, this);
    }
    catch(...) {
        running_.store(false);
        static_cast<void>(robot_->deactivate());
        throw;
    }
}

void PyRobotSession::stop() {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    running_.store(false);
    fault_recovery_cv_.notify_all();
    if(worker_.joinable()) {
        worker_.join();
    }
    if(!robot_) {
        return;
    }

    const RobotState state = robot_->get_state();
    if(state == RobotState::ACTIVE) {
        const auto result = robot_->deactivate();
        if(!result) {
            throw SerialArmPythonError(make_robot_error("Robot deactivate", result.error()));
        }
    }
    else if(state == RobotState::FAULT) {
        const auto result = robot_->force_deactivate();
        if(!result) {
            throw SerialArmPythonError(make_robot_error("Robot force_deactivate", result.error()));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.robot_state = robot_->get_state();
    fault_hold_mode_cache_ = FaultHoldMode::RIGID_HOLD;
}

void PyRobotSession::reset_fault() {
    clear_fault();
}

void PyRobotSession::clear_fault() {
    submit_fault_recovery_request(FaultRecoveryRequest::CLEAR_FAULT);
}

void PyRobotSession::enter_fault_compliant_recovery() {
    submit_fault_recovery_request(FaultRecoveryRequest::ENTER_COMPLIANT);
}

void PyRobotSession::return_to_fault_rigid_hold() {
    submit_fault_recovery_request(FaultRecoveryRequest::RETURN_RIGID);
}

void PyRobotSession::submit_fault_recovery_request(FaultRecoveryRequest request) {
    if(!configured_ || !robot_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }

    std::unique_lock<std::mutex> lock(mutex_);
    fault_recovery_cv_.wait(lock, [this] {
        return completed_fault_recovery_sequence_ >= fault_recovery_sequence_ || !running_.load();
        });

    if(!running_.load()) {
        throw SerialArmPythonError("online FAULT recovery requires a running RobotSession");
    }
    if(snapshot_.robot_state != RobotState::FAULT) {
        throw SerialArmPythonError("FAULT recovery request requires RobotState::FAULT");
    }

    const std::uint64_t sequence = ++fault_recovery_sequence_;
    requested_fault_recovery_ = request;
    fault_recovery_error_.clear();

    fault_recovery_cv_.wait(lock, [this, sequence] {
        return completed_fault_recovery_sequence_ >= sequence || !running_.load();
        });

    if(completed_fault_recovery_sequence_ < sequence) {
        throw SerialArmPythonError("RobotSession stopped before FAULT recovery request completed");
    }
    if(!fault_recovery_error_.empty()) {
        throw SerialArmPythonError(fault_recovery_error_);
    }
}

// ! ========================= 命 令 / 调 参 方 法 实 现 ========================= ! //

void PyRobotSession::set_impedance_mode(JointImpedanceMode mode) {
    if(!running_.load()) {
        throw SerialArmPythonError("RobotSession must be running before set_impedance_mode()");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if(snapshot_.robot_state != RobotState::ACTIVE) {
        throw SerialArmPythonError("set_impedance_mode() requires RobotState::ACTIVE");
    }
    requested_impedance_mode_ = mode;
    requested_hold_policy_=HoldTargetPolicy::PRESERVE_NOMINAL;
    requested_execution_profile_.reset();
    has_goal_=false;
    ++impedance_sequence_;
}

void PyRobotSession::submit_execution_update(std::function<tl::expected<void,RobotFault>()> update) {
    std::lock_guard<std::mutex> single(admittance_update_mutex_);
    std::unique_lock<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    if(!configured_ || !robot_)throw SerialArmPythonError("RobotSession is not configured");
    if(!running_.load()) {
        if(worker_.joinable())worker_.join();
        auto result=update();if(!result)throw SerialArmPythonError(make_robot_error("execution profile",result.error()));return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if(snapshot_.robot_state!=RobotState::ACTIVE)throw SerialArmPythonError("execution profile requires ACTIVE or INACTIVE");
    pending_execution_update_=std::move(update);const auto sequence=++execution_sequence_;
    lifecycle.unlock();
    fault_recovery_cv_.wait(lock,[&]{return completed_execution_sequence_>=sequence||!running_.load();});
    if(completed_execution_sequence_<sequence)throw SerialArmPythonError("session stopped before execution profile update");
    if(!execution_error_.empty())throw SerialArmPythonError(execution_error_);
}
void PyRobotSession::set_hold_compensation_cfg(const HoldCompensationCfg& cfg) {
    submit_execution_update([this,cfg]{auto result=robot_->set_hold_compensation_cfg(cfg);if(result){std::lock_guard<std::mutex> lock(mutex_);cfg_.hold_compensation=cfg;}return result;});
}
void PyRobotSession::set_servo_tracking_gains(const JointImpedanceGains& gains) {
    submit_execution_update([this,gains]{auto result=robot_->set_servo_tracking_gains(gains);if(result){std::lock_guard<std::mutex> lock(mutex_);cfg_.ctrller.servo_tracking_gains=gains;}return result;});
}
void PyRobotSession::set_execution_gain_profile(JointGainProfile profile) {
    submit_execution_update([this,profile]{auto result=robot_->set_execution_gain_profile(profile);if(result){std::lock_guard<std::mutex> lock(mutex_);requested_execution_profile_=profile;}return result;});
}
void PyRobotSession::clear_execution_gain_profile() {
    submit_execution_update([this]{robot_->clear_execution_gain_profile();{std::lock_guard<std::mutex> lock(mutex_);requested_execution_profile_.reset();}return tl::expected<void,RobotFault>{};});
}

void PyRobotSession::set_reference_behavior(ReferenceBehavior behavior) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!configured_ || !robot_)throw SerialArmPythonError("RobotSession is not configured");
    if(running_.load() || robot_->get_state()!=RobotState::INACTIVE)
        throw SerialArmPythonError("set_reference_behavior() requires an INACTIVE session");
    if(worker_.joinable())worker_.join();
    const auto result=robot_->set_reference_behavior(behavior);
    if(!result)throw SerialArmPythonError(make_robot_error("Robot set_reference_behavior",result.error()));
    std::lock_guard<std::mutex> lock(mutex_);
    cfg_.capability.admittance.reference_behavior=behavior;
    runtime_.clear_commands();
}

void PyRobotSession::set_admittance_cfg(const AdmittanceCapabilityCfg& cfg) {
    std::lock_guard<std::mutex> single_update(admittance_update_mutex_);
    std::unique_lock<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!configured_ || !robot_)throw SerialArmPythonError("RobotSession is not configured");
    if(!running_.load()) {
        if(worker_.joinable())worker_.join();
        const auto result=robot_->set_admittance_cfg(cfg);
        if(!result)throw SerialArmPythonError(make_robot_error("Robot set_admittance_cfg",result.error()));
        std::lock_guard<std::mutex> lock(mutex_);cfg_.capability.admittance=cfg;return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if(snapshot_.robot_state!=RobotState::ACTIVE)throw SerialArmPythonError("set_admittance_cfg() requires ACTIVE or INACTIVE");
    pending_admittance_=cfg;admittance_error_.clear();
    const auto sequence=++admittance_sequence_;
    lifecycle_lock.unlock();
    fault_recovery_cv_.wait(lock,[&]{return completed_admittance_sequence_>=sequence || !running_.load();});
    if(completed_admittance_sequence_<sequence)throw SerialArmPythonError("RobotSession stopped before parameter update completed");
    if(!admittance_error_.empty())throw SerialArmPythonError(admittance_error_);
}

void PyRobotSession::set_model_feedforward_mode(ModelFeedforwardMode mode) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!cfg_.dynamics.gravity_correction_path.empty() && mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)
        throw SerialArmPythonError("gravity correction does not support full inverse dynamics");
    if(!configured_ || !robot_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }
    if(running_.load() || robot_->get_state() != RobotState::INACTIVE) {
        throw SerialArmPythonError("set_model_feedforward_mode() requires an INACTIVE session");
    }
    if(worker_.joinable()) {
        worker_.join();
    }

    const auto result = robot_->set_model_feedforward_mode(mode);
    if(!result) {
        throw SerialArmPythonError(make_robot_error("Robot set_model_feedforward_mode", result.error()));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    cfg_.runtime.model_feedforward_mode = mode;
}

void PyRobotSession::set_gravity_scale(const JointVector& gravity_scale) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    validate_joint_vector(gravity_scale, "gravity_scale");
    for(double value : gravity_scale) {
        if(value < 0.0 || value > 2.0) {
            throw SerialArmPythonError("gravity_scale values must be in [0, 2]");
        }
    }

    if(!running_.load()) {
        if(worker_.joinable()) {
            worker_.join();
        }
        const auto result = dynamics_->set_gravity_scale(gravity_scale);
        if(!result) {
            throw SerialArmPythonError("Dynamics set_gravity_scale failed; DynamicsErr=" + std::to_string(static_cast<int>(result.error())));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if(running_.load() && snapshot_.robot_state != RobotState::ACTIVE) {
        throw SerialArmPythonError("set_gravity_scale() requires RobotState::ACTIVE while the session is running");
    }
    requested_gravity_scale_ = gravity_scale;
    ++gravity_sequence_;
}

void PyRobotSession::move_to(const JointVector& pos, double speed_scale) {
    validate_joint_vector(pos, "pos");
    if(!running_.load()) {
        throw SerialArmPythonError("RobotSession must be running before move_to()");
    }
    if(!std::isfinite(speed_scale) || speed_scale <= 0.0 || speed_scale > 1.0) {
        throw SerialArmPythonError("speed_scale must be in (0, 1]");
    }

    for(std::size_t i = 0; i < pos.size(); ++i) {
        if(cfg_.safety.limits.has_position_limit.empty() || cfg_.safety.limits.has_position_limit[i] != 0) {
            const double min_pos = cfg_.safety.limits.min_pos[i] + cfg_.safety.limits.pos_margin[i];
            const double max_pos = cfg_.safety.limits.max_pos[i] - cfg_.safety.limits.pos_margin[i];
            if(pos[i] < min_pos || pos[i] > max_pos) {
                throw SerialArmPythonError("position goal exceeds Safety soft limit at joint index " + std::to_string(i));
            }
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if(snapshot_.robot_state != RobotState::ACTIVE) {
        throw SerialArmPythonError("move_to() requires RobotState::ACTIVE");
    }
    if(requested_impedance_mode_==JointImpedanceMode::COMPLIANT_DRAG || requested_execution_profile_==JointGainProfile::DRAG)
        throw SerialArmPythonError("leave drag mode before move_to()");
    if(!is_tracking_mode(requested_impedance_mode_)) {
        requested_impedance_mode_=(requested_execution_profile_==JointGainProfile::COMPLIANT || (!requested_execution_profile_ && requested_impedance_mode_==JointImpedanceMode::COMPLIANT_HOLD))?
            JointImpedanceMode::COMPLIANT_TRACKING:JointImpedanceMode::RIGID_TRACKING;
        ++impedance_sequence_;
    }
    goal_pos_ = pos;
    speed_scale_ = speed_scale;
    has_goal_ = true;
}

void PyRobotSession::hold_current() {
    if(!running_.load()) {
        throw SerialArmPythonError("RobotSession must be running before hold_current()");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if(snapshot_.robot_state != RobotState::ACTIVE) {
        throw SerialArmPythonError("hold_current() requires RobotState::ACTIVE");
    }
    requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    requested_hold_policy_=HoldTargetPolicy::CAPTURE_MEASURED;
    requested_execution_profile_.reset();
    ++impedance_sequence_;
    has_goal_ = false;
}

// ! ========================= 状 态 / 快 照 读 取 方 法 实 现 ========================= ! //

RobotSessionSnapshot PyRobotSession::get_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

RobotState PyRobotSession::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_.robot_state;
}

FaultHoldMode PyRobotSession::get_fault_hold_mode() const {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!robot_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_hold_mode_cache_;
}

bool PyRobotSession::is_configured() const noexcept {
    return configured_;
}

bool PyRobotSession::is_running() const noexcept {
    return running_.load();
}

RobotCfg PyRobotSession::get_config() const {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!configured_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return cfg_;
}

DynamicsInfo PyRobotSession::get_dynamics_info() const {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    if(!dynamics_) {
        throw SerialArmPythonError("Dynamics is not configured");
    }
    return dynamics_->get_info();
}

std::vector<RobotSessionActuatorInfo> PyRobotSession::get_actuator_info() const {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    return actuator_info_;
}

// ! ========================= 周 期 线 程 / 参 考 生 成 方 法 实 现 ========================= ! //

/**
 * @brief 运行固定频率控制周期并应用 Python 侧提交的最新请求
 *
 * 工作线程串行执行模式切换、重力比例更新、参考生成、Robot::set_cmd() 和 Robot::cycle()；退出前在 ACTIVE 状态下执行安全失能
 */
void PyRobotSession::loop() noexcept {
    std::uint64_t applied_impedance_sequence=0,applied_gravity_sequence=0;
    JointImpedanceMode applied_impedance_mode=JointImpedanceMode::RIGID_HOLD;
    bool fault_reported=false;
    auto handle_robot_failure = [&](const std::string& message) -> bool {
        set_worker_error(message);
        if(robot_ && robot_->get_state()==RobotState::FAULT && robot_->is_fault_holding()) {
            invalidate_motion_requests_for_fault(applied_impedance_sequence,applied_gravity_sequence,applied_impedance_mode);
            runtime_.clear_commands();
            return true;
        }
        running_.store(false);fault_recovery_cv_.notify_all();return false;
    };
    RuntimeHooks hooks;
    hooks.running=[this]{return running_.load();};
    hooks.before_cycle=[&](Robot::TimePoint now,double dt) {
        std::function<tl::expected<void,RobotFault>()> execution;
        std::uint64_t execution_sequence=0;
        {std::lock_guard<std::mutex> lock(mutex_);execution=std::move(pending_execution_update_);pending_execution_update_={};execution_sequence=execution_sequence_;}
        if(execution) {
            auto result=execution();std::lock_guard<std::mutex> lock(mutex_);
            execution_error_=result?std::string{}:make_robot_error("execution profile",result.error());
            completed_execution_sequence_=execution_sequence;fault_recovery_cv_.notify_all();
        }
        std::optional<AdmittanceCapabilityCfg> update;
        std::uint64_t update_sequence=0;
        {std::lock_guard<std::mutex> lock(mutex_);update=std::move(pending_admittance_);pending_admittance_.reset();update_sequence=admittance_sequence_;}
        if(update) {
            const auto applied=robot_->set_admittance_cfg(*update);
            std::lock_guard<std::mutex> lock(mutex_);
            if(applied){cfg_.capability.admittance=*update;admittance_error_.clear();}
            else admittance_error_=make_robot_error("Robot set_admittance_cfg",applied.error());
            completed_admittance_sequence_=update_sequence;
            fault_recovery_cv_.notify_all();
        }
        JointImpedanceMode impedance_mode;
        HoldTargetPolicy hold_policy;
        std::optional<JointGainProfile> execution_profile;
        JointVector gravity_scale;
        std::uint64_t impedance_sequence;
        std::uint64_t gravity_sequence;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            impedance_mode = requested_impedance_mode_;
            hold_policy=requested_hold_policy_;
            execution_profile=requested_execution_profile_;
            gravity_scale = requested_gravity_scale_;
            impedance_sequence = impedance_sequence_;
            gravity_sequence = gravity_sequence_;
        }

        if(gravity_sequence != applied_gravity_sequence) {
            const auto result = dynamics_->set_gravity_scale(gravity_scale);
            if(!result) {
                set_worker_error("Dynamics set_gravity_scale failed; DynamicsErr=" + std::to_string(static_cast<int>(result.error())));
                running_.store(false);
                fault_recovery_cv_.notify_all();
                return false;
            }
            applied_gravity_sequence = gravity_sequence;
        }

        if(impedance_sequence != applied_impedance_sequence) {
            const auto result = runtime_.set_impedance_mode(*robot_,impedance_mode, now,hold_policy);
            if(!result) {
                if(handle_robot_failure(make_robot_error("Robot set_impedance_mode", result.error()))) {
                    return false;
                }
                return false;
            }

            if(execution_profile) {
                const auto profile=robot_->set_execution_gain_profile(*execution_profile);
                if(!profile) {set_worker_error(make_robot_error("execution profile handover",profile.error()));running_.store(false);return false;}
            }
            applied_impedance_mode = impedance_mode;
            applied_impedance_sequence = impedance_sequence;
            ref_pos_ = robot_->get_joint_state().pos;
            ref_vel_.assign(ref_pos_.size(), 0.0);

            std::lock_guard<std::mutex> lock(mutex_);
            if(!has_goal_) {
                goal_pos_ = ref_pos_;
            }
        }

        if(is_tracking_mode(applied_impedance_mode)) {
            update_reference(dt);
            const auto cmd_result = runtime_.submit_reference(*robot_,JointPosVelCmd{ ref_pos_, ref_vel_ }, now,MotionSource::STREAM);
            if(!cmd_result) {
                if(handle_robot_failure(make_robot_error("Robot set_cmd", cmd_result.error()))) {
                    return false;
                }
                return false;
            }
        }

        return true;
    };
    hooks.after_step=[&](const RuntimeSnapshot& step) {
        if(step.cycle) {
            if(dynamics_->is_updated())static_cast<void>(dynamics_->update_reference(step.cycle->joint_ref_acc));
            fault_reported=false;
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.cycle=*step.cycle;
            snapshot_.sequence=step.sequence;snapshot_.monotonic_ns=step.monotonic_ns;
            snapshot_.dynamics=dynamics_->get_state();
            snapshot_.robot_state=RobotState::ACTIVE;
            snapshot_.valid=true;snapshot_.last_error.clear();
            fault_hold_mode_cache_=FaultHoldMode::RIGID_HOLD;
            return;
        }
        if(step.fault && (!fault_reported || !step.fault_hold_maintained)) {
            fault_reported=true;
            handle_robot_failure(make_robot_error("ControlRuntime step",*step.fault));
        }
        if(step.state==RobotState::FAULT) {
            std::lock_guard<std::mutex> lock(mutex_);
            if(pending_admittance_) {
                pending_admittance_.reset();completed_admittance_sequence_=admittance_sequence_;
                admittance_error_="parameter update cancelled by FAULT";fault_recovery_cv_.notify_all();
            }
        }
        if(robot_->get_state()==RobotState::FAULT && step.fault_hold_maintained) {
            process_fault_recovery_request(applied_impedance_sequence,applied_impedance_mode);
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.robot_state=robot_->get_state();
            snapshot_.cycle.joint_state=robot_->get_joint_state();snapshot_.valid=false;
            fault_hold_mode_cache_=robot_->get_state()==RobotState::FAULT ? robot_->get_fault_hold_mode() : FaultHoldMode::RIGID_HOLD;
        } else if(step.state!=RobotState::ACTIVE) {
            running_.store(false);fault_recovery_cv_.notify_all();
        }
    };
    try {runtime_.run(*robot_,cfg_.runtime.ctrl_frequency_hz,cfg_.safety.max_dt_s,hooks);}
    catch(const std::exception& error){set_worker_error(error.what());running_.store(false);}
    // A stopped worker must never leave a periodic FAULT hold unattended
    if(robot_) {
        const RobotState state = robot_->get_state();
        if(state == RobotState::ACTIVE) {
            const auto result = robot_->deactivate();
            if(!result) {
                set_worker_error(make_robot_error("Robot deactivate", result.error()));
            }
        }
        else if(state == RobotState::FAULT) {
            const auto result = robot_->force_deactivate();
            if(!result) {
                set_worker_error(make_robot_error("Robot force_deactivate", result.error()));
            }
        }
    }

    running_.store(false);
    fault_recovery_cv_.notify_all();
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.robot_state = robot_ ? robot_->get_state() : RobotState::UNCONFIGURED;
    fault_hold_mode_cache_ = FaultHoldMode::RIGID_HOLD;
}

void PyRobotSession::invalidate_motion_requests_for_fault(
    std::uint64_t& applied_impedance_sequence,
    std::uint64_t& applied_gravity_sequence,
    JointImpedanceMode& applied_impedance_mode) noexcept {
    try {
        const JointVector measured = robot_->get_joint_state().pos;
        const FaultHoldMode hold_mode = robot_->get_fault_hold_mode();
        std::lock_guard<std::mutex> lock(mutex_);
        ref_pos_ = measured;
        ref_vel_.assign(measured.size(), 0.0);
        goal_pos_ = measured;
        has_goal_ = false;
        requested_execution_profile_.reset();
        pending_execution_update_={};execution_error_="FAULT interrupted execution profile update";completed_execution_sequence_=execution_sequence_;
        pending_admittance_.reset();admittance_error_="FAULT interrupted admittance update";completed_admittance_sequence_=admittance_sequence_;
        fault_recovery_cv_.notify_all();
        speed_scale_ = 0.3;
        requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
        ++impedance_sequence_;
        applied_impedance_sequence = impedance_sequence_;
        requested_gravity_scale_ = dynamics_->get_gravity_scale();
        ++gravity_sequence_;
        applied_gravity_sequence = gravity_sequence_;
        applied_impedance_mode = JointImpedanceMode::RIGID_HOLD;
        snapshot_.robot_state = RobotState::FAULT;
        fault_hold_mode_cache_ = hold_mode;
    }
    catch(...) {
    }
}

void PyRobotSession::process_fault_recovery_request(
    std::uint64_t& applied_impedance_sequence,
    JointImpedanceMode& applied_impedance_mode) noexcept {
    FaultRecoveryRequest request = FaultRecoveryRequest::NONE;
    std::uint64_t sequence = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(fault_recovery_sequence_ <= completed_fault_recovery_sequence_) {
            return;
        }
        request = requested_fault_recovery_;
        sequence = fault_recovery_sequence_;
    }

    std::string error;
    try {
        tl::expected<void, RobotFault> result;
        const char* action = "Robot FAULT recovery";
        switch(request) {
            case FaultRecoveryRequest::ENTER_COMPLIANT:
                action = "Robot enter_fault_compliant_recovery";
                result = robot_->enter_fault_compliant_recovery();
                break;
            case FaultRecoveryRequest::RETURN_RIGID:
                action = "Robot return_to_fault_rigid_hold";
                result = robot_->return_to_fault_rigid_hold();
                break;
            case FaultRecoveryRequest::CLEAR_FAULT:
                action = "Robot clear_fault";
                result = robot_->clear_fault();
                break;
            case FaultRecoveryRequest::NONE:
                result = { };
                break;
        }

        if(!result) {
            error = make_robot_error(action, result.error());
        }
        else if(request == FaultRecoveryRequest::CLEAR_FAULT) {
            const JointVector measured = robot_->get_joint_state().pos;
            std::lock_guard<std::mutex> lock(mutex_);
            ref_pos_ = measured;
            ref_vel_.assign(measured.size(), 0.0);
            goal_pos_ = measured;
            has_goal_ = false;
            speed_scale_ = 0.3;
            requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
            ++impedance_sequence_;
            applied_impedance_sequence = impedance_sequence_;
            applied_impedance_mode = JointImpedanceMode::RIGID_HOLD;
            snapshot_.robot_state = RobotState::ACTIVE;
            snapshot_.last_error.clear();
            snapshot_.valid = false;
            fault_hold_mode_cache_ = FaultHoldMode::RIGID_HOLD;
        }
    }
    catch(const std::exception& ex) {
        error = ex.what();
    }
    catch(...) {
        error = "unknown exception while processing FAULT recovery request";
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(robot_ && robot_->get_state() == RobotState::FAULT) {
            snapshot_.robot_state = RobotState::FAULT;
            fault_hold_mode_cache_ = robot_->get_fault_hold_mode();
        }
        requested_fault_recovery_ = FaultRecoveryRequest::NONE;
        completed_fault_recovery_sequence_ = sequence;
        fault_recovery_error_ = std::move(error);
    }
    fault_recovery_cv_.notify_all();
}

/**
 * @brief 根据当前目标更新梯形位置速度参考
 * @param dt 当前参考生成时间步长
 */
void PyRobotSession::update_reference(double dt) {
    JointVector goal;
    double speed_scale;
    bool has_goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal = goal_pos_;
        speed_scale = speed_scale_;
        has_goal = has_goal_;
    }

    if(!has_goal) {
        ref_vel_.assign(ref_vel_.size(), 0.0);
        return;
    }

    advance_joint_reference(ref_pos_,ref_vel_,goal,cfg_.safety.limits.max_vel,
        cfg_.safety.limits.max_acc,speed_scale,dt);
}

/**
 * @brief 校验 Python 输入关节向量的长度和有限性
 * @param values 待校验关节向量
 * @param name 参数名称
 */
void PyRobotSession::validate_joint_vector(const JointVector& values, const char* name) const {
    if(!configured_) {
        throw SerialArmPythonError("RobotSession is not configured");
    }
    if(values.size() != cfg_.joint_names.size()) {
        throw SerialArmPythonError(std::string(name) + " size mismatch");
    }
    for(double value : values) {
        if(!std::isfinite(value)) {
            throw SerialArmPythonError(std::string(name) + " contains NaN or Inf");
        }
    }
}

/**
 * @brief 将周期线程错误写入会话快照
 * @param message 错误文本
 */
void PyRobotSession::set_worker_error(const std::string& message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.last_error = message;
        snapshot_.robot_state = robot_ ? robot_->get_state() : RobotState::UNCONFIGURED;
    }
    catch(...) {
    }
}

/**
 * @brief 创建 Robot 使用的动力学前馈回调
 * @return 根据前馈模式返回零向量、重力补偿或逆动力学结果的回调
 */
ModelFeedforwardFn PyRobotSession::make_model_feedforward() {
    return [this](auto mode, const auto& state, const auto& acc, const auto& ref_acc, double) -> tl::expected<JointVector, ModelFeedforwardErr> {
        const auto result = dynamics_->update(state, acc, ref_acc);
        if(!result) {
            return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
        }
        if(mode == ModelFeedforwardMode::GRAVITY) {
            return dynamics_->get_gravity_compensation();
        }
        if(mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            return dynamics_->get_inverse_dynamics();
        }
        return JointVector(cfg_.joint_names.size(), 0.0);
        };
}

InteractionModelStateFn PyRobotSession::make_interaction_model_state() {
    return [this](const JointState& state, double) -> tl::expected<InteractionModelState, ModelFeedforwardErr> {
        if(!dynamics_->update_state(state,JointVector(state.pos.size(),0.0))) {
            return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
        }
        InteractionModelState snapshot;
        snapshot.gravity = dynamics_->get_gravity_compensation();
        snapshot.coriolis = dynamics_->get_coriolis();
        const auto& mass = dynamics_->get_mass_matrix();
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
}

} // namespace serial_arm
