#include "serial_arm/robot.hpp"
#include "serial_arm/core/park_policy.hpp"
#include "serial_arm/core/reference_governor.hpp"
#include "serial_arm/core/gain_quantization.hpp"
#include "serial_arm/runtime/control_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <type_traits>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 判断向量中的所有元素是否均为有限值
 * @param values 待检查向量
 * @return 所有元素均有限时返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

tl::expected<void,SafetyFault> project_command_gains(JointCtrlCmd& cmd,const JointLimitCfg& limits) {
    const auto n=limits.max_kp.size();
    if(cmd.kp.size()!=n || cmd.kd.size()!=n) {
        SafetyFault fault;fault.code=SafetyErr::INVALID_CMD_SIZE;return tl::make_unexpected(fault);
    }
    for(std::size_t i=0;i<n;++i) {
        const auto kp=project_protocol_gain(cmd.kp[i],limits.max_kp[i],limits.kp_resolution.empty()?0:limits.kp_resolution[i]);
        const auto kd=project_protocol_gain(cmd.kd[i],limits.max_kd[i],limits.kd_resolution.empty()?0:limits.kd_resolution[i]);
        if(!kp || !kd) {
            SafetyFault fault;fault.code=!kp?SafetyErr::CMD_KP_LIMIT:SafetyErr::CMD_KD_LIMIT;fault.index=i;
            fault.value=!kp?cmd.kp[i]:cmd.kd[i];fault.limit=!kp?limits.max_kp[i]:limits.max_kd[i];
            return tl::make_unexpected(fault);
        }
        cmd.kp[i]=*kp;cmd.kd[i]=*kd;
    }
    return {};
}

// Unknown timestamps leave legacy backend safety checks unchanged; callers
// must still expose unknown measurement age in telemetry rather than zero
// Malformed, future, or missing per-axis timestamps in a supplied vector fail closed
static double available_feedback_age_s(const ActuatorState& state, std::size_t joints) {
    const auto& stamps = state.feedback_received_ns;
    if(stamps.empty()) return 0.0;
    if(stamps.size() != joints) return std::numeric_limits<double>::infinity();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
    const auto oldest = *std::min_element(stamps.begin(), stamps.end());
    const auto newest = *std::max_element(stamps.begin(), stamps.end());
    if(oldest == 0 || newest > static_cast<std::uint64_t>(now_ns)) return std::numeric_limits<double>::infinity();
    return static_cast<double>(now_ns-oldest)/1.0e9;
}

// Timestamped backends measure data age at transport reception. A delayed
// control thread is a separate dt-watchdog condition, not stale sensor data
static tl::expected<void,SafetyFault> check_observed_state(
    const Safety& safety, const JointState& joints, const ActuatorState& actuators,
    double legacy_read_interval = 0.0, bool position_recovery = false) {
    const auto& stamps=actuators.feedback_received_ns;
    const double age=stamps.empty()?legacy_read_interval:available_feedback_age_s(actuators,joints.pos.size());
    auto result=position_recovery?safety.check_state_for_position_recovery(joints,actuators,age):
        safety.check_state(joints,actuators,age);
    if(!result && !stamps.empty() && stamps.size()==joints.pos.size() &&
       (result.error().code==SafetyErr::STATE_TIMEOUT || result.error().code==SafetyErr::INVALID_STATE_AGE)) {
        result.error().index=static_cast<std::size_t>(std::min_element(stamps.begin(),stamps.end())-stamps.begin());
        if(result.error().code==SafetyErr::INVALID_STATE_AGE) {
            const auto now_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
            const auto invalid=std::find_if(stamps.begin(),stamps.end(),[&](auto stamp){return stamp==0 || stamp>static_cast<std::uint64_t>(now_ns);});
            if(invalid!=stamps.end())result.error().index=static_cast<std::size_t>(invalid-stamps.begin());
        }
    }
    return result;
}

InteractionControllerCfg make_interaction_cfg(const AdmittanceCapabilityCfg& admittance, std::size_t joints_count, bool shadow_required=false, const JointVector& velocity_resolution={}) {
    InteractionControllerCfg interaction_cfg;
    interaction_cfg.enabled = admittance.observer_enabled || !admittance.observer.momentum_gain.empty() || shadow_required;
    interaction_cfg.admittance_enabled=admittance.enabled;
    if(!interaction_cfg.enabled) return interaction_cfg;

    interaction_cfg.momentum.joints_count = joints_count;
    interaction_cfg.momentum.gain = admittance.observer.momentum_gain;
    interaction_cfg.momentum.initial_residual = admittance.calibration.torque_bias;
    interaction_cfg.external_torque.joints_count = joints_count;
    interaction_cfg.external_torque.torque_bias = admittance.calibration.torque_bias;
    interaction_cfg.external_torque.torque_threshold = admittance.calibration.torque_threshold;
    interaction_cfg.external_torque.friction = admittance.calibration.friction;
    interaction_cfg.external_torque.friction.velocity_resolution=velocity_resolution;
    interaction_cfg.admittance.joints_count = joints_count;
    interaction_cfg.admittance.enabled = admittance.joint_enabled;
    interaction_cfg.admittance.mass = admittance.controller.mass;
    interaction_cfg.admittance.damping = admittance.controller.damping;
    interaction_cfg.admittance.stiffness = admittance.controller.stiffness;
    interaction_cfg.admittance.max_delta_q = admittance.controller.max_delta_q;
    interaction_cfg.admittance.max_delta_q_dot = admittance.controller.max_delta_q_dot;
    return interaction_cfg;
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 析构时尽力安全停机并释放硬件资源
 */
Robot::~Robot() {
    if(motor_bus_) {
        disable_noexcept();
        motor_bus_->cleanup();
    }
}

/**
 * @brief 配置 Robot 并接管 MotorBus 所有权
 * @param cfg Robot 完整配置
 * @param motor_bus 待接管的 MotorBus 实例
 * @param model_feedforward 可选模型前馈函数
 * @return 配置成功返回空 expected，失败返回 RobotFault
 */
tl::expected<void, RobotFault> Robot::configure(
    const RobotCfg& requested_cfg,
    std::unique_ptr<MotorBus> motor_bus,
    ModelFeedforwardFn model_feedforward,
    InteractionModelStateFn interaction_model_state) {
    if(state_ != RobotState::UNCONFIGURED) {
        return tl::make_unexpected(make_fault(RobotErr::ALREADY_CONFIGURED));
    }
    if(!motor_bus) {
        return tl::make_unexpected(make_fault(RobotErr::NULL_MOTOR_BUS));
    }

    RobotCfg cfg=requested_cfg;
    cfg.ctrller.control_path=cfg.runtime.control_path;
    cfg.capability.admittance.observer_enabled=!cfg.capability.admittance.observer.momentum_gain.empty();
    cfg.capability.admittance.enabled=!cfg.capability.admittance.joint_enabled.empty();
    const auto hardware_safety=resolve_connected_safety_cfg(cfg,motor_bus->capabilities());
    if(!hardware_safety)return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    cfg.safety=*hardware_safety;

    const auto valid_cfg = validate_robot_core_cfg(cfg);
    if(!valid_cfg) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    }
    auto aligned_bus = align_motor_bus(std::move(motor_bus), cfg.joint_names);
    if(!aligned_bus) return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    motor_bus = std::move(*aligned_bus);
    const auto& capabilities = motor_bus->capabilities();
    if(capabilities.size() != cfg.joint_names.size())
        return tl::make_unexpected(make_fault(RobotErr::MOTOR_BUS_SIZE_MISMATCH));
    for(std::size_t i = 0; i < capabilities.size(); ++i) {
        if(!capabilities[i].joint_name.empty() && capabilities[i].joint_name != cfg.joint_names[i])
            return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    }
    if(motor_bus->size() != cfg.joint_names.size()) {
        return tl::make_unexpected(make_fault(RobotErr::MOTOR_BUS_SIZE_MISMATCH));
    }
    if(cfg.runtime.control_path==ControlPath::MIT && cfg.runtime.model_feedforward_mode != ModelFeedforwardMode::NONE && !model_feedforward) {
        RobotFault fault = make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED);
        fault.code = RobotErr::INVALID_CFG;
        return tl::make_unexpected(fault);
    }
    if((cfg.capability.admittance.observer_enabled || cfg.hold_compensation.enabled) &&
        !interaction_model_state) {
        RobotFault fault = make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED);
        fault.code = RobotErr::INVALID_CFG;
        return tl::make_unexpected(fault);
    }

    const auto ctrller_result = ctrller_.configure(cfg.ctrller);
    if(!ctrller_result) {
        return tl::make_unexpected(make_ctrller_fault(ctrller_result.error()));
    }
    const auto mapper_result = mapper_.configure(cfg.mapper);
    if(!mapper_result) {
        return tl::make_unexpected(make_mapper_fault(mapper_result.error()));
    }
    const auto safety_result = safety_.configure(cfg.safety);
    if(!safety_result) {
        return tl::make_unexpected(make_safety_fault(safety_result.error()));
    }

    friction_velocity_resolution_.resize(cfg.joint_names.size());
    for(std::size_t i=0;i<friction_velocity_resolution_.size();++i)
        friction_velocity_resolution_[i]=capabilities[i].velocity_resolution/std::abs(cfg.mapper.pos_ratio[i]);
    InteractionControllerCfg interaction_cfg = make_interaction_cfg(cfg.capability.admittance, cfg.joint_names.size(),cfg.hold_compensation.enabled,friction_velocity_resolution_);
    const auto interaction_result = interaction_controller_.configure(interaction_cfg);
    if(!interaction_result) {
        return tl::make_unexpected(make_interaction_fault(interaction_result.error()));
    }

    cfg_ = cfg;
    // Optional disabled HOLD compensation still has a complete runtime snapshot
    // so session persistence and baseline candidates never publish empty axes
    const auto joints=cfg_.joint_names.size();
    if(cfg_.hold_compensation.integral_gain.empty())cfg_.hold_compensation.integral_gain.assign(joints,.15);
    if(cfg_.hold_compensation.max_torque.empty())cfg_.hold_compensation.max_torque.assign(joints,.15);
    if(cfg_.hold_compensation.max_rate.empty())cfg_.hold_compensation.max_rate.assign(joints,.05);
    if(cfg_.hold_compensation.contact_threshold.empty())cfg_.hold_compensation.contact_threshold.assign(joints,.2);
    requested_safety_ = cfg.safety;
    motor_bus_ = std::move(motor_bus);
    model_feedforward_ = std::move(model_feedforward);
    interaction_model_state_ = std::move(interaction_model_state);
    last_fault_.reset();
    clear_runtime_state();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 连接、使能并用真实状态初始化控制器
 * @return 成功返回空 expected，失败返回 RobotFault
 */
tl::expected<void, RobotFault> Robot::activate() {
    if(state_ == RobotState::UNCONFIGURED) {
        return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    }
    if(state_ == RobotState::FAULT) {
        // A failed FIRST activation (including motors not powered yet) must
        // not trap the operator in FAULT forever. Only startup bus faults
        // without an active holding controller are eligible for retry.
        // An in-motion safety FAULT must still be cleared explicitly
        const bool startup_failure = !fault_hold_active_ && !has_completed_cycle_ && current_fault_ &&
            (current_fault_->code==RobotErr::MOTOR_BUS_CONNECT_FAILED ||
             current_fault_->code==RobotErr::MOTOR_BUS_ACTIVATE_FAILED);
        if(!startup_failure) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
        const auto reset = clear_fault();
        if(!reset) return tl::make_unexpected(reset.error());
    }
    if(state_ == RobotState::ACTIVE) {
        return tl::make_unexpected(make_fault(RobotErr::ALREADY_ACTIVE));
    }
    if(!cfg_.runtime.write_enabled) {
        return tl::make_unexpected(make_fault(RobotErr::WRITE_DISABLED));
    }

    const auto connected = motor_bus_->connect();
    if(!connected) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_CONNECT_FAILED, connected.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    RobotCfg requested=cfg_;
    requested.safety=requested_safety_;
    const auto limits=resolve_connected_safety_cfg(requested,motor_bus_->capabilities());
    if(!limits) return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    Safety replacement;
    const auto configured=replacement.configure(*limits);
    if(!configured) return tl::make_unexpected(make_safety_fault(configured.error()));

    // Connecting may refresh drive ranges and their feedback quantization.
    // Use those actual joint-side steps for the runtime friction model too.
    JointVector velocity_resolution(cfg_.joint_names.size());
    const auto& capabilities=motor_bus_->capabilities();
    for(std::size_t i=0;i<velocity_resolution.size();++i)
        velocity_resolution[i]=capabilities[i].velocity_resolution/std::abs(cfg_.mapper.pos_ratio[i]);
    const auto interaction=interaction_controller_.reconfigure(make_interaction_cfg(
        cfg_.capability.admittance,cfg_.joint_names.size(),cfg_.hold_compensation.enabled,velocity_resolution));
    if(!interaction) return tl::make_unexpected(make_interaction_fault(interaction.error()));
    friction_velocity_resolution_=std::move(velocity_resolution);
    safety_=std::move(replacement);
    cfg_.safety=*limits;

    ActuatorVector startup_speed_limit;
    if(cfg_.runtime.control_path==ControlPath::CSP) {
        startup_speed_limit.resize(cfg_.joint_names.size());
        for(std::size_t i=0;i<startup_speed_limit.size();++i)
            startup_speed_limit[i]=std::abs(cfg_.mapper.pos_ratio[i])*cfg_.safety.limits.max_vel[i];
    }
    const auto selected = motor_bus_->select_control_path(cfg_.runtime.control_path,startup_speed_limit);
    if(!selected) {
        const auto fault = make_bus_fault(RobotErr::MOTOR_BUS_ACTIVATE_FAILED, selected.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto active = motor_bus_->activate();
    if(!active) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_ACTIVATE_FAILED, active.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) {
        const RobotFault fault = make_mapper_fault(joint_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto checked_state = check_observed_state(safety_,joint_state.value(),actuator_state.value());
    if(!checked_state) {
        const RobotFault fault = make_safety_fault(checked_state.error());
        const SafetyAction action = safety_.action_for(checked_state.error().code);
        if(action == SafetyAction::STOP_HOLD) {
            actuator_state_ = actuator_state.value();
            joint_state_ = joint_state.value();
            has_state_ = true;
        }
        enter_fault(fault, action);
        return tl::make_unexpected(fault);
    }

    clear_calibration_hold_gains();
    clear_calibration_tracking_gains();
    ctrller_.reset();
    const auto initialized = ctrller_.initialize(joint_state.value());
    if(!initialized) {
        const RobotFault fault = make_ctrller_fault(initialized.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }
    interaction_controller_.reset();
    interaction_state_={};

    const auto history = safety_.reset_cmd_history(joint_state.value());
    if(!history) {
        const RobotFault fault = make_safety_fault(history.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    nominal_hold_target_=joint_state_.pos;
    hold_integral_torque_.assign(cfg_.joint_names.size(),0.0);
    joint_acc_.assign(cfg_.joint_names.size(), 0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(), 0.0);
    model_feedforward_cache_.assign(cfg_.joint_names.size(), 0.0);

    last_joint_cmd_.pos = joint_state_.pos;
    last_joint_cmd_.vel.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.tor.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kp.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kd.assign(cfg_.joint_names.size(), 0.0);

    has_state_ = true;
    feedback_aligner_.reset();observer_sample_ns_=0;

    accepted_nominal_cmd_=JointCtrlCmd{};
    last_nominal_velocity_.assign(cfg_.joint_names.size(),0.0);
    has_completed_cycle_ = false;
    dt_overrun_watchdog_.reset();
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;

    handover_source_gains_.reset();handover_source_cmd_={};
    const TimePoint activated_at = Clock::now();
    last_cycle_time_ = activated_at;
    last_state_time_ = activated_at;
    last_cmd_time_ = activated_at;

    last_fault_.reset();
    state_ = RobotState::ACTIVE;
    return {};
}

/**
 * @brief 设置跟踪参考命令
 */
tl::expected<void, RobotFault> Robot::set_cmd(const JointCmd& cmd, TimePoint now, bool continuous) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    // Validate the requested target before reference shaping so an illegal
    // request cannot be silently turned into an apparently safe trajectory.
    const auto violation=std::visit([&](const auto& value)->std::optional<SafetyFault> {
        using T=std::decay_t<decltype(value)>;
        if(value.pos.size()!=cfg_.joint_names.size()) return std::nullopt;
        for(std::size_t i=0;i<value.pos.size();++i) {
            if(cfg_.safety.limits.has_position_limit[i]) {
                const double lo=cfg_.safety.limits.min_pos[i]+cfg_.safety.limits.pos_margin[i];
                const double hi=cfg_.safety.limits.max_pos[i]-cfg_.safety.limits.pos_margin[i];
                if(value.pos[i]<lo-cfg_.safety.numeric_tolerance || value.pos[i]>hi+cfg_.safety.numeric_tolerance)
                    return SafetyFault{SafetyErr::CMD_POS_LIMIT,i,value.pos[i],value.pos[i]<lo?lo:hi};
            }
            if constexpr(!std::is_same_v<T,JointPosCmd>) {
                if(value.vel.size()==value.pos.size() && std::abs(value.vel[i])>cfg_.safety.limits.max_vel[i]+cfg_.safety.numeric_tolerance)
                    return SafetyFault{SafetyErr::CMD_VEL_LIMIT,i,value.vel[i],cfg_.safety.limits.max_vel[i]};
            }
        }
        return std::nullopt;
    },cmd);
    if(violation) return tl::make_unexpected(make_safety_fault(*violation));
    if(cfg_.runtime.control_path==ControlPath::CSP && std::holds_alternative<JointPosVelTorCmd>(cmd))
        return tl::make_unexpected(make_ctrller_fault(JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE));
    const auto result = ctrller_.set_cmd(cmd);
    if(!result) return tl::make_unexpected(make_ctrller_fault(result.error()));
    full_command_active_=false;

    last_cmd_time_ = now;
    has_external_cmd_ = true;
    continuous_cmd_ = continuous;
    return {};
}

/**
 * @brief 复用同一观测时刻的动力学、外力估计与可选导纳
 */
tl::expected<Robot::InteractionEvaluation,RobotFault> Robot::update_interaction(
    const JointState& state,const ActuatorState& actuator_state,const JointVector& joint_acc,
    bool joint_acc_valid,JointCtrlCmd joint_cmd,double controller_dt,bool admittance_active,bool drag,bool position_recovery) {
    const auto count=cfg_.joint_names.size();
    const double nominal_dt=1.0/cfg_.runtime.ctrl_frequency_hz;
    const auto state_received_at=Clock::now();
    bool new_measurement=false,observation_valid=false;
    double measurement_dt=0.0,feedback_skew_ms=std::numeric_limits<double>::quiet_NaN();
    MeasuredDynamicsSnapshot model_snapshot;
    JointState observer_state;
    std::uint64_t observer_timestamp_ns=0;
    InteractionOutput interaction_telemetry;
    std::vector<std::uint8_t> safety_position_margin_active;
    std::vector<std::uint8_t> safety_velocity_margin_active;

    const auto& stamps=actuator_state.feedback_received_ns;
    const auto& timing=actuator_state.feedback_timing_valid;
    const auto received_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(state_received_at.time_since_epoch()).count());
    if(stamps.size()==count && !stamps.empty()) {
        const auto bounds=std::minmax_element(stamps.begin(),stamps.end());
        feedback_skew_ms=static_cast<double>(*bounds.second-*bounds.first)/1.0e6;
    }
    const auto aligned=feedback_aligner_.update(state,stamps,timing,received_ns,
        static_cast<std::uint64_t>(cfg_.safety.state_timeout_s*1.0e9),
        static_cast<std::uint64_t>(2.0*nominal_dt*1.0e9));
    observation_valid=aligned.valid;
    new_measurement=aligned.fresh;
    if(new_measurement) {
        observer_timestamp_ns=aligned.timestamp_ns;
        measurement_dt=observer_sample_ns_==0 ? nominal_dt :
            (observer_timestamp_ns>observer_sample_ns_ ? static_cast<double>(observer_timestamp_ns-observer_sample_ns_)/1.0e9 : 0.0);
        if(measurement_dt<=0.0 || measurement_dt>cfg_.safety.state_timeout_s) {
            observation_valid=false;new_measurement=false;observer_sample_ns_=0;
            feedback_aligner_.reset();
        } else {
            observer_state=aligned.state;
            observer_sample_ns_=observer_timestamp_ns;
        }
    } else if(!observation_valid) observer_sample_ns_=0;

    // Dynamics and measured torque/velocity use the same interpolated
    // joint-side timestamp whenever an observer update is performed.
    const JointState& model_state=new_measurement ? observer_state : state;
    const auto snapshot=interaction_model_state_(model_state,controller_dt);
    if(!snapshot) {
        const auto fault=make_model_fault(snapshot.error());
            return tl::make_unexpected(fault);
    }
    model_snapshot=*snapshot;
    bool model_valid=model_snapshot.gravity.size()==count && model_snapshot.coriolis.size()==count &&
        model_snapshot.mass_matrix.size()==count && finite_vector(model_snapshot.gravity) && finite_vector(model_snapshot.coriolis);
    if(model_valid) for(std::size_t i=0;i<count;++i)
        if(model_snapshot.mass_matrix[i].size()!=count || !finite_vector(model_snapshot.mass_matrix[i]) ||
           model_snapshot.mass_matrix[i][i]<=0.0) {model_valid=false;break;}
    if(!model_valid) {
        const auto fault=make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD);
            return tl::make_unexpected(fault);
    }
    JointVector interaction_model_torque;
    if(joint_acc_valid) {
        interaction_model_torque=model_snapshot.gravity;
        for(std::size_t i=0;i<count;++i) {
            interaction_model_torque[i]+=model_snapshot.coriolis[i];
            for(std::size_t j=0;j<count;++j) interaction_model_torque[i]+=model_snapshot.mass_matrix[i][j]*joint_acc[j];
        }
    }
    const std::size_t joints_count = cfg_.joint_names.size();
    safety_position_margin_active.assign(joints_count, 0);
    safety_velocity_margin_active.assign(joints_count, 0);
    JointVector min_delta_q(joints_count, 0.0);
    JointVector max_delta_q(joints_count, 0.0);
    JointVector min_delta_q_dot(joints_count, 0.0);
    JointVector max_delta_q_dot(joints_count, 0.0);
    const auto& admittance_cfg = cfg_.capability.admittance;
    const auto& limits = cfg_.safety.limits;

    for(std::size_t i = 0; i < joints_count; ++i) {
        if(!admittance_cfg.enabled) continue;
        min_delta_q[i] = -admittance_cfg.controller.max_delta_q[i];
        max_delta_q[i] = admittance_cfg.controller.max_delta_q[i];
        min_delta_q_dot[i] = -admittance_cfg.controller.max_delta_q_dot[i];
        max_delta_q_dot[i] = admittance_cfg.controller.max_delta_q_dot[i];

        if(position_recovery && limits.has_position_limit[i] &&
           (joint_cmd.pos[i]<limits.min_pos[i]+limits.pos_margin[i] || joint_cmd.pos[i]>limits.max_pos[i]-limits.pos_margin[i])) {
            // A violated joint may start outside its normal interval, but
            // generated recovery motion may only reduce the violation.
            if(joint_cmd.pos[i]>limits.max_pos[i]-limits.pos_margin[i]) {
                min_delta_q[i]=std::max(min_delta_q[i],limits.min_pos[i]+limits.pos_margin[i]-joint_cmd.pos[i]);
                max_delta_q[i]=std::min(0.0,last_joint_cmd_.pos[i]-joint_cmd.pos[i]);max_delta_q_dot[i]=0;
            } else {
                max_delta_q[i]=std::min(max_delta_q[i],limits.max_pos[i]-limits.pos_margin[i]-joint_cmd.pos[i]);
                min_delta_q[i]=std::max(0.0,last_joint_cmd_.pos[i]-joint_cmd.pos[i]);min_delta_q_dot[i]=0;
            }
            continue;
        }
        if(limits.has_position_limit[i] != 0) {
            const double cmd_min_pos = limits.min_pos[i] + limits.pos_margin[i];
            const double cmd_max_pos = limits.max_pos[i] - limits.pos_margin[i];
            if(joint_cmd.pos[i] < cmd_min_pos || joint_cmd.pos[i] > cmd_max_pos) {
                min_delta_q[i] = 0.0;
                max_delta_q[i] = 0.0;
            }
            else {
                min_delta_q[i] = std::max(min_delta_q[i], cmd_min_pos - joint_cmd.pos[i]);
                max_delta_q[i] = std::min(max_delta_q[i], cmd_max_pos - joint_cmd.pos[i]);
            }
        }

        const double max_cmd_vel = limits.max_vel[i];
        if(joint_cmd.vel[i] < -max_cmd_vel || joint_cmd.vel[i] > max_cmd_vel) {
            min_delta_q_dot[i] = 0.0;
            max_delta_q_dot[i] = 0.0;
        }
        else {
            min_delta_q_dot[i] = std::max(min_delta_q_dot[i], -max_cmd_vel - joint_cmd.vel[i]);
            max_delta_q_dot[i] = std::min(max_delta_q_dot[i], max_cmd_vel - joint_cmd.vel[i]);
        }

        if(min_delta_q[i] > -admittance_cfg.controller.max_delta_q[i] || max_delta_q[i] < admittance_cfg.controller.max_delta_q[i]) {
            safety_position_margin_active[i] = 1;
        }
        if(min_delta_q_dot[i] > -admittance_cfg.controller.max_delta_q_dot[i] || max_delta_q_dot[i] < admittance_cfg.controller.max_delta_q_dot[i]) {
            safety_velocity_margin_active[i] = 1;
        }
    }

    GeneralizedMomentumInput momentum_input;
    momentum_input.measured_torque=model_state.tor;
    momentum_input.gravity=model_snapshot.gravity;
    momentum_input.coriolis=model_snapshot.coriolis;
    momentum_input.mass_matrix=model_snapshot.mass_matrix;
    momentum_input.velocity=model_state.vel;
    momentum_input.dt=measurement_dt>0?measurement_dt:nominal_dt;

    const auto interaction = interaction_controller_.update(InteractionInput{
        model_state.tor,
        interaction_model_torque,
        joint_cmd,
        controller_dt,
        std::move(min_delta_q),
        std::move(max_delta_q),
        std::move(min_delta_q_dot),
        std::move(max_delta_q_dot),
        model_state.vel,
        std::move(momentum_input),
        joint_acc_valid,
        new_measurement,
        observation_valid,
        measurement_dt,
        admittance_active,
        drag,
        model_state.pos,
        });
    if(!interaction) {
        const RobotFault fault = make_interaction_fault(interaction.error());
        return tl::make_unexpected(fault);
    }
    interaction_telemetry = interaction.value();
    joint_cmd = interaction->corrected_cmd;

    InteractionEvaluation evaluation;
    evaluation.output=std::move(interaction_telemetry);evaluation.model_snapshot=std::move(model_snapshot);
    evaluation.observer_state=std::move(observer_state);evaluation.new_measurement=new_measurement;
    evaluation.observation_valid=observation_valid;evaluation.measurement_dt=measurement_dt;
    evaluation.feedback_skew_ms=feedback_skew_ms;evaluation.observer_timestamp_ns=observer_timestamp_ns;
    evaluation.safety_position_margin_active=std::move(safety_position_margin_active);
    evaluation.safety_velocity_margin_active=std::move(safety_velocity_margin_active);
    return evaluation;
}

tl::expected<void,RobotFault> Robot::set_gravity_verification_active(bool active) {
    if(state_==RobotState::UNCONFIGURED)return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(active && state_!=RobotState::INACTIVE)return tl::make_unexpected(make_fault(RobotErr::NOT_INACTIVE));
    gravity_verification_active_=active;
    return {};
}

tl::expected<void, RobotFault> Robot::set_full_cmd(const JointCtrlCmd& cmd, TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    if(cfg_.runtime.control_path==ControlPath::CSP)
        return tl::make_unexpected(make_ctrller_fault(JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE));
    const auto result = ctrller_.set_full_cmd(cmd);
    if(!result) return tl::make_unexpected(make_ctrller_fault(result.error()));

    full_command_active_=true;
    continuous_cmd_=true;

    last_cmd_time_ = now;
    has_external_cmd_ = true;
    return {};
}

/**
 * @brief 切换阻抗模式
 */
bool Robot::begin_calibration_hold(const JointImpedanceGains& gains) {
    const auto n=cfg_.joint_names.size();
    if(state_!=RobotState::ACTIVE || cfg_.runtime.control_path!=ControlPath::MIT || gains.kp.size()!=n || gains.kd.size()!=n ||
       last_joint_cmd_.kp.size()!=n || last_joint_cmd_.kd.size()!=n)return false;
    for(std::size_t i=0;i<n;++i){
        if(!std::isfinite(gains.kp[i])||!std::isfinite(gains.kd[i]) ||
           gains.kp[i]<0||gains.kd[i]<0||
           gains.kp[i]>cfg_.safety.limits.max_kp[i] || gains.kd[i]>cfg_.safety.limits.max_kd[i])return false;
    }
    // Capture the ACTUAL previous control effort before changing mode.
    // No cycle should briefly revert to the user's RIGID Kp/Kd
    JointImpedanceGains origin{last_joint_cmd_.kp,last_joint_cmd_.kd};
    clear_calibration_tracking_gains();
    const auto switched=set_impedance_mode(JointImpedanceMode::RIGID,Clock::now());
    if(!switched)return false;
    calibration_hold_source_=std::move(origin);
    calibration_hold_gains_=gains;
    calibration_hold_started_=Clock::now();
    return true;
}

bool Robot::set_calibration_hold_gains(const JointImpedanceGains& gains) {
    const std::size_t count = cfg_.joint_names.size();
    if(state_ != RobotState::ACTIVE || cfg_.runtime.control_path!=ControlPath::MIT || ctrller_.get_impedance_mode() != JointImpedanceMode::RIGID ||
        gains.kp.size() != count || gains.kd.size() != count) return false;
    for(std::size_t i = 0; i < count; ++i) {
        if(!std::isfinite(gains.kp[i]) || !std::isfinite(gains.kd[i]) ||
            gains.kp[i] < 0.0 || gains.kd[i] < 0.0 ||
            gains.kp[i] > cfg_.safety.limits.max_kp[i] ||
            gains.kd[i] > cfg_.safety.limits.max_kd[i]) return false;
    }
    const auto now=Clock::now();
    JointImpedanceGains source=cfg_.ctrller.rigid_gains;
    if(calibration_hold_gains_ && calibration_hold_source_) {
        const double elapsed=std::chrono::duration<double>(now-calibration_hold_started_).count();
        const double x=std::clamp(elapsed/0.30,0.0,1.0);
        const double blend=x*x*(3.0-2.0*x);
        for(std::size_t i=0;i<count;++i){
            source.kp[i]=calibration_hold_source_->kp[i]+blend*(calibration_hold_gains_->kp[i]-calibration_hold_source_->kp[i]);
            source.kd[i]=calibration_hold_source_->kd[i]+blend*(calibration_hold_gains_->kd[i]-calibration_hold_source_->kd[i]);
        }
    }
    calibration_hold_source_=std::move(source);
    calibration_hold_gains_ = gains;
    calibration_hold_started_ = now;
    return true;
}

void Robot::clear_calibration_hold_gains() noexcept {
    if(state_==RobotState::ACTIVE && calibration_hold_gains_) {
        handover_source_cmd_=last_joint_cmd_;handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};handover_started_=Clock::now();
    }
    calibration_hold_gains_.reset();
    calibration_hold_source_.reset();
}

tl::expected<void, RobotFault> Robot::stop_calibration_response() {
    if(state_!=RobotState::ACTIVE)return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    enter_fault(make_fault(RobotErr::CALIBRATION_RESPONSE_UNSTABLE),SafetyAction::STOP_HOLD);
    return {};
}

bool Robot::set_calibration_tracking_gains(const JointImpedanceGains& gains, JointImpedanceMode mode) {
    if(mode!=JointImpedanceMode::DRAG && mode!=JointImpedanceMode::RIGID && mode!=JointImpedanceMode::COMPLIANT && mode!=JointImpedanceMode::SERVO_TRACKING)return false;
    const std::size_t count = cfg_.joint_names.size();
    if(state_ != RobotState::ACTIVE || cfg_.runtime.control_path!=ControlPath::MIT || gains.kp.size()!=count || gains.kd.size()!=count) return false;
    for(std::size_t i=0;i<count;++i) {
        if(!std::isfinite(gains.kp[i]) || !std::isfinite(gains.kd[i]) ||
            gains.kp[i]<0.0 || gains.kd[i]<0.0 ||
            gains.kp[i]>cfg_.safety.limits.max_kp[i] ||
            gains.kd[i]>cfg_.safety.limits.max_kd[i]) return false;
    }
    if(mode==JointImpedanceMode::DRAG && std::any_of(gains.kp.begin(),gains.kp.end(),[](double value){return value!=0;}))return false;
    const auto now=Clock::now();
    // Revisions start at the CURRENT blended gains, never jump back to the
    // user's rigid tracking gains on each automatic adjustment
    JointImpedanceGains source=mode==JointImpedanceMode::DRAG ? cfg_.ctrller.drag_gains :
        mode==JointImpedanceMode::COMPLIANT ? cfg_.ctrller.compliant_gains : cfg_.ctrller.rigid_gains;
    if(calibration_tracking_gains_ && calibration_tracking_source_ && calibration_tracking_mode_==mode) {
        gain_source_=GainSource::CALIBRATION_OVERRIDE;
        const double elapsed=std::chrono::duration<double>(now-calibration_tracking_started_).count();
        const double x=std::clamp(elapsed/0.12,0.0,1.0);
        const double blend=x*x*(3.0-2.0*x);
        for(std::size_t i=0;i<count;++i){
            source.kp[i]=calibration_tracking_source_->kp[i]+blend*(calibration_tracking_gains_->kp[i]-calibration_tracking_source_->kp[i]);
            source.kd[i]=calibration_tracking_source_->kd[i]+blend*(calibration_tracking_gains_->kd[i]-calibration_tracking_source_->kd[i]);
        }
    } else if(has_last_joint_cmd_ && last_joint_cmd_.kp.size()==count && last_joint_cmd_.kd.size()==count) {
        source=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};
    }
    calibration_tracking_source_=std::move(source);
    calibration_tracking_mode_=mode;
    calibration_tracking_gains_=gains;
    calibration_tracking_started_=now;
    return true;
}

void Robot::clear_calibration_tracking_gains() noexcept {
    if(state_==RobotState::ACTIVE && calibration_tracking_gains_) {
        handover_source_cmd_=last_joint_cmd_;handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};handover_started_=Clock::now();
    }
    calibration_tracking_gains_.reset();
    calibration_tracking_source_.reset();
    calibration_tracking_mode_=JointImpedanceMode::RIGID;
}

bool Robot::set_runtime_impedance_gains(JointImpedanceMode mode, const JointImpedanceGains& gains) {
    if(state_ != RobotState::INACTIVE) return false;
    const std::size_t count=cfg_.joint_names.size();
    if(gains.kp.size()!=count || gains.kd.size()!=count) return false;
    for(std::size_t i=0;i<count;++i) {
        if(!std::isfinite(gains.kp[i]) || !std::isfinite(gains.kd[i]) ||
           gains.kp[i]<0.0 || gains.kd[i]<0.0 ||
           gains.kp[i]>cfg_.safety.limits.max_kp[i] ||
           gains.kd[i]>cfg_.safety.limits.max_kd[i]) return false;
    }
    auto candidate=cfg_.ctrller;
    switch(mode) {
        case JointImpedanceMode::RIGID:
        case JointImpedanceMode::SERVO_TRACKING: candidate.rigid_gains=gains; break;
        case JointImpedanceMode::COMPLIANT: candidate.compliant_gains=gains; break;
        case JointImpedanceMode::DRAG: candidate.drag_gains=gains; break;
        default: return false;
    }
    const auto updated=ctrller_.configure(candidate);
    if(!updated)return false;
    cfg_.ctrller=std::move(candidate);
    clear_calibration_tracking_gains();
    clear_calibration_hold_gains();
    return true;
}

bool Robot::begin_park_rigid_gain_ramp(double duration_s, TimePoint now) {
    if(state_!=RobotState::ACTIVE ||
       ctrller_.get_impedance_mode()!=JointImpedanceMode::RIGID ||
       !std::isfinite(duration_s) || duration_s < 0.1 || duration_s > 2.0 ||
       now < last_cycle_time_) return false;
    // A calibration override must not supersede the shutdown gain handover
    clear_calibration_tracking_gains();
    park_rigid_ramp_started_=now;
    park_rigid_ramp_duration_s_=duration_s;
    return true;
}

tl::expected<void, RobotFault> Robot::set_impedance_mode(JointImpedanceMode mode, TimePoint now, HoldTargetPolicy target_policy) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE || !has_state_) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(mode==JointImpedanceMode::DRAG && cfg_.runtime.control_path==ControlPath::MIT && !model_feedforward_)
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    if(mode==JointImpedanceMode::DRAG && !cfg_.dynamics.gravity_model_validated && !gravity_verification_active_)
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    if(cfg_.runtime.control_path==ControlPath::CSP &&
       (mode==JointImpedanceMode::DRAG || mode==JointImpedanceMode::COMPLIANT) && !cfg_.capability.admittance.enabled)
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    if(mode==ctrller_.get_impedance_mode() && target_policy==HoldTargetPolicy::PRESERVE_NOMINAL) return {};
    clear_calibration_disturbance();
    const auto original_nominal=(accepted_nominal_cmd_.pos.size()==cfg_.joint_names.size())?accepted_nominal_cmd_.pos:joint_state_.pos;
    const auto previous_mode=ctrller_.get_impedance_mode();
    const auto mode_result=ctrller_.set_impedance_mode(mode,joint_state_);
    if(!mode_result) return tl::make_unexpected(make_ctrller_fault(mode_result.error()));
    full_command_active_=false;
    handover_source_cmd_=last_joint_cmd_;
    handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};
    handover_started_=now;
    clear_calibration_hold_gains();clear_calibration_tracking_gains();
    park_rigid_ramp_started_.reset();
    hold_integral_torque_.assign(cfg_.joint_names.size(),0.0);

    // Anchor the new interaction law to the last accepted physical reference.
    // The nominal command remains available regardless of the interaction mode.
    const bool leaving_drag = previous_mode==JointImpedanceMode::DRAG && mode!=JointImpedanceMode::DRAG;
    const auto anchor = target_policy==HoldTargetPolicy::CAPTURE_MEASURED || leaving_drag ? joint_state_.pos :
        (last_joint_cmd_.pos.size()==cfg_.joint_names.size() ? last_joint_cmd_.pos : original_nominal);
    nominal_hold_target_=anchor;
    interaction_controller_.reset_reference();
    interaction_state_.delta_q.assign(cfg_.joint_names.size(),0.0);
    interaction_state_.delta_q_dot.assign(cfg_.joint_names.size(),0.0);
    if(target_policy==HoldTargetPolicy::CAPTURE_MEASURED || leaving_drag) {
        ctrller_.set_hold_position(anchor);
        const auto anchored=ctrller_.set_cmd(JointPosCmd{anchor});
        if(!anchored) return tl::make_unexpected(make_ctrller_fault(anchored.error()));
        has_external_cmd_=false;
        continuous_cmd_=false;
        last_joint_cmd_.pos=anchor;last_joint_cmd_.vel.assign(cfg_.joint_names.size(),0.0);
        const auto history=safety_.reset_cmd_history(joint_state_);
        if(!history)return tl::make_unexpected(make_safety_fault(history.error()));
    }
    accepted_nominal_cmd_.pos=anchor;
    accepted_nominal_cmd_.vel.assign(cfg_.joint_names.size(),0.0);
    last_nominal_velocity_.assign(cfg_.joint_names.size(),0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(),0.0);
    has_last_joint_cmd_ = true;
    return {};
}

/**
 * @brief 设置模型前馈模式
 */
tl::expected<void, RobotFault> Robot::set_model_feedforward_mode(ModelFeedforwardMode mode) {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::INACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_INACTIVE));
    if(mode != ModelFeedforwardMode::NONE && !model_feedforward_) {
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    }

    cfg_.runtime.model_feedforward_mode = mode;
    return {};
}

/**
 * @brief 运行时更新导纳能力配置
 */
tl::expected<void, RobotFault> Robot::set_admittance_cfg(const AdmittanceCapabilityCfg& admittance_cfg) {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if((admittance_cfg.observer_enabled || !admittance_cfg.observer.momentum_gain.empty()) && !interaction_model_state_) {
        RobotFault fault = make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED);
        fault.code = RobotErr::INVALID_CFG;
        return tl::make_unexpected(fault);
    }

    RobotCfg candidate = cfg_;
    candidate.capability.admittance = admittance_cfg;
    candidate.capability.admittance.observer_enabled=!admittance_cfg.observer.momentum_gain.empty();
    candidate.capability.admittance.enabled=!admittance_cfg.joint_enabled.empty();
    if(!validate_robot_core_cfg(candidate)) return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));

    if(state_==RobotState::ACTIVE && candidate.capability.admittance.enabled!=cfg_.capability.admittance.enabled)
        return tl::make_unexpected(make_fault(RobotErr::NOT_INACTIVE));
    const auto configured=interaction_controller_.reconfigure(make_interaction_cfg(candidate.capability.admittance,cfg_.joint_names.size(),
        cfg_.hold_compensation.enabled,friction_velocity_resolution_));
    if(!configured) return tl::make_unexpected(make_interaction_fault(configured.error()));
    cfg_.capability.admittance=candidate.capability.admittance;
    return {};
}

tl::expected<void, RobotFault> Robot::set_control_path(ControlPath path) {
    if(state_==RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_!=RobotState::INACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_INACTIVE));
    if(path!=ControlPath::MIT && path!=ControlPath::CSP) return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    auto candidate=cfg_;
    candidate.runtime.control_path=path;candidate.ctrller.control_path=path;
    if(!validate_robot_core_cfg(candidate)) return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    if(!motor_bus_ || std::any_of(motor_bus_->capabilities().begin(),motor_bus_->capabilities().end(),
        [path](const auto& cap){return path==ControlPath::CSP ? !cap.supports_csp : !cap.supports_mit;}))
        return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    const auto configured=ctrller_.configure(candidate.ctrller);
    if(!configured)return tl::make_unexpected(make_ctrller_fault(configured.error()));
    cfg_.ctrller.control_path=path;cfg_.runtime.control_path=path;
    interaction_controller_.reset();
    interaction_state_={};
    feedback_aligner_.reset(); observer_sample_ns_=0;
    return {};
}

JointGainProfile Robot::get_gain_profile() const noexcept {
    const auto mode=ctrller_.get_impedance_mode();
    if(mode==JointImpedanceMode::DRAG) return JointGainProfile::DRAG;
    return (mode==JointImpedanceMode::COMPLIANT)?
        JointGainProfile::COMPLIANT:JointGainProfile::RIGID;
}
const JointVector& Robot::get_hold_target() const noexcept { return nominal_hold_target_; }
const HoldCompensationCfg& Robot::get_hold_compensation_cfg() const noexcept {return cfg_.hold_compensation;}
tl::expected<void,RobotFault> Robot::set_hold_compensation_cfg(const HoldCompensationCfg& hold) {
    if(state_==RobotState::UNCONFIGURED)return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(hold.enabled && !interaction_model_state_)return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    auto candidate=cfg_;candidate.hold_compensation=hold;
    if(!validate_robot_core_cfg(candidate))return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    const auto configured=interaction_controller_.reconfigure(make_interaction_cfg(candidate.capability.admittance,candidate.joint_names.size(),
        hold.enabled,friction_velocity_resolution_));
    if(!configured)return tl::make_unexpected(make_interaction_fault(configured.error()));
    cfg_.hold_compensation=hold;hold_integral_torque_.assign(cfg_.joint_names.size(),0.0);
    // FAULT cleanup restores configuration without altering fault-hold actuation
    if(state_==RobotState::ACTIVE) {
        handover_source_cmd_=last_joint_cmd_;handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};handover_started_=Clock::now();
    }
    return {};
}
tl::expected<void,RobotFault> Robot::set_calibration_model_feedforward_mode(ModelFeedforwardMode mode) {
    if(state_!=RobotState::ACTIVE && state_!=RobotState::INACTIVE)return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(mode!=ModelFeedforwardMode::NONE && mode!=ModelFeedforwardMode::GRAVITY && mode!=ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)
        return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    if(mode!=ModelFeedforwardMode::NONE && !model_feedforward_)return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    calibration_feedforward_mode_=mode;
    if(state_==RobotState::ACTIVE) {handover_source_cmd_=last_joint_cmd_;handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};handover_started_=Clock::now();}
    return {};
}
void Robot::clear_calibration_model_feedforward_mode() {
    calibration_feedforward_mode_.reset();
    if(state_==RobotState::ACTIVE) {handover_source_cmd_=last_joint_cmd_;handover_source_gains_=JointImpedanceGains{last_joint_cmd_.kp,last_joint_cmd_.kd};handover_started_=Clock::now();}
}

tl::expected<void,RobotFault> Robot::set_calibration_disturbance(const JointVector& torque,TimePoint now) {
    if(state_!=RobotState::ACTIVE || cfg_.runtime.control_path!=ControlPath::MIT || torque.size()!=cfg_.joint_names.size())
        return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    for(std::size_t i=0;i<torque.size();++i)
        if(!std::isfinite(torque[i]) || std::abs(torque[i])>.05*cfg_.safety.limits.max_effort[i])
            return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    calibration_disturbance_=torque;calibration_disturbance_at_=now;return {};
}
void Robot::clear_calibration_disturbance() noexcept { calibration_disturbance_.clear(); }

const AdmittanceCapabilityCfg& Robot::get_admittance_cfg() const noexcept {
    return cfg_.capability.admittance;
}


/**
 * @brief 执行一次完整控制周期
 */
tl::expected<RobotCycleOutput, RobotFault> Robot::cycle(TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || now < last_state_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        const RobotFault fault = make_fault(RobotErr::INVALID_TIME);
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const double dt = has_completed_cycle_ ? seconds_between(now, last_cycle_time_) : nominal_dt;
    // The actual wall-clock dt is observed for the safety watchdog. When one or
    // two consecutive cycles exceed the limit, advance the controller by the
    // nominal dt, not by the delayed wall-clock interval. Command/state timeouts
    // still use real timestamps and all other fault conditions stay immediate
    const bool dt_fault_required = dt_overrun_watchdog_.fault_required(dt, cfg_.safety.max_dt_s);
    const double controller_dt = dt > cfg_.safety.max_dt_s ? nominal_dt : dt;

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const TimePoint state_received_at = Clock::now();
    const double legacy_read_interval = has_completed_cycle_ ? seconds_between(state_received_at, last_state_time_) : 0.0;

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) {
        const RobotFault fault = make_mapper_fault(joint_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto checked_state = check_observed_state(safety_,joint_state.value(),actuator_state.value(),legacy_read_interval);
    if(!checked_state) {
        const auto code=checked_state.error().code;
        if(code==SafetyErr::JOINT_POS_LIMIT || code==SafetyErr::JOINT_VEL_LIMIT) {
            // These finite, fresh measurements are authoritative even when a
            // software motion boundary was crossed. Fault hold captures here.
            actuator_state_=*actuator_state;joint_state_=*joint_state;last_state_time_=state_received_at;
        }
        const RobotFault fault = make_safety_fault(checked_state.error());
        enter_fault(fault, safety_.action_for(code));
        return tl::make_unexpected(fault);
    }

    // Estimate before replacing the previous observation, using feedback sampling times
    bool joint_acc_valid = false;
    const JointVector joint_acc = estimate_joint_acc(joint_state.value(), actuator_state.value(), seconds_between(state_received_at, last_state_time_), joint_acc_valid);
    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    last_state_time_ = state_received_at;
    has_state_ = true;

    if(continuous_cmd_ && has_external_cmd_) {
        const double cmd_age_s = seconds_between(now, last_cmd_time_);
        const auto cmd_age = safety_.check_cmd_age(cmd_age_s);
        if(!cmd_age) {
            const RobotFault fault = make_safety_fault(cmd_age.error());
            enter_fault(fault, safety_.action_for(cmd_age.error().code));
            return tl::make_unexpected(fault);
        }
    }

    if(dt_fault_required) {
        const SafetyFault overdue{SafetyErr::INVALID_DT, std::numeric_limits<std::size_t>::max(), dt, cfg_.safety.max_dt_s};
        const RobotFault fault = make_safety_fault(overdue);
        enter_fault(fault, safety_.action_for(overdue.code));
        return tl::make_unexpected(fault);
    }


    JointCtrllerInput input;
    input.state = joint_state.value();
    input.model_feedforward.assign(cfg_.joint_names.size(), 0.0);
    input.dt = controller_dt;

    const auto ctrl_output = ctrller_.update(input);
    if(!ctrl_output) {
        const RobotFault fault = make_ctrller_fault(ctrl_output.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    JointCtrlCmd joint_cmd = ctrl_output->cmd;
    const auto count=cfg_.joint_names.size();
    const bool position_path=cfg_.runtime.control_path==ControlPath::CSP;
    const auto selected_mode=ctrller_.get_impedance_mode();
    if(!position_path) apply_gain_overrides(joint_cmd,now);
    if(selected_mode==JointImpedanceMode::DRAG) std::fill(joint_cmd.kp.begin(),joint_cmd.kp.end(),0.0);
    joint_cmd.path=cfg_.runtime.control_path;
    if(position_path) { joint_cmd.kp.assign(count,0.0); joint_cmd.kd.assign(count,0.0); joint_cmd.tor.assign(count,0.0); joint_cmd.position_speed_limit=cfg_.safety.limits.max_vel; }
    const JointVector requested_kp=joint_cmd.kp,requested_kd=joint_cmd.kd;
    const auto projected=project_command_gains(joint_cmd,cfg_.safety.limits);
    if(!projected) {
        const auto fault=make_safety_fault(projected.error());enter_fault(fault,SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const bool interaction_enabled=cfg_.capability.admittance.observer_enabled || cfg_.hold_compensation.enabled;
    const bool admittance_active=position_path && cfg_.capability.admittance.enabled && (selected_mode==JointImpedanceMode::COMPLIANT || selected_mode==JointImpedanceMode::DRAG);
    JointCtrlCmd nominal_cmd=joint_cmd;
    if(position_path && !continuous_cmd_ && !full_command_active_ && has_last_joint_cmd_) {
        auto q=accepted_nominal_cmd_.pos,v=accepted_nominal_cmd_.vel;
        if(q.size()!=count) {q=last_joint_cmd_.pos;v=last_joint_cmd_.vel;}
        advance_joint_reference(q,v,nominal_cmd.pos,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,1.0,controller_dt);
        nominal_cmd.pos=std::move(q);nominal_cmd.vel=std::move(v);
    }
    // Kp is zero in MIT drag, so follow the measured position in the
    // transmitted reference. This keeps the reference anchored for the next
    // interaction mode while retaining the independent nominal motion target.
    if(!position_path && selected_mode==JointImpedanceMode::DRAG) joint_cmd.pos=joint_state->pos;
    JointVector nominal_acc(count,0.0);
    std::vector<std::uint8_t> nominal_limited(count,0);
    if(position_path) {
        if(accepted_nominal_cmd_.pos.size()!=count) accepted_nominal_cmd_=last_joint_cmd_;
        ReferenceGovernorInput motion;
        motion.requested=nominal_cmd;
        motion.requested.kp.assign(count,0.0);motion.requested.kd.assign(count,0.0);motion.requested.tor.assign(count,0.0);
        motion.previous=accepted_nominal_cmd_;
        motion.measured=joint_state.value();
        motion.requested_acceleration.assign(count,0.0);
        motion.limits=cfg_.safety.limits;motion.dt=controller_dt;
        auto shaped=govern_reference(motion);
        if(!shaped) {
            auto fault=make_fault(RobotErr::REFERENCE_GOVERNOR_FAILED);
            fault.reference_governor_err=shaped.error();
            enter_fault(fault,SafetyAction::STOP_HOLD);
            return tl::make_unexpected(fault);
        }
        nominal_cmd.pos=shaped->accepted.pos;nominal_cmd.vel=shaped->accepted.vel;
        nominal_acc=shaped->acceleration;
        nominal_limited=shaped->reference_limited;
        joint_cmd.pos=nominal_cmd.pos;joint_cmd.vel=nominal_cmd.vel;
    } else if(last_nominal_velocity_.size()==count) {
        for(std::size_t i=0;i<count;++i) nominal_acc[i]=std::clamp(
            (nominal_cmd.vel[i]-last_nominal_velocity_[i])/controller_dt,
            -cfg_.safety.limits.max_acc[i],cfg_.safety.limits.max_acc[i]);
    }
    InteractionEvaluation interaction_evaluation;
    if(interaction_enabled) {
        auto evaluated=update_interaction(joint_state.value(),actuator_state.value(),joint_acc,joint_acc_valid,
            joint_cmd,controller_dt,admittance_active,selected_mode==JointImpedanceMode::DRAG);
        if(!evaluated) {enter_fault(evaluated.error(),SafetyAction::STOP_HOLD);return tl::make_unexpected(evaluated.error());}
        interaction_evaluation=std::move(*evaluated);
        joint_cmd=interaction_evaluation.output.corrected_cmd;
    }
    auto& interaction_telemetry=interaction_evaluation.output;
    auto& model_snapshot=interaction_evaluation.model_snapshot;
    auto& observer_state=interaction_evaluation.observer_state;
    const auto new_measurement=interaction_evaluation.new_measurement;
    const auto measurement_dt=interaction_evaluation.measurement_dt;
    const auto feedback_skew_ms=interaction_evaluation.feedback_skew_ms;
    const auto observer_timestamp_ns=interaction_evaluation.observer_timestamp_ns;
    const auto& safety_position_margin_active=interaction_evaluation.safety_position_margin_active;
    const auto& safety_velocity_margin_active=interaction_evaluation.safety_velocity_margin_active;

    const auto current_mode=ctrller_.get_impedance_mode();
    const bool positioning_hold=current_mode==JointImpedanceMode::RIGID || current_mode==JointImpedanceMode::COMPLIANT || current_mode==JointImpedanceMode::SERVO_TRACKING;
    // Shape point targets, but preserve externally timed continuous references.
    if(!position_path && !full_command_active_ && !continuous_cmd_ && positioning_hold && has_last_joint_cmd_) {
        auto q=last_joint_cmd_.pos,v=last_joint_cmd_.vel;
        advance_joint_reference(q,v,joint_cmd.pos,cfg_.safety.limits.max_vel,cfg_.safety.limits.max_acc,1.0,controller_dt);
        joint_cmd.pos=std::move(q);joint_cmd.vel=std::move(v);
    }
    JointVector joint_ref_acc=estimate_joint_ref_acc(joint_cmd,controller_dt);
    if(admittance_active && interaction_telemetry.delta_q_ddot.size()==count)
        for(std::size_t i=0;i<count;++i) joint_ref_acc[i]=nominal_acc[i]+interaction_telemetry.delta_q_ddot[i];
    const JointVector virtual_pos=joint_cmd.pos,virtual_vel=joint_cmd.vel,virtual_acc=joint_ref_acc;
    tl::expected<JointVector,RobotFault> model_feedforward=JointVector(count,0.0);
    if(!position_path && selected_mode==JointImpedanceMode::DRAG && model_feedforward_) {
        const JointVector zero(count,0.0);
        const auto gravity=model_feedforward_(ModelFeedforwardMode::GRAVITY,joint_state.value(),zero,zero,controller_dt);
        if(!gravity) model_feedforward=tl::make_unexpected(make_model_fault(gravity.error()));
        else model_feedforward=gravity.value();
    } else if(!position_path) model_feedforward=compute_model_feedforward(joint_state.value(),joint_acc,joint_ref_acc,controller_dt);
    if(!model_feedforward) {
        enter_fault(model_feedforward.error(),SafetyAction::STOP_HOLD);
        return tl::make_unexpected(model_feedforward.error());
    }
    if(model_feedforward->size()!=count || !finite_vector(*model_feedforward)) {
        const auto fault=make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD);
        enter_fault(fault,SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }
    for(std::size_t i=0;i<count;++i) joint_cmd.tor[i]+=model_feedforward.value()[i];
    JointVector applied_disturbance(count,0.0);
    if(!position_path && calibration_disturbance_.size()==count && now>=calibration_disturbance_at_ &&
       seconds_between(now,calibration_disturbance_at_)<=.1) {
        applied_disturbance=calibration_disturbance_;
        for(std::size_t i=0;i<count;++i)joint_cmd.tor[i]+=applied_disturbance[i];
    }
    if(hold_integral_torque_.size()!=count) hold_integral_torque_.assign(count,0.0);
    hold_integrator_frozen_=true;
    hold_integrator_freeze_reasons_.assign(count,"disabled_or_mode");
    const auto& hold_cfg=cfg_.hold_compensation;
    nominal_hold_target_=nominal_cmd.pos;
    const bool rigid_positioning_hold=(current_mode==JointImpedanceMode::RIGID ||
        current_mode==JointImpedanceMode::SERVO_TRACKING) && get_gain_profile()==JointGainProfile::RIGID;
    if(hold_cfg.enabled && rigid_positioning_hold && !position_path && !full_command_active_) {
        hold_integrator_frozen_=false;
        for(std::size_t i=0;i<count;++i) {
            auto& reason=hold_integrator_freeze_reasons_[i];reason="integrating";
            if(!interaction_telemetry.state.valid || interaction_telemetry.tau_ext_hat.size()!=count)reason="observer_invalid";
            else if(std::abs(joint_state->vel[i])>hold_cfg.velocity_threshold || std::abs(joint_cmd.vel[i])>hold_cfg.velocity_threshold)reason="moving";
            else if(std::abs(nominal_hold_target_[i]-joint_state->pos[i])>hold_cfg.error_threshold)reason="outside_error_window";
            else if(std::abs(interaction_telemetry.tau_ext_hat[i])>hold_cfg.contact_threshold[i])reason="contact";
            const bool quiet=reason=="integrating";hold_integrator_frozen_=hold_integrator_frozen_ || !quiet;
            const double pd=joint_cmd.kp[i]*(joint_cmd.pos[i]-joint_state->pos[i])+joint_cmd.kd[i]*(joint_cmd.vel[i]-joint_state->vel[i]);
            double candidate=hold_integral_torque_[i];
            if(quiet) candidate+=std::clamp(hold_cfg.integral_gain[i]*(nominal_hold_target_[i]-joint_state->pos[i])*controller_dt,
                -hold_cfg.max_rate[i]*controller_dt,hold_cfg.max_rate[i]*controller_dt);
            const double lower=std::max({-hold_cfg.max_torque[i],-cfg_.safety.limits.max_effort[i]-joint_cmd.tor[i],-cfg_.safety.limits.max_effort[i]-joint_cmd.tor[i]-pd});
            const double upper=std::min({hold_cfg.max_torque[i],cfg_.safety.limits.max_effort[i]-joint_cmd.tor[i],cfg_.safety.limits.max_effort[i]-joint_cmd.tor[i]-pd});
            if(lower<=upper)hold_integral_torque_[i]=std::clamp(candidate,lower,upper);
            else {hold_integral_torque_[i]=0;hold_integrator_frozen_=true;reason="effort_budget";}
            if(lower<=upper && std::abs(candidate-hold_integral_torque_[i])>1e-12)reason="torque_or_rate_limit";
            joint_cmd.tor[i]+=hold_integral_torque_[i];
        }
    } else hold_integral_torque_.assign(count,0.0);
    if(!position_path && selected_mode!=JointImpedanceMode::DRAG && handover_source_gains_ && !full_command_active_ && handover_source_cmd_.pos.size()==count) {
        const double x=std::clamp(std::chrono::duration<double>(now-handover_started_).count()/.3,0.0,1.0),blend=x*x*(3-2*x);
        for(std::size_t i=0;i<count;++i) {
            const double old_total=handover_source_cmd_.tor[i]+handover_source_cmd_.kp[i]*(handover_source_cmd_.pos[i]-joint_state->pos[i])+
                handover_source_cmd_.kd[i]*(handover_source_cmd_.vel[i]-joint_state->vel[i]);
            const double new_pd=joint_cmd.kp[i]*(joint_cmd.pos[i]-joint_state->pos[i])+joint_cmd.kd[i]*(joint_cmd.vel[i]-joint_state->vel[i]);
            joint_cmd.tor[i]+=(1-blend)*(old_total-joint_cmd.tor[i]-new_pd);
        }
    }
    ReferenceGovernorInput governor_input;
    governor_input.requested=joint_cmd;
    governor_input.measured=joint_state.value();
    governor_input.previous=last_joint_cmd_;
    governor_input.requested_acceleration=joint_ref_acc;
    governor_input.mass_matrix=model_snapshot.mass_matrix;
    governor_input.nonlinear=model_snapshot.gravity;
    if(governor_input.nonlinear.size()==count)
        for(std::size_t i=0;i<count;++i) governor_input.nonlinear[i]+=model_snapshot.coriolis[i];
    governor_input.external_torque=interaction_telemetry.tau_ext_hat;
    governor_input.limits=cfg_.safety.limits;
    governor_input.dt=controller_dt;
    governor_input.model_admittance=false;
    tl::expected<ReferenceGovernorOutput,ReferenceGovernorErr> governed=ReferenceGovernorOutput{};
    if(position_path) governed=govern_reference(governor_input);
    else {
        // Native MIT commands retain their explicit Safety admission semantics
        // rather than silently becoming a newly generated position trajectory
        Safety preview=safety_;
        auto admitted=preview.check_joint_cmd(joint_state.value(),joint_cmd,controller_dt);
        if(!admitted) {
            const auto fault=make_safety_fault(admitted.error());
            enter_fault(fault,safety_.action_for(admitted.error().code));
            return tl::make_unexpected(fault);
        }
        ReferenceGovernorOutput direct;
        direct.accepted=*admitted;direct.acceleration=joint_ref_acc;
        direct.reference_limited.assign(count,0);direct.torque_limited.assign(count,0);
        direct.predicted_total_torque.resize(count);
        bool feasible=true;
        for(std::size_t i=0;i<count;++i) {
            direct.predicted_total_torque[i]=admitted->tor[i]+admitted->kp[i]*(admitted->pos[i]-joint_state->pos[i])+
                admitted->kd[i]*(admitted->vel[i]-joint_state->vel[i]);
            if(std::abs(direct.predicted_total_torque[i])>cfg_.safety.limits.max_effort[i]+cfg_.safety.numeric_tolerance)
                feasible=false;
        }
        if(feasible) governed=std::move(direct);
        else governed=tl::make_unexpected(ReferenceGovernorErr::INFEASIBLE);
    }
    if(!governed) {
        auto fault=make_fault(RobotErr::REFERENCE_GOVERNOR_FAILED);
        fault.reference_governor_err=governed.error();
        enter_fault(fault,SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }
    joint_cmd=governed->accepted;
    joint_ref_acc=governed->acceleration;
    for(std::size_t i=0;i<count;++i) model_feedforward.value()[i]=joint_cmd.tor[i]-nominal_cmd.tor[i];
    if(admittance_active) {
        // Back-calculate accepted state instead of integrating behind saturation
        JointVector accepted_delta(count),accepted_velocity(count);
        for(std::size_t i=0;i<count;++i) {
            accepted_delta[i]=joint_cmd.pos[i]-nominal_cmd.pos[i];
            accepted_velocity[i]=joint_cmd.vel[i]-nominal_cmd.vel[i];
        }
        auto committed=interaction_controller_.accept_reference(accepted_delta,accepted_velocity);
        if(!committed) return tl::make_unexpected(make_interaction_fault(committed.error()));
        interaction_telemetry.delta_q_ddot.resize(count);
        for(std::size_t i=0;i<count;++i) interaction_telemetry.delta_q_ddot[i]=joint_ref_acc[i]-nominal_acc[i];
        interaction_telemetry.delta_q=accepted_delta;
        interaction_telemetry.delta_q_dot=accepted_velocity;
        interaction_telemetry.state.delta_q=accepted_delta;
        interaction_telemetry.state.delta_q_dot=accepted_velocity;
    }

    const auto safe_cmd = safety_.check_joint_cmd(joint_state.value(), joint_cmd, controller_dt);
    if(!safe_cmd) {
        const RobotFault fault = make_safety_fault(safe_cmd.error());
        enter_fault(fault, safety_.action_for(safe_cmd.error().code));
        return tl::make_unexpected(fault);
    }

    const auto actuator_cmd = mapper_.to_actuator_cmd(safe_cmd.value());
    if(!actuator_cmd) {
        const RobotFault fault = make_mapper_fault(actuator_cmd.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const auto write_result = motor_bus_->write(actuator_cmd.value());
    const auto command_written_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
    if(!write_result) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_WRITE_FAILED, write_result.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    joint_acc_ = joint_acc;
    joint_ref_acc_ = joint_ref_acc;
    model_feedforward_cache_ = model_feedforward.value();
    last_joint_cmd_ = safe_cmd.value();
    last_nominal_velocity_=nominal_cmd.vel;
    accepted_nominal_cmd_=nominal_cmd;
    has_state_ = true;
    has_completed_cycle_ = true;
    has_last_joint_cmd_ = true;
    last_cycle_time_ = now;
    last_state_time_ = state_received_at;

    RobotCycleOutput output;
    output.actuator_state = actuator_state_;
    output.joint_state = joint_state_;
    output.interaction_mode=ctrller_.get_impedance_mode();
    output.control_path=cfg_.runtime.control_path;
    output.joint_acc = joint_acc_;
    output.joint_ref_acc = joint_ref_acc_;
    output.model_feedforward = model_feedforward_cache_;
    output.joint_cmd = safe_cmd.value();
    output.requested_kp=requested_kp;output.requested_kd=requested_kd;
    output.actuator_cmd = actuator_cmd.value();
    output.joint_acc_valid = joint_acc_valid;
    const auto& received = actuator_state_.feedback_received_ns;
    if(received.size() == cfg_.joint_names.size()) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
        const auto oldest = *std::min_element(received.begin(), received.end());
        const auto newest = *std::max_element(received.begin(), received.end());
        if(oldest > 0 && newest <= static_cast<std::uint64_t>(now_ns))
            output.feedback_age_ms = static_cast<double>(now_ns - oldest) / 1.0e6;
    }
    output.dt = dt;
    output.gain_profile=get_gain_profile();output.gain_source=gain_source_;
    output.effective_kp=joint_cmd.kp;output.effective_kd=joint_cmd.kd;output.hold_target_pos=nominal_hold_target_;
    output.hold_integral_torque=hold_integral_torque_;output.hold_integrator_frozen=hold_integrator_frozen_;
    output.hold_integrator_freeze_reasons=hold_integrator_freeze_reasons_;output.calibration_disturbance=applied_disturbance;
    output.command_written_ns=command_written_ns;
    output.parameter_update_progress=interaction_controller_.parameter_update_progress();
    output.observer_valid=interaction_telemetry.state.valid;
    output.observer_state=observer_state;
    output.observer_timestamp_ns=observer_timestamp_ns;
    output.new_measurement=new_measurement;
    output.measurement_dt=measurement_dt;
    output.feedback_skew_ms=feedback_skew_ms;
    output.nominal_ref_pos=nominal_cmd.pos;output.nominal_ref_vel=nominal_cmd.vel;output.nominal_ref_acc=nominal_acc;
    output.virtual_ref_pos=virtual_pos;output.virtual_ref_vel=virtual_vel;output.virtual_ref_acc=virtual_acc;
    output.accepted_ref_pos=joint_cmd.pos;output.accepted_ref_vel=joint_cmd.vel;output.accepted_ref_acc=joint_ref_acc;
    output.predicted_total_torque=governed->predicted_total_torque;
    output.reference_limited=governed->reference_limited;output.torque_limited=governed->torque_limited;
    for(std::size_t i=0;i<count;++i) if(nominal_limited[i]) output.reference_limited[i]=1;
    output.admittance_active = admittance_active;
    if(!interaction_enabled) interaction_state_={};
    if(interaction_enabled) {
        output.residual_raw = interaction_telemetry.residual.residual;
        output.full_id_residual_raw = interaction_telemetry.full_id_residual_raw;
        output.residual_filtered = interaction_telemetry.residual.residual_filtered;
        output.bias_compensated = interaction_telemetry.bias_compensated;
        output.friction_residual_hat = interaction_telemetry.friction_residual_hat;
        output.friction_compensated = interaction_telemetry.friction_compensated;
        output.interaction_state = interaction_telemetry.state;
        interaction_state_ = interaction_telemetry.state;
        output.tau_ext_hat = interaction_telemetry.tau_ext_hat;
        output.delta_q = interaction_telemetry.delta_q;
        output.delta_q_dot = interaction_telemetry.delta_q_dot;
        output.delta_q_ddot = interaction_telemetry.delta_q_ddot;
        output.torque_threshold_active = interaction_telemetry.threshold_active;
        output.delta_q_limited = interaction_telemetry.delta_q_limited;
        output.delta_q_dot_limited = interaction_telemetry.delta_q_dot_limited;
        output.safety_position_margin_active = std::move(safety_position_margin_active);
        output.safety_velocity_margin_active = std::move(safety_velocity_margin_active);
    }
    return output;
}

/**
 * @brief 安全停止并失能，回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::deactivate() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ == RobotState::INACTIVE) return {};

    const auto result = motor_bus_->deactivate();
    if(!result && result.error() != MotorBusErr::NOT_CONNECTED) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_DEACTIVATE_FAILED, result.error());
        state_ = RobotState::FAULT;
        last_fault_ = fault;
        return tl::make_unexpected(fault);
    }

    clear_calibration_hold_gains();
    clear_calibration_tracking_gains();
    park_rigid_ramp_started_.reset();
    ctrller_.reset();
    interaction_controller_.reset();
    interaction_state_={};
    safety_.clear_cmd_history();
    clear_runtime_state();
    last_fault_.reset();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 无条件停止并失能，允许从 ACTIVE 或 FAULT 回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::force_deactivate() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::INACTIVE) return {};

    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorCtrlCmd{};
    const auto result = motor_bus_->deactivate();
    if(!result && result.error() != MotorBusErr::NOT_CONNECTED) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_DEACTIVATE_FAILED, result.error());
        last_fault_ = fault;
        state_ = RobotState::FAULT;
        return tl::make_unexpected(fault);
    }

    clear_calibration_hold_gains();
    clear_calibration_tracking_gains();
    park_rigid_ramp_started_.reset();
    ctrller_.reset();
    interaction_controller_.reset();
    interaction_state_={};
    safety_.clear_cmd_history();
    clear_runtime_state();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 清除 FAULT 锁存并回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::reset_fault() {
    return clear_fault();
}

/**
 * @brief 人工请求进入 FAULT 受限柔性恢复
 */
tl::expected<void, RobotFault> Robot::enter_fault_drag_recovery() {
    if(state_==RobotState::UNCONFIGURED)return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_!=RobotState::FAULT)return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    const auto& policy=cfg_.safety.fault_recovery;
    if(!fault_hold_active_ || !policy.allow_drag_recovery || !policy.require_operator_request ||
       !cfg_.dynamics.gravity_model_validated || !is_drag_recovery_fault_allowed())
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    const bool position_path=cfg_.runtime.control_path==ControlPath::CSP;
    if(position_path ? (!cfg_.capability.admittance.enabled || !interaction_model_state_) :
        (!model_feedforward_ || cfg_.ctrller.drag_gains.kd.size()!=cfg_.joint_names.size()))
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    const auto actuators=motor_bus_->read();
    if(!actuators)return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED,actuators.error()));
    const auto joints=mapper_.to_joint_state(*actuators);
    if(!joints)return tl::make_unexpected(make_mapper_fault(joints.error()));
    const bool position_recovery=current_fault_ && current_fault_->code==RobotErr::SAFETY_FAILED &&
        current_fault_->safety_fault.code==SafetyErr::JOINT_POS_LIMIT;
    const auto checked=check_observed_state(safety_,*joints,*actuators,0,position_recovery);
    if(!checked)return tl::make_unexpected(make_safety_fault(checked.error()));
    const auto reset=reset_reference_to_measured(*joints);
    if(!reset)return reset;
    const auto mode=ctrller_.set_impedance_mode(JointImpedanceMode::DRAG,*joints);
    if(!mode)return tl::make_unexpected(make_ctrller_fault(mode.error()));
    actuator_state_=*actuators;joint_state_=*joints;
    fault_hold_mode_=FaultHoldMode::DRAG_RECOVERY;fault_recovery_started_at_=Clock::now();
    return maintain_fault_hold();
}

/**
 * @brief 操作员取消柔性恢复并返回 FAULT 刚性保持
 */
tl::expected<void, RobotFault> Robot::return_to_fault_rigid_hold() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    const auto reset=reset_reference_to_measured(joint_state_);
    if(!reset)return reset;
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    return maintain_fault_hold();
}

/**
 * @brief 清除 FAULT 并以当前实测位置进入 ACTIVE + RIGID
 */
tl::expected<void, RobotFault> Robot::clear_fault() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    if(!fault_hold_active_) {
        // Disabled faults keep observing hardware. Clearing one only restores
        // INACTIVE after fresh healthy feedback; it never re-enables a drive.
        const auto observed=update_fault_reaction(Clock::now());
        if(!observed)return observed;
        if(clear_fault_valid_cycles_<3 || std::any_of(joint_state_.vel.begin(),joint_state_.vel.end(),
            [&](double v){return std::abs(v)>cfg_.shutdown.velocity_tolerance;}))
            return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
        ctrller_.reset();interaction_controller_.reset();safety_.clear_cmd_history();
        clear_runtime_state();current_fault_.reset();last_fault_.reset();state_=RobotState::INACTIVE;
        return {};
    }

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error()));

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) return tl::make_unexpected(make_mapper_fault(joint_state.error()));

    const auto checked_state = check_observed_state(safety_,joint_state.value(),actuator_state.value());
    if(!checked_state) return tl::make_unexpected(make_safety_fault(checked_state.error()));

    for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
        if(std::abs(joint_state->vel[i]) > cfg_.shutdown.velocity_tolerance) {
            return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
        }
    }

    if(clear_fault_valid_cycles_ < 3) {
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    }

    const auto reference = reset_reference_to_measured(joint_state.value());
    if(!reference) return tl::make_unexpected(reference.error());

    const TimePoint resumed_at = Clock::now();
    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    last_cycle_time_ = resumed_at;
    last_state_time_ = resumed_at;
    last_cmd_time_ = resumed_at;
    has_state_ = true;
    feedback_aligner_.reset();observer_sample_ns_=0;

    accepted_nominal_cmd_=JointCtrlCmd{};
    last_nominal_velocity_.assign(cfg_.joint_names.size(),0.0);
    has_completed_cycle_ = false;
    dt_overrun_watchdog_.reset();
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorCtrlCmd{};
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    current_fault_.reset();
    state_ = RobotState::ACTIVE;
    return {};
}

/**
 * @brief 在 FAULT 状态持续刷新刚性保持命令
 */
tl::expected<void, RobotFault> Robot::maintain_fault_hold() {
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    const bool was_holding=fault_hold_active_;
    const auto result = update_fault_reaction(Clock::now());
    if(!result && was_holding) {
        fault_hold_active_ = false;
        disable_noexcept();
    }
    return result;
}

/**
 * @brief 获取当前 Robot 生命周期状态
 */
RobotState Robot::get_state() const noexcept {
    return state_;
}

/**
 * @brief 获取当前控制器阻抗模式
 */
JointImpedanceMode Robot::get_impedance_mode() const noexcept {
    return ctrller_.get_impedance_mode();
}

/**
 * @brief 获取当前模型前馈模式
 */
ModelFeedforwardMode Robot::get_model_feedforward_mode() const noexcept {
    return calibration_feedforward_mode_.value_or(cfg_.runtime.model_feedforward_mode);
}

/**
 * @brief 获取最近一次合法的关节状态
 */
const JointState& Robot::get_joint_state() const noexcept {
    return joint_state_;
}

/**
 * @brief 获取最近一次关节加速度估计
 */
const JointVector& Robot::get_joint_acc() const noexcept {
    return joint_acc_;
}

/**
 * @brief 获取最近一次关节参考加速度
 */
const JointVector& Robot::get_joint_ref_acc() const noexcept {
    return joint_ref_acc_;
}

/**
 * @brief 获取最近一次模型前馈力矩
 */
const InteractionState& Robot::get_interaction_state() const noexcept {
    return interaction_state_;
}

const JointVector& Robot::get_model_feedforward() const noexcept {
    return model_feedforward_cache_;
}

/**
 * @brief 获取最近一次合法的执行器状态
 */
const ActuatorState& Robot::get_actuator_state() const noexcept {
    return actuator_state_;
}

/**
 * @brief 获取最近一次锁存的故障信息
 */
const tl::optional<RobotFault>& Robot::get_last_fault() const noexcept {
    return last_fault_;
}

/**
 * @brief 获取当前是否正在执行故障刚性保持
 */
bool Robot::is_fault_holding() const noexcept {
    return fault_hold_active_;
}

/**
 * @brief 获取 FAULT 内部保持模式
 */
FaultHoldMode Robot::get_fault_hold_mode() const noexcept {
    return fault_hold_mode_;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 计算当前周期的关节加速度估计
 */
JointVector Robot::estimate_joint_acc(const JointState& state, const ActuatorState& observed, double dt, bool& valid) const {
    JointVector result(cfg_.joint_names.size(), 0.0);
    valid = false;
    if(!has_completed_cycle_ || joint_state_.vel.size() != state.vel.size()) return result;
    const auto& current = observed.feedback_received_ns;
    const auto& previous = actuator_state_.feedback_received_ns;
    const bool timestamped = current.size() == result.size() && previous.size() == result.size();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    const auto timing_ok = [&](const ActuatorState& value, std::size_t i) {
        return value.feedback_timing_valid.empty() ||
            (value.feedback_timing_valid.size()==result.size() && value.feedback_timing_valid[i]!=0);
    };
    bool all_valid = timestamped && cfg_.runtime.joint_acc_filter_alpha > 0.0;
    for(std::size_t i = 0; i < result.size(); ++i) {
        double sample_dt = dt;
        if(timestamped) {
            if(previous[i] > 0 && current[i] == previous[i] && current[i] <= static_cast<std::uint64_t>(now_ns) &&
               static_cast<double>(now_ns-current[i])/1.0e9 <= cfg_.safety.state_timeout_s) {
                // No new observation: preserve filter memory but do not label
                // a repeated cached sample as a measured zero acceleration
                result[i] = i < joint_acc_.size() ? joint_acc_[i] : 0.0;
                all_valid = false;
                continue;
            }
            if(previous[i] == 0 || current[i] <= previous[i] || current[i] > static_cast<std::uint64_t>(now_ns) ||
               static_cast<double>(now_ns-current[i])/1.0e9 > cfg_.safety.state_timeout_s) {
                all_valid = false;
                continue;
            }
            if(!timing_ok(observed,i) || !timing_ok(actuator_state_,i)) {
                all_valid=false;
                continue;
            }
            sample_dt = static_cast<double>(current[i]-previous[i])/1.0e9;
        }
        // Never use controller retiming to differentiate measured velocities
        if(!std::isfinite(sample_dt) || sample_dt <= 0 || sample_dt > cfg_.safety.state_timeout_s) {
            all_valid = false;
            continue;
        }
        const double raw = (state.vel[i]-joint_state_.vel[i])/sample_dt;
        const double previous_acc = i < joint_acc_.size() ? joint_acc_[i] : 0.0;
        result[i] = cfg_.runtime.joint_acc_filter_alpha*raw + (1.0-cfg_.runtime.joint_acc_filter_alpha)*previous_acc;
        if(!std::isfinite(result[i])) { result[i] = 0.0; all_valid = false; }
    }
    valid = all_valid;
    return result;
}

/**
 * @brief 计算当前周期的关节参考加速度
 */
JointVector Robot::estimate_joint_ref_acc(const JointCtrlCmd& cmd, double dt) const {
    JointVector result(cfg_.joint_names.size(), 0.0);
    if(!has_last_joint_cmd_ || last_joint_cmd_.vel.size() != cmd.vel.size()) return result;

    for(std::size_t i = 0; i < result.size(); ++i) {
        const double raw_ref_acc = (cmd.vel[i] - last_joint_cmd_.vel[i]) / dt;
        const double max_ref_acc = cfg_.safety.limits.max_acc[i];
        result[i] = std::clamp(raw_ref_acc, -max_ref_acc, max_ref_acc);
    }
    return result;
}

/**
 * @brief 计算当前周期的模型前馈项
 */
tl::expected<JointVector, RobotFault> Robot::compute_model_feedforward(const JointState& state, const JointVector& joint_acc, const JointVector& joint_ref_acc, double dt) const {
    if(!model_feedforward_) {
        if(get_model_feedforward_mode() == ModelFeedforwardMode::NONE) {
            return JointVector(cfg_.joint_names.size(), 0.0);
        }
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    }

    const auto result = model_feedforward_(get_model_feedforward_mode(), state, joint_acc, joint_ref_acc, dt);
    if(!result) return tl::make_unexpected(make_model_fault(result.error()));
    if(result->size() != cfg_.joint_names.size() || !finite_vector(result.value())) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD));
    }
    return result.value();
}

/**
 * @brief 构造仅包含通用错误码的故障对象
 */
RobotFault Robot::make_fault(RobotErr code) const noexcept {
    RobotFault fault;
    fault.code = code;
    return fault;
}

/**
 * @brief 构造包含 MotorBus 子错误的故障对象
 */
RobotFault Robot::make_bus_fault(RobotErr code, MotorBusErr err) const noexcept {
    RobotFault fault = make_fault(code);
    fault.motor_bus_err = err;
    return fault;
}

/**
 * @brief 构造包含映射器子错误的故障对象
 */
RobotFault Robot::make_mapper_fault(JointActuatorMapErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::MAPPER_FAILED);
    fault.mapper_err = err;
    return fault;
}

/**
 * @brief 构造包含控制器子错误的故障对象
 */
RobotFault Robot::make_ctrller_fault(JointCtrllerErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::CTRLLER_FAILED);
    fault.ctrller_err = err;
    return fault;
}

/**
 * @brief 构造包含安全检查子错误的故障对象
 */
RobotFault Robot::make_safety_fault(const SafetyFault& safety_fault) const noexcept {
    RobotFault fault = make_fault(RobotErr::SAFETY_FAILED);
    fault.safety_fault = safety_fault;
    return fault;
}

/**
 * @brief 构造包含模型前馈子错误的故障对象
 */
RobotFault Robot::make_model_fault(ModelFeedforwardErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::MODEL_FEEDFORWARD_FAILED);
    fault.model_feedforward_err = err;
    return fault;
}

/**
 * @brief 构造包含导纳能力子错误的故障对象
 */
RobotFault Robot::make_interaction_fault(InteractionControllerErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::INTERACTION_FAILED);
    fault.interaction_err = err;
    return fault;
}

/**
 * @brief 进入 FAULT 状态并执行对应安全动作
 */
void Robot::enter_fault(const RobotFault& fault, SafetyAction action) noexcept {
    clear_calibration_disturbance();
    gravity_verification_active_=false;
    park_rigid_ramp_started_.reset();
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorCtrlCmd{};
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    clear_fault_valid_cycles_ = 0;
    dt_overrun_watchdog_.reset();
    if(action == SafetyAction::STOP_HOLD) {
        // FAULT 默认刚性保持：先清外部命令，再以最新合法实测位置建参考，避免旧轨迹在清故障后继续执行
        fault_hold_active_ = start_fault_hold_noexcept();
        if(!fault_hold_active_) stop_or_disable_noexcept();
    }
    else {
        disable_noexcept();
    }

    clear_calibration_hold_gains();
    clear_calibration_tracking_gains();
    ctrller_.reset();
    interaction_controller_.reset();
    interaction_state_={};
    safety_.clear_cmd_history();
    has_external_cmd_ = false;
    state_ = RobotState::FAULT;
    last_fault_ = fault;
    current_fault_ = fault;
}

/**
 * @brief 尝试停机，失败后降级为失能
 */
void Robot::stop_or_disable_noexcept() noexcept {
    if(!motor_bus_) return;
    const auto stopped = motor_bus_->stop();
    if(!stopped) (void)motor_bus_->deactivate();
}

/**
 * @brief 使用最近一次合法状态构造并发送故障刚性保持命令
 */
bool Robot::start_fault_hold_noexcept() noexcept {
    if(!motor_bus_ || !has_state_ || joint_state_.pos.size() != cfg_.joint_names.size() ||
        seconds_between(Clock::now(), last_state_time_) > cfg_.safety.state_timeout_s ||
        available_feedback_age_s(actuator_state_, cfg_.joint_names.size()) > cfg_.safety.state_timeout_s) return false;

    if(!reset_reference_to_measured(joint_state_)) return false;
    const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const auto hold_cmd = build_fault_joint_cmd(joint_state_, FaultHoldMode::RIGID_HOLD, nominal_dt);
    if(!hold_cmd) return false;
    safety_.clear_cmd_history();
    const auto safe_hold = safety_.check_joint_cmd_for_position_recovery(joint_state_, hold_cmd.value(), nominal_dt);
    if(!safe_hold) return false;
    const auto actuator_cmd = mapper_.to_actuator_cmd(safe_hold.value());
    if(!actuator_cmd) return false;
    const auto written = motor_bus_->write(actuator_cmd.value());
    if(!written) return false;

    fault_hold_cmd_ = actuator_cmd.value();
    return true;
}

/**
 * @brief 根据 FAULT 内部模式刷新保持命令
 */
tl::expected<void, RobotFault> Robot::update_fault_reaction(TimePoint now) {
    fault_feedback_.observer_valid=false;fault_feedback_.new_measurement=false;fault_feedback_.admittance_active=false;
    fault_feedback_.feedback_age_ms=available_feedback_age_s(actuator_state_,cfg_.joint_names.size())*1000;
    const double nominal_dt=1.0/cfg_.runtime.ctrl_frequency_hz;
    const double elapsed=last_cycle_time_==TimePoint{}?nominal_dt:seconds_between(now,last_cycle_time_);
    const double dt=std::clamp(elapsed,nominal_dt,cfg_.safety.max_dt_s);
    const auto actuators=motor_bus_->read();
    if(!actuators) {clear_fault_valid_cycles_=0;return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED,actuators.error()));}
    const auto joints=mapper_.to_joint_state(*actuators);
    if(!joints) {clear_fault_valid_cycles_=0;return tl::make_unexpected(make_mapper_fault(joints.error()));}
    actuator_state_=*actuators;joint_state_=*joints;has_state_=true;
    fault_feedback_={};fault_feedback_.actuator_state=*actuators;fault_feedback_.joint_state=*joints;
    const auto n=cfg_.joint_names.size();fault_feedback_.joint_cmd=last_joint_cmd_;
    for(auto* values:{&fault_feedback_.joint_cmd.pos,&fault_feedback_.joint_cmd.vel,&fault_feedback_.joint_cmd.tor,&fault_feedback_.joint_cmd.kp,&fault_feedback_.joint_cmd.kd})
        if(values->size()!=n)values->assign(n,0.0);
    fault_feedback_.control_path=cfg_.runtime.control_path;fault_feedback_.dt=dt;
    fault_feedback_.interaction_mode=fault_hold_mode_==FaultHoldMode::DRAG_RECOVERY?JointImpedanceMode::DRAG:JointImpedanceMode::RIGID;
    fault_feedback_.feedback_age_ms=available_feedback_age_s(*actuators,cfg_.joint_names.size())*1000;
    Safety health=safety_;
    if(!fault_hold_active_) {auto health_cfg=cfg_.safety;health_cfg.require_all_actuators_enabled=false;
        const auto configured=health.configure(health_cfg);if(!configured)return tl::make_unexpected(make_safety_fault(configured.error()));}
    auto checked=check_observed_state(health,*joints,*actuators);
    if(checked && cfg_.runtime.control_path==ControlPath::CSP) for(std::size_t i=0;i<n;++i) {
        if(std::abs(joints->tor[i])>cfg_.safety.limits.max_effort[i]+cfg_.safety.numeric_tolerance) {
            checked=tl::make_unexpected(SafetyFault{SafetyErr::JOINT_EFFORT_LIMIT,i,joints->tor[i],cfg_.safety.limits.max_effort[i]});break;
        }
    }
    if(checked)++clear_fault_valid_cycles_;else clear_fault_valid_cycles_=0;
    last_state_time_=now;last_cycle_time_=now;
    if(!fault_hold_active_)return checked?tl::expected<void,RobotFault>{}:tl::make_unexpected(make_safety_fault(checked.error()));
    const bool position_recovery=current_fault_ && current_fault_->code==RobotErr::SAFETY_FAILED &&
        current_fault_->safety_fault.code==SafetyErr::JOINT_POS_LIMIT;
    if(!checked && !(position_recovery && checked.error().code==SafetyErr::JOINT_POS_LIMIT))
        return tl::make_unexpected(make_safety_fault(checked.error()));
    if(fault_hold_mode_==FaultHoldMode::DRAG_RECOVERY &&
       seconds_between(now,fault_recovery_started_at_)>cfg_.safety.fault_recovery.recovery_timeout_s) {
        const auto reset=reset_reference_to_measured(*joints);if(!reset)return reset;
        fault_hold_mode_=FaultHoldMode::RIGID_HOLD;
    }
    auto command=build_fault_joint_cmd(*joints,fault_hold_mode_,dt);
    if(!command)return tl::make_unexpected(command.error());
    const auto safe=position_recovery?safety_.check_joint_cmd_for_position_recovery(*joints,*command,dt):
        safety_.check_joint_cmd(*joints,*command,dt);
    if(!safe)return tl::make_unexpected(make_safety_fault(safe.error()));
    const auto mapped=mapper_.to_actuator_cmd(*safe);
    if(!mapped)return tl::make_unexpected(make_mapper_fault(mapped.error()));
    const auto written=motor_bus_->write(*mapped);
    if(!written)return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_WRITE_FAILED,written.error()));
    fault_hold_cmd_=*mapped;last_joint_cmd_=*safe;has_last_joint_cmd_=true;
    fault_feedback_.joint_cmd=*safe;fault_feedback_.actuator_cmd=*mapped;
    fault_feedback_.accepted_ref_pos=safe->pos;fault_feedback_.accepted_ref_vel=safe->vel;
    fault_feedback_.command_written_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
    return {};
}

/**
 * @brief 构造故障保持 Joint 命令
 */
tl::expected<JointCtrlCmd, RobotFault> Robot::build_fault_joint_cmd(const JointState& state, FaultHoldMode mode, double dt) {
    const auto n=cfg_.joint_names.size();const bool csp=cfg_.runtime.control_path==ControlPath::CSP;
    const bool drag=mode==FaultHoldMode::DRAG_RECOVERY;
    JointCtrlCmd cmd;cmd.path=cfg_.runtime.control_path;
    cmd.pos=last_joint_cmd_.pos.size()==n?last_joint_cmd_.pos:state.pos;
    cmd.vel.assign(n,0);cmd.tor.assign(n,0);
    cmd.kp=csp?JointVector(n,0):(drag?JointVector(n,0):cfg_.ctrller.rigid_gains.kp);
    cmd.kd=csp?JointVector(n,0):(drag?cfg_.ctrller.drag_gains.kd:cfg_.ctrller.rigid_gains.kd);
    if(csp)cmd.position_speed_limit=cfg_.safety.limits.max_vel;
    if(drag)cmd.pos=csp?nominal_hold_target_:state.pos;
    const bool position_recovery=current_fault_ && current_fault_->code==RobotErr::SAFETY_FAILED && current_fault_->safety_fault.code==SafetyErr::JOINT_POS_LIMIT;
    if(cfg_.capability.admittance.observer_enabled && interaction_model_state_) {
        auto interaction=update_interaction(state,actuator_state_,JointVector(n,0),false,cmd,dt,csp&&drag,drag,position_recovery);
        if(!interaction)return tl::make_unexpected(interaction.error());
        cmd=interaction->output.corrected_cmd;
        const auto& out=interaction->output;
        fault_feedback_.observer_valid=out.state.valid;fault_feedback_.new_measurement=interaction->new_measurement;
        fault_feedback_.observer_state=interaction->observer_state;fault_feedback_.observer_timestamp_ns=interaction->observer_timestamp_ns;
        fault_feedback_.measurement_dt=interaction->measurement_dt;fault_feedback_.feedback_skew_ms=interaction->feedback_skew_ms;
        fault_feedback_.tau_ext_hat=out.tau_ext_hat;fault_feedback_.friction_compensated=out.friction_compensated;
        fault_feedback_.friction_residual_hat=out.friction_residual_hat;fault_feedback_.bias_compensated=out.bias_compensated;
        fault_feedback_.residual_raw=out.residual.residual;fault_feedback_.residual_filtered=out.residual.residual_filtered;
        fault_feedback_.delta_q=out.delta_q;fault_feedback_.delta_q_dot=out.delta_q_dot;
        fault_feedback_.interaction_state=out.state;fault_feedback_.admittance_active=csp&&drag;
    }
    if(!csp && model_feedforward_ && (drag || get_model_feedforward_mode()!=ModelFeedforwardMode::NONE)) {
        const JointVector zero(n,0);const auto g=model_feedforward_(ModelFeedforwardMode::GRAVITY,state,zero,zero,dt);
        if(!g)return tl::make_unexpected(make_model_fault(g.error()));
        if(g->size()!=n || !finite_vector(*g))return tl::make_unexpected(make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD));
        cmd.tor=*g;
    }
    const auto projected=project_command_gains(cmd,cfg_.safety.limits);
    if(!projected)return tl::make_unexpected(make_safety_fault(projected.error()));
    if(csp && drag) {
        ReferenceGovernorInput input;input.requested=cmd;input.previous=last_joint_cmd_;input.measured=state;
        input.requested_acceleration.assign(n,0);input.limits=cfg_.safety.limits;input.dt=dt;
        if(position_recovery)for(std::size_t i=0;i<n;++i) {
            const double previous=input.previous.pos[i];
            if(previous>input.limits.max_pos[i]-input.limits.pos_margin[i])input.limits.max_pos[i]=previous+input.limits.pos_margin[i];
            if(previous<input.limits.min_pos[i]+input.limits.pos_margin[i])input.limits.min_pos[i]=previous-input.limits.pos_margin[i];
        }
        const auto governed=govern_reference(input);
        if(!governed) {auto fault=make_fault(RobotErr::REFERENCE_GOVERNOR_FAILED);fault.reference_governor_err=governed.error();return tl::make_unexpected(fault);}
        cmd=governed->accepted;JointVector q(n),v(n);
        for(std::size_t i=0;i<n;++i){q[i]=cmd.pos[i]-nominal_hold_target_[i];v[i]=cmd.vel[i];}
        const auto committed=interaction_controller_.accept_reference(q,v);
        if(!committed)return tl::make_unexpected(make_interaction_fault(committed.error()));
    }
    for(std::size_t i=0;i<n;++i) {
        const double total=cmd.tor[i]+cmd.kp[i]*(cmd.pos[i]-state.pos[i])+cmd.kd[i]*(cmd.vel[i]-state.vel[i]);
        if(!std::isfinite(total) || std::abs(total)>cfg_.safety.limits.max_effort[i]+cfg_.safety.numeric_tolerance) {
            auto fault=make_fault(RobotErr::REFERENCE_GOVERNOR_FAILED);fault.reference_governor_err=ReferenceGovernorErr::INFEASIBLE;return tl::make_unexpected(fault);
        }
    }
    return cmd;
}

/**
 * @brief 判断当前锁存故障是否允许柔性恢复
 */
bool Robot::is_drag_recovery_fault_allowed() const noexcept {
    if(!current_fault_) return false;
    if(current_fault_->code == RobotErr::MOTOR_BUS_READ_FAILED || current_fault_->code == RobotErr::MOTOR_BUS_WRITE_FAILED) return false;
    if(current_fault_->code != RobotErr::SAFETY_FAILED) return false;
    // 通信、供电、执行器内部错误、非有限状态与持续状态超时禁止柔性恢复；软件只能继续尽力刷新最后合法保持命令
    switch(current_fault_->safety_fault.code) {
        case SafetyErr::CMD_TRACKING_ERROR:
        case SafetyErr::CMD_TIMEOUT:
        case SafetyErr::CMD_POS_LIMIT:
        case SafetyErr::CMD_VEL_LIMIT:
        case SafetyErr::CMD_POS_STEP_LIMIT:
        case SafetyErr::CMD_VEL_STEP_LIMIT:
        case SafetyErr::JOINT_POS_LIMIT:
        case SafetyErr::JOINT_VEL_LIMIT:
            return true;
        default:
            return false;
    }
}

/**
 * @brief 清除外部命令并把参考重置为实测位置
 */
tl::expected<void, RobotFault> Robot::reset_reference_to_measured(const JointState& state) {
    clear_calibration_disturbance();
    nominal_hold_target_=state.pos;hold_integral_torque_.assign(cfg_.joint_names.size(),0.0);
    calibration_feedforward_mode_.reset();handover_source_gains_.reset();full_command_active_=false;
accepted_nominal_cmd_.pos=state.pos;accepted_nominal_cmd_.vel.assign(cfg_.joint_names.size(),0.0);
    park_rigid_ramp_started_.reset();
    clear_calibration_hold_gains();
    clear_calibration_tracking_gains();
    ctrller_.reset();
    interaction_controller_.reset();
    interaction_state_={};
    const auto initialized = ctrller_.initialize(state);
    if(!initialized) return tl::make_unexpected(make_ctrller_fault(initialized.error()));

    const auto mode_result = ctrller_.set_impedance_mode(JointImpedanceMode::RIGID, state);
    if(!mode_result) return tl::make_unexpected(make_ctrller_fault(mode_result.error()));

    const auto history = safety_.reset_cmd_history(state);
    if(!history) return tl::make_unexpected(make_safety_fault(history.error()));

    joint_acc_.assign(cfg_.joint_names.size(), 0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(), 0.0);
    model_feedforward_cache_.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.pos = state.pos;
    last_joint_cmd_.vel.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.tor.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kp.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kd.assign(cfg_.joint_names.size(), 0.0);
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;
    return {};
}

/**
 * @brief 直接失能硬件，忽略错误
 */
void Robot::disable_noexcept() noexcept {
    if(!motor_bus_) return;
    (void)motor_bus_->deactivate();
}

/**
 * @brief 清空运行时缓存状态
 */
void Robot::apply_gain_overrides(JointCtrlCmd& cmd, TimePoint now) {
    gain_source_=full_command_active_?GainSource::FULL_COMMAND:GainSource::CONFIGURED;
    if(full_command_active_) return;
    const auto profile=get_gain_profile();
    const JointImpedanceGains* gains=nullptr;
    switch(profile) {
        case JointGainProfile::SERVO_TRACKING:gains=&cfg_.ctrller.rigid_gains;break;
        case JointGainProfile::DRAG:gains=&cfg_.ctrller.drag_gains;break;
        case JointGainProfile::COMPLIANT:gains=&cfg_.ctrller.compliant_gains;break;
        case JointGainProfile::RIGID:gains=&cfg_.ctrller.rigid_gains;break;
    }
    cmd.kp=gains->kp;cmd.kd=gains->kd;
    // Calibration hold gains are independent of normal user Kp/Kd.  Blend
    // smoothly on entry, before the standard joint safety checker
    if(calibration_hold_gains_ && ctrller_.get_impedance_mode() == JointImpedanceMode::RIGID) {
        gain_source_=GainSource::CALIBRATION_OVERRIDE;
        const double elapsed = std::chrono::duration<double>(now-calibration_hold_started_).count();
        const double fraction = std::clamp(elapsed / 0.30, 0.0, 1.0);
        const double smooth = fraction*fraction*(3.0-2.0*fraction);
        for(std::size_t i=0;i<cmd.kp.size();++i) {
            const auto& origin=calibration_hold_source_ ? *calibration_hold_source_ : cfg_.ctrller.rigid_gains;
            cmd.kp[i] = origin.kp[i]+smooth*(calibration_hold_gains_->kp[i]-origin.kp[i]);
            cmd.kd[i] = origin.kd[i]+smooth*(calibration_hold_gains_->kd[i]-origin.kd[i]);
        }
    }


    if(calibration_tracking_gains_ && ctrller_.get_impedance_mode() == calibration_tracking_mode_) {
        gain_source_=GainSource::CALIBRATION_OVERRIDE;
        const double elapsed=std::chrono::duration<double>(now-calibration_tracking_started_).count();
        const double fraction=std::clamp(elapsed/0.12,0.0,1.0);
        const double smooth=fraction*fraction*(3.0-2.0*fraction);
        const auto& source=calibration_tracking_source_ ? *calibration_tracking_source_ :
            (calibration_tracking_mode_==JointImpedanceMode::DRAG ? cfg_.ctrller.drag_gains :
             calibration_tracking_mode_==JointImpedanceMode::COMPLIANT ? cfg_.ctrller.compliant_gains : cfg_.ctrller.rigid_gains);
        for(std::size_t i=0;i<cmd.kp.size();++i) {
            cmd.kp[i]=source.kp[i]+smooth*(calibration_tracking_gains_->kp[i]-source.kp[i]);
            cmd.kd[i]=source.kd[i]+smooth*(calibration_tracking_gains_->kd[i]-source.kd[i]);
        }
    }

    // Park is the only user of this temporary handover. The first RIGID
    // command has compliant gains, then Kp/Kd increase continuously while its
    // reference travels from actual measured q; safety limits still apply
    if(park_rigid_ramp_started_ &&
       ctrller_.get_impedance_mode()==JointImpedanceMode::RIGID) {
        gain_source_=GainSource::PARK_OVERRIDE;
        const double elapsed=std::chrono::duration<double>(now-*park_rigid_ramp_started_).count();
        const double fraction=park_rigid_gain_fraction(elapsed,park_rigid_ramp_duration_s_);
        for(std::size_t i=0;i<cmd.kp.size();++i) {
            const auto& soft=cfg_.ctrller.compliant_gains;
            cmd.kp[i]=soft.kp[i]+fraction*(cmd.kp[i]-soft.kp[i]);
            cmd.kd[i]=soft.kd[i]+fraction*(cmd.kd[i]-soft.kd[i]);
        }
        if(fraction>=1.0) park_rigid_ramp_started_.reset();
    }
    if(handover_source_gains_ && handover_source_gains_->kp.size()==cmd.kp.size()) {
        const double x=std::clamp(std::chrono::duration<double>(now-handover_started_).count()/.3,0.0,1.0),blend=x*x*(3-2*x);
        for(std::size_t i=0;i<cmd.kp.size();++i) {
            cmd.kp[i]=handover_source_gains_->kp[i]+blend*(cmd.kp[i]-handover_source_gains_->kp[i]);
            cmd.kd[i]=handover_source_gains_->kd[i]+blend*(cmd.kd[i]-handover_source_gains_->kd[i]);
        }
        if(x<1)gain_source_=GainSource::TRANSITION;else handover_source_gains_.reset();
    }

}

void Robot::clear_runtime_state() noexcept {
    clear_calibration_disturbance();
    handover_source_gains_.reset();calibration_feedforward_mode_.reset();full_command_active_=false;
    nominal_hold_target_.clear();hold_integral_torque_.clear();handover_source_cmd_={};
    park_rigid_ramp_started_.reset();
    interaction_controller_.reset();
    interaction_state_={};
    joint_state_ = JointState{};
    joint_acc_.clear();
    joint_ref_acc_.clear();
    model_feedforward_cache_.clear();
    last_joint_cmd_ = JointCtrlCmd{};
    actuator_state_ = ActuatorState{};
    has_state_ = false;
    feedback_aligner_.reset();observer_sample_ns_=0;

    accepted_nominal_cmd_=JointCtrlCmd{};
    last_nominal_velocity_.assign(cfg_.joint_names.size(),0.0);
    has_completed_cycle_ = false;
    dt_overrun_watchdog_.reset();
    has_external_cmd_ = false;
    has_last_joint_cmd_ = false;
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorCtrlCmd{};
    last_cycle_time_ = TimePoint{};
    last_state_time_ = TimePoint{};
    last_cmd_time_ = TimePoint{};
}

/**
 * @brief 计算两个时间点之间的秒数差
 */
double Robot::seconds_between(TimePoint newer, TimePoint older) const noexcept {
    return std::chrono::duration<double>(newer - older).count();
}

} // namespace serial_arm
