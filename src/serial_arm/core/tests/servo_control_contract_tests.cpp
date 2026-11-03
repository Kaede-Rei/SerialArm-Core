#include "serial_arm/robot.hpp"
#include "serial_arm/runtime/control_runtime.hpp"
#include "serial_arm/dynamics/calibration_replay_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace serial_arm;
namespace {
void check(bool value,const char* message) { if(!value)throw std::runtime_error(message); }
void near(double value,double expected,double tolerance,const char* message) {
    if(!std::isfinite(value)||std::abs(value-expected)>tolerance)throw std::runtime_error(message);
}
constexpr double step_s=.005;

void endpoint_reference_contract() {
    for(double target:{1.0,.1,-.1,1e-6,-1e-6}) {
        JointVector q{0},v{0};bool complete=false;
        for(int iteration=0;iteration<10000&&!complete;++iteration) {
            const double previous_q=q[0],previous_v=v[0];
            complete=advance_joint_reference(q,v,{target},{1},{2},1,step_s);
            near((q[0]-previous_q)/step_s,v[0],1e-10,"public trajectory endpoint q and dq disagree");
            check(std::abs(v[0]-previous_v)<=2*step_s+1e-12,"public trajectory endpoint exceeds acceleration");
            check(std::abs(v[0])<=1+1e-12,"public trajectory exceeds velocity");
        }
        check(complete,"public trajectory does not finish");
        near(q[0],target,1e-12,"public trajectory misses its endpoint");
        near(v[0],0,1e-12,"public trajectory reports completion with nonzero velocity");
    }
    for(bool lag_stopped:{false,true})for(double initial_velocity:{.1,-.1}) {
        JointVector q{0},v{initial_velocity},measured{-.3};
        const int braking_steps=static_cast<int>(std::ceil(std::abs(initial_velocity)/(2*step_s)))+1;
        for(int iteration=0;iteration<braking_steps;++iteration) {
            const double previous_q=q[0],previous_v=v[0];
            check(!advance_joint_reference(q,v,{1},{1},{2},lag_stopped?1:0,step_s,
                lag_stopped?&measured:nullptr),"paused reference should not complete a distant goal");
            near((q[0]-previous_q)/step_s,v[0],1e-10,"paused braking loses q/dq consistency");
            check(std::abs(v[0]-previous_v)<=2*step_s+1e-12,"paused braking exceeds acceleration");
        }
        near(v[0],0,0,"paused reference does not stop within its finite braking bound");
        const double stopped_position=q[0];
        advance_joint_reference(q,v,{1},{1},{2},lag_stopped?1:0,step_s,lag_stopped?&measured:nullptr);
        near(q[0],stopped_position,0,"paused reference drifts after reaching rest");
    }
}

RobotCfg fixture(ReferenceBehavior behavior,bool observer_enabled=true) {
    HardwareCapabilities caps;
    for(int i=1;i<=6;++i)caps.push_back({"motor"+std::to_string(i),-100,100,100,100,5000,50,"joint"+std::to_string(i)});
    auto loaded=load_robot_cfg(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/core/gray.yaml",caps);
    check(bool(loaded),"servo contract fixture load");auto cfg=*loaded;const auto n=cfg.joint_names.size();
    cfg.runtime.write_enabled=true;cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    cfg.safety_limit_source.reset();
    cfg.safety.limits={std::vector<std::uint8_t>(n,1),JointVector(n,-5),JointVector(n,5),JointVector(n,5),
        JointVector(n,100),JointVector(n,100),JointVector(n,500),JointVector(n,50),JointVector(n,0)};
    cfg.safety.fault_recovery.compliant_recovery.max_vel.assign(n,2);
    cfg.safety.state_timeout_s=20;cfg.safety.max_dt_s=.05;cfg.safety.cmd_timeout_s=20;
    cfg.safety.require_continuous_cmd=false;
    cfg.ctrller.rigid_hold_gains={JointVector(n,20),JointVector(n,4)};
    cfg.ctrller.rigid_tracking_gains={JointVector(n,30),JointVector(n,5)};
    cfg.ctrller.compliant_hold_gains={JointVector(n,6),JointVector(n,1)};
    cfg.ctrller.compliant_tracking_gains={JointVector(n,8),JointVector(n,2)};
    cfg.ctrller.compliant_drag_gains={JointVector(n,0),JointVector(n,.3)};
    cfg.ctrller.servo_tracking_gains={JointVector(n,13),JointVector(n,3)};
    auto& adm=cfg.capability.admittance;adm.enabled=observer_enabled;adm.reference_behavior=behavior;
    adm.observer.mode=AdmittanceObserverMode::MOMENTUM;adm.observer.momentum_gain.assign(n,50);
    adm.calibration.torque_bias.assign(n,0);adm.calibration.torque_threshold.assign(n,0);adm.calibration.friction.enabled=false;
    adm.controller.mass.assign(n,1);adm.controller.damping.assign(n,4);adm.controller.stiffness.assign(n,10);
    adm.controller.max_delta_q.assign(n,1);adm.controller.max_delta_q_dot.assign(n,2);
    return cfg;
}

// A local unit-inertia actuator executes the actual MIT packet. No hardware or
// wall-clock sleeps are used; receive timestamps progress on a separate clock.
struct PlantBus final:MotorBus {
    HardwareCapabilities caps;
    ActuatorState state;
    ActuatorCtrlCmd command;
    JointVector external;
    std::uint64_t stamp;
    bool timing_valid{true};
    bool fail_next_read{false};
    int writes{0};
    explicit PlantBus(const RobotCfg& cfg) {
        const auto n=cfg.joint_names.size();
        for(const auto& name:cfg.joint_names)caps.push_back({name,-100,100,100,100,5000,50,name});
        state.pos.assign(n,0);state.vel.assign(n,0);state.tor.assign(n,0);
        state.online.assign(n,1);state.enabled.assign(n,1);state.err_code.assign(n,0);
        command={state.pos,state.vel,state.tor,state.tor,state.tor};external.assign(n,0);
        stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count()-15000000000ULL;
    }
    tl::expected<void,MotorBusErr> configure(const std::string&)override{return {};}
    tl::expected<void,MotorBusErr> connect()override{return {};}
    tl::expected<void,MotorBusErr> activate()override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override {
        if(fail_next_read) {fail_next_read=false;return tl::make_unexpected(MotorBusErr::READ_FAILED);}
        for(std::size_t i=0;i<caps.size();++i) {
            const double torque=command.tor[i]+command.kp[i]*(command.pos[i]-state.pos[i])+command.kd[i]*(command.vel[i]-state.vel[i]);
            state.vel[i]+=step_s*(torque+external[i]);state.pos[i]+=step_s*state.vel[i];state.tor[i]=torque;
        }
        stamp+=5000000;
        state.feedback_received_ns.assign(caps.size(),stamp);
        state.feedback_timing_valid.assign(caps.size(),timing_valid?1:0);
        return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& value)override{++writes;command=value;return {};}
    tl::expected<void,MotorBusErr> stop()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{return {};}
    tl::expected<void,MotorBusErr> recover()override{return {};}
    void cleanup()noexcept override{}
    const HardwareCapabilities& capabilities()const noexcept override{return caps;}
    std::size_t size()const noexcept override{return caps.size();}
};
struct Rig {
    RobotCfg cfg;
    PlantBus* bus;
    Robot robot;
    ModelFeedforwardMode last_feed_mode{ModelFeedforwardMode::NONE};
    Robot::TimePoint clock;
    explicit Rig(RobotCfg configuration):cfg(std::move(configuration)) {
        const auto valid=validate_robot_core_cfg(cfg);
        if(!valid)throw std::runtime_error("servo rig configuration: "+valid.error().message);
        auto hardware=std::make_unique<PlantBus>(cfg);bus=hardware.get();const auto n=cfg.joint_names.size();
        auto ff=[this,n](ModelFeedforwardMode mode,const JointState&,const JointVector&,const JointVector&,double) {
            last_feed_mode=mode;
            return tl::expected<JointVector,ModelFeedforwardErr>(JointVector(n,0));
        };
        auto model=[n](const JointState&,double) {
            MeasuredDynamicsSnapshot value;value.gravity.assign(n,0);value.coriolis.assign(n,0);
            value.mass_matrix.assign(n,JointVector(n,0));for(std::size_t i=0;i<n;++i)value.mass_matrix[i][i]=1;
            return tl::expected<InteractionModelState,ModelFeedforwardErr>(std::move(value));
        };
        check(bool(robot.configure(cfg,std::move(hardware),ff,model)),"servo rig configure");
        check(bool(robot.activate()),"servo rig activate");clock=Robot::Clock::now();
    }
    RobotCycleOutput step(int count=1) {
        RobotCycleOutput last;
        for(int i=0;i<count;++i) {
            clock+=std::chrono::milliseconds(5);
            auto value=robot.cycle(clock);
            if(!value)throw std::runtime_error("servo rig cycle fault: "+std::to_string(static_cast<int>(value.error().code)));
            last=std::move(*value);
        }
        return last;
    }
    void mode(JointImpedanceMode value,HoldTargetPolicy policy=HoldTargetPolicy::PRESERVE_NOMINAL) {
        check(bool(robot.set_impedance_mode(value,clock,policy)),"servo rig mode switch");
    }
};

void execution_profile_contract() {
    Rig native(fixture(ReferenceBehavior::NATIVE_IMPEDANCE));
    auto out=native.step();near(out.effective_kp[0],20,1e-10,"native HOLD loses rigid HOLD gains");
    native.mode(JointImpedanceMode::RIGID_TRACKING);out=native.step(80);
    near(out.effective_kp[0],30,1e-10,"native TRACK loses distinct rigid TRACK gains");
    native.mode(JointImpedanceMode::COMPLIANT_HOLD);out=native.step(80);
    near(out.effective_kp[0],6,1e-10,"native HOLD loses compliant HOLD gains");
    native.mode(JointImpedanceMode::COMPLIANT_TRACKING);out=native.step(80);
    near(out.effective_kp[0],8,1e-10,"native TRACK loses distinct compliant TRACK gains");
    check(bool(native.robot.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING)),"native explicit servo admission");
    out=native.step(80);check(out.gain_profile==JointGainProfile::SERVO_TRACKING,"native explicit servo is not the executed profile");
    near(out.effective_kp[0],13,1e-10,"native explicit servo uses another gain bank");
    native.mode(JointImpedanceMode::RIGID_HOLD);out=native.step(80);
    check(out.gain_profile==JointGainProfile::RIGID,"mode change retained a stale profile override");
    near(out.effective_kp[0],20,1e-10,"mode change did not restore configured HOLD gain");

    Rig model(fixture(ReferenceBehavior::MODEL_ADMITTANCE));out=model.step();
    check(out.gain_profile==JointGainProfile::SERVO_TRACKING,"MODEL HOLD does not select servo");
    near(out.effective_kp[0],13,1e-10,"MODEL HOLD executes rigid gains instead of servo");
    model.mode(JointImpedanceMode::COMPLIANT_TRACKING);out=model.step(80);
    near(out.effective_kp[0],13,1e-10,"MODEL TRACK gain source differs from MODEL HOLD");
}

void target_and_pause_contract() {
    Rig rig(fixture(ReferenceBehavior::MODEL_ADMITTANCE));rig.bus->external[0]=.5;
    auto before=rig.step(200);check(before.delta_q[0]>.005,"force did not produce a measurable virtual displacement");
    const auto original=rig.robot.get_hold_target();
    const auto previous_delta=rig.robot.get_interaction_state().delta_q;
    check(!rig.robot.set_cmd(JointPosCmd{JointVector(rig.cfg.joint_names.size(),.2)},rig.clock),"HOLD accepted a tracking-only reference");
    near(rig.robot.get_interaction_state().delta_q[0],previous_delta[0],1e-12,"rejected reference changed virtual state");
    rig.mode(JointImpedanceMode::COMPLIANT_HOLD);
    auto changed=rig.step();near(changed.hold_target_pos[0],original[0],1e-12,"mode change baked force displacement into HOLD target");
    near(changed.nominal_ref_pos[0],original[0],1e-10,"mode change shifted the nominal reference");
    const double paused_delta=changed.delta_q[0];rig.robot.set_admittance_suspended(true);
    auto paused=rig.step(20);
    near(paused.hold_target_pos[0],original[0],1e-12,"pause shifted the original HOLD target");
    near(paused.nominal_ref_pos[0],original[0],1e-10,"pause baked displacement into nominal");
    near(paused.delta_q[0],paused_delta,1e-8,"pause failed to retain separate virtual displacement");
    rig.bus->external[0]=0;rig.robot.set_admittance_suspended(false);
    auto resumed=rig.step(1000);
    near(resumed.hold_target_pos[0],original[0],1e-12,"resume changed the original HOLD target");
    check(std::abs(resumed.delta_q[0])<.002,"released virtual spring did not return toward its original target");
    const double measured=rig.robot.get_joint_state().pos[0];
    rig.mode(JointImpedanceMode::RIGID_HOLD,HoldTargetPolicy::CAPTURE_MEASURED);
    near(rig.robot.get_hold_target()[0],measured,1e-12,"capture-current policy did not adopt measured position");
}

HoldCompensationCfg compensation(std::size_t n) {
    HoldCompensationCfg cfg;cfg.enabled=true;cfg.integral_gain.assign(n,2);
    cfg.max_torque.assign(n,.1);cfg.max_rate.assign(n,.2);cfg.contact_threshold.assign(n,1);
    cfg.velocity_threshold=.5;cfg.error_threshold=.2;return cfg;
}
void bounded_hold_learning_contract() {
    auto cfg=fixture(ReferenceBehavior::MODEL_ADMITTANCE,false);cfg.hold_compensation=compensation(cfg.joint_names.size());
    Rig rig(cfg);rig.bus->external[0]=-.2;
    auto learned=rig.step(600);
    check(!rig.robot.get_admittance_cfg().enabled,"HOLD shadow changes the persistent interaction switch");
    check(learned.observer_valid,"enabled HOLD compensation lacks a real shadow observation");
    check(!learned.admittance_active && learned.reference_behavior==ReferenceBehavior::NATIVE_IMPEDANCE,
        "HOLD shadow activates disabled MODEL control");
    near(learned.delta_q[0],0,1e-12,"HOLD shadow generates an MDK position offset");
    check(!learned.hold_integrator_frozen,"quiet low-speed HOLD never admitted learning");
    check(learned.hold_integral_torque[0]>.005,"HOLD integral did not oppose a persistent positioning error");
    check(learned.hold_integral_torque[0]<=.1+1e-10,"HOLD integral exceeded its configured torque budget");
    const double accepted=learned.hold_integral_torque[0];
    rig.bus->timing_valid=false;auto invalid=rig.step(20);
    check(invalid.hold_integrator_frozen,"invalid observation still admits HOLD learning");
    near(invalid.hold_integral_torque[0],accepted,1e-10,"invalid observation changes learned HOLD torque");
    rig.bus->timing_valid=true;rig.bus->external[0]=-3;
    auto contact=rig.step(100);
    check(std::abs(contact.tau_ext_hat[0])>1,"contact fixture did not cross the configured observation envelope");
    check(contact.hold_integrator_frozen,"detected contact still admits HOLD learning");
    const double frozen=contact.hold_integral_torque[0];contact=rig.step(10);
    near(contact.hold_integral_torque[0],frozen,1e-10,"detected contact winds up the HOLD integral");
    auto limited=compensation(cfg.joint_names.size());limited.max_torque.assign(cfg.joint_names.size(),.005);
    check(bool(rig.robot.set_hold_compensation_cfg(limited)),"bounded HOLD candidate admission");
    rig.bus->external[0]=-.2;
    auto bounded=rig.step(600);
    check(std::abs(bounded.hold_integral_torque[0])<=.005+1e-10,"persistent error winds up beyond HOLD budget");
    limited.enabled=false;check(bool(rig.robot.set_hold_compensation_cfg(limited)),"disable HOLD compensation");
    auto disabled=rig.step(80);near(disabled.hold_integral_torque[0],0,1e-12,"disabled HOLD compensation leaves a hidden torque bias");
    check(!disabled.observer_valid,"disabling the last shadow consumer keeps the observer active");
}

void fault_configuration_restore_contract() {
    Rig rig(fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false));
    const auto original_gains=rig.robot.get_config().ctrller.servo_tracking_gains;
    const auto original_hold=rig.robot.get_hold_compensation_cfg();
    const auto count=rig.cfg.joint_names.size();
    check(bool(rig.robot.set_servo_tracking_gains({JointVector(count,40),JointVector(count,4)})),"temporary servo setup");
    check(bool(rig.robot.set_hold_compensation_cfg(compensation(count))),"temporary HOLD setup");
    auto trial=rig.step(80);
    check(trial.observer_valid && !trial.admittance_active,"runtime HOLD candidate lacks native shadow observation");
    rig.bus->fail_next_read=true;rig.clock+=std::chrono::milliseconds(5);
    auto fault=rig.robot.cycle(rig.clock);
    check(!fault && rig.robot.get_state()==RobotState::FAULT,"fixture failed to enter a real read fault");
    const auto fault_code=rig.robot.get_last_fault()->code;
    const auto writes=rig.bus->writes;
    check(bool(rig.robot.set_servo_tracking_gains(original_gains)),"FAULT prevents servo candidate rollback");
    check(bool(rig.robot.set_hold_compensation_cfg(original_hold)),"FAULT prevents HOLD candidate rollback");
    check(rig.bus->writes==writes,"FAULT configuration rollback sends an actuator command");
    check(rig.robot.get_state()==RobotState::FAULT && rig.robot.get_last_fault()->code==fault_code,
        "configuration rollback clears or replaces the latched fault");
    check(rig.robot.get_config().ctrller.servo_tracking_gains.kp==original_gains.kp &&
        rig.robot.get_config().ctrller.servo_tracking_gains.kd==original_gains.kd,"FAULT retains trial servo values");
    check(!rig.robot.get_hold_compensation_cfg().enabled,"FAULT retains trial HOLD learning");
    check(!rig.robot.set_servo_tracking_gains({JointVector(count,501),JointVector(count,4)}),
        "FAULT rollback bypasses configured gain limits");
    check(rig.robot.get_config().ctrller.servo_tracking_gains.kp==original_gains.kp,
        "rejected FAULT configuration mutates restored gains");
    check(!rig.robot.activate(),"configuration rollback resumes execution from FAULT");
    check(bool(rig.robot.force_deactivate()) && bool(rig.robot.activate()),"explicit restart after rollback");
    rig.clock=Robot::Clock::now();
    check(bool(rig.robot.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING)),"restored servo selection");
    auto restored=rig.step(80);
    near(restored.effective_kp[0],original_gains.kp[0],1e-10,"restart executes leftover trial servo gains");
    near(restored.hold_integral_torque[0],0,1e-12,"restart executes leftover trial HOLD compensation");
}

void hold_shadow_requires_model_contract() {
    auto cfg=fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false);
    cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;
    Robot robot;
    check(bool(robot.configure(cfg,std::make_unique<PlantBus>(cfg))),"disabled observer without a model must remain supported");
    check(!robot.set_hold_compensation_cfg(compensation(cfg.joint_names.size())),"HOLD accepts shadow without a model callback");
    check(!robot.get_hold_compensation_cfg().enabled,"rejected shadow request enables HOLD compensation");
    cfg.hold_compensation=compensation(cfg.joint_names.size());Robot missing;
    check(!missing.configure(cfg,std::make_unique<PlantBus>(cfg)),"startup HOLD accepts a missing model callback");
}

void hold_compensation_purpose_contract() {
    auto cfg=fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false);cfg.hold_compensation=compensation(cfg.joint_names.size());
    Rig rig(cfg);rig.bus->external[0]=-.2;auto rigid=rig.step(600);
    check(!rigid.hold_integrator_frozen && rigid.hold_integral_torque[0]>.005,"rigid positioning HOLD does not learn its validated trim");
    rig.mode(JointImpedanceMode::COMPLIANT_HOLD);auto soft=rig.step();
    check(soft.hold_integrator_frozen,"compliant HOLD consumes rigid positioning trim");
    near(soft.hold_integral_torque[0],0,0,"compliant HOLD retains rigid integral torque");
    rig.mode(JointImpedanceMode::RIGID_HOLD);
    check(bool(rig.robot.set_execution_gain_profile(JointGainProfile::SERVO_TRACKING)),"Native SERVO HOLD admission");
    auto servo=rig.step(80);
    check(servo.hold_integrator_frozen,"Native SERVO HOLD consumes rigid positioning trim");
    near(servo.hold_integral_torque[0],0,0,"Native SERVO HOLD retains rigid integral torque");
    check(bool(rig.robot.set_execution_gain_profile(JointGainProfile::RIGID)),"restore rigid purpose");
    auto reset=rig.step();
    check(std::abs(reset.hold_integral_torque[0])<.001,"restored rigid HOLD resumes an old learned trim");
    auto relearned=rig.step(600);
    check(!relearned.hold_integrator_frozen && relearned.hold_integral_torque[0]>.005,"restored rigid HOLD does not learn from a clean state");
}
void calibration_feedforward_contract() {
    auto cfg=fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false);
    cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::FULL_INVERSE_DYNAMICS;
    Rig rig(cfg);rig.step(2);
    check(rig.last_feed_mode==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS,"fixture does not execute configured full feedforward");
    check(bool(rig.robot.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE)),"teaching native strategy");
    check(bool(rig.robot.set_calibration_model_feedforward_mode(ModelFeedforwardMode::GRAVITY)),"temporary teaching gravity selection");
    rig.step(80);
    check(rig.last_feed_mode==ModelFeedforwardMode::GRAVITY,"temporary teaching selection does not affect executed feedforward");
    check(rig.robot.get_config().runtime.model_feedforward_mode==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS,"teaching changes persisted feedforward");
    rig.robot.clear_calibration_model_feedforward_mode();rig.robot.clear_calibration_reference_behavior();rig.step(80);
    check(rig.last_feed_mode==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS,"teaching exit does not restore original full feedforward");
    check(bool(rig.robot.set_calibration_model_feedforward_mode(ModelFeedforwardMode::GRAVITY)),"temporary teaching selection before fault");
    rig.bus->fail_next_read=true;rig.clock+=std::chrono::milliseconds(5);auto fault=rig.robot.cycle(rig.clock);
    check(!fault && rig.robot.get_state()==RobotState::FAULT,"teaching fixture fault");
    const auto writes=rig.bus->writes;rig.robot.clear_calibration_model_feedforward_mode();rig.robot.clear_calibration_reference_behavior();
    check(rig.robot.get_model_feedforward_mode()==ModelFeedforwardMode::FULL_INVERSE_DYNAMICS,"FAULT cleanup retains teaching feedforward");
    check(rig.robot.get_state()==RobotState::FAULT && rig.bus->writes==writes,"teaching cleanup moves or clears a faulted robot");
}
void calibration_hold_trim_contract() {
    Rig rig(fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false));
    check(bool(rig.robot.set_calibration_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE)),"trim probe native strategy");
    check(bool(rig.robot.set_hold_compensation_cfg(compensation(rig.cfg.joint_names.size()))),"trim probe compensation");
    rig.robot.set_admittance_suspended(false);rig.bus->external[0]=-.2;auto out=rig.step(600);
    check(out.observer_valid && !out.admittance_active,"native trim probe must observe without applying MDK");
    check(!out.hold_integrator_frozen && out.hold_integral_torque[0]>.005,"native trim probe cannot collect integral samples");
}
void admittance_replay_layers_contract() {
    using namespace serial_arm::calibration_replay_supervisor;
    auto okay=evaluate_reference_layers({0},{0},{.1},{.1},{0},{1});
    check(okay.valid && !okay.hard_violation && okay.state==State::FOLLOWING,"legitimate MDK displacement is treated as path tracking failure");
    auto lag=evaluate_reference_layers({.07},{0},{.1},{.1},{0},{1});
    check(lag.valid && lag.nominal_lag && lag.state==State::PAUSE_FOR_FEEDBACK,"saturated nominal path still advances replay time");
    auto bad_servo=evaluate_reference_layers({0},{0},{.1},{.21},{0},{1});
    check(bad_servo.valid && bad_servo.hard_violation && !bad_servo.nominal_lag,"accepted-reference following hard limit is lost");
    auto bad_path=evaluate_reference_layers({.12},{0},{.1},{.1},{0},{1});
    check(bad_path.valid && bad_path.hard_violation && bad_path.nominal_lag,"nominal path hard bound is lost");
    check(!evaluate_reference_layers({0},{},{.1},{.1},{0},{1}).valid,"missing accepted/nominal authority is silently accepted");
}
void calibration_strategy_contract() {
    Rig rig(fixture(ReferenceBehavior::NATIVE_IMPEDANCE,false));
    const auto original=rig.robot.get_admittance_cfg();
    auto native=rig.step();check(!native.admittance_active,"disabled capability executes MDK");
    near(native.hold_integral_torque[0],0,1e-12,"default disabled HOLD compensation changes actuation");
    check(bool(rig.robot.set_calibration_reference_behavior(ReferenceBehavior::MODEL_ADMITTANCE)),"temporary MODEL trial on a disabled profile");
    auto trial=rig.step(80);check(trial.admittance_active&&trial.observer_valid,"temporary MODEL trial is not the actual executed controller");
    check(rig.robot.get_admittance_cfg().enabled==original.enabled,"temporary MODEL trial modifies persistent capability selection");
    check(rig.robot.get_admittance_cfg().reference_behavior==original.reference_behavior,"temporary MODEL trial modifies persistent strategy");
    near(trial.effective_kp[0],13,1e-10,"temporary MODEL trial executes a different servo profile");
    rig.robot.clear_calibration_reference_behavior();auto restored=rig.step(80);
    check(!restored.admittance_active,"exiting temporary MODEL trial does not restore native execution");
    check(restored.gain_profile==JointGainProfile::RIGID,"exiting temporary MODEL trial retains the temporary gain profile");
    near(restored.effective_kp[0],20,1e-10,"exiting temporary MODEL trial fails to restore configured HOLD gains");
}
}
int main() {
    try {
        endpoint_reference_contract();execution_profile_contract();target_and_pause_contract();
        bounded_hold_learning_contract();fault_configuration_restore_contract();hold_shadow_requires_model_contract();calibration_strategy_contract();
        calibration_feedforward_contract();calibration_hold_trim_contract();admittance_replay_layers_contract();hold_compensation_purpose_contract();
        std::cout<<"servo control contracts passed\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
