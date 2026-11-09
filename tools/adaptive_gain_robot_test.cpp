// Runs Robot against an in-memory MotorBus. No hardware backend is loaded and
// every connect/activate/write below is a method on this memory-only fixture.
#include "serial_arm/robot.hpp"
#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include "serial_arm/runtime/control_runtime.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace serial_arm;
void check(bool v,const char* message){if(!v)throw std::runtime_error(message);}
class MemoryBus final:public MotorBus {
public:
    HardwareCapabilities caps{{"a",-2,2,2,1,100,5,"a"},{"b",-2,2,2,1,100,5,"b"}};
    ActuatorState state{{0,0},{0,0},{0,0},{1,1},{1,1},{0,0},{},{1,1}};
    ActuatorCtrlCmd last;std::size_t writes=0,stops=0;
    tl::expected<void,MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,MotorBusErr> connect()override{return {};}
    tl::expected<void,MotorBusErr> activate()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{return {};}
    tl::expected<void,MotorBusErr> stop()override{++stops;return {};}
    tl::expected<void,MotorBusErr> recover()override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override {
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
        state.feedback_received_ns.assign(2,ns);return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& cmd)override{last=cmd;++writes;return {};}
    const HardwareCapabilities& capabilities()const noexcept override{return caps;}
    std::size_t size()const noexcept override{return 2;}
    void cleanup()noexcept override{}
};
RobotCfg configuration(bool integral=false) {
    RobotCfg c;c.joint_names={"a","b"};c.runtime.write_enabled=true;c.runtime.ctrl_frequency_hz=200;
    c.shutdown.park_before_disable=false;c.shutdown.park_pos={0,0};
    c.ctrller.joints_count=2;
    const JointImpedanceGains g{{2,2},{.2,.2}};
    c.ctrller.rigid_hold_gains=c.ctrller.rigid_tracking_gains=c.ctrller.compliant_hold_gains=
        c.ctrller.compliant_tracking_gains=c.ctrller.servo_tracking_gains=g;
    c.ctrller.compliant_drag_gains={{0,0},{.2,.2}};
    c.mapper={2,{1,1},{1,1},{1,1},{0,0},{0,0}};
    c.safety.joints_count=2;c.safety.max_dt_s=.2;c.safety.state_timeout_s=.5;
    c.safety.limits={{1,1},{-2,-2},{2,2},{2,2},{100,100},{1,1},{100,100},{5,5},{.01,.01},{},{}};
    c.safety.fault_recovery.compliant_recovery={{.1,.1},{.1,.1},{.1,.1},.5};
    c.dynamics.urdf_path="memory-only-not-loaded.urdf";c.dynamics.joint_names=c.joint_names;c.dynamics.gravity_scale={1,1};
    c.capability.admittance.reference_behavior=ReferenceBehavior::NATIVE_IMPEDANCE;
    if(integral) {
        c.hold_compensation={true,{1,1},{.1,.1},{1,1},{.2,.2},.02,.03};
        auto& a=c.capability.admittance;a.joint_enabled={1,1};a.observer.momentum_gain={5,5};
        a.calibration.torque_bias={0,0};a.calibration.torque_threshold={.01,.01};
        a.controller.mass={1,1};a.controller.damping={1,1};a.controller.stiffness={1,1};
        a.controller.max_delta_q={.1,.1};a.controller.max_delta_q_dot={.2,.2};
    }
    return c;
}
struct Fixture {
    Robot robot;MemoryBus* bus{nullptr};double feedforward{0};
    explicit Fixture(bool integral=false,double ff=0):feedforward(ff) {
        auto cfg=configuration(integral);cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
        const auto valid=validate_robot_core_cfg(cfg);
        if(!valid)throw std::runtime_error("in-memory config invalid: "+valid.error().message);
        auto memory=std::make_unique<MemoryBus>();bus=memory.get();
        const auto configured=robot.configure(cfg,std::move(memory),
            [this](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{feedforward,feedforward};},
            [](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{0,0},{0,0},{{1,0},{0,1}}};});
        if(!configured)throw std::runtime_error("in-memory Robot configure failed, code="+std::to_string(static_cast<int>(configured.error().code))+", safety="+std::to_string(static_cast<int>(configured.error().safety_fault.code))+", controller="+std::to_string(static_cast<int>(configured.error().ctrller_err)));
        check(bool(robot.activate()),"in-memory Robot activation failed");
    }
    RobotCycleOutput cycle() {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto result=robot.cycle();
        if(!result)throw std::runtime_error("in-memory cycle failed with RobotErr "+std::to_string(static_cast<int>(result.error().code)));
        return *result;
    }
};
int main() {
    try {
        // Analysis HOLD must survive a stationary interval, while an ordinary
        // abandoned tracking command still trips the unchanged command timeout.
        Fixture assessment;
        ControlRuntime runtime;
        check(bool(runtime.set_impedance_mode(assessment.robot,JointImpedanceMode::RIGID_TRACKING)),"assessment tracking setup failed");
        check(bool(runtime.submit_reference(assessment.robot,JointPosVelCmd{{.002,0},{0,0}},Robot::Clock::now(),MotionSource::CALIBRATION)),"assessment input rejected");
        check(runtime.step(assessment.robot).cycle.has_value(),"assessment tracking cycle failed");
        check(bool(runtime.set_impedance_mode(assessment.robot,JointImpedanceMode::RIGID_HOLD)),"analysis HOLD failed");
        for(unsigned i=0;i<30;++i){std::this_thread::sleep_for(std::chrono::milliseconds(5));check(runtime.step(assessment.robot).state==RobotState::ACTIVE,"stationary analysis HOLD must not expire a streaming command");}
        check(std::abs(assessment.robot.get_hold_target()[0]-.002)<1e-12,"analysis HOLD must preserve final nominal reference");
        check(bool(runtime.set_impedance_mode(assessment.robot,JointImpedanceMode::RIGID_TRACKING)),"stream timeout setup failed");
        check(bool(runtime.submit_reference(assessment.robot,JointPosVelCmd{{.002,0},{0,0}},Robot::Clock::now(),MotionSource::STREAM)),"stream input rejected");
        runtime.step(assessment.robot);
        for(unsigned i=0;i<30 && assessment.robot.get_state()==RobotState::ACTIVE;++i){std::this_thread::sleep_for(std::chrono::milliseconds(5));runtime.step(assessment.robot);}
        check(assessment.robot.get_state()==RobotState::FAULT && assessment.robot.get_last_fault()->safety_fault.code==SafetyErr::CMD_TIMEOUT,"ordinary command loss must still produce CMD_TIMEOUT");
        for(double amplitude:{.0015,.002}) {
        Fixture guarded;
        check(guarded.robot.set_calibration_tracking_gains({{10,10},{.2,.2}},JointImpedanceMode::RIGID_HOLD),"guard candidate setup failed");
        servo_tuning::NoiseEstimate guard_noise;guard_noise.encoder_step_rad=.0001;
        servo_tuning::CommandConditionedMotionGuard response_guard({},guard_noise);
        bool tripped=false;
        for(unsigned i=0;i<100 && !tripped;++i) {
            const double t=.005*i;
            guarded.bus->state.pos[0]=amplitude*std::sin(2*servo_tuning::pi*50*t);
            guarded.bus->state.vel[0]=amplitude*2*servo_tuning::pi*50*std::cos(2*servo_tuning::pi*50*t);
            const auto cycle=guarded.cycle();servo_tuning::Sample row;
            row.time_s=t;row.reference=cycle.joint_cmd.pos[0];row.actual=cycle.joint_state.pos[0];row.new_measurement=true;
            row.feedback_sequence=cycle.actuator_state.feedback_received_ns[0];
            tripped=response_guard.observe(row).triggered;
            if(tripped)check(bool(guarded.robot.stop_calibration_response()),"observed instability must enter the existing stop reaction");
        }
        check(tripped && guarded.robot.get_state()==RobotState::FAULT && guarded.robot.get_last_fault()->code==RobotErr::CALIBRATION_RESPONSE_UNSTABLE,"guarded calibration must latch a named fault");
        // The larger velocity cannot enter a zero-velocity hold within this
        // fixture's acceleration limit; the existing reaction must stop it.
        if(amplitude<.002) {
            check(!guarded.bus->stops && std::abs(guarded.bus->last.kp[0]-2)<1e-6 && std::abs(guarded.bus->last.kd[0]-.2)<1e-6,"feasible stop hold must use configured recovery gains");
            for(unsigned i=0;i<3;++i)check(bool(guarded.robot.maintain_fault_hold()),"guard stop must retain the normal fault hold path");
        } else check(guarded.bus->stops>0,"infeasible hold must use the existing motor stop fallback");
        check(guarded.robot.get_state()==RobotState::FAULT,"calibration rejection must not auto-clear the fault");
        }
        Fixture f;
        check(!f.robot.set_calibration_disturbance({.05001,0}),"disturbance larger than 5 percent effort must be rejected");
        check(!f.robot.set_calibration_disturbance({.01}),"disturbance wrong mapping must be rejected");
        check(bool(f.robot.set_calibration_disturbance({.04,-.03})),"bounded disturbance must be accepted");
        auto applied=f.cycle();
        check(std::abs(applied.joint_cmd.tor[0]-.04)<1e-9 && std::abs(applied.calibration_disturbance[1]+.03)<1e-9,"known additive disturbance must reach normal accepted MIT command");
        check(applied.command_written_ns>0 && applied.command_written_ns>=applied.actuator_state.feedback_received_ns[0],"successful bus write time and receive time must retain causal order");
        std::this_thread::sleep_for(std::chrono::milliseconds(105));
        auto expired=f.cycle();check(expired.calibration_disturbance[0]==0 && expired.joint_cmd.tor[0]==0,"unrefreshed disturbance must expire after 100 ms");
        check(bool(f.robot.set_calibration_disturbance({.04,0})),"bounded disturbance reset failed");f.robot.clear_calibration_disturbance();
        check(f.cycle().calibration_disturbance[0]==0,"explicit clear must remove disturbance");
        check(bool(f.robot.set_calibration_disturbance({.04,0})),"bounded disturbance before deactivate failed");
        check(bool(f.robot.deactivate()) && bool(f.robot.activate()),"in-memory lifecycle reset failed");
        check(f.cycle().calibration_disturbance[0]==0,"lifecycle reset must not replay stale disturbance");

        Fixture limited(false,.98);limited.cycle();
        check(bool(limited.robot.set_calibration_disturbance({.04,0})),"bounded overlay for total-effort test must be accepted before Safety");
        auto rejected=limited.robot.cycle();
        check(!rejected && limited.robot.get_state()==RobotState::FAULT,"normal total-effort Safety must reject feedforward plus disturbance overflow");
        check(!limited.bus->last.tor.empty() && std::abs(limited.bus->last.tor[0])<=1,"fault exit must not write excessive effort");
        limited.feedforward=0;
        for(unsigned k=0;k<3;++k)check(bool(limited.robot.maintain_fault_hold()),"in-memory fault hold observation failed");
        check(bool(limited.robot.clear_fault()),"in-memory clear fault failed");
        check(limited.cycle().calibration_disturbance[0]==0,"cleared fault must not resume old independent disturbance");

        Fixture displaced;const auto old_hold=displaced.robot.get_hold_target();displaced.cycle();
        displaced.bus->state.pos={-.011446,-.022507};const auto fresh=displaced.cycle();
        const auto now_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
        const auto center=servo_tuning::measured_probe_center(fresh.joint_state.pos,fresh.actuator_state.feedback_received_ns,now_ns);
        check(center && *center==fresh.joint_state.pos && *center!=old_hold,"startup center must use actual fresh feedback when nominal hold target differs by the recorded joint3/joint4 offsets");
        check(displaced.robot.get_hold_target()==old_hold,"center test must actually preserve a distinct old nominal hold target");
        check(!servo_tuning::measured_probe_center(fresh.joint_state.pos,fresh.actuator_state.feedback_received_ns,now_ns+31000000),"stale actual positions must not become establishment center");

        Fixture integral(true);
        integral.cycle();integral.bus->state.pos[0]=-.001;integral.bus->state.vel[1]=.03;
        integral.cycle();auto held=integral.cycle();
        check(held.hold_integrator_freeze_reasons.size()==2 && held.hold_integral_torque.size()==2,"integrator must expose per-axis actual state");
        check(held.hold_integrator_freeze_reasons[0]=="integrating" && held.hold_integrator_freeze_reasons[1]=="moving","one moving joint must freeze only its integrator");
        check(held.hold_integral_torque[0]>0 && held.hold_integral_torque[1]==0,"actual per-axis integral compensation must match freeze reason");
        // Exercise the actual CSV writer/loader with an untagged teaching
        // cycle as well as the historical 6-vector row format
        struct Temporary {std::filesystem::path path;~Temporary(){std::filesystem::remove_all(path);}} temporary{
            std::filesystem::temp_directory_path()/("serialarm-gain-record-"+std::to_string(Robot::Clock::now().time_since_epoch().count()))};
        ModelCalibrationMetadata metadata;metadata.task_id="memory-record-test";metadata.joint_names={"a","b"};
        ModelCalibrationRecorder recorder(128);check(bool(recorder.start(temporary.path,metadata)),"temporary recorder start failed");
        ModelCalibrationFrame frame;frame.monotonic_ns=1;frame.cycle=1;frame.phase="teaching";frame.direction="none";frame.speed_tier="none";
        frame.acceleration_source="test";frame.impedance_mode="RIGID_HOLD";frame.feedback_age_ms=0;frame.valid=true;
        frame.position=frame.velocity=frame.torque=frame.acceleration=frame.reference_position=frame.reference_velocity={0,0};
        frame.requested_kp={.083,12.4};frame.requested_kd={.017,1};
        frame.command_kp={500.0/4095,101*500.0/4095};frame.command_kd={14*5.0/4095,819*5.0/4095};
        frame.command_kp_code={1,101};frame.command_kd_code={14,819};
        recorder.push(frame);recorder.stop();
        auto loaded=load_model_calibration_frames(temporary.path,2);check(bool(loaded) && loaded->size()==1 && loaded->front().valid && loaded->front().gain_stage=="none","new full-cycle CSV must round-trip when no gain stage is active");
        check(loaded->front().requested_kp==frame.requested_kp && loaded->front().command_kp==frame.command_kp && loaded->front().command_kp_code==frame.command_kp_code && loaded->front().command_kd_code==frame.command_kd_code,"requested gains and accepted protocol codes must survive actual CSV writer/loader");
        std::ifstream original(temporary.path/"frames.csv");std::string header,line;std::getline(original,header);std::getline(original,line);original.close();
        std::stringstream fields(line);std::string item;std::vector<std::string> columns;while(std::getline(fields,item,','))columns.push_back(item);
        for(const std::size_t count:{23,29,33,58}) {
            {std::ofstream legacy(temporary.path/"frames.csv");legacy<<"legacy_header\n";for(std::size_t i=0;i<count;++i){if(i)legacy<<',';legacy<<columns[i];}legacy<<'\n';}
            auto historical=load_model_calibration_frames(temporary.path,2);check(bool(historical) && historical->size()==1 && historical->front().valid && historical->front().requested_kp.empty() && historical->front().command_kp_code.empty(),"all four historical CSV schemas must remain readable without invented protocol evidence");
        }
        std::cout<<"in-memory Robot checks passed: bounded independent disturbance, 100 ms expiry, actual write timestamps, normal Safety, lifecycle/fault clearing, per-axis integral telemetry, extended and legacy CSV round-trip\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
