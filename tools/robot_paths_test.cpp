#include "serial_arm/robot.hpp"
#include "serial_arm/runtime/control_runtime.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
using namespace serial_arm;
void check(bool ok,const char* why){if(!ok) throw std::runtime_error(why);}
class TestBus final:public MotorBus {
public:
    HardwareCapabilities caps{{"a",-2,2,2,5,100,5,"a",0,0,0,true,true}};
    ActuatorState state{{0},{0},{0},{1},{1},{0},{},{1}};
    ActuatorCtrlCmd last;
    int writes{0};
    int activations{0};
    int* deactivation_counter{nullptr};
    std::uint64_t feedback_stamp_ns{0};
    double connected_velocity_resolution{-1.0};
    ControlPath path{ControlPath::MIT};
    ActuatorVector startup_speed_limit;
    tl::expected<void,MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,MotorBusErr> connect() override{
        if(connected_velocity_resolution >= 0.0) caps[0].velocity_resolution=connected_velocity_resolution;
        return {};
    }
    tl::expected<void,MotorBusErr> activate() override{++activations;state.enabled={1};return {};}
    tl::expected<void,MotorBusErr> deactivate() override{if(deactivation_counter)++*deactivation_counter;state.enabled={0};return {};}
    tl::expected<void,MotorBusErr> stop() override{return {};}
    tl::expected<void,MotorBusErr> recover() override{return {};}
    tl::expected<void,MotorBusErr> select_control_path(ControlPath p,const ActuatorVector& speed_limit = {}) override{path=p;startup_speed_limit=speed_limit;return {};}
    tl::expected<ActuatorState,MotorBusErr> read() override {
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
        state.feedback_received_ns={feedback_stamp_ns?feedback_stamp_ns:static_cast<std::uint64_t>(ns)};return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& cmd) override{++writes;last=cmd;return {};}
    const HardwareCapabilities& capabilities() const noexcept override{return caps;}
    std::size_t size() const noexcept override{return 1;}
    void cleanup() noexcept override{}
};
class SkewBus final:public MotorBus {
public:
    bool moving{false};
    HardwareCapabilities caps{{"a",-2,2,2,5,100,5,"a",0,0,0,true,true},
                              {"b",-2,2,2,5,100,5,"b",0,0,0,true,true}};
    const std::uint64_t origin=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count())-10000000;
    tl::expected<void,MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,MotorBusErr> connect() override{return {};}
    tl::expected<void,MotorBusErr> activate() override{return {};}
    tl::expected<void,MotorBusErr> deactivate() override{return {};}
    tl::expected<void,MotorBusErr> stop() override{return {};}
    tl::expected<void,MotorBusErr> recover() override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read() override {
      const auto now=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count());
      const auto a=now-2000000,b=now-1000000;
      return ActuatorState{{moving?(a-origin)*1e-9:0,moving?(b-origin)*2e-9:0},{moving?1.0:0.0,moving?2.0:0.0},{0,0},{1,1},{1,1},{0,0},{a,b},{1,1}};
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd&) override{return {};}
    const HardwareCapabilities& capabilities() const noexcept override{return caps;}
    std::size_t size() const noexcept override{return 2;}
    void cleanup() noexcept override{}
};
RobotCfg cfg(ControlPath path) {
    RobotCfg c;c.joint_names={"a"};c.runtime.write_enabled=true;c.runtime.ctrl_frequency_hz=200;c.runtime.control_path=path;
    c.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;

    c.ctrller.joints_count=1;c.ctrller.rigid_gains={{4},{.5}};c.ctrller.compliant_gains={{2},{.2}};c.ctrller.drag_gains={{0},{.2}};
    c.mapper={1,{1},{1},{1},{0},{0}};
    c.safety.fault_recovery.velocity_tolerance={.05};c.safety.joints_count=1;c.safety.max_dt_s=.2;c.safety.state_timeout_s=.5;c.safety.cmd_timeout_s=.04;
    auto& l=c.safety.limits;l.has_position_limit={1};l.min_pos={-2};l.max_pos={2};l.max_vel={2};l.max_acc={100};l.max_effort={5};l.max_kp={100};l.max_kd={5};l.pos_margin={.01};l.max_tracking_error={.5};

    c.dynamics.urdf_path="memory.urdf";c.dynamics.joint_names=c.joint_names;c.dynamics.gravity_scale={1};c.dynamics.gravity_model_validated=true;
    auto& a=c.capability.admittance;a.validation={true,true,{.05}};a.enabled=true;a.joint_enabled={1};a.observer.momentum_gain={25};
    a.calibration.torque_bias={0};a.calibration.torque_threshold={0};a.calibration.friction.velocity_transition={.03};
    a.calibration.friction.positive_coulomb={0};a.calibration.friction.positive_viscous={0};
    a.calibration.friction.negative_coulomb={0};a.calibration.friction.negative_viscous={0};
    a.controller.mass={1};a.controller.damping={2};a.controller.stiffness={4};a.controller.max_delta_q={.2};a.controller.max_delta_q_dot={.5};
    return c;
}
int main(){
  for(const auto path:{ControlPath::MIT,ControlPath::CSP}) {
    int disabled=0;
    {
      Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
      raw->state.pos={.35};raw->state.enabled={0};raw->deactivation_counter=&disabled;
      auto c=cfg(path);c.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;c.capability.admittance={};
      if(path==ControlPath::CSP)c.ctrller.rigid_gains={};
      check(bool(robot.configure(c,std::move(bus))),"disabled observation configure failed");
      check(bool(robot.connect()),"disabled observation connect failed");
      auto observed=robot.observe();
      check(bool(observed) && observed->joint_state.pos==JointVector{.35} && observed->actuator_state.enabled==std::vector<std::uint8_t>{0},"disabled feedback was hidden");
      ControlRuntime runtime;const auto step=runtime.step(robot);
      check(step.observation.has_value() && !step.cycle && step.state==RobotState::INACTIVE,"disabled feedback was published as a control cycle");
      check(raw->activations==0 && raw->writes==0 && disabled==0,"observation sent an enable, write or disable");
    }
    check(disabled==0,"destroying observation-only Robot disabled hardware");
    Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
    raw->state.pos={.35};raw->state.enabled={0};
    auto c=cfg(path);c.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;c.capability.admittance={};
    if(path==ControlPath::CSP)c.ctrller.rigid_gains={};
    check(bool(robot.configure(c,std::move(bus))) && bool(robot.activate()),"current-position startup failed");
    check(raw->writes==1 && raw->last.pos==ActuatorVector{.35} && raw->last.vel==ActuatorVector{0},"startup omitted the current-position anchor");
    check(path==ControlPath::MIT?raw->last.kp[0]>0:raw->last.kp[0]==0,"startup selected incorrect drive command");
    check(bool(robot.force_deactivate()),"startup fixture disable failed");
  }
  for(const int failure:{0,1,2,3}) {
    Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();raw->state.enabled={0};
    auto c=cfg(ControlPath::MIT);c.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;c.capability.admittance={};
    if(failure==0)raw->state.pos={2.1};
    if(failure==1)raw->state.err_code={1};
    if(failure==2)c.ctrller.rigid_gains={};
    if(failure==3)c.ctrller.rigid_gains={{0},{0}};
    check(bool(robot.configure(c,std::move(bus))) && bool(robot.connect()),"preflight fixture could not connect");
    check(bool(robot.observe()),"unsafe hardware could not be inspected without motion");
    check(!robot.activate() && robot.get_state()==RobotState::INACTIVE,"unsafe first command was enabled");
    check(raw->activations==0 && raw->writes==0,"preflight rejected after enable or control write");
  }
  {
    auto c=cfg(ControlPath::CSP);c.capability.admittance.validation.observation_validated=false;c.capability.admittance.validation.response_validated=false;
    Robot robot;auto bus=std::make_unique<TestBus>();
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{0},{0},{{1}}};};
    check(bool(robot.configure(c,std::move(bus),f,m)),"commissioning config failed");
    check(bool(robot.activate()),"commissioning activation failed");
    check(!robot.set_impedance_mode(JointImpedanceMode::COMPLIANT),"unverified admittance enabled");
    check(!robot.set_impedance_mode(JointImpedanceMode::DRAG),"unverified drag enabled");
    check(!robot.set_admittance_validation(true),"test enabled without external-effort verification");
    auto a=robot.get_admittance_cfg();a.validation.observation_validated=true;
    check(bool(robot.set_admittance_cfg(a)),"observer validation update failed");
    check(bool(robot.set_admittance_validation(true)),"finite stiffness test rejected");
    check(robot.get_impedance_mode()==JointImpedanceMode::COMPLIANT,"test did not enter compliant mode");
    check(!robot.set_impedance_mode(JointImpedanceMode::DRAG),"trial bypassed drag acceptance");
    auto zero=a;zero.controller.stiffness={0};
    check(!robot.set_admittance_cfg(zero),"trial accepted zero stiffness");
    check(bool(robot.set_admittance_validation(false)),"test could not stop");
    check(robot.get_impedance_mode()==JointImpedanceMode::RIGID,"test did not stop in rigid hold");
    a.validation.response_validated=true;a.validation.velocity_tolerance={.04};
    check(bool(robot.set_admittance_cfg(a)) && !robot.get_admittance_cfg().validation.response_validated,
        "changed response tolerance reused old acceptance");
    a.validation.response_validated=true;check(bool(robot.set_admittance_cfg(a)),"response acceptance failed");
    check(bool(robot.set_impedance_mode(JointImpedanceMode::DRAG)),"verified drag rejected");
    a.controller.damping={3};check(bool(robot.set_admittance_cfg(a)),"response parameter update failed");
    check(!robot.get_admittance_cfg().validation.response_validated,"response evidence survived parameter change");
    check(robot.get_impedance_mode()==JointImpedanceMode::RIGID,"parameter change did not hold measured pose");
    a=robot.get_admittance_cfg();a.calibration.torque_bias={.1};a.validation={true,true,{.05}};
    check(bool(robot.set_admittance_cfg(a)),"observation parameter update failed");
    check(!robot.get_admittance_cfg().validation.observation_validated,"new bias reused old acceptance");
    check(robot.get_admittance_cfg().validation.velocity_tolerance==JointVector{.05},"revoking quality erased response tolerance configuration");
  }
  auto invalid_drag=cfg(ControlPath::MIT);
  invalid_drag.ctrller.drag_gains.kp[0]=.01;
  check(!validate_robot_core_cfg(invalid_drag),"nonzero drag Kp accepted by Robot configuration");
  for(const auto path:{ControlPath::MIT,ControlPath::CSP}) {
    auto c=cfg(path);c.mapper.pos_ratio={2};
    auto& friction=c.capability.admittance.calibration.friction;
    friction.enabled=true;friction.velocity_transition={.03};
    friction.positive_coulomb={1};friction.negative_coulomb={-1};
    Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
    raw->caps[0].velocity_resolution=.002;
    raw->connected_velocity_resolution=.04;raw->state.vel={.1};
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{0},{0},{{1}}};};
    check(bool(robot.configure(c,std::move(bus),f,m)),"connected resolution fixture configuration failed");
    for(const double actuator_resolution:{.04,.02}) {
      raw->connected_velocity_resolution=actuator_resolution;
      raw->feedback_stamp_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count())-20000000;
      check(bool(robot.activate()),"connected resolution fixture activation failed");
      double prediction=0;
      for(int i=0;i<4;++i) {
        // Real position progress distinguishes motion from stationary velocity-code jitter.
        raw->state.pos[0]+=.1*.005;
        raw->feedback_stamp_ns+=5000000;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto out=robot.cycle();check(bool(out),"connected resolution fixture cycle failed");
        check(out->friction_residual_hat.size()==1,"connected resolution friction diagnostic missing");
        prediction=out->friction_residual_hat[0];
      }
      // Actuator dq=.1 maps to joint dq=.05. With resolution=.04/2,
      // the low-speed blend is smoothstep((.05-.02)/(.08-.02))=.5.
      const double expected=actuator_resolution==.04?.5:1.0;
      check(std::abs(prediction-expected)<1e-10,"friction used stale or unmapped pre-connect velocity resolution");
      check(bool(robot.force_deactivate()),"connected resolution fixture deactivation failed");
    }
  }
  for(const auto path:{ControlPath::MIT,ControlPath::CSP}) {
    Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{.4};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{.4},{0},{{1}}};};
    auto configured=robot.configure(cfg(path),std::move(bus),f,m);
    if(!configured) throw std::runtime_error("configure "+std::to_string(static_cast<int>(configured.error().code)));
    check(bool(robot.activate()),"activation failed");check(raw->path==path,"wrong selected path");
    if(path==ControlPath::CSP) check(raw->startup_speed_limit.size()==1 && raw->startup_speed_limit[0]==2.0,"startup speed limit missing");
    const auto bad_target=robot.set_cmd(JointPosCmd{{3.0}});
    check(!bad_target && bad_target.error().safety_fault.code==SafetyErr::CMD_POS_LIMIT,"out-of-range point accepted");
    const auto bad_velocity=robot.set_cmd(JointPosVelCmd{{0},{3.0}});
    check(!bad_velocity && bad_velocity.error().safety_fault.code==SafetyErr::CMD_VEL_LIMIT,"out-of-range velocity accepted");
    check(bool(robot.set_cmd(JointPosCmd{{.01}})),"point command rejected");
    for(int i=0;i<30;++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=robot.cycle();
      if(!out) throw std::runtime_error("cycle "+std::to_string(static_cast<int>(out.error().code))+" safety "+std::to_string(static_cast<int>(out.error().safety_fault.code)));
      check(raw->last.path==path,"wrong command path");
      if(path==ControlPath::CSP) check(raw->last.kp[0]==0 && raw->last.kd[0]==0 && raw->last.tor[0]==0 && raw->last.position_speed_limit[0]>0,"CSP command contaminated");
      else check(raw->last.kp[0]>0 && std::abs(raw->last.tor[0]-.4)<1e-8,"MIT gains or gravity missing");
      raw->state.pos=raw->last.pos;
    }
    check(robot.get_state()==RobotState::ACTIVE,"point command expired");
    check(bool(robot.set_impedance_mode(JointImpedanceMode::COMPLIANT)),"switch to compliant failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=robot.cycle();check(bool(out),"switch cycle failed");
    check(std::abs(out->joint_cmd.pos[0]-raw->state.pos[0])<.01,"switch jumped");
    if(path==ControlPath::MIT) {
      check(robot.set_calibration_tracking_gains({{1.1},{.3}},JointImpedanceMode::COMPLIANT),"compliant calibration override rejected");
      for(int i=0;i<75;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"compliant calibration cycle failed");
        raw->state.pos=out->joint_cmd.pos;
      }
      check(out->interaction_mode==JointImpedanceMode::COMPLIANT && std::abs(out->joint_cmd.kp[0]-1.1)<1e-8 &&
            std::abs(out->joint_cmd.kd[0]-.3)<1e-8,"compliant calibration mode/gains mismatch");
      robot.clear_calibration_tracking_gains();
    }
    check(bool(robot.set_impedance_mode(JointImpedanceMode::DRAG)),"switch to drag failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"drag cycle failed");
    check(out->joint_cmd.kp[0]==0,"drag kp must be zero");
    if(path==ControlPath::MIT) {
      // Physical dragging moves the arm without changing the MIT position reference.
      raw->state.pos[0]=.08;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"moved drag cycle failed");
      check(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID)),"drag exit failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"drag exit cycle failed");
      check(std::abs(out->joint_cmd.pos[0]-.08)<1e-6,"drag exit returned to stale reference");
    }
    if(path==ControlPath::CSP) {
      // The observer drives compliant displacement without a new motion command.
      for(int i=0;i<60;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"CSP drag point cycle failed");
        raw->state.pos=out->joint_cmd.pos;
      }
      const auto anchor=out->joint_cmd.pos[0];
      check(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID)),"CSP drag exit failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"CSP drag exit cycle failed");
      check(std::abs(out->joint_cmd.pos[0]-anchor)<.002,"CSP drag exit returned to stale reference");
      check(bool(robot.set_impedance_mode(JointImpedanceMode::DRAG)),"CSP second drag entry failed");
      check(bool(robot.set_cmd(JointPosCmd{{.4}})),"CSP in-drag point rejected");
      for(int i=0;i<3;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"CSP in-drag motion failed");raw->state.pos=out->joint_cmd.pos;
      }
      const auto moving_anchor=out->joint_cmd.pos[0];
      check(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID)),"CSP moving drag exit failed");
      for(int i=0;i<20;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"CSP post-switch motion failed");raw->state.pos=out->joint_cmd.pos;
      }
      check(std::abs(out->joint_cmd.pos[0]-moving_anchor)<.02,"drag exit continued an old in-drag target");
      check(bool(robot.set_cmd(JointPosCmd{{.8}})),"pre-drag point rejected");
      for(int i=0;i<3;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"pre-drag point cycle failed");raw->state.pos=out->joint_cmd.pos;
      }
      check(bool(robot.set_impedance_mode(JointImpedanceMode::DRAG)),"pending drag entry failed");
      check(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID)),"pending drag exit failed");
      const auto pending_anchor=out->joint_cmd.pos[0];
      for(int i=0;i<20;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));out=robot.cycle();check(bool(out),"pending target continuation failed");raw->state.pos=out->joint_cmd.pos;
      }
      check(std::abs(out->joint_cmd.pos[0]-pending_anchor)<.02,"drag exit continued a pre-drag target");
      check(bool(robot.set_cmd(JointPosCmd{{out->joint_cmd.pos[0]}},Robot::Clock::now(),true)),"CSP stream rejected");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));check(bool(robot.cycle()),"CSP stream cycle failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(60));check(!robot.cycle(),"CSP timeout not latched");
      const auto recovery=robot.enter_fault_drag_recovery();
      check(bool(recovery) && robot.get_fault_hold_mode()==FaultHoldMode::DRAG_RECOVERY,"CSP drag recovery unavailable");
    }
    if(path==ControlPath::MIT) {
      ControlRuntime runtime;
      check(bool(runtime.set_impedance_mode(robot,JointImpedanceMode::RIGID)),"stream mode failed");
      check(bool(runtime.submit_reference(robot,JointPosCmd{{.01}},Robot::Clock::now())),"runtime point rejected");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));check(runtime.step(robot).cycle.has_value(),"runtime point cycle failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      const auto point_result=runtime.step(robot);
      check(point_result.cycle.has_value(),"runtime point expired");
      check(bool(runtime.submit_reference(robot,JointPosVelCmd{point_result.cycle->joint_cmd.pos, point_result.cycle->joint_cmd.vel},Robot::Clock::now(),MotionSource::STREAM)),"stream command rejected");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));check(runtime.step(robot).cycle.has_value(),"stream initial cycle failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      auto timeout=runtime.step(robot);
      check(timeout.state==RobotState::FAULT && timeout.fault && timeout.fault->safety_fault.code==SafetyErr::CMD_TIMEOUT,"stream expiry not latched");
    }
  }
  {
    auto c=cfg(ControlPath::MIT);c.ctrller.allow_full_cmd=true;
    Robot robot;
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{.4};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{.4},{0},{{1}}};};
    check(bool(robot.configure(c,std::make_unique<TestBus>(),f,m)) && bool(robot.activate()),"full command fixture setup failed");
    JointCtrlCmd command;command.pos={0};command.vel={0};command.tor={0};command.kp={4};command.kd={.5};
    check(bool(robot.set_full_cmd(command)),"full command rejected");
    ControlRuntime runtime;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));check(runtime.step(robot).cycle.has_value(),"full command initial cycle failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    const auto expired=runtime.step(robot);
    check(expired.state==RobotState::FAULT && expired.fault && expired.fault->safety_fault.code==SafetyErr::CMD_TIMEOUT,
        "full control command must require continuous refresh even without a runtime reference source");
  }
  for(const auto mode:{JointImpedanceMode::RIGID,JointImpedanceMode::SERVO_TRACKING}) {
    auto c=cfg(ControlPath::MIT);
    c.hold_compensation.enabled=true;
    c.hold_compensation.integral_gain={1};c.hold_compensation.max_torque={.2};
    c.hold_compensation.max_rate={1};c.hold_compensation.contact_threshold={1};
    c.hold_compensation.velocity_threshold={.02};c.hold_compensation.error_threshold={.03};
    Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();raw->state.tor={.4};
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{.4};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{.4},{0},{{1}}};};
    check(bool(robot.configure(c,std::move(bus),f,m)) && bool(robot.activate()),"holding compensation fixture setup failed");
    check(bool(robot.set_impedance_mode(mode)),"holding compensation mode rejected");
    check(bool(robot.set_cmd(JointPosCmd{{.01}})),"holding compensation target rejected");
    double compensation=0;
    for(int i=0;i<35;++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      auto out=robot.cycle();check(bool(out),"holding compensation cycle failed");
      compensation=out->hold_integral_torque[0];
    }
    check(compensation>0,"rigid and servo tracking must both compensate steady position error");
  }
  {
    Robot robot;auto bus=std::make_unique<TestBus>();
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{.4};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{.4},{0},{{1}}};};
    check(bool(robot.configure(cfg(ControlPath::MIT),std::move(bus),f,m)),"external fixture configuration failed");
    check(bool(robot.activate()),"external fixture activation failed");
    ControlRuntime runtime;
    check(bool(runtime.submit_reference(robot,JointPosCmd{{0}},Robot::Clock::now(),MotionSource::EXTERNAL)),"external command rejected");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));check(runtime.step(robot).cycle.has_value(),"external initial cycle failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));auto timeout=runtime.step(robot);
    check(timeout.state==RobotState::FAULT && timeout.fault && timeout.fault->safety_fault.code==SafetyErr::CMD_TIMEOUT,"external stream expiry not latched");
  }
  {
    auto c=cfg(ControlPath::MIT);c.joint_names={"a","b"};c.ctrller.joints_count=2;
    c.ctrller.rigid_gains={{4,4},{.5,.5}};c.ctrller.compliant_gains={{2,2},{.2,.2}};c.ctrller.drag_gains={{0,0},{.2,.2}};
    c.mapper={2,{1,1},{1,1},{1,1},{0,0},{0,0}};c.safety.joints_count=2;c.safety.fault_recovery.velocity_tolerance={.05,.001};
    auto& l=c.safety.limits;l.has_position_limit={1,1};l.min_pos={-2,-2};l.max_pos={2,2};l.max_vel={2,2};l.max_acc={100,100};l.max_effort={5,5};l.max_kp={100,100};l.max_kd={5,5};l.pos_margin={.01,.01};l.max_tracking_error={.5,.5};
    c.dynamics.joint_names=c.joint_names;c.dynamics.gravity_scale={1,1};
    auto& a=c.capability.admittance;a.validation.velocity_tolerance={.05,.001};a.calibration.friction.velocity_transition={.03,.001};a.joint_enabled={1,1};a.observer.momentum_gain={25,25};a.calibration.torque_bias={0,0};a.calibration.torque_threshold={0,0};
    a.calibration.friction.positive_coulomb={0,0};a.calibration.friction.positive_viscous={0,0};a.calibration.friction.negative_coulomb={0,0};a.calibration.friction.negative_viscous={0,0};
    a.controller.mass={1,1};a.controller.damping={2,2};a.controller.stiffness={4,4};a.controller.max_delta_q={.2,.2};a.controller.max_delta_q_dot={.5,.5};
    JointState evaluated;Robot robot;
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0,0};};
    auto m=[&](const JointState& state,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{evaluated=state;return InteractionModelState{{0,0},{0,0},{{1,0},{0,1}}};};
    auto skew_bus=std::make_unique<SkewBus>();auto* skew=skew_bus.get();
    check(bool(robot.configure(c,std::move(skew_bus),f,m)) && bool(robot.activate()),"skew fixture setup failed");
    skew->moving=true;
    bool seen=false;
    for(int i=0;i<5;++i){std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=robot.cycle();check(bool(out),"skew fixture cycle failed");
      if(out->new_measurement) {seen=true;check(std::abs(out->observer_state.pos[1]-2*out->observer_state.pos[0])<1e-9,"observer state not aligned");
        check(std::abs(evaluated.pos[1]-2*evaluated.pos[0])<1e-9,"model used asynchronous positions");}}
    check(seen,"skewed feedback never became alignable");
  }

  // A human mode change must capture the dragged pose even if a completed
  // background STREAM kept sending its old stationary target.
  for(auto path:{ControlPath::MIT,ControlPath::CSP}) for(auto dest:{JointImpedanceMode::RIGID,JointImpedanceMode::COMPLIANT}) {
    auto c=cfg(path);Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{0},{0},{{1}}};};
    check(bool(robot.configure(c,std::move(bus),f,m))&&bool(robot.activate()),"handover configure");
    ControlRuntime runtime;check(bool(runtime.set_impedance_mode(robot,JointImpedanceMode::DRAG)),"handover drag");raw->state.pos={.2};
    for(int i=0;i<4;i++) {std::this_thread::sleep_for(std::chrono::milliseconds(5));auto now=Robot::Clock::now();
      check(bool(runtime.submit_reference(robot,JointPosVelCmd{{0},{0}},now,MotionSource::STREAM)),"stale stream");
      check(runtime.step(robot,now).cycle.has_value(),"drag refresh");}
    check(bool(runtime.submit_reference(robot,JointPosVelCmd{{0},{0}},Robot::Clock::now(),MotionSource::STREAM)),"pending old frame");
    check(bool(runtime.set_impedance_mode(robot,dest)),"handover destination");check(runtime.source()==MotionSource::NONE,"old mailbox survived drag exit");
    for(int i=0;i<20;i++) {std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=runtime.step(robot);
      check(out.cycle && std::abs(out.cycle->joint_cmd.pos[0]-.2)<1e-6,"drag exit pulled to stale target");}
  }
  {
    // CSP RIGID needs neither external observation nor MIT gain groups.
    auto c=cfg(ControlPath::CSP);c.capability.admittance={};c.ctrller.rigid_gains={};c.ctrller.compliant_gains={};c.ctrller.drag_gains={};
    c.safety.limits.max_vel={5.4};c.safety.limits.max_acc={10.8};Robot robot;
    auto bus=std::make_unique<TestBus>();auto* raw=bus.get();raw->caps[0].max_vel=5.4;
    check(bool(robot.configure(c,std::move(bus)))&&bool(robot.activate()),"CSP rigid wrongly requires MDK/PD");
    check(!robot.set_impedance_mode(JointImpedanceMode::COMPLIANT),"CSP compliant accepted without MDK");
    check(bool(robot.set_cmd(JointPosCmd{{1}})),"raw CSP point rejected");auto now=Robot::Clock::now();double peak=0;
    for(int i=0;i<300;i++) {now+=std::chrono::milliseconds(5);auto out=robot.cycle(now);check(bool(out),"raw CSP point cycle");
      peak=std::max(peak,out->joint_cmd.pos[0]);raw->state.pos=out->joint_cmd.pos;}
    if(peak>1+1e-6 || std::abs(raw->last.pos[0]-1)>1e-6) throw std::runtime_error("CSP point missing braking or endpoint capture: peak="+std::to_string(peak)+" final="+std::to_string(raw->last.pos[0])+" v="+std::to_string(raw->last.vel[0]));
  }
  {
    auto c=cfg(ControlPath::MIT);Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();c.capability.admittance={};
    c.shutdown.mit.velocity_tolerance={10};c.safety.fault_recovery.velocity_tolerance={.01};
    check(bool(robot.configure(c,std::move(bus),[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};}))&&bool(robot.activate()),"disabled fault fixture");
    raw->state.err_code={1};ControlRuntime runtime;auto fault=runtime.step(robot);
    check(fault.state==RobotState::FAULT && !robot.is_fault_holding(),"driver fault must disable");
    const int writes=raw->writes,activations=raw->activations;check(!robot.clear_fault(),"unhealthy driver fault cleared");
    raw->state.err_code={0};raw->state.pos={.15};
    raw->state.vel={.02};for(int i=0;i<4;++i)(void)runtime.step(robot);
    check(!robot.clear_fault(),"parking tolerance changed the independent fault-clear stationary threshold");
    raw->state.vel={0};
    for(int i=0;i<4;i++){auto observed=runtime.step(robot);check(!observed.cycle && observed.fault_feedback && std::abs(observed.fault_feedback->joint_state.pos[0]-.15)<1e-12,"disabled FAULT feedback stale");}
    check(raw->writes==writes&&raw->activations==activations,"disabled FAULT sent actuation");
    check(bool(robot.clear_fault())&&robot.get_state()==RobotState::INACTIVE,"healthy disabled fault did not return INACTIVE");
    check(raw->writes==writes&&raw->activations==activations,"clear fault automatically activated drive");
  }
  for(auto path:{ControlPath::MIT,ControlPath::CSP}) {
    auto c=cfg(path);Robot robot;auto bus=std::make_unique<TestBus>();auto* raw=bus.get();
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};};
    auto m=[](const JointState&,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{return InteractionModelState{{0},{0},{{1}}};};
    check(bool(robot.configure(c,std::move(bus),f,m))&&bool(robot.activate()),"drag recovery fixture");raw->state.pos={2.02};ControlRuntime runtime;
    auto fault=runtime.step(robot);check(fault.state==RobotState::FAULT&&robot.is_fault_holding(),"position fault must capture fresh pose");
    check(std::abs(raw->last.pos[0]-2.02)<1e-9,"position fault held an old pose");
    check(bool(robot.enter_fault_drag_recovery()),"drag recovery rejected");
    if(path==ControlPath::MIT) {raw->state.pos={1.9};for(int i=0;i<5;i++)check(bool(robot.maintain_fault_hold()),"MIT drag refresh");check(raw->last.kp[0]==0,"fault drag must use zero Kp");}
    else {raw->state.tor={1};for(int i=0;i<90;i++){std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=runtime.step(robot);
      check(out.fault_hold_maintained&&out.fault_feedback,"CSP recovery stopped");check(raw->last.pos[0]<=2.02+1e-9,"CSP recovery commanded outward");raw->state.pos=raw->last.pos;}
      check(raw->last.pos[0]<2 && raw->last.kp[0]==0&&raw->last.kd[0]==0&&raw->last.tor[0]==0,"CSP recovery did not reuse observer/MDK position path");}
    check(robot.get_state()==RobotState::FAULT,"drag recovery cleared fault implicitly");
    for(int i=0;i<4;i++)check(bool(robot.maintain_fault_hold()),"healthy recovery refresh");
    check(bool(robot.clear_fault())&&robot.get_state()==RobotState::ACTIVE,"recovered pose could not clear fault");
  }
  {
    auto c=cfg(ControlPath::MIT);c.dynamics.gravity_model_validated=false;c.capability.admittance={};Robot robot;
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0};};
    check(bool(robot.configure(c,std::make_unique<TestBus>(),f))&&bool(robot.activate()),"unverified model fixture");
    check(!robot.set_impedance_mode(JointImpedanceMode::DRAG),"unverified normal drag accepted");
    check(bool(robot.force_deactivate())&&bool(robot.set_gravity_verification_active(true))&&bool(robot.activate()),"verification context");
    check(bool(robot.set_impedance_mode(JointImpedanceMode::DRAG)),"first gravity verification circular gate");
    check(bool(robot.set_gravity_verification_active(false)),"verification context cleanup");
  }
  std::cout<<"robot paths passed\n";
}
