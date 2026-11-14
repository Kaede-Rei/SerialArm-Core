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
    ControlPath path{ControlPath::MIT};
    ActuatorVector startup_speed_limit;
    tl::expected<void,MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,MotorBusErr> connect() override{return {};}
    tl::expected<void,MotorBusErr> activate() override{return {};}
    tl::expected<void,MotorBusErr> deactivate() override{return {};}
    tl::expected<void,MotorBusErr> stop() override{return {};}
    tl::expected<void,MotorBusErr> recover() override{return {};}
    tl::expected<void,MotorBusErr> select_control_path(ControlPath p,const ActuatorVector& speed_limit = {}) override{path=p;startup_speed_limit=speed_limit;return {};}
    tl::expected<ActuatorState,MotorBusErr> read() override {
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
        state.feedback_received_ns={static_cast<std::uint64_t>(ns)};return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& cmd) override{last=cmd;return {};}
    const HardwareCapabilities& capabilities() const noexcept override{return caps;}
    std::size_t size() const noexcept override{return 1;}
    void cleanup() noexcept override{}
};
class SkewBus final:public MotorBus {
public:
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
      return ActuatorState{{(a-origin)*1e-9,(b-origin)*2e-9},{1,2},{0,0},{1,1},{1,1},{0,0},{a,b},{1,1}};
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd&) override{return {};}
    const HardwareCapabilities& capabilities() const noexcept override{return caps;}
    std::size_t size() const noexcept override{return 2;}
    void cleanup() noexcept override{}
};
RobotCfg cfg(ControlPath path) {
    RobotCfg c;c.joint_names={"a"};c.runtime.write_enabled=true;c.runtime.ctrl_frequency_hz=200;c.runtime.control_path=path;
    c.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    c.shutdown.park_before_disable=false;c.shutdown.park_pos={0};
    c.ctrller.joints_count=1;c.ctrller.rigid_gains={{4},{.5}};c.ctrller.compliant_gains={{2},{.2}};c.ctrller.drag_gains={{0},{.2}};
    c.mapper={1,{1},{1},{1},{0},{0}};
    c.safety.joints_count=1;c.safety.max_dt_s=.2;c.safety.state_timeout_s=.5;c.safety.cmd_timeout_s=.04;
    auto& l=c.safety.limits;l.has_position_limit={1};l.min_pos={-2};l.max_pos={2};l.max_vel={2};l.max_acc={100};l.max_effort={5};l.max_kp={100};l.max_kd={5};l.pos_margin={.01};l.max_tracking_error={.5};
    c.safety.fault_recovery.compliant_recovery={{.1},{.1},{.1},.5};
    c.dynamics.urdf_path="memory.urdf";c.dynamics.joint_names=c.joint_names;c.dynamics.gravity_scale={1};
    auto& a=c.capability.admittance;a.enabled=true;a.joint_enabled={1};a.observer.momentum_gain={25};
    a.calibration.torque_bias={0};a.calibration.torque_threshold={0};
    a.calibration.friction.positive_coulomb={0};a.calibration.friction.positive_viscous={0};
    a.calibration.friction.negative_coulomb={0};a.calibration.friction.negative_viscous={0};
    a.controller.mass={1};a.controller.damping={2};a.controller.stiffness={4};a.controller.max_delta_q={.2};a.controller.max_delta_q_dot={.5};
    return c;
}
int main(){
  auto invalid_drag=cfg(ControlPath::MIT);
  invalid_drag.ctrller.drag_gains.kp[0]=.01;
  check(!validate_robot_core_cfg(invalid_drag),"nonzero drag Kp accepted by Robot configuration");
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
      check(out->joint_cmd.pos[0]>moving_anchor+.003,"mode switch discarded active point target");
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
      check(out->joint_cmd.pos[0]>pending_anchor+.003,"drag switch discarded prior active point target");
      check(bool(robot.set_cmd(JointPosCmd{{out->joint_cmd.pos[0]}},Robot::Clock::now(),true)),"CSP stream rejected");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));check(bool(robot.cycle()),"CSP stream cycle failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(60));check(!robot.cycle(),"CSP timeout not latched");
      const auto recovery=robot.enter_fault_compliant_recovery();
      check(!recovery && recovery.error().code==RobotErr::FAULT_RECOVERY_NOT_ALLOWED,"CSP fault falsely reported compliant recovery");
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
    auto c=cfg(ControlPath::MIT);c.joint_names={"a","b"};c.shutdown.park_pos={0,0};c.ctrller.joints_count=2;
    c.ctrller.rigid_gains={{4,4},{.5,.5}};c.ctrller.compliant_gains={{2,2},{.2,.2}};c.ctrller.drag_gains={{0,0},{.2,.2}};
    c.mapper={2,{1,1},{1,1},{1,1},{0,0},{0,0}};c.safety.joints_count=2;
    auto& l=c.safety.limits;l.has_position_limit={1,1};l.min_pos={-2,-2};l.max_pos={2,2};l.max_vel={2,2};l.max_acc={100,100};l.max_effort={5,5};l.max_kp={100,100};l.max_kd={5,5};l.pos_margin={.01,.01};l.max_tracking_error={.5,.5};
    c.safety.fault_recovery.compliant_recovery={{.1,.1},{.1,.1},{.1,.1},.5};c.dynamics.joint_names=c.joint_names;c.dynamics.gravity_scale={1,1};
    auto& a=c.capability.admittance;a.joint_enabled={1,1};a.observer.momentum_gain={25,25};a.calibration.torque_bias={0,0};a.calibration.torque_threshold={0,0};
    a.calibration.friction.positive_coulomb={0,0};a.calibration.friction.positive_viscous={0,0};a.calibration.friction.negative_coulomb={0,0};a.calibration.friction.negative_viscous={0,0};
    a.controller.mass={1,1};a.controller.damping={2,2};a.controller.stiffness={4,4};a.controller.max_delta_q={.2,.2};a.controller.max_delta_q_dot={.5,.5};
    JointState evaluated;Robot robot;
    auto f=[](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector{0,0};};
    auto m=[&](const JointState& state,double)->tl::expected<InteractionModelState,ModelFeedforwardErr>{evaluated=state;return InteractionModelState{{0,0},{0,0},{{1,0},{0,1}}};};
    check(bool(robot.configure(c,std::make_unique<SkewBus>(),f,m)) && bool(robot.activate()),"skew fixture setup failed");
    bool seen=false;
    for(int i=0;i<5;++i){std::this_thread::sleep_for(std::chrono::milliseconds(5));auto out=robot.cycle();check(bool(out),"skew fixture cycle failed");
      if(out->new_measurement) {seen=true;check(std::abs(out->observer_state.pos[1]-2*out->observer_state.pos[0])<1e-9,"observer state not aligned");
        check(std::abs(evaluated.pos[1]-2*evaluated.pos[0])<1e-9,"model used asynchronous positions");}}
    check(seen,"skewed feedback never became alignable");
  }
  std::cout<<"robot paths passed\n";
}
