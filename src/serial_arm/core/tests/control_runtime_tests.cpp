#include "serial_arm/runtime/control_runtime.hpp"
#include "serial_arm/runtime/joint_motion.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace serial_arm;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}

struct RuntimeBus final:MotorBus {
    HardwareCapabilities info;
    ActuatorState state;
    bool fail_read{false};
    int writes{0};
    explicit RuntimeBus(const RobotCfg& cfg) {
        const auto n=cfg.joint_names.size();
        for(const auto& name:cfg.joint_names)info.push_back({name,-100,100,100,100,5000,50,name});
        JointActuatorMapper mapper;require(bool(mapper.configure(cfg.mapper)),"mapper");
        JointCtrlCmd neutral{cfg.shutdown.park_pos,JointVector(n,0),JointVector(n,0),JointVector(n,0),JointVector(n,0)};
        auto command=mapper.to_actuator_cmd(neutral);require(bool(command),"neutral mapping");
        state.pos=command->pos;state.vel.assign(n,0);state.tor.assign(n,0);
        state.online.assign(n,1);state.enabled.assign(n,1);state.err_code.assign(n,0);
    }
    tl::expected<void,MotorBusErr> configure(const std::string&)override{return {};}
    tl::expected<void,MotorBusErr> connect()override{return {};}
    tl::expected<void,MotorBusErr> activate()override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override{
        if(fail_read){fail_read=false;return tl::make_unexpected(MotorBusErr::READ_FAILED);}return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd&)override{++writes;return {};}
    tl::expected<void,MotorBusErr> stop()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{return {};}
    tl::expected<void,MotorBusErr> recover()override{return {};}
    void cleanup()noexcept override{}
    const HardwareCapabilities& capabilities()const noexcept override{return info;}
    std::size_t size()const noexcept override{return info.size();}
};

RobotCfg fixture() {
    HardwareCapabilities caps;
    for(int i=1;i<=6;++i)caps.push_back({"motor"+std::to_string(i),-100,100,100,100,5000,50,"joint"+std::to_string(i)});
    auto cfg=load_robot_cfg(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/core/gray.yaml",caps);
    require(bool(cfg),"runtime fixture");
    cfg->runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;
    cfg->runtime.write_enabled=true;cfg->capability.admittance.enabled=false;
    return *cfg;
}

void mailbox_and_fault() {
    auto cfg=fixture();auto bus=std::make_unique<RuntimeBus>(cfg);auto* raw=bus.get();
    Robot robot;ControlRuntime runtime;
    require(bool(robot.configure(cfg,std::move(bus))),"configure");require(bool(robot.activate()),"activate");
    const auto n=cfg.joint_names.size();auto reference=JointPosVelCmd{cfg.shutdown.park_pos,JointVector(n,0)};
    require(!runtime.submit_reference(robot,reference,Robot::Clock::now(),MotionSource::STREAM),"hold rejects stream");
    require(bool(runtime.set_impedance_mode(robot,JointImpedanceMode::RIGID_TRACKING)),"tracking");
    auto now=Robot::Clock::now();reference.pos[0]=.001;
    require(bool(runtime.submit_reference(robot,reference,now,MotionSource::STREAM)),"first source acquires motion");
    const auto busy=runtime.submit_reference(robot,reference,now,MotionSource::CALIBRATION);
    require(!busy && busy.error().code==RobotErr::MOTION_SOURCE_BUSY,"another source cannot replace motion and reports ownership conflict");
    reference.pos[0]=.002;
    require(bool(runtime.submit_reference(robot,reference,now,MotionSource::STREAM)),"owner can replace pending frame");
    int writes=raw->writes;
    auto tick=runtime.step(robot,Robot::Clock::now());
    require(tick.cycle.has_value()&&!tick.fault,"common cycle succeeds");
    require(raw->writes==writes+1,"one accepted frame produces one hardware write");
    require(std::abs(tick.cycle->nominal_ref_pos[0]-.002)<1e-12,"latest reference wins within its owner");
    require(runtime.snapshot().sequence==tick.sequence && tick.monotonic_ns>0,"consistent snapshot");
    require(bool(runtime.set_impedance_mode(robot,JointImpedanceMode::COMPLIANT_DRAG)),"drag");
    require(robot.is_admittance_suspended(),"drag explicitly selects native execution");
    require(bool(runtime.set_impedance_mode(robot,JointImpedanceMode::RIGID_TRACKING)),"leave drag");
    require(!robot.is_admittance_suspended(),"leave drag restores interaction suspension");
    require(runtime.source()==MotionSource::NONE,"mode transition revokes old reference owner");
    require(bool(runtime.submit_reference(robot,reference,Robot::Clock::now(),MotionSource::CALIBRATION)),"new task acquires source");
    raw->fail_read=true;
    tick=runtime.step(robot,Robot::Clock::now());
    require(tick.fault.has_value()&&tick.state==RobotState::FAULT,"read failure reaches common fault branch");
    require(tick.fault_hold_maintained,"common executor maintains fault hold");
    require(runtime.source()==MotionSource::NONE,"fault revokes task source");
    require(!runtime.submit_reference(robot,reference,Robot::Clock::now(),MotionSource::CALIBRATION),"fault rejects late task frame");
    require(bool(robot.force_deactivate()),"cleanup");
}

void shared_progression() {
    JointVector q{0},v{0},target{.2},vmax{.5},amax{1};
    bool complete=false;
    for(int i=0;i<1000 && !complete;++i) {
        const double previous=v[0];
        complete=advance_joint_reference(q,v,target,vmax,amax,1,.005);
        require(std::abs(v[0])<=.5+1e-12,"shared reference velocity bound");
        require(std::abs(v[0]-previous)<=.005+1e-12,"shared reference acceleration bound");
    }
    require(complete&&std::abs(q[0]-.2)<1e-9,"shared reference reaches goal");
    q={0};v={.1};target={1};
    advance_joint_reference(q,v,target,vmax,amax,0,.005);
    require(std::abs(v[0]-.095)<1e-12,"zero speed scale performs bounded park braking");
    bool rejected=false;try{advance_joint_reference(q,v,target,vmax,amax,1,0);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"reference rejects invalid dt");
}

void executor_ownership() {
    auto cfg=fixture();Robot robot;ControlRuntime runtime;
    require(bool(robot.configure(cfg,std::make_unique<RuntimeBus>(cfg))),"executor configure");
    require(bool(robot.activate()),"executor activate");
    std::atomic<bool> running{true},entered{false};
    RuntimeHooks hooks;hooks.running=[&]{return running.load();};
    hooks.before_cycle=[&](auto,double){entered.store(true);return true;};
    std::thread worker([&]{runtime.run(robot,200,.02,hooks);});
    for(int i=0;i<100&&!entered.load();++i)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    bool rejected=false;
    try{runtime.run(robot,200,.02,hooks);}catch(const std::logic_error&){rejected=true;}
    bool external_rejected=false;
    try{runtime.step(robot);}catch(const std::logic_error&){external_rejected=true;}
    running.store(false);worker.join();
    require(entered.load()&&rejected&&external_rejected,"same runtime rejects a second scheduler and external tick");
    require(runtime.snapshot().sequence>0,"shared scheduler publishes snapshots");
    require(bool(robot.deactivate()),"executor deactivate");
}

void point_motion_lifecycle() {
    JointMotionTracker tracker;
    const auto id=tracker.begin({0.01,0.01},{"revolute","prismatic"});
    require(id>0&&tracker.status().state=="running","acceptance is a running request, never completion");
    require(tracker.status().units==std::vector<std::string>({"rad","m"}),"joint type units stay explicit");
    RobotCycleOutput cycle;cycle.feedback_age_ms=1;
    cycle.actuator_state.feedback_timing_valid={1,1};
    cycle.joint_state.pos={0,0};cycle.joint_state.vel={0,0};
    auto observe=[&](int sample){cycle.actuator_state.feedback_received_ns={1000000000ull+10000000ull*sample,1000000000ull+10000000ull*sample};tracker.update(cycle,50);};
    for(int i=1;i<=30;++i)observe(i);
    require(tracker.status().state=="running","a stationary 10 mrad or 10 mm error cannot complete");
    cycle.joint_state.pos={0.01,0.01};
    for(int i=31;i<=60;++i)observe(i);
    require(tracker.status().state=="running","measured target before reference ends cannot complete");
    tracker.reference_complete(true);
    cycle.joint_state.pos[1]=0.009;
    for(int i=61;i<=90;++i)observe(i);
    require(tracker.status().state=="running","1 mm prismatic error is not within arrival tolerance");
    cycle.joint_state.pos[1]=0.01;observe(91);
    for(int i=0;i<100;++i)tracker.update(cycle,50);
    require(tracker.status().state=="running","repeated feedback cannot accrue settling time");
    cycle.feedback_age_ms=51;observe(92);cycle.feedback_age_ms=1;
    require(tracker.status().settled_s==0,"stale feedback breaks settling");
    for(int i=93;i<102;++i)observe(i);
    require(tracker.status().state=="running","target must remain stable for 150 ms");
    for(int i=102;i<=110;++i)observe(i);
    require(tracker.status().state=="completed","fresh stable arrival completes the original request");
    const auto next=tracker.begin({0.02},{"revolute"});
    require(next>id&&tracker.status().state=="running","replacement has a unique motion id");
    tracker.finish("interrupted","hold");
    require(tracker.status().state=="interrupted","hold interrupts without reporting completion");
    tracker.begin({0.03},{"revolute"});tracker.finish("failed","feedback timeout");
    tracker.finish("interrupted","clear old command");
    require(tracker.status().state=="failed","cleanup preserves the failure result");
    for(const auto* direction:{"forward","reverse","return"}) {
        const auto plan=teach_replay_plan(direction,30);
        require(plan.total_passes==1&&plan.cycles==1,"single-direction replay is exactly one pass");
    }
    require(teach_replay_plan("cycle",30).total_passes==60,"cycle N means N complete round trips");
    const auto contact=teach_replay_plan("contact_cycle",3);
    require(contact.total_passes==6&&contact.first_reverse,"contact cycle starts safe-to-contact and ends safe");
}

int main(){try{mailbox_and_fault();shared_progression();executor_ownership();point_motion_lifecycle();std::cout<<"control runtime contracts passed\n";return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
