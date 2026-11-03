#include "serial_arm/robot.hpp"
#include "serial_arm/core/reference_governor.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace serial_arm;
namespace {
void check(bool value,const char* message) { if(!value)throw std::runtime_error(message); }
void near(double a,double b,double tolerance,const char* message) { check(std::abs(a-b)<=tolerance,message); }
JointLimitCfg limits(std::size_t n) {
    return {std::vector<std::uint8_t>(n,1),JointVector(n,-5),JointVector(n,5),JointVector(n,10),
        JointVector(n,1000),JointVector(n,100),JointVector(n,1000),JointVector(n,50),JointVector(n,0)};
}
ReferenceGovernorInput request(std::size_t n) {
    ReferenceGovernorInput input;
    input.requested={JointVector(n,0),JointVector(n,0),JointVector(n,0),JointVector(n,0),JointVector(n,0)};
    input.previous=input.requested;
    input.measured={JointVector(n,0),JointVector(n,0),JointVector(n,0)};
    input.requested_acceleration.assign(n,0);input.limits=limits(n);input.dt=.01;
    return input;
}
void governor_constraints() {
    auto in=request(1);in.model_admittance=true;in.mass_matrix={{1}};
    in.requested.pos={1};in.requested.vel={100};in.requested.tor={10000};in.requested_acceleration={10000};
    in.limits.max_acc={10};in.limits.max_vel={1};in.limits.max_effort={20};
    auto out=govern_reference(in);check(bool(out),"motion projection is feasible");
    check(out->reference_limited[0]&&out->torque_limited[0],"projection publishes its reasons");
    check(std::abs(out->acceleration[0])<=10.000001,"actual accepted acceleration is bounded");
    near(out->accepted.pos[0],in.previous.pos[0]+in.dt*out->accepted.vel[0],1e-10,"accepted q/v are one motion");
    near(out->accepted.tor[0],out->acceleration[0],1e-7,"feedforward follows accepted acceleration");

    in=request(1);in.requested.pos={.05};in.requested.vel={5};in.requested.kp={100};in.limits.max_effort={1};
    out=govern_reference(in);check(bool(out),"PD budget projection");
    check(out->predicted_total_torque[0]<=1.000001&&out->torque_limited[0],"budget includes local PD");
    check(out->accepted.pos[0]<=.01000001,"reference yields before PD exceeds torque budget");

    in=request(2);in.model_admittance=true;in.mass_matrix={{2,1},{1,2}};
    in.requested.pos={.02,-.01};in.requested.vel={2,-1};in.requested_acceleration={200,-100};
    in.requested.tor={300,0};in.limits.max_effort={1,1};
    out=govern_reference(in);check(bool(out),"coupled inertia projection");
    for(std::size_t i=0;i<2;++i) {
        const double force=in.mass_matrix[i][0]*out->acceleration[0]+in.mass_matrix[i][1]*out->acceleration[1];
        near(out->accepted.tor[i],force,1e-7,"full inertia matrix is retained");
        check(std::abs(force)<=1.000001,"coupled axes obey torque limits");
    }
    in=request(1);in.model_admittance=true;in.mass_matrix={{1}};in.requested.tor={2};
    in.limits.max_effort={1};in.limits.max_acc={.1};
    check(!govern_reference(in),"infeasible support torque is rejected rather than silently clipped");
}

void dissipative_integrator_and_update() {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg{1,{1},{.001},{1},{1},{10},{100}};
    check(bool(controller.configure(cfg)),"small inertia configure");
    check(bool(controller.accept({.1},{0})),"initial virtual state");
    double energy=.005;
    for(int i=0;i<1000;++i) {
        auto value=controller.update({{0},.005,{},{},{},{}});check(bool(value),"implicit step");
        const double next=.5*.001*value->delta_q_dot[0]*value->delta_q_dot[0]+.5*value->delta_q[0]*value->delta_q[0];
        check(next<=energy+1e-12,"unforced virtual energy decays even beyond old Euler stability range");energy=next;
    }
    check(bool(controller.accept({.12},{.04})),"set accepted offset");
    cfg.mass={1};cfg.damping={5};cfg.stiffness={0};
    check(bool(controller.reconfigure(cfg)),"zero stiffness and online ramp");
    auto held=controller.hold({-10},{10});check(bool(held),"read unchanged reference");
    near(held->delta_q[0],.12,1e-12,"parameter update preserves displacement");
    for(int i=0;i<80;++i)check(bool(controller.update({{0},.005,{},{},{},{}})),"parameter ramp step");
    near(controller.parameter_update_progress(),1,1e-10,"parameter ramp completes");

    GeneralizedMomentumObserver observer;
    check(bool(observer.configure({1,{1000},{0}})),"high observer gain configure");
    GeneralizedMomentumInput input{{-1},{0},{0},{{1}},{0},.005,{}};
    double last=0;
    for(int i=0;i<100;++i) {
        auto estimate=observer.update(input);check(bool(estimate),"stable observer step");
        last=estimate->tau_ext_hat[0];check(std::isfinite(last)&&std::abs(last)<2,"observer does not diverge for gain times dt greater than two");
    }
    near(last,1,1e-8,"momentum observer preserves physical force sign");
}

struct PlantBus final:MotorBus {
    HardwareCapabilities caps;
    ActuatorState state;
    ActuatorCtrlCmd command;
    JointVector force,feedback_bias;
    std::uint64_t stamp{0};
    bool frozen{false};
    explicit PlantBus(const RobotCfg& cfg) {
        const auto n=cfg.joint_names.size();
        for(const auto& name:cfg.joint_names)caps.push_back({name,-100,100,100,100,5000,50,name});
        state.pos.assign(n,0);state.vel.assign(n,0);state.tor.assign(n,0);
        state.online.assign(n,1);state.enabled.assign(n,1);state.err_code.assign(n,0);
        state.feedback_timing_valid.assign(n,1);force.assign(n,0);feedback_bias.assign(n,0);
        command={state.pos,state.vel,state.tor,state.tor,state.tor};
        stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count()-20000000;
    }
    tl::expected<void,MotorBusErr> configure(const std::string&)override{return {};}
    tl::expected<void,MotorBusErr> connect()override{return {};}
    tl::expected<void,MotorBusErr> activate()override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override {
        if(!frozen)stamp+=1000;
        state.feedback_received_ns.assign(caps.size(),stamp);return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& value)override{command=value;return {};}
    tl::expected<void,MotorBusErr> stop()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{return {};}
    tl::expected<void,MotorBusErr> recover()override{return {};}
    void cleanup()noexcept override{}
    const HardwareCapabilities& capabilities()const noexcept override{return caps;}
    std::size_t size()const noexcept override{return caps.size();}
};
RobotCfg fixture() {
    HardwareCapabilities caps;
    for(int i=1;i<=6;++i)caps.push_back({"motor"+std::to_string(i),-100,100,100,100,5000,50,"joint"+std::to_string(i)});
    auto loaded=load_robot_cfg(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/core/gray.yaml",caps);
    check(bool(loaded),"architecture fixture load");auto cfg=*loaded;const auto n=cfg.joint_names.size();
    cfg.runtime.write_enabled=true;cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    cfg.safety_limit_source.reset();cfg.safety.limits=limits(n);cfg.safety.limits.max_acc.assign(n,100);
    cfg.safety.state_timeout_s=1;cfg.safety.max_dt_s=.1;
    cfg.ctrller.rigid_hold_gains={JointVector(n,50),JointVector(n,5)};
    auto& adm=cfg.capability.admittance;adm.enabled=true;adm.reference_behavior=ReferenceBehavior::MODEL_ADMITTANCE;
    adm.observer.mode=AdmittanceObserverMode::MOMENTUM;adm.observer.momentum_gain.assign(n,50);
    adm.calibration.torque_bias.assign(n,0);adm.calibration.torque_threshold.assign(n,0);adm.calibration.friction.enabled=false;
    adm.controller.mass.assign(n,1);adm.controller.damping.assign(n,5.5);adm.controller.stiffness.assign(n,15);
    adm.controller.max_delta_q.assign(n,1);adm.controller.max_delta_q_dot.assign(n,2);return cfg;
}
void measured_snapshot_and_bias() {
    auto cfg=fixture();const auto n=cfg.joint_names.size();
    auto bus=std::make_unique<PlantBus>(cfg);auto* raw=bus.get();int snapshots=0,forward_calls=0;
    for(std::size_t i=0;i<n;++i) {cfg.capability.admittance.calibration.torque_bias[i]=-.2;raw->state.tor[i]=1.2;}
    Robot robot;
    ModelFeedforwardFn ff=[&](ModelFeedforwardMode mode,const JointState&,const JointVector&,const JointVector&,double) {
        ++forward_calls;return tl::expected<JointVector,ModelFeedforwardErr>(JointVector(n,mode==ModelFeedforwardMode::NONE?0:1));
    };
    InteractionModelStateFn model=[&](const JointState&,double) {
        ++snapshots;MeasuredDynamicsSnapshot snapshot;snapshot.gravity.assign(n,1);snapshot.coriolis.assign(n,0);
        snapshot.mass_matrix.assign(n,JointVector(n,0));for(std::size_t i=0;i<n;++i)snapshot.mass_matrix[i][i]=1;
        return tl::expected<InteractionModelState,ModelFeedforwardErr>(snapshot);
    };
    check(bool(robot.configure(cfg,std::move(bus),ff,model)),"model reference configure");
    check(bool(robot.activate()),"model reference activate");
    auto first=robot.cycle();check(bool(first),"cold MOMENTUM start succeeds without measured acceleration");
    check(!first->joint_acc_valid&&first->observer_valid,"MOMENTUM does not inherit FULL_ID acceleration requirement");
    check(snapshots==1&&forward_calls==0,"measured model is independent of actuation callback side effects");
    near(first->joint_cmd.tor[0],1,1e-7,"measurement bias never changes actuation feedforward");
    raw->frozen=true;
    auto repeated=robot.cycle();check(bool(repeated)&&!repeated->new_measurement,"cached frame is not a new force observation");
    near(repeated->tau_ext_hat[0],first->tau_ext_hat[0],1e-10,"cached force does not reintegrate observer");
    auto changed=robot.get_admittance_cfg();changed.controller.damping.assign(n,8);
    check(bool(robot.set_admittance_cfg(changed)),"active parameter update succeeds");
    check(!robot.set_reference_behavior(ReferenceBehavior::NATIVE_IMPEDANCE),"active behavior switch is rejected");
    robot.set_admittance_suspended(true);raw->frozen=false;
    auto shadow=robot.cycle();check(bool(shadow)&&shadow->observer_valid,"suspending MDK retains valid shadow observation");
    near(shadow->tau_ext_hat[0],first->tau_ext_hat[0],1e-10,"suspension preserves observer state");
    check(bool(robot.set_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG)),"drag handover");
    auto drag=robot.cycle();check(bool(drag)&&drag->observer_valid,"native drag retains shadow observation");
}

void nominal_step_is_not_external_compliance() {
    auto cfg=fixture();const auto n=cfg.joint_names.size();cfg.safety.limits.max_acc.assign(n,1);
    auto bus=std::make_unique<PlantBus>(cfg);
    Robot robot;
    InteractionModelStateFn model=[n](const JointState&,double) {
        MeasuredDynamicsSnapshot snapshot;snapshot.gravity.assign(n,0);snapshot.coriolis.assign(n,0);
        snapshot.mass_matrix.assign(n,JointVector(n,0));for(std::size_t i=0;i<n;++i)snapshot.mass_matrix[i][i]=1;
        return tl::expected<InteractionModelState,ModelFeedforwardErr>(snapshot);
    };
    ModelFeedforwardFn ff=[n](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double) {
        return tl::expected<JointVector,ModelFeedforwardErr>(JointVector(n,0));
    };
    check(bool(robot.configure(cfg,std::move(bus),ff,model)),"nominal step configure");
    check(bool(robot.activate()),"nominal step activate");
    check(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING)),"tracking handover");
    check(bool(robot.set_cmd(JointPosCmd{JointVector(n,.5)})),"task reference step");
    auto step=robot.cycle();check(bool(step),"nominal step is governed");
    check(step->nominal_ref_pos[0]>0 && step->nominal_ref_pos[0]<.01,"nominal gets its own motion projection");
    near(step->virtual_ref_pos[0],step->nominal_ref_pos[0],1e-10,"task step does not create a fictitious force offset");
    near(step->accepted_ref_pos[0],step->nominal_ref_pos[0],1e-10,"feasible zero-force accepted reference follows nominal");
    near(step->delta_q[0],0,1e-10,"accepted MDK state uses governed nominal baseline");
}

void model_reference_comparison() {
    // The same physical plant, force, virtual MDK and local servo gains isolate
    // the architectural change from identification and sensor quality effects.
    const char* path=std::getenv("SERIALARM_INTERACTION_DEMO");std::ofstream csv;
    if(path) {csv.open(path);csv<<"time_s,external_nm,virtual_rad,legacy_rad,model_reference_rad,legacy_torque_nm,model_reference_torque_nm\n";}
    JointAdmittanceController desired;check(bool(desired.configure({1,{1},{1},{5.5},{15},{1},{2}})),"demo model");
    double q_old=0,v_old=0,q_new=0,v_new=0,peak_error_new=0,plateau_old=0,plateau_new=0;
    constexpr double h=.001,kp=20,kd=8;
    for(int i=0;i<8000;++i) {
        const double t=i*h,force=t>=1&&t<4?1.0:0.0;
        auto virtual_motion=desired.update({{force},h,{},{},{},{}});check(bool(virtual_motion),"demo virtual step");
        const double qv=virtual_motion->delta_q[0],vv=virtual_motion->delta_q_dot[0],av=virtual_motion->delta_q_ddot[0];
        const double old_torque=kp*(qv-q_old)+kd*(vv-v_old);
        const double new_torque=kp*(qv-q_new)+kd*(vv-v_new)+av-force;
        v_old+=h*(old_torque+force);q_old+=h*v_old;
        v_new+=h*(new_torque+force);q_new+=h*v_new;
        peak_error_new=std::max(peak_error_new,std::abs(q_new-qv));
        if(i==3999) {plateau_old=std::abs(q_old-qv);plateau_new=std::abs(q_new-qv);}
        if(csv&&i%5==0)csv<<t<<','<<force<<','<<qv<<','<<q_old<<','<<q_new<<','<<old_torque<<','<<new_torque<<'\n';
    }
    check(plateau_old>.04,"legacy extra physical compliance is reproduced");
    check(plateau_new<.001&&peak_error_new<.002,"model-reference torque composition follows virtual admittance");
    std::cout<<"offline plateau tracking error: legacy="<<plateau_old<<" model_reference="<<plateau_new<<" rad\n";
}
}
int main() {
    try {
        governor_constraints();dissipative_integrator_and_update();measured_snapshot_and_bias();nominal_step_is_not_external_compliance();model_reference_comparison();
        std::cout<<"interaction architecture contracts passed\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
