#include "serial_arm/robot.hpp"
#include "serial_arm/config/limit_resolver.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/gravity_correction.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include "serial_arm/transport/serial_port.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace serial_arm;
namespace fs = std::filesystem;
void require(bool result, const char* message) { if(!result) throw std::runtime_error(message); }
struct FakeBus final : MotorBus {
    HardwareCapabilities caps;
    ActuatorState state;
    bool fail_read{false}, fail_write{false}, fail_activate{false}, fail_disable{false};
    int disables{0}, stops{0}, writes{0};
    double peak_torque{0};
    ActuatorCtrlCmd last_command;
    explicit FakeBus(const RobotCfg& cfg) {
        const auto n = cfg.joint_names.size();
        caps.resize(n);
        for(std::size_t i=0;i<n;++i) {
            caps[i] = {"motor"+std::to_string(i), -100,100,100,100,5000,50,cfg.joint_names[i]};
        }
        JointActuatorMapper mapper;
        require(bool(mapper.configure(cfg.mapper)), "fake mapper configure");
        JointCtrlCmd neutral;
        neutral.pos=cfg.shutdown.park_pos;
        neutral.vel.assign(n,0);neutral.tor.assign(n,0);neutral.kp.assign(n,0);neutral.kd.assign(n,0);
        const auto actual=mapper.to_actuator_cmd(neutral);
        require(bool(actual),"fake neutral map");
        state.pos=actual->pos;state.vel.assign(n,0);state.tor.assign(n,0);
        state.online.assign(n,1);state.enabled.assign(n,1);state.err_code.assign(n,0);
    }
    tl::expected<void,MotorBusErr> configure(const std::string&) override { return {}; }
    tl::expected<void,MotorBusErr> connect() override { return {}; }
    tl::expected<void,MotorBusErr> activate() override {
        if(fail_activate) return tl::make_unexpected(MotorBusErr::ENABLE_FAILED);
        return {};
    }
    tl::expected<ActuatorState,MotorBusErr> read() override {
        if(fail_read) { fail_read=false;return tl::make_unexpected(MotorBusErr::READ_FAILED); }
        return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        ++writes;
        last_command=cmd;
        if(fail_write) return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
        for(double torque:cmd.tor) peak_torque=std::max(peak_torque,std::abs(torque));
        return {};
    }
    tl::expected<void,MotorBusErr> stop() override { ++stops; return {}; }
    tl::expected<void,MotorBusErr> deactivate() override {
        ++disables;
        if(fail_disable) return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
        return {};
    }
    tl::expected<void,MotorBusErr> recover() override { return {}; }
    void cleanup() noexcept override {}
    const HardwareCapabilities& capabilities() const noexcept override { return caps; }
    std::size_t size() const noexcept override { return caps.size(); }
};
RobotCfg fixture() {
    HardwareCapabilities caps;
    for(int i=1;i<=6;++i) caps.push_back({"motor"+std::to_string(i),-100,100,100,100,5000,50,"joint"+std::to_string(i)});
    const auto result=load_robot_cfg(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/core/gray.yaml",caps);
    if(!result) throw std::runtime_error(result.error().message);
    auto cfg=*result;
    cfg.runtime.write_enabled=true;cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;
    cfg.capability.admittance.enabled=false;
    return cfg;
}
void limits_and_identity() {
    RobotModelInfo model;model.joint_names={"axis"};model.joint_limits={{"axis",true,-10,10,20,100}};
    JointActuatorMapCfg map;map.joints_count=1;map.pos_ratio={2};map.tor_ratio={2};map.direction={-1};
    map.joint_zero_offset={1};map.actuator_zero_offset={2};
    HardwareCapabilities caps={{"motor",-4,8,6,10,100,5,"axis"}};
    SafetyPolicyCfg policy;policy.max_acc={1};policy.cmd_vel_scale=.5;policy.state_vel_scale=1;
    policy.max_dt_s=.02;policy.state_timeout_s=.05;policy.cmd_timeout_s=.1;
    auto result=LimitResolver{}.resolve(model,map,caps,policy);
    require(bool(result),"mapped limits resolve");
    require(std::abs(result->joints[0].hard_min_pos+2)<1e-12,"direction and offset lower limit");
    require(std::abs(result->joints[0].hard_max_pos-4)<1e-12,"direction and offset upper limit");
    require(std::abs(result->joints[0].max_cmd_vel-1.5)<1e-12,"hardware velocity intersection");
    caps[0].joint_name="wrong";
    require(!LimitResolver{}.resolve(model,map,caps,policy),"reject swapped joint identity");
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);bus->caps[0].joint_name="wrong";
    Robot robot;require(!robot.configure(cfg,std::move(bus)),"Robot rejects swapped identity");
}
void named_permutation() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();
    std::reverse(raw->caps.begin(),raw->caps.end());
    std::reverse(raw->state.pos.begin(),raw->state.pos.end());
    Robot robot;require(bool(robot.configure(cfg,std::move(bus))),"named permutation configure");
    require(bool(robot.activate()),"named permutation activate");
    for(std::size_t i=0;i<cfg.joint_names.size();++i)
        require(std::abs(robot.get_joint_state().pos[i]-cfg.shutdown.park_pos[i])<1e-12,"feedback follows joint identity");
    require(bool(robot.cycle()),"named permutation control cycle");
    for(std::size_t i=0;i<raw->state.pos.size();++i)
        require(std::abs(raw->last_command.pos[i]-raw->state.pos[i])<1e-12,"commands preserve physical actuator identity");
}
void fault_shutdown() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    require(bool(robot.configure(cfg,std::move(bus))),"robot configure");
    require(bool(robot.activate()),"robot activate");
    raw->fail_read=true;require(!robot.cycle(),"injected read failure");
    require(robot.is_fault_holding(),"read fault enters hold");
    raw->fail_write=true;require(!robot.maintain_fault_hold(),"injected hold write failure");
    require(raw->disables>0,"hold failure requests disable");
    require(!robot.is_fault_holding(),"failed hold cannot remain active");
}
void failed_startup_disable_retry() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    require(bool(robot.configure(cfg,std::move(bus))),"startup retry configure");
    raw->fail_activate=true;raw->fail_disable=true;
    require(!robot.activate()&&robot.get_state()==RobotState::FAULT,"activation failure stays faulted");
    require(!robot.force_deactivate()&&robot.get_state()==RobotState::FAULT,"unacknowledged disable must not report INACTIVE");
    raw->fail_disable=false;
    require(bool(robot.force_deactivate())&&robot.get_state()==RobotState::INACTIVE,"disable can be retried after startup fault");
    raw->fail_activate=false;
    require(bool(robot.activate()),"enable after confirmed disable recovers");
}

void first_hold_limits() {
    auto cfg=fixture();cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    cfg.safety.limits.max_effort.assign(cfg.joint_names.size(),1);
    double torque=0;
    auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    const auto model=[&](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)
        ->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector(cfg.joint_names.size(),torque);};
    require(bool(robot.configure(cfg,std::move(bus),model)),"gravity robot configure");
    require(bool(robot.activate()),"gravity robot activate");
    torque=20;require(!robot.cycle(),"reject excessive feedforward");
    require(raw->peak_torque<=1,"first fault hold must not bypass effort limit");
    require(!robot.is_fault_holding(),"unsafe first hold rejected");
}
void continuous_and_correction(const fs::path& temp) {
    const auto urdf=temp/"continuous.urdf";
    std::ofstream(urdf)<<R"(<robot name="test"><link name="base"/><link name="tip"><inertial><origin xyz="1 0 0"/><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link><joint name="spin" type="continuous"><parent link="base"/><child link="tip"/><axis xyz="0 1 0"/><limit effort="10" velocity="2"/></joint></robot>)";
    DynamicsCfg cfg;cfg.urdf_path=urdf.string();cfg.joint_names={"spin"};cfg.base_frame="base";cfg.tool_frame="tip";cfg.gravity_scale={1};
    Dynamics dynamics;require(bool(dynamics.configure(cfg)),"continuous configure");
    JointState state;state.pos={0};state.vel={0};state.tor={0};
    require(bool(dynamics.update(state,{0},{0})),"continuous update");
    const auto original=dynamics.get_state().gravity;
    state.pos={2*std::acos(-1.)};require(bool(dynamics.update(state,{0},{0})),"continuous full turn");
    require(std::abs(dynamics.get_state().gravity[0]-original[0])<1e-8,"continuous periodic gravity");
    require(bool(dynamics.get_gravity_regression(state.pos)),"continuous gravity regressor");
    const auto correction=temp/"gravity.yaml";
    std::ofstream(correction)<<"schema: serial_arm_gravity_correction\nurdf_fingerprint: '"<<model_calibration_file_fingerprint(urdf.string())
        <<"'\njoint_names: [spin]\nstatic_pass: true\nfirst_moments: {tip: [2, 0, 0]}\nresidual: {torque_bias: [0.2], torque_threshold: [0.1]}\n";
    cfg.gravity_correction_path=correction.string();Dynamics corrected;
    require(bool(corrected.configure(cfg)),"valid correction configure");
    state.pos={0};require(bool(corrected.update(state,{0},{0})),"corrected update");
    require(std::abs(corrected.get_state().effective_gravity[0]-2*original[0])<1e-8,"configured correction is applied");
    RobotCfg runtime;runtime.joint_names={"spin"};runtime.dynamics=cfg;
    runtime.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    runtime.capability.admittance.observer.mode=AdmittanceObserverMode::MOMENTUM;
    Dynamics shared;require(bool(configure_robot_dynamics(runtime,shared)),"shared runtime correction configure");
    require(runtime.capability.admittance.calibration.torque_bias==JointVector{.2},"shared residual bias");
    require(runtime.capability.admittance.calibration.torque_threshold==JointVector{.1},"shared residual threshold");
    cfg.gravity_correction_path=(temp/"missing.yaml").string();Dynamics invalid;
    require(!invalid.configure(cfg),"missing correction cannot be ignored");
}
void profile_error(const fs::path& temp) {
    const auto file=temp/"profiles.yaml";std::ofstream(file)<<"profiles:\n  arm:\n    core:\n      package: [wrong, type]\n      config: core.yaml\n    hardware: {}\n";
    RobotProfileLoadOptions options;options.profile_file=file.string();
    require(!load_robot_profile_core("arm",options),"invalid Profile returns expected error");
}
void serial_exclusion() {
    const int master=::posix_openpt(O_RDWR|O_NOCTTY);
    require(master>=0 && ::grantpt(master)==0 && ::unlockpt(master)==0,"create PTY");
    const std::string device=::ptsname(master);
    transport::SerialPort first(device);
    const auto child=::fork();require(child>=0,"fork serial exclusion");
    if(child==0) {
        try { transport::SerialPort second(device); ::_exit(1); }
        catch(...) { ::_exit(0); }
    }
    int status=0;::waitpid(child,&status,0);
    require(WIFEXITED(status)&&WEXITSTATUS(status)==0,"second process must not open owned serial port");
    first.close();transport::SerialPort reopened(device);reopened.close();::close(master);
}

void recording_roundtrip(const fs::path& temp) {
    ModelCalibrationMetadata metadata;metadata.task_id="timing-test";metadata.joint_names={"axis"};
    ModelCalibrationRecorder recorder;const auto folder=temp/"recording";
    require(bool(recorder.start(folder,metadata)),"start timing recorder");
    ModelCalibrationFrame frame;frame.position={0};frame.velocity={.1};frame.torque={0};frame.acceleration={2};
    frame.reference_position={0};frame.reference_velocity={0};frame.feedback_age_ms=20;frame.valid=true;
    frame.phase="static_reverse";frame.acceleration_source="robot_feedback_velocity_filter";
    frame.feedback_received_ns={12345678901234567ULL};frame.feedback_timing_valid={1};
    recorder.push(frame);recorder.stop();
    const auto loaded=load_model_calibration_frames(folder.string(),1);
    require(bool(loaded)&&loaded->size()==1,"read extended timestamp CSV");
    require(loaded->front().feedback_received_ns==frame.feedback_received_ns,"nanosecond timestamp roundtrip is exact");
    require(loaded->front().feedback_timing_valid==frame.feedback_timing_valid,"timing validity roundtrip");
}
void sampled_acceleration() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    const auto n=cfg.joint_names.size();
    auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()-30000000;
    raw->state.feedback_received_ns.assign(n,stamp);
    raw->state.feedback_timing_valid.assign(n,1);
    require(bool(robot.configure(cfg,std::move(bus))),"timestamped configure");
    require(bool(robot.activate()),"timestamped activate");
    auto first=robot.cycle();require(bool(first)&&!first->joint_acc_valid,"first acceleration invalid");
    for(auto& value:raw->state.feedback_received_ns)value+=10000000;
    raw->state.vel.assign(n,.1);
    auto second=robot.cycle();require(bool(second)&&second->joint_acc_valid,"new receive timestamps are valid");
    require(std::abs(second->joint_acc[0]-2)<1e-6,"differentiate reception interval not decode interval");
    auto repeated=robot.cycle();require(bool(repeated)&&!repeated->joint_acc_valid,"repeated feedback invalid");
    require(std::abs(repeated->joint_acc[0]-2)<1e-6,"repeated feedback keeps filter memory");
    for(auto& value:raw->state.feedback_received_ns)value+=1000000;
    raw->state.feedback_timing_valid.assign(n,0);
    raw->state.vel.assign(n,.2);
    auto burst=robot.cycle();require(bool(burst)&&!burst->joint_acc_valid,"transport burst cannot produce trusted qdd");
    raw->state.feedback_received_ns.assign(n,stamp-40000000);
    require(!robot.cycle(),"old receive timestamp must fail freshness after delayed decode");
}
void full_id_validity() {
    InteractionControllerCfg cfg;cfg.enabled=true;cfg.residual.joints_count=1;
    cfg.external_torque.joints_count=1;
    cfg.admittance={1,{1},{1},{1},{1},{1},{1}};
    InteractionController observer;require(bool(observer.configure(cfg)),"FULL_ID observer configure");
    JointVector measured{0},model{2};JointCtrlCmd nominal{{0},{0},{0},{1},{1}};
    InteractionInput input{measured,model,nominal,.01,{-1},{1},{-1},{1},{0},{},true};
    auto valid=observer.update(input);require(bool(valid)&&valid->state.valid,"valid FULL_ID observation");
    const double held=valid->delta_q[0];require(held>0,"FULL_ID initial displacement");
    input.full_id_acceleration_valid=false;model[0]=std::numeric_limits<double>::quiet_NaN();
    auto skipped=observer.update(input);require(bool(skipped)&&!skipped->state.valid,"invalid acceleration skips FULL_ID residual");
    require(skipped->full_id_residual_raw.empty()&&skipped->tau_ext_hat.empty(),"invalid residual is not published as observed force");
    require(skipped->delta_q[0]==held&&skipped->delta_q_dot[0]==0,"invalid sample holds displacement without integrating");
    input.full_id_acceleration_valid=true;model[0]=0;
    auto resumed=observer.update(input);require(bool(resumed)&&resumed->state.valid,"FULL_ID resumes with a fresh sample");
    require(resumed->tau_ext_hat[0]==0,"FULL_ID recovery has no stale force");
    cfg.observer_mode=AdmittanceObserverMode::MOMENTUM;cfg.momentum.joints_count=1;cfg.momentum.gain={10};
    InteractionController momentum;require(bool(momentum.configure(cfg)),"momentum configure");
    input.full_id_acceleration_valid=false;model.clear();
    input.momentum_input={{0},{0},{0},{{1}},{0},.01};
    auto independent=momentum.update(input);require(bool(independent)&&independent->state.valid,"MOMENTUM does not require qdd validity");
}
int main() {
    const auto temp=fs::temp_directory_path()/("serialarm-contract-"+std::to_string(::getpid()));fs::create_directories(temp);
    try {
        recording_roundtrip(temp);sampled_acceleration();full_id_validity();limits_and_identity();named_permutation();fault_shutdown();failed_startup_disable_retry();first_hold_limits();continuous_and_correction(temp);profile_error(temp);serial_exclusion();
        fs::remove_all(temp);std::cout<<"runtime contracts passed\n";return 0;
    } catch(const std::exception& error) {fs::remove_all(temp);std::cerr<<error.what()<<'\n';return 1;}
}
