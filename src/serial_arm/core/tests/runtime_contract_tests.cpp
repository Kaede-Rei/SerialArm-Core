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
#include <thread>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace serial_arm;
namespace fs = std::filesystem;
void require(bool result, const char* message) { if(!result) throw std::runtime_error(message); }
struct FakeBus final : MotorBus {
    HardwareCapabilities caps;
    HardwareCapabilities connected_caps;
    ActuatorState state;
    bool fail_read{false}, fail_write{false}, fail_activate{false}, fail_disable{false}, fresh_timestamps{false};
    int disables{0}, stops{0}, writes{0}, activations{0};
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
    tl::expected<void,MotorBusErr> connect() override {
        if(!connected_caps.empty()) caps=connected_caps;
        return {};
    }
    tl::expected<void,MotorBusErr> activate() override {
        ++activations;
        if(fail_activate) return tl::make_unexpected(MotorBusErr::ENABLE_FAILED);
        return {};
    }
    tl::expected<ActuatorState,MotorBusErr> read() override {
        if(fail_read) { fail_read=false;return tl::make_unexpected(MotorBusErr::READ_FAILED); }
        if(fresh_timestamps) {
            const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            state.feedback_received_ns.assign(size(),stamp);
        }
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

void activation_preserves_running_fault() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    require(bool(robot.configure(cfg,std::move(bus))),"fault retry configure");
    raw->fail_activate=true;
    require(!robot.activate(),"inject first activation failure");
    raw->fail_activate=false;
    require(bool(robot.activate()),"first activation failure remains retryable");
    require(bool(robot.cycle()),"complete running cycle");
    raw->fail_read=true;
    require(!robot.cycle()&&robot.is_fault_holding(),"inject running hold fault");
    for(unsigned i=0;i<3;++i) require(bool(robot.maintain_fault_hold()),"valid fault hold observation");
    const auto retried=robot.activate();
    require(!retried&&retried.error().code==RobotErr::FAULTED,"activate must reject a running fault");
    require(robot.get_state()==RobotState::FAULT&&robot.is_fault_holding(),"failed activate preserves fault latch");
    require(bool(robot.clear_fault())&&robot.get_state()==RobotState::ACTIVE,"explicit clear still restores hold");
}

void connected_limits_follow_policy() {
    auto original=fixture();FakeBus nominal(original);
    for(auto& cap:nominal.caps) cap.max_effort=2;
    auto loaded=load_robot_cfg(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/core/gray.yaml",nominal.caps);
    require(bool(loaded),"load provisional hardware limits");
    auto cfg=*loaded;cfg.runtime.write_enabled=true;cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;
    cfg.capability.admittance.enabled=false;
    auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();raw->caps=nominal.caps;
    raw->connected_caps=nominal.caps;
    for(auto& cap:raw->connected_caps) cap.max_effort=3;
    Robot robot;require(bool(robot.configure(cfg,std::move(bus))),"connected limits configure");
    require(bool(robot.activate()),"activate with actual wider encoding range");
    require(robot.get_config().safety.limits.max_effort[0]==3,"actual wider range replaces provisional default");
    require(bool(robot.force_deactivate()),"stop before range refresh");
    for(auto& cap:raw->connected_caps) cap.max_effort=.5;
    require(bool(robot.activate()),"activate with actual narrower encoding range");
    require(robot.get_config().safety.limits.max_effort[0]==.5,"actual narrower range reaches Core safety");
    require(bool(robot.force_deactivate()),"stop before range restores");
    for(auto& cap:raw->connected_caps) cap.max_effort=3;
    require(bool(robot.activate())&&robot.get_config().safety.limits.max_effort[0]==3,
        "reconnect resolves original policy rather than accumulated minima");
    cfg.safety.limits.max_effort[0]=1;
    auto edited=resolve_connected_safety_cfg(cfg,raw->connected_caps);
    require(bool(edited)&&edited->limits.max_effort[0]==1,"preserve explicit SDK safety edit");
    cfg=*loaded;
    cfg.safety_limit_source->policy.max_effort_override.assign(cfg.joint_names.size(),1);
    auto policy=resolve_connected_safety_cfg(cfg,raw->connected_caps);
    require(bool(policy)&&policy->limits.max_effort[0]==1,"explicit policy is not widened by real hardware");
    cfg.safety_limit_source.reset();
    auto manual=resolve_connected_safety_cfg(cfg,raw->connected_caps);
    require(bool(manual)&&manual->limits.max_effort[0]==2,"programmatic configuration remains an explicit ceiling");
    require(bool(robot.force_deactivate()),"disable before invalid capability check");
    const auto activations=raw->activations;
    raw->connected_caps[0].max_effort=0;
    require(!robot.activate()&&raw->activations==activations&&robot.get_state()==RobotState::INACTIVE,
        "invalid real limits must be rejected before enabling actuators");

    cfg=fixture();cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::GRAVITY;
    auto torque_bus=std::make_unique<FakeBus>(cfg);auto* torque_raw=torque_bus.get();
    torque_raw->connected_caps=torque_raw->caps;
    for(auto& cap:torque_raw->connected_caps) cap.max_effort=.5;
    const auto model=[&](ModelFeedforwardMode,const JointState&,const JointVector&,const JointVector&,double)
        ->tl::expected<JointVector,ModelFeedforwardErr>{return JointVector(cfg.joint_names.size(),1);};
    Robot torque_robot;require(bool(torque_robot.configure(cfg,std::move(torque_bus),model))&&bool(torque_robot.activate()),
        "real-range effort enforcement setup");
    const auto rejected=torque_robot.cycle();
    require(!rejected&&rejected.error().code==RobotErr::SAFETY_FAILED&&
        rejected.error().safety_fault.code==SafetyErr::CMD_EFFORT_LIMIT&&torque_raw->peak_torque<=.5,
        "Core must reject over-range model torque before a backend write failure");
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
    recorder.push(frame);
    bool active_snapshot_rejected=false;
    try{(void)recorder.frames();}catch(const std::logic_error&){active_snapshot_rejected=true;}
    require(active_snapshot_rejected,"active full-history snapshot must be rejected before blocking the producer");
    recorder.push(frame);recorder.stop();
    recorder.push(frame);
    const auto finished=recorder.status();
    require(finished.accepted_frames==2 && finished.written_frames==2 && !finished.write_failed,"stop drains accepted frames and rejects later producer writes");
    const auto loaded=load_model_calibration_frames(folder.string(),1);
    require(bool(loaded)&&loaded->size()==2,"read extended timestamp CSV");
    require(loaded->front().feedback_received_ns==frame.feedback_received_ns,"nanosecond timestamp roundtrip is exact");
    require(loaded->front().feedback_timing_valid==frame.feedback_timing_valid,"timing validity roundtrip");
}
void paired_static_recording(const fs::path& temp) {
    ModelCalibrationMetadata metadata;metadata.task_id="paired";metadata.joint_names={"axis"};
    ModelCalibrationRecorder recorder;
    require(bool(recorder.start(temp/"paired",metadata)),"start paired recorder");
    std::vector<ModelCalibrationFrame> rows;
    for(std::size_t i=0;i<35;++i) {
        ModelCalibrationFrame f;f.position={.2};f.velocity={0};f.torque={1};f.acceleration={0};
        f.reference_position={.2};f.reference_velocity={0};f.valid=true;f.feedback_age_ms=1;
        f.pose_group=1;f.validation=false;f.direction="reverse";f.phase="static_reverse";
        f.cycle=i+1;f.monotonic_ns=(i+1)*5000000ULL;rows.push_back(f);
    }
    require(recorder.approve_static_group(rows),"reverse static group accepted");
    require(!recorder.approve_static_group(rows),"duplicate reverse group rejected");
    for(auto& f:rows){f.direction="forward";f.phase="static_forward";f.cycle+=100;f.monotonic_ns+=500000000ULL;}
    require(recorder.approve_static_group(rows),"same pose forward group accepted");
    require(recorder.approved_static_frames().size()==70,"both directions retained");
    const auto grouped=group_static_calibration_frames(recorder.approved_static_frames(),30);
    require(grouped.size()==1 && grouped.front().samples.size()==70,"tail trimming must preserve both directions");
    rows[0].pose_group=2;
    require(!recorder.approve_static_group(rows),"mixed pose window rejected");
    for(auto& f:rows){f.pose_group=2;f.direction="reverse";f.phase="static_reverse";f.position={.2};f.cycle+=100;f.monotonic_ns+=500000000ULL;}
    require(recorder.approve_static_group(rows),"second reverse pose accepted");
    for(auto& f:rows){f.direction="forward";f.phase="static_forward";f.position={.23};f.cycle+=100;f.monotonic_ns+=500000000ULL;}
    require(recorder.approve_static_group(rows),"mismatched actual pose remains an observation");
    const auto quality=recorder.status();
    require(quality.bidirectional_static_groups==2 && quality.static_pair_position_warnings==1,"pose repeatability warning recorded during acquisition");
    require(std::abs(quality.max_static_pair_position_delta_rad-.03)<1e-10,"actual pairing position delta");
    recorder.stop();
}
void feedback_age_is_not_control_gap() {
    auto cfg=fixture();auto bus=std::make_unique<FakeBus>(cfg);auto* raw=bus.get();Robot robot;
    raw->fresh_timestamps=true;
    require(bool(robot.configure(cfg,std::move(bus)))&&bool(robot.activate()),"fresh feedback robot setup");
    require(bool(robot.cycle()),"first fresh cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    require(bool(robot.cycle()),"fresh timestamp must not be labeled stale after a scheduler gap");
    raw->fresh_timestamps=false;
    const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    raw->state.feedback_received_ns.assign(cfg.joint_names.size(),now);
    raw->state.feedback_received_ns[2]=now-70000000;
    const auto stale=robot.cycle();
    require(!stale&&stale.error().safety_fault.code==SafetyErr::STATE_TIMEOUT,"genuine stale feedback still faults immediately");
    require(stale.error().safety_fault.index==2,"timeout identifies the stale joint");

    auto timed_bus=std::make_unique<FakeBus>(cfg);timed_bus->fresh_timestamps=true;Robot timed;
    require(bool(timed.configure(cfg,std::move(timed_bus)))&&bool(timed.activate())&&bool(timed.cycle()),"dt watchdog setup");
    for(int i=0;i<3;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        const auto result=timed.cycle();
        if(i<2)require(bool(result),"isolated timing gaps retain watchdog grace");
        else require(!result&&result.error().safety_fault.code==SafetyErr::INVALID_DT,"repeated controller delays still fault as INVALID_DT");
    }
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
        recording_roundtrip(temp);paired_static_recording(temp);feedback_age_is_not_control_gap();sampled_acceleration();full_id_validity();limits_and_identity();named_permutation();fault_shutdown();failed_startup_disable_retry();activation_preserves_running_fault();connected_limits_follow_policy();first_hold_limits();continuous_and_correction(temp);profile_error(temp);serial_exclusion();
        fs::remove_all(temp);std::cout<<"runtime contracts passed\n";return 0;
    } catch(const std::exception& error) {fs::remove_all(temp);std::cerr<<error.what()<<'\n';return 1;}
}
