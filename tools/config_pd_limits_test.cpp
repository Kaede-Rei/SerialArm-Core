// Offline configuration/protocol checks; the only bus below lives in memory.
#include "serial_arm/robot.hpp"
#include "serial_arm/core/gain_quantization.hpp"
#include "dm_hw/damiao.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
using namespace serial_arm;
namespace fs=std::filesystem;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool near(double a,double b){return std::abs(a-b)<1e-9;}
class MemoryBus final:public MotorBus {
public:
    HardwareCapabilities caps;ActuatorCtrlCmd last;JointVector position,velocity;
    std::size_t writes{0},stops{0},deactivations{0};
    explicit MemoryBus(HardwareCapabilities values):caps(std::move(values)),position(caps.size(),0),velocity(caps.size(),0){}
    tl::expected<void,MotorBusErr> configure(const std::string&)override{return {};}
    tl::expected<void,MotorBusErr> connect()override{return {};}
    tl::expected<void,MotorBusErr> activate()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{++deactivations;return {};}
    tl::expected<void,MotorBusErr> stop()override{++stops;return {};}
    tl::expected<void,MotorBusErr> recover()override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override{
        ActuatorState state;const auto n=caps.size();state.pos=position;state.vel=velocity;state.tor.assign(n,0);
        state.online.assign(n,1);state.enabled.assign(n,1);state.err_code.assign(n,0);
        const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(Robot::Clock::now().time_since_epoch()).count();
        state.feedback_received_ns.assign(n,stamp);state.feedback_timing_valid.assign(n,1);return state;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& value)override{last=value;++writes;return {};}
    const HardwareCapabilities& capabilities()const noexcept override{return caps;}
    std::size_t size()const noexcept override{return caps.size();}
    void cleanup()noexcept override{}
};
void write_yaml(const fs::path& path,const YAML::Node& node){std::ofstream out(path);out<<node;require(bool(out),"cannot write temporary fixture");}
RobotCfg load(const fs::path& path,const HardwareCapabilities& caps){
    const auto result=load_robot_cfg(path.string(),caps);
    if(!result)throw std::runtime_error(path.filename().string()+": "+result.error().message);
    return *result;
}
YAML::Node flat_config(const RobotCfg& cfg){
    YAML::Node root;root["joints"]["names"]=cfg.joint_names;
    auto runtime=root["runtime"];runtime["ctrl_frequency_hz"]=200;runtime["write_enabled"]=false;runtime["model_feedforward_mode"]="NONE";
    auto safety=root["safety"];safety["cmd_timeout_s"]=.1;safety["state_timeout_s"]=.05;safety["max_dt_s"]=.05;safety["numeric_tolerance"]=1e-9;
    safety["require_all_actuators_online"]=true;safety["require_all_actuators_enabled"]=true;safety["require_continuous_cmd"]=false;
    auto recovery=safety["fault_recovery"];recovery["default_mode"]="rigid_hold";recovery["allow_compliant_recovery"]=false;
    recovery["require_operator_request"]=true;recovery["gravity_model_validated"]=false;recovery["recovery_timeout_s"]=30;
    recovery["compliant_recovery"]["kp"]=JointVector(cfg.joint_names.size(),0);
    recovery["compliant_recovery"]["kd"]=JointVector(cfg.joint_names.size(),0);
    recovery["compliant_recovery"]["max_vel"]=cfg.safety.limits.max_vel;recovery["compliant_recovery"]["effort_scale"]=.5;
    auto limits=root["limits"];limits["min_pos"]=cfg.safety.limits.min_pos;limits["max_pos"]=cfg.safety.limits.max_pos;
    limits["max_vel"]=cfg.safety.limits.max_vel;limits["max_acc"]=cfg.safety.limits.max_acc;limits["max_effort"]=cfg.safety.limits.max_effort;limits["pos_margin"]=cfg.safety.limits.pos_margin;
    auto map=root["mapping"];map["pos_ratio"]=cfg.mapper.pos_ratio;map["tor_ratio"]=cfg.mapper.tor_ratio;map["direction"]=cfg.mapper.direction;map["joint_zero_offset"]=cfg.mapper.joint_zero_offset;map["actuator_zero_offset"]=cfg.mapper.actuator_zero_offset;
    auto control=root["controller"];control["allow_full_cmd"]=false;
    const auto gain=[&](const char* key,const JointImpedanceGains& g){control[key]["kp"]=g.kp;control[key]["kd"]=g.kd;};
    gain("rigid_hold",cfg.ctrller.rigid_hold_gains);gain("rigid_tracking",cfg.ctrller.rigid_tracking_gains);gain("compliant_hold",cfg.ctrller.compliant_hold_gains);
    gain("compliant_tracking",cfg.ctrller.compliant_tracking_gains);gain("compliant_drag",cfg.ctrller.compliant_drag_gains);gain("identification_tracking",cfg.ctrller.servo_tracking_gains);
    auto dynamics=root["dynamics"];dynamics["urdf_path"]=cfg.dynamics.urdf_path;dynamics["base_frame"]=cfg.dynamics.base_frame;dynamics["tool_frame"]=cfg.dynamics.tool_frame;
    dynamics["gravity"]=JointVector{0,0,-9.81};dynamics["gravity_scale"]=cfg.dynamics.gravity_scale;
    return root;
}
int main(int argc,char** argv){try{
    require(argc==3,"usage: config_pd_limits_test REPO TEMP");const fs::path repo=argv[1],temp=argv[2];
    const auto source=repo/"src/robot_supports/robots/dm_arm/description/config/core/gray.yaml";
    auto yaml=YAML::LoadFile(source.string());yaml["model"]["urdf_path"]=fs::weakly_canonical(source.parent_path()/yaml["model"]["urdf_path"].as<std::string>()).string();
    const auto names=yaml["model"]["joint_names"].as<std::vector<std::string>>();HardwareCapabilities caps;
    for(const auto& name:names)caps.push_back({name,-12.5,12.5,30,28,500,5,name,500.0/4095,5.0/4095});
    const auto section=temp/"section.yaml";write_yaml(section,yaml);const auto baseline=load(section,caps);
    for(std::size_t i=0;i<names.size();++i)require(near(baseline.safety.limits.max_kp[i],500)&&near(baseline.safety.limits.max_kd[i],5),"hardware-only gain bounds not loaded");
    yaml["safety_policy"]["max_kp_override"]="invalid unused old value";yaml["safety_policy"]["max_kd_override"]["unknown_axis"]=-std::numeric_limits<double>::infinity();
    write_yaml(section,yaml);const auto ignored=load(section,caps);require(ignored.safety.limits.max_kp==baseline.safety.limits.max_kp,"deprecated sectioned PD altered bounds");
    auto widened=YAML::Clone(yaml);for(const auto& name:names)widened["safety_policy"]["max_effort_override"][name]=1;
    widened["safety_policy"]["max_effort_override"][names[0]]=10000;write_yaml(section,widened);
    require(!load_robot_cfg(section.string(),caps),"effort policy validation was removed");write_yaml(section,yaml);
    auto flat=flat_config(baseline);const auto legacy=temp/"flat.yaml";write_yaml(legacy,flat);const auto missing=load(legacy,caps);
    require(missing.safety.limits.max_kp==baseline.safety.limits.max_kp,"legacy PD keys remain required");
    flat["limits"]["max_kp"]["bad"]="not a vector";flat["limits"]["max_kd"]=JointVector{-1};write_yaml(legacy,flat);
    require(load(legacy,caps).safety.limits.max_kp==baseline.safety.limits.max_kp,"legacy PD values were parsed/validated");
    auto requested=ignored;requested.safety.limits.max_kp={std::numeric_limits<double>::quiet_NaN()};requested.safety.limits.max_kd={-1};
    requested.safety.limits.max_effort[0]=9;requested.safety.limits.max_vel[0]=.5;
    requested.safety.fault_recovery.compliant_recovery.max_vel[0]=.5;
    auto refreshed=caps;for(auto& cap:refreshed){cap.max_kp=700;cap.max_kd=7;cap.kp_resolution=700.0/4095;cap.kd_resolution=7.0/4095;}
    const auto connected=resolve_connected_safety_cfg(requested,refreshed);require(bool(connected),"capability refresh retained stale/user PD bounds");
    require(near(connected->limits.max_kp[0],700)&&near(connected->limits.kp_resolution[0],700.0/4095),"refreshed hardware PD/quantum missing");
    require(near(connected->limits.max_effort[0],9)&&near(connected->limits.max_vel[0],.5),"non-PD safety edits were discarded");
    const auto model=ModelLoader{}.load(baseline.dynamics.urdf_path,names);require(bool(model),"test model missing");
    auto mapping=baseline.mapper;mapping.pos_ratio[0]=2;mapping.tor_ratio[0]=3;mapping.direction[0]=-1;
    SafetyPolicyCfg policy;policy.max_acc.assign(names.size(),10);policy.max_dt_s=.05;policy.state_timeout_s=.05;policy.cmd_timeout_s=.1;
    policy.max_kp_override={-1};policy.max_kd_override={std::numeric_limits<double>::quiet_NaN()};
    const auto resolved=LimitResolver{}.resolve(*model,mapping,caps,policy);require(bool(resolved),"deprecated programmatic overrides were validated");
    require(near(resolved->joints[0].max_kp,3000)&&near(resolved->joints[0].kp_resolution,6*500.0/4095),"gain ratio or quantum mapping incorrect");
    JointActuatorMapper converter;require(bool(converter.configure(mapping)),"test mapping invalid");
    JointCtrlCmd command;command.pos.assign(names.size(),0);command.vel=command.tor=command.kp=command.kd=command.pos;
    command.kp[0]=37*resolved->joints[0].kp_resolution;command.kd[0]=13*resolved->joints[0].kd_resolution;
    const auto mapped=converter.to_actuator_cmd(command);require(bool(mapped),"mapped command invalid");
    require(damiao::encode_mit_gain(static_cast<float>(mapped->kp[0]),500)==37&&damiao::encode_mit_gain(static_cast<float>(mapped->kd[0]),5)==13,"mapped accepted gain changed protocol code");
    const auto restored=resolve_from_safety_cfg(names,to_safety_cfg(*resolved));require(near(restored.joints[0].kd_resolution,6*5.0/4095),"resolved quantum round-trip lost");
    require(!project_protocol_gain(-.01,500,500.0/4095)&&!project_protocol_gain(500.01,500,500.0/4095),"illegal gain silently projected into range");
    require(near(*project_protocol_gain(.082997,500,0),.082997),"continuous backend changed");
    for(unsigned code=0;code<=4095;++code)for(const double maximum:{500.0,5.0}){
        const double center=code*maximum/4095;const auto projected=project_protocol_gain(center,maximum,maximum/4095);
        require(bool(projected)&&damiao::encode_mit_gain(static_cast<float>(*projected),maximum)==code,"MIT gain centre changed code during encoding");
    }
    auto cfg=baseline;cfg.runtime.write_enabled=true;cfg.runtime.model_feedforward_mode=ModelFeedforwardMode::NONE;cfg.shutdown.park_before_disable=false;
    cfg.ctrller.allow_full_cmd=true;
    cfg.ctrller.rigid_hold_gains={JointVector(names.size(),.50001*500/4095),JointVector(names.size(),.50001*5/4095)};
    cfg.capability.admittance.enabled=false;cfg.hold_compensation.enabled=false;cfg.safety.require_continuous_cmd=false;
    auto bus=std::make_unique<MemoryBus>(caps);auto* memory=bus.get();Robot robot;require(bool(robot.configure(cfg,std::move(bus))),"memory Robot configure failed");
    require(bool(robot.activate()),"memory Robot activate failed");require(bool(robot.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING)),"memory mode failed");
    require(robot.set_calibration_tracking_gains({JointVector(names.size(),499),JointVector(names.size(),4.9)}),"legal protocol gain still restricted by user ceiling");
    const JointImpedanceGains gains{JointVector(names.size(),.082997),JointVector(names.size(),.001738)};
    require(robot.set_calibration_tracking_gains(gains),"legal gains above old policy refused");RobotCycleOutput output;
    for(unsigned k=0;k<125;++k){std::this_thread::sleep_for(std::chrono::milliseconds(5));require(bool(robot.set_cmd(JointPosVelCmd{JointVector(names.size(),0),JointVector(names.size(),0)})),"memory reference rejected");auto cycle=robot.cycle();require(bool(cycle),"memory cycle failed");output=*cycle;}
    require(near(output.requested_kp[0],gains.kp[0])&&near(output.requested_kd[0],gains.kd[0]),"pre-quantization requests not preserved");
    require(near(output.joint_cmd.kp[0],500.0/4095)&&near(output.joint_cmd.kd[0],5.0/4095),"cycle did not apply encodable gains");
    require(output.actuator_cmd.kp==memory->last.kp&&output.joint_cmd.kp==output.effective_kp,"accepted gain telemetry differs from sent command");
    auto illegal=gains;illegal.kp[0]=500.01;require(!robot.set_calibration_tracking_gains(illegal),"gain above protocol maximum accepted");
    auto invalid_command=output.joint_cmd;invalid_command.kp[0]=500.01;
    require(bool(robot.set_full_cmd(invalid_command)),"memory full command setup failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));const auto rejected=robot.cycle();
    require(!rejected&&rejected.error().safety_fault.code==SafetyErr::CMD_KP_LIMIT,"illegal runtime gain was silently rounded into range");
    require(robot.is_fault_holding()&&near(memory->last.kp[0],500.0/4095)&&near(memory->last.kd[0],5.0/4095),"initial fault hold bypassed gain projection");
    require(bool(robot.maintain_fault_hold())&&near(memory->last.kp[0],500.0/4095)&&near(memory->last.kd[0],5.0/4095),"maintained fault hold bypassed gain projection");
    require(bool(robot.force_deactivate()),"memory deactivate failed");
    cfg.safety.fault_recovery.allow_compliant_recovery=true;cfg.safety.fault_recovery.gravity_model_validated=true;
    cfg.safety.fault_recovery.compliant_recovery.kp.assign(names.size(),1.50001*500/4095);
    cfg.safety.fault_recovery.compliant_recovery.kd.assign(names.size(),1.50001*5/4095);
    auto recovery_bus=std::make_unique<MemoryBus>(caps);auto* recovery_memory=recovery_bus.get();Robot recovery_robot;
    require(bool(recovery_robot.configure(cfg,std::move(recovery_bus)))&&bool(recovery_robot.activate()),"recovery memory fixture failed");
    require(bool(recovery_robot.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING)),"recovery memory mode failed");
    auto out_of_range=output.joint_cmd;out_of_range.pos[0]=cfg.safety.limits.max_pos[0]+1;
    require(bool(recovery_robot.set_full_cmd(out_of_range)),"recovery invalid command setup failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));const auto position_fault=recovery_robot.cycle();
    require(!position_fault&&position_fault.error().safety_fault.code==SafetyErr::CMD_POS_LIMIT,"position safety changed during gain projection");
    require(bool(recovery_robot.enter_fault_compliant_recovery())&&near(recovery_memory->last.kp[0],2*500.0/4095)&&near(recovery_memory->last.kd[0],2*5.0/4095),"compliant fault recovery bypassed gain projection");
    require(bool(recovery_robot.return_to_fault_rigid_hold())&&near(recovery_memory->last.kp[0],500.0/4095)&&near(recovery_memory->last.kd[0],5.0/4095),"return to rigid fault hold bypassed gain projection");
    require(bool(recovery_robot.force_deactivate()),"recovery memory deactivate failed");
    // Raw half-code damping would predict less than 1 Nm, but nearest encoding
    // produces one full code and exceeds that same unchanged effort limit.
    auto budget_cfg=cfg;budget_cfg.safety.limits.max_effort[0]=1;budget_cfg.safety.numeric_tolerance=1e-9;
    const double gravity=.9999,negative_velocity=-.1;
    require(gravity-budget_cfg.ctrller.rigid_hold_gains.kd[0]*negative_velocity<1&&gravity-(5.0/4095)*negative_velocity>1+budget_cfg.safety.numeric_tolerance,"half-code torque counterexample invalid");
    const auto gravity_model=[&](ModelFeedforwardMode mode,const JointState&,const JointVector&,const JointVector&,double)->tl::expected<JointVector,ModelFeedforwardErr>{
        JointVector result(names.size(),0);if(mode==ModelFeedforwardMode::GRAVITY)result[0]=gravity;return result;
    };
    for(const bool moving_at_start:{true,false}) {
        auto budget_bus=std::make_unique<MemoryBus>(caps);auto* budget_memory=budget_bus.get();
        if(moving_at_start)budget_memory->velocity[0]=negative_velocity;
        Robot budget_robot;require(bool(budget_robot.configure(budget_cfg,std::move(budget_bus),gravity_model))&&bool(budget_robot.activate()),"budget memory fixture failed");
        require(bool(budget_robot.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING)),"budget memory mode failed");
        auto bad=output.joint_cmd;bad.kp[0]=500.01;require(bool(budget_robot.set_full_cmd(bad)),"budget fault setup rejected");
        const auto before=budget_memory->writes;std::this_thread::sleep_for(std::chrono::milliseconds(5));require(!budget_robot.cycle(),"budget fault missing");
        if(moving_at_start) {
            require(!budget_robot.is_fault_holding()&&budget_memory->writes==before&&budget_memory->stops>0,"over-budget initial fault hold was written");
        } else {
            require(budget_robot.is_fault_holding()&&near(budget_memory->last.kd[0],5.0/4095),"safe initial fault hold not quantized");
            budget_memory->velocity[0]=negative_velocity;const auto held_writes=budget_memory->writes;
            const auto maintained=budget_robot.maintain_fault_hold();
            require(!maintained&&maintained.error().code==RobotErr::REFERENCE_GOVERNOR_FAILED&&maintained.error().reference_governor_err==ReferenceGovernorErr::INFEASIBLE,"over-budget maintained hold not rejected");
            require(budget_memory->writes==held_writes&&budget_memory->deactivations>0&&!budget_robot.is_fault_holding(),"over-budget maintained fault hold was sent");
        }
        require(bool(budget_robot.force_deactivate()),"budget memory deactivate failed");
    }
    std::cout<<"PASS: deprecated PD configuration ignored; hardware limits/refresh/mapping retained; 8192 code centres, memory Robot accepted gains and fault-hold torque budgets verified\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
