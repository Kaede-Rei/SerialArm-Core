#include "serial_arm/hardware/gain_context.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs=std::filesystem;
namespace {
constexpr const char* plugin="serial_arm_hardware_damiao";
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool equal(const YAML::Node& a,const YAML::Node& b) {
    if(!a || a.IsNull())return !b || b.IsNull();
    if(!b || b.IsNull() || a.Type()!=b.Type())return false;
    if(a.IsMap()) {
        if(a.size()!=b.size())return false;
        for(const auto& item:a)if(!equal(item.second,b[item.first.as<std::string>()]))return false;
        return true;
    }
    if(a.IsSequence()) {
        if(a.size()!=b.size())return false;
        for(std::size_t i=0;i<a.size();++i)if(!equal(a[i],b[i]))return false;
        return true;
    }
    if(a.Scalar()==b.Scalar())return true;
    try {const double x=a.as<double>(),y=b.as<double>();return std::isfinite(x) && std::isfinite(y) && x==y;}
    catch(const YAML::Exception&){return false;}
}
bool compatible(const YAML::Node& a,const YAML::Node& b,const std::string& driver=plugin) {
    return equal(serial_arm::gain_hardware_context(a,driver),serial_arm::gain_hardware_context(b,driver));
}
YAML::Node fixture(const std::string& port) {
    auto node=YAML::Load(R"(
buses:
  main_can: {type: can, backend: damiao_usb2can, baudrate: 921600}
damiao:
  bus: main_can
  refresh_state_in_read: false
  feedback_timeout_s: 0.05
  activation_retries: 3
  startup_read_cycles: 5
  stop_kp: 10
  stop_kd: 0.15
  stop_cycles: 5
  actuators:
    joint1: {name: actuator1, motor_id: 1, master_id: 0, motor_type: DM4340}
    joint2: {name: actuator2, motor_id: 2, master_id: 0, motor_type: DM4310}
)");
    node["buses"]["main_can"]["device"]=port;
    return node;
}
void semantics(const fs::path& temporary) {
    check(fs::is_character_file("/dev/null") && fs::is_character_file("/dev/zero"),"test requires existing distinct character devices");
    const auto alias=temporary/"serial-alias";
    fs::create_symlink("/dev/null",alias);
    const auto saved=fixture("/dev/null");
    auto recorded=fixture(alias.string());
    recorded["damiao"]["serial_port"]=alias.string();recorded["damiao"]["baudrate"]=921600;
    const auto original=YAML::Dump(recorded);
    check(!equal(saved,recorded),"fixture did not recreate override materialization mismatch");
    check(compatible(saved,recorded),"same device aliases and redundant override fields were rejected");
    check(YAML::Dump(recorded)==original,"comparison modified the immutable task snapshot");
    check(!compatible(saved,recorded,"custom_hardware"),"Damiao-specific normalization was applied to an unknown driver");

    auto changed=YAML::Clone(recorded);changed["damiao"]["serial_port"]="/dev/zero";
    check(!compatible(saved,changed),"a flat serial override to another device was ignored");
    changed=YAML::Clone(recorded);changed["damiao"]["baudrate"]=115200;
    check(!compatible(saved,changed),"the driver's flat baudrate precedence was ignored");
    changed=YAML::Clone(recorded);changed["buses"]["main_can"]["baudrate"]=115200;
    check(!compatible(saved,changed),"a changed named-bus declaration was ignored");
    changed=YAML::Clone(recorded);changed["buses"]["main_can"]["backend"]="another_backend";
    check(!compatible(saved,changed),"different CAN backend was accepted");
    changed=YAML::Clone(recorded);changed["buses"]["main_can"]["type"]="serial";
    check(!compatible(saved,changed),"different bus type was accepted");
    for(const char* field:{"motor_id","master_id","motor_type"}) {
        changed=YAML::Clone(recorded);
        changed["damiao"]["actuators"]["joint1"][field]=std::string(field)=="motor_type"?YAML::Node("DM4310"):YAML::Node(99);
        check(!compatible(saved,changed),"different actuator/protocol declaration was accepted");
    }
    changed=YAML::Clone(recorded);changed["damiao"]["feedback_timeout_s"]=.1;
    check(!compatible(saved,changed),"different feedback timing was accepted");
    changed=YAML::Clone(recorded);changed["damiao"]["stop_kd"]=.5;
    check(!compatible(saved,changed),"different stop behavior was accepted");
    changed=YAML::Clone(recorded);changed["damiao"]["bus"]="other";
    check(!compatible(saved,changed),"different selected bus was accepted");

    auto flat_override=fixture("/dev/zero");
    flat_override["damiao"]["serial_port"]="/dev/null";flat_override["damiao"]["baudrate"]=115200;
    const auto resolved=serial_arm::gain_hardware_context(flat_override,plugin);
    check(resolved["damiao"]["serial_port"].as<std::string>()=="/dev/null" &&
          resolved["damiao"]["baudrate"].as<int>()==115200,"authored flat fields lost priority to the named bus");
    auto default_bus=fixture("/dev/null");default_bus["damiao"].remove("bus");
    check(serial_arm::gain_hardware_context(default_bus,plugin)["damiao"]["serial_port"].as<std::string>()=="/dev/null",
          "the driver's default main_can bus was not resolved");

    const auto missing=temporary/"missing";fs::create_symlink(temporary/"absent",missing);
    check(serial_arm::gain_serial_device_identity(missing.string())==missing.string(),"a missing path was assumed to identify hardware");
    const auto regular=temporary/"regular";std::ofstream(regular)<<"not hardware";
    const auto regular_alias=temporary/"regular-alias";fs::create_symlink(regular,regular_alias);
    check(serial_arm::gain_serial_device_identity(regular_alias.string())==regular_alias.string(),"regular-file aliases were treated as serial devices");
    check(serial_arm::gain_hardware_driver_identity("/lib/libserial_arm_hardware_damiao.so.1")==plugin,
          "driver path identity differs from native comparison");
}
}
int main(int argc,char** argv) {
    const auto temporary=fs::temp_directory_path()/std::string("serialarm-hardware-context-")
        .append(std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try {
        fs::create_directories(temporary);semantics(temporary);
        if(argc==3)check(compatible(YAML::LoadFile(argv[1]),YAML::LoadFile(argv[2])),
            "the actual recorded/current hardware contexts still differ");
        fs::remove_all(temporary);
        std::cout<<"gain hardware context: same device aliases, override materialization, driver precedence, real mismatch rejection PASS\n";
    }catch(const std::exception& error){fs::remove_all(temporary);std::cerr<<error.what()<<'\n';return 1;}
}
