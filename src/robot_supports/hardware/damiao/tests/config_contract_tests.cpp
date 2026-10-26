#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"
#include <yaml-cpp/yaml.h>
#include "dm_hw/damiao.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
using namespace serial_arm;
int main(){
    const auto file=std::filesystem::temp_directory_path()/("serialarm-damiao-"+std::to_string(::getpid())+".yaml");
    try {
        damiao::Motor motor(damiao::DM4310,1,0);
        const auto old=std::chrono::duration_cast<std::chrono::nanoseconds>((std::chrono::steady_clock::now()-std::chrono::milliseconds(40)).time_since_epoch()).count();
        motor.receive_data(1,2,3,1,old,true);
        if(std::chrono::duration_cast<std::chrono::nanoseconds>(motor.get_state_received_at().time_since_epoch()).count()!=old)
            throw std::runtime_error("motor decode replaced transport timestamp");
        motor.receive_data(9,9,9,1,old-1000,true);
        if(motor.get_position()!=1)throw std::runtime_error("time-reversed feedback overwrote newer state");
        const auto source=YAML::LoadFile(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/hardware.yaml");
        const auto first=source["damiao"]["actuators"].begin()->first.as<std::string>();
        for(const auto& ids:std::vector<std::pair<int,int>>{{17,0},{0x800,1},{1,0x800}}){
            auto config=YAML::Clone(source);config["damiao"]["actuators"][first]["motor_id"]=ids.first;config["damiao"]["actuators"][first]["master_id"]=ids.second;
            std::ofstream(file)<<YAML::Dump(config);DamiaoMotorBus bus;
            if(bus.configure(file.string()))throw std::runtime_error("ambiguous or out-of-range ID accepted");
        }
        std::ofstream(file)<<YAML::Dump(source);DamiaoMotorBus valid;
        if(!valid.configure(file.string()))throw std::runtime_error("valid configuration rejected");
        if(valid.capabilities().empty()||valid.capabilities()[0].joint_name!=first)throw std::runtime_error("hardware identity lost");
        std::filesystem::remove(file);std::cout<<"driver configuration contracts passed\n";return 0;
    }catch(const std::exception& error){std::filesystem::remove(file);std::cerr<<error.what()<<'\n';return 1;}
}
