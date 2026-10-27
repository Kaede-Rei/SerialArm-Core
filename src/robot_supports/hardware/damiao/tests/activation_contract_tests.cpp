#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <atomic>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
using namespace serial_arm;
struct Adapter {
 int master; std::atomic<bool> running{true},mute{false};std::thread worker;std::array<unsigned char,16> status{};
 Adapter():master(::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK)) {
  if(master<0||::grantpt(master)||::unlockpt(master))throw std::runtime_error("PTY setup");
  worker=std::thread([this]{loop();});
 }
 ~Adapter(){running=false;if(worker.joinable())worker.join();::close(master);}
 void reply(unsigned id,const unsigned char* data){
  if(mute)return;
  std::array<unsigned char,16> b{0xaa,0x11,8,0,0,0,0};
  b[3]=id&255;b[4]=(id>>8)&255;std::copy(data,data+8,b.begin()+7);b[15]=0x55;
  const auto sent=::write(master,b.data(),b.size());
  if(sent!=static_cast<ssize_t>(b.size()))running=false;
 }
 void feedback(unsigned id){std::array<unsigned char,8> b{static_cast<unsigned char>((status[id]<<4)|id),0x80,0,0x80,0x08,0,0,0};reply(0,b.data());}
 void loop(){std::vector<unsigned char> q;while(running){
  pollfd p{master,POLLIN,0};if(::poll(&p,1,2)<=0)continue;unsigned char b[4096];auto n=::read(master,b,sizeof(b));if(n<=0)continue;q.insert(q.end(),b,b+n);
  while(q.size()>=30){if(q[0]!=0x55||q[1]!=0xaa){q.erase(q.begin());continue;}
   unsigned id=q[13]|(q[14]<<8);const auto* data=q.data()+21;
   if(id==0x7ff){unsigned motor=data[0]|(data[1]<<8);if(motor<status.size()){
    if(data[2]==0xcc)feedback(motor);
    else if(data[2]==0x33&&data[3]>=damiao::PMAX&&data[3]<=damiao::TMAX){
     std::array<unsigned char,8> response;std::copy(data,data+8,response.begin());
     const float value=data[3]==damiao::PMAX?12.5f:(data[3]==damiao::VMAX?(motor<=3?8.0f:30.0f):(motor<=3?28.0f:10.0f));
     std::memcpy(response.data()+4,&value,sizeof(value));reply(motor,response.data());
    }else if(data[2]==0x55||data[2]==0x33)reply(motor,data);
   }}else if(id>0&&id<status.size()){
    bool management=true;for(unsigned j=0;j<7;++j)management&=data[j]==0xff;
    if(management&&data[7]==0xfc)status[id]=1;
    if(management&&data[7]==0xfd)status[id]=0;
    feedback(id);
   }
   q.erase(q.begin(),q.begin()+30);
  }
 }}
};
int main(){
 try{
  Adapter adapter;
  auto config=YAML::LoadFile(std::string(SERIAL_ARM_REPO)+"/src/robot_supports/robots/dm_arm/description/config/hardware.yaml");
  config["buses"]["main_can"]["device"]=::ptsname(adapter.master);
  config["damiao"]["activation_retries"]=1;
  auto file=std::filesystem::temp_directory_path()/("serialarm-activation-"+std::to_string(::getpid())+".yaml");
  std::ofstream(file)<<YAML::Dump(config);
  DamiaoMotorBus bus;
  if(!bus.configure(file.string())||!bus.connect())throw std::runtime_error("driver setup");
  const auto active=bus.activate();
  if(!active)throw std::runtime_error("timely enable acknowledgement rejected as offline");
  if(!bus.deactivate())throw std::runtime_error("disable confirmation failed");
  if(!bus.activate())throw std::runtime_error("reenable after disable failed");
  adapter.mute=true;
  if(bus.deactivate())throw std::runtime_error("missing disable acknowledgement reported success");
  adapter.mute=false;
  if(!bus.deactivate())throw std::runtime_error("retry after communication returns failed");
  bus.cleanup();std::filesystem::remove(file);std::cout<<"ACTIVATION_AND_DISABLE_PASS\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
