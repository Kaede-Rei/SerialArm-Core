#include "serial_arm/hardware/motor_bus.hpp"
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace serial_arm;
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
ActuatorCapability cap(const char* joint,double velocity,double effort){
    ActuatorCapability result;result.joint_name=joint;result.actuator_name=std::string("motor_")+joint;
    result.min_pos=-12.5;result.max_pos=12.5;result.max_vel=velocity;result.max_effort=effort;
    result.max_kp=500.;result.max_kd=5.;return result;
}
class ProbeBus final:public MotorBus {
public:
    HardwareCapabilities caps{cap("j2",8.,28.),cap("j1",30.,10.)};
    HardwareCapabilities pending=caps;
    bool fail_connect=false,fail_recover=false;
    int connects=0,recovers=0,activations=0,writes=0;
    ActuatorCtrlCmd last_command;
    tl::expected<void,MotorBusErr> configure(const std::string&)override{return {};}
    tl::expected<void,MotorBusErr> connect()override{
        ++connects;caps=pending;
        if(fail_connect)return tl::make_unexpected(MotorBusErr::READ_FAILED);
        return {};
    }
    tl::expected<void,MotorBusErr> recover()override{
        ++recovers;caps=pending;
        if(fail_recover)return tl::make_unexpected(MotorBusErr::RECOVER_FAILED);
        return {};
    }
    tl::expected<void,MotorBusErr> activate()override{++activations;return {};}
    tl::expected<ActuatorState,MotorBusErr> read()override{
        ActuatorState s;s.pos={1.,2.};s.vel={3.,4.};s.tor={5.,6.};s.online={1,1};s.enabled={0,0};s.err_code={0,0};
        s.feedback_received_ns={100,200};s.feedback_timing_valid={0,1};return s;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& c)override{++writes;last_command=c;return {};}
    tl::expected<void,MotorBusErr> stop()override{return {};}
    tl::expected<void,MotorBusErr> deactivate()override{return {};}
    void cleanup()noexcept override{}
    const HardwareCapabilities& capabilities()const noexcept override{return caps;}
    std::size_t size()const noexcept override{return caps.size();}
};
void expect_caps(const MotorBus& b,double first,double second){
    const auto& c=b.capabilities();require(c.size()==2,"capability count");
    require(c[0].joint_name=="j1"&&c[1].joint_name=="j2","joint order lost");
    require(c[0].max_vel==first&&c[1].max_vel==second,"capabilities not refreshed/reordered atomically");
}
}
int main(){try{
    auto raw=std::make_unique<ProbeBus>();auto* probe=raw.get();
    auto result=align_motor_bus(std::move(raw),{"j1","j2"});require(bool(result),"align failed");
    auto bus=std::move(*result);expect_caps(*bus,30.,8.);
    probe->pending={cap("j2",10.,24.),cap("j1",35.,12.)};
    require(bool(bus->connect()),"connect failed");expect_caps(*bus,35.,10.);
    require(bus->capabilities()[0].max_effort==12.&&bus->capabilities()[1].max_effort==24.,"effort refresh lost");
    require(probe->activations==0&&probe->writes==0,"connect expanded to activation/command");
    probe->pending={cap("j2",11.,25.),cap("j1",36.,13.)};probe->fail_connect=true;
    auto failed=bus->connect();require(!failed&&failed.error()==MotorBusErr::READ_FAILED,"connect error lost");expect_caps(*bus,35.,10.);
    probe->fail_connect=false;require(bool(bus->connect()),"connect retry failed");expect_caps(*bus,36.,11.);
    probe->pending={cap("j2",12.,26.),cap("j1",37.,14.)};
    require(bool(bus->recover()),"recover failed");expect_caps(*bus,37.,12.);
    probe->pending={cap("j2",13.,27.),cap("j1",38.,15.)};probe->fail_recover=true;
    auto recovery=bus->recover();require(!recovery&&recovery.error()==MotorBusErr::RECOVER_FAILED,"recovery error lost");expect_caps(*bus,37.,12.);
    probe->fail_recover=false;require(bool(bus->recover()),"recover retry failed");expect_caps(*bus,38.,13.);
    probe->pending={cap("j2",14.,28.),cap("wrong_joint",39.,16.)};
    auto identity=bus->connect();require(!identity&&identity.error()==MotorBusErr::INVALID_CFG,"changed mapping accepted");expect_caps(*bus,38.,13.);
    probe->pending={cap("j2",14.,28.)};
    auto size=bus->connect();require(!size&&size.error()==MotorBusErr::INVALID_CFG,"changed size accepted");expect_caps(*bus,38.,13.);
    probe->pending={cap("j2",14.,28.),cap("j1",39.,16.)};require(bool(bus->connect()),"valid retry failed");expect_caps(*bus,39.,14.);
    auto state=bus->read();require(bool(state),"read failed");
    require(state->pos==ActuatorVector({2.,1.})&&state->vel==ActuatorVector({4.,3.})&&state->tor==ActuatorVector({6.,5.}),"feedback permutation changed");
    require(state->feedback_received_ns==std::vector<std::uint64_t>({200,100}),"timestamp permutation changed");
    ActuatorCtrlCmd c;c.pos={10.,20.};c.vel={1.,2.};c.tor={3.,4.};c.kp={5.,6.};c.kd={.1,.2};
    require(bool(bus->write(c)),"write failed");
    require(probe->last_command.pos==ActuatorVector({20.,10.})&&probe->last_command.vel==ActuatorVector({2.,1.}),"command permutation changed");
    require(probe->activations==0,"metadata operations enabled motors");
    // An already aligned backend is returned directly and exposes fresh values
    // without a wrapper cache; both alignment branches must retain this property.
    auto direct=std::make_unique<ProbeBus>();auto* direct_probe=direct.get();
    direct->caps={cap("j1",30.,10.),cap("j2",8.,28.)};direct->pending={cap("j1",31.,11.),cap("j2",10.,29.)};
    auto aligned=align_motor_bus(std::move(direct),{"j1","j2"});require(bool(aligned),"direct align failed");
    require(bool((*aligned)->connect()),"direct connect failed");expect_caps(**aligned,31.,10.);
    require(direct_probe->activations==0&&direct_probe->writes==0,"direct connect expanded permission");
    std::cout<<"ORDERED_CAPABILITY_REFRESH_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
