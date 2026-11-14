#include "serial_arm/hardware/motor_bus.hpp"
#include <cassert>
#include <iostream>
using namespace serial_arm;
class Bus final : public MotorBus {
public:
    HardwareCapabilities caps{{"b",-2,2,2,1,100,5,"b"},{"a",-2,2,2,1,100,5,"a"}};
    ActuatorVector speed;
    tl::expected<void,MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,MotorBusErr> connect() override{return {};}
    tl::expected<ActuatorState,MotorBusErr> read() override{return tl::make_unexpected(MotorBusErr::READ_FAILED);}
    tl::expected<void,MotorBusErr> select_control_path(ControlPath,const ActuatorVector& value = {}) override {speed=value;return {};}
    tl::expected<void,MotorBusErr> activate() override{return {};}
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd&) override{return {};}
    tl::expected<void,MotorBusErr> stop() override{return {};}
    tl::expected<void,MotorBusErr> deactivate() override{return {};}
    tl::expected<void,MotorBusErr> recover() override{return {};}
    const HardwareCapabilities& capabilities() const noexcept override{return caps;}
    std::size_t size() const noexcept override{return 2;}
    void cleanup() noexcept override{}
};
int main(){
    auto bus=std::make_unique<Bus>();auto* raw=bus.get();
    auto ordered=align_motor_bus(std::move(bus),{"a","b"});assert(ordered);
    assert((*ordered)->select_control_path(ControlPath::CSP,{.3,.6}));
    assert(raw->speed==ActuatorVector({.6,.3}));
    assert(!(*ordered)->select_control_path(ControlPath::CSP,{.3}));
    assert((*ordered)->select_control_path(ControlPath::MIT,{}));assert(raw->speed.empty());
    std::cout<<"ordered startup speed passed\n";
}
