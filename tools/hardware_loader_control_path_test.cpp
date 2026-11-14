#include "serial_arm/hardware/hardware_loader.hpp"

#include <iostream>
#include <stdexcept>

using namespace serial_arm;

#ifdef SERIAL_ARM_CONTROL_PATH_TEST_PLUGIN

// A dynamic plugin with no physical transport. Activation verifies the exact
// CSP speed limits received through HardwareLoader's lifecycle wrapper.
class ControlPathPlugin final : public MotorBus {
public:
    tl::expected<void, MotorBusErr> configure(const std::string&) override { return {}; }
    tl::expected<void, MotorBusErr> connect() override { return {}; }
    tl::expected<ActuatorState, MotorBusErr> read() override { return ActuatorState{}; }
    tl::expected<void, MotorBusErr> select_control_path(
        ControlPath path, const ActuatorVector& speed_limit = {}) override {
        if(path==ControlPath::CSP && speed_limit!=ActuatorVector{0.4,0.7})
            return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        selected_=path;
        speed_limit_=speed_limit;
        return {};
    }
    tl::expected<void, MotorBusErr> activate() override {
        if(selected_!=ControlPath::CSP || speed_limit_!=ActuatorVector{0.4,0.7})
            return tl::make_unexpected(MotorBusErr::INVALID_STATE);
        return {};
    }
    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd&) override { return {}; }
    tl::expected<void, MotorBusErr> stop() override { return {}; }
    tl::expected<void, MotorBusErr> deactivate() override { return {}; }
    tl::expected<void, MotorBusErr> recover() override { return {}; }
    const HardwareCapabilities& capabilities() const noexcept override { return capabilities_; }
    void cleanup() noexcept override {}
    std::size_t size() const noexcept override { return 2; }
private:
    ControlPath selected_{ControlPath::MIT};
    ActuatorVector speed_limit_;
    HardwareCapabilities capabilities_;
};

extern "C" std::uint64_t serial_arm_motor_bus_abi() { return motor_bus_abi_fingerprint; }
extern "C" MotorBus* create_motor_bus() { return new ControlPathPlugin(); }
extern "C" void destroy_motor_bus(MotorBus* bus) { delete bus; }

#else

int main(int argc, char** argv) {
    if(argc!=3) throw std::runtime_error("expected plugin and temporary config paths");
    HardwareLoader loader;
    auto loaded=loader.load(argv[1],argv[2]);
    if(!loaded) throw std::runtime_error("could not dynamically load test plugin");
    auto& bus=**loaded;
    const auto selected=bus.select_control_path(ControlPath::CSP,{0.4,0.7});
    if(!selected) throw std::runtime_error("dynamic wrapper rejected supported CSP path");
    if(!bus.activate()) throw std::runtime_error("dynamic wrapper lost path or per-axis speed limits");
    const auto rejected=bus.select_control_path(ControlPath::CSP,{0.4,-0.7});
    if(rejected || rejected.error()!=MotorBusErr::INVALID_CMD)
        throw std::runtime_error("dynamic wrapper did not preserve plugin rejection");
    if(!bus.select_control_path(ControlPath::MIT))
        throw std::runtime_error("dynamic wrapper rejected MIT path");
    std::cout<<"PASS: dynamically loaded MotorBus forwards MIT/CSP, per-axis speed limits and plugin errors; no hardware\n";
}

#endif
