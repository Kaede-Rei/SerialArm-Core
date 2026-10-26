#include "serial_arm/hardware/motor_bus.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

class MockHardware final : public serial_arm::MotorBus {
    serial_arm::HardwareCapabilities caps_;
    serial_arm::ActuatorState state_;
    int delay_ms_{0};
public:
    tl::expected<void,serial_arm::MotorBusErr> configure(const std::string& path) override {
        try {
            const auto root=YAML::LoadFile(path);
            const auto names=root["joints"].as<std::vector<std::string>>();
            const auto pos=root["positions"].as<std::vector<double>>();
            if(names.empty()||pos.size()!=names.size()) return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CFG);
            caps_.clear();
            for(const auto& name:names) caps_.push_back({name,-100,100,100,100,5000,50,name});
            state_.pos=pos;state_.vel.assign(names.size(),0);state_.tor.assign(names.size(),0);
            state_.online.assign(names.size(),1);state_.enabled.assign(names.size(),0);state_.err_code.assign(names.size(),0);
            delay_ms_=root["activation_delay_ms"]?root["activation_delay_ms"].as<int>():0;
            return {};
        } catch(...) {return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CFG);}
    }
    tl::expected<void,serial_arm::MotorBusErr> connect() override {return {};}
    tl::expected<void,serial_arm::MotorBusErr> activate() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
        std::fill(state_.enabled.begin(),state_.enabled.end(),1);return {};
    }
    tl::expected<serial_arm::ActuatorState,serial_arm::MotorBusErr> read() override {return state_;}
    tl::expected<void,serial_arm::MotorBusErr> write(const serial_arm::ActuatorCtrlCmd& cmd) override {
        state_.pos=cmd.pos;state_.vel=cmd.vel;state_.tor=cmd.tor;return {};
    }
    tl::expected<void,serial_arm::MotorBusErr> stop() override {return {};}
    tl::expected<void,serial_arm::MotorBusErr> deactivate() override {std::fill(state_.enabled.begin(),state_.enabled.end(),0);return {};}
    tl::expected<void,serial_arm::MotorBusErr> recover() override {return {};}
    void cleanup() noexcept override {}
    const serial_arm::HardwareCapabilities& capabilities() const noexcept override {return caps_;}
    std::size_t size() const noexcept override {return caps_.size();}
};
extern "C" std::uint64_t serial_arm_motor_bus_abi() {return serial_arm::motor_bus_abi_fingerprint;}
extern "C" serial_arm::MotorBus* create_motor_bus() {return new MockHardware;}
extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {delete bus;}
