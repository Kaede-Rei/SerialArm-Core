#include "serial_arm/hardware/motor_bus.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

namespace {

class FakeHardwareBus final : public serial_arm::MotorBus {
public:
    tl::expected<void, serial_arm::MotorBusErr> configure(const std::string& config_path) override {
        try {
            const YAML::Node root = YAML::LoadFile(config_path);
            const YAML::Node node = root["damiao"] ? root["damiao"] : root;
            if(!node || !node.IsMap()) return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CFG);
            const std::string bus = node["bus"].as<std::string>();
            const std::string serial_port = node["serial_port"].as<std::string>();
            const int baudrate = node["baudrate"].as<int>();
            capabilities_.clear();
            serial_arm::ActuatorCapability capability;
            capability.actuator_name = bus + "|" + serial_port + "|" + std::to_string(baudrate);
            capabilities_.push_back(std::move(capability));
            return {};
        }
        catch(const YAML::Exception&) {
            return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CFG);
        }
    }

    tl::expected<void, serial_arm::MotorBusErr> connect() override { return {}; }
    tl::expected<serial_arm::ActuatorState, serial_arm::MotorBusErr> read() override { return serial_arm::ActuatorState{}; }
    tl::expected<void, serial_arm::MotorBusErr> activate() override { return {}; }
    tl::expected<void, serial_arm::MotorBusErr> write(const serial_arm::ActuatorCtrlCmd&) override { return {}; }
    tl::expected<void, serial_arm::MotorBusErr> stop() override { return {}; }
    tl::expected<void, serial_arm::MotorBusErr> deactivate() override { return {}; }
    tl::expected<void, serial_arm::MotorBusErr> recover() override { return {}; }
    const serial_arm::HardwareCapabilities& capabilities() const noexcept override { return capabilities_; }
    void cleanup() noexcept override {}
    std::size_t size() const noexcept override { return capabilities_.size(); }

private:
    serial_arm::HardwareCapabilities capabilities_;
};

} // namespace

extern "C" serial_arm::MotorBus* create_motor_bus() {
    return new FakeHardwareBus();
}

extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {
    delete bus;
}
