#include "serial_arm/hardware/motor_bus.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

class FaultInjectHardwareBus final : public serial_arm::MotorBus {
public:
    FaultInjectHardwareBus() {
        constexpr std::size_t kJoints = 6;
        capabilities_.reserve(kJoints);
        for(std::size_t i = 0; i < kJoints; ++i) {
            serial_arm::ActuatorCapability cap;
            cap.actuator_name = "fault_inject_actuator" + std::to_string(i + 1);
            cap.min_pos = -10.0;
            cap.max_pos = 10.0;
            cap.max_vel = 20.0;
            cap.max_effort = 100.0;
            cap.max_kp = 500.0;
            cap.max_kd = 50.0;
            capabilities_.push_back(std::move(cap));
        }

        state_.pos.assign(kJoints, 0.0);
        state_.vel.assign(kJoints, 0.0);
        state_.tor.assign(kJoints, 0.0);
        state_.online.assign(kJoints, 1);
        state_.enabled.assign(kJoints, 1);
        state_.err_code.assign(kJoints, 0);
    }

    tl::expected<void, serial_arm::MotorBusErr> configure(const std::string&) override {
        return {};
    }

    tl::expected<void, serial_arm::MotorBusErr> connect() override {
        connected_ = true;
        return {};
    }

    tl::expected<serial_arm::ActuatorState, serial_arm::MotorBusErr> read() override {
        if(!connected_) return tl::make_unexpected(serial_arm::MotorBusErr::NOT_CONNECTED);
        if(active_) {
            ++active_read_count_;
            if(!fault_injected_ && active_read_count_ >= kFaultAfterActiveReads) {
                fault_injected_ = true;
                return tl::make_unexpected(serial_arm::MotorBusErr::READ_FAILED);
            }
        }
        return state_;
    }

    tl::expected<void, serial_arm::MotorBusErr> activate() override {
        if(!connected_) return tl::make_unexpected(serial_arm::MotorBusErr::NOT_CONNECTED);
        active_ = true;
        return {};
    }

    tl::expected<void, serial_arm::MotorBusErr> write(const serial_arm::ActuatorCtrlCmd& cmd) override {
        if(!active_) return tl::make_unexpected(serial_arm::MotorBusErr::NOT_ACTIVE);
        if(cmd.pos.size() == state_.pos.size()) state_.pos = cmd.pos;
        if(cmd.vel.size() == state_.vel.size()) state_.vel = cmd.vel;
        if(cmd.tor.size() == state_.tor.size()) state_.tor = cmd.tor;
        return {};
    }

    tl::expected<void, serial_arm::MotorBusErr> stop() override { return {}; }

    tl::expected<void, serial_arm::MotorBusErr> deactivate() override {
        active_ = false;
        state_.enabled.assign(state_.enabled.size(), 0);
        return {};
    }

    tl::expected<void, serial_arm::MotorBusErr> recover() override { return {}; }

    const serial_arm::HardwareCapabilities& capabilities() const noexcept override {
        return capabilities_;
    }

    void cleanup() noexcept override {
        active_ = false;
        connected_ = false;
    }

    std::size_t size() const noexcept override { return capabilities_.size(); }

private:
    static constexpr std::size_t kFaultAfterActiveReads = 35;

    serial_arm::HardwareCapabilities capabilities_;
    serial_arm::ActuatorState state_;
    std::size_t active_read_count_{ 0 };
    bool connected_{ false };
    bool active_{ false };
    bool fault_injected_{ false };
};

} // namespace

extern "C" serial_arm::MotorBus* create_motor_bus() {
    return new FaultInjectHardwareBus();
}

extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {
    delete bus;
}
