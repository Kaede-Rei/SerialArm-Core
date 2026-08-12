#include "dm_hw/damiao.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace {

using serial_arm::transport::CanBus;
using serial_arm::transport::CanErr;
using serial_arm::transport::CanFrame;

class MockCanBus final : public CanBus {
public:
    tl::expected<void, CanErr> open() override {
        open_ = true;
        return {};
    }

    void close() noexcept override {
        open_ = false;
    }

    bool is_open() const noexcept override {
        return open_;
    }

    tl::expected<void, CanErr> send(const CanFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.push_back(frame);
        return {};
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if(rx_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
        CanFrame frame = rx_.front();
        rx_.pop_front();
        return frame;
    }

    void flush() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
    }

    void push_rx(const CanFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::size_t tx_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_.size();
    }

private:
    bool open_{ false };
    mutable std::mutex mutex_;
    std::deque<CanFrame> rx_;
    std::deque<CanFrame> tx_;
};

CanFrame feedback_frame(std::uint8_t slave_id) {
    CanFrame frame;
    frame.id = 0;
    frame.size = 8;
    frame.data[0] = slave_id;
    frame.data[1] = 0x80;
    frame.data[2] = 0x00;
    frame.data[3] = 0x80;
    frame.data[4] = 0x08;
    frame.data[5] = 0x00;
    return frame;
}

CanFrame parameter_frame(std::uint32_t slave_id, std::uint8_t response_type, std::uint8_t reg_id, std::uint32_t value) {
    CanFrame frame;
    frame.id = 0x7FF;
    frame.size = 8;
    frame.data[0] = static_cast<std::uint8_t>(slave_id & 0xFF);
    frame.data[1] = static_cast<std::uint8_t>((slave_id >> 8) & 0xFF);
    frame.data[2] = response_type;
    frame.data[3] = reg_id;
    frame.data[4] = static_cast<std::uint8_t>(value & 0xFF);
    frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
    frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
    return frame;
}

std::shared_ptr<MockCanBus> make_open_bus() {
    auto bus = std::make_shared<MockCanBus>();
    (void)bus->open();
    return bus;
}

} // namespace

TEST(DamiaoMotorControlTests, SharedMasterZeroTargetedFeedbackMatchesSlaveId) {
    auto bus = make_open_bus();
    auto channel = bus->create_channel();
    damiao::MotorControl control(channel);
    damiao::Motor motor1(damiao::DM4310, 1, 0);
    damiao::Motor motor2(damiao::DM4310, 2, 0);
    control.add_motor(&motor1);
    control.add_motor(&motor2);

    bus->push_rx(feedback_frame(2));
    bus->push_rx(feedback_frame(1));

    EXPECT_TRUE(control.receive_feedback_for(motor1, std::chrono::milliseconds(20)));
    EXPECT_EQ(motor1.get_state_seq(), 1u);
    EXPECT_EQ(motor2.get_state_seq(), 1u);
}

TEST(DamiaoMotorControlTests, NonZeroMasterIdMustBeUnique) {
    auto bus = make_open_bus();
    auto channel = bus->create_channel();
    damiao::MotorControl control(channel);
    damiao::Motor motor1(damiao::DM4310, 1, 0x11);
    damiao::Motor motor2(damiao::DM4310, 2, 0x11);

    control.add_motor(&motor1);
    EXPECT_THROW(control.add_motor(&motor2), std::invalid_argument);
}

TEST(DamiaoMotorControlTests, TargetedParameterReceiveMatchesSlaveAndRegister) {
    auto bus = make_open_bus();
    auto channel = bus->create_channel();
    damiao::MotorControl control(channel);
    damiao::Motor motor1(damiao::DM4310, 1, 0);
    damiao::Motor motor2(damiao::DM4310, 2, 0);
    control.add_motor(&motor1);
    control.add_motor(&motor2);

    bus->push_rx(parameter_frame(2, damiao::PARAM_READ_CMD, damiao::CTRL_MODE, 24));
    bus->push_rx(parameter_frame(1, damiao::PARAM_READ_CMD, damiao::MST_ID, 11));
    bus->push_rx(parameter_frame(1, damiao::PARAM_READ_CMD, damiao::CTRL_MODE, 42));

    const float value = control.read_motor_param(motor1, damiao::CTRL_MODE, std::chrono::milliseconds(20));
    EXPECT_EQ(value, 42.0f);
    EXPECT_EQ(motor1.get_param_as_uint32(damiao::CTRL_MODE), 42u);
    EXPECT_FALSE(motor2.has_param(damiao::CTRL_MODE));
    EXPECT_EQ(bus->tx_size(), 1u);
}
