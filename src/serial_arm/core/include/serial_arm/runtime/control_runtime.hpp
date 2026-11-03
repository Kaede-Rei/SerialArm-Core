#pragma once

#include "serial_arm/robot.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>

namespace serial_arm {

enum class MotionSource { NONE, EXTERNAL, STREAM, REPLAY, CALIBRATION, PARK };

struct RuntimeSnapshot {
    std::uint64_t sequence{0};
    std::uint64_t monotonic_ns{0};
    RobotState state{RobotState::UNCONFIGURED};
    MotionSource source{MotionSource::NONE};
    std::optional<RobotCycleOutput> cycle;
    std::optional<RobotFault> fault;
    bool fault_hold_maintained{false};
};

struct RuntimeHooks {
    // All hooks run in the same serialized context as Robot::cycle.
    std::function<bool()> running;
    std::function<bool(Robot::TimePoint, double)> before_cycle;
    std::function<void(const RuntimeSnapshot&)> after_step;
    std::mutex* serialization_mutex{nullptr};
};

// One executor is shared by the terminal, Python session and ROS2 adapter.
// Adapters own transport and presentation, never a second Robot control loop.
class ControlRuntime {
public:
    void run(Robot& robot, double frequency_hz, double max_dt_s, const RuntimeHooks& hooks);
    RuntimeSnapshot step(Robot& robot, Robot::TimePoint now = Robot::Clock::now());
    RuntimeSnapshot snapshot() const;

    tl::expected<void, RobotFault> submit_reference(
        Robot& robot, JointCmd command, Robot::TimePoint received_at,
        MotionSource source = MotionSource::EXTERNAL);
    tl::expected<void, RobotFault> set_impedance_mode(
        Robot& robot, JointImpedanceMode mode, Robot::TimePoint now = Robot::Clock::now(), HoldTargetPolicy target_policy=HoldTargetPolicy::PRESERVE_NOMINAL);
    void clear_commands();
    MotionSource source() const;

private:
    RuntimeSnapshot step_impl(Robot& robot, Robot::TimePoint now);
    struct PendingReference { JointCmd command; Robot::TimePoint received_at; };
    mutable std::mutex command_mutex_;
    std::optional<PendingReference> pending_;
    MotionSource source_{MotionSource::NONE};
    std::optional<bool> drag_suspension_;
    mutable std::mutex snapshot_mutex_;
    RuntimeSnapshot snapshot_;
    std::atomic<bool> executing_{false};
};

// Shared bounded point-to-point progression, including the parking follow guard.
bool advance_joint_reference(
    JointVector& position, JointVector& velocity, const JointVector& target,
    const JointVector& max_velocity, const JointVector& max_acceleration,
    double speed_scale, double dt, const JointVector* measured_position = nullptr);

} // namespace serial_arm
