#pragma once

#include <chrono>

namespace serial_arm::calibration_reference_watchdog {

// Only protects a LIVE, otherwise healthy control cycle during replay following
// pauses. Never replaces the independent native command/state safety watchdogs.
class Heartbeat {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    static constexpr auto refresh_interval = std::chrono::milliseconds(25);
    static constexpr auto feedback_deadline = std::chrono::milliseconds(65);
    static constexpr auto catchup_timeout = std::chrono::milliseconds(1200);
    static constexpr auto poll_interval = std::chrono::milliseconds(15);

    explicit Heartbeat(TimePoint initial) noexcept
        : began_(initial), last_sent_(initial), last_feedback_(initial) {}

    void command_sent(TimePoint now) noexcept { last_sent_ = now; }
    void control_cycle_received(TimePoint now) noexcept {
        feedback_received_ = true;
        last_feedback_ = now;
    }
    bool feedback_received() const noexcept { return feedback_received_; }
    bool feedback_stalled(TimePoint now) const noexcept {
        return now - last_feedback_ > feedback_deadline;
    }
    bool catchup_expired(TimePoint now) const noexcept {
        return now - began_ >= catchup_timeout;
    }
    bool refresh_due(TimePoint now) const noexcept {
        return now - last_sent_ >= refresh_interval;
    }

private:
    TimePoint began_;
    TimePoint last_sent_;
    TimePoint last_feedback_;
    bool feedback_received_{false};
};

}  // namespace serial_arm::calibration_reference_watchdog
