#pragma once

#include <chrono>
#include <optional>

namespace serial_arm::calibration_reference_watchdog {

// Only protects a LIVE, otherwise healthy control cycle during replay following
// pauses. Never replaces the independent native command/state safety watchdogs.
class Heartbeat {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    static constexpr auto refresh_interval = std::chrono::milliseconds(25);
    static constexpr auto feedback_deadline = std::chrono::milliseconds(65);
    // Recovery begins on entering the soft-limit pause, never on reference dispatch
    // Adaptive search allows bounded gain-ramp settling without weakening hard safety
    static constexpr auto frozen_catchup_timeout = std::chrono::milliseconds(1200);
    static constexpr auto adaptive_catchup_timeout = std::chrono::milliseconds(3000);
    static constexpr auto poll_interval = std::chrono::milliseconds(15);

    explicit Heartbeat(TimePoint initial) noexcept
        : last_sent_(initial), last_feedback_(initial) {}

    void command_sent(TimePoint now) noexcept { last_sent_ = now; }
    void control_cycle_received(TimePoint now) noexcept {
        feedback_received_ = true;
        last_feedback_ = now;
    }
    bool feedback_received() const noexcept { return feedback_received_; }
    bool feedback_stalled(TimePoint now) const noexcept {
        return now - last_feedback_ > feedback_deadline;
    }
    void begin_catchup(TimePoint now) noexcept {
        if(!catchup_started_)catchup_started_=now;
    }
    bool catchup_expired(TimePoint now,bool adaptive) const noexcept {
        return catchup_started_.has_value() && now-*catchup_started_ >=
            (adaptive ? adaptive_catchup_timeout : frozen_catchup_timeout);
    }
    double catchup_elapsed_s(TimePoint now) const noexcept {
        if(!catchup_started_)return 0.0;
        return std::chrono::duration<double>(now-*catchup_started_).count();
    }
    bool refresh_due(TimePoint now) const noexcept {
        return now - last_sent_ >= refresh_interval;
    }

private:
    std::optional<TimePoint> catchup_started_;
    TimePoint last_sent_;
    TimePoint last_feedback_;
    bool feedback_received_{false};
};

}  // namespace serial_arm::calibration_reference_watchdog
