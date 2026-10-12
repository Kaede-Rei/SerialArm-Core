#include "serial_arm/dynamics/calibration_reference_watchdog.hpp"
#include <cassert>
#include <chrono>

using serial_arm::calibration_reference_watchdog::Heartbeat;
using namespace std::chrono_literals;

int main() {
    auto start = Heartbeat::TimePoint{};
    Heartbeat tracking(start);
    int refreshed = 0;
    // Simulate a robot trailing its reference for a full second, with healthy
    // 200 Hz cycles: commands MUST remain fresh throughout the following pause.
    for(int milliseconds = 5; milliseconds <= 1000; milliseconds += 5) {
        auto now = start + std::chrono::milliseconds(milliseconds);
        tracking.control_cycle_received(now);
        assert(!tracking.feedback_stalled(now));
        assert(!tracking.catchup_expired(now));
        if(tracking.refresh_due(now)) {
            tracking.command_sent(now);
            ++refreshed;
        }
    }
    assert(refreshed >= 35 && refreshed <= 45);

    // Loss of control feedback is NOT masked by the reference heartbeat.
    Heartbeat stalled(start);
    stalled.command_sent(start + 25ms);
    assert(!stalled.feedback_stalled(start + 65ms));
    assert(stalled.feedback_stalled(start + 66ms));

    Heartbeat too_far(start);
    for(int i=1; i<240; ++i)
        too_far.control_cycle_received(start + std::chrono::milliseconds(i*5));
    assert(!too_far.catchup_expired(start+1199ms));
    assert(too_far.catchup_expired(start+1200ms));
}
