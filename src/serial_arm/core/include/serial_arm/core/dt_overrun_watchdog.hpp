#pragma once

#include <cmath>
#include <cstddef>

namespace serial_arm {

// Only a finite, positive cycle over the configured maximum contributes to
// the consecutive-overrun count. Invalid/negative dt remains an immediate fault
// and all other safety checks remain independent.
class DtOverrunWatchdog {
public:
    static constexpr std::size_t kConsecutiveLimit = 3;

    bool fault_required(double dt, double maximum_dt) noexcept {
        if(!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(maximum_dt) || maximum_dt <= 0.0) {
            consecutive_ = 0;
            return true;
        }
        if(dt <= maximum_dt) {
            consecutive_ = 0;
            return false;
        }
        if(consecutive_ < kConsecutiveLimit) ++consecutive_;
        return consecutive_ >= kConsecutiveLimit;
    }

    void reset() noexcept { consecutive_ = 0; }
    std::size_t consecutive() const noexcept { return consecutive_; }

private:
    std::size_t consecutive_{ 0 };
};

} // namespace serial_arm
