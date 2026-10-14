#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>

namespace serial_arm::calibration_gain_adaptation {
// Statistics from actual motor feedback while replaying the demonstrated
// path. A new calibration task NEVER infers suitable gains from user Kp/Kd.
struct PassMetrics {
    std::size_t samples{0};
    std::size_t pauses{0};
    std::size_t revisions{0};
    double error_ratio_sum{0.0};
    double worst_error_ratio{0.0};
    void observe(double error_to_soft_limit) {
        if(!std::isfinite(error_to_soft_limit)){
            worst_error_ratio=1.0e30;
            return;
        }
        ++samples;
        error_ratio_sum+=error_to_soft_limit;
        worst_error_ratio=std::max(worst_error_ratio,error_to_soft_limit);
    }
};
// Tolerances are dimensionless tracking quality relative to the *existing*
// conservative soft following limits, not fractions of actuator Kp/Kd.
// A full-speed round trip with gains frozen throughout must pass this test.
// Search trips are never validation data.
inline bool quality_acceptable(const PassMetrics& metrics) {
    return metrics.samples>=60 && metrics.revisions==0 &&
        metrics.pauses<=std::max<std::size_t>(2,metrics.samples/100) &&
        metrics.error_ratio_sum/static_cast<double>(metrics.samples)<=0.70 &&
        metrics.worst_error_ratio<=1.50;
}
// Independent gain calibration must be a short, bounded operation.  The
// First search traversal runs at 80 percent, the return runs at full speed
// The second round trip freezes the candidate and verifies at full speed
// Four traversals are bounded and safety limits remain unchanged
inline constexpr std::array<double,2> round_trip_rates{0.80,1.0};
inline constexpr std::size_t max_traversals() noexcept {
    return round_trip_rates.size()*2;
}
inline constexpr bool is_validation_round_trip(std::size_t round_trip_index) noexcept {
    return round_trip_index+1==round_trip_rates.size();
}
inline double worst_case_pass_factor() {
    double factor=0.0;
    factor=1.0/round_trip_rates.front()+1.0/round_trip_rates.back()+
        2.0/round_trip_rates.back();
    return factor;
}
} // namespace serial_arm::calibration_gain_adaptation
