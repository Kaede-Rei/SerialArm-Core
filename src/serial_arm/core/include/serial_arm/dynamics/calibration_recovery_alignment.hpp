#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace serial_arm::calibration_recovery {
constexpr double kPositionToleranceRad = 0.08;
constexpr double kVelocityToleranceRadS = 0.05;
constexpr double kFeedbackMaxAgeMs = 500.0;
struct Choice {
    bool valid{false};
    bool reverse_recorded_path{false};
    bool at_endpoint{false};
    double max_error_rad{std::numeric_limits<double>::infinity()};
    double max_speed_rad_s{std::numeric_limits<double>::infinity()};
};
inline Choice select_endpoint(const std::vector<double>& actual,
                             const std::vector<double>& velocity,
                             const std::vector<double>& recorded_start,
                             const std::vector<double>& recorded_end,
                             double feedback_age_ms) {
    Choice result;
    if(actual.empty() || actual.size() != velocity.size() ||
       actual.size() != recorded_start.size() || actual.size() != recorded_end.size() ||
       !std::isfinite(feedback_age_ms) || feedback_age_ms < 0 || feedback_age_ms > kFeedbackMaxAgeMs)
        return result;
    double first=0.0, last=0.0, vmax=0.0;
    for(std::size_t i=0;i<actual.size();++i) {
        if(!std::isfinite(actual[i]) || !std::isfinite(velocity[i]) ||
           !std::isfinite(recorded_start[i]) || !std::isfinite(recorded_end[i])) return result;
        first=std::max(first,std::abs(actual[i]-recorded_start[i]));
        last=std::max(last,std::abs(actual[i]-recorded_end[i]));
        vmax=std::max(vmax,std::abs(velocity[i]));
    }
    // Reversing the candidate means its back() is the *original first sample*.
    // The worker always starts by traversing candidate in reverse.
    result.valid=true;
    result.reverse_recorded_path=first < last;
    result.max_error_rad=std::min(first,last);
    result.max_speed_rad_s=vmax;
    result.at_endpoint=result.max_error_rad<=kPositionToleranceRad && vmax<=kVelocityToleranceRadS;
    return result;
}
}
