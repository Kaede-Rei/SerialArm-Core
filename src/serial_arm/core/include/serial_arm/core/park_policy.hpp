#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace serial_arm {

// Reduce only the park reference speed if the soft controller falls behind.
// At the hard lag threshold it decelerates; it never accelerates the reference.
inline double park_reference_speed_fraction(double reference, double measured) {
    constexpr double soft_lag = 0.08;
    constexpr double hard_lag = 0.18;
    if(!std::isfinite(reference) || !std::isfinite(measured)) return 0.0;
    return std::clamp((hard_lag - std::abs(reference - measured)) /
                      (hard_lag - soft_lag), 0.0, 1.0);
}

// During the timed 75/25 switch, check actual motion only: the reference
// is deliberately reset to measured position after changing impedance mode.
// Never switch to high gain while the robot is far away, moving too fast,
// or while any joint feedback is invalid.
inline bool park_safe_for_timed_rigid(const std::vector<double>& measured,
                                     const std::vector<double>& measured_vel,
                                     const std::vector<double>& target,
                                     double position_tolerance,
                                     double velocity_tolerance,
                                     double relaxed_ratio) {
    if(measured.empty() || measured.size()!=target.size() ||
       measured_vel.size()!=target.size() ||
       !std::isfinite(position_tolerance) || !std::isfinite(velocity_tolerance) ||
       !std::isfinite(relaxed_ratio) || position_tolerance<=0.0 ||
       velocity_tolerance<=0.0 || relaxed_ratio<1.0) return false;
    const double pos_bound=std::min(position_tolerance*relaxed_ratio, 0.20);
    const double vel_bound=std::min(velocity_tolerance*relaxed_ratio, 0.12);
    for(std::size_t i=0;i<target.size();++i) {
        if(!std::isfinite(measured[i]) || !std::isfinite(measured_vel[i]) ||
           !std::isfinite(target[i]) ||
           std::abs(measured[i]-target[i])>pos_bound ||
           std::abs(measured_vel[i])>vel_bound) return false;
    }
    return true;
}

inline bool park_time_to_rigid(double elapsed_s, double timeout_s, double compliant_fraction) {
    return std::isfinite(elapsed_s) && std::isfinite(timeout_s) &&
           std::isfinite(compliant_fraction) && timeout_s>0.0 &&
           compliant_fraction>0.0 && compliant_fraction<1.0 &&
           elapsed_s>=timeout_s*compliant_fraction;
}

} // namespace serial_arm
