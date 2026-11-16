#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace serial_arm {

inline constexpr double kParkFollowingPositionRatio=2.0;

// Reduce only the park reference speed if the controller falls behind.
// At the hard lag threshold it decelerates; it never accelerates the reference
inline double park_reference_speed_fraction(double reference, double measured, double position_tolerance) {
    if(!std::isfinite(reference) || !std::isfinite(measured) ||
       !std::isfinite(position_tolerance) || position_tolerance <= 0.0) return 0.0;
    // Follow normally within the configured per-axis settling tolerance;
    // brake at twice that tolerance, independently of rotary or linear units
    return std::clamp((kParkFollowingPositionRatio-std::abs(reference-measured)/position_tolerance)/(kParkFollowingPositionRatio-1.0),0.0,1.0);
}

} // namespace serial_arm
