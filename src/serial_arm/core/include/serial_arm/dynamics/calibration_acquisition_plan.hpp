#pragma once
#include <array>
#include <cstddef>

namespace serial_arm::calibration_acquisition {
// Each reverse/forward pair starts and ends at the demonstrated replay start.
// Rates are fractions of the already retimed, safety-checked nominal path
struct DynamicPass { bool reverse; int speed_tier; double rate; };
inline constexpr std::array<DynamicPass,4> dynamic_passes{{
    {true,1,0.5},{false,1,0.5},{true,2,1.0},{false,2,1.0}
}};
inline constexpr double traversal_factor = 8.0; // two static + 2 slow + 2 fast
inline double estimated_seconds(double one_way_s,std::size_t poses,double dwell_s) {
    return traversal_factor*one_way_s+2.0*static_cast<double>(poses)*dwell_s+2.0;
}
inline constexpr std::size_t progress_units(std::size_t poses) { return 2*poses+6; }
}
