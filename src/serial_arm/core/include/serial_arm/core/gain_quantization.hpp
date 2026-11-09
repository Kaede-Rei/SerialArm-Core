#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace serial_arm {

// Reject an illegal requested gain before selecting the nearest encodable
// protocol value. A zero resolution leaves a continuous backend unchanged.
inline std::optional<double> project_protocol_gain(double requested,double maximum,double resolution) {
    if(!std::isfinite(requested) || requested<0 || !std::isfinite(maximum) || maximum<0 || requested>maximum ||
       !std::isfinite(resolution) || resolution<0)return std::nullopt;
    if(resolution==0)return requested;
    const double maximum_code=std::floor(maximum/resolution+1e-9);
    return std::min(maximum,std::min(std::round(requested/resolution),maximum_code)*resolution);
}

} // namespace serial_arm
