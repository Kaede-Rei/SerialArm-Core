#pragma once

#include "serial_arm/core/types.hpp"
#include <algorithm>
#include <cmath>
#include <deque>

namespace serial_arm {

// At low speed, estimate velocity from a short position window instead of
// interpreting signed velocity quantization codes as physical motion.
// The same state feeds dynamics, momentum and friction identification.
class ObservationVelocity {
public:
    void reset() { samples_.clear(); time_ = 0.0; }

    JointVector update(const JointVector& position, const JointVector& velocity,
                       const JointVector& resolution, double dt) {
        if(position.size() != velocity.size() || resolution.size() != velocity.size() ||
           !std::isfinite(dt) || dt <= 0.0) { reset(); return velocity; }
        if(!samples_.empty() && samples_.back().position.size() != position.size()) reset();
        time_ += dt;
        samples_.push_back({time_, position});
        while(samples_.size() > 2 && time_ - samples_[1].time >= window_s) samples_.pop_front();
        JointVector result = velocity;
        double mean_time = 0.0;
        for(const auto& sample : samples_) mean_time += sample.time;
        mean_time /= samples_.size();
        double variance = 0.0;
        for(const auto& sample : samples_) variance += (sample.time - mean_time) * (sample.time - mean_time);
        for(std::size_t i = 0; i < result.size(); ++i) {
            if(resolution[i] <= 0.0) continue;
            double slope = 0.0;
            // Center positions as well as time to avoid cancellation at a
            // fixed encoder code, independent of the absolute joint position.
            if(variance > 0.0) for(const auto& sample : samples_)
                slope += (sample.time - mean_time) * (sample.position[i] - position[i]) / variance;
            const double x = std::clamp(std::abs(velocity[i]) / resolution[i] - 3.0, 0.0, 1.0);
            const double weight = x * x * (3.0 - 2.0 * x);
            result[i] = slope + weight * (velocity[i] - slope);
        }
        return result;
    }

private:
    static constexpr double window_s = 0.1;
    struct Sample { double time; JointVector position; };
    std::deque<Sample> samples_;
    double time_{0.0};
};

} // namespace serial_arm
