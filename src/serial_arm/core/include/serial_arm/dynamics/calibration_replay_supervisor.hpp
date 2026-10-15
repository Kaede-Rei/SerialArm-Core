#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace serial_arm::calibration_replay_supervisor {
// A soft path governor; the native joint, motor and command watchdog limits
// remain unchanged.  Soft limits are strictly BELOW the historical following
// tolerance.  Do not interpret these as general-purpose collision safety.
enum class State { FOLLOWING, PAUSE_FOR_FEEDBACK };
struct Assessment {
    State state{State::PAUSE_FOR_FEEDBACK};
    bool valid{false};
    bool hard_violation{false};
    std::size_t worst_joint{0};
    double max_error{0.0};
    double peak_velocity{0.0};
    double recovery_velocity_limit{0.0};
    std::vector<double> soft_limit;
    std::vector<double> hard_limit;
};
inline Assessment evaluate(const std::vector<double>& reference,
                           const std::vector<double>& actual,
                           const std::vector<double>& velocity,
                           const std::vector<double>& max_velocity,
                           double pause_soft_ratio=1.0) {
    Assessment result;
    const auto count=reference.size();
    if(count==0 || actual.size()!=count || velocity.size()!=count || max_velocity.size()!=count ||
       !std::isfinite(pause_soft_ratio) || pause_soft_ratio<1.0)
        return result;
    result.soft_limit.resize(count);
    result.hard_limit.resize(count);
    result.state=State::FOLLOWING;
    double most_excess=-1.0;
    for(std::size_t i=0;i<count;++i) {
        if(!std::isfinite(reference[i])||!std::isfinite(actual[i])||
           !std::isfinite(velocity[i])||!std::isfinite(max_velocity[i])||max_velocity[i]<=0)
            return result;
        const double hard=std::clamp(max_velocity[i]*0.10,0.035,0.14);
        const double soft=0.42*hard;
        const double error=std::abs(reference[i]-actual[i]);
        result.hard_limit[i]=hard;
        result.soft_limit[i]=soft;
        result.peak_velocity=std::max(result.peak_velocity,std::abs(velocity[i]));
        result.recovery_velocity_limit=std::max(result.recovery_velocity_limit,
                                                std::min(0.25,0.07*max_velocity[i]));
        const double excess=error/soft;
        if(excess>most_excess){most_excess=excess;result.worst_joint=i;result.max_error=error;}
        // During frozen verification the instant error allowance must match
        // the verification metric. Do not treat its softer diagnostic
        // threshold as a hard actuator limit. Remain below the real hard bound.
        const double pause_limit=std::min(soft*pause_soft_ratio,0.80*hard);
        if(error>pause_limit)result.state=State::PAUSE_FOR_FEEDBACK;
        if(error>=hard)result.hard_violation=true;
    }
    result.valid=true;
    return result;
}
// Slow the SEARCH trajectory before the soft following limit is reached
// This governor does not run during frozen validation and cannot relax limits
inline double search_progress_scale(const Assessment& assessment) noexcept {
    if(!assessment.valid || assessment.soft_limit.empty())return 0.0;
    const double soft=assessment.soft_limit[assessment.worst_joint];
    if(!std::isfinite(soft) || soft<=0.0)return 0.0;
    const double ratio=assessment.max_error/soft;
    if(!std::isfinite(ratio) || ratio<0.0)return 0.0;
    // No slowdown for good tracking, smooth reduction approaching the limit
    return std::clamp((1.0-ratio)/0.70,0.20,1.0);
}
// Keep the governor from introducing a new acceleration step on the
// already retimed path; reserve acceleration headroom for curvature
inline double search_governor_slew_limit(const std::vector<double>& max_velocity,
                                         const std::vector<double>& max_acceleration,
                                         double frequency) noexcept {
    if(max_velocity.empty() || max_velocity.size()!=max_acceleration.size() ||
       !std::isfinite(frequency) || frequency<=0.0)return 0.0;
    double limit=1.0;
    for(std::size_t i=0;i<max_velocity.size();++i) {
        if(!std::isfinite(max_velocity[i]) || !std::isfinite(max_acceleration[i]) ||
           max_velocity[i]<=0.0 || max_acceleration[i]<=0.0)return 0.0;
        limit=std::min(limit,0.12*max_acceleration[i] /
            (0.94*max_velocity[i]*frequency));
    }
    return std::clamp(limit,0.0,1.0);
}
// The user trajectory may have a nonzero velocity at the sampled reference;
// as soon as path time stops, that derivative is no longer a valid setpoint.
inline std::vector<double> command_velocity(const std::vector<double>& requested, bool waiting) {
    return waiting ? std::vector<double>(requested.size(), 0.0) : requested;
}
// Update solely in response to measured following errors. The initial
// dynamics-based gains are only a starting estimate. Actuator limits only CAP
// the revision; they never supply a 'target percentage' or a guessed optimum.
// Gains must be applied with a continuous ramp by Robot.
inline bool revise_tracking(std::vector<double>& kp,std::vector<double>& kd,
                            const Assessment& assessment,
                            const std::vector<double>& reference,
                            const std::vector<double>& actual,
                            const std::vector<double>& velocity,
                            const std::vector<double>& max_kp,
                            const std::vector<double>& max_kd) {
    const auto n=reference.size();
    if(!assessment.valid || kp.size()!=n || kd.size()!=n || actual.size()!=n ||
       velocity.size()!=n || max_kp.size()!=n || max_kd.size()!=n ||
       assessment.soft_limit.size()!=n)return false;
    bool changed=false;
    for(std::size_t i=0;i<n;++i){
        const double e=reference[i]-actual[i];
        const double soft=assessment.soft_limit[i];
        if(!std::isfinite(e)||!std::isfinite(velocity[i])||!std::isfinite(kp[i])||
           !std::isfinite(kd[i])||!std::isfinite(max_kp[i])||!std::isfinite(max_kd[i])||
           max_kp[i]<=0||max_kd[i]<=0)return false;
        // Relative-error deadband prevents permanently hardening otherwise
        // healthy joints. A drifting joint gets more position authority and
        // braking. No response -> eventually stop rather than guessing harder.
        const double normalized=std::abs(e)/soft;
        if(normalized<0.65)continue;
        const double growth=1.0+std::min(0.80,0.40*normalized);
        const double next_kp=std::min(max_kp[i],kp[i]*growth);
        // Velocity damping is increased only if the joint is actually moving
        // away from its target, or is still moving rapidly near it.
        const bool moving_away=(actual[i]-reference[i])*velocity[i]>0.0;
        const double next_kd=moving_away ? std::min(max_kd[i],kd[i]*1.35) : kd[i];
        changed=changed || next_kp>kp[i]+1e-7 || next_kd>kd[i]+1e-7;
        kp[i]=next_kp;kd[i]=next_kd;
    }
    return changed;
}
// Compliant calibration is constrained by the measured or configured rigid
// reference rather than the motor ceiling. A gain set that has no meaningful
// stiffness separation is not a valid compliant candidate.
struct CompliantBounds {
    std::vector<double> kp_ceiling;
    std::vector<double> kd_ceiling;
    bool valid{false};
};
inline CompliantBounds compliant_bounds(const std::vector<double>& rigid_kp,
                                       const std::vector<double>& rigid_kd,
                                       const std::vector<double>& safety_kp,
                                       const std::vector<double>& safety_kd) {
    CompliantBounds bounds;
    const auto n=rigid_kp.size();
    if(n==0 || rigid_kd.size()!=n || safety_kp.size()!=n || safety_kd.size()!=n)return bounds;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(rigid_kp[i]) || !std::isfinite(rigid_kd[i]) ||
           !std::isfinite(safety_kp[i]) || !std::isfinite(safety_kd[i]) ||
           rigid_kp[i]<=0.0 || rigid_kd[i]<=0.0 || safety_kp[i]<=0.0 || safety_kd[i]<=0.0)
            return bounds;
        bounds.kp_ceiling.push_back(std::min(safety_kp[i],0.65*rigid_kp[i]));
        bounds.kd_ceiling.push_back(std::min(safety_kd[i],0.85*rigid_kd[i]));
    }
    bounds.valid=true;
    return bounds;
}
inline bool within_compliant_bounds(const std::vector<double>& kp,
                                    const std::vector<double>& kd,
                                    const CompliantBounds& bounds) {
    if(!bounds.valid || kp.size()!=bounds.kp_ceiling.size() || kd.size()!=kp.size())return false;
    for(std::size_t i=0;i<kp.size();++i)
        if(!std::isfinite(kp[i]) || !std::isfinite(kd[i]) || kp[i]<0 || kd[i]<0 ||
           kp[i]>bounds.kp_ceiling[i]+1e-8 || kd[i]>bounds.kd_ceiling[i]+1e-8)
            return false;
    return true;
}
// Only fresh feedback changes compliant gains. Healthy tracking slowly probes
// lower stiffness while increased position error requests just enough recovery
// authority, never exceeding the rigid-relative ceiling. No force measurement
// is implied by this purely position-domain search.
inline bool revise_compliant(std::vector<double>& kp,std::vector<double>& kd,
                             const Assessment& assessment,
                             const std::vector<double>& reference,
                             const std::vector<double>& actual,
                             const std::vector<double>& velocity,
                             const CompliantBounds& bounds) {
    const auto n=reference.size();
    if(!assessment.valid || !bounds.valid || kp.size()!=n || kd.size()!=n ||
       actual.size()!=n || velocity.size()!=n || assessment.soft_limit.size()!=n ||
       bounds.kp_ceiling.size()!=n || bounds.kd_ceiling.size()!=n)return false;
    bool changed=false;
    for(std::size_t i=0;i<n;++i) {
        const double soft=assessment.soft_limit[i];
        if(!(soft>0) || !std::isfinite(reference[i]) || !std::isfinite(actual[i]) ||
           !std::isfinite(velocity[i]) || !std::isfinite(kp[i]) || !std::isfinite(kd[i]))return false;
        const double ratio=std::abs(reference[i]-actual[i])/soft;
        double next_kp=kp[i],next_kd=kd[i];
        if(ratio<0.35) {
            // Probe a softer candidate only while the joint has error margin
            next_kp=std::max(0.01,kp[i]*0.985);
        } else if(ratio>0.85) {
            next_kp=std::min(bounds.kp_ceiling[i],kp[i]*(1.0+0.25*std::min(ratio,2.0)));
            const bool moving_away=(actual[i]-reference[i])*velocity[i]>0.0;
            if(moving_away)next_kd=std::min(bounds.kd_ceiling[i],kd[i]*1.12);
        }
        next_kp=std::min(next_kp,bounds.kp_ceiling[i]);
        next_kd=std::min(next_kd,bounds.kd_ceiling[i]);
        changed |= std::abs(next_kp-kp[i])>1e-7 || std::abs(next_kd-kd[i])>1e-7;
        kp[i]=next_kp;kd[i]=next_kd;
    }
    return changed;
}
} // namespace serial_arm::calibration_replay_supervisor
