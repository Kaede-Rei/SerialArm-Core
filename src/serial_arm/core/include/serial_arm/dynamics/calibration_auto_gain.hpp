#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace serial_arm::calibration_auto_gain {
struct Gains { std::vector<double> kp, kd; };
// Mass matrix diagonal has units kg.m²; d(gravity)/dq has units Nm/rad.
// The safety values are CEILINGS, not tuning targets or percentage seeds.
inline Gains from_model(const std::vector<double>& inertia, const std::vector<double>& gravity_gradient,
                        const std::vector<double>& kp_ceiling, const std::vector<double>& kd_ceiling,
                        double natural_frequency=5.0) {
    const auto n=inertia.size();
    if(!n || gravity_gradient.size()!=n || kp_ceiling.size()!=n || kd_ceiling.size()!=n ||
       !std::isfinite(natural_frequency) || natural_frequency<=0) throw std::invalid_argument("invalid calibration model dimensions");
    Gains result;
    for(std::size_t i=0;i<n;++i) {
        const double mass=inertia[i], stiffness=std::abs(gravity_gradient[i]);
        if(!std::isfinite(mass)||mass<=1e-8||!std::isfinite(stiffness)||
           !std::isfinite(kp_ceiling[i])||kp_ceiling[i]<=0||!std::isfinite(kd_ceiling[i])||kd_ceiling[i]<=0)
            throw std::invalid_argument("invalid model inertia or actuator safety bound");
        const double kp=mass*natural_frequency*natural_frequency+stiffness;
        const double kd=2.3*std::sqrt(mass*kp);
        result.kp.push_back(std::min(kp,kp_ceiling[i]));
        result.kd.push_back(std::min(kd,kd_ceiling[i]));
    }
    return result;
}
// Observe oscillation and slow drift instead of scaling the user's nominal gains.
// A failed sample may prompt a bounded response change; successful adaptation
// is NEVER inferred solely from reaching a larger stiffness.
inline bool revise(Gains& target, const std::vector<double>& inertia,
                   const std::vector<double>& kp_ceiling, const std::vector<double>& kd_ceiling,
                   const std::vector<double>& torque_std, const std::vector<double>& pos_drift,
                   const std::vector<double>& velocity_peak) {
    if(target.kp.size()!=inertia.size()||target.kd.size()!=inertia.size() ||
       torque_std.size()!=inertia.size()||pos_drift.size()!=inertia.size()||velocity_peak.size()!=inertia.size()) return false;
    bool changed=false;
    for(std::size_t i=0;i<inertia.size();++i) {
        // Per-joint tuning is based on measured excess motion/effort variance.
        const bool oscillating=torque_std[i]>0.045 && velocity_peak[i]>0.007;
        const bool slipping=pos_drift[i]>0.003 || velocity_peak[i]>0.025;
        const double mass=std::max(1e-7,inertia[i]);
        double kp=target.kp[i],kd=target.kd[i];
        if(oscillating) {
            // Reduce oscillatory stiffness first; raise damping if actuator permits.
            kp=std::max(0.01,kp/(1.0+0.35*std::min(3.0,torque_std[i]/0.045)));
            kd=std::min(kd_ceiling[i],kd+0.65*std::sqrt(mass*target.kp[i]));
        } else if(slipping) {
            const double wn=std::sqrt(std::max(1e-7,kp)/mass);
            kp=std::min(kp_ceiling[i],kp+mass*wn*wn*0.20);
            kd=std::min(kd_ceiling[i],std::max(kd,2.1*std::sqrt(mass*kp)));
        }
        kp=std::clamp(kp,0.01,kp_ceiling[i]);
        kd=std::clamp(kd,0.0,kd_ceiling[i]);
        changed|=std::abs(kp-target.kp[i])>1e-5||std::abs(kd-target.kd[i])>1e-5;
        target.kp[i]=kp;target.kd[i]=kd;
    }
    return changed;
}
} // namespace serial_arm::calibration_auto_gain
