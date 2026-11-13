#pragma once

#include "serial_arm/dynamics/calibration_retime.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {

// A visit is a time map of the original checked Hermite path, not a new
// joint-space line. Indices identify a particular traversal of repeated poses.
// Its cost is automatic-stage wall time; it never replaces the recorded T.
struct TaughtAnchorVisitPlan {
    bool valid{false};
    std::string reason;
    std::vector<std::vector<double>> positions;
    std::size_t from_source_index{0}, to_source_index{0};
    double sample_dt{0}, source_start_s{0}, source_end_s{0};
    double source_distance_s{0}, direction{1};
    double source_rate{0}, ramp_s{0}, cruise_s{0}, duration_s{0};
    double round_trip_cost_s{0};
};

struct TaughtAnchorVisitSample {
    bool valid{false}, complete{false};
    std::vector<double> position, velocity, acceleration;
    double source_time_s{0}, source_progress{0}, source_rate{0}, source_acceleration{0};
    std::size_t source_index{0};
};

// Bounds are Bernstein convex-hull bounds for each *existing* Hermite
// segment. Velocity/acceleration constraints are supplied by user Safety;
// neither model gain initialization nor inferred hard stops enter this plan.
inline TaughtAnchorVisitPlan plan_taught_anchor_visit(
    const std::vector<std::vector<double>>& checked_positions, double sample_dt,
    std::size_t from_source_index, std::size_t to_source_index,
    const std::vector<double>& max_velocity, const std::vector<double>& max_acceleration) {
    TaughtAnchorVisitPlan out;
    const auto joints=max_velocity.size();
    if(checked_positions.size()<2 || !joints || max_acceleration.size()!=joints ||
       !std::isfinite(sample_dt) || sample_dt<=0 ||
       from_source_index>=checked_positions.size() || to_source_index>=checked_positions.size()) {
        out.reason="taught_anchor_visit_mapping_invalid";return out;
    }
    for(std::size_t j=0;j<joints;++j)
        if(!std::isfinite(max_velocity[j]) || max_velocity[j]<=0 ||
           !std::isfinite(max_acceleration[j]) || max_acceleration[j]<=0) {
            out.reason="taught_anchor_visit_safety_limits_invalid";return out;
        }
    for(const auto& q:checked_positions) {
        if(q.size()!=joints) {out.reason="taught_anchor_visit_mapping_invalid";return out;}
        for(double value:q)if(!std::isfinite(value)) {
            out.reason="taught_anchor_visit_position_invalid";return out;
        }
    }
    out.positions=checked_positions;out.sample_dt=sample_dt;
    out.from_source_index=from_source_index;out.to_source_index=to_source_index;
    out.source_start_s=from_source_index*sample_dt;out.source_end_s=to_source_index*sample_dt;
    out.direction=to_source_index>=from_source_index?1:-1;
    out.source_distance_s=std::abs(out.source_end_s-out.source_start_s);
    if(!std::isfinite(out.source_start_s) || !std::isfinite(out.source_end_s)) {
        out.reason="taught_anchor_visit_time_invalid";return out;
    }
    if(from_source_index==to_source_index) {
        out.valid=true;out.reason="already_at_source_anchor";return out;
    }
    std::vector<double> velocity_bound(joints,0),acceleration_bound(joints,0);
    const auto lower=std::min(from_source_index,to_source_index);
    const auto upper=std::max(from_source_index,to_source_index);
    for(std::size_t k=lower;k<upper;++k)for(std::size_t j=0;j<joints;++j) {
        const double d=checked_positions[k+1][j]-checked_positions[k][j];
        const double m0=calibration_retime::node_tangent(checked_positions,k,j);
        const double m1=calibration_retime::node_tangent(checked_positions,k+1,j);
        velocity_bound[j]=std::max(velocity_bound[j],
            std::max({std::abs(m0),std::abs(3*d-m0-m1),std::abs(m1)})/sample_dt);
        acceleration_bound[j]=std::max(acceleration_bound[j],
            std::max(std::abs(6*d-4*m0-2*m1),std::abs(-6*d+2*m0+4*m1))/(sample_dt*sample_dt));
    }
    // Quintic rate ramps: h(u)=10u^3-15u^4+6u^5, max h'=15/8.
    // qddot = q_ss * source_rate^2 + q_s * source_acceleration.
    // Select the shortest feasible member of this fixed time-map family;
    // this searches only execution time, never gains or robot-specific values.
    constexpr double minimum_ramp_s=1.0, maximum_rate_derivative=1.875;
    double maximum_rate=std::min(1.0,out.source_distance_s/minimum_ramp_s);
    for(std::size_t j=0;j<joints;++j) {
        if(velocity_bound[j]>0)maximum_rate=std::min(maximum_rate,max_velocity[j]/velocity_bound[j]);
        if(acceleration_bound[j]>0)maximum_rate=std::min(maximum_rate,
            std::sqrt(max_acceleration[j]/acceleration_bound[j])*(1-1e-12));
    }
    double best=std::numeric_limits<double>::infinity();
    for(unsigned choice=1;choice<=64;++choice) {
        const double rate=maximum_rate*choice/64.0;
        if(!(rate>0) || !std::isfinite(rate))continue;
        double ramp=minimum_ramp_s;
        for(std::size_t j=0;j<joints;++j) {
            const double remaining=max_acceleration[j]-acceleration_bound[j]*rate*rate;
            if(!(remaining>0)) {ramp=std::numeric_limits<double>::infinity();break;}
            ramp=std::max(ramp,maximum_rate_derivative*rate*velocity_bound[j]/remaining);
        }
        if(!std::isfinite(ramp) || rate*ramp>out.source_distance_s)continue;
        const double cruise=std::max(0.0,out.source_distance_s/rate-ramp);
        const double duration=2*ramp+cruise;
        if(duration<best) {
            best=duration;out.source_rate=rate;out.ramp_s=ramp;out.cruise_s=cruise;out.duration_s=duration;
        }
    }
    if(!std::isfinite(best)) {out.reason="taught_anchor_visit_time_plan_unavailable";return out;}
    out.round_trip_cost_s=2*out.duration_s;
    out.valid=std::isfinite(out.round_trip_cost_s);
    out.reason=out.valid?"original_taught_path_visit_planned":"taught_anchor_visit_time_invalid";
    return out;
}

// Does not inspect actual tracking error. Fresh actual feedback determines
// subsequent measurement room; it never changes this path's branch identity.
inline bool taught_anchor_visit_fits_budget(const TaughtAnchorVisitPlan& plan,
    double remaining_s,double measurement_s,double exit_reserve_s) {
    return plan.valid && std::isfinite(remaining_s) && std::isfinite(measurement_s) &&
        std::isfinite(exit_reserve_s) && remaining_s>=0 && measurement_s>=0 && exit_reserve_s>=0 &&
        plan.round_trip_cost_s+measurement_s+exit_reserve_s<=remaining_s;
}

inline TaughtAnchorVisitSample sample_taught_anchor_visit(const TaughtAnchorVisitPlan& plan,double elapsed_s) {
    TaughtAnchorVisitSample out;
    if(!plan.valid || !std::isfinite(elapsed_s))return out;
    const double t=std::clamp(elapsed_s,0.0,plan.duration_s);
    double distance=0,rate=0,acceleration=0;
    const auto ramp=[](double u) {return u*u*u*(10+u*(-15+6*u));};
    const auto ramp_derivative=[](double u) {return 30*u*u*(1-u)*(1-u);};
    const auto ramp_integral=[](double u) {return u*u*u*u*(2.5+u*(-3+u));};
    if(plan.duration_s>0 && t<plan.duration_s) {
        if(t<plan.ramp_s) {
            const double u=t/plan.ramp_s;
            distance=plan.source_rate*plan.ramp_s*ramp_integral(u);
            rate=plan.source_rate*ramp(u);
            acceleration=plan.source_rate*ramp_derivative(u)/plan.ramp_s;
        } else if(t<plan.ramp_s+plan.cruise_s) {
            distance=plan.source_rate*(.5*plan.ramp_s+t-plan.ramp_s);rate=plan.source_rate;
        } else {
            const double u=(plan.duration_s-t)/plan.ramp_s;
            distance=plan.source_distance_s-plan.source_rate*plan.ramp_s*ramp_integral(u);
            rate=plan.source_rate*ramp(u);
            acceleration=-plan.source_rate*ramp_derivative(u)/plan.ramp_s;
        }
    } else distance=plan.source_distance_s;
    out.source_time_s=std::clamp(plan.source_start_s+plan.direction*distance,
        std::min(plan.source_start_s,plan.source_end_s),std::max(plan.source_start_s,plan.source_end_s));
    out.source_progress=out.source_time_s/plan.sample_dt;
    if(t==0)out.source_progress=static_cast<double>(plan.from_source_index);
    if(t>=plan.duration_s)out.source_progress=static_cast<double>(plan.to_source_index);
    out.source_rate=plan.direction*rate;out.source_acceleration=plan.direction*acceleration;
    const auto raw=calibration_retime::interpolate(plan.positions,out.source_progress,plan.sample_dt,1.0);
    out.position=raw.position;out.velocity=raw.velocity;out.acceleration.resize(raw.position.size());
    const auto lower=std::min(static_cast<std::size_t>(std::floor(out.source_progress)),plan.positions.size()-2);
    out.source_index=std::min(static_cast<std::size_t>(std::floor(out.source_progress)),plan.positions.size()-1);
    const double u=out.source_progress-static_cast<double>(lower);
    for(std::size_t j=0;j<out.position.size();++j) {
        const double d=plan.positions[lower+1][j]-plan.positions[lower][j];
        const double m0=calibration_retime::node_tangent(plan.positions,lower,j);
        const double m1=calibration_retime::node_tangent(plan.positions,lower+1,j);
        const double curvature=((6-12*u)*d+(6*u-4)*m0+(6*u-2)*m1)/(plan.sample_dt*plan.sample_dt);
        out.acceleration[j]=curvature*rate*rate+raw.velocity[j]*out.source_acceleration;
        out.velocity[j]*=out.source_rate;
    }
    out.valid=true;out.complete=elapsed_s>=plan.duration_s;
    return out;
}

} // namespace serial_arm::servo_tuning
