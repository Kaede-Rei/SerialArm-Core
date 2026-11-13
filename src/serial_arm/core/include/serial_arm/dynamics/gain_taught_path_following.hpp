#pragma once

#include "serial_arm/dynamics/calibration_retime.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {

namespace taught_path_detail {
// Prepared once per immutable demonstration and limit set. Scanning all source
// knots in every control cycle would itself delay the live control owner.
struct Prepared {
    const void* path{nullptr};
    const void* points{nullptr};
    std::size_t count{0},joints{0};
    double sample_dt{0},requested_rate{0},rate_limit{0},rate_acceleration{0};
    std::vector<double> max_velocity,max_acceleration;
};
inline std::shared_ptr<const Prepared> prepare(const std::vector<std::vector<double>>& path,
    double sample_dt,const std::vector<double>& max_velocity,
    const std::vector<double>& max_acceleration,double max_rate) {
    if(path.size()<2 || path.front().empty() || !(sample_dt>0) || !std::isfinite(sample_dt) ||
       !(max_rate>0) || max_rate>1 || !std::isfinite(max_rate))return {};
    const auto n=path.front().size();
    if(max_velocity.size()!=n || max_acceleration.size()!=n)return {};
    for(std::size_t i=0;i<n;++i)if(!std::isfinite(max_velocity[i]) || max_velocity[i]<=0 ||
        !std::isfinite(max_acceleration[i]) || max_acceleration[i]<=0)return {};
    for(const auto& q:path) {
        if(q.size()!=n)return {};
        for(double value:q)if(!std::isfinite(value))return {};
    }
    std::vector<double> slope(n,0),curvature(n,0);
    for(std::size_t k=0;k+1<path.size();++k)for(std::size_t i=0;i<n;++i) {
        const double d=path[k+1][i]-path[k][i];
        const double m0=calibration_retime::node_tangent(path,k,i);
        const double m1=calibration_retime::node_tangent(path,k+1,i);
        const double a=6*d-4*m0-2*m1,b=-6*d+3*m0+3*m1;
        double derivative=std::max(std::abs(m0),std::abs(m1));
        if(b!=0) {
            const double u=-a/(2*b);
            if(u>0 && u<1)derivative=std::max(derivative,std::abs(m0+a*u+b*u*u));
        }
        slope[i]=std::max(slope[i],derivative/sample_dt);
        curvature[i]=std::max(curvature[i],std::max(std::abs(a),std::abs(a+2*b))/(sample_dt*sample_dt));
    }
    auto out=std::make_shared<Prepared>();out->path=&path;out->points=path.data();
    out->count=path.size();out->joints=n;out->sample_dt=sample_dt;out->requested_rate=max_rate;
    out->max_velocity=max_velocity;out->max_acceleration=max_acceleration;
    out->rate_limit=max_rate;out->rate_acceleration=2;
    // qdd = q''(phase)*rate^2 + q'(phase)*rate_dot. Independent
    // allowances keep both terms within the same supplied Safety acceleration.
    for(std::size_t i=0;i<n;++i) {
        if(slope[i]>0) {
            out->rate_limit=std::min(out->rate_limit,max_velocity[i]/slope[i]);
            out->rate_acceleration=std::min(out->rate_acceleration,.45*max_acceleration[i]/slope[i]);
        }
        if(curvature[i]>0)out->rate_limit=std::min(out->rate_limit,std::sqrt(.5*max_acceleration[i]/curvature[i]));
    }
    if(!(out->rate_limit>0) || !(out->rate_acceleration>0) ||
       !std::isfinite(out->rate_limit) || !std::isfinite(out->rate_acceleration))return {};
    return out;
}
// Shape-preserving Hermite is monotone on each joint's source interval. The
// endpoints and intervening source knots therefore bound the entire braking
// path, including a genuine out-and-back movement; no sampled peak is omitted.
inline bool stopping_path_inside(const std::vector<std::vector<double>>& path,double sample_dt,
    double first_s,double last_s,const std::vector<double>& actual,
    const std::vector<double>& lead,const std::vector<double>& uncertainty,bool reserve) {
    const auto inside=[&](const std::vector<double>& q) {
        for(std::size_t i=0;i<q.size();++i) {
            const double room=lead[i]-(reserve && !uncertainty.empty()?uncertainty[i]:0);
            if(std::abs(q[i]-actual[i])>room+1e-12)return false;
        }
        return true;
    };
    if(!inside(calibration_retime::interpolate(path,first_s/sample_dt,sample_dt,1).position) ||
       !inside(calibration_retime::interpolate(path,last_s/sample_dt,sample_dt,1).position))return false;
    const auto begin=static_cast<std::size_t>(std::floor(first_s/sample_dt))+1;
    const auto end=std::min(path.size()-1,static_cast<std::size_t>(std::floor(last_s/sample_dt)));
    for(auto k=begin;k<=end;++k)if(!inside(path[k]))return false;
    return true;
}
} // namespace taught_path_detail

struct TaughtPathProgress {
    double source_time_s{0},rate{0};
    std::vector<double> previous_velocity;
    std::shared_ptr<const taught_path_detail::Prepared> prepared;
    bool feedback_recovering{false};
};
struct TaughtPathAdvance {
    bool valid{false},complete{false},feedback_limited{false};
    TaughtPathProgress progress;
    std::vector<double> position,velocity;
    double source_duration_s{0};
    std::string reason;
};

// Follow the existing demonstration's continuous time axis, slowing its common
// phase when feedback consumes the available lead. All joints retain the same
// phase, and every q/qd is the original Hermite reference and its derivative.
// The path must remain immutable until progress is reset. This helper neither
// changes the original recording T nor supplies or extends a task deadline.
inline TaughtPathAdvance advance_taught_path(const std::vector<std::vector<double>>& path,
    double sample_dt,TaughtPathProgress state,const std::vector<double>& actual,
    const std::vector<double>& lead,const std::vector<double>& max_velocity,
    const std::vector<double>& max_acceleration,double dt,
    const std::vector<double>& uncertainty={},double max_rate=.65,bool strict_feedback_lead=true) {
    TaughtPathAdvance out;out.progress=state;
    const auto reject=[&](const char* why){out.reason=why;return out;};
    if(!std::isfinite(dt) || dt<=0 || !std::isfinite(state.source_time_s) || state.source_time_s<0 ||
       !std::isfinite(state.rate) || state.rate<0)return reject("taught_path_progress_invalid");
    const auto n=actual.size();
    if(!n || lead.size()!=n || max_velocity.size()!=n || max_acceleration.size()!=n ||
       (!uncertainty.empty() && uncertainty.size()!=n) ||
       (!state.previous_velocity.empty() && state.previous_velocity.size()!=n))return reject("taught_path_mapping_invalid");
    for(std::size_t i=0;i<n;++i)if(!std::isfinite(actual[i]) || !std::isfinite(lead[i]) || lead[i]<=0 ||
       (!uncertainty.empty() && (!std::isfinite(uncertainty[i]) || uncertainty[i]<0 || uncertainty[i]>=lead[i])) ||
       (!state.previous_velocity.empty() && !std::isfinite(state.previous_velocity[i])))return reject("taught_path_feedback_or_reserve_invalid");
    auto prepared=state.prepared;
    if(!prepared || prepared->path!=&path || prepared->points!=path.data() || prepared->count!=path.size() ||
       prepared->sample_dt!=sample_dt || prepared->requested_rate!=max_rate ||
       prepared->max_velocity!=max_velocity || prepared->max_acceleration!=max_acceleration) {
        prepared=taught_path_detail::prepare(path,sample_dt,max_velocity,max_acceleration,max_rate);
        if(!prepared)return reject("taught_path_geometry_or_limits_invalid");
    }
    if(prepared->joints!=n)return reject("taught_path_mapping_invalid");
    const double duration=sample_dt*static_cast<double>(path.size()-1);
    out.source_duration_s=duration;
    if(!std::isfinite(duration) || state.source_time_s>duration+1e-12 ||
       state.rate>prepared->rate_limit+1e-12)return reject("taught_path_state_outside_limits");
    if(state.rate>0 && state.previous_velocity.empty())return reject("taught_path_previous_velocity_missing");
    const auto current=calibration_retime::interpolate(path,state.source_time_s/sample_dt,sample_dt,1);
    bool outside_lead=false;
    for(std::size_t i=0;i<n;++i)outside_lead=outside_lead || std::abs(current.position[i]-actual[i])>lead[i]+1e-12;
    if(strict_feedback_lead && outside_lead)return reject("taught_path_reference_outside_feedback_lead");
    if(state.source_time_s>=duration && state.rate==0) {
        out.valid=true;out.complete=!strict_feedback_lead || !outside_lead;out.feedback_limited=outside_lead;
        out.position=path.back();out.velocity.assign(n,0);
        out.progress.prepared=prepared;out.progress.previous_velocity=out.velocity;
        out.progress.feedback_recovering=false;
        out.reason=outside_lead?"taught_path_reference_complete_with_feedback_warning":"taught_path_complete";return out;
    }
    const double acceleration=prepared->rate_acceleration;
    const double minimum=std::max(0.0,state.rate-acceleration*dt);
    double maximum=std::min(prepared->rate_limit,state.rate+acceleration*dt);
    const double remaining=std::max(0.0,duration-state.source_time_s);
    const double discriminant=acceleration*acceleration*dt*dt-4*acceleration*dt*state.rate+8*acceleration*remaining;
    const double endpoint_rate=discriminant>=0?std::max(0.0,.5*(-acceleration*dt+std::sqrt(discriminant))):0;
    maximum=std::min(maximum,endpoint_rate);
    const auto next_phase=[&](double rate) {
        const double distance=rate==0 && state.rate<acceleration*dt ?
            state.rate*state.rate/(2*acceleration):.5*(state.rate+rate)*dt;
        return state.source_time_s+distance;
    };
    const auto can_stop=[&](double rate,bool reserve) {
        const double phase=next_phase(rate);
        const double stopped=phase+rate*rate/(2*acceleration);
        return phase<=duration+1e-12 && stopped<=duration+1e-12 &&
            taught_path_detail::stopping_path_inside(path,sample_dt,state.source_time_s,
                std::min(duration,stopped),actual,lead,uncertainty,reserve);
    };
    if(maximum+1e-12<minimum)return reject("taught_path_state_cannot_brake_before_source_endpoint");
    const bool soft_recovery=outside_lead || !can_stop(minimum,false);
    if(strict_feedback_lead && soft_recovery)
        return reject("taught_path_feedback_lead_below_required_braking_distance");
    double selected=maximum;
    bool reserve_used=false;
    if(soft_recovery) {
        // A discovery following distance is a scheduling preference, not a
        // replacement Safety limit. Feedback may move across it between two
        // samples. Brake the SAME source phase at the allowed acceleration;
        // never jump q to actual, erase velocity, or alter the measurement.
        selected=minimum;out.feedback_limited=true;
    } else if(!can_stop(selected,true)) {
        out.feedback_limited=true;
        if(!can_stop(minimum,true)) {selected=minimum;reserve_used=true;}
        else {
            double low=minimum,high=maximum;
            for(int k=0;k<36;++k) {
                const double middle=.5*(low+high);
                if(can_stop(middle,true))low=middle;else high=middle;
            }
            selected=low;
        }
    }
    double phase=next_phase(selected);
    if(selected==0 && duration-phase<=1e-12)phase=duration;
    const auto reference=calibration_retime::interpolate(path,phase/sample_dt,sample_dt,1);
    out.position=reference.position;out.velocity.resize(n);
    for(std::size_t i=0;i<n;++i) {
        out.velocity[i]=reference.velocity[i]*selected;
        const double previous=state.previous_velocity.empty()?0:state.previous_velocity[i];
        if(std::abs(out.velocity[i])>max_velocity[i]+1e-10 ||
           std::abs(out.velocity[i]-previous)>max_acceleration[i]*dt+1e-10)
            return reject("taught_path_sampled_velocity_or_acceleration_exceeded");
    }
    out.valid=true;out.complete=phase>=duration && selected==0 && (!strict_feedback_lead || !soft_recovery);
    out.progress.source_time_s=phase;out.progress.rate=selected;
    out.progress.previous_velocity=out.velocity;out.progress.prepared=prepared;
    out.progress.feedback_recovering=soft_recovery && !out.complete;
    out.reason=out.complete?(out.feedback_limited?"taught_path_reference_complete_with_feedback_warning":"taught_path_complete"):soft_recovery?
        (selected==0?"taught_path_waiting_for_soft_feedback_recovery":"taught_path_braking_for_soft_feedback_recovery"):
        state.feedback_recovering?"taught_path_soft_feedback_recovered":reserve_used?"taught_path_braking_with_feedback_reserve":
        out.feedback_limited?"feedback_bounded_taught_path":"continuous_taught_path";
    return out;
}
} // namespace serial_arm::servo_tuning
