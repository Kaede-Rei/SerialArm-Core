#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {

struct TaughtMeasurementRoomRequest {
    // Required *one-sided* displacement for the proposed small-signal test.
    // These are experimental requirements, never inferred mechanical limits.
    std::vector<double> required_room_rad;
    std::vector<double> observed_position_step_rad, stationary_noise_rms_rad;
    std::vector<bool> requested_axes;
    std::vector<std::size_t> preferred_source_indices;
};

struct TaughtMeasurementRoom {
    bool valid{false}, any_measurable{false};
    std::vector<double> negative_room_rad, positive_room_rad, required_room_rad;
    std::vector<bool> negative_measurable, positive_measurable, bidirectional_measurable;
    std::vector<std::string> axis_reasons;
    std::string reason;
};

struct TaughtMeasurementAnchor {
    bool available{false}, preferred{false};
    std::size_t source_index{0}, measurable_axis_count{0};
    double minimum_normalized_room{0}, mean_normalized_room{0};
    std::vector<double> position;
    TaughtMeasurementRoom room;
};

struct TaughtMeasurementSchedule {
    // valid describes input geometry, not successful identification. A valid
    // path without an eligible anchor remains executable at its normal clock.
    bool valid{false}, has_anchor{false};
    std::vector<double> demonstrated_lower_rad, demonstrated_upper_rad;
    TaughtMeasurementAnchor common_anchor;
    std::vector<TaughtMeasurementAnchor> axis_anchors;
    std::string reason;
};

namespace taught_measurement_detail {
inline bool room_request_valid(const TaughtMeasurementRoomRequest& request, std::size_t joints) {
    if(!joints || request.required_room_rad.size()!=joints ||
       (!request.observed_position_step_rad.empty() && request.observed_position_step_rad.size()!=joints) ||
       (!request.stationary_noise_rms_rad.empty() && request.stationary_noise_rms_rad.size()!=joints) ||
       (!request.requested_axes.empty() && request.requested_axes.size()!=joints)) return false;
    for(std::size_t j=0;j<joints;++j) {
        const double step=request.observed_position_step_rad.empty()?0:request.observed_position_step_rad[j];
        const double noise=request.stationary_noise_rms_rad.empty()?0:request.stationary_noise_rms_rad[j];
        if(!std::isfinite(request.required_room_rad[j]) || request.required_room_rad[j]<0 ||
           !std::isfinite(step) || step<0 || !std::isfinite(noise) || noise<0) return false;
    }
    return true;
}
inline bool requested(const TaughtMeasurementRoomRequest& request,std::size_t joint) {
    return request.requested_axes.empty() || request.requested_axes[joint];
}
inline double numerical_tolerance(double a,double b) {
    return 64*std::numeric_limits<double>::epsilon()*std::max({1.0,std::abs(a),std::abs(b)});
}
inline bool same_pose(const std::vector<double>& a,const std::vector<double>& b,
                      const TaughtMeasurementRoomRequest& request) {
    for(std::size_t j=0;j<a.size();++j) {
        const double step=request.observed_position_step_rad.empty()?0:request.observed_position_step_rad[j];
        if(std::abs(a[j]-b[j])>std::max(.5*step,numerical_tolerance(a[j],b[j]))) return false;
    }
    return true;
}
inline bool better(const TaughtMeasurementAnchor& a,const TaughtMeasurementAnchor& b) {
    if(!b.available) return a.available;
    if(a.measurable_axis_count!=b.measurable_axis_count) return a.measurable_axis_count>b.measurable_axis_count;
    if(std::abs(a.minimum_normalized_room-b.minimum_normalized_room)>1e-12)
        return a.minimum_normalized_room>b.minimum_normalized_room;
    if(std::abs(a.mean_normalized_room-b.mean_normalized_room)>1e-12)
        return a.mean_normalized_room>b.mean_normalized_room;
    return a.preferred && !b.preferred;
}
} // namespace taught_measurement_detail

// Re-evaluate the actual fresh measurement, not distance to a planned pose.
// The caller still owns fresh-feedback, Safety and stationary-response checks.
// A skipped axis requests another demonstrated location; it is not a fault.
inline TaughtMeasurementRoom evaluate_taught_measurement_room(
    const std::vector<double>& actual_fresh_position,
    const std::vector<double>& demonstrated_lower_rad,
    const std::vector<double>& demonstrated_upper_rad,
    const TaughtMeasurementRoomRequest& request) {
    TaughtMeasurementRoom out;
    const auto joints=actual_fresh_position.size();
    if(!taught_measurement_detail::room_request_valid(request,joints) ||
       demonstrated_lower_rad.size()!=joints || demonstrated_upper_rad.size()!=joints) {
        out.reason="taught_measurement_room_mapping_invalid";return out;
    }
    out.negative_room_rad.resize(joints);out.positive_room_rad.resize(joints);
    out.required_room_rad.resize(joints);out.negative_measurable.resize(joints,false);
    out.positive_measurable.resize(joints,false);out.bidirectional_measurable.resize(joints,false);
    out.axis_reasons.resize(joints);
    for(std::size_t j=0;j<joints;++j) {
        const double actual=actual_fresh_position[j],lower=demonstrated_lower_rad[j],upper=demonstrated_upper_rad[j];
        if(!std::isfinite(actual) || !std::isfinite(lower) || !std::isfinite(upper) || lower>upper) {
            out.reason="taught_measurement_room_position_invalid";return out;
        }
        const double step=request.observed_position_step_rad.empty()?0:request.observed_position_step_rad[j];
        const double noise=request.stationary_noise_rms_rad.empty()?0:request.stationary_noise_rms_rad[j];
        const double required=std::max({request.required_room_rad[j],3*step,6*noise});
        out.required_room_rad[j]=required;
        // The actual center is never clamped into the demonstrated envelope.
        // Outside feedback may continue along the path, but cannot start a
        // fixed-reference perturbation claiming that envelope as its evidence.
        const double tolerance=taught_measurement_detail::numerical_tolerance(lower,upper);
        const bool inside=actual>=lower-tolerance && actual<=upper+tolerance;
        out.negative_room_rad[j]=inside?std::max(0.0,actual-lower):0;
        out.positive_room_rad[j]=inside?std::max(0.0,upper-actual):0;
        const bool wanted=taught_measurement_detail::requested(request,j);
        out.negative_measurable[j]=wanted && inside && required>0 && out.negative_room_rad[j]+tolerance>=required;
        out.positive_measurable[j]=wanted && inside && required>0 && out.positive_room_rad[j]+tolerance>=required;
        out.bidirectional_measurable[j]=out.negative_measurable[j] && out.positive_measurable[j];
        out.any_measurable=out.any_measurable || out.bidirectional_measurable[j];
        out.axis_reasons[j]=!wanted?"axis_not_requested":!inside?"actual_center_outside_demonstrated_range_continue_path":
            required<=0?"measurement_displacement_and_resolution_unspecified":out.bidirectional_measurable[j]?
            "actual_center_has_measurable_bidirectional_room":"insufficient_bidirectional_room_continue_path_or_skip_axis";
    }
    out.valid=true;
    out.reason=out.any_measurable?"actual_feedback_has_measurable_room":"continue_demonstrated_path_no_measurable_anchor_here";
    return out;
}

inline TaughtMeasurementRoom evaluate_taught_measurement_room(
    const std::vector<double>& actual_fresh_position,const TaughtMeasurementSchedule& schedule,
    const TaughtMeasurementRoomRequest& request) {
    if(!schedule.valid) {TaughtMeasurementRoom out;out.reason="taught_measurement_schedule_invalid";return out;}
    return evaluate_taught_measurement_room(actual_fresh_position,schedule.demonstrated_lower_rad,
                                            schedule.demonstrated_upper_rad,request);
}

// Geometry selection only: checked positions and already remapped pose-target
// indices enter unchanged. No URDF margin, zero-pose assumption or hard-stop
// discovery is introduced. An interior visit repeating either endpoint is not
// an interior equilibrium merely because its array index is in the middle.
inline TaughtMeasurementSchedule plan_taught_measurement_schedule(
    const std::vector<std::vector<double>>& checked_path,const TaughtMeasurementRoomRequest& request) {
    TaughtMeasurementSchedule out;
    if(checked_path.size()<2 || !taught_measurement_detail::room_request_valid(request,checked_path.front().size())) {
        out.reason="taught_measurement_schedule_mapping_invalid";return out;
    }
    const auto joints=checked_path.front().size();
    out.demonstrated_lower_rad.assign(joints,std::numeric_limits<double>::infinity());
    out.demonstrated_upper_rad.assign(joints,-std::numeric_limits<double>::infinity());
    out.axis_anchors.resize(joints);
    for(const auto& q:checked_path) {
        if(q.size()!=joints) {out.reason="taught_measurement_schedule_mapping_invalid";return out;}
        for(std::size_t j=0;j<joints;++j) {
            if(!std::isfinite(q[j])) {out.reason="taught_measurement_schedule_position_invalid";return out;}
            out.demonstrated_lower_rad[j]=std::min(out.demonstrated_lower_rad[j],q[j]);
            out.demonstrated_upper_rad[j]=std::max(out.demonstrated_upper_rad[j],q[j]);
        }
    }
    for(const auto index:request.preferred_source_indices)if(index>=checked_path.size()) {
        out.reason="taught_measurement_preferred_index_invalid";return out;
    }
    std::vector<bool> preferred(checked_path.size(),false);
    for(const auto index:request.preferred_source_indices)preferred[index]=true;
    for(std::size_t k=1;k+1<checked_path.size();++k) {
        const auto& q=checked_path[k];
        if(taught_measurement_detail::same_pose(q,checked_path.front(),request) ||
           taught_measurement_detail::same_pose(q,checked_path.back(),request)) continue;
        TaughtMeasurementAnchor anchor;anchor.source_index=k;anchor.position=q;anchor.preferred=preferred[k];
        anchor.room=evaluate_taught_measurement_room(q,out.demonstrated_lower_rad,out.demonstrated_upper_rad,request);
        double sum=0,minimum=1;
        for(std::size_t j=0;j<joints;++j)if(anchor.room.bidirectional_measurable[j]) {
            const double span=out.demonstrated_upper_rad[j]-out.demonstrated_lower_rad[j];
            const double normalized=2*std::min(anchor.room.negative_room_rad[j],anchor.room.positive_room_rad[j])/span;
            ++anchor.measurable_axis_count;minimum=std::min(minimum,normalized);sum+=normalized;
        }
        anchor.available=anchor.measurable_axis_count>0;
        anchor.minimum_normalized_room=anchor.available?minimum:0;
        anchor.mean_normalized_room=anchor.available?sum/anchor.measurable_axis_count:0;
        for(std::size_t j=0;j<joints;++j)if(anchor.room.bidirectional_measurable[j]) {
            const auto& previous=out.axis_anchors[j];
            const double previous_room=previous.available?
                std::min(previous.room.negative_room_rad[j],previous.room.positive_room_rad[j]):-1;
            const double available=std::min(anchor.room.negative_room_rad[j],anchor.room.positive_room_rad[j]);
            if(available>previous_room+1e-12 || (std::abs(available-previous_room)<=1e-12 && anchor.preferred && !previous.preferred)) {
                out.axis_anchors[j]=anchor;
            }
        }
        if(anchor.available && taught_measurement_detail::better(anchor,out.common_anchor))out.common_anchor=anchor;
    }
    out.valid=true;out.has_anchor=out.common_anchor.available;
    out.reason=out.has_anchor?"interior_demonstrated_measurement_anchor_selected":
        "continue_demonstrated_path_no_interior_measurable_anchor";
    return out;
}

struct TaughtMeasurementTimeWarp {
    bool valid{false};
    double source_duration_s{0},anchor_source_s{0},transition_s{0},dwell_s{0};
    double braking_wall_start_s{0},dwell_wall_start_s{0},resume_wall_start_s{0};
    double normal_wall_resume_s{0},wall_duration_s{0},inserted_time_s{0};
    std::string reason;
};
struct TaughtMeasurementClock {
    bool valid{false},at_anchor{false},complete{false};
    double wall_time_s{0},source_time_s{0},rate{0},rate_acceleration_s_inv{0};
    std::string phase;
};

// Each braking/resume transition is two cubic source-time pieces. Source rate
// and acceleration join continuously at 1x / stopped / 1x. A transition uses
// transition_s/2 source seconds, hence requires only that much path on each
// side. This map adds real elapsed cost; it never changes the recording T used
// by the task budget. The caller reserves that cost before admitting a dwell.
inline TaughtMeasurementTimeWarp make_taught_measurement_time_warp(
    double source_duration_s,double anchor_source_s,double dwell_s,double transition_s=.6) {
    TaughtMeasurementTimeWarp out;
    out.source_duration_s=source_duration_s;out.anchor_source_s=anchor_source_s;
    out.dwell_s=dwell_s;out.transition_s=transition_s;
    if(!std::isfinite(source_duration_s) || source_duration_s<=0 || !std::isfinite(anchor_source_s) ||
       !std::isfinite(dwell_s) || dwell_s<0 || !std::isfinite(transition_s) || transition_s<.3 || transition_s>1) {
        out.reason="taught_measurement_time_warp_input_invalid";return out;
    }
    if(anchor_source_s<transition_s/2 || source_duration_s-anchor_source_s<transition_s/2) {
        out.reason="continue_path_insufficient_time_for_smooth_measurement_dwell";return out;
    }
    out.braking_wall_start_s=anchor_source_s-transition_s/2;
    out.dwell_wall_start_s=out.braking_wall_start_s+transition_s;
    out.resume_wall_start_s=out.dwell_wall_start_s+dwell_s;
    out.normal_wall_resume_s=out.resume_wall_start_s+transition_s;
    out.inserted_time_s=transition_s+dwell_s;
    out.wall_duration_s=source_duration_s+out.inserted_time_s;
    out.valid=true;out.reason="split_cubic_dwell_preserves_original_path_clock_elsewhere";return out;
}

inline TaughtMeasurementClock sample_taught_measurement_time_warp(
    const TaughtMeasurementTimeWarp& warp,double wall_time_s) {
    TaughtMeasurementClock out;
    if(!warp.valid || !std::isfinite(wall_time_s) || wall_time_s<0)return out;
    out.valid=true;out.wall_time_s=std::min(wall_time_s,warp.wall_duration_s);
    const double time=out.wall_time_s,T=warp.transition_s;
    if(time<warp.braking_wall_start_s) {
        out.source_time_s=time;out.rate=1;out.phase="original_path_1x";
    } else if(time<warp.dwell_wall_start_s) {
        const double u=std::clamp((time-warp.braking_wall_start_s)/T,0.0,1.0);
        if(u<.5) {
            out.source_time_s=warp.braking_wall_start_s+T*(u-2*u*u*u/3);
            out.rate=1-2*u*u;out.rate_acceleration_s_inv=-4*u/T;
        } else {
            const double v=1-u;
            out.source_time_s=warp.anchor_source_s-2*T*v*v*v/3;
            out.rate=2*v*v;out.rate_acceleration_s_inv=-4*v/T;
        }
        out.phase="braking_to_interior_measurement_anchor";
    } else if(time<warp.resume_wall_start_s) {
        out.source_time_s=warp.anchor_source_s;out.at_anchor=true;out.phase="interior_measurement_dwell";
    } else if(time<warp.normal_wall_resume_s) {
        const double u=std::clamp((time-warp.resume_wall_start_s)/T,0.0,1.0);
        if(u<.5) {
            out.source_time_s=warp.anchor_source_s+2*T*u*u*u/3;
            out.rate=2*u*u;out.rate_acceleration_s_inv=4*u/T;
        } else {
            const double v=1-u;
            out.source_time_s=warp.anchor_source_s+T*(u-.5+2*v*v*v/3);
            out.rate=1-2*v*v;out.rate_acceleration_s_inv=4*v/T;
        }
        out.phase="resuming_original_path_1x";
    } else {
        out.source_time_s=time-warp.inserted_time_s;out.rate=1;out.phase="original_path_1x";
    }
    out.source_time_s=std::clamp(out.source_time_s,0.0,warp.source_duration_s);
    out.complete=time>=warp.wall_duration_s;
    // The source path itself owns its original endpoint q/v. This is a time
    // map only, so retaining rate=1 at completion preserves that derivative.
    return out;
}

inline TaughtMeasurementClock advance_taught_measurement_time_warp(
    const TaughtMeasurementTimeWarp& warp,double previous_wall_time_s,double elapsed_step_s) {
    if(!std::isfinite(previous_wall_time_s) || previous_wall_time_s<0 ||
       !std::isfinite(elapsed_step_s) || elapsed_step_s<0)return {};
    return sample_taught_measurement_time_warp(warp,previous_wall_time_s+elapsed_step_s);
}

} // namespace serial_arm::servo_tuning
