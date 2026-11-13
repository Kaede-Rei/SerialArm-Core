#pragma once

#include "serial_arm/dynamics/gain_trajectory_tuning.hpp"

namespace serial_arm::servo_tuning {

// Zero stiffness renders damping; it does not command a return to the pose at
// which dragging began. Quiet stability and motion-release decay are distinct
// validation scopes, so an unexcited axis still has a usable quiet result.
struct DragUseRequirements {
    double stationary_window_s{.6},stationary_speed_rad_s{.003},stationary_drift_rad{.004};
    double initial_motion_speed_rad_s{.003},final_speed_rad_s{.003},maximum_decay_s{1};
    double release_window_s{1.5},maximum_speed_growth{1.4};
};
struct DragStationaryAssessment {
    bool measurement_complete{false},passed{false},limited{false};
    std::size_t fresh_samples{0};double duration_s{0},position_span_rad{0},speed_rad_s{0},raw_peak_velocity_rad_s{0};
    std::string reason{"stationary_drag_not_measured"};
    std::string validation_scope{"zero_stiffness_zero_commanded_disturbance_stability_only_release_not_measured"};
};
struct DragReleaseAssessment {
    bool measurement_complete{false},passed{false},limited{false},motion_observed{false},sustained_oscillation{false};
    std::size_t fresh_samples{0};double initial_speed_rad_s{0},peak_speed_rad_s{0},final_speed_rad_s{0},decay_s{0};
    double release_position_rad{0},final_position_rad{0},post_release_travel_rad{0},raw_peak_velocity_rad_s{0};
    std::string reason{"drag_release_not_measured"};
    std::string validation_scope{"known_motion_release_zero_stiffness_decay_not_position_return_or_contact_force"};
};

namespace drag_use_detail {
struct Window {
    std::vector<Sample> rows;bool continuous{true},known_gains{true},constant_gains{true},zero_stiffness{true},zero_input{true};
};
inline Window window(const std::vector<Sample>& rows,double begin,double end) {
    Window out;std::uint64_t sequence=0;
    for(const auto& row:rows)if(row.time_s>=begin && row.time_s<=end && row.new_measurement) {
        if(row.feedback_sequence && row.feedback_sequence==sequence)continue;
        sequence=row.feedback_sequence;
        if(!std::isfinite(row.time_s) || !std::isfinite(row.actual) || !std::isfinite(row.velocity)) {out.continuous=false;continue;}
        if(!out.rows.empty()) {
            const double dt=row.time_s-out.rows.back().time_s;if(dt<=0 || dt>.06)out.continuous=false;
            if(std::abs(row.actual_kp-out.rows.front().actual_kp)>1e-6 || std::abs(row.actual_kd-out.rows.front().actual_kd)>1e-6)out.constant_gains=false;
        }
        out.known_gains=out.known_gains && row.actual_gains_known && std::isfinite(row.actual_kp) &&
            std::isfinite(row.actual_kd) && row.actual_kd>=0;
        out.zero_stiffness=out.zero_stiffness && std::abs(row.actual_kp)<=1e-10;
        out.zero_input=out.zero_input && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm) &&
            std::abs(row.independent_disturbance_nm)<=1e-10;
        out.rows.push_back(row);
    }
    return out;
}
inline void validate(const DragUseRequirements& req) {
    for(double value:{req.stationary_window_s,req.stationary_speed_rad_s,req.stationary_drift_rad,
        req.initial_motion_speed_rad_s,req.final_speed_rad_s,req.maximum_decay_s,req.release_window_s,req.maximum_speed_growth})
        if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid drag-use requirement");
}
}

inline DragStationaryAssessment assess_drag_stationarity(const std::vector<Sample>& rows,
    const NoiseEstimate& noise,const DragUseRequirements& req={}) {
    drag_use_detail::validate(req);DragStationaryAssessment out;if(rows.empty())return out;
    const auto measured=drag_use_detail::window(rows,rows.front().time_s,rows.back().time_s);
    out.fresh_samples=measured.rows.size();if(measured.rows.empty())return out;
    out.duration_s=measured.rows.back().time_s-measured.rows.front().time_s;
    double low=measured.rows.front().actual,high=low;
    for(const auto& row:measured.rows) {
        low=std::min(low,row.actual);high=std::max(high,row.actual);out.limited=out.limited || row.limited;
        out.raw_peak_velocity_rad_s=std::max(out.raw_peak_velocity_rad_s,std::abs(row.velocity));
    }
    out.position_span_rad=high-low;out.speed_rad_s=drag_position_speed(measured.rows,noise,req.stationary_window_s);
    out.measurement_complete=measured.continuous && measured.known_gains && measured.constant_gains && measured.zero_stiffness &&
        measured.zero_input && out.fresh_samples>=30 && out.duration_s>=req.stationary_window_s-.02;
    if(!measured.zero_stiffness)out.reason="drag_requires_actual_zero_stiffness";
    else if(!measured.known_gains || !measured.constant_gains)out.reason="drag_requires_known_constant_damping";
    else if(!measured.zero_input)out.reason="drag_quiet_window_requires_known_zero_commanded_disturbance";
    else if(!out.measurement_complete)out.reason="insufficient_quiet_drag_measurement";
    else if(out.limited)out.reason="limited_drag_measurement";
    else if(out.position_span_rad>req.stationary_drift_rad || out.speed_rad_s>req.stationary_speed_rad_s)
        out.reason="zero_stiffness_bias_drift_requires_feedforward_review";
    else {out.passed=true;out.reason="zero_stiffness_quiet_stability_qualified_release_separate";}
    return out;
}

inline DragReleaseAssessment assess_drag_release(const std::vector<Sample>& rows,const NoiseEstimate& noise,
    double release_s,const DragUseRequirements& req={},bool known_motion_preparation=false) {
    drag_use_detail::validate(req);DragReleaseAssessment out;
    if(rows.empty() || !std::isfinite(release_s))return out;
    const auto released=drag_use_detail::window(rows,release_s,rows.back().time_s);
    const auto initial=drag_use_detail::window(rows,release_s-.25,release_s+.02);
    if(released.rows.empty() || initial.rows.empty())return out;
    bool known_input=false;
    for(const auto& row:rows)if(row.time_s<release_s && row.independent_disturbance_known &&
        std::isfinite(row.independent_disturbance_nm) && std::abs(row.independent_disturbance_nm)>1e-8)known_input=true;
    out.initial_speed_rad_s=drag_position_speed(initial.rows,noise,.25);
    out.release_position_rad=released.rows.front().actual;out.final_position_rad=released.rows.back().actual;
    out.fresh_samples=released.rows.size();std::vector<Sample> history;
    for(const auto& row:initial.rows)if(row.time_s<release_s)history.push_back(row);
    const double floor=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    for(const auto& row:released.rows) {
        history.push_back(row);const double speed=drag_position_speed(history,noise,.25);
        out.peak_speed_rad_s=std::max(out.peak_speed_rad_s,speed);
        out.raw_peak_velocity_rad_s=std::max(out.raw_peak_velocity_rad_s,std::abs(row.velocity));
        out.post_release_travel_rad=std::max(out.post_release_travel_rad,std::abs(row.actual-out.release_position_rad));
        out.limited=out.limited || row.limited;if(speed>req.final_speed_rad_s)out.decay_s=row.time_s-release_s;
    }
    out.final_speed_rad_s=drag_position_speed(released.rows,noise,req.stationary_window_s);
    out.sustained_oscillation=tail_position_reversals(released.rows,release_s,floor)>=4 && out.post_release_travel_rad>floor;
    out.motion_observed=out.initial_speed_rad_s>=req.initial_motion_speed_rad_s && out.post_release_travel_rad>floor;
    out.measurement_complete=released.continuous && released.known_gains && released.constant_gains && released.zero_stiffness &&
        released.zero_input && (known_input || known_motion_preparation) && out.fresh_samples>=30 &&
        released.rows.back().time_s-release_s>=req.release_window_s-.02 && out.motion_observed;
    if(!released.zero_stiffness)out.reason="drag_release_requires_actual_zero_stiffness";
    else if(!released.zero_input)out.reason="drag_release_input_not_removed_or_unknown";
    else if(!known_input && !known_motion_preparation)out.reason="drag_release_preparation_unmeasured";
    else if(!out.motion_observed)out.reason="drag_release_motion_unresolved_quiet_result_remains_available";
    else if(!out.measurement_complete)out.reason="insufficient_constant_damping_release_measurement";
    else if(out.limited)out.reason="limited_drag_release";
    else if(out.sustained_oscillation)out.reason="drag_release_repeated_position_oscillation";
    else if(out.final_speed_rad_s>req.final_speed_rad_s)out.reason="drag_release_still_moving";
    else if(out.decay_s>req.maximum_decay_s)out.reason="drag_release_decay_too_slow";
    else if(out.peak_speed_rad_s>std::max(.025,req.maximum_speed_growth*out.initial_speed_rad_s))out.reason="drag_release_speed_increased";
    else {out.passed=true;out.reason="zero_stiffness_release_decay_qualified_no_position_return_required";}
    return out;
}

inline TrajectoryCandidateReport drag_damping_candidate_report(const Gains& current,const std::vector<double>& kd_max,
    const std::vector<DragStationaryAssessment>& stationary,const std::vector<DragReleaseAssessment>& release,
    const DragUseRequirements& req={}) {
    drag_use_detail::validate(req);const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || kd_max.size()!=n || stationary.size()!=n || release.size()!=n)
        throw std::invalid_argument("drag damping candidate mapping mismatch");
    TrajectoryCandidateReport out;out.gains=current;out.reasons.resize(n);
    for(std::size_t i=0;i<n;++i) {
        if(current.kp[i]!=0 || !std::isfinite(current.kd[i]) || current.kd[i]<0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 || current.kd[i]>kd_max[i])
            throw std::invalid_argument("drag candidate requires zero stiffness and valid damping capability");
        const auto& quiet=stationary[i];const auto& motion=release[i];auto& reason=out.reasons[i];
        if(motion.measurement_complete && !motion.limited && motion.sustained_oscillation) {
            out.gains.kd[i]=.9*current.kd[i];reason="measured_zero_stiffness_release_ringing_small_damping_reduction_comparison";
        } else if(quiet.measurement_complete && !quiet.passed)reason="zero_stiffness_drift_needs_feedforward_or_load_review_not_unbounded_damping";
        else if(motion.measurement_complete && !motion.passed && !motion.limited) {
            out.gains.kd[i]=std::min(kd_max[i],1.08*current.kd[i]);reason="measured_release_decay_small_damping_only_comparison";
        } else if(quiet.passed && !motion.measurement_complete)reason="quiet_drag_goal_met_release_scope_unmeasured";
        else reason=quiet.passed && motion.passed?"quiet_drag_and_release_goals_met":"drag_measurement_incomplete";
        out.changed=out.changed || out.gains.kd[i]!=current.kd[i];
    }
    return out;
}

// Compare complete measured groups under the same preparation. Lack of motion
// in a later trial cannot erase an already measured release scope and turn a
// failed release into a newly "qualified" quiet-only controller.
inline bool drag_use_preferred(const std::vector<DragStationaryAssessment>& newquiet,
    const std::vector<DragReleaseAssessment>& newrelease,const std::vector<DragStationaryAssessment>& oldquiet,
    const std::vector<DragReleaseAssessment>& oldrelease) {
    const auto n=oldquiet.size();
    if(!n || newquiet.size()!=n || newrelease.size()!=n || oldrelease.size()!=n)return false;
    bool scope_improved=false;double old_score=0,new_score=0;
    for(std::size_t i=0;i<n;++i) {
        const auto& q=newquiet[i];const auto& previous=oldquiet[i];
        const auto& release=newrelease[i];const auto& former=oldrelease[i];
        if(!q.measurement_complete || q.limited || (previous.passed && !q.passed))return false;
        if(former.measurement_complete && (!release.measurement_complete || release.limited || (former.passed && !release.passed)))return false;
        if(previous.measurement_complete) {
            if(q.position_span_rad>previous.position_span_rad+std::max(.0002,.1*previous.position_span_rad) ||
               q.speed_rad_s>previous.speed_rad_s+std::max(.0005,.1*previous.speed_rad_s))return false;
            old_score+=previous.position_span_rad/.004+previous.speed_rad_s/.003;
            new_score+=q.position_span_rad/.004+q.speed_rad_s/.003;
        }
        if(former.measurement_complete) {
            if(std::abs(release.initial_speed_rad_s-former.initial_speed_rad_s)>std::max(.005,.25*former.initial_speed_rad_s))return false;
            if(release.final_speed_rad_s>former.final_speed_rad_s+std::max(.0005,.1*former.final_speed_rad_s) ||
               release.decay_s>former.decay_s+.1 ||
               release.peak_speed_rad_s>former.peak_speed_rad_s+std::max(.001,.1*former.peak_speed_rad_s))return false;
            old_score+=former.final_speed_rad_s/.003+former.decay_s;
            new_score+=release.final_speed_rad_s/.003+release.decay_s;
        }
        scope_improved=scope_improved || (!previous.measurement_complete && q.measurement_complete) ||
            (!previous.passed && q.passed) || (!former.passed && release.passed);
    }
    return scope_improved || new_score+1e-6<old_score;
}

} // namespace serial_arm::servo_tuning
