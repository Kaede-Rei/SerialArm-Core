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
    bool input_measurement_complete{false},preparation_motion_observed{false},release_motion_observed{false};
    bool fast_stop_observed{false},decay_time_is_upper_bound{false};
    double input_peak_nm{0},decay_time_upper_bound_s{std::numeric_limits<double>::quiet_NaN()};
    std::size_t preparation_samples{0};
    bool feedback_clock_aligned{false},release_timing_known{false},speed_comparison_valid{false};
    bool pulse_response_motion_observed{false},demonstrated_motion_preparation{false};
    double command_release_s{std::numeric_limits<double>::quiet_NaN()},release_speed_window_s{0};
    double preparation_motion_span_rad{0};
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
// Commands retain their write/sample clocks. Only the local position-analysis
// copy uses the encoder's real clock; a host row containing old feedback must
// never move that feedback across an unload boundary.
inline Window feedback_rows(const std::vector<Sample>& rows) {
    Window out;std::uint64_t sequence=0;
    for(const auto& original:rows)if(original.new_measurement) {
        if(original.feedback_sequence && original.feedback_sequence==sequence)continue;
        sequence=original.feedback_sequence;
        if(!std::isfinite(original.command_time_s) || !std::isfinite(original.feedback_time_s) ||
           original.feedback_time_s>original.command_time_s+.005 ||
           original.command_time_s-original.feedback_time_s>.06 || !std::isfinite(original.actual) ||
           !std::isfinite(original.velocity)) {out.continuous=false;continue;}
        auto row=original;row.time_s=row.feedback_time_s;
        if(!out.rows.empty()) {
            const double dt=row.time_s-out.rows.back().time_s;
            if(dt<=0 || dt>.06)out.continuous=false;
            if(std::abs(row.actual_kp-out.rows.front().actual_kp)>1e-6 ||
               std::abs(row.actual_kd-out.rows.front().actual_kd)>1e-6)out.constant_gains=false;
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
inline double position_span(const std::vector<Sample>& rows) {
    if(rows.empty())return 0;
    double low=rows.front().actual,high=low;
    for(const auto& row:rows){low=std::min(low,row.actual);high=std::max(high,row.actual);}
    return high-low;
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
    drag_use_detail::validate(req);DragReleaseAssessment out;out.demonstrated_motion_preparation=known_motion_preparation;
    if(rows.empty() || !std::isfinite(release_s))return out;
    const auto feedback=drag_use_detail::feedback_rows(rows);
    out.feedback_clock_aligned=feedback.continuous && feedback.rows.size()>=3;
    bool known_input=false,preparation_known=true,reference_velocity_zero=true,zero_after_release=true,command_gains_known=true;
    double last_powered=-std::numeric_limits<double>::infinity(),first_powered=std::numeric_limits<double>::infinity();
    // Input evidence is checked on command writes, including repeats of a
    // feedback frame. Repeated feedback never counts as another encoder sample.
    double previous_command=-std::numeric_limits<double>::infinity();
    for(const auto& row:rows) {
        const bool known=row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm);
        preparation_known=preparation_known && known && std::isfinite(row.command_time_s) &&
            row.command_time_s>=previous_command;
        if(std::isfinite(row.command_time_s))previous_command=row.command_time_s;
        if(row.independent_input_planned_known)preparation_known=preparation_known &&
            std::isfinite(row.planned_independent_disturbance_nm) &&
            std::abs(row.independent_disturbance_nm-row.planned_independent_disturbance_nm)<=1e-8;
        // A demonstrated approach is a known preparation, not part of the
        // free P=0 experiment. Its actual PD/reference may change during the
        // existing handover; every command remains observable. The released
        // experiment below independently requires P=0, fixed D and zero drive.
        if(!known_motion_preparation) {
            reference_velocity_zero=reference_velocity_zero && std::isfinite(row.reference_velocity) &&
                std::abs(row.reference_velocity)<=1e-8;
            command_gains_known=command_gains_known && row.actual_gains_known && std::isfinite(row.actual_kp) &&
                std::abs(row.actual_kp)<=1e-10 && std::isfinite(row.actual_kd) && row.actual_kd>=0 &&
                std::abs(row.actual_kd-rows.front().actual_kd)<=1e-6;
        } else command_gains_known=command_gains_known && row.actual_gains_known &&
            std::isfinite(row.actual_kp) && row.actual_kp>=0 && std::isfinite(row.actual_kd) && row.actual_kd>=0 &&
            std::isfinite(row.reference) && std::isfinite(row.reference_velocity);
        out.limited=out.limited || row.limited;
        if(known && std::abs(row.independent_disturbance_nm)>1e-8) {
            out.input_peak_nm=std::max(out.input_peak_nm,std::abs(row.independent_disturbance_nm));
            known_input=true;first_powered=std::min(first_powered,row.command_time_s);
            last_powered=std::max(last_powered,row.command_time_s);
        }
    }
    // A nonzero residual after the caller's observed unload is not another
    // preparation pulse, and must not silently shift the declared experiment.
    for(const auto& row:rows)if(row.time_s>=release_s &&
        (!row.independent_disturbance_known || !std::isfinite(row.independent_disturbance_nm) ||
         std::abs(row.independent_disturbance_nm)>1e-8))zero_after_release=false;
    for(const auto& row:rows)if(row.independent_disturbance_known &&
        std::isfinite(row.command_time_s) && std::abs(row.independent_disturbance_nm)<=1e-8 &&
        (known_motion_preparation?
            (row.time_s>=release_s && row.actual_gains_known && std::isfinite(row.actual_kp) &&
             std::abs(row.actual_kp)<=1e-10 && std::isfinite(row.reference_velocity) && std::abs(row.reference_velocity)<=1e-8):
            (known_input?row.command_time_s>last_powered:row.time_s>=release_s))) {
        out.command_release_s=row.command_time_s;break;
    }
    out.release_timing_known=preparation_known && zero_after_release &&
        std::isfinite(out.command_release_s) && std::abs(out.command_release_s-release_s)<=.06;
    if(!out.feedback_clock_aligned || !out.release_timing_known) {
        out.reason="drag_release_actual_input_or_timing_unknown";return out;
    }
    const double unload=out.command_release_s;
    bool released_zero_stiffness=true,released_constant_damping=true;double released_damping=0;bool release_gain_seen=false;
    for(const auto& row:rows)if(row.command_time_s>=unload) {
        released_zero_stiffness=released_zero_stiffness && row.actual_gains_known &&
            std::isfinite(row.actual_kp) && std::abs(row.actual_kp)<=1e-10;
        const bool known=row.actual_gains_known && std::isfinite(row.actual_kd) && row.actual_kd>=0;
        if(!release_gain_seen && known){released_damping=row.actual_kd;release_gain_seen=true;}
        released_constant_damping=released_constant_damping && known && std::abs(row.actual_kd-released_damping)<=1e-6;
        reference_velocity_zero=reference_velocity_zero && std::isfinite(row.reference_velocity) &&
            std::abs(row.reference_velocity)<=1e-8;
    }
    std::vector<double> intervals;
    for(std::size_t k=1;k<feedback.rows.size();++k)intervals.push_back(feedback.rows[k].time_s-feedback.rows[k-1].time_s);
    std::sort(intervals.begin(),intervals.end());const double dt=intervals[intervals.size()/2];
    // A short pulse needs a short, resolution-aware speed window; the same
    // length is used on both sides. A long quiet baseline is never compared
    // against a peak window containing the actual powered movement.
    const double powered_s=known_input && !known_motion_preparation?unload-first_powered:.25;
    out.release_speed_window_s=std::clamp(std::max(10*dt,powered_s),.05,.25);
    const double w=out.release_speed_window_s;
    const auto initial=drag_use_detail::window(feedback.rows,unload-w,std::nextafter(unload,-std::numeric_limits<double>::infinity()));
    const auto prepared=drag_use_detail::window(feedback.rows,feedback.rows.front().time_s,
        std::nextafter(unload,-std::numeric_limits<double>::infinity()));
    const auto released=drag_use_detail::window(feedback.rows,unload,feedback.rows.back().time_s);
    if(released.rows.empty() || initial.rows.empty())return out;
    const double floor=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    out.preparation_samples=prepared.rows.size();out.fresh_samples=released.rows.size();
    out.preparation_motion_span_rad=drag_use_detail::position_span(initial.rows);
    out.initial_speed_rad_s=drag_position_speed(initial.rows,noise,w);
    out.preparation_motion_observed=out.preparation_motion_span_rad>floor &&
        out.initial_speed_rad_s>=req.initial_motion_speed_rad_s;
    out.speed_comparison_valid=initial.rows.size()>=3 &&
        initial.rows.back().time_s-initial.rows.front().time_s>=w-3*dt && out.preparation_motion_observed;
    out.release_position_rad=released.rows.front().actual;out.final_position_rad=released.rows.back().actual;
    // Keep movement straddling a short pulse/unload boundary as pulse-response
    // evidence. Its unresolved pre-unload speed cannot set a damping direction.
    const auto pulse_response=drag_use_detail::window(feedback.rows,
        known_input && !known_motion_preparation?first_powered-dt:unload-w,unload+w);
    out.pulse_response_motion_observed=drag_use_detail::position_span(pulse_response.rows)>floor;
    double first_quiet=std::numeric_limits<double>::quiet_NaN();std::vector<Sample> history;
    for(const auto& row:released.rows) {
        history.push_back(row);
        // Require a full post-unload speed window, with no powered samples.
        if(row.time_s-unload>=w) {
            const double speed=drag_position_speed(history,noise,w);
            out.peak_speed_rad_s=std::max(out.peak_speed_rad_s,speed);
            if(speed>req.final_speed_rad_s)out.decay_s=row.time_s-unload;
        }
        out.raw_peak_velocity_rad_s=std::max(out.raw_peak_velocity_rad_s,std::abs(row.velocity));
        out.post_release_travel_rad=std::max(out.post_release_travel_rad,std::abs(row.actual-out.release_position_rad));
        if(!std::isfinite(first_quiet) && row.time_s-unload>=req.stationary_window_s &&
           drag_position_speed(history,noise,req.stationary_window_s)<=req.final_speed_rad_s)
            first_quiet=row.time_s-unload;
    }
    out.final_speed_rad_s=drag_position_speed(released.rows,noise,req.stationary_window_s);
    out.sustained_oscillation=tail_position_reversals(released.rows,unload+w,floor)>=4 && out.post_release_travel_rad>floor;
    out.release_motion_observed=out.post_release_travel_rad>floor;
    const bool experiment_gains=known_motion_preparation?
        (released_zero_stiffness && released_constant_damping && release_gain_seen):
        (feedback.known_gains && feedback.constant_gains && feedback.zero_stiffness);
    out.input_measurement_complete=reference_velocity_zero && command_gains_known && experiment_gains && prepared.rows.size()>=3 &&
        (known_input || known_motion_preparation) && out.fresh_samples>=30 &&
        released.rows.back().time_s-unload>=req.release_window_s-.02 && !out.limited;
    const bool prepared_motion=out.preparation_motion_observed ||
        (!known_motion_preparation && out.pulse_response_motion_observed);
    out.fast_stop_observed=out.input_measurement_complete &&
        prepared_motion &&
        std::isfinite(first_quiet) && !out.sustained_oscillation &&
        out.final_speed_rad_s<=req.final_speed_rad_s && !out.release_motion_observed;
    out.motion_observed=prepared_motion &&
        (out.release_motion_observed || out.fast_stop_observed);
    if(out.fast_stop_observed) {
        out.decay_time_is_upper_bound=true;out.decay_time_upper_bound_s=first_quiet;out.decay_s=first_quiet;
        out.validation_scope="known_commanded_pulse_position_response_fast_stop_upper_bound_not_actuator_torque_removal_time";
    } else out.validation_scope="command_write_referenced_zero_stiffness_position_decay_not_measured_actuator_torque_removal";
    out.measurement_complete=out.input_measurement_complete && out.motion_observed;
    if(known_motion_preparation)out.validation_scope=out.fast_stop_observed?
        "demonstrated_motion_then_measured_zero_stiffness_fast_stop_upper_bound":
        "demonstrated_motion_then_measured_zero_stiffness_free_position_decay";
    if(!released_zero_stiffness || (!known_motion_preparation && !feedback.zero_stiffness))out.reason="drag_release_requires_actual_zero_stiffness";
    else if(!released_constant_damping)out.reason="drag_release_requires_known_constant_damping";
    else if(!zero_after_release)out.reason="drag_release_input_not_removed_or_unknown";
    else if(!known_input && !known_motion_preparation)out.reason="drag_release_preparation_unmeasured";
    else if(!reference_velocity_zero)out.reason="drag_release_reference_velocity_not_zero";
    else if(!out.motion_observed)out.reason="drag_release_motion_unresolved_quiet_result_remains_available";
    else if(!out.measurement_complete)out.reason="insufficient_constant_damping_release_measurement";
    else if(out.sustained_oscillation)out.reason="drag_release_repeated_position_oscillation";
    else if(out.final_speed_rad_s>req.final_speed_rad_s)out.reason="drag_release_still_moving";
    else if(out.decay_s>req.maximum_decay_s)out.reason="drag_release_decay_too_slow";
    else if(!out.speed_comparison_valid && !out.fast_stop_observed)out.reason="drag_release_motion_measured_initial_speed_unresolved";
    else if(out.speed_comparison_valid &&
        out.peak_speed_rad_s>std::max(.025,req.maximum_speed_growth*out.initial_speed_rad_s))
        out.reason="drag_release_speed_increased_input_response_timing_review";
    else {out.passed=true;out.reason=out.fast_stop_observed?"drag_release_fast_stop_upper_bound_recorded":
        "zero_stiffness_release_decay_qualified_no_position_return_required";}
    return out;
}

struct DragInputRetry {
    bool retry{false};double next_input_nm{0};std::string reason{"drag_input_retry_not_requested"};
};
// The ceiling may be the minimum sampled-waveform ceiling. The returned force
// is a planning request: regenerate the pulse timing and its impulse/effort
// bounds before executing it, and reserve the complete observation window.
inline DragInputRetry next_drag_release_input(const DragReleaseAssessment& measured,
    double current_input_nm,double input_ceiling_nm,double torque_quantum_nm) {
    DragInputRetry out;out.next_input_nm=current_input_nm;
    if(!measured.input_measurement_complete || measured.limited || measured.sustained_oscillation) {
        out.reason="drag_input_retry_requires_complete_known_unlimited_input";return out;
    }
    if(measured.preparation_motion_observed || measured.motion_observed || measured.pulse_response_motion_observed) {
        out.reason="drag_preparation_motion_resolved_no_input_increase";return out;
    }
    if(measured.release_motion_observed) {
        out.reason="drag_released_motion_without_resolved_preparation_no_input_increase";return out;
    }
    for(double x:{current_input_nm,input_ceiling_nm,measured.input_peak_nm})if(!std::isfinite(x) || x<=0) {
        out.reason="drag_input_retry_requires_measured_positive_input";return out;
    }
    if(!std::isfinite(torque_quantum_nm) || torque_quantum_nm<0) {
        out.reason="drag_input_retry_requires_valid_torque_quantum";return out;
    }
    const double current=std::max(current_input_nm,measured.input_peak_nm);
    double next=std::max(2*current,current+torque_quantum_nm);
    if(torque_quantum_nm>0)next=std::ceil(next/torque_quantum_nm-1e-12)*torque_quantum_nm;
    next=std::min(next,input_ceiling_nm);
    if(torque_quantum_nm>0)next=std::floor(next/torque_quantum_nm+1e-12)*torque_quantum_nm;
    if(next<=current+1e-12){out.reason="drag_input_current_domain_ceiling_no_resolved_motion";return out;}
    out.retry=true;out.next_input_nm=next;out.reason="drag_known_unresolved_input_bounded_increase";return out;
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
        if(motion.measurement_complete && motion.release_timing_known && !motion.limited && motion.sustained_oscillation) {
            out.gains.kd[i]=.9*current.kd[i];reason="measured_zero_stiffness_release_ringing_small_damping_reduction_comparison";
        } else if(quiet.measurement_complete && !quiet.passed)reason="zero_stiffness_drift_needs_feedforward_or_load_review_not_unbounded_damping";
        else if(motion.measurement_complete && motion.release_timing_known && motion.speed_comparison_valid &&
            !motion.passed && !motion.limited && (motion.reason=="drag_release_still_moving" ||
            motion.reason=="drag_release_decay_too_slow")) {
            out.gains.kd[i]=std::min(kd_max[i],1.08*current.kd[i]);reason="measured_release_decay_small_damping_only_comparison";
        } else if(motion.measurement_complete && !motion.passed)
            reason="drag_release_input_timing_or_speed_comparison_requires_review_no_damping_direction";
        else if(quiet.passed && !motion.measurement_complete)reason="quiet_drag_goal_met_release_scope_unmeasured";
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
