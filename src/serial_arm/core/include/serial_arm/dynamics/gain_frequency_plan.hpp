#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {

// Experiment design constants, not mechanical facts or measured bandwidth.
// Match the existing estimator's two .7 s smooth edges, >=150 fresh samples
// and >=60 samples per half, with small sample-count headroom.
struct MitFrequencyPlanPolicy {
    static constexpr std::array<double,3> tone_ratios{.5,1,2};
    static constexpr unsigned controller_experiments=4;
    static constexpr double low_frequency_cycles=3;
    static constexpr double excluded_edge_time_s=1.4;
    static constexpr double maximum_cycles_per_feedback_sample=.1;
    static constexpr double maximum_feedback_age_cycles=.125; // 45 degrees
    // Existing MIT estimator rejects >60 ms raw feedback ages or fresh gaps.
    static constexpr double maximum_feedback_pipeline_s=.06;
    static constexpr unsigned minimum_selected_samples=160;
    static constexpr unsigned minimum_samples_per_half=64;
};

struct MitFrequencyTorquePolicy {
    // Match probe()'s existing zero-mean three-tone instrument. The design
    // target is twice the estimator's encoder/static-noise position floor.
    static constexpr std::array<double,3> tone_weights{.4,.35,.25};
    static constexpr double position_signal_margin=2;
    static constexpr double known_torque_code_margin=2;
    static constexpr double demonstrated_room_fraction=.5;
};

struct MitFrequencyTorquePlanRequest {
    double inertia_kg_m2{0};
    // Actual protocol values for the complete four-controller axis group.
    std::array<double,4> actual_kp{},actual_kd{};
    std::array<double,3> frequencies_hz{};
    double encoder_step_rad{0},noise_rms_rad{0};
    double negative_room_rad{0},positive_room_rad{0};
    // Existing execution allowance, not a physical motor-capability estimate.
    double effort_allowance_nm{0};
    // Optional known command quantum after joint mapping; zero means unknown.
    double torque_quantum_nm{0};
};

struct MitFrequencyTorquePlan {
    // eligible means a model-predicted input exists for this exact measured
    // room/4-controller/3-tone plan, not measured SNR or hardware stability.
    bool valid{false},eligible{false};
    double initial_amplitude_nm{0},maximum_amplitude_nm{0},minimum_required_amplitude_nm{0};
    double position_noise_floor_rad{0},position_target_rad{0},maximum_predicted_excursion_rad{0};
    double predicted_min_tone_position_amplitude_rad{0},minimum_resolved_input_amplitude_nm{0};
    double predicted_min_tone_at_max_rad{0};
    std::array<std::array<double,3>,4> predicted_tone_position_per_nm{};
    std::array<std::array<double,3>,4> predicted_tone_position_amplitude_rad{};
    std::array<double,4> predicted_peak_excursion_rad{};
    std::string source{"urdf_local_rigid_body_current_PD_AC_input_design_not_measured_actuator_response"};
    std::string reason;
    std::vector<std::string> missing_evidence;
};

// H_k(jw)=1/(P_k-J*w^2+j*D_k*w) is an input-amplitude prior for a fixed
// reference and independently commanded torque, not the position-reference
// transfer function (whose MIT reference-velocity numerator is different).
// A common amplitude must resolve every tone under every actual controller,
// while sum_j |H_k(jw_j)|*weight_j*amplitude fits reserved demonstrated room.
// This finite-frequency AC design does not use a static P*room torque ceiling.
// Unknown inner filtering, coupling, friction and transients are not certified
// by the rigid-body prediction; the caller retains actual-domain/Safety guards.
inline MitFrequencyTorquePlan plan_mit_frequency_torque_experiment(const MitFrequencyTorquePlanRequest& request) {
    using Policy=MitFrequencyTorquePolicy;
    MitFrequencyTorquePlan out;
    const auto missing=[&](const char* reason,const char* evidence) {
        out.reason=reason;out.missing_evidence.push_back(evidence);return out;
    };
    for(double value:{request.inertia_kg_m2,request.negative_room_rad,request.positive_room_rad,request.effort_allowance_nm})
        if(!std::isfinite(value) || value<=0)
            return missing("mit_frequency_torque_invalid_model_room_or_effort","positive_URDF_local_inertia_actual_two_sided_demonstrated_room_and_existing_effort_allowance");
    for(double value:{request.encoder_step_rad,request.noise_rms_rad,request.torque_quantum_nm})
        if(!std::isfinite(value) || value<0)
            return missing("mit_frequency_torque_invalid_resolution_or_noise","finite_nonnegative_measured_encoder_step_static_noise_and_known_torque_quantum");
    for(std::size_t j=0;j<3;++j)
        if(!std::isfinite(request.frequencies_hz[j]) || request.frequencies_hz[j]<=0 ||
           (j && request.frequencies_hz[j]<=request.frequencies_hz[j-1]))
            return missing("mit_frequency_torque_invalid_frequency_plan","three_ordered_resolvable_positive_input_frequencies");
    for(std::size_t k=0;k<4;++k)
        if(!std::isfinite(request.actual_kp[k]) || request.actual_kp[k]<=0 ||
           !std::isfinite(request.actual_kd[k]) || request.actual_kd[k]<0)
            return missing("mit_frequency_torque_invalid_actual_controller_group","complete_four_actual_positive_P_nonnegative_D_controllers");
    out.position_noise_floor_rad=std::max({1e-7,2*request.encoder_step_rad,3*request.noise_rms_rad});
    out.position_target_rad=Policy::position_signal_margin*out.position_noise_floor_rad;
    out.maximum_predicted_excursion_rad=Policy::demonstrated_room_fraction*std::min(request.negative_room_rad,request.positive_room_rad);
    std::array<std::array<double,3>,4> response_per_nm{};
    double smallest_tone_response=std::numeric_limits<double>::infinity(),largest_sum_response=0;
    for(std::size_t k=0;k<4;++k) {
        double sum=0;
        for(std::size_t j=0;j<3;++j) {
            const double w=2*3.14159265358979323846*request.frequencies_hz[j];
            const double denominator=std::hypot(request.actual_kp[k]-request.inertia_kg_m2*w*w,request.actual_kd[k]*w);
            const double response=Policy::tone_weights[j]/denominator;
            if(!std::isfinite(response) || response<=0)
                return missing("mit_frequency_torque_unresolvable_rigid_body_response","finite_nonresonant_local_rigid_body_frequency_response_prior");
            response_per_nm[k][j]=response;out.predicted_tone_position_per_nm[k][j]=response;sum+=response;
            smallest_tone_response=std::min(smallest_tone_response,response);
        }
        largest_sum_response=std::max(largest_sum_response,sum);
    }
    out.minimum_required_amplitude_nm=out.position_target_rad/smallest_tone_response;
    if(request.torque_quantum_nm>0) {
        for(double weight:Policy::tone_weights)
            out.minimum_resolved_input_amplitude_nm=std::max(out.minimum_resolved_input_amplitude_nm,
                Policy::known_torque_code_margin*request.torque_quantum_nm/weight);
        out.minimum_required_amplitude_nm=std::max(out.minimum_required_amplitude_nm,out.minimum_resolved_input_amplitude_nm);
    } else out.missing_evidence.push_back("independent_torque_command_quantum_unknown_actual_input_resolution_requires_measurement");
    out.maximum_amplitude_nm=std::min(request.effort_allowance_nm,out.maximum_predicted_excursion_rad/largest_sum_response);
    if(!std::isfinite(out.minimum_required_amplitude_nm) || !std::isfinite(out.maximum_amplitude_nm) ||
       !std::isfinite(out.position_target_rad) || !std::isfinite(out.maximum_predicted_excursion_rad) || out.maximum_amplitude_nm<=0)
        return missing("mit_frequency_torque_numerically_unresolvable","finite_input_signal_and_complete_predicted_displacement_envelope");
    out.valid=true;
    out.predicted_min_tone_at_max_rad=out.maximum_amplitude_nm*smallest_tone_response;
    if(out.minimum_required_amplitude_nm>out.maximum_amplitude_nm) {
        if(out.minimum_required_amplitude_nm>request.effort_allowance_nm)
            out.missing_evidence.push_back("required_resolved_tone_input_exceeds_existing_effort_allowance");
        if(out.minimum_required_amplitude_nm*largest_sum_response>out.maximum_predicted_excursion_rad)
            out.missing_evidence.push_back("required_resolved_tone_input_exceeds_reserved_actual_demonstrated_room");
        return missing("mit_frequency_torque_signal_and_existing_bounds_incompatible",
            "lower_or_different_measurable_frequency_band_or_more_actual_demonstrated_room_within_existing_budget_and_Safety");
    }
    out.initial_amplitude_nm=out.minimum_required_amplitude_nm;
    out.predicted_min_tone_position_amplitude_rad=std::numeric_limits<double>::infinity();
    for(std::size_t k=0;k<4;++k)for(std::size_t j=0;j<3;++j) {
        const double predicted=out.initial_amplitude_nm*response_per_nm[k][j];
        out.predicted_tone_position_amplitude_rad[k][j]=predicted;
        out.predicted_peak_excursion_rad[k]+=predicted;
        out.predicted_min_tone_position_amplitude_rad=std::min(out.predicted_min_tone_position_amplitude_rad,predicted);
    }
    out.eligible=true;out.reason="mit_frequency_torque_common_input_resolves_four_controller_rigid_body_prior_within_demonstrated_room";
    return out;
}

struct MitFrequencyPlanRequest {
    double inertia_kg_m2{0},actual_kp{0},actual_kd{0};
    // Observed fresh-feedback cadence/age, not configured control frequency or
    // an assumption about the actuator's unreported embedded velocity filter.
    double feedback_period_s{0},feedback_age_s{0};
    // Already excludes the caller's required validation and exit reserve.
    // Per-controller overhead includes baseline, gain transition and tail;
    // probe_duration_s below already includes both smooth input edges.
    double measurement_budget_s{0},per_controller_overhead_s{0};
    // Pure numerical co-design candidate; zero selects the rigid-body prior.
    // It changes which band is measured, never the prior or original budget.
    double preferred_center_hz{0};
};

struct MitFrequencyPlan {
    // valid: inputs and rigid-body prior can be evaluated. eligible: the full
    // four-controller measurement fits actual feedback and remaining budget.
    // Neither means identification succeeded or that a controller is stable.
    bool valid{false},eligible{false},prior_crossover_covered{false};
    bool feedback_band_limited{false},budget_band_limited{false};
    double rigid_body_crossover_hz{0},selected_center_hz{0};
    std::array<double,3> frequencies_hz{};
    double probe_duration_s{0},total_cost_s{0};
    double observable_low_hz{0},observable_high_hz{0};
    std::string source{"urdf_local_inertia_and_current_actual_PD_rigid_body_probe_prior_not_measured_bandwidth"};
    std::string reason;
    std::vector<std::string> missing_evidence;
};

// |(P+D*jw)/(J*(jw)^2)|=1 gives
// w^2 = ((D/J)^2 + hypot((D/J)^2, 2P/J))/2.
// This predicts where to measure a rigid-body loop; measured internal filters,
// delay, friction, coupling and flexible modes still determine the real FRF.
// No old configuration gains, joint identity, final error or settling target
// enters. actual_PD is the current controller group established for this run.
inline MitFrequencyPlan plan_mit_frequency_experiment(const MitFrequencyPlanRequest& request) {
    using Policy=MitFrequencyPlanPolicy;
    MitFrequencyPlan out;
    const auto missing=[&](const char* reason,const char* evidence) {
        out.reason=reason;out.missing_evidence.push_back(evidence);return out;
    };
    for(double value:{request.inertia_kg_m2,request.actual_kp,request.feedback_period_s})
        if(!std::isfinite(value) || value<=0)
            return missing("mit_frequency_plan_invalid_model_or_actual_controller","positive_local_URDF_inertia_current_actual_P_and_measured_feedback_period");
    for(double value:{request.actual_kd,request.feedback_age_s,request.measurement_budget_s,request.per_controller_overhead_s,request.preferred_center_hz})
        if(!std::isfinite(value) || value<0)
            return missing("mit_frequency_plan_invalid_observation_or_budget","finite_nonnegative_current_D_feedback_age_and_existing_budget_costs");
    const double d=request.actual_kd/request.inertia_kg_m2,p=request.actual_kp/request.inertia_kg_m2;
    const double d2=d*d,w2=.5*d2+.5*std::hypot(d2,2*p);
    out.rigid_body_crossover_hz=std::sqrt(w2)/(2*3.14159265358979323846);
    if(!std::isfinite(out.rigid_body_crossover_hz) || out.rigid_body_crossover_hz<=0)
        return missing("mit_frequency_plan_rigid_body_prior_unresolvable","finite_current_controller_rigid_body_probe_prior");
    out.valid=true;
    const double dt=request.feedback_period_s,age=request.feedback_age_s;
    if(dt>Policy::maximum_feedback_pipeline_s || age>Policy::maximum_feedback_pipeline_s)
        return missing("mit_frequency_plan_feedback_pipeline_not_observable","measured_fresh_feedback_period_and_age_inside_existing_estimator_clock_domain");
    out.observable_high_hz=Policy::maximum_cycles_per_feedback_sample/dt;
    if(age>0)out.observable_high_hz=std::min(out.observable_high_hz,Policy::maximum_feedback_age_cycles/age);
    const double per_trial=request.measurement_budget_s/Policy::controller_experiments-request.per_controller_overhead_s;
    // Do not round a window above the original wall-clock allowance.
    const double maximum_probe=std::floor(per_trial/dt)*dt;
    const double observable_time=maximum_probe-Policy::excluded_edge_time_s-age;
    const double sample_time=std::max(Policy::minimum_selected_samples*dt,
                                    2*Policy::minimum_samples_per_half*dt+age);
    if(observable_time<sample_time)
        return missing("mit_frequency_plan_complete_controller_set_exceeds_budget","budget_for_all_four_complete_frequency_windows_and_existing_baseline_transition_tail");
    out.observable_low_hz=Policy::low_frequency_cycles/observable_time;
    const double minimum_center=out.observable_low_hz/Policy::tone_ratios.front();
    const double maximum_center=out.observable_high_hz/Policy::tone_ratios.back();
    if(!std::isfinite(minimum_center) || !std::isfinite(maximum_center) || minimum_center>maximum_center)
        return missing("mit_frequency_plan_budget_and_feedback_band_do_not_overlap","complete_low_frequency_cycles_and_resolved_high_frequency_feedback_in_one_three_tone_window");
    // A shifted center is a scheduling choice, not a changed bandwidth claim.
    // The embedded velocity filter is unknown: a very high D/J prior does not
    // forbid useful measurement of the observable lower band (or vice versa).
    // Only the measured band may be used by the subsequent loop-shaping step.
    if(out.rigid_body_crossover_hz<out.observable_low_hz)
        out.missing_evidence.push_back("rigid_body_prior_below_complete_low_frequency_budget_coverage");
    if(out.rigid_body_crossover_hz>out.observable_high_hz)
        out.missing_evidence.push_back("rigid_body_prior_above_measured_feedback_observable_band");
    const double preferred=request.preferred_center_hz>0?request.preferred_center_hz:out.rigid_body_crossover_hz;
    out.selected_center_hz=std::clamp(preferred,minimum_center,maximum_center);
    out.feedback_band_limited=out.selected_center_hz<preferred;
    out.budget_band_limited=out.selected_center_hz>preferred;
    for(unsigned k=0;k<3;++k)out.frequencies_hz[k]=Policy::tone_ratios[k]*out.selected_center_hz;
    out.prior_crossover_covered=out.rigid_body_crossover_hz>=out.frequencies_hz.front() &&
        out.rigid_body_crossover_hz<=out.frequencies_hz.back();
    if(!out.prior_crossover_covered && out.missing_evidence.empty())
        out.missing_evidence.push_back("rigid_body_prior_outside_selected_partial_measurement_band");
    const double observation=std::max(Policy::low_frequency_cycles/out.frequencies_hz.front(),sample_time);
    const double duration=Policy::excluded_edge_time_s+age+observation;
    out.probe_duration_s=std::ceil(duration/dt-1e-10)*dt;
    out.total_cost_s=Policy::controller_experiments*(out.probe_duration_s+request.per_controller_overhead_s);
    if(!std::isfinite(out.total_cost_s) || out.total_cost_s>request.measurement_budget_s+1e-10)
        return missing("mit_frequency_plan_complete_window_cannot_fit_existing_budget","all_four_controller_experiments_with_complete_tone_and_fresh_feedback_coverage");
    out.eligible=true;
    out.reason=out.feedback_band_limited?"mit_frequency_plan_center_limited_by_measured_feedback":
        out.budget_band_limited?"mit_frequency_plan_center_limited_by_complete_experiment_budget":
        request.preferred_center_hz>0?"mit_frequency_plan_budgeted_numeric_band_candidate":
        "mit_frequency_plan_from_current_rigid_body_loop_prior";
    return out;
}

struct MitFrequencyCoDesignRequest {
    MitFrequencyPlanRequest frequency;
    MitFrequencyTorquePlanRequest torque;
};
struct MitFrequencyCoDesignPlan {
    bool valid{false},eligible{false};
    MitFrequencyPlan frequency;
    MitFrequencyTorquePlan torque;
    std::size_t numeric_bands_examined{0};
    std::string reason;
    std::vector<std::string> missing_evidence;
};

// All comparisons here are numerical input planning, not hardware trials.
// Half-octave candidates descend from the closest observable rigid-body prior
// to the lowest band that all four full windows can afford. Return the first
// signal-resolved plan; never extend the caller's existing wall-clock budget.
inline MitFrequencyCoDesignPlan plan_mit_frequency_experiment_co_design(MitFrequencyCoDesignRequest request) {
    MitFrequencyCoDesignPlan out;
    if(request.frequency.inertia_kg_m2!=request.torque.inertia_kg_m2 ||
       request.frequency.actual_kp!=request.torque.actual_kp[0] || request.frequency.actual_kd!=request.torque.actual_kd[0]) {
        out.reason="mit_frequency_co_design_model_or_actual_controller_mapping_mismatch";
        out.missing_evidence.push_back("same_URDF_local_inertia_and_actual_incumbent_controller_in_frequency_and_input_plan");return out;
    }
    request.frequency.preferred_center_hz=0;
    out.frequency=plan_mit_frequency_experiment(request.frequency);out.valid=out.frequency.valid;
    if(!out.frequency.eligible) {
        out.reason=out.frequency.reason;out.missing_evidence=out.frequency.missing_evidence;return out;
    }
    const double lowest_center=out.frequency.observable_low_hz/MitFrequencyPlanPolicy::tone_ratios.front();
    double center=out.frequency.selected_center_hz;
    for(;;) {
        request.frequency.preferred_center_hz=center;
        out.frequency=plan_mit_frequency_experiment(request.frequency);
        request.torque.frequencies_hz=out.frequency.frequencies_hz;
        out.torque=plan_mit_frequency_torque_experiment(request.torque);++out.numeric_bands_examined;
        if(!out.torque.valid && out.torque.reason.find("invalid")!=std::string::npos) {
            out.valid=false;out.reason=out.torque.reason;out.missing_evidence=out.torque.missing_evidence;return out;
        }
        if(out.frequency.eligible && out.torque.eligible) {
            out.eligible=true;out.reason="mit_frequency_co_design_resolved_common_input_and_complete_four_controller_budget";
            out.missing_evidence=out.frequency.missing_evidence;
            out.missing_evidence.insert(out.missing_evidence.end(),out.torque.missing_evidence.begin(),out.torque.missing_evidence.end());return out;
        }
        if(center<=lowest_center)break;
        const double next=std::max(lowest_center,center*std::sqrt(.5));
        if(next>=center)break;
        center=next;
    }
    out.reason="mit_frequency_co_design_no_resolved_band_within_existing_budget_and_demonstrated_domain";
    out.missing_evidence=out.torque.missing_evidence;
    out.missing_evidence.push_back("lower_frequency_signal_resolution_requires_more_existing_time_or_demonstrated_room_not_invented_measurements");
    return out;
}
} // namespace serial_arm::servo_tuning
