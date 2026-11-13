#pragma once

#include "serial_arm/dynamics/gain_drag_tuning.hpp"

namespace serial_arm::servo_tuning {

// Encode the design ceiling before encoding the candidate. A sub-code
// restoring gain is missing capability, never an exception halfway through
// an otherwise measurable purpose. All later trials use this same group.
inline bool encode_compliant_candidate(Gains& candidate,std::vector<double>& pmax,std::vector<double>& dmax,
    const std::vector<ModelAxisLimits>& limits) {
    const auto n=limits.size();
    if(!n || candidate.kp.size()!=n || candidate.kd.size()!=n || pmax.size()!=n || dmax.size()!=n)return false;
    for(std::size_t i=0;i<n;++i) {
        const auto& x=limits[i];
        pmax[i]=quantize_gain(pmax[i],x.grid.kp_step,x.kp_max);
        dmax[i]=quantize_gain(dmax[i],x.grid.kd_step,x.kd_max);
        if(pmax[i]<=0 || dmax[i]<=0)return false;
        candidate.kp[i]=std::min(pmax[i],quantize_gain(candidate.kp[i],x.grid.kp_step,x.kp_max,true));
        candidate.kd[i]=std::min(dmax[i],quantize_gain(candidate.kd[i],x.grid.kd_step,x.kd_max,true));
        if(candidate.kp[i]<=0 || candidate.kd[i]<=0)return false;
    }
    return true;
}

// A compliant use renders a bounded force/displacement response and recovers
// after unloading. The caller supplies its local/model response envelope;
// rigid trajectory precision is deliberately not a compliance requirement.
struct CompliantUseRequirements {
    double maximum_displacement_rad{.04},recovery_error_rad{.008},final_drift_rad{.004};
    double final_speed_rad_s{.01},recovery_time_s{1.5},tail_s{1.5};
};
struct CompliantUseAssessment {
    bool measurement_complete{false},input_measurement_complete{false},passed{false},limited{false},input_known{false},yield_observed{false};
    bool stable_recovery{false},sustained_oscillation{false},reference_fixed{false},gains_constant{false};
    std::size_t fresh_samples{0},release_reversals{0};
    double input_peak_nm{0},motion_floor_rad{0},initial_equilibrium_rad{0},initial_reference_error_rad{0};
    double peak_displacement_rad{0},signed_compliance_rad_nm{0},release_recovery_s{0};
    double final_equilibrium_error_rad{0},final_reference_error_rad{0},final_drift_rad{0},final_speed_rad_s{0};
    double displacement_limit_rad{0};
    double measured_kp{std::numeric_limits<double>::quiet_NaN()},measured_kd{std::numeric_limits<double>::quiet_NaN()};
    double encoder_step_rad{0},position_noise_rms_rad{0};
    double displacement_mean_rad{std::numeric_limits<double>::quiet_NaN()};
    double displacement_mean_uncertainty_rad{std::numeric_limits<double>::quiet_NaN()};
    double final_reference_mean_error_rad{std::numeric_limits<double>::quiet_NaN()};
    double final_reference_mean_uncertainty_rad{std::numeric_limits<double>::quiet_NaN()};
    std::size_t mean_baseline_samples{0},mean_hold_samples{0},mean_tail_samples{0};
    unsigned unresolved_input_direction_mask{0};
    bool unresolved_both_input_directions_at_ceiling{false};
    std::string reason{"compliant_use_not_measured"};
    std::string validation_scope{"known_joint_torque_yield_and_unloaded_equilibrium_recovery_not_contact_force"};
};

namespace compliant_use_detail {
struct MeanEvidence {double mean{0},uncertainty{0};std::size_t samples{0};};
inline MeanEvidence mean_evidence(const std::vector<double>& values,const NoiseEstimate& noise) {
    MeanEvidence out;out.samples=values.size();if(values.empty())return out;
    for(double x:values)out.mean+=x;
    out.mean/=values.size();
    double variance=0,covariance=0;
    for(std::size_t i=0;i<values.size();++i) {
        const double x=values[i]-out.mean;variance+=x*x;
        if(i)covariance+=x*(values[i-1]-out.mean);
    }
    variance/=values.size();
    const double rho=variance>0?std::clamp(covariance/(values.size()*variance),0.0,.99):0;
    const double effective=std::clamp(values.size()*(1-rho)/(1+rho),1.0,static_cast<double>(values.size()));
    // Repeated copies of one code never average away its quantization
    // interval. Serial correlation reduces, rather than invents, evidence.
    out.uncertainty=.5*noise.encoder_step_rad+3*std::sqrt(std::max(variance,noise.rms_rad*noise.rms_rad)/effective);
    return out;
}
inline void mean_response(CompliantUseAssessment& out,const std::vector<double>& baseline,
    const std::vector<double>& held,const std::vector<double>& tail_error,const NoiseEstimate& noise) {
    out.encoder_step_rad=noise.encoder_step_rad;out.position_noise_rms_rad=noise.rms_rad;
    const auto b=mean_evidence(baseline,noise),h=mean_evidence(held,noise),t=mean_evidence(tail_error,noise);
    out.mean_baseline_samples=b.samples;out.mean_hold_samples=h.samples;out.mean_tail_samples=t.samples;
    if(b.samples>=3 && h.samples>=3) {
        out.displacement_mean_rad=std::abs(h.mean-b.mean);
        out.displacement_mean_uncertainty_rad=b.uncertainty+h.uncertainty;
    }
    if(t.samples>=3){out.final_reference_mean_error_rad=std::abs(t.mean);out.final_reference_mean_uncertainty_rad=t.uncertainty;}
}
}

inline CompliantUseRequirements compliant_use_requirements(double local_range_rad,
    double model_predicted_displacement_rad,const NoiseEstimate& noise) {
    if(!std::isfinite(local_range_rad) || local_range_rad<=0 ||
       !std::isfinite(model_predicted_displacement_rad) || model_predicted_displacement_rad<=0)
        throw std::invalid_argument("compliant use requires a declared local and model response envelope");
    const double floor=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    CompliantUseRequirements out;
    out.maximum_displacement_rad=std::min(local_range_rad,std::max(2*model_predicted_displacement_rad,4*floor));
    out.recovery_error_rad=std::max(floor,std::min(.008,.25*out.maximum_displacement_rad));
    out.final_drift_rad=std::max(floor,std::min(.004,.125*out.maximum_displacement_rad));
    return out;
}

inline CompliantUseAssessment assess_compliant_response(const std::vector<Sample>& rows,
    const NoiseEstimate& noise,double release_s,const CompliantUseRequirements& req={}) {
    CompliantUseAssessment out;out.displacement_limit_rad=req.maximum_displacement_rad;
    for(double value:{req.maximum_displacement_rad,req.recovery_error_rad,req.final_drift_rad,
        req.final_speed_rad_s,req.recovery_time_s,req.tail_s})
        if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid compliant-use requirement");
    if(rows.empty() || !std::isfinite(release_s) || release_s<=rows.front().time_s)return out;
    const auto measured=drag_use_detail::window(rows,rows.front().time_s,rows.back().time_s);
    const auto& fresh=measured.rows;out.fresh_samples=fresh.size();if(fresh.empty())return out;
    out.measured_kp=fresh.front().actual_kp;out.measured_kd=fresh.front().actual_kd;
    out.motion_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    double origin=0,reference=0;std::size_t origin_samples=0;
    std::vector<double> baseline_positions,held_positions,tail_errors;
    // A quiet pre-input baseline is mandatory. A state already displaced by
    // the injected torque cannot be relabelled as the unloaded equilibrium.
    for(const auto& row:fresh)if(row.time_s<=fresh.front().time_s+.15 && row.time_s<release_s &&
        row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm) &&
        std::abs(row.independent_disturbance_nm)<=1e-10) {
        origin+=row.actual;reference+=row.reference;++origin_samples;baseline_positions.push_back(row.actual);
    }
    if(origin_samples<3){out.reason="compliant_unloaded_baseline_missing";return out;}
    origin/=origin_samples;reference/=origin_samples;out.initial_equilibrium_rad=origin;
    out.initial_reference_error_rad=origin-reference;out.reference_fixed=true;
    out.input_known=true;out.gains_constant=measured.known_gains && measured.constant_gains;
    bool positive_stiffness=true,input_removed=true,baseline_still=true;
    double baseline_low=origin,baseline_high=origin,input_energy=0,input_displacement=0;
    double tail_low=1e30,tail_high=-1e30,last_bad=release_s;
    const double tail_start=std::max(release_s+.05,fresh.back().time_s-.5);
    std::size_t tail_count=0;std::vector<Sample> released,history;
    for(const auto& row:fresh) {
        out.limited=out.limited || row.limited;
        positive_stiffness=positive_stiffness && row.actual_kp>0;
        out.reference_fixed=out.reference_fixed && std::isfinite(row.reference) &&
            std::isfinite(row.reference_velocity) && std::abs(row.reference-reference)<=std::max(1e-7,noise.encoder_step_rad) &&
            std::abs(row.reference_velocity)<=1e-8;
        out.input_known=out.input_known && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm);
        const double displacement=row.actual-origin;
        out.peak_displacement_rad=std::max(out.peak_displacement_rad,std::abs(displacement));
        if(row.time_s<=fresh.front().time_s+.15 && std::abs(row.independent_disturbance_nm)<=1e-10) {
            baseline_low=std::min(baseline_low,row.actual);baseline_high=std::max(baseline_high,row.actual);
        }
        if(row.time_s<release_s && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm)) {
            const double input=row.independent_disturbance_nm;
            out.input_peak_nm=std::max(out.input_peak_nm,std::abs(input));
            input_energy+=input*input;input_displacement+=input*displacement;
            if(row.time_s>=fresh.front().time_s+.75*(release_s-fresh.front().time_s))held_positions.push_back(row.actual);
        }
        if(row.time_s>=release_s) {
            input_removed=input_removed && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm) &&
                std::abs(row.independent_disturbance_nm)<=1e-10;
            released.push_back(row);history.push_back(row);
            if(std::abs(displacement)>req.recovery_error_rad ||
               drag_position_speed(history,noise,.25)>req.final_speed_rad_s)last_bad=row.time_s;
        }
        if(row.time_s>=tail_start) {
            tail_errors.push_back(row.actual-row.reference);
            ++tail_count;tail_low=std::min(tail_low,row.actual);tail_high=std::max(tail_high,row.actual);
            out.final_equilibrium_error_rad=std::max(out.final_equilibrium_error_rad,std::abs(displacement));
            out.final_reference_error_rad=std::max(out.final_reference_error_rad,std::abs(row.actual-row.reference));
        }
    }
    baseline_still=baseline_high-baseline_low<=out.motion_floor_rad;
    out.signed_compliance_rad_nm=input_energy>1e-12?input_displacement/input_energy:0;
    // This finite-window ratio describes this input only; it is not a claimed
    // static stiffness identification or a measured end-effector contact force.
    out.yield_observed=out.peak_displacement_rad>out.motion_floor_rad && out.signed_compliance_rad_nm>0;
    out.final_drift_rad=tail_count?tail_high-tail_low:std::numeric_limits<double>::infinity();
    compliant_use_detail::mean_response(out,baseline_positions,held_positions,tail_errors,noise);
    out.final_speed_rad_s=drag_position_speed(released,noise,.5);
    out.release_reversals=tail_position_reversals(released,release_s,out.motion_floor_rad);
    out.sustained_oscillation=out.release_reversals>=4 && out.final_drift_rad>out.motion_floor_rad;
    out.release_recovery_s=std::max(0.0,last_bad-release_s);
    out.input_measurement_complete=measured.continuous && out.gains_constant && positive_stiffness && out.input_known &&
        out.input_peak_nm>1e-8 && input_removed && baseline_still && out.reference_fixed && fresh.size()>=100 &&
        fresh.back().time_s-release_s>=req.tail_s-.02 && tail_count>=20;
    out.measurement_complete=out.input_measurement_complete && out.yield_observed;
    out.stable_recovery=out.final_equilibrium_error_rad<=req.recovery_error_rad && out.final_drift_rad<=req.final_drift_rad &&
        out.final_speed_rad_s<=req.final_speed_rad_s && out.release_recovery_s<=req.recovery_time_s &&
        fresh.back().time_s-last_bad>=.25 && !out.sustained_oscillation;
    if(!out.input_known || out.input_peak_nm<=1e-8)out.reason="known_independent_compliant_input_missing";
    else if(!input_removed)out.reason="compliant_input_not_removed";
    else if(!positive_stiffness || !out.gains_constant)out.reason="compliant_requires_known_constant_restoring_gains";
    else if(!out.reference_fixed)out.reason="compliant_yield_requires_fixed_reference";
    else if(!baseline_still)out.reason="compliant_unloaded_baseline_not_stationary";
    else if(!measured.continuous || fresh.size()<100)out.reason="insufficient_fresh_compliant_feedback";
    else if(out.limited)out.reason="limited_compliant_input_or_response";
    else if(!out.yield_observed)out.reason="compliant_yield_unresolved_do_not_substitute_rigid_tracking";
    else if(out.peak_displacement_rad>req.maximum_displacement_rad)out.reason="compliant_yield_above_model_local_envelope";
    else if(out.sustained_oscillation)out.reason="compliant_repeated_release_oscillation";
    else if(!out.measurement_complete)out.reason="insufficient_compliant_release_window";
    else if(!out.stable_recovery)out.reason="compliant_unloaded_equilibrium_recovery_incomplete";
    else {out.passed=true;out.reason="bounded_compliant_yield_and_stable_recovery_qualified";}
    return out;
}

inline TrajectoryCandidateReport compliant_candidate_report(const Gains& current,
    const std::vector<double>& kp_max,const std::vector<double>& kd_max,
    const std::vector<CompliantUseAssessment>& measured,const std::vector<GainCommandGrid>& grids={}) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || kp_max.size()!=n || kd_max.size()!=n || measured.size()!=n ||
       (!grids.empty() && grids.size()!=n))
        throw std::invalid_argument("compliant candidate mapping mismatch");
    TrajectoryCandidateReport out;out.gains=current;out.reasons.resize(n);
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(current.kp[i]) || current.kp[i]<=0 || !std::isfinite(current.kd[i]) || current.kd[i]<0 ||
           !std::isfinite(kp_max[i]) || kp_max[i]<=0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 ||
           current.kp[i]>kp_max[i] || current.kd[i]>kd_max[i])
            throw std::invalid_argument("compliant candidate requires valid restoring gains and protocol capability");
        const auto& assessment=measured[i];auto& reason=out.reasons[i];
        if(assessment.measurement_complete && !assessment.limited && assessment.sustained_oscillation) {
            out.gains.kp[i]=.92*current.kp[i];reason="measured_compliant_release_reversals_small_stiffness_reduction_comparison";
        } else if(assessment.measurement_complete && !assessment.limited &&
            assessment.peak_displacement_rad>assessment.displacement_limit_rad) {
            out.gains.kp[i]=std::min(kp_max[i],1.08*current.kp[i]);reason="measured_yield_above_model_local_envelope_small_stiffness_comparison";
        } else if(assessment.input_measurement_complete && !assessment.yield_observed && !assessment.limited) {
            if(assessment.unresolved_both_input_directions_at_ceiling) {
                out.gains.kp[i]=.92*current.kp[i];
                reason="both_legal_input_directions_measured_unresolved_small_stiffness_reduction_comparison";
            } else reason="known_input_yield_unresolved_increase_measurement_input_before_gain_revision";
        } else if(assessment.measurement_complete && !assessment.limited && assessment.stable_recovery &&
                  assessment.peak_displacement_rad<std::max(2*assessment.motion_floor_rad,.25*assessment.displacement_limit_rad)) {
            out.gains.kp[i]=.92*current.kp[i];out.gains.kd[i]=.92*current.kd[i];
            reason="measured_small_yield_and_quiet_release_bounded_restoring_and_damping_reduction_comparison";
        } else if(assessment.measurement_complete && !assessment.limited && !assessment.sustained_oscillation &&
                  !assessment.stable_recovery && assessment.release_recovery_s>0 && current.kd[i]>0) {
            out.gains.kd[i]=.92*current.kd[i];
            reason="measured_slow_compliant_release_small_damping_reduction_comparison_not_a_cause_identification";
        } else reason=assessment.passed?"compliant_yield_and_recovery_goals_met":"compliant_evidence_incomplete_no_gain_revision";
        if(!grids.empty()) {
            const auto& grid=grids[i];
            for(double step:{grid.kp_step,grid.kd_step})if(!std::isfinite(step) || step<0)
                throw std::invalid_argument("invalid compliant candidate protocol grid");
            // Keep one positive restoring/damping code when a percentage
            // rounds away; the entire discrete group is still only a trial.
            if(grid.kp_step>0 && out.gains.kp[i]<current.kp[i])out.gains.kp[i]=
                std::max(grid.kp_step,quantize_gain(out.gains.kp[i],grid.kp_step,kp_max[i]));
            if(grid.kd_step>0 && out.gains.kd[i]<current.kd[i] && current.kd[i]>0)out.gains.kd[i]=
                std::max(grid.kd_step,quantize_gain(out.gains.kd[i],grid.kd_step,kd_max[i]));
        }
        out.changed=out.changed || out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i];
    }
    bool resolved_change=false;
    for(std::size_t i=0;i<n;++i)resolved_change=resolved_change || (measured[i].measurement_complete && measured[i].yield_observed &&
        (out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i]));
    if(resolved_change)for(std::size_t i=0;i<n;++i)if(measured[i].input_measurement_complete && !measured[i].yield_observed &&
        !measured[i].measurement_complete && !measured[i].unresolved_both_input_directions_at_ceiling &&
        (out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i])) {
        // Design this subset before measuring the complete new group. Never
        // splice separately measured axis winners after the comparison.
        out.gains.kp[i]=current.kp[i];out.gains.kd[i]=current.kd[i];
        out.reasons[i]="unresolved_yield_axis_unchanged_during_resolved_subset_whole_group_comparison";
    }
    return out;
}

// Compare whole measured purposes under the same independent input. Passing
// the final requirement is reported separately: two unqualified controllers
// may still have a resolved improvement worth retaining. Never trade a worse
// axis, lost yielding evidence or newly limited/oscillatory motion for gains
// elsewhere. The caller also checks the complete same-condition path trials.
inline bool compliant_use_preferred_impl(const std::vector<CompliantUseAssessment>& candidate,
    const std::vector<CompliantUseAssessment>& incumbent,const Gains* candidate_gains,const Gains* incumbent_gains,
    bool candidate_completed,bool incumbent_completed) {
    if(candidate.empty() || candidate.size()!=incumbent.size())
        throw std::invalid_argument("compliant purpose comparison mapping mismatch");
    const bool mapped_gains=candidate_gains && incumbent_gains;
    if(mapped_gains && (candidate_gains->kp.size()!=candidate.size() || candidate_gains->kd.size()!=candidate.size() ||
        incumbent_gains->kp.size()!=candidate.size() || incumbent_gains->kd.size()!=candidate.size()))
        throw std::invalid_argument("compliant purpose comparison gains mapping mismatch");
    if(mapped_gains && (!candidate_completed || !incumbent_completed))return false;
    bool improved=false;
    for(std::size_t i=0;i<candidate.size();++i) {
        const auto& a=candidate[i];const auto& b=incumbent[i];
        const bool unchanged=mapped_gains && candidate_gains->kp[i]==incumbent_gains->kp[i] &&
            candidate_gains->kd[i]==incumbent_gains->kd[i];
        const bool measured_parameters=mapped_gains && std::isfinite(a.measured_kp) && std::isfinite(a.measured_kd) &&
            std::isfinite(b.measured_kp) && std::isfinite(b.measured_kd) &&
            std::abs(a.measured_kp-candidate_gains->kp[i])<1e-6 && std::abs(a.measured_kd-candidate_gains->kd[i])<1e-6 &&
            std::abs(b.measured_kp-incumbent_gains->kp[i])<1e-6 && std::abs(b.measured_kd-incumbent_gains->kd[i])<1e-6;
        if(mapped_gains && !measured_parameters)return false;
        const bool unresolved_unchanged=unchanged && measured_parameters && !a.yield_observed && !b.yield_observed &&
            !a.measurement_complete && !b.measurement_complete && a.motion_floor_rad>0 && b.motion_floor_rad>0 &&
            a.peak_displacement_rad<=a.motion_floor_rad && b.peak_displacement_rad<=b.motion_floor_rad;
        if((!unresolved_unchanged && (!a.measurement_complete || !a.yield_observed || (b.yield_observed && !b.measurement_complete))) ||
           !a.input_measurement_complete || !b.input_measurement_complete ||
           !a.input_known || !b.input_known || !a.reference_fixed || !b.reference_fixed ||
           !a.gains_constant || !b.gains_constant || a.limited || b.limited || a.sustained_oscillation ||
           a.validation_scope!=b.validation_scope)return false;
        for(const auto* value:{&a,&b}) {
            for(double metric:{value->motion_floor_rad,value->input_peak_nm,value->displacement_limit_rad,
                value->peak_displacement_rad,value->final_equilibrium_error_rad,value->final_drift_rad,
                value->final_speed_rad_s,value->release_recovery_s})
                if(!std::isfinite(metric) || metric<0)return false;
            if(value->input_peak_nm<=0 || value->displacement_limit_rad<=0)return false;
        }
        if((!unresolved_unchanged && (!std::isfinite(a.signed_compliance_rad_nm) || a.signed_compliance_rad_nm<=0)) ||
           std::abs(a.input_peak_nm-b.input_peak_nm)>std::max(1e-9,1e-6*std::max(a.input_peak_nm,b.input_peak_nm)))
            return false;
        const double floor=std::max({a.motion_floor_rad,b.motion_floor_rad,.0002});
        // A new model-derived envelope cannot enlarge the old measured scope
        // and make a larger excursion look like an improvement. Displacement
        // inside the common envelope is useful yielding, not an error to zero.
        const double envelope=std::min(a.displacement_limit_rad,b.displacement_limit_rad);
        const double excess=std::max(0.0,a.peak_displacement_rad-envelope);
        const double old_excess=std::max(0.0,b.peak_displacement_rad-envelope);
        const double error_margin=std::max(floor,.05*b.final_equilibrium_error_rad);
        const double drift_margin=std::max(floor,.05*b.final_drift_rad);
        // The final speed estimator uses a half-second position window.
        const double speed_margin=std::max(2*floor,.05*b.final_speed_rad_s);
        const double time_margin=std::max(.05,.05*b.release_recovery_s);
        if(excess>old_excess+floor ||
           a.final_equilibrium_error_rad>b.final_equilibrium_error_rad+error_margin ||
           a.final_drift_rad>b.final_drift_rad+drift_margin ||
           a.final_speed_rad_s>b.final_speed_rad_s+speed_margin ||
           (b.yield_observed && !(a.stable_recovery && b.stable_recovery) &&
            a.release_recovery_s>b.release_recovery_s+time_margin))return false;
        if(unresolved_unchanged)continue; // Unknown yielding adds no improvement or pass claim.
        improved=improved || !b.yield_observed || b.sustained_oscillation ||
            (a.peak_displacement_rad<=envelope && b.peak_displacement_rad<=envelope &&
             a.peak_displacement_rad>b.peak_displacement_rad+floor &&
             a.signed_compliance_rad_nm>b.signed_compliance_rad_nm+floor/a.input_peak_nm) ||
            excess<old_excess-floor ||
            a.final_equilibrium_error_rad<b.final_equilibrium_error_rad-error_margin ||
            a.final_drift_rad<b.final_drift_rad-drift_margin ||
            a.final_speed_rad_s<b.final_speed_rad_s-speed_margin ||
            (b.yield_observed && a.release_recovery_s<b.release_recovery_s-time_margin);
    }
    return improved;
}

inline bool compliant_use_preferred(const std::vector<CompliantUseAssessment>& candidate,
    const std::vector<CompliantUseAssessment>& incumbent) {
    return compliant_use_preferred_impl(candidate,incumbent,nullptr,nullptr,false,false);
}
inline bool compliant_use_preferred(const std::vector<CompliantUseAssessment>& candidate,
    const std::vector<CompliantUseAssessment>& incumbent,const Gains& candidate_gains,const Gains& incumbent_gains,
    bool candidate_completed,bool incumbent_completed) {
    return compliant_use_preferred_impl(candidate,incumbent,&candidate_gains,&incumbent_gains,candidate_completed,incumbent_completed);
}

} // namespace serial_arm::servo_tuning
