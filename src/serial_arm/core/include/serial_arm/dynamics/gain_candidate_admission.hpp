#pragma once

#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

struct WholeGainAdmission {
    bool allowed{false};
    std::vector<std::string> axis_reasons;
    std::vector<double> error_envelope_rad,velocity_error_envelope_rad_s;
    std::vector<double> feedback_effort_budget_nm;
    std::vector<double> incumbent_PD_envelope_nm,proposal_PD_envelope_nm,increase_PD_envelope_nm;
};

// The caller supplies fresh accepted-reference minus actual-feedback errors,
// after its existing feedback-age check. This admits one unchanged, complete
// candidate group for a real comparison; it does not validate that candidate
// or return a projected mixture of gains from different trials.
inline WholeGainAdmission admit_complete_gain_candidate(const Gains& incumbent,const Gains& proposal,
    const std::vector<ModelAxisLimits>& axes,const std::vector<double>& measured_peak_error_rad,
    const std::vector<double>& measured_peak_velocity_error_rad_s,
    const std::vector<double>& fresh_accepted_minus_actual_rad,
    const std::vector<double>& fresh_velocity_error_rad_s,
    const std::vector<double>& feedback_effort_budget_nm={}) {
    WholeGainAdmission out;
    const auto n=axes.size();
    out.axis_reasons.assign(n,"candidate_admission_input_valid");
    const auto matches=[&](const auto& values){return values.size()==n;};
    if(!n || !matches(incumbent.kp) || !matches(incumbent.kd) || !matches(proposal.kp) || !matches(proposal.kd) ||
       !matches(measured_peak_error_rad) || !matches(measured_peak_velocity_error_rad_s) ||
       !matches(fresh_accepted_minus_actual_rad) || !matches(fresh_velocity_error_rad_s) ||
       (!feedback_effort_budget_nm.empty() && !matches(feedback_effort_budget_nm))) {
        out.axis_reasons.assign(n,"candidate_admission_missing_or_mismatched_evidence");
        return out;
    }
    auto limits=axes;
    bool valid=true;
    for(std::size_t i=0;i<n;++i) {
        if(!feedback_effort_budget_nm.empty())limits[i].effort_budget=feedback_effort_budget_nm[i];
        const auto& bound=limits[i];
        const auto reject=[&](const char* reason){out.axis_reasons[i]=reason;valid=false;};
        if(!std::isfinite(bound.effort_budget) || bound.effort_budget<=0 ||
           !std::isfinite(bound.kp_max) || bound.kp_max<=0 || !std::isfinite(bound.kd_max) || bound.kd_max<=0 ||
           !std::isfinite(bound.grid.kp_step) || bound.grid.kp_step<0 ||
           !std::isfinite(bound.grid.kd_step) || bound.grid.kd_step<0) {
            reject("candidate_admission_invalid_protocol_or_effort_budget");continue;
        }
        if(!std::isfinite(measured_peak_error_rad[i]) || measured_peak_error_rad[i]<0 ||
           !std::isfinite(measured_peak_velocity_error_rad_s[i]) || measured_peak_velocity_error_rad_s[i]<0 ||
           !std::isfinite(fresh_accepted_minus_actual_rad[i]) || !std::isfinite(fresh_velocity_error_rad_s[i])) {
            reject("candidate_admission_missing_finite_measured_or_fresh_error_evidence");continue;
        }
        for(const auto* gains:{&incumbent,&proposal}) {
            if(!std::isfinite(gains->kp[i]) || gains->kp[i]<0 || !std::isfinite(gains->kd[i]) || gains->kd[i]<0) {
                reject("candidate_admission_nonfinite_or_negative_gain");break;
            }
            if(gains->kp[i]>bound.kp_max || gains->kd[i]>bound.kd_max) {
                reject("candidate_admission_gain_outside_protocol");break;
            }
        }
        if(out.axis_reasons[i]!="candidate_admission_input_valid")continue;
        try {
            // Check the proposed encoded group without silently changing it.
            for(const auto& [value,step,maximum]:{
                std::array<double,3>{proposal.kp[i],bound.grid.kp_step,bound.kp_max},
                std::array<double,3>{proposal.kd[i],bound.grid.kd_step,bound.kd_max}}) {
                const double encoded=quantize_gain(value,step,maximum);
                if(std::abs(encoded-value)>1e-10*std::max(1.0,std::abs(value))) {
                    reject("candidate_admission_protocol_encoding_would_change_group");break;
                }
            }
        } catch(const std::exception&) {reject("candidate_admission_invalid_protocol_or_effort_budget");}
    }
    if(!valid) {
        for(auto& reason:out.axis_reasons)if(reason=="candidate_admission_input_valid")
            reason="candidate_admission_not_evaluated_due_to_invalid_group_evidence";
        return out; // Empty numeric vectors mean evidence could not be evaluated.
    }
    out.error_envelope_rad.resize(n);out.velocity_error_envelope_rad_s.resize(n);
    out.feedback_effort_budget_nm.resize(n);out.incumbent_PD_envelope_nm.resize(n);
    out.proposal_PD_envelope_nm.resize(n);out.increase_PD_envelope_nm.resize(n);
    std::vector<double> helper_error(n);
    for(std::size_t i=0;i<n;++i) {
        const double E=std::max(measured_peak_error_rad[i],std::abs(fresh_accepted_minus_actual_rad[i]));
        const double V=std::max(measured_peak_velocity_error_rad_s[i],std::abs(fresh_velocity_error_rad_s[i]));
        out.error_envelope_rad[i]=E;out.velocity_error_envelope_rad_s[i]=V;
        out.feedback_effort_budget_nm[i]=limits[i].effort_budget;
        out.incumbent_PD_envelope_nm[i]=incumbent.kp[i]*E+incumbent.kd[i]*V;
        out.proposal_PD_envelope_nm[i]=proposal.kp[i]*E+proposal.kd[i]*V;
        out.increase_PD_envelope_nm[i]=std::max(0.0,proposal.kp[i]-incumbent.kp[i])*E+
            std::max(0.0,proposal.kd[i]-incumbent.kd[i])*V;
        // The shared helper divides by E. Zero measured error remains zero in
        // the evidence above; this positive arithmetic floor invents no motion.
        helper_error[i]=std::max(E,std::numeric_limits<double>::min());
        if(!std::isfinite(out.incumbent_PD_envelope_nm[i]) || !std::isfinite(out.proposal_PD_envelope_nm[i]) ||
           !std::isfinite(out.increase_PD_envelope_nm[i])) {
            out.axis_reasons[i]="candidate_admission_nonfinite_pd_envelope";valid=false;
        }
    }
    if(!valid) {
        out.incumbent_PD_envelope_nm.clear();out.proposal_PD_envelope_nm.clear();out.increase_PD_envelope_nm.clear();
        return out;
    }
    ConstrainedMeasuredGainStep bounded;
    try {bounded=constrain_measured_gain_step(incumbent,proposal,limits,helper_error,out.velocity_error_envelope_rad_s);}
    catch(const std::exception&) {
        out.axis_reasons.assign(n,"candidate_admission_measured_constraint_could_not_be_evaluated");return out;
    }
    out.allowed=true;
    for(std::size_t i=0;i<n;++i) {
        const double budget=out.feedback_effort_budget_nm[i],old=out.incumbent_PD_envelope_nm[i];
        const double requested=out.proposal_PD_envelope_nm[i],increase=out.increase_PD_envelope_nm[i];
        const double tolerance=1e-10*std::max({1.0,budget,old,requested});
        if(old>budget+tolerance) {
            // The incumbent was already measured. Reducing both components
            // adds no feedback demand, even while its tracking error is large.
            if(proposal.kp[i]<=incumbent.kp[i] && proposal.kd[i]<=incumbent.kd[i])
                out.axis_reasons[i]="candidate_admission_componentwise_reduction_from_measured_over_budget_group";
            else {
                out.axis_reasons[i]="candidate_admission_measured_envelope_has_no_room_for_feedback_increase";
                out.allowed=false;
            }
        } else if(requested>budget+tolerance || increase>std::max(0.0,budget-old)+tolerance) {
            out.axis_reasons[i]="candidate_admission_feedback_or_increase_envelope_exceeds_effort_budget";
            out.allowed=false;
        } else if(std::abs(bounded.gains.kp[i]-proposal.kp[i])>1e-10*std::max(1.0,proposal.kp[i]) ||
                  std::abs(bounded.gains.kd[i]-proposal.kd[i])>1e-10*std::max(1.0,proposal.kd[i])) {
            out.axis_reasons[i]="candidate_admission_measured_step_would_clip_complete_group";
            out.allowed=false;
        } else out.axis_reasons[i]="candidate_admission_complete_group_within_measured_and_fresh_effort_envelope";
    }
    return out;
}

} // namespace serial_arm::servo_tuning
