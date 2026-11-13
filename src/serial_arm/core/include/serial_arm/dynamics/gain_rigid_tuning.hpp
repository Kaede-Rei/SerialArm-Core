#pragma once

#include "serial_arm/dynamics/gain_compliant_tuning.hpp"

namespace serial_arm::servo_tuning {

// These are measured, fixed-reference hold quantities, not accuracy targets.
// A complete hold includes fresh continuous feedback, known zero independent
// input and a single actual PD group; the caller keeps its raw-cycle record.
struct RigidHoldEvidence {
    bool complete{false},gains_constant{false},reference_fixed{false},input_known_zero{false};
    bool limited{false},sustained_oscillation{false};
    double static_error_rad{std::numeric_limits<double>::quiet_NaN()};
    double drift_rad{std::numeric_limits<double>::quiet_NaN()};
    double speed_rad_s{std::numeric_limits<double>::quiet_NaN()};
    double motion_floor_rad{0};
    double reference_position_rad{std::numeric_limits<double>::quiet_NaN()};
};
struct RigidUseEvidence {
    CompliantUseAssessment disturbance;
    RigidHoldEvidence hold;
    double measured_kp{std::numeric_limits<double>::quiet_NaN()};
    double measured_kd{std::numeric_limits<double>::quiet_NaN()};
    bool fixed_gain_measurement_complete{false};
    // Same accepted torque waveform, direction and timing, excluding PD.
    // Execution scope includes load/feedforward/path declarations. Neither
    // signature is inferred from the input peak or a final pass/fail label.
    std::string input_signature,execution_scope;
    double time_resolution_s{.02};
};

namespace rigid_tuning_detail {
inline bool nonnegative(double value){return std::isfinite(value) && value>=0;}
inline bool same(double a,double b){return std::isfinite(a) && std::isfinite(b) && std::abs(a-b)<=1e-9*std::max({1.0,std::abs(a),std::abs(b)});}
inline bool hold_known(const RigidHoldEvidence& value) {
    return value.complete && value.gains_constant && value.reference_fixed && value.input_known_zero &&
        nonnegative(value.static_error_rad) && nonnegative(value.drift_rad) && nonnegative(value.speed_rad_s) &&
        nonnegative(value.motion_floor_rad) && std::isfinite(value.reference_position_rad);
}
inline bool input_known(const CompliantUseAssessment& value) {
    // A stiff response below position resolution still has complete input
    // evidence. Requiring visible yielding here would invert the rigid goal.
    return value.input_measurement_complete && value.input_known && value.reference_fixed && value.gains_constant &&
        std::isfinite(value.input_peak_nm) && value.input_peak_nm>0 && nonnegative(value.motion_floor_rad) &&
        std::isfinite(value.initial_equilibrium_rad) && std::isfinite(value.initial_reference_error_rad) &&
        nonnegative(value.peak_displacement_rad) && nonnegative(value.final_reference_error_rad) &&
        nonnegative(value.final_equilibrium_error_rad) && nonnegative(value.final_drift_rad) &&
        nonnegative(value.final_speed_rad_s) && nonnegative(value.release_recovery_s);
}
inline bool measured(const RigidUseEvidence& value,double p,double d) {
    return same(value.measured_kp,p) && same(value.measured_kd,d) &&
        (value.fixed_gain_measurement_complete || input_known(value.disturbance) || hold_known(value.hold));
}
inline double step(double value,double maximum,double quantum,int direction) {
    constexpr double fraction=.12;
    double target=value*(direction>0?1+fraction:1-fraction);
    target=quantize_gain(std::min(target,maximum),quantum,maximum,direction<0);
    if(quantum>0) {
        const auto code=gain_grid_code(value,quantum,maximum);
        if(direction>0 && target<=value)target=quantize_gain(std::min(maximum,(code+1)*quantum),quantum,maximum);
        if(direction<0 && target>=value && code>1)target=quantize_gain((code-1)*quantum,quantum,maximum,true);
    }
    // Restoring stiffness remains strictly positive; zero damping is legal.
    return target;
}
inline bool changed(double a,double b){return !same(a,b);}
}

// A small empirical comparison, not an identified optimum. Known independent
// displacement or measured unloaded static error gives a stiffness direction;
// repeated measured release oscillation gives a reduction comparison. No
// square-root P/D linkage, precision gate or robot-specific gain is used.
// The caller must admit the unchanged complete group with the existing
// admit_complete_gain_candidate() before execution and remeasure it afterward.
inline TrajectoryCandidateReport rigid_candidate_report(const Gains& current,
    const std::vector<double>& kp_max,const std::vector<double>& kd_max,
    const std::vector<RigidUseEvidence>& measured,const std::vector<GainCommandGrid>& grids={}) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || kp_max.size()!=n || kd_max.size()!=n || measured.size()!=n ||
       (!grids.empty() && grids.size()!=n))throw std::invalid_argument("rigid candidate mapping mismatch");
    TrajectoryCandidateReport out;out.gains=current;out.reasons.resize(n);
    for(std::size_t i=0;i<n;++i) {
        const auto grid=grids.empty()?GainCommandGrid{}:grids[i];
        const double p=current.kp[i],d=current.kd[i];
        if(!std::isfinite(p) || p<=0 || !rigid_tuning_detail::nonnegative(d) ||
           !std::isfinite(kp_max[i]) || kp_max[i]<=0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 ||
           p>kp_max[i] || d>kd_max[i] || !rigid_tuning_detail::nonnegative(grid.kp_step) ||
           !rigid_tuning_detail::nonnegative(grid.kd_step))throw std::invalid_argument("rigid candidate requires restoring gains and protocol capability");
        if(!rigid_tuning_detail::same(quantize_gain(p,grid.kp_step,kp_max[i]),p) ||
           !rigid_tuning_detail::same(quantize_gain(d,grid.kd_step,kd_max[i]),d))
            throw std::invalid_argument("rigid source gains must already match the protocol grid");
        const auto& evidence=measured[i];const auto& response=evidence.disturbance;const auto& hold=evidence.hold;
        auto& reason=out.reasons[i];reason="rigid_evidence_incomplete_no_gain_revision";
        if(!rigid_tuning_detail::measured(evidence,p,d))continue;
        const bool disturbance=rigid_tuning_detail::input_known(response),holding=rigid_tuning_detail::hold_known(hold);
        if((disturbance && response.limited) || (holding && hold.limited)) {
            reason="rigid_limited_measurement_requires_input_and_capability_review";continue;
        }
        const bool repeated=(disturbance && response.sustained_oscillation) || (holding && hold.sustained_oscillation);
        if(repeated) {
            const double lower=rigid_tuning_detail::step(p,kp_max[i],grid.kp_step,-1);
            if(lower>0 && lower<p) {
                out.gains.kp[i]=lower;reason="rigid_repeated_release_oscillation_bounded_stiffness_reduction_comparison";
            } else if(d<kd_max[i]) {
                out.gains.kd[i]=rigid_tuning_detail::step(d,kd_max[i],grid.kd_step,1);
                reason="rigid_repeated_release_oscillation_adjacent_damping_comparison";
            } else reason="rigid_repeated_release_oscillation_no_lower_restoring_code_or_damping_room";
        } else if((disturbance && (response.peak_displacement_rad>response.motion_floor_rad ||
                                  response.final_reference_error_rad>response.motion_floor_rad)) ||
                  (holding && hold.static_error_rad>hold.motion_floor_rad)) {
            out.gains.kp[i]=rigid_tuning_detail::step(p,kp_max[i],grid.kp_step,1);
            reason=out.gains.kp[i]>p?"rigid_resolved_displacement_or_static_error_bounded_stiffness_comparison":
                "rigid_stiffness_protocol_ceiling_no_further_local_step";
        } else reason="rigid_response_unresolved_retain_complete_measured_group";
        out.changed=out.changed || rigid_tuning_detail::changed(out.gains.kp[i],p) || rigid_tuning_detail::changed(out.gains.kd[i],d);
    }
    return out;
}

// Keep one complete actually measured group. Scope, input and actual gains
// must match; no axis is assembled from an independently preferred trial.
// All quantities are references for empirical selection, never pass targets.
inline bool rigid_use_preferred(const Gains& candidate_gains,const std::vector<RigidUseEvidence>& candidate,
    const Gains& baseline_gains,const std::vector<RigidUseEvidence>& baseline) {
    const auto n=baseline.size();
    if(!n || candidate.size()!=n || candidate_gains.kp.size()!=n || candidate_gains.kd.size()!=n ||
       baseline_gains.kp.size()!=n || baseline_gains.kd.size()!=n)
        throw std::invalid_argument("rigid use comparison mapping mismatch");
    bool improved=false;
    for(std::size_t i=0;i<n;++i) {
        const auto& a=candidate[i];const auto& b=baseline[i];
        if(!rigid_tuning_detail::measured(a,candidate_gains.kp[i],candidate_gains.kd[i]) ||
           !rigid_tuning_detail::measured(b,baseline_gains.kp[i],baseline_gains.kd[i]) ||
           candidate_gains.kp[i]<=0 || baseline_gains.kp[i]<=0 || candidate_gains.kd[i]<0 || baseline_gains.kd[i]<0 ||
           a.input_signature.empty() || a.input_signature!=b.input_signature ||
           a.execution_scope.empty() || a.execution_scope!=b.execution_scope ||
           !std::isfinite(a.time_resolution_s) || a.time_resolution_s<=0 ||
           !std::isfinite(b.time_resolution_s) || b.time_resolution_s<=0)return false;
        const auto& x=a.disturbance;const auto& y=b.disturbance;
        const bool ax=rigid_tuning_detail::input_known(x),by=rigid_tuning_detail::input_known(y);
        const bool ah=rigid_tuning_detail::hold_known(a.hold),bh=rigid_tuning_detail::hold_known(b.hold);
        const bool gains_changed=rigid_tuning_detail::changed(candidate_gains.kp[i],baseline_gains.kp[i]) ||
            rigid_tuning_detail::changed(candidate_gains.kd[i],baseline_gains.kd[i]);
        if((by && !ax) || (bh && !ah) || (gains_changed && !(ax && by) && !(ah && bh)))return false;
        if((x.limited && !y.limited) || (a.hold.limited && !b.hold.limited) ||
           (x.sustained_oscillation && (!y.sustained_oscillation || x.release_reversals>y.release_reversals)) ||
           (a.hold.sustained_oscillation && !b.hold.sustained_oscillation))return false;
        if(ax && by) {
            if(x.limited || y.limited || x.validation_scope!=y.validation_scope ||
               !rigid_tuning_detail::same(x.input_peak_nm,y.input_peak_nm) ||
               !rigid_tuning_detail::same(x.initial_equilibrium_rad-x.initial_reference_error_rad,
                                        y.initial_equilibrium_rad-y.initial_reference_error_rad))return false;
            const double noise=std::max(x.motion_floor_rad,y.motion_floor_rad);
            const double time=std::max(a.time_resolution_s,b.time_resolution_s);
            // The same tail speed estimator uses a .5 s observation window.
            const double speed_noise=2*noise;
            if(x.peak_displacement_rad>y.peak_displacement_rad+noise ||
               x.final_reference_error_rad>y.final_reference_error_rad+noise ||
               x.final_equilibrium_error_rad>y.final_equilibrium_error_rad+noise ||
               x.final_drift_rad>y.final_drift_rad+noise ||
               x.final_speed_rad_s>y.final_speed_rad_s+speed_noise ||
               x.release_recovery_s>y.release_recovery_s+time)return false;
            improved=improved || x.peak_displacement_rad<y.peak_displacement_rad-noise ||
                x.final_reference_error_rad<y.final_reference_error_rad-noise;
        }
        if(ah && bh) {
            if(a.hold.limited || b.hold.limited || !rigid_tuning_detail::same(a.hold.reference_position_rad,b.hold.reference_position_rad))return false;
            const double noise=std::max(a.hold.motion_floor_rad,b.hold.motion_floor_rad);
            if(a.hold.static_error_rad>b.hold.static_error_rad+noise || a.hold.drift_rad>b.hold.drift_rad+noise ||
               a.hold.speed_rad_s>b.hold.speed_rad_s+2*noise)return false;
            improved=improved || a.hold.static_error_rad<b.hold.static_error_rad-noise;
        }
    }
    return improved;
}

} // namespace serial_arm::servo_tuning
