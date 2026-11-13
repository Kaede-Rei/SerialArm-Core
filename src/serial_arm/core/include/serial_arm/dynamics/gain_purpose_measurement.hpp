#pragma once

#include "serial_arm/dynamics/gain_compliant_tuning.hpp"

namespace serial_arm::servo_tuning {

// This is an input plan for one selected joint, not a mechanical model. The
// caller holds the whole PD group/reference fixed and commands zero additional
// torque on every other joint. Plan phases use the recorded control-sample
// clock; evaluation separately retains command-write and feedback clocks.
struct PurposeInputPrior {
    bool valid{false};
    std::size_t independent_samples{0},low_speed_pd_samples{0},rejected_samples{0};
    double independent_peak_nm{0},low_speed_pd_level_nm{0},seed_nm{0},low_speed_cutoff_rad_s{0};
    std::string source,reason{"purpose_input_prior_not_measured"};
    std::string interpretation{"known_independent_command_and_low_speed_host_PD_proxy_not_friction_identification"};
};

// A measured input-scale prior, not a friction model or an estimate of the
// internal motor feedback. Keep the independent input and the host PD proxy
// separately observable; do not substitute a planned/clipped/unknown input.
inline PurposeInputPrior estimate_purpose_input_prior(const std::vector<Sample>& rows,
    const NoiseEstimate& noise,const std::string& source_record_path) {
    PurposeInputPrior out;out.source=source_record_path;
    struct Point {double speed,pd;bool position_error_resolved;};std::vector<Point> unloaded;std::uint64_t sequence=0;
    const double position_floor=std::max(3*noise.encoder_step_rad,6*noise.rms_rad);
    for(const auto& r:rows) {
        if(!r.new_measurement || (r.feedback_sequence && r.feedback_sequence==sequence))continue;
        sequence=r.feedback_sequence;
        bool known=r.actual_gains_known && r.independent_disturbance_known && !r.limited;
        for(double x:{r.time_s,r.command_time_s,r.feedback_time_s,r.actual,r.velocity,r.reference,r.reference_velocity,
                      r.actual_kp,r.actual_kd,r.independent_disturbance_nm})known=known && std::isfinite(x);
        known=known && r.actual_kp>=0 && r.actual_kd>=0 && r.feedback_time_s<=r.command_time_s+.005 &&
            r.command_time_s-r.feedback_time_s<=.06;
        if(r.independent_input_planned_known)known=known && std::isfinite(r.planned_independent_disturbance_nm) &&
            std::abs(r.independent_disturbance_nm-r.planned_independent_disturbance_nm)<=1e-8;
        if(!known){++out.rejected_samples;continue;}
        if(std::abs(r.independent_disturbance_nm)>1e-10) {
            ++out.independent_samples;
            out.independent_peak_nm=std::max(out.independent_peak_nm,std::abs(r.independent_disturbance_nm));
        } else if(r.actual_kp>0) {
            const double pd=r.actual_kp*(r.reference-r.actual)+r.actual_kd*(r.reference_velocity-r.velocity);
            unloaded.push_back({std::max(std::abs(r.reference_velocity),std::abs(r.velocity)),std::abs(pd),
                std::abs(r.reference-r.actual)>position_floor});
        }
    }
    const auto quantile=[](std::vector<double> values,double fraction) {
        if(values.empty())return 0.0;
        const auto k=static_cast<std::size_t>(fraction*(values.size()-1));
        std::nth_element(values.begin(),values.begin()+k,values.end());return values[k];
    };
    std::vector<double> speeds,levels;
    for(const auto& p:unloaded)speeds.push_back(p.speed);
    // "Low speed" is relative to this measured path, not a particular arm's
    // fixed rad/s setting. Zero-speed endpoints remain useful PD evidence.
    out.low_speed_cutoff_rad_s=quantile(speeds,.25);
    for(const auto& p:unloaded)if(p.position_error_resolved && p.speed<=out.low_speed_cutoff_rad_s+1e-12)levels.push_back(p.pd);
    out.low_speed_pd_samples=levels.size();
    if(levels.size()>=8)out.low_speed_pd_level_nm=quantile(levels,.75);
    if(out.independent_samples<8)out.independent_peak_nm=0;
    out.seed_nm=std::max(out.independent_peak_nm,out.low_speed_pd_level_nm);
    out.valid=!source_record_path.empty() && out.seed_nm>0 && std::isfinite(out.seed_nm) &&
        std::isfinite(noise.encoder_step_rad) && noise.encoder_step_rad>=0 && std::isfinite(noise.rms_rad) && noise.rms_rad>=0;
    out.reason=out.valid?"measured_known_input_and_low_speed_PD_scale_prior":"purpose_input_prior_insufficient_known_samples";
    return out;
}

struct PurposePulseOptions {
    double available_s{std::numeric_limits<double>::infinity()},tail_s{1.5};
    double feedback_interval_s{.005},torque_quantum_nm{0},previous_input_peak_nm{0};
    bool zero_stiffness{false};
    int direction{0}; // 0 selects roomier direction; +/-1 requests a checked direction.
    PurposeInputPrior input_prior;
};
struct PurposePulsePlan {
    bool valid{false};int direction{0};
    double amplitude_nm{0},baseline_s{0},rise_s{0},hold_s{0},fall_s{0},release_s{0},duration_s{0};
    double motion_floor_rad{0},target_displacement_rad{0},input_ceiling_nm{0},torque_quantum_nm{0};
    double feedback_interval_s{.005};
    PurposeInputPrior input_prior;
    std::string reason{"purpose_pulse_not_planned"};
    bool timing_shortened{false},prior_input_representable{false};
    double impulse_bound_area_s{0},maximum_sampled_input_ceiling_nm{0};
    bool restoring_domain_bounded{false};
    std::size_t observation_extensions{0};
    std::string supplement_action{"initial_pulse"};
    double opposite_input_ceiling_nm{0};
    bool opposite_direction_valid{false},direction_reversed{false};
    unsigned unresolved_input_direction_mask{0};
    double unresolved_measured_kp{std::numeric_limits<double>::quiet_NaN()};
    double unresolved_measured_kd{std::numeric_limits<double>::quiet_NaN()};
};

inline PurposePulsePlan plan_purpose_pulse(const ModelAxisLimits& bounds,const NoiseEstimate& noise,
    double inertia,double kp,double kd,const PurposePulseOptions& options={}) {
    PurposePulsePlan out;
    for(double x:{inertia,bounds.effort_budget,bounds.max_velocity,bounds.max_acceleration,options.tail_s,options.feedback_interval_s})
        if(!std::isfinite(x) || x<=0){out.reason="purpose_pulse_model_or_capability_missing";return out;}
    for(double x:{kp,kd,noise.encoder_step_rad,noise.rms_rad,options.torque_quantum_nm,options.previous_input_peak_nm})
        if(!std::isfinite(x) || x<0){out.reason="purpose_pulse_invalid_measured_input";return out;}
    if((!options.zero_stiffness && kp<=0) || (options.zero_stiffness && kp!=0)) {
        out.reason="purpose_pulse_stiffness_mode_mismatch";return out;
    }
    if(std::isnan(options.available_s) || options.available_s<=0 ||
       !std::isfinite(bounds.reference_min_offset_rad) || !std::isfinite(bounds.reference_max_offset_rad) ||
       bounds.reference_min_offset_rad>0 || bounds.reference_max_offset_rad<0) {
        out.reason="purpose_pulse_measured_domain_or_budget_missing";return out;
    }
    out.motion_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    const double negative=-bounds.reference_min_offset_rad,positive=bounds.reference_max_offset_rad;
    if(options.direction<-1 || options.direction>1){out.reason="purpose_pulse_invalid_requested_direction";return out;}
    out.direction=options.direction?options.direction:(positive>=negative?1:-1);
    const double room=(out.direction>0?positive:negative)-out.motion_floor_rad;
    const double opposite_room=(out.direction>0?negative:positive)-out.motion_floor_rad;
    if(room<4*out.motion_floor_rad){out.reason="purpose_pulse_demonstrated_room_below_resolution";return out;}
    out.target_displacement_rad=std::min(.25*room,6*out.motion_floor_rad);
    out.torque_quantum_nm=options.torque_quantum_nm;out.feedback_interval_s=options.feedback_interval_s;
    const double model_time=options.zero_stiffness?(kd>0?inertia/kd:0):std::max(std::sqrt(inertia/kp),kd/kp);
    const double move_time=std::max(out.target_displacement_rad/bounds.max_velocity,
        std::sqrt(out.target_displacement_rad/bounds.max_acceleration));
    out.baseline_s=std::max(.25,20*options.feedback_interval_s);
    out.rise_s=std::max({.15,.5*model_time,2*move_time,12*options.feedback_interval_s});out.fall_s=out.rise_s;
    out.hold_s=std::max({.3,4*model_time,30*options.feedback_interval_s});
    out.release_s=out.baseline_s+out.rise_s+out.hold_s+out.fall_s;
    out.duration_s=out.release_s+options.tail_s;
    // Acceleration/effort are existing execution allowances, not a claimed
    // friction identification. A previously measured input is retained even
    // when a softer PD group would otherwise shrink the experiment to zero.
    out.input_ceiling_nm=bounds.effort_budget;
    double desired=kp*out.target_displacement_rad;
    if(!options.zero_stiffness) {
        // This bounds the static restoring displacement tau/P at this
        // measured pose. Reserving half the one-sided room leaves recovery
        // and model-error room; it is neither a gain ceiling nor an inferred
        // actuator/stiction limit. Quantization and old measured inputs must
        // obey the same current pose/PD domain before anything is emitted.
        out.input_ceiling_nm=std::min(out.input_ceiling_nm,.5*kp*room);
        out.restoring_domain_bounded=true;
    }
    if(options.zero_stiffness) {
        double area=out.hold_s+.5*(out.rise_s+out.fall_s);
        const auto desired_input=[&](double input_area) {
            const double release_speed=std::max(2*DragUseRequirements{}.initial_motion_speed_rad_s,
                4*out.motion_floor_rad/std::max(.05,model_time));
            return std::max({kd*out.target_displacement_rad/input_area,
                inertia*out.target_displacement_rad/(input_area*input_area),kd*release_speed,
                inertia*release_speed/(out.rise_s+out.hold_s)});
        };
        double requested=std::max({std::min(desired_input(area),inertia*bounds.max_acceleration),
            options.previous_input_peak_nm,options.torque_quantum_nm});
        if(options.input_prior.valid && std::isfinite(options.input_prior.seed_nm) && options.input_prior.seed_nm>0)
            requested=std::max(requested,options.input_prior.seed_nm);
        if(options.torque_quantum_nm>0)requested=std::ceil(requested/options.torque_quantum_nm-1e-12)*options.torque_quantum_nm;
        if(kd>0 && requested>0) {
            // A long low-force waveform can fit the free-release travel
            // allowance while containing no representable/useful input.
            // Shorten its smooth pulse, never the existing impulse bound.
            const double timing_reserve=2*options.feedback_interval_s;
            const double target_area=std::min({area,std::max(0.0,.5*room*kd/requested-timing_reserve),
                std::max(0.0,.5*bounds.max_velocity*inertia/requested-timing_reserve)});
            if(target_area<area) {
                const double minimum=2*options.feedback_interval_s;
                const double ratio=target_area/area;
                const double sampled_area=std::max(2*minimum,target_area);
                out.rise_s=std::clamp(out.rise_s*ratio,minimum,sampled_area-minimum);out.fall_s=out.rise_s;
                out.hold_s=sampled_area-out.rise_s;
                const double shortened=out.hold_s+.5*(out.rise_s+out.fall_s);
                out.timing_shortened=shortened<area-1e-12;area=shortened;
                out.release_s=out.baseline_s+out.rise_s+out.hold_s+out.fall_s;
                out.duration_s=out.release_s+options.tail_s;
            }
        }
        // Reserve two command intervals beyond the analytic smooth-pulse
        // area for sampled hold/unload timing; measured input is still
        // checked and the caller monitors the demonstrated position domain.
        out.impulse_bound_area_s=area+2*options.feedback_interval_s;
        // M*qdd+D*qd=tau has total released travel impulse/D. No return to
        // the original position is presumed for the zero-stiffness purpose.
        const double travel_cap=kd>0?.5*room*kd/out.impulse_bound_area_s:
            .5*room*inertia/(out.duration_s*out.duration_s);
        out.input_ceiling_nm=std::min({out.input_ceiling_nm,travel_cap,
            kd>0?.5*inertia*bounds.max_velocity/out.impulse_bound_area_s:inertia*bounds.max_velocity/out.duration_s});
        if(kd>0) {
            const double minimum_area=6*options.feedback_interval_s;
            out.maximum_sampled_input_ceiling_nm=std::min({bounds.effort_budget,
                .5*room*kd/minimum_area,.5*inertia*bounds.max_velocity/minimum_area});
        }
        desired=desired_input(area);
    }
    if(!options.zero_stiffness || kd==0)out.maximum_sampled_input_ceiling_nm=out.input_ceiling_nm;
    // A static bias or directional friction can hide a pulse that opposes
    // the existing feedback effort. Preserve the opposite direction's OWN
    // demonstrated room and identical waveform limits for one bounded retry.
    // This is not an amplitude increase or a change during a fixed comparison.
    if(opposite_room>=4*out.motion_floor_rad) {
        out.opposite_input_ceiling_nm=bounds.effort_budget;
        if(!options.zero_stiffness)out.opposite_input_ceiling_nm=std::min(out.opposite_input_ceiling_nm,.5*kp*opposite_room);
        else {
            const double travel_cap=kd>0?.5*opposite_room*kd/out.impulse_bound_area_s:
                .5*opposite_room*inertia/(out.duration_s*out.duration_s);
            const double speed_cap=kd>0?.5*inertia*bounds.max_velocity/out.impulse_bound_area_s:
                inertia*bounds.max_velocity/out.duration_s;
            out.opposite_input_ceiling_nm=std::min({out.opposite_input_ceiling_nm,travel_cap,speed_cap});
        }
        if(options.torque_quantum_nm>0)out.opposite_input_ceiling_nm=
            std::floor(out.opposite_input_ceiling_nm/options.torque_quantum_nm+1e-12)*options.torque_quantum_nm;
        out.opposite_direction_valid=out.opposite_input_ceiling_nm>0;
    }
    // M*a only seeds an unmeasured input. It cannot identify a static force
    // cap or discard an already measured torque that overcame stiction.
    out.input_prior=options.input_prior;
    double amplitude=std::max(std::min(desired,inertia*bounds.max_acceleration),options.previous_input_peak_nm);
    if(options.input_prior.valid && std::isfinite(options.input_prior.seed_nm) && options.input_prior.seed_nm>0)
        amplitude=std::max(amplitude,options.input_prior.seed_nm);
    if(options.torque_quantum_nm>0)amplitude=std::ceil(amplitude/options.torque_quantum_nm)*options.torque_quantum_nm;
    out.amplitude_nm=std::min(amplitude,out.input_ceiling_nm);
    if(options.torque_quantum_nm>0)out.amplitude_nm=std::floor(out.amplitude_nm/options.torque_quantum_nm+1e-12)*options.torque_quantum_nm;
    if(out.amplitude_nm<=0){out.reason="purpose_pulse_no_representable_input";return out;}
    out.prior_input_representable=options.input_prior.valid &&
        out.amplitude_nm+1e-12>=options.input_prior.seed_nm;
    if(out.duration_s>options.available_s){out.reason="purpose_pulse_window_does_not_fit_remaining_budget";return out;}
    out.valid=true;out.reason="single_axis_model_noise_and_measured_room_bounded_pulse";return out;
}

// A free P=0 release may end at another pose. A damping comparison must
// not attribute the changed load/inertia at that pose to its new D code.
inline bool purpose_comparison_pose(const std::vector<double>& baseline,const std::vector<double>& actual,
    const std::vector<NoiseEstimate>& noise) {
    if(baseline.empty() || actual.size()!=baseline.size() || noise.size()!=baseline.size())return false;
    for(std::size_t i=0;i<baseline.size();++i) {
        const double floor=std::max({3*noise[i].encoder_step_rad,6*noise[i].rms_rad,.0002});
        if(!std::isfinite(baseline[i]) || !std::isfinite(actual[i]) || std::abs(actual[i]-baseline[i])>floor)return false;
    }
    return true;
}

struct PurposeConditionComparison {
    bool comparable{false};
    double relative_M_change{std::numeric_limits<double>::quiet_NaN()};
    double max_gravity_change{std::numeric_limits<double>::quiet_NaN()}; // Nm, not a dimensionless ratio.
    std::string reason{"purpose_comparison_model_or_input_missing"};
};

// Experimental condition tolerances, not Safety/PD/physical capability caps.
// A P=0 release need not reproduce its initial encoder code. Compare complete
// mapped inertia and actually executed gravity terms after the bounded return
// instead. Relative generalized inertia detects a weak-axis change even when
// a heavier joint dominates the ordinary matrix norm.
inline PurposeConditionComparison purpose_comparison_condition(const std::vector<double>& baseline_mass,
    const std::vector<double>& current_mass,const std::vector<double>& baseline_gravity,
    const std::vector<double>& current_gravity,const std::vector<double>& pulse_amplitudes_nm,
    const std::vector<double>& torque_quantum_nm) {
    PurposeConditionComparison out;const auto n=baseline_gravity.size();
    if(!n || current_gravity.size()!=n || pulse_amplitudes_nm.size()!=n || torque_quantum_nm.size()!=n ||
       baseline_mass.size()/n!=n || baseline_mass.size()%n || current_mass.size()!=baseline_mass.size())return out;
    Eigen::MatrixXd before(n,n),after(n,n);bool known_input=false,gravity_comparable=true;
    out.max_gravity_change=0;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(baseline_gravity[i]) || !std::isfinite(current_gravity[i]) ||
           !std::isfinite(pulse_amplitudes_nm[i]) || !std::isfinite(torque_quantum_nm[i]) || torque_quantum_nm[i]<0) {
            out.reason="purpose_comparison_gravity_or_input_unknown";return out;
        }
        const double input=std::abs(pulse_amplitudes_nm[i]),difference=std::abs(current_gravity[i]-baseline_gravity[i]);
        known_input=known_input || input>0;out.max_gravity_change=std::max(out.max_gravity_change,difference);
        // One actual mapped protocol code is unresolved command precision;
        // it is not permission to exceed a hardware effort allowance.
        gravity_comparable=gravity_comparable && difference<=.1*input+torque_quantum_nm[i]+1e-12;
        for(std::size_t j=0;j<n;++j) {
            const auto k=i*n+j;
            if(!std::isfinite(baseline_mass[k]) || !std::isfinite(current_mass[k])) {
                out.reason="purpose_comparison_mass_model_unknown";return out;
            }
            before(i,j)=baseline_mass[k];after(i,j)=current_mass[k];
        }
    }
    if(!known_input){out.reason="purpose_comparison_known_input_missing";return out;}
    if(!before.isApprox(before.transpose(),1e-9) || !after.isApprox(after.transpose(),1e-9) ||
       Eigen::LLT<Eigen::MatrixXd>(before).info()!=Eigen::Success ||
       Eigen::LLT<Eigen::MatrixXd>(after).info()!=Eigen::Success) {
        out.reason="purpose_comparison_mass_model_not_positive_definite";return out;
    }
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(after,before,Eigen::EigenvaluesOnly);
    if(modes.info()!=Eigen::Success || !modes.eigenvalues().allFinite() || modes.eigenvalues().minCoeff()<=0) {
        out.reason="purpose_comparison_mass_model_not_identifiable";return out;
    }
    out.relative_M_change=(modes.eigenvalues().array()-1).abs().maxCoeff();
    if(out.relative_M_change>.05+1e-12)out.reason="purpose_comparison_inertia_changed_beyond_experiment_tolerance";
    else if(!gravity_comparable)out.reason="purpose_comparison_executed_gravity_changed_beyond_input_tolerance";
    else {out.comparable=true;out.reason="purpose_comparison_model_and_executed_gravity_comparable";}
    return out;
}

inline double purpose_pulse_input(double time_s,const PurposePulsePlan& plan) {
    if(!plan.valid || !std::isfinite(time_s) || time_s<plan.baseline_s || time_s>=plan.release_s)return 0;
    const auto smooth=[](double u){u=std::clamp(u,0.0,1.0);return u*u*(3-2*u);};
    const double t=time_s-plan.baseline_s;
    double scale=1;
    if(t<plan.rise_s)scale=smooth(t/plan.rise_s);
    else if(time_s>plan.release_s-plan.fall_s)scale=smooth((plan.release_s-time_s)/plan.fall_s);
    return plan.direction*plan.amplitude_nm*scale;
}

struct PurposePulseAssessment {
    CompliantUseAssessment compliant;
    bool input_measurement_complete{false},direct_response_complete{false},yield_observed{false},opposite_response{false},hold_settled{false};
    bool model_identified{false}; // Direct pulse evidence never manufactures poles/inertia/bandwidth.
    double hold_signed_displacement_rad{std::numeric_limits<double>::quiet_NaN()};
    double hold_mean_input_nm{std::numeric_limits<double>::quiet_NaN()};
    double effective_compliance_rad_nm{std::numeric_limits<double>::quiet_NaN()};
    double actual_release_s{std::numeric_limits<double>::quiet_NaN()};
    double hold_position_span_rad{0},hold_speed_rad_s{0};
    bool feedback_clock_aligned{false};
    double actual_plateau_begin_s{std::numeric_limits<double>::quiet_NaN()},actual_plateau_end_s{std::numeric_limits<double>::quiet_NaN()};
    std::size_t baseline_samples{0},hold_samples{0},release_samples{0},duplicate_samples{0};
    std::string reason{"purpose_pulse_not_measured"};
};

inline PurposePulseAssessment assess_purpose_pulse(const std::vector<Sample>& rows,const NoiseEstimate& noise,
    const PurposePulsePlan& plan,const CompliantUseRequirements& req={}) {
    PurposePulseAssessment out;auto& c=out.compliant;c.displacement_limit_rad=req.maximum_displacement_rad;
    c.validation_scope="isolated_known_joint_torque_plateau_yield_and_unloaded_recovery_not_contact_force";
    if(!plan.valid || rows.empty()){out.reason=c.reason="purpose_pulse_plan_or_samples_missing";return out;}
    for(double x:{req.maximum_displacement_rad,req.recovery_error_rad,req.final_drift_rad,req.final_speed_rad_s,req.recovery_time_s,req.tail_s})
        if(!std::isfinite(x) || x<=0)throw std::invalid_argument("invalid purpose-pulse response requirement");
    const auto measured=drag_use_detail::window(rows,0,plan.duration_s+.06);
    const auto aligned=drag_use_detail::feedback_rows(measured.rows);const auto& fresh=aligned.rows;
    out.feedback_clock_aligned=aligned.continuous && !fresh.empty();
    c.fresh_samples=fresh.size();out.duplicate_samples=rows.size()-fresh.size();
    c.motion_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    if(fresh.empty()){out.reason=c.reason="purpose_pulse_fresh_feedback_missing";return out;}
    c.measured_kp=fresh.front().actual_kp;c.measured_kd=fresh.front().actual_kd;
    c.gains_constant=measured.known_gains && measured.constant_gains;
    double baseline=0,reference=0,hold_pos=0,hold_input=0;std::size_t hold_input_samples=0;
    const double base_end=plan.baseline_s-std::min(.02,.1*plan.baseline_s);
    const double hold_begin=plan.baseline_s+plan.rise_s+.5*plan.hold_s;
    const double hold_end=plan.release_s-plan.fall_s;
    double base_min=1e30,base_max=-1e30,hold_min=1e30,hold_max=-1e30;
    bool known=true,timing_valid=true,input_matches=true,positive=true,removed=true,command_reference_fixed=true;
    const double input_tolerance=std::max(1e-7,1.1*plan.torque_quantum_nm+.02*plan.amplitude_nm);
    std::vector<Sample> held,released;
    std::vector<double> baseline_positions,held_positions,tail_errors;
    const auto input_time=[](const Sample& r){return r.reference_sample_time_known?r.reference_sample_time_s:r.command_time_s;};
    // Map waveform sample boundaries to actual command writes. Encoder samples
    // remain on feedback_time_s, including when the write used an earlier
    // control-reference sample. This is commanded-input timing, not an
    // assertion that the motor torque port has no delay.
    const auto written_at=[&](double sample_time) {
        for(const auto& r:measured.rows)if(input_time(r)>=sample_time-1e-10)return r.command_time_s;
        return std::numeric_limits<double>::quiet_NaN();
    };
    const double actual_base_end=written_at(base_end);
    out.actual_plateau_begin_s=written_at(hold_begin);out.actual_plateau_end_s=written_at(hold_end);
    out.actual_release_s=written_at(plan.release_s);
    // Inspect every command, also when no new encoder frame arrived. Dropping
    // repeated feedback is correct for position statistics, but must not hide
    // a clipped/unknown command or a transient change of the actual PD group.
    for(const auto& r:rows)if(r.time_s>=0 && r.time_s<=plan.duration_s+.06) {
        const double sample=input_time(r);
        known=known && r.independent_disturbance_known && std::isfinite(r.independent_disturbance_nm);
        timing_valid=timing_valid && std::isfinite(r.command_time_s) && std::isfinite(sample) &&
            sample<=r.command_time_s+.005 && r.command_time_s-sample<=.06;
        input_matches=input_matches && std::isfinite(sample) &&
            std::abs(r.independent_disturbance_nm-purpose_pulse_input(sample,plan))<=input_tolerance;
        if(r.independent_input_planned_known)input_matches=input_matches &&
            std::isfinite(r.planned_independent_disturbance_nm) &&
            std::abs(r.independent_disturbance_nm-r.planned_independent_disturbance_nm)<=1e-8;
        c.gains_constant=c.gains_constant && r.actual_gains_known &&
            std::isfinite(r.actual_kp) && std::isfinite(r.actual_kd) &&
            std::abs(r.actual_kp-c.measured_kp)<=1e-6 && std::abs(r.actual_kd-c.measured_kd)<=1e-6;
        c.limited=c.limited || r.limited;
        command_reference_fixed=command_reference_fixed && std::isfinite(r.reference) && std::isfinite(r.reference_velocity) &&
            std::abs(r.reference-fresh.front().reference)<=std::max(1e-7,noise.encoder_step_rad) &&
            std::abs(r.reference_velocity)<=1e-8;
        if(sample>=plan.release_s)removed=removed && std::abs(r.independent_disturbance_nm)<=1e-10;
    }
    for(const auto& r:fresh) {
        c.limited=c.limited || r.limited;positive=positive && r.actual_kp>0;
        known=known && r.independent_disturbance_known && std::isfinite(r.independent_disturbance_nm);
        const double command=input_time(r);
        timing_valid=timing_valid && std::isfinite(r.command_time_s) && std::isfinite(r.feedback_time_s) && std::isfinite(command) &&
            r.feedback_time_s<=r.command_time_s+.005 && r.command_time_s-r.feedback_time_s<=.06 &&
            command<=r.command_time_s+.005 && r.command_time_s-command<=.06;
        if(!std::isfinite(r.reference) || !std::isfinite(r.reference_velocity))known=false;
        // The pulse is evaluated at the recorded control-reference sample,
        // while command_written may be later. Unknown sample times fall back
        // only to a known write clock; no delayed position is retimestamped.
        input_matches=input_matches && std::abs(r.independent_disturbance_nm-purpose_pulse_input(command,plan))<=input_tolerance;
        c.input_peak_nm=std::max(c.input_peak_nm,std::abs(r.independent_disturbance_nm));
        if(r.time_s<=actual_base_end) {
            baseline+=r.actual;reference+=r.reference;++out.baseline_samples;
            baseline_positions.push_back(r.actual);
            base_min=std::min(base_min,r.actual);base_max=std::max(base_max,r.actual);
        }
        if(r.time_s>=out.actual_plateau_begin_s && r.time_s<=out.actual_plateau_end_s) {
            hold_pos+=r.actual;++out.hold_samples;held.push_back(r);
            held_positions.push_back(r.actual);
            hold_min=std::min(hold_min,r.actual);hold_max=std::max(hold_max,r.actual);
        }
        if(command>=hold_begin && command<=hold_end){hold_input+=r.independent_disturbance_nm;++hold_input_samples;}
        if(command>=plan.release_s)removed=removed && std::abs(r.independent_disturbance_nm)<=1e-10;
        if(r.time_s>=out.actual_release_s){released.push_back(r);++out.release_samples;}
    }
    c.input_known=known;c.reference_fixed=command_reference_fixed;
    if(out.baseline_samples<3 || out.hold_samples<3 || hold_input_samples<3 || out.release_samples<3){out.reason=c.reason="purpose_pulse_baseline_plateau_or_release_missing";return out;}
    baseline/=out.baseline_samples;reference/=out.baseline_samples;hold_pos/=out.hold_samples;hold_input/=hold_input_samples;
    c.initial_equilibrium_rad=baseline;c.initial_reference_error_rad=baseline-reference;
    out.hold_signed_displacement_rad=plan.direction*(hold_pos-baseline);out.hold_mean_input_nm=hold_input;
    out.hold_position_span_rad=hold_max-hold_min;out.hold_speed_rad_s=drag_position_speed(held,noise,std::max(.05,.4*plan.hold_s));
    out.hold_settled=out.hold_position_span_rad<=std::max(2*c.motion_floor_rad,.15*std::abs(hold_pos-baseline)) &&
        out.hold_speed_rad_s<=std::max(req.final_speed_rad_s,2*c.motion_floor_rad/std::max(.1,.5*plan.hold_s));
    double tail_min=1e30,tail_max=-1e30,last_bad=out.actual_release_s;std::vector<Sample> history;
    const double tail_start=std::max(out.actual_release_s,fresh.back().time_s-.5);
    for(const auto& r:fresh) {
        c.reference_fixed=c.reference_fixed && std::abs(r.reference-reference)<=std::max(1e-7,noise.encoder_step_rad) && std::abs(r.reference_velocity)<=1e-8;
        c.peak_displacement_rad=std::max(c.peak_displacement_rad,std::abs(r.actual-baseline));
        if(r.time_s>=out.actual_release_s) {
            history.push_back(r);if(std::abs(r.actual-baseline)>req.recovery_error_rad || drag_position_speed(history,noise,.25)>req.final_speed_rad_s)last_bad=r.time_s;
        }
        if(r.time_s>=tail_start) {
            tail_errors.push_back(r.actual-r.reference);
            tail_min=std::min(tail_min,r.actual);tail_max=std::max(tail_max,r.actual);
            c.final_equilibrium_error_rad=std::max(c.final_equilibrium_error_rad,std::abs(r.actual-baseline));
            c.final_reference_error_rad=std::max(c.final_reference_error_rad,std::abs(r.actual-r.reference));
        }
    }
    c.final_drift_rad=tail_max-tail_min;c.final_speed_rad_s=drag_position_speed(released,noise,.5);
    compliant_use_detail::mean_response(c,baseline_positions,held_positions,tail_errors,noise);
    c.release_recovery_s=std::max(0.0,last_bad-out.actual_release_s);
    c.release_reversals=tail_position_reversals(released,out.actual_release_s,c.motion_floor_rad);
    c.sustained_oscillation=c.release_reversals>=4 && c.final_drift_rad>c.motion_floor_rad;
    c.stable_recovery=c.final_equilibrium_error_rad<=req.recovery_error_rad && c.final_drift_rad<=req.final_drift_rad &&
        c.final_speed_rad_s<=req.final_speed_rad_s && c.release_recovery_s<=req.recovery_time_s &&
        fresh.back().time_s-last_bad>=.25 && !c.sustained_oscillation;
    const double first=input_time(fresh.front()),last=input_time(fresh.back());
    out.input_measurement_complete=measured.continuous && out.feedback_clock_aligned && c.gains_constant && positive && known && timing_valid && input_matches && removed &&
        c.reference_fixed && !c.limited && base_max-base_min<=c.motion_floor_rad && fresh.size()>=30 &&
        first<=std::max(.03,3*plan.feedback_interval_s) && last>=plan.duration_s-std::max(.02,3*plan.feedback_interval_s) &&
        fresh.back().time_s-out.actual_release_s>=req.tail_s-.02 &&
        std::abs(hold_input)>=.8*plan.amplitude_nm;
    out.direct_response_complete=out.input_measurement_complete && out.hold_settled;
    out.yield_observed=out.hold_signed_displacement_rad>c.motion_floor_rad;
    out.opposite_response=out.hold_signed_displacement_rad < -c.motion_floor_rad;
    if(out.direct_response_complete && out.yield_observed && std::abs(hold_input)>1e-10)
        out.effective_compliance_rad_nm=(hold_pos-baseline)/hold_input;
    c.input_measurement_complete=out.input_measurement_complete;c.yield_observed=out.yield_observed;
    // Directional evidence belongs to this actual complete PD group. A fixed
    // comparison reuses its waveform, not evidence acquired under older gains.
    if(std::isfinite(plan.unresolved_measured_kp) && std::isfinite(plan.unresolved_measured_kd) &&
       std::abs(plan.unresolved_measured_kp-c.measured_kp)<=1e-6 &&
       std::abs(plan.unresolved_measured_kd-c.measured_kd)<=1e-6)
        c.unresolved_input_direction_mask=plan.unresolved_input_direction_mask;
    if(out.direct_response_complete && !out.yield_observed && !out.opposite_response &&
       c.peak_displacement_rad<=c.motion_floor_rad &&
       plan.amplitude_nm>=plan.input_ceiling_nm-std::max(1e-10,plan.torque_quantum_nm))
        c.unresolved_input_direction_mask|=plan.direction>0?2u:1u;
    c.unresolved_both_input_directions_at_ceiling=c.unresolved_input_direction_mask==3;
    c.measurement_complete=out.direct_response_complete && out.yield_observed;
    c.signed_compliance_rad_nm=out.effective_compliance_rad_nm;
    if(!known)out.reason="purpose_pulse_actual_input_unknown";
    else if(!timing_valid)out.reason="purpose_pulse_actual_command_or_feedback_timing_invalid";
    else if(!input_matches)out.reason="purpose_pulse_actual_input_does_not_match_command_plan";
    else if(c.limited)out.reason="purpose_pulse_input_or_response_limited";
    else if(!c.gains_constant || !positive)out.reason="purpose_pulse_requires_known_constant_restoring_gains";
    else if(!c.reference_fixed)out.reason="purpose_pulse_reference_not_fixed";
    else if(!out.input_measurement_complete)out.reason="purpose_pulse_input_measurement_incomplete";
    else if(!out.hold_settled)out.reason="purpose_pulse_plateau_not_settled";
    else if(out.opposite_response)out.reason="purpose_pulse_resolved_response_opposes_known_input";
    else if(!out.yield_observed)out.reason="purpose_pulse_response_below_measured_resolution";
    else if(c.peak_displacement_rad>req.maximum_displacement_rad)out.reason="purpose_pulse_yield_above_declared_local_envelope";
    else if(!c.stable_recovery)out.reason="purpose_pulse_unloaded_recovery_incomplete";
    else {c.passed=true;out.reason="purpose_pulse_resolved_plateau_yield_and_stable_recovery";}
    c.reason=out.reason;return out;
}

// The caller reserves the returned complete window before executing it.
// Resolved but unsettled restoring motion needs more observation at the SAME
// input, not more force. A P=0 impulse cannot be extended this way: its travel
// and speed ceilings depend on the original integrated pulse area.
inline PurposePulsePlan next_purpose_pulse(const PurposePulsePlan& previous,
    const PurposePulseAssessment& measured) {
    auto out=previous;out.valid=false;
    if(!previous.valid || !measured.input_measurement_complete || measured.compliant.limited ||
       measured.compliant.sustained_oscillation) {
        out.reason="purpose_pulse_retry_requires_complete_unlimited_known_input";return out;
    }
    out.unresolved_input_direction_mask=measured.compliant.unresolved_input_direction_mask;
    out.unresolved_measured_kp=measured.compliant.measured_kp;out.unresolved_measured_kd=measured.compliant.measured_kd;
    if(measured.opposite_response) {out.reason="purpose_pulse_resolved_response_opposes_known_input";return out;}
    if(!measured.hold_settled && (measured.yield_observed ||
        measured.compliant.peak_displacement_rad>previous.motion_floor_rad)) {
        if(!previous.restoring_domain_bounded || previous.impulse_bound_area_s>0) {
            out.reason="purpose_pulse_unsettled_observation_requires_restoring_domain_not_free_impulse";return out;
        }
        const double extension=std::max(previous.hold_s,12*previous.feedback_interval_s);
        if(!std::isfinite(extension) || extension<=0 || !std::isfinite(previous.duration_s+extension)) {
            out.reason="purpose_pulse_observation_duration_invalid";return out;
        }
        out.hold_s+=extension;out.release_s+=extension;out.duration_s+=extension;
        ++out.observation_extensions;out.supplement_action="extend_same_input_observation";
        out.valid=true;out.reason="purpose_pulse_resolved_unsettled_extend_same_input_observation";return out;
    }
    if(measured.yield_observed || measured.compliant.peak_displacement_rad>previous.motion_floor_rad) {
        out.reason="purpose_pulse_resolved_response_no_input_increase";return out;
    }
    const double actual=measured.compliant.input_peak_nm;
    if(!std::isfinite(actual) || actual<=0 || !std::isfinite(previous.amplitude_nm) || previous.amplitude_nm<=0 ||
       !std::isfinite(previous.input_ceiling_nm) || previous.input_ceiling_nm<=0 ||
       !std::isfinite(previous.torque_quantum_nm) || previous.torque_quantum_nm<0) {
        out.reason="purpose_pulse_retry_requires_measured_positive_input";return out;
    }
    const double current=std::max(actual,previous.amplitude_nm),step=previous.torque_quantum_nm;
    double proposed=std::max(2*current,current+step);
    if(step>0)proposed=std::ceil(proposed/step-1e-12)*step;
    proposed=std::min(proposed,previous.input_ceiling_nm);
    if(step>0)proposed=std::floor(proposed/step+1e-12)*step;
    if(proposed<=current+1e-12) {
        if((previous.direction==1 || previous.direction==-1) && previous.opposite_direction_valid && !previous.direction_reversed &&
           std::isfinite(previous.opposite_input_ceiling_nm) && previous.opposite_input_ceiling_nm>0) {
            out.direction=-previous.direction;out.direction_reversed=true;
            out.input_ceiling_nm=previous.opposite_input_ceiling_nm;
            out.opposite_input_ceiling_nm=previous.input_ceiling_nm;
            out.opposite_direction_valid=false;
            out.amplitude_nm=std::min(current,out.input_ceiling_nm);
            if(step>0)out.amplitude_nm=std::floor(out.amplitude_nm/step+1e-12)*step;
            if(out.amplitude_nm>0) {
                out.prior_input_representable=out.input_prior.valid && out.amplitude_nm+1e-12>=out.input_prior.seed_nm;
                out.valid=true;out.supplement_action="measure_opposite_legal_direction";
                out.reason="purpose_pulse_unresolved_at_current_ceiling_measure_opposite_direction";return out;
            }
        }
        out.reason="purpose_pulse_current_domain_ceiling_no_resolved_response";return out;
    }
    out.amplitude_nm=proposed;out.valid=true;out.supplement_action="increase_unresolved_input";
    out.reason="measured_unresolved_response_bounded_input_increase";return out;
}

} // namespace serial_arm::servo_tuning
