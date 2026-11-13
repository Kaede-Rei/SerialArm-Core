#pragma once

#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

// Closed-loop excitation -> measured frequency response -> loop-shaping ->
// real comparison is the workflow described publicly by MathWorks' Closed-Loop
// PID Autotuner and Beckhoff TE5960. This implementation is independent of their
// software. Its result covers the sampled host command/receive frequency band,
// not unmeasured mechanical modes or a full-frequency stability certificate.
struct EmpiricalMitFrequencyPoint {
    double hz{0};
    std::complex<double> plant{},physical_plant{},reference_response{},disturbance_response{};
    std::complex<double> feedback_velocity_per_position{},executed_feedforward_per_position{};
    double input_amplitude_nm{0},output_amplitude_rad{0},instrument_amplitude{0};
    double input_noise_floor_nm{0},output_noise_floor_rad{0},coherence{0},half_relative_error{0};
    double loop_input_amplitude_nm{0},loop_input_noise_floor_nm{0},velocity_half_relative_error{0};
};
struct EmpiricalMitFrequencyResponse {
    bool valid{false},tracking_response_valid{false},disturbance_response_valid{false};
    bool isolated_axis_excitation{false};
    ExcitationKind kind{ExcitationKind::ReferenceTracking};
    std::vector<EmpiricalMitFrequencyPoint> points;
    std::size_t fresh_samples{0},duplicate_feedback{0};
    double sample_period_s{0},measured_kp{0},measured_kd{0},executed_state_feedforward_gradient{0};
    std::string reason{"insufficient_frequency_response_evidence"};
    std::vector<std::string> missing_evidence;
    std::string input_definition{"reconstructed_actual_mit_pd_plus_executed_feedforward_and_independent_torque"};
    std::string time_definition{"host_command_time_for_input_and_host_feedback_receive_time_for_output"};
    std::string plant_scope{"isolated_joint_port_with_other_axes_under_fixed_closed_loop_hold_not_free_space_mimo_dynamics"};
    std::string loop_plant_definition{"isolated_fixed_reference_disturbance_Y_divided_by_actual_PD_plus_D_with_executed_FF_inside_controlled_port"};
};

namespace frequency_response_detail {
struct SpectralFit {
    bool valid{false};
    Eigen::MatrixXd coefficients;
    double residual_rms{0},coherence{0};
};
inline SpectralFit fit(const std::vector<const Sample*>& rows,const Requirements& req,int channel) {
    SpectralFit out;if(rows.size()<50)return out;
    Eigen::MatrixXd a(rows.size(),7);Eigen::VectorXd y(rows.size());
    for(std::size_t k=0;k<rows.size();++k) {
        const auto& s=*rows[k];const double t=channel==1 || channel==4?s.feedback_time_s:s.command_time_s;
        a(k,0)=1;for(unsigned j=0;j<3;++j){const double angle=2*pi*req.frequencies_hz[j]*t;a(k,1+2*j)=std::sin(angle);a(k,2+2*j)=std::cos(angle);}
        if(channel==0)y[k]=s.actual_kp*(s.reference-s.actual)+s.actual_kd*(s.reference_velocity-s.velocity)+s.applied_feedforward_nm+s.independent_disturbance_nm;
        else if(channel==1)y[k]=s.actual;
        else if(channel==2)y[k]=s.reference;
        else if(channel==3)y[k]=s.independent_disturbance_nm;
        else if(channel==4)y[k]=s.velocity;
        else if(channel==5)y[k]=s.applied_feedforward_nm;
        else y[k]=s.actual_kp*(s.reference-s.actual)+s.actual_kd*(s.reference_velocity-s.velocity)+s.independent_disturbance_nm;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);if(qr.rank()!=7)return out;
    out.coefficients=qr.solve(y);const double residual=(a*out.coefficients-y).squaredNorm();
    out.residual_rms=std::sqrt(residual/rows.size());const double variance=(y.array()-y.mean()).matrix().squaredNorm();
    out.coherence=variance>1e-24?std::clamp(1-residual/variance,0.0,1.0):0;
    out.valid=out.coefficients.allFinite() && std::isfinite(out.residual_rms);return out;
}
inline std::complex<double> tone(const SpectralFit& fit,unsigned j) {
    return {fit.coefficients(1+2*j,0),fit.coefficients(2+2*j,0)};
}
inline std::vector<double> phases(const EmpiricalMitFrequencyResponse& response) {
    std::vector<double> out;for(const auto& point:response.points) {
        double phase=std::arg(point.plant);
        if(out.empty()){if(phase>pi/2)phase-=2*pi;}
        else {while(phase-out.back()>pi)phase-=2*pi;while(phase-out.back()<-pi)phase+=2*pi;}
        out.push_back(phase);
    }return out;
}
inline std::complex<double> interpolate_plant(const EmpiricalMitFrequencyResponse& response,const std::vector<double>& phase,double hz) {
    std::size_t right=1;while(right+1<response.points.size() && response.points[right].hz<hz)++right;
    const auto& a=response.points[right-1];const auto& b=response.points[right];
    const double u=std::clamp(std::log(hz/a.hz)/std::log(b.hz/a.hz),0.0,1.0);
    return std::polar(std::exp((1-u)*std::log(std::abs(a.plant))+u*std::log(std::abs(b.plant))),
        (1-u)*phase[right-1]+u*phase[right]);
}
inline std::complex<double> interpolate_velocity(const EmpiricalMitFrequencyResponse& response,double hz) {
    std::size_t right=1;while(right+1<response.points.size() && response.points[right].hz<hz)++right;
    const auto& a=response.points[right-1];const auto& b=response.points[right];
    const double u=std::clamp(std::log(hz/a.hz)/std::log(b.hz/a.hz),0.0,1.0);
    return (1-u)*a.feedback_velocity_per_position+u*b.feedback_velocity_per_position;
}
}

inline EmpiricalMitFrequencyResponse estimate_mit_frequency_response(const std::vector<Sample>& rows,const Requirements& req,const MitExperiment& experiment,
    bool isolated_axis_excitation=false,bool reference_feedforward_is_known_position_only=false) {
    using namespace frequency_response_detail;EmpiricalMitFrequencyResponse out;out.kind=experiment.kind;
    const auto reject=[&](const std::string& reason,const std::string& missing){out.reason=reason;out.missing_evidence.push_back(missing);return out;};
    out.isolated_axis_excitation=isolated_axis_excitation;
    if(!isolated_axis_excitation)return reject("frequency_response_axis_excitation_not_isolated","one_active_axis_with_other_axes_under_fixed_position_and_gain_hold");
    if(!std::isfinite(req.amplitude_rad) || req.amplitude_rad<=0 || !std::isfinite(req.encoder_step_rad) || req.encoder_step_rad<0 ||
       !std::isfinite(req.noise_rms_rad) || req.noise_rms_rad<0)
        return reject("invalid_frequency_response_excitation_or_noise","finite_positive_amplitude_and_measured_nonnegative_noise_floor");
    if(req.motion_probe || req.stationary_probe || !std::isfinite(req.probe_duration_s) || req.probe_duration_s<=1.4)
        return reject("frequency_response_requires_multitone_experiment","three_frequency_excitation_not_motion_only_or_static");
    for(unsigned i=0;i<3;++i)if(!std::isfinite(req.frequencies_hz[i]) || req.frequencies_hz[i]<=0 || (i && req.frequencies_hz[i]<=req.frequencies_hz[i-1]))
        return reject("invalid_frequency_response_plan","ordered_positive_excitation_frequencies");
    if((req.probe_duration_s-1.4)*req.frequencies_hz.front()<1.5)
        return reject("frequency_response_observation_too_short","longer_window_for_independent_half_window_comparison");
    if(!std::isfinite(experiment.kp) || experiment.kp<0 || !std::isfinite(experiment.kd) || experiment.kd<0 ||
       !std::isfinite(experiment.executed_state_feedforward_gradient))
        return reject("invalid_frequency_response_controller","finite_nonnegative_actual_mit_gains");
    out.executed_state_feedforward_gradient=experiment.executed_state_feedforward_gradient;
    const bool disturbance=experiment.kind==ExcitationKind::IndependentTorqueDisturbance;
    if(!disturbance && !reference_feedforward_is_known_position_only)
        return reject("reference_frequency_response_feedforward_scope_unknown","explicit_none_or_known_position_only_state_feedforward_for_reference_experiment");
    if(!disturbance)out.loop_plant_definition="reference_port_with_explicit_known_position_only_executed_feedforward_removed_from_physical_denominator";
    std::vector<const Sample*> selected,first,second;std::vector<double> periods;
    std::uint64_t sequence=0;double previous_time=-1,previous_feedback=-1;
    std::optional<double> disturbance_reference;
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || row.time_s<previous_time)return reject("invalid_frequency_response_time","monotonic_host_time");
        previous_time=row.time_s;if(row.time_s<.7 || row.time_s>=req.probe_duration_s-.7)continue;
        if(row.limited)return reject("limited_frequency_response_input","unlimited_real_command_and_feedback_evidence");
        if(!row.actual_gains_known || !std::isfinite(row.actual_kp) || !std::isfinite(row.actual_kd) ||
           std::abs(row.actual_kp-experiment.kp)>1e-6*std::max(1e-9,std::abs(experiment.kp)) ||
           std::abs(row.actual_kd-experiment.kd)>1e-6*std::max(1e-9,std::abs(experiment.kd)))
            return reject("frequency_response_executed_gains_unknown_or_changing","stable_recorded_actual_mit_gains");
        if(!row.feedforward_known || !std::isfinite(row.applied_feedforward_nm) ||
           !row.independent_disturbance_known || !std::isfinite(row.independent_disturbance_nm))
            return reject("frequency_response_actual_effort_components_unknown","executed_feedforward_and_independent_torque_columns");
        if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual) || !std::isfinite(row.velocity))
            return reject("nonfinite_frequency_response_feedback","finite_actual_reference_and_feedback");
        if(disturbance) {
            if(!disturbance_reference)disturbance_reference=row.reference;
            if(std::abs(row.reference-*disturbance_reference)>1e-8 || std::abs(row.reference_velocity)>1e-8)
                return reject("independent_disturbance_reference_not_fixed","constant_actual_reference_position_and_zero_reference_velocity_for_independent_torque");
        }
        if(!row.new_measurement)continue;
        if(row.feedback_sequence && row.feedback_sequence==sequence){++out.duplicate_feedback;continue;}
        if(row.feedback_sequence && sequence && row.feedback_sequence<sequence)
            return reject("frequency_response_feedback_sequence_reversed","monotonic_fresh_feedback_sequence");
        if(row.feedback_sequence)sequence=row.feedback_sequence;
        if(!std::isfinite(row.command_time_s) || !std::isfinite(row.feedback_time_s) || row.command_time_s>row.time_s+.001 ||
           row.feedback_time_s>row.time_s+.001 || std::abs(row.command_time_s-row.time_s)>.06 ||
           row.time_s-row.feedback_time_s>.06 || (previous_feedback>=0 && (row.feedback_time_s<=previous_feedback || row.feedback_time_s-previous_feedback>.06)))
            return reject("frequency_response_clock_or_feedback_gap","causal_command_feedback_times_and_continuous_fresh_samples");
        if(previous_feedback>=0)periods.push_back(row.feedback_time_s-previous_feedback);
        previous_feedback=row.feedback_time_s;
        selected.push_back(&row);(row.time_s<.5*req.probe_duration_s?first:second).push_back(&row);
    }
    out.fresh_samples=selected.size();if(selected.size()<150 || first.size()<60 || second.size()<60)
        return reject("insufficient_frequency_response_feedback","fresh_complete_window_and_two_independent_half_windows");
    if(selected.front()->time_s>.76 || selected.back()->time_s<req.probe_duration_s-.76)
        return reject("frequency_response_window_incomplete","actual_feedback_covering_the_planned_observation_window");
    std::sort(periods.begin(),periods.end());out.sample_period_s=periods[periods.size()/2];out.measured_kp=experiment.kp;out.measured_kd=experiment.kd;
    if(req.frequencies_hz.back()>=.4/out.sample_period_s)return reject("frequency_response_above_usable_sample_band","lower_excitation_frequency_or_faster_actual_feedback");
    std::array<SpectralFit,7> full,a,b;
    for(unsigned channel=0;channel<7;++channel){full[channel]=fit(selected,req,channel);a[channel]=fit(first,req,channel);b[channel]=fit(second,req,channel);
        if(!full[channel].valid || !a[channel].valid || !b[channel].valid)return reject("frequency_response_fit_rank_or_time_coverage_insufficient","separable_excitation_and_longer_half_windows");}
    if(full[1].coherence<.60 || a[1].coherence<.50 || b[1].coherence<.50)
        return reject("frequency_response_output_fit_quality_insufficient","resolved_repeatable_position_response_at_excitation_frequencies");
    for(unsigned j=0;j<3;++j) {
        EmpiricalMitFrequencyPoint point;point.hz=req.frequencies_hz[j];
        const auto U=tone(full[0],j),Y=tone(full[1],j),R=tone(full[2],j),D=tone(full[3],j),V=tone(full[4],j),F=tone(full[5],j),E=tone(full[6],j);
        point.input_amplitude_nm=std::abs(U);point.output_amplitude_rad=std::abs(Y);point.instrument_amplitude=std::abs(disturbance?D:R);
        point.input_noise_floor_nm=std::max(1e-6,6*full[0].residual_rms/std::sqrt(static_cast<double>(selected.size())));
        point.output_noise_floor_rad=std::max({1e-7,2*req.encoder_step_rad,3*req.noise_rms_rad,6*full[1].residual_rms/std::sqrt(static_cast<double>(selected.size()))});
        point.coherence=std::min({full[1].coherence,a[1].coherence,b[1].coherence});
        if(point.instrument_amplitude<(disturbance?1e-6:std::max(1e-6,.025*req.amplitude_rad)))
            return reject("frequency_response_instrument_missing_at_tone","known_reference_or_independent_torque_at_every_requested_frequency");
        if(point.input_amplitude_nm<=point.input_noise_floor_nm)
            return reject("frequency_response_net_effort_cancels_into_noise","larger_distinguishable_actual_total_effort_at_requested_frequencies");
        if(point.output_amplitude_rad<=point.output_noise_floor_rad)
            return reject("frequency_response_position_below_noise_floor","larger_safe_resolved_position_response");
        point.physical_plant=Y/U;
        point.executed_feedforward_per_position=disturbance?F/Y:std::complex<double>{experiment.executed_state_feedforward_gradient,0};
        if(!disturbance && std::abs(F-experiment.executed_state_feedforward_gradient*Y)>
           std::max(6*full[5].residual_rms/std::sqrt(static_cast<double>(selected.size()))+1e-6,.20*std::abs(experiment.executed_state_feedforward_gradient*Y)))
            return reject("reference_feedforward_exceeds_declared_position_only_scope","fixed_reference_independent_disturbance_for_dynamic_or_external_feedforward");
        const auto loop_input=disturbance?E:U-experiment.executed_state_feedforward_gradient*Y;
        point.loop_input_amplitude_nm=std::abs(loop_input);
        point.loop_input_noise_floor_nm=std::max(1e-6,6*full[disturbance?6:0].residual_rms/std::sqrt(static_cast<double>(selected.size())));
        if(point.loop_input_amplitude_nm<=point.loop_input_noise_floor_nm)
            return reject("frequency_response_effective_loop_input_cancels_into_noise","resolved_actual_PD_plus_independent_torque_for_the_executed_strategy");
        point.plant=Y/loop_input;
        const double velocity_floor=std::max(1e-6,6*full[4].residual_rms/std::sqrt(static_cast<double>(selected.size())));
        if(std::abs(V)<=velocity_floor)return reject("frequency_response_velocity_feedback_unresolved","measured_velocity_filter_response_at_each_requested_frequency");
        point.feedback_velocity_per_position=V/Y;
        if(std::abs(R)>1e-8)point.reference_response=Y/R;
        if(std::abs(D)>1e-8)point.disturbance_response=Y/D;
        const auto Ua=tone(a[0],j),Ub=tone(b[0],j),Ya=tone(a[1],j),Yb=tone(b[1],j);
        if(std::abs(Ua)<=point.input_noise_floor_nm || std::abs(Ub)<=point.input_noise_floor_nm ||
           std::abs(Ya)<=.5*point.output_noise_floor_rad || std::abs(Yb)<=.5*point.output_noise_floor_rad)
            return reject("frequency_response_half_window_unresolved","distinguishable_input_and_output_in_each_half_window");
        const auto Ea=disturbance?tone(a[6],j):Ua-experiment.executed_state_feedforward_gradient*Ya;
        const auto Eb=disturbance?tone(b[6],j):Ub-experiment.executed_state_feedforward_gradient*Yb;
        if(std::abs(Ea)<=point.loop_input_noise_floor_nm || std::abs(Eb)<=point.loop_input_noise_floor_nm)
            return reject("executed_strategy_half_window_input_unresolved","repeatable_local_controlled_port_response");
        point.half_relative_error=std::abs(Ya/Ea-Yb/Eb)/std::max(1e-12,std::abs(point.plant));
        point.velocity_half_relative_error=std::abs(tone(a[4],j)/Ya-tone(b[4],j)/Yb)/std::max(1e-12,std::abs(point.feedback_velocity_per_position));
        if(!std::isfinite(point.half_relative_error) || point.half_relative_error>.35)
            return reject("frequency_response_half_windows_inconsistent","repeatable_operating_condition_and_longer_or_additional_measurement");
        if(!std::isfinite(point.velocity_half_relative_error) || point.velocity_half_relative_error>.35)
            return reject("frequency_response_velocity_half_windows_inconsistent","repeatable_measured_velocity_feedback_response");
        if(disturbance && (std::abs(R)>std::max(1e-6,.025*req.amplitude_rad)))
            return reject("independent_disturbance_reference_not_fixed","fixed_reference_for_disturbance_compliance_evidence");
        out.points.push_back(point);
    }
    out.valid=true;out.tracking_response_valid=!disturbance;out.disturbance_response_valid=disturbance;
    out.reason="empirical_mit_plant_measured_in_sampled_host_frequency_band";return out;
}

struct SampledLoopPoint {double hz{0},magnitude{0},phase_deg{0};};
struct SampledLoopCrossover {double hz{0},phase_margin_deg{0};bool descending{false};};
struct SampledPdLoopReport {
    bool valid{false},crossover_covered{false};std::vector<SampledLoopPoint> points;std::vector<SampledLoopCrossover> crossovers;
    double crossover_hz{0},phase_margin_deg{0};std::string reason{"insufficient_empirical_plant"};
    std::vector<std::string> missing_evidence;
    std::string scope{"measured_frequency_band_only_not_full_frequency_stability_certification"};
};
inline SampledPdLoopReport evaluate_mit_pd_loop(double kp,double kd,const EmpiricalMitFrequencyResponse& response) {
    using namespace frequency_response_detail;SampledPdLoopReport out;
    if(!response.valid || !response.isolated_axis_excitation || response.points.size()<3 || !std::isfinite(kp) || kp<=0 || !std::isfinite(kd) || kd<0)return out;
    const auto phase=phases(response);
    for(std::size_t k=0;k<response.points.size();++k) {
        const auto& p=response.points[k];if(p.hz<=0 || std::abs(p.plant)<=0 || !std::isfinite(std::abs(p.plant)) || (k && p.hz<=response.points[k-1].hz))return out;
        const auto c=kp+kd*p.feedback_velocity_per_position;
        const double magnitude=std::abs(c*p.plant);
        if(!std::isfinite(magnitude) || magnitude<=1e-12){out.reason="sampled_controller_cancellation_has_unresolved_phase";return out;}
        out.points.push_back({p.hz,magnitude,(phase[k]+std::arg(c))*180/pi});
    }
    out.valid=true;
    for(std::size_t k=1;k<out.points.size();++k) {
        const auto& a=out.points[k-1];const auto& b=out.points[k];const double la=std::log(a.magnitude),lb=std::log(b.magnitude);
        if((la>0 && lb<=0) || (la<0 && lb>=0)) {
            const double u=la/(la-lb);const double hz=std::exp(std::log(a.hz)+u*std::log(b.hz/a.hz));
            out.crossovers.push_back({hz,180+(1-u)*a.phase_deg+u*b.phase_deg,la>lb});
        }
    }
    if(out.crossovers.size()!=1){out.reason=out.crossovers.empty()?"unity_gain_crossover_not_bracketed":"multiple_sampled_unity_crossovers_need_more_evidence";
        out.missing_evidence.push_back(out.points.front().magnitude<=1?"lower_frequency_loop_evidence":"higher_or_denser_frequency_loop_evidence");return out;}
    const auto& cross=out.crossovers.front();out.crossover_hz=cross.hz;out.phase_margin_deg=cross.phase_margin_deg;
    out.crossover_covered=cross.descending && out.points.front().magnitude>1 && out.points.back().magnitude<1;
    out.reason=out.crossover_covered?"single_descending_crossover_bracketed_in_measured_band":"loop_band_edges_do_not_cover_crossover";
    if(!out.crossover_covered)out.missing_evidence.push_back("lower_and_higher_frequency_evidence_around_intended_crossover");
    return out;
}

struct PdLoopShapingRequest {
    double target_crossover_hz{0},min_phase_margin_deg{50},target_phase_margin_deg{60},max_phase_margin_deg{85};
    double error_bound_rad{0},velocity_error_bound_rad{0};
};
struct SampledPdGainProposal {
    double kp{0},kd{0};bool valid{false},changed{false};std::string reason{"insufficient_empirical_plant"};
    std::vector<std::string> missing_evidence;SampledPdLoopReport measured_loop,candidate_loop;
    double design_target_crossover_hz{0},design_target_phase_margin_deg{0};
};
inline SampledPdGainProposal shape_mit_pd(double kp,double kd,const EmpiricalMitFrequencyResponse& response,
    const ModelAxisLimits& limits,const PdLoopShapingRequest& request) {
    using namespace frequency_response_detail;SampledPdGainProposal out;out.kp=kp;out.kd=kd;
    out.design_target_crossover_hz=request.target_crossover_hz;out.design_target_phase_margin_deg=request.target_phase_margin_deg;
    out.measured_loop=evaluate_mit_pd_loop(kp,kd,response);
    if(!response.valid || !response.isolated_axis_excitation || response.points.size()<3){out.missing_evidence=response.missing_evidence;return out;}
    if(!std::isfinite(request.target_crossover_hz) || request.target_crossover_hz<=0 ||
       !std::isfinite(request.min_phase_margin_deg) || !std::isfinite(request.target_phase_margin_deg) || !std::isfinite(request.max_phase_margin_deg) ||
       request.min_phase_margin_deg<=0 || request.min_phase_margin_deg>request.target_phase_margin_deg || request.target_phase_margin_deg>request.max_phase_margin_deg || request.max_phase_margin_deg>=180)
        throw std::invalid_argument("invalid sampled PD loop design target");
    validate_model_limits(limits);(void)gain_working_domain(limits,request.error_bound_rad,request.velocity_error_bound_rad);
    if(request.target_crossover_hz<=response.points.front().hz || request.target_crossover_hz>=response.points.back().hz) {
        out.reason="requested_crossover_outside_measured_frequency_band";
        out.missing_evidence.push_back(request.target_crossover_hz<=response.points.front().hz?"lower_frequency_plant_evidence":"higher_frequency_plant_evidence");return out;
    }
    const auto phase=phases(response);double best=std::numeric_limits<double>::infinity();
    // Finite numerical loop shaping, not physical search. Solve
    // C_target=Kp+Kd*(measured V/Y), then project protocol/effort limits and
    // recheck every measured loop point. No ideal velocity estimator is assumed.
    for(unsigned f=0;f<=80;++f) {
        const double hz=std::exp(std::log(response.points.front().hz)+(.01+.98*f/80)*std::log(response.points.back().hz/response.points.front().hz));
        const auto plant=interpolate_plant(response,phase,hz);
        std::size_t right=1;while(right+1<response.points.size() && response.points[right].hz<hz)++right;
        const double u=std::log(hz/response.points[right-1].hz)/std::log(response.points[right].hz/response.points[right-1].hz);
        const double plant_phase=(1-u)*phase[right-1]+u*phase[right];
        const auto velocity=interpolate_velocity(response,hz);
        for(unsigned m=0;m<=13;++m) {
            const double margin=request.min_phase_margin_deg+(request.max_phase_margin_deg-request.min_phase_margin_deg)*m/12;
            const double lead=m==13?0:(margin-180)*pi/180-plant_phase;if(lead<0 || lead>=.5*pi)continue;
            const double magnitude=1/std::abs(plant);
            const auto desired=std::polar(magnitude,lead);
            if(std::abs(velocity.imag())<1e-8 && std::abs(desired.imag())>1e-8)continue;
            const double proposed_d=std::abs(desired.imag())<=1e-8?0:desired.imag()/velocity.imag();
            const double proposed_p=desired.real()-proposed_d*velocity.real();
            if(proposed_d<0 || proposed_p<=0)continue;
            const auto projected=constrain_candidate(proposed_p,proposed_d,limits,
                request.error_bound_rad,request.velocity_error_bound_rad);
            if(projected.first<=0)continue;
            const auto loop=evaluate_mit_pd_loop(projected.first,projected.second,response);
            if(!loop.valid || !loop.crossover_covered || loop.phase_margin_deg<request.min_phase_margin_deg || loop.phase_margin_deg>request.max_phase_margin_deg)continue;
            const double score=std::abs(std::log(loop.crossover_hz/request.target_crossover_hz))+
                .01*std::abs(loop.phase_margin_deg-request.target_phase_margin_deg);
            if(score<best){best=score;out.kp=projected.first;out.kd=projected.second;out.candidate_loop=loop;out.valid=true;}
        }
    }
    if(!out.valid){out.reason="no_pd_design_meets_sampled_margin_after_protocol_and_effort_constraints";
        out.missing_evidence.push_back("additional_frequency_evidence_or_more_legal_feedback_effort_for_requested_design");return out;}
    out.changed=out.kp!=kp || out.kd!=kd;out.reason=out.changed?"empirical_sampled_loop_shaping_candidate_requires_real_comparison":"current_pd_meets_sampled_loop_design";
    return out;
}

} // namespace serial_arm::servo_tuning
