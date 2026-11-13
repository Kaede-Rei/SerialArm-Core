#pragma once

#include "serial_arm/dynamics/frequency_response_gain_tuning.hpp"
#include "serial_arm/dynamics/gain_frequency_plan.hpp"
#include <array>
#include <string>

namespace serial_arm::servo_tuning {

// The independent command-to-returned-position channel is measured directly.
// No host reconstruction of the embedded MIT position/velocity feedback is
// used. Other controllers, reference, load and executed feedforward must remain
// fixed while the active axis's actual protocol gains are varied.
struct MitAutotuneOperatingPoint {
    Gains actual_gains;
    std::vector<double> fixed_reference_rad;
    std::size_t excited_axis{0};
    std::string condition_id;
    bool isolated_axis_excitation{false};
    double maximum_operating_excursion_rad{.04};
};
struct MitClosedLoopToneEstimate {
    double hz{0};
    std::complex<double> closed_loop_response{},input_tone_nm{},output_tone_rad{};
    std::complex<double> first_half_response{},second_half_response{};
    double input_dc_nm{0},input_noise_floor_nm{0},output_noise_floor_rad{0};
    double input_tone_fit_r_squared{0},output_tone_fit_r_squared{0},half_relative_error{0};
};
struct MitClosedLoopTrialEstimate {
    bool valid{false},unloaded_baseline_measured{false};
    MitAutotuneOperatingPoint operating_point;
    std::vector<MitClosedLoopToneEstimate> points;
    std::size_t fresh_samples{0},duplicate_feedback{0};
    double sample_period_s{0},input_start_s{.25},probe_duration_s{0};
    double encoder_step_rad{0},noise_rms_rad{0};
    double unloaded_mean_position_rad{0},unloaded_span_rad{0},maximum_excursion_rad{0};
    std::string reason{"independent_closed_loop_response_not_measured"};
    std::vector<std::string> missing_evidence;
    std::string input_definition{"known_independent_commanded_joint_torque_not_reconstructed_embedded_PD"};
    std::string time_definition{"input_host_command_write_and_output_host_feedback_receive_times_relative_to_known_input_start"};
};
struct MitClosedLoopIdentificationPoint {
    double hz{0},design_condition_number{0};
    std::complex<double> a{},b{},c{},g_eff{},internal_velocity{};
    double half_plant_relative_error{0},half_velocity_relative_error{0},gain_holdout_relative_error{0};
};
struct MitClosedLoopIdentification {
    bool valid{false},gain_holdout_verified{false};
    EmpiricalMitFrequencyResponse response;
    std::vector<MitClosedLoopIdentificationPoint> points;
    std::string reason{"insufficient_independent_controller_experiments"};
    std::vector<std::string> missing_evidence;
    std::string model_definition{"inverse_command_to_position_response_Z_equals_a_plus_b_times_actual_P_plus_c_times_actual_D"};
    std::string plant_definition{"G_eff_equals_b_over_a_with_fixed_state_feedforward_inside_a_not_pure_mechanical_plant"};
    std::string velocity_definition{"relative_internal_velocity_channel_equals_c_over_b_not_reported_host_velocity"};
};

namespace mit_closed_loop_detail {
inline bool finite(std::complex<double> value) {
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}
inline double relative(std::complex<double> first,std::complex<double> second,double floor=1e-12) {
    return std::abs(first-second)/std::max({std::abs(first),std::abs(second),floor});
}
struct ToneFit {
    bool valid{false};
    Eigen::VectorXd coefficients;
    double residual_rms{0},r_squared{0};
};
inline ToneFit fit(const std::vector<const Sample*>& rows,const Requirements& req,
    bool output,double input_start_s) {
    ToneFit out;if(rows.size()<50)return out;
    Eigen::MatrixXd design(rows.size(),7);Eigen::VectorXd values(rows.size());
    for(std::size_t k=0;k<rows.size();++k) {
        const auto& row=*rows[k];
        const double time=(output?row.feedback_time_s:row.command_time_s)-input_start_s;
        design(k,0)=1;
        for(unsigned tone=0;tone<3;++tone) {
            const double angle=2*pi*req.frequencies_hz[tone]*time;
            design(k,1+2*tone)=std::sin(angle);design(k,2+2*tone)=std::cos(angle);
        }
        values[k]=output?row.actual:row.independent_disturbance_nm;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    if(qr.rank()!=7)return out;
    out.coefficients=qr.solve(values);
    const double residual=(design*out.coefficients-values).squaredNorm();
    const double variance=(values.array()-values.mean()).matrix().squaredNorm();
    out.residual_rms=std::sqrt(residual/static_cast<double>(rows.size()));
    out.r_squared=variance>1e-24?std::clamp(1-residual/variance,0.0,1.0):0;
    out.valid=out.coefficients.allFinite() && std::isfinite(out.residual_rms);
    return out;
}
inline std::complex<double> tone(const ToneFit& fit,unsigned index) {
    return {fit.coefficients(1+2*index),fit.coefficients(2+2*index)};
}
inline bool close(double first,double second,double tolerance=1e-8) {
    return std::isfinite(first) && std::isfinite(second) &&
        std::abs(first-second)<=tolerance*std::max({1.0,std::abs(first),std::abs(second)});
}
inline bool operating_point_valid(const MitAutotuneOperatingPoint& point) {
    const auto n=point.actual_gains.kp.size();
    if(!point.isolated_axis_excitation || point.condition_id.empty() || n==0 ||
       point.actual_gains.kd.size()!=n || point.fixed_reference_rad.size()!=n || point.excited_axis>=n ||
       !std::isfinite(point.maximum_operating_excursion_rad) || point.maximum_operating_excursion_rad<=0)return false;
    for(std::size_t i=0;i<n;++i)
        if(!std::isfinite(point.actual_gains.kp[i]) || point.actual_gains.kp[i]<0 ||
           !std::isfinite(point.actual_gains.kd[i]) || point.actual_gains.kd[i]<0 ||
           !std::isfinite(point.fixed_reference_rad[i]))return false;
    return point.actual_gains.kp[point.excited_axis]>0;
}
}

inline MitClosedLoopTrialEstimate estimate_mit_closed_loop_trial(const std::vector<Sample>& rows,
    const Requirements& req,const MitAutotuneOperatingPoint& operating_point,double input_start_s=.25) {
    using namespace mit_closed_loop_detail;
    MitClosedLoopTrialEstimate out;out.operating_point=operating_point;
    out.input_start_s=input_start_s;out.probe_duration_s=req.probe_duration_s;
    out.encoder_step_rad=req.encoder_step_rad;out.noise_rms_rad=req.noise_rms_rad;
    const auto reject=[&](const std::string& reason,const std::string& missing) {
        out.reason=reason;out.missing_evidence.push_back(missing);return out;
    };
    if(!operating_point_valid(operating_point))
        return reject("invalid_or_nonisolated_mit_operating_point","mapped_actual_full_controller_fixed_reference_and_common_condition_id");
    if(!std::isfinite(input_start_s) || input_start_s<0 || !std::isfinite(req.probe_duration_s) || req.probe_duration_s<=1.4 ||
       !std::isfinite(req.encoder_step_rad) || req.encoder_step_rad<0 || !std::isfinite(req.noise_rms_rad) || req.noise_rms_rad<0 ||
       req.motion_probe || req.stationary_probe)
        return reject("invalid_mit_independent_frequency_experiment","finite_three_tone_experiment_duration_and_measured_position_noise");
    for(unsigned i=0;i<3;++i)
        if(!std::isfinite(req.frequencies_hz[i]) || req.frequencies_hz[i]<=0 || (i && req.frequencies_hz[i]<=req.frequencies_hz[i-1]))
            return reject("invalid_mit_independent_tones","three_ordered_positive_frequencies");
    if((req.probe_duration_s-1.4)*req.frequencies_hz.front()<1.5)
        return reject("mit_independent_observation_too_short","at_least_one_and_half_low_frequency_cycles_after_smooth_edges");
    const auto axis=operating_point.excited_axis;
    const double kp=operating_point.actual_gains.kp[axis],kd=operating_point.actual_gains.kd[axis];
    const double motion_floor=std::max({3*req.encoder_step_rad,6*req.noise_rms_rad,.0002});
    std::vector<const Sample*> selected,first,second,baseline;
    std::vector<double> periods;
    std::uint64_t sequence=0;double previous_time=-std::numeric_limits<double>::infinity();
    double previous_command=-std::numeric_limits<double>::infinity(),previous_feedback=-std::numeric_limits<double>::infinity();
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || row.time_s<previous_time)
            return reject("mit_independent_time_reversed","monotonic_record_time");
        previous_time=row.time_s;
        if(!std::isfinite(row.command_time_s) || !std::isfinite(row.feedback_time_s))
            return reject("mit_independent_clock_unknown","actual_command_write_and_feedback_receive_timestamps");
        if(row.command_time_s<previous_command-1e-9)
            return reject("mit_independent_command_time_reversed","monotonic_command_write_time");
        previous_command=row.command_time_s;
        if(row.command_time_s>row.time_s+.001 || std::abs(row.command_time_s-row.time_s)>.06 ||
           row.feedback_time_s>row.time_s+.001 || row.time_s-row.feedback_time_s>.06)
            return reject("mit_independent_clock_or_feedback_age_invalid","causal_host_times_and_fresh_feedback_within_recorded_pipeline");
        if(!row.new_measurement)continue;
        if(row.feedback_sequence && row.feedback_sequence==sequence){++out.duplicate_feedback;continue;}
        if((row.feedback_sequence && sequence && row.feedback_sequence<sequence) || row.feedback_time_s<=previous_feedback)
            return reject("mit_independent_feedback_reversed","ordered_distinct_feedback_receive_times");
        if(row.feedback_sequence)sequence=row.feedback_sequence;
        const bool in_window=row.command_time_s-input_start_s>=.7 && row.command_time_s-input_start_s<req.probe_duration_s-.7 &&
            row.feedback_time_s-input_start_s>=.7 && row.feedback_time_s-input_start_s<req.probe_duration_s-.7;
        if(in_window && std::isfinite(previous_feedback) && row.feedback_time_s-previous_feedback>.06)
            return reject("mit_independent_feedback_gap","continuous_fresh_position_samples_in_observation_window");
        if(in_window && std::isfinite(previous_feedback))periods.push_back(row.feedback_time_s-previous_feedback);
        previous_feedback=row.feedback_time_s;
        if(!std::isfinite(row.actual))return reject("mit_independent_position_nonfinite","finite_raw_position_feedback");
        if(input_start_s>0 && row.command_time_s>=0 && row.feedback_time_s>=0 && row.command_time_s<input_start_s-.02) {
            if(!row.independent_disturbance_known || !std::isfinite(row.independent_disturbance_nm) ||
               std::abs(row.independent_disturbance_nm)>1e-9)
                return reject("mit_independent_unloaded_baseline_input_unknown","measured_zero_independent_torque_before_input_start");
            baseline.push_back(&row);
        }
        if(!in_window)continue;
        if(row.limited)return reject("mit_independent_command_limited","unlimited_actual_input_and_position_evidence");
        if(!row.actual_gains_known || !close(row.actual_kp,kp,1e-6) || !close(row.actual_kd,kd,1e-6))
            return reject("mit_independent_actual_gains_unknown_or_changing","one_constant_actual_protocol_PD_group_per_experiment");
        if(!row.independent_disturbance_known || !std::isfinite(row.independent_disturbance_nm))
            return reject("mit_independent_input_unknown","known_independent_commanded_torque_not_host_reconstructed_PD");
        if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) ||
           !close(row.reference,operating_point.fixed_reference_rad[axis]) || std::abs(row.reference_velocity)>1e-8)
            return reject("mit_independent_reference_not_fixed","same_fixed_reference_and_zero_reference_velocity");
        selected.push_back(&row);
        (row.command_time_s-input_start_s<.5*req.probe_duration_s?first:second).push_back(&row);
    }
    out.fresh_samples=selected.size();
    if(selected.size()<150 || first.size()<60 || second.size()<60 || periods.empty())
        return reject("mit_independent_feedback_insufficient","full_fresh_observation_and_two_independent_half_windows");
    if(selected.front()->command_time_s-input_start_s>.76 || selected.back()->command_time_s-input_start_s<req.probe_duration_s-.76 ||
       selected.front()->feedback_time_s-input_start_s>.76 || selected.back()->feedback_time_s-input_start_s<req.probe_duration_s-.76)
        return reject("mit_independent_observation_incomplete","feedback_covering_the_planned_three_tone_window");
    if(input_start_s>0) {
        if(baseline.size()<10)return reject("mit_independent_unloaded_baseline_insufficient","fresh_unloaded_position_samples_before_known_input_start");
        double low=baseline.front()->actual,high=low;
        for(const auto* row:baseline) {out.unloaded_mean_position_rad+=row->actual;low=std::min(low,row->actual);high=std::max(high,row->actual);}
        out.unloaded_mean_position_rad/=baseline.size();out.unloaded_span_rad=high-low;
        if(out.unloaded_span_rad>motion_floor)
            return reject("mit_independent_unloaded_baseline_not_quiet","quiet_initial_operating_point_before_frequency_input");
        out.unloaded_baseline_measured=true;
    } else {
        for(const auto* row:selected)out.unloaded_mean_position_rad+=row->actual;
        out.unloaded_mean_position_rad/=selected.size();
    }
    for(const auto* row:selected)out.maximum_excursion_rad=std::max(out.maximum_excursion_rad,std::abs(row->actual-out.unloaded_mean_position_rad));
    if(out.maximum_excursion_rad>operating_point.maximum_operating_excursion_rad)
        return reject("mit_independent_local_operating_domain_exceeded","position_response_inside_declared_local_model_range");
    std::sort(periods.begin(),periods.end());out.sample_period_s=periods[periods.size()/2];
    if(req.frequencies_hz.back()>=.4/out.sample_period_s)
        return reject("mit_independent_frequency_above_sample_band","resolved_input_band_below_actual_feedback_sampling_limit");
    const auto d=fit(selected,req,false,input_start_s),y=fit(selected,req,true,input_start_s);
    const auto da=fit(first,req,false,input_start_s),ya=fit(first,req,true,input_start_s);
    const auto db=fit(second,req,false,input_start_s),yb=fit(second,req,true,input_start_s);
    if(!d.valid || !y.valid || !da.valid || !ya.valid || !db.valid || !yb.valid)
        return reject("mit_independent_tone_fit_rank_insufficient","separable_three_tone_input_and_position_fit_in_each_half");
    if(d.r_squared<.8 || da.r_squared<.7 || db.r_squared<.7 || y.r_squared<.6 || ya.r_squared<.5 || yb.r_squared<.5)
        return reject("mit_independent_tone_fit_quality_insufficient","repeatable_three_tone_position_and_known_input_R_squared");
    for(unsigned j=0;j<3;++j) {
        MitClosedLoopToneEstimate p;p.hz=req.frequencies_hz[j];
        p.input_tone_nm=tone(d,j);p.output_tone_rad=tone(y,j);p.input_dc_nm=d.coefficients(0);
        p.input_tone_fit_r_squared=std::min({d.r_squared,da.r_squared,db.r_squared});
        p.output_tone_fit_r_squared=std::min({y.r_squared,ya.r_squared,yb.r_squared});
        p.input_noise_floor_nm=std::max(1e-6,6*d.residual_rms/std::sqrt(static_cast<double>(selected.size())));
        p.output_noise_floor_rad=std::max({1e-7,2*req.encoder_step_rad,3*req.noise_rms_rad,6*y.residual_rms/std::sqrt(static_cast<double>(selected.size()))});
        if(std::abs(p.input_tone_nm)<=p.input_noise_floor_nm ||
           std::abs(tone(da,j))<=p.input_noise_floor_nm || std::abs(tone(db,j))<=p.input_noise_floor_nm)
            return reject("mit_independent_input_tone_below_noise","known_resolved_independent_torque_at_each_tone_and_half");
        if(std::abs(p.output_tone_rad)<=p.output_noise_floor_rad ||
           std::abs(tone(ya,j))<=p.output_noise_floor_rad || std::abs(tone(yb,j))<=p.output_noise_floor_rad)
            return reject("mit_independent_position_tone_below_noise","resolved_raw_position_response_at_each_tone_and_half");
        p.closed_loop_response=p.output_tone_rad/p.input_tone_nm;
        p.first_half_response=tone(ya,j)/tone(da,j);p.second_half_response=tone(yb,j)/tone(db,j);
        p.half_relative_error=relative(p.first_half_response,p.second_half_response);
        if(!finite(p.closed_loop_response) || !finite(p.first_half_response) || !finite(p.second_half_response) || p.half_relative_error>.35)
            return reject("mit_independent_half_windows_inconsistent","repeatable_command_to_position_response_in_both_halves");
        out.points.push_back(p);
    }
    out.valid=true;out.reason="independent_command_to_returned_position_three_tone_response_measured";return out;
}

inline MitClosedLoopIdentification combine_mit_closed_loop_trials(const std::vector<MitClosedLoopTrialEstimate>& trials) {
    using namespace mit_closed_loop_detail;
    MitClosedLoopIdentification out;
    const auto reject=[&](const std::string& reason,const std::string& missing) {
        out.reason=reason;out.missing_evidence.push_back(missing);
        out.response.valid=false;out.response.reason=reason;out.response.missing_evidence=out.missing_evidence;return out;
    };
    // A failed controller window can stop the sequence before three trials
    // exist. Preserve that measured failure instead of replacing it with the
    // secondary fact that the remaining experiments were not performed.
    for(const auto& trial:trials)if(!trial.valid && !trial.reason.empty() &&
        trial.reason!="independent_closed_loop_response_not_measured") {
        out.reason=trial.reason;out.missing_evidence=trial.missing_evidence;
        out.response.reason=out.reason;out.response.missing_evidence=out.missing_evidence;return out;
    }
    if(trials.size()<3)return reject("mit_controller_experiments_insufficient","three_noncollinear_actual_controller_experiments");
    const auto& base=trials.front();const auto& context=base.operating_point;
    if(!base.valid || !operating_point_valid(context) || base.points.size()<3)
        return reject("mit_controller_experiment_invalid","valid_independent_frequency_measurement_for_every_controller");
    const auto axis=context.excited_axis,n=context.actual_gains.kp.size();
    const double p0=context.actual_gains.kp[axis],d0=context.actual_gains.kd[axis];
    double pscale=0,dscale=0;
    for(std::size_t i=0;i<trials.size();++i) {
        const auto& trial=trials[i];const auto& point=trial.operating_point;
        if(!trial.valid || !operating_point_valid(point) || trial.points.size()!=base.points.size())
            return reject("mit_controller_experiment_invalid","valid_same_frequency_evidence_for_every_controller");
        if(point.excited_axis!=axis || point.actual_gains.kp.size()!=n || point.condition_id!=context.condition_id)
            return reject("mit_controller_operating_condition_changed","same_active_axis_load_feedforward_and_execution_context");
        for(std::size_t j=0;j<n;++j) {
            if(!close(point.fixed_reference_rad[j],context.fixed_reference_rad[j]))
                return reject("mit_controller_reference_changed","same_fixed_full_reference_during_all_controller_experiments");
            if(j!=axis && (!close(point.actual_gains.kp[j],context.actual_gains.kp[j],1e-6) ||
                          !close(point.actual_gains.kd[j],context.actual_gains.kd[j],1e-6)))
                return reject("mit_held_other_axis_gains_changed","identical_actual_other_axis_hold_controllers");
        }
        if(!close(trial.probe_duration_s,base.probe_duration_s) || trial.unloaded_baseline_measured!=base.unloaded_baseline_measured)
            return reject("mit_controller_measurement_window_changed","same_input_duration_and_unloaded_baseline_scope");
        if(trial.unloaded_baseline_measured) {
            const double radius=std::min(point.maximum_operating_excursion_rad,context.maximum_operating_excursion_rad);
            const double allowed=std::max({3*std::max(trial.encoder_step_rad,base.encoder_step_rad),
                6*std::max(trial.noise_rms_rad,base.noise_rms_rad),.25*radius});
            if(std::abs(trial.unloaded_mean_position_rad-base.unloaded_mean_position_rad)>allowed)
                return reject("mit_controller_unloaded_operating_posture_changed","same_local_actual_unloaded_operating_posture");
        }
        if(i<3) {pscale=std::max(pscale,std::abs(point.actual_gains.kp[axis]-p0));dscale=std::max(dscale,std::abs(point.actual_gains.kd[axis]-d0));}
        for(std::size_t j=0;j<base.points.size();++j) {
            const auto& a=base.points[j];const auto& b=trial.points[j];
            if(!close(a.hz,b.hz) || !finite(b.closed_loop_response) || std::abs(b.closed_loop_response)<=1e-12 ||
               !finite(b.first_half_response) || !finite(b.second_half_response) ||
               std::abs(b.first_half_response)<=1e-12 || std::abs(b.second_half_response)<=1e-12)
                return reject("mit_controller_frequency_evidence_changed","same_resolved_nonzero_tones_and_half_window_responses");
            const double input_tolerance=std::max(6*std::max(a.input_noise_floor_nm,b.input_noise_floor_nm),.05*std::abs(a.input_tone_nm));
            if(std::abs(a.input_tone_nm-b.input_tone_nm)>input_tolerance ||
               std::abs(a.input_dc_nm-b.input_dc_nm)>std::max(input_tolerance,.05*std::abs(a.input_dc_nm)))
                return reject("mit_controller_independent_input_changed","same_actual_independent_torque_waveform_amplitude_and_phase");
        }
    }
    if(pscale<=1e-8*std::max(1.0,p0) || dscale<=1e-8*std::max(1.0,d0))
        return reject("mit_controller_gain_variations_degenerate","independent_actual_protocol_P_and_D_changes");
    Eigen::Matrix3d design;
    for(unsigned i=0;i<3;++i) {
        design(i,0)=1;design(i,1)=(trials[i].operating_point.actual_gains.kp[axis]-p0)/pscale;
        design(i,2)=(trials[i].operating_point.actual_gains.kd[axis]-d0)/dscale;
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(design);
    const double condition=svd.singularValues()(0)/svd.singularValues()(2);
    if(!std::isfinite(condition) || condition>100)
        return reject("mit_controller_gain_design_ill_conditioned","well_separated_noncollinear_actual_controller_groups");
    const Eigen::Matrix3cd inverse=design.inverse().cast<std::complex<double>>();
    const auto solve=[&](std::size_t tone_index,int half) {
        Eigen::Vector3cd z;
        for(unsigned i=0;i<3;++i) {
            const auto& p=trials[i].points[tone_index];
            z(i)=1.0/(half==1?p.first_half_response:half==2?p.second_half_response:p.closed_loop_response);
        }
        const Eigen::Vector3cd normalized=inverse*z;
        std::array<std::complex<double>,3> abc;
        abc[1]=normalized(1)/pscale;abc[2]=normalized(2)/dscale;
        abc[0]=normalized(0)-abc[1]*p0-abc[2]*d0;return abc;
    };
    out.response.kind=ExcitationKind::IndependentTorqueDisturbance;out.response.isolated_axis_excitation=true;
    out.response.measured_kp=p0;out.response.measured_kd=d0;out.response.sample_period_s=base.sample_period_s;
    out.response.input_definition="independently_commanded_torque_and_returned_position_under_multiple_constant_actual_embedded_MIT_controllers";
    out.response.time_definition=base.time_definition;
    out.response.plant_scope="other_loops_fixed_equivalent_internal_position_feedback_port_with_fixed_executed_state_feedforward_not_pure_mechanical_plant";
    out.response.loop_plant_definition="G_eff=b/a_and_internal_velocity_channel=c/b_from_inverse_closed_loop_H=a+bP+cD_common_external_transport_cancels";
    out.response.executed_state_feedforward_gradient=std::numeric_limits<double>::quiet_NaN();
    for(const auto& trial:trials)out.response.fresh_samples+=trial.fresh_samples;
    for(std::size_t j=0;j<base.points.size();++j) {
        const auto abc=solve(j,0),first=solve(j,1),second=solve(j,2);
        MitClosedLoopIdentificationPoint p;p.hz=base.points[j].hz;p.design_condition_number=condition;
        p.a=abc[0];p.b=abc[1];p.c=abc[2];
        if(!finite(p.a) || !finite(p.b) || !finite(p.c) ||
           !finite(first[0]) || !finite(first[1]) || !finite(first[2]) ||
           !finite(second[0]) || !finite(second[1]) || !finite(second[2]) ||
           std::abs(p.a)<=1e-12 || std::abs(p.b)<=1e-12 ||
           std::abs(first[0])<=1e-12 || std::abs(second[0])<=1e-12 || std::abs(first[1])<=1e-12 || std::abs(second[1])<=1e-12)
            return reject("mit_internal_feedback_port_unidentifiable","resolved_affine_intercept_and_internal_position_feedback_channel");
        p.g_eff=p.b/p.a;p.internal_velocity=p.c/p.b;
        p.half_plant_relative_error=relative(first[1]/first[0],second[1]/second[0]);
        p.half_velocity_relative_error=relative(first[2]/first[1],second[2]/second[1],1e-6);
        if(!finite(p.g_eff) || !finite(p.internal_velocity) || p.half_plant_relative_error>.35 || p.half_velocity_relative_error>.35)
            return reject("mit_internal_feedback_port_half_windows_inconsistent","repeatable_G_eff_and_internal_velocity_from_independent_half_windows");
        for(std::size_t i=3;i<trials.size();++i) {
            const double kp=trials[i].operating_point.actual_gains.kp[axis],kd=trials[i].operating_point.actual_gains.kd[axis];
            const auto& h=trials[i].points[j];
            p.gain_holdout_relative_error=std::max({p.gain_holdout_relative_error,
                relative(p.a+p.b*kp+p.c*kd,1.0/h.closed_loop_response),
                relative(first[0]+first[1]*kp+first[2]*kd,1.0/h.first_half_response),
                relative(second[0]+second[1]*kp+second[2]*kd,1.0/h.second_half_response)});
        }
        if(p.gain_holdout_relative_error>.20)
            return reject("mit_affine_inverse_response_gain_holdout_failed","independent_extra_controller_response_matching_the_three_controller_prediction");
        out.points.push_back(p);
        EmpiricalMitFrequencyPoint empirical;const auto& measured=base.points[j];
        empirical.hz=p.hz;empirical.plant=p.g_eff;empirical.feedback_velocity_per_position=p.internal_velocity;
        const double unknown=std::numeric_limits<double>::quiet_NaN();
        empirical.physical_plant={unknown,unknown};empirical.executed_feedforward_per_position={unknown,unknown};
        empirical.disturbance_response=measured.closed_loop_response;
        empirical.input_amplitude_nm=std::abs(measured.input_tone_nm);empirical.output_amplitude_rad=std::abs(measured.output_tone_rad);
        empirical.instrument_amplitude=empirical.input_amplitude_nm;empirical.input_noise_floor_nm=measured.input_noise_floor_nm;
        empirical.output_noise_floor_rad=measured.output_noise_floor_rad;
        // Legacy FRF storage has a field named coherence. Tone-fit R² remains
        // in MitClosedLoopToneEstimate; it is not spectral coherence and must
        // not be placed in that field or reported as a coherence measurement.
        empirical.coherence=unknown;
        empirical.half_relative_error=p.half_plant_relative_error;
        empirical.velocity_half_relative_error=p.half_velocity_relative_error;
        empirical.loop_input_amplitude_nm=unknown;empirical.loop_input_noise_floor_nm=unknown;
        out.response.points.push_back(empirical);
    }
    out.gain_holdout_verified=trials.size()>3;out.valid=true;out.response.valid=true;
    out.response.disturbance_response_valid=true;
    out.reason=out.gain_holdout_verified?"equivalent_internal_MIT_loop_identified_and_extra_controller_holdout_verified":
        "three_controller_local_model_without_extra_gain_holdout";
    out.response.reason=out.reason;
    if(!out.gain_holdout_verified)out.missing_evidence.push_back("independent_extra_controller_gain_holdout_not_measured");
    return out;
}

template<std::size_t N>
inline MitClosedLoopIdentification combine_mit_closed_loop_trials(const std::array<MitClosedLoopTrialEstimate,N>& trials) {
    return combine_mit_closed_loop_trials(std::vector<MitClosedLoopTrialEstimate>(trials.begin(),trials.end()));
}

} // namespace serial_arm::servo_tuning
