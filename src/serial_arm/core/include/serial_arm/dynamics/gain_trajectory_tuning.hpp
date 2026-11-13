#pragma once

#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

// First establish and validate a complete taught trajectory. Optional
// disturbance supplements and local comparisons compete for the remaining
// original wall budget afterwards; reserving every possible supplement here
// previously stopped discovery near its last endpoint despite ample time.
inline double discovery_required_validation_reserve(double current_frozen_path_s,
    const std::vector<double>& other_required_paths_s,double required_hold_s,double required_drag_s,double exit_s=2) {
    double reserve=0;
    for(double value:{current_frozen_path_s,required_hold_s,required_drag_s,exit_s}) {
        if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid required gain validation cost");
        reserve+=value;
    }
    for(double value:other_required_paths_s) {
        if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid remaining gain trajectory cost");
        reserve+=value;
    }
    return reserve;
}

inline bool trajectory_measurement_complete(const std::vector<Sample>& rows,double duration_s,double tail_s) {
    if(rows.empty() || rows.front().time_s>.06 || rows.back().time_s<duration_s+tail_s-.06)return false;
    std::size_t fresh=0;double previous=rows.front().time_s;
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || !std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) ||
           !std::isfinite(row.actual) || !std::isfinite(row.velocity) || row.time_s<0 || row.time_s<previous)return false;
        previous=row.time_s;
        if(row.new_measurement)++fresh;
    }
    return fresh>0;
}

inline std::vector<Sample> rolling_noise_window(const std::vector<Sample>& rows,double settle_s=.4,double window_s=1.2) {
    std::vector<Sample> window;if(rows.empty())return window;
    const double begin=std::max(settle_s,rows.back().time_s-window_s);
    for(const auto& row:rows)if(row.time_s>=begin)window.push_back(row);
    return window;
}

inline NoiseEstimate usable_noise_estimate(const NoiseEstimate& measured) {
    if(measured.stationary)return measured;
    // A moving baseline is not a measurement of stationary jitter. Preserve
    // observed encoder resolution, but never absorb drift into noise margins
    NoiseEstimate conservative;conservative.samples=measured.samples;conservative.encoder_step_rad=measured.encoder_step_rad;
    conservative.span_rad=measured.span_rad;conservative.drift_rad=measured.drift_rad;
    return conservative;
}

// A demonstrated path is not a three-tone identification signal. Its measured
// tracking errors and commanded direction coverage are the acceptance evidence
struct TrajectoryTuningEvidence {
    std::size_t samples{0};
    double position_floor_rad{0},bias_rad{0},centered_rms_rad{0},reference_rms_rad{0};
    double in_phase_gain{1},lag_s{0},amplitude_error_rms_rad{0},velocity_error_rms_rad{0};
    bool valid{false};
};

// This is a description of a measured closed-loop residual, not a pole fit or
// a mechanical-resonance diagnosis. A commanded sinusoid (including one the
// motor cannot follow) must not become evidence of an uncommanded oscillation.
struct TrajectoryResidualMotion {
    bool valid{false},sustained_uncommanded{false},reference_component_present{false};
    std::size_t examined_windows{0},repeated_windows{0};
    double position_floor_rad{0},frequency_hz{0},amplitude_rad{0},reference_amplitude_rad{0};
    double explained_fraction{0},duration_s{0};
};

// A complete, short window of the taught path. Unlike spectral identification,
// this evidence accepts one-way motion and a stationary endpoint; it never
// requires an artificial excursion on the other side of a mechanical stop.
struct TrajectorySegmentAssessment {
    std::size_t samples{0},position_reversals{0},reference_reversals{0};
    int reference_direction{0};
    double duration_s{0},position_floor_rad{0},observed_position_step_rad{0},reference_span_rad{0},actual_span_rad{0};
    double reference_travel_rad{0},actual_travel_rad{0},mean_error_rad{0},rms_error_rad{0},peak_error_rad{0};
    double early_error_rms_rad{0},late_error_rms_rad{0},velocity_error_rms_rad_s{0};
    double velocity_error_peak_rad_s{0};
    double pd_position_rms_nm{0},pd_velocity_rms_nm{0},feedback_age_max_s{0},sample_interval_max_s{0};
    bool valid{false},continuous{true},gains_constant{true},gains_known{false},limited{false},timing_known{false};
    bool reference_quiet{false},position_quiet{false},uncommanded_reversals{false},velocity_noise_risk{false};
    std::string reason{"insufficient_complete_segment_measurement"};
};
struct TrajectoryCandidateReport {Gains gains;std::vector<std::string> reasons;bool changed{false};};

inline TrajectorySegmentAssessment assess_trajectory_segment(const std::vector<Sample>& rows,
    const Requirements&,const NoiseEstimate& noise,double minimum_duration_s=.6) {
    TrajectorySegmentAssessment out;
    if(!std::isfinite(minimum_duration_s) || minimum_duration_s<=0 || !std::isfinite(noise.encoder_step_rad) || noise.encoder_step_rad<0 ||
       !std::isfinite(noise.rms_rad) || noise.rms_rad<0)return out;
    out.position_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    std::vector<const Sample*> fresh;std::uint64_t sequence=0;
    for(const auto& row:rows)if(row.new_measurement && (!row.feedback_sequence || row.feedback_sequence!=sequence)) {
        sequence=row.feedback_sequence;
        if(!std::isfinite(row.time_s) || !std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) ||
           !std::isfinite(row.actual) || !std::isfinite(row.velocity))return out;
        if(!fresh.empty()) {
            const double dt=row.time_s-fresh.back()->time_s;
            out.sample_interval_max_s=std::max(out.sample_interval_max_s,dt);
            if(dt<=0 || dt>.06)out.continuous=false;
        }
        out.limited=out.limited || row.limited;fresh.push_back(&row);
    }
    out.samples=fresh.size();if(fresh.empty())return out;
    std::vector<double> positions;positions.reserve(fresh.size());for(const auto* row:fresh)positions.push_back(row->actual);
    out.observed_position_step_rad=observed_encoder_step(std::move(positions));
    const double step=noise.encoder_step_rad>0 && out.observed_position_step_rad>0?
        std::min(noise.encoder_step_rad,out.observed_position_step_rad):std::max(noise.encoder_step_rad,out.observed_position_step_rad);
    out.position_floor_rad=std::max({3*step,6*noise.rms_rad,.0002});
    out.duration_s=fresh.back()->time_s-fresh.front()->time_s;
    if(out.samples<50 || out.duration_s<minimum_duration_s || !out.continuous)return out;
    double qlo=fresh.front()->actual,qhi=qlo,rlo=fresh.front()->reference,rhi=rlo;
    double square=0,early=0,late=0,vsquare=0,psquare=0,dsquare=0;std::size_t early_n=0,late_n=0,known=0,timed=0;
    const double middle=.5*(fresh.front()->time_s+fresh.back()->time_s);
    for(const auto* row:fresh) {
        const double e=row->reference-row->actual,v=row->reference_velocity-row->velocity;
        out.mean_error_rad+=e;square+=e*e;vsquare+=v*v;out.peak_error_rad=std::max(out.peak_error_rad,std::abs(e));
        out.velocity_error_peak_rad_s=std::max(out.velocity_error_peak_rad_s,std::abs(v));
        if(row->time_s<middle){early+=e*e;++early_n;}else{late+=e*e;++late_n;}
        qlo=std::min(qlo,row->actual);qhi=std::max(qhi,row->actual);rlo=std::min(rlo,row->reference);rhi=std::max(rhi,row->reference);
        if(row->actual_gains_known && std::isfinite(row->actual_kp) && std::isfinite(row->actual_kd)) {
            ++known;psquare+=std::pow(row->actual_kp*e,2);dsquare+=std::pow(row->actual_kd*v,2);
            if(std::abs(row->actual_kp-fresh.front()->actual_kp)>1e-6 || std::abs(row->actual_kd-fresh.front()->actual_kd)>1e-6)out.gains_constant=false;
        }
        if(std::isfinite(row->command_time_s) && std::isfinite(row->feedback_time_s)) {
            ++timed;const double age=row->command_time_s-row->feedback_time_s;
            if(age<-.002 || age>.06)out.continuous=false;
            out.feedback_age_max_s=std::max(out.feedback_age_max_s,age);
        }
    }
    out.gains_known=known==fresh.size();out.timing_known=timed==fresh.size();
    out.mean_error_rad/=out.samples;out.rms_error_rad=std::sqrt(square/out.samples);out.velocity_error_rms_rad_s=std::sqrt(vsquare/out.samples);
    out.early_error_rms_rad=std::sqrt(early/std::max<std::size_t>(1,early_n));out.late_error_rms_rad=std::sqrt(late/std::max<std::size_t>(1,late_n));
    if(out.gains_known){out.pd_position_rms_nm=std::sqrt(psquare/out.samples);out.pd_velocity_rms_nm=std::sqrt(dsquare/out.samples);}
    out.reference_span_rad=rhi-rlo;out.actual_span_rad=qhi-qlo;
    const double commanded_travel=fresh.back()->reference-fresh.front()->reference;
    if(std::abs(commanded_travel)>out.position_floor_rad)out.reference_direction=commanded_travel>0?1:-1;
    const auto excursion=[&](bool actual) {
        double anchor=actual?fresh.front()->actual:fresh.front()->reference,extreme=anchor,travel=0;int direction=0;std::size_t reversals=0;
        for(const auto* row:fresh) {
            const double q=actual?row->actual:row->reference;
            if(!direction){if(std::abs(q-anchor)>=out.position_floor_rad){direction=q>anchor?1:-1;extreme=q;}}
            else if((q-extreme)*direction>=0)extreme=q;
            else if((extreme-q)*direction>=out.position_floor_rad){travel+=std::abs(extreme-anchor);anchor=extreme;extreme=q;direction=-direction;++reversals;}
        }
        return std::pair<std::size_t,double>{reversals,travel+std::abs(extreme-anchor)};
    };
    const auto q=excursion(true),r=excursion(false);out.position_reversals=q.first;out.reference_reversals=r.first;
    out.actual_travel_rad=q.second;out.reference_travel_rad=r.second;
    out.reference_quiet=out.reference_span_rad<=out.position_floor_rad;out.position_quiet=out.actual_span_rad<=out.position_floor_rad;
    const bool growing=out.late_error_rms_rad>1.25*out.early_error_rms_rad+out.position_floor_rad;
    out.uncommanded_reversals=q.first>=r.first+2 && q.second>2*r.second+4*out.position_floor_rad &&
        (q.first>=4 || growing);
    // These are estimated MIT terms evaluated at returned velocity, not a
    // measurement of the actuator's internal damping torque or encoder rate.
    out.velocity_noise_risk=out.gains_known && (out.reference_quiet || out.uncommanded_reversals) &&
        out.pd_velocity_rms_nm>2*std::max(out.pd_position_rms_nm,fresh.front()->actual_kp*out.position_floor_rad) &&
        out.velocity_error_rms_rad_s>std::max(.01,2*noise.velocity_rms);
    out.valid=out.continuous && !out.limited;
    out.reason=!out.valid?(out.limited?"limited_segment":"discontinuous_segment_feedback"):
        out.uncommanded_reversals?"uncommanded_segment_reversals":out.velocity_noise_risk?"velocity_feedback_noise_dominates_pd":
        out.gains_known && !out.gains_constant?"segment_feedback_gains_still_changing":"complete_taught_segment_measurement";
    return out;
}

inline TrajectoryCandidateReport trajectory_segment_candidate_report(const Gains& current,
    const std::vector<double>& kp_max,const std::vector<double>& kd_max,
    const std::vector<TrajectorySegmentAssessment>& measured,const Requirements& req,const std::vector<double>& kp_step={}) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || kp_max.size()!=n || kd_max.size()!=n || measured.size()!=n || (!kp_step.empty() && kp_step.size()!=n))
        throw std::invalid_argument("trajectory segment candidate mapping mismatch");
    TrajectoryCandidateReport out;out.gains=current;out.reasons.resize(n);
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(current.kp[i]) || current.kp[i]<0 || !std::isfinite(current.kd[i]) || current.kd[i]<0 ||
           !std::isfinite(kp_max[i]) || kp_max[i]<=0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 ||
           current.kp[i]>kp_max[i] || current.kd[i]>kd_max[i])throw std::invalid_argument("invalid segment gain capability");
        const auto& a=measured[i];auto& reason=out.reasons[i];reason=a.reason;
        if(!a.valid || !a.gains_known || !a.gains_constant)continue;
        if(a.velocity_noise_risk) {
            out.gains.kd[i]=.9*current.kd[i];reason="measured_segment_velocity_feedback_noise_reduce_damping_only";
        } else if(a.uncommanded_reversals) {
            out.gains.kp[i]=.92*current.kp[i];reason="measured_segment_uncommanded_reversals_reduce_stiffness_only";
        } else if(a.sample_interval_max_s>.02 || (a.timing_known && a.feedback_age_max_s>.025)) {
            reason="segment_timing_insufficient_for_feedback_increase";
        } else if(a.peak_error_rad>req.peak_error_rad && std::abs(a.mean_error_rad)>a.position_floor_rad &&
                  a.peak_error_rad>2*a.position_floor_rad &&
                  (!a.reference_direction || a.mean_error_rad*a.reference_direction>a.position_floor_rad)) {
            // One-way measured tracking and a blocked reference are useful
            // evidence. Require neither a symmetric probe nor a bandwidth fit.
            const double step=kp_step.empty()?0:kp_step[i];
            if(!std::isfinite(step) || step<0)throw std::invalid_argument("invalid segment gain grid");
            const bool unresolved=a.position_quiet && a.position_reversals==0;
            out.gains.kp[i]=std::min(std::max((unresolved?1.25:1.08)*current.kp[i],current.kp[i]+step),kp_max[i]);
            reason=unresolved?"measured_taught_segment_unresolved_motion_bounded_stiffness_only_not_hardware_limit":
                "measured_taught_segment_tracking_deficit_small_stiffness_only_comparison";
        } else reason="segment_requires_no_supported_gain_increase";
        out.changed=out.changed || out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i];
    }
    return out;
}

struct TrajectoryAssessment {
    Metrics metrics;
    MotionObservation motion;
    TrajectoryTuningEvidence tuning;
    TrajectoryResidualMotion residual_motion;
    TrajectorySegmentAssessment feedback_evidence;
    double reference_positive_travel_rad{0},reference_negative_travel_rad{0};
    double steady_error_rad{0},dynamic_overshoot_rad{0},local_reversal_overshoot_rad{0};
    std::size_t tail_position_reversals{0};
    bool reference_coverage{false},stationary_reference{false},responsive{false},passed{false};
    std::string reason{"insufficient_measurements"};
};

inline TrajectoryResidualMotion trajectory_residual_motion(const std::vector<Sample>& rows,
    const NoiseEstimate& noise,double end_s,double settling_s) {
    TrajectoryResidualMotion out;
    out.position_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    if(!std::isfinite(end_s) || end_s<=0 || !std::isfinite(settling_s) || settling_s<=0 ||
       !std::isfinite(noise.encoder_step_rad) || noise.encoder_step_rad<0 ||
       !std::isfinite(noise.rms_rad) || noise.rms_rad<0 || !std::isfinite(out.position_floor_rad))return out;
    std::vector<const Sample*> fresh;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=0 && row.time_s<=end_s) {
        if(!std::isfinite(row.time_s) || !std::isfinite(row.actual) || !std::isfinite(row.reference))return out;
        if(!fresh.empty() && (row.time_s<=fresh.back()->time_s || row.time_s-fresh.back()->time_s>.06))return out;
        fresh.push_back(&row);
    }
    if(fresh.size()<100)return out;
    // Four cycles per window distinguish repeated excursions from one turn or
    // one overshoot. The upper frequency follows the actual sample interval;
    // at least five measured samples per cycle are required.
    const double window_s=std::clamp(2*settling_s,.8,2.0);
    const double step_s=.5*window_s;
    std::vector<double> intervals;intervals.reserve(fresh.size()-1);
    for(std::size_t k=1;k<fresh.size();++k)intervals.push_back(fresh[k]->time_s-fresh[k-1]->time_s);
    std::sort(intervals.begin(),intervals.end());
    const double max_frequency=std::min(40.0,.2/intervals[intervals.size()/2]);
    const double min_frequency=4/window_s,frequency_step=.5/window_s;
    if(max_frequency<min_frequency || fresh.back()->time_s-fresh.front()->time_s<window_s)return out;
    std::size_t begin=0,finish=0,run=0;double run_start=0,run_frequency=0;
    for(double start=fresh.front()->time_s;start+window_s<=fresh.back()->time_s+1e-9;start+=step_s) {
        while(begin<fresh.size() && fresh[begin]->time_s<start)++begin;
        finish=std::max(finish,begin);
        while(finish<fresh.size() && fresh[finish]->time_s<=start+window_s)++finish;
        if(finish-begin<40)continue;
        ++out.examined_windows;
        // Remove offset, slope and curvature before spectral comparison. This
        // keeps a slow demonstration leg or load bias out of the ripple band.
        Eigen::Matrix3d normal=Eigen::Matrix3d::Zero();
        Eigen::Vector3d residual_rhs=Eigen::Vector3d::Zero(),reference_rhs=Eigen::Vector3d::Zero();
        const double reference_origin=fresh[begin]->reference;
        for(std::size_t k=begin;k<finish;++k) {
            const double u=2*(fresh[k]->time_s-start)/window_s-1;
            const Eigen::Vector3d basis(1,u,u*u);
            normal+=basis*basis.transpose();
            residual_rhs+=basis*(fresh[k]->actual-fresh[k]->reference);
            reference_rhs+=basis*(fresh[k]->reference-reference_origin);
        }
        const auto decomposition=normal.ldlt();
        if(decomposition.info()!=Eigen::Success)continue;
        const Eigen::Vector3d residual_trend=decomposition.solve(residual_rhs),reference_trend=decomposition.solve(reference_rhs);
        struct Point {double time,weight,error,reference;};std::vector<Point> points;points.reserve(finish-begin);
        double error_energy=0;
        for(std::size_t k=begin;k<finish;++k) {
            const double relative=(fresh[k]->time_s-start)/window_s,u=2*relative-1;
            const Eigen::Vector3d basis(1,u,u*u);const double weight=.5-.5*std::cos(2*pi*relative);
            const double error=fresh[k]->actual-fresh[k]->reference-basis.dot(residual_trend);
            points.push_back({fresh[k]->time_s-start,weight,error,fresh[k]->reference-reference_origin-basis.dot(reference_trend)});
            error_energy+=weight*error*error;
        }
        double best_amp=0,best_reference=0,best_fraction=0,best_frequency=0;
        for(double frequency=min_frequency;frequency<=max_frequency+1e-9;frequency+=frequency_step) {
            double ss=0,cc=0,sc=0,se=0,ce=0,sq=0,cq=0;
            for(const auto& point:points) {
                const double s=std::sin(2*pi*frequency*point.time),c=std::cos(2*pi*frequency*point.time);
                ss+=point.weight*s*s;cc+=point.weight*c*c;sc+=point.weight*s*c;
                se+=point.weight*s*point.error;ce+=point.weight*c*point.error;
                sq+=point.weight*s*point.reference;cq+=point.weight*c*point.reference;
            }
            const double determinant=ss*cc-sc*sc;if(determinant<=1e-12)continue;
            const double es=(se*cc-ce*sc)/determinant,ec=(ce*ss-se*sc)/determinant;
            const double qs=(sq*cc-cq*sc)/determinant,qc=(cq*ss-sq*sc)/determinant;
            const double amplitude=std::hypot(es,ec),reference_amplitude=std::hypot(qs,qc);
            const double fraction=error_energy>0?std::clamp((es*se+ec*ce)/error_energy,0.0,1.0):0;
            if(amplitude>2*out.position_floor_rad && fraction>=.35 &&
               reference_amplitude>std::max(.5*out.position_floor_rad,.2*amplitude))out.reference_component_present=true;
            if(amplitude>best_amp && fraction>=.35 &&
               reference_amplitude<=std::max(.5*out.position_floor_rad,.2*amplitude)) {
                best_amp=amplitude;best_reference=reference_amplitude;best_fraction=fraction;best_frequency=frequency;
            }
        }
        if(best_amp<=2*out.position_floor_rad){run=0;continue;}
        const bool same_band=run && std::abs(best_frequency-run_frequency)<=std::max(frequency_step,.2*run_frequency);
        if(!same_band){run=1;run_start=start;run_frequency=best_frequency;}else ++run;
        if(run>=2 && run>out.repeated_windows) {
            out.repeated_windows=run;out.frequency_hz=best_frequency;out.amplitude_rad=best_amp;
            out.reference_amplitude_rad=best_reference;out.explained_fraction=best_fraction;
            out.duration_s=start+window_s-run_start;out.sustained_uncommanded=true;
        }
    }
    out.valid=out.examined_windows>=2;return out;
}

inline double trajectory_local_reversal_overshoot(const std::vector<Sample>& rows,double end_s,
    double position_floor,double stationary_bias,double settling_s) {
    std::vector<const Sample*> fresh;for(const auto& row:rows)if(row.new_measurement && row.time_s>=0 && row.time_s<=end_s+settling_s)fresh.push_back(&row);
    if(fresh.size()<3)return 0;
    struct Turn {std::size_t index;int previous_direction;};std::vector<Turn> turns;
    std::size_t extreme=0;int direction=0;
    for(std::size_t k=1;k<fresh.size() && fresh[k]->time_s<=end_s;++k) {
        const double delta=fresh[k]->reference-fresh[extreme]->reference;
        if(!direction){if(std::abs(delta)>=position_floor){direction=delta>0?1:-1;extreme=k;}}
        else if(delta*direction>0)extreme=k;
        else if(-delta*direction>=position_floor){turns.push_back({extreme,direction});direction=-direction;extreme=k;}
    }
    double overshoot=0;
    for(std::size_t k=0;k<turns.size();++k) {
        const auto turn=turns[k];const auto& pivot=*fresh[turn.index];
        const double end=std::min(pivot.time_s+settling_s,k+1<turns.size()?fresh[turns[k+1].index]->time_s:end_s+settling_s);
        for(std::size_t i=turn.index;i<fresh.size() && fresh[i]->time_s<=end;++i)
            overshoot=std::max(overshoot,turn.previous_direction*(fresh[i]->actual-pivot.reference)-
                std::max(0.0,turn.previous_direction*stationary_bias));
    }
    return overshoot;
}

inline TrajectoryTuningEvidence trajectory_tuning_evidence(const std::vector<Sample>& rows,
                                                           const NoiseEstimate& noise,double end_s) {
    TrajectoryTuningEvidence out;
    if(!std::isfinite(end_s) || end_s<=0 || !std::isfinite(noise.encoder_step_rad) || noise.encoder_step_rad<0 ||
       !std::isfinite(noise.rms_rad) || noise.rms_rad<0)return out;
    out.position_floor_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002});
    double q=0,v=0,error=0;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=.7 && row.time_s<end_s-.7) {
        if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual))return out;
        q+=row.reference;v+=row.reference_velocity;error+=row.actual-row.reference;++out.samples;
    }
    if(out.samples<100)return out;
    const double count=static_cast<double>(out.samples);q/=count;v/=count;error/=count;
    double qq=0,vv=0,qv=0,qe=0,ve=0,ee=0;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=.7 && row.time_s<end_s-.7) {
        const double a=row.reference-q,b=row.reference_velocity-v,e=row.actual-row.reference-error;
        qq+=a*a;vv+=b*b;qv+=a*b;qe+=a*e;ve+=b*e;ee+=e*e;
    }
    out.bias_rad=error;out.centered_rms_rad=std::sqrt(ee/count);out.reference_rms_rad=std::sqrt(qq/count);
    const double determinant=qq*vv-qv*qv;
    // Separating amplitude from velocity-correlated error needs two independent
    // reference components. Never fit a moving DC offset or an ill-conditioned
    // path, and never time-shift feedback to improve the reported errors
    if(out.reference_rms_rad<=out.position_floor_rad || vv/count<=1e-12 ||
       !std::isfinite(determinant) || determinant<=1e-6*qq*vv)return out;
    const double amplitude=(qe*vv-ve*qv)/determinant,velocity=(ve*qq-qe*qv)/determinant;
    out.in_phase_gain=1+amplitude;out.lag_s=-velocity;
    out.amplitude_error_rms_rad=std::abs(amplitude)*out.reference_rms_rad;
    out.velocity_error_rms_rad=std::abs(velocity)*std::sqrt(vv/count);
    out.valid=std::isfinite(out.in_phase_gain) && std::isfinite(out.lag_s) &&
        std::isfinite(out.centered_rms_rad) && std::isfinite(out.amplitude_error_rms_rad) &&
        std::isfinite(out.velocity_error_rms_rad);
    return out;
}

inline std::size_t tail_position_reversals(const std::vector<Sample>& rows,double start_s,double motion_floor) {
    bool initialized=false;int direction=0;double extreme=0;std::size_t reversals=0;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=start_s) {
        if(!initialized){extreme=row.actual;initialized=true;continue;}
        if(!direction) {
            if(row.actual>extreme+motion_floor){direction=1;extreme=row.actual;}
            else if(row.actual<extreme-motion_floor){direction=-1;extreme=row.actual;}
        } else if(direction>0) {
            if(row.actual>extreme)extreme=row.actual;
            else if(row.actual<extreme-motion_floor){++reversals;direction=-1;extreme=row.actual;}
        } else {
            if(row.actual<extreme)extreme=row.actual;
            else if(row.actual>extreme+motion_floor){++reversals;direction=1;extreme=row.actual;}
        }
    }
    return reversals;
}

inline TrajectorySegmentAssessment trajectory_feedback_evidence(const std::vector<Sample>& rows,
    const Requirements& req,const NoiseEstimate& noise) {
    auto out=assess_trajectory_segment(rows,req,noise);
    if(rows.empty())return out;
    // Inspect complete local windows as well as the full traversal: a quiet
    // tail must not hide the onset of reversals during an earlier path leg.
    std::size_t begin=0,end=0;
    for(double start=rows.front().time_s;start+.6<=rows.back().time_s;start+=.5) {
        while(begin<rows.size() && rows[begin].time_s<start)++begin;
        end=std::max(end,begin);while(end<rows.size() && rows[end].time_s<=start+1)++end;
        const auto local=assess_trajectory_segment(std::vector<Sample>(rows.begin()+begin,rows.begin()+end),req,noise);
        if(!local.valid || !local.gains_constant)continue;
        out.uncommanded_reversals=out.uncommanded_reversals || local.uncommanded_reversals;
        out.velocity_noise_risk=out.velocity_noise_risk || local.velocity_noise_risk;
        out.pd_position_rms_nm=std::max(out.pd_position_rms_nm,local.pd_position_rms_nm);
        out.pd_velocity_rms_nm=std::max(out.pd_velocity_rms_nm,local.pd_velocity_rms_nm);
    }
    return out;
}

inline TrajectoryAssessment assess_trajectory(const std::vector<Sample>& rows,Requirements req,
                                               const NoiseEstimate& noise,double excitation_end_s) {
    TrajectoryAssessment out;
    if(rows.empty() || !std::isfinite(excitation_end_s) || excitation_end_s<=0)return out;
    req.probe_duration_s=excitation_end_s;
    req.noise_rms_rad=noise.rms_rad;req.encoder_step_rad=noise.encoder_step_rad;
    req.velocity_noise_floor=noise.velocity_peak+1e-6;
    out.metrics=assess(rows,req,excitation_end_s);
    out.motion=observe_motion(rows,req,noise);
    out.tuning=trajectory_tuning_evidence(rows,noise,excitation_end_s);
    out.residual_motion=trajectory_residual_motion(rows,noise,excitation_end_s,req.settling_time_s);
    out.feedback_evidence=trajectory_feedback_evidence(rows,req,noise);
    // A motor velocity code can alternate at rest after a monotone approach.
    // Count reversals only when fresh positions actually move back across the
    // measured encoder/noise floor; preserve the raw tracking diagnostics
    out.tail_position_reversals=tail_position_reversals(rows,excitation_end_s,
        std::max({3*out.motion.encoder_step_rad,6*noise.rms_rad,.0002}));
    out.metrics.oscillating=out.residual_motion.sustained_uncommanded ||
        (out.tail_position_reversals>=4 && out.metrics.tail_drift>req.settling_error_rad);
    double tail_sum=0,tail_reference_low=1e30,tail_reference_high=-1e30;
    double tail_actual_low=1e30,tail_actual_high=-1e30;
    double reference_low=1e30,reference_high=-1e30,actual_low=1e30,actual_high=-1e30;
    std::size_t tail_count=0;
    const double settled_window_start=std::max(excitation_end_s+.05,rows.back().time_s-.5);
    for(const auto& row:rows)if(row.new_measurement) {
        reference_low=std::min(reference_low,row.reference);reference_high=std::max(reference_high,row.reference);
        actual_low=std::min(actual_low,row.actual);actual_high=std::max(actual_high,row.actual);
        if(row.time_s<settled_window_start)continue;
        tail_sum+=row.actual-row.reference;++tail_count;
        tail_reference_low=std::min(tail_reference_low,row.reference);
        tail_reference_high=std::max(tail_reference_high,row.reference);
        tail_actual_low=std::min(tail_actual_low,row.actual);
        tail_actual_high=std::max(tail_actual_high,row.actual);
    }
    if(tail_count)out.steady_error_rad=tail_sum/tail_count;
    const bool stationary_bias=tail_count>=20 && tail_reference_high-tail_reference_low<1e-6 &&
        tail_actual_high-tail_actual_low<=std::max({3*out.motion.encoder_step_rad,6*noise.rms_rad,.0002}) && !out.metrics.oscillating;
    // Keep raw overshoot as diagnostic evidence. Candidate direction separates
    // a stationary load offset from dynamic overshoot; neither is substituted
    // for the final stationary-target positioning score.
    const double bias=stationary_bias?out.steady_error_rad:0.0;
    out.dynamic_overshoot_rad=std::max({0.0,actual_high-reference_high-std::max(0.0,bias),
        reference_low-actual_low+std::min(0.0,bias)});
    out.local_reversal_overshoot_rad=trajectory_local_reversal_overshoot(rows,excitation_end_s,
        std::max({3*out.motion.encoder_step_rad,6*noise.rms_rad,.0002}),bias,req.settling_time_s);
    out.dynamic_overshoot_rad=std::max(out.dynamic_overshoot_rad,out.local_reversal_overshoot_rad);
    // Identify meaningful monotone legs without assuming that the first point
    // lies at the middle of an arbitrary demonstrated path
    const Sample* first=nullptr;const Sample* previous=nullptr;int direction=0;
    const auto leg=[&](const Sample* a,const Sample* b) {
        if(!a || !b)return;
        const double travel=b->reference-a->reference;
        out.reference_positive_travel_rad=std::max(out.reference_positive_travel_rad,travel);
        out.reference_negative_travel_rad=std::max(out.reference_negative_travel_rad,-travel);
    };
    for(const auto& row:rows)if(row.new_measurement && row.time_s<=excitation_end_s) {
        if(!first){first=previous=&row;continue;}
        const double delta=row.reference-previous->reference;
        const int next=delta>1e-8?1:delta< -1e-8?-1:0;
        if(next && direction && next!=direction){leg(first,previous);first=previous;}
        if(next)direction=next;
        previous=&row;
    }
    leg(first,previous);
    const double floor=std::max({6*out.motion.encoder_step_rad,6*noise.rms_rad,1e-6});
    const bool positive_requested=out.reference_positive_travel_rad>=floor,negative_requested=out.reference_negative_travel_rad>=floor;
    out.stationary_reference=out.metrics.reference_span<floor;
    out.reference_coverage=out.stationary_reference || positive_requested || negative_requested;
    const double requested_travel=positive_requested && negative_requested?
        std::min(out.reference_positive_travel_rad,out.reference_negative_travel_rad):
        std::max(out.reference_positive_travel_rad,out.reference_negative_travel_rad);
    const double required=std::max(floor,.2*requested_travel);
    out.motion.minimum_motion_span_rad=required;
    out.motion.bidirectional=positive_requested && negative_requested && out.motion.positive_travel_rad>=required && out.motion.negative_travel_rad>=required;
    out.motion.quantization_boundary=!out.motion.bidirectional && out.motion.encoder_step_rad>0 &&
        std::min(out.motion.positive_travel_rad,out.motion.negative_travel_rad)>0 &&
        std::min(out.motion.positive_travel_rad,out.motion.negative_travel_rad)+out.motion.encoder_step_rad>=required && out.metrics.response_rms_ratio>=.2;
    const bool requested_motion_observed=(!positive_requested || out.motion.positive_travel_rad>=required) &&
        (!negative_requested || out.motion.negative_travel_rad>=required);
    out.responsive=!out.stationary_reference && out.reference_coverage && requested_motion_observed && out.metrics.response_rms_ratio>=.2;
    const auto& m=out.metrics;
    out.motion.qualified=out.responsive && m.measurement_samples>=100 && m.continuous && m.limited_samples==0;
    // Global linear correlation is useful diagnostic context, but Coulomb
    // friction can lower it before tuning. It is neither FRF nor a first-pass
    // rejection gate for a general path
    out.metrics.observable=out.responsive;
    // Dynamic peak/RMS/lag and settling time remain measured diagnostics and
    // tuning evidence. The user's accuracy threshold scores the final fixed
    // target only; it is not an arrival or continuation gate.
    out.passed=(out.stationary_reference || out.motion.qualified) && m.samples>=100 &&
        m.measurement_samples>=100 && m.continuous && m.limited_samples==0 &&
        m.endpoint_window_available && m.endpoint_passed && !m.oscillating;
    out.metrics.passed=out.passed;
    if(m.measurement_samples<100 || !m.continuous)out.reason="insufficient_measurements";
    else if(m.limited_samples)out.reason="limited";
    else if(!m.endpoint_window_available)out.reason=m.endpoint_reason;
    else if(out.stationary_reference)out.reason=out.passed?"stationary_reference_qualified_no_motion_response_claim":m.endpoint_reason;
    else if(!out.reference_coverage)out.reason="trajectory_coverage_insufficient";
    else if(std::max(out.motion.positive_travel_rad,out.motion.negative_travel_rad)<required)out.reason="no_motion";
    else if(!requested_motion_observed)out.reason="requested_direction_response_insufficient";
    else if(!out.responsive)out.reason="weak_response";
    else if(m.oscillating)out.reason="sustained_oscillation";
    else out.reason=out.passed?"final_target_window_qualified":m.endpoint_reason;
    out.motion.reason=out.motion.qualified?(positive_requested && negative_requested?"motion_established":"requested_one_way_motion_established"):out.reason;
    return out;
}

inline bool trajectory_tuning_measurement(const TrajectoryAssessment& a) {
    const auto& m=a.metrics;
    return m.samples>=100 && m.measurement_samples>=100 && m.continuous && m.limited_samples==0 &&
        std::isfinite(m.rms_error) && m.rms_error>=0 && std::isfinite(m.peak_error) && m.peak_error>=0 &&
        std::isfinite(m.tail_error_rad) && m.tail_error_rad>=0 && std::isfinite(m.velocity_rms) && m.velocity_rms>=0 &&
        std::isfinite(a.dynamic_overshoot_rad) && a.dynamic_overshoot_rad>=0;
}

inline bool trajectory_tuning_evidence_valid(const TrajectoryTuningEvidence& t) {
    return t.valid && t.samples>=100 && std::isfinite(t.position_floor_rad) && t.position_floor_rad>0 &&
        std::isfinite(t.bias_rad) && std::isfinite(t.centered_rms_rad) && t.centered_rms_rad>=0 &&
        std::isfinite(t.reference_rms_rad) && t.reference_rms_rad>t.position_floor_rad &&
        std::isfinite(t.in_phase_gain) && std::isfinite(t.lag_s) &&
        std::isfinite(t.amplitude_error_rms_rad) && t.amplitude_error_rms_rad>=0 &&
        std::isfinite(t.velocity_error_rms_rad) && t.velocity_error_rms_rad>=0;
}

// Reference limits report quality; they are not gain multipliers. Candidate
// steps are small experiments within Safety, then compared against measured
// incumbents before any candidate is retained
inline Gains trajectory_candidate(const Gains& current,const std::vector<double>& kp_max,
                                   const std::vector<double>& kd_max,
                                   const std::vector<TrajectoryAssessment>& assessments,const Requirements& req) {
    const auto n=current.kp.size();
    if(current.kd.size()!=n || kp_max.size()!=n || kd_max.size()!=n || assessments.size()!=n)
        throw std::invalid_argument("trajectory gain candidate dimension mismatch");
    Gains next=current;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(current.kp[i]) || current.kp[i]<0 || !std::isfinite(current.kd[i]) || current.kd[i]<0 ||
           !std::isfinite(kp_max[i]) || kp_max[i]<=0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 ||
           current.kp[i]>kp_max[i] || current.kd[i]>kd_max[i])
            throw std::invalid_argument("invalid trajectory gain or safety bound");
        const auto& a=assessments[i];const auto& m=a.metrics;
        if(!trajectory_tuning_measurement(a) || !a.reference_coverage)continue;
        const bool ring=m.oscillating || a.feedback_evidence.uncommanded_reversals;
        // A failed communication path or reversed mapping is not evidence for
        // more actuator stiffness. Low linear correlation alone is not used
        if(!a.responsive && !ring)continue;
        if(a.feedback_evidence.velocity_noise_risk) {
            next.kd[i]=.9*current.kd[i];
            continue;
        }
        if(ring) {
            next.kp[i]=.92*current.kp[i];
            // Neither a reversal nor one overshoot identifies low damping.
            // Change one feedback term per comparison, preserving the actual
            // whole measured controller until its new traversal is assessed.
            continue;
        }
        if(a.feedback_evidence.valid && (a.feedback_evidence.sample_interval_max_s>.02 ||
           (a.feedback_evidence.timing_known && a.feedback_evidence.feedback_age_max_s>.025)))continue;
        const auto& t=a.tuning;
        if(!trajectory_tuning_evidence_valid(t)) {
            // The endpoint acceptance tolerance is not an optimization gate.
            // In a one-way/short record the measured position floor supplies
            // the resolvable residual threshold instead of the user's goal.
            auto local_req=req;local_req.peak_error_rad=2*std::max(.0002,a.feedback_evidence.position_floor_rad);
            const auto local=trajectory_segment_candidate_report({{current.kp[i]},{current.kd[i]}},{kp_max[i]},{kd_max[i]},
                {a.feedback_evidence},local_req);
            next.kp[i]=local.gains.kp[0];next.kd[i]=local.gains.kd[0];continue;
        }
        if(t.in_phase_gain<1 && t.amplitude_error_rms_rad>t.position_floor_rad &&
           t.amplitude_error_rms_rad>t.velocity_error_rms_rad+t.position_floor_rad) {
            const double step=std::clamp(1-t.in_phase_gain,.02,.08);
            // A damping ceiling is a real constraint, not an implicit Kp
            // ceiling. Try a smaller measured stiffness step if Kd is capped.
            const double allowed=1+(current.kd[i]>=kd_max[i]-1e-12?.5:1.0)*step;
            next.kp[i]=std::min(current.kp[i]*allowed,kp_max[i]);
        } else if(t.lag_s>0 && t.velocity_error_rms_rad>t.position_floor_rad &&
                  t.velocity_error_rms_rad>t.amplitude_error_rms_rad+t.position_floor_rad) {
            // MIT already contains a reference-velocity term. Lag alone does
            // not establish excessive physical damping. A small Kp trial can
            // improve disturbance rejection and lag even when Kd is capped;
            // only the next comparable real traversal can justify retention.
            next.kp[i]=std::min(1.08*current.kp[i],kp_max[i]);
        } else if(t.in_phase_gain<1 && t.lag_s>0 && t.amplitude_error_rms_rad>t.position_floor_rad &&
                  t.velocity_error_rms_rad>t.position_floor_rad) {
            // Friction commonly produces both an amplitude deficit and a
            // direction/velocity-correlated residual. Neither must dominate
            // the other to support the same small P-only comparison.
            next.kp[i]=std::min(1.08*current.kp[i],kp_max[i]);
        }
    }
    return next;
}
inline TrajectoryCandidateReport trajectory_candidate_report(const Gains& current,const std::vector<double>& kp_max,
    const std::vector<double>& kd_max,const std::vector<TrajectoryAssessment>& assessments,const Requirements& req) {
    TrajectoryCandidateReport out;out.gains=trajectory_candidate(current,kp_max,kd_max,assessments,req);out.reasons.resize(current.kp.size());
    for(std::size_t i=0;i<current.kp.size();++i) {
        const auto& a=assessments[i];const bool changed=out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i];out.changed=out.changed || changed;
        if(changed && a.feedback_evidence.velocity_noise_risk)out.reasons[i]="measured_path_velocity_feedback_noise_small_damping_only_comparison";
        else if(changed && (a.metrics.oscillating || a.feedback_evidence.uncommanded_reversals))out.reasons[i]="measured_uncommanded_oscillation_reduce_stiffness_preserve_measured_damping_not_inferred_mechanical_resonance";
        else if(changed && a.tuning.velocity_error_rms_rad>a.tuning.amplitude_error_rms_rad+a.tuning.position_floor_rad)
            out.reasons[i]="measured_velocity_related_tracking_residual_bounded_stiffness_comparison_not_a_delay_identification";
        else if(changed && a.tuning.lag_s>0 && a.tuning.amplitude_error_rms_rad>a.tuning.position_floor_rad &&
                a.tuning.velocity_error_rms_rad>a.tuning.position_floor_rad)
            out.reasons[i]="measured_mixed_amplitude_and_velocity_deficit_small_stiffness_only_comparison";
        else if(changed)out.reasons[i]="measured_amplitude_deficit_bounded_stiffness_comparison";
        else if(!trajectory_tuning_measurement(a) || !a.reference_coverage)out.reasons[i]="insufficient_complete_path_measurement";
        else out.reasons[i]="no_supported_local_gain_direction_from_measured_path";
    }
    return out;
}

// One budget-admitted, whole-group same-path poll, not a plant identification
// or a retained result. Every evidence-supported axis may participate; only
// one feedback channel per axis changes. This is a budget-truncated constrained
// direct comparison, not a full positive-basis pattern-search convergence claim
// (Kolda/Lewis/Torczon, SIAM Review 2003, doi:10.1137/S003614450242889).
// Effort admission uses the full measured error envelope, never the short probe
// amplitude or a desired final tracking error. Poll 1 tests lower D separately
// when a lag residual remains; it does not identify D as the cause of that lag.
inline TrajectoryCandidateReport budgeted_trajectory_local_candidate(const Gains& current,
    const std::vector<ModelAxisLimits>& limits,const std::vector<TrajectoryAssessment>& assessments,
    const Requirements& req,std::size_t poll=0,double step_scale=1) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || limits.size()!=n || assessments.size()!=n ||
       !std::isfinite(step_scale) || step_scale<=0 || step_scale>1)
        throw std::invalid_argument("budgeted trajectory candidate mapping or step mismatch");
    std::vector<double> pmax(n),dmax(n);
    for(std::size_t i=0;i<n;++i) {
        const auto& x=limits[i];
        for(double value:{x.kp_max,x.kd_max,x.effort_budget})
            if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid local trajectory gain capability");
        for(double step:{x.grid.kp_step,x.grid.kd_step})
            if(!std::isfinite(step) || step<0)throw std::invalid_argument("invalid local trajectory gain grid");
        pmax[i]=x.kp_max;dmax[i]=x.kd_max;
    }
    auto directions=trajectory_candidate_report(current,pmax,dmax,assessments,req);
    if(poll%2)for(std::size_t i=0;i<n;++i) {
        directions.gains.kp[i]=current.kp[i];directions.gains.kd[i]=current.kd[i];
        const auto& a=assessments[i];const auto& t=a.tuning;const auto& f=a.feedback_evidence;
        if(trajectory_tuning_evidence_valid(t) && t.lag_s>0 &&
           t.velocity_error_rms_rad>t.position_floor_rad && !a.metrics.oscillating && !f.uncommanded_reversals &&
           current.kd[i]>0) {
            directions.gains.kd[i]=.9*current.kd[i];
            directions.reasons[i]="separate_lower_damping_poll_for_resolved_lag_not_identified_damping_cause";
        } else if(f.velocity_noise_risk && current.kd[i]>0) {
            directions.gains.kd[i]=.9*current.kd[i];
            directions.reasons[i]="measured_path_velocity_feedback_noise_small_damping_only_comparison";
        } else directions.reasons[i]="no_resolved_evidence_for_alternative_damping_poll";
    }
    TrajectoryCandidateReport out;out.gains=current;out.reasons=directions.reasons;
    for(std::size_t i=0;i<n;++i) {
        const auto& a=assessments[i];const auto& f=a.feedback_evidence;const auto& x=limits[i];
        auto& reason=out.reasons[i];
        if(!trajectory_tuning_measurement(a) || !a.reference_coverage || !a.responsive ||
           !f.valid || !f.continuous || f.limited || !f.gains_known || !f.gains_constant || f.samples<100 ||
           !std::isfinite(f.peak_error_rad) || f.peak_error_rad<0 ||
           !std::isfinite(f.velocity_error_peak_rad_s) || f.velocity_error_peak_rad_s<0) {
            reason="local_comparison_requires_complete_unlimited_constant_controller_response";continue;
        }
        if(!f.timing_known || !std::isfinite(f.sample_interval_max_s) || f.sample_interval_max_s<=0 ||
           f.sample_interval_max_s>.02 || !std::isfinite(f.feedback_age_max_s) ||
           f.feedback_age_max_s<0 || f.feedback_age_max_s>.025) {
            reason="local_comparison_requires_known_fresh_execution_timing";continue;
        }
        const bool change_p=directions.gains.kp[i]!=current.kp[i];
        const bool change_d=directions.gains.kd[i]!=current.kd[i];
        if(change_p==change_d)continue; // No direction, or a confounded two-term proposal.
        const double before=change_p?current.kp[i]:current.kd[i];
        const double directed=change_p?directions.gains.kp[i]:directions.gains.kd[i];
        const double requested=before+step_scale*(directed-before);
        const double step=change_p?x.grid.kp_step:x.grid.kd_step,ceiling=change_p?x.kp_max:x.kd_max;
        const bool increase=requested>before;
        double value=requested;
        const double E=std::max(a.metrics.peak_error,f.peak_error_rad),V=f.velocity_error_peak_rad_s;
        double resolvable_reduction=0;
        if(change_p && increase) {
            const double floor=std::max({a.tuning.position_floor_rad,f.position_floor_rad,.0002});
            resolvable_reduction=3*floor;
            // Trial design only: with a locally constant residual load,
            // e~tau/P predicts e-e' = e*(1-P/P'). Require a resolvable
            // comparison instead of spending a full traversal on a change
            // below observed resolution. This does not identify that load
            // or promise the improvement on the actual coupled mechanism.
            if(!std::isfinite(floor) || E<=resolvable_reduction ||
               resolvable_reduction/(E-resolvable_reduction)>.5 || before<=0) {
                reason="measured_error_resolution_needs_more_than_half_stiffness_trial_not_identification";continue;
            }
            value=std::max(value,before*(1+resolvable_reduction/(E-resolvable_reduction)));
            value=quantize_gain(value,step,ceiling,true);
        }
        if(step>0) {
            const auto code=gain_grid_code(before,step,ceiling);
            if(!increase && !code) {reason="local_comparison_at_zero_protocol_code";continue;}
            const double adjacent=increase?(code+1)*step:(code-1)*step;
            value=increase?std::max(value,adjacent):std::min(value,adjacent);
        }
        const double coefficient=change_p?E:V;
        const double fixed=change_p?current.kd[i]*V:current.kp[i]*E;
        // A sum of independently observed peaks is a conservative admission
        // envelope; it is not a measurement of the actuator's embedded torque.
        double cap=ceiling;
        if(coefficient>0)cap=std::min(cap,std::max(0.0,(x.effort_budget-fixed)/coefficient));
        else if(fixed>x.effort_budget)cap=0;
        value=quantize_gain(std::min(value,cap),step,ceiling);
        const double candidate_effort=fixed+value*coefficient;
        const bool moved=increase?value>before:value<before;
        const bool code_moved=step==0 || gain_grid_code(value,step,ceiling)!=gain_grid_code(before,step,ceiling);
        // Do not turn an increase into a decrease, or enlarge a reduction
        // merely to reach an effort bound; the next trial remains local.
        const double allowed_reduction=step>0?std::max(before-requested,step)+step:before-requested;
        if(!moved || !code_moved || candidate_effort>x.effort_budget*(1+1e-10) ||
           (!increase && before-value>allowed_reduction+1e-12)) {
            reason="measured_peak_error_velocity_or_protocol_bounds_no_local_code_step";continue;
        }
        if(change_p && increase && (value>1.5*before+1e-12 ||
           E*(1-before/value)<resolvable_reduction*(1-1e-10))) {
            reason="measured_resolution_effort_or_protocol_bounds_prevent_resolvable_half_stiffness_trial";continue;
        }
        reason="budgeted_same_path_whole_group_candidate_comparison_not_identification;"+directions.reasons[i];
        if(change_p && increase)reason+=";quasistatic_residual_assumption_only_resolvable_stiffness_trial_not_a_fit";
        if(change_p)out.gains.kp[i]=value;else out.gains.kd[i]=value;
        out.changed=true;
    }
    return out;
}

struct TrajectoryDampingCandidateReport : TrajectoryCandidateReport {
    std::vector<double> nominal_damping_ratio,critical_damping_free,timing_damping_ceiling,observed_timing_s;
};

// A low bootstrap damping value is an unmeasured entry prior, not a value to
// freeze after P has changed. Resolved repeated residual motion plus a low
// model ratio supports one independent D experiment; neither a tracking lag
// nor one overshoot establishes low damping. P remains exactly unchanged.
inline TrajectoryDampingCandidateReport trajectory_damping_candidate_report(const Gains& current,
    const std::vector<double>& kp_max,const std::vector<double>& kd_max,const std::vector<double>& free_inertia,
    const std::vector<TrajectoryAssessment>& measured,const Requirements&,
    const std::vector<double>& kd_step={}) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || kp_max.size()!=n || kd_max.size()!=n || free_inertia.size()!=n || measured.size()!=n ||
       (!kd_step.empty() && kd_step.size()!=n))throw std::invalid_argument("trajectory damping candidate mapping mismatch");
    TrajectoryDampingCandidateReport out;out.gains=current;out.reasons.resize(n);
    out.nominal_damping_ratio.resize(n);out.critical_damping_free.resize(n);
    out.timing_damping_ceiling.resize(n);out.observed_timing_s.resize(n);
    for(std::size_t i=0;i<n;++i) {
        const double P=current.kp[i],D=current.kd[i],I=free_inertia[i],step=kd_step.empty()?0:kd_step[i];
        if(!std::isfinite(P) || P<0 || !std::isfinite(D) || D<0 || !std::isfinite(I) || I<=0 ||
           !std::isfinite(kp_max[i]) || kp_max[i]<=0 || !std::isfinite(kd_max[i]) || kd_max[i]<=0 ||
           P>kp_max[i] || D>kd_max[i] || !std::isfinite(step) || step<0)
            throw std::invalid_argument("invalid measured damping candidate prior or protocol capability");
        const auto& a=measured[i];const auto& feedback=a.feedback_evidence;const auto& residual=a.residual_motion;
        auto& reason=out.reasons[i];reason="no_resolved_repeated_residual_evidence_for_damping_increase";
        out.critical_damping_free[i]=P>0?2*std::sqrt(P*I):0;
        out.nominal_damping_ratio[i]=P>0?D/out.critical_damping_free[i]:0;
        out.observed_timing_s[i]=feedback.sample_interval_max_s+feedback.feedback_age_max_s;
        if(!trajectory_tuning_measurement(a) || !a.reference_coverage || !feedback.valid ||
           !feedback.gains_known || !feedback.gains_constant) {
            reason="damping_comparison_requires_complete_constant_controller_measurement";continue;
        }
        if(feedback.velocity_noise_risk) {
            out.gains.kd[i]=.9*D;reason="measured_velocity_feedback_noise_small_damping_reduction_only";
        } else if(P<=0 || !a.responsive)reason="damping_increase_requires_measured_restoring_motion_response";
        else if(!feedback.timing_known || feedback.sample_interval_max_s<=0 || feedback.sample_interval_max_s>.02 ||
                feedback.feedback_age_max_s<0 || feedback.feedback_age_max_s>.025)
            reason="damping_increase_requires_known_fresh_execution_timing";
        else {
            // A pure delayed viscous feedback term loses stability at a phase
            // product pi/2. Keep this local design product below .8 using the
            // observed worst command interval plus feedback age and URDF free
            // effective inertia. This bounds an experiment, not a claim about
            // the unknown actuator inner loop or full coupled stability.
            const double timing=out.observed_timing_s[i];
            const double ceiling=std::min(kd_max[i],.8*I/timing);out.timing_damping_ceiling[i]=ceiling;
            const bool repeated=residual.valid && residual.sustained_uncommanded && residual.repeated_windows>=2 &&
                residual.amplitude_rad>2*residual.position_floor_rad && residual.frequency_hz>0 &&
                residual.reference_amplitude_rad<=std::max(.5*residual.position_floor_rad,.2*residual.amplitude_rad);
            if(repeated && out.nominal_damping_ratio[i]<.45) {
                // The increment is at most one local nominal-ratio trial,
                // not a demand to attain critical damping. A tiny bootstrap
                // D still gets a meaningful +.05 modeled-ratio comparison.
                const double candidate=D+std::max({.20*D,.05*out.critical_damping_free[i],step});
                if(ceiling>=D+std::max(step,1e-12)) {
                    out.gains.kd[i]=std::min(candidate,ceiling);
                    reason="repeated_nonreference_residual_and_low_model_damping_small_independent_d_comparison";
                } else reason="measured_timing_or_protocol_bounds_independent_damping_comparison";
            } else if(repeated)reason="repeated_residual_without_low_model_damping_does_not_support_more_damping";
        }
        out.changed=out.changed || out.gains.kd[i]!=D;
    }
    return out;
}

inline bool trajectory_trial_preferred(const Gains& candidate,const std::vector<TrajectoryAssessment>& measured,
                                        const Gains& incumbent,const std::vector<TrajectoryAssessment>& incumbent_measured,
                                        const Gains& seed,const Requirements&) {
    const auto n=seed.kp.size();
    if(!n || seed.kd.size()!=n || candidate.kp.size()!=n || candidate.kd.size()!=n ||
       incumbent.kp.size()!=n || incumbent.kd.size()!=n || measured.size()!=n || incumbent_measured.size()!=n)
        throw std::invalid_argument("trajectory gain comparison dimension mismatch");
    double candidate_square=0,incumbent_square=0,noise_square=0,candidate_cost=0,incumbent_cost=0;
    double candidate_peak_square=0,incumbent_peak_square=0;
    std::size_t candidate_rings=0,incumbent_rings=0;
    for(std::size_t i=0;i<n;++i) {
        for(const double value:{seed.kp[i],seed.kd[i],candidate.kp[i],candidate.kd[i],incumbent.kp[i],incumbent.kd[i]})
            if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid trajectory comparison gain");
        const auto& a=measured[i];const auto& b=incumbent_measured[i];
        if(!trajectory_tuning_measurement(a) || !trajectory_tuning_measurement(b))return false;
        const bool ar=a.metrics.oscillating || a.feedback_evidence.uncommanded_reversals;
        const bool br=b.metrics.oscillating || b.feedback_evidence.uncommanded_reversals;
        if(ar && !br)return false;
        if(a.feedback_evidence.velocity_noise_risk && !b.feedback_evidence.velocity_noise_risk)return false;
        candidate_rings+=ar;incumbent_rings+=br;
        const auto& at=a.tuning;const auto& bt=b.tuning;
        const double acentered=trajectory_tuning_evidence_valid(at)?at.centered_rms_rad:a.metrics.rms_error;
        const double bcentered=trajectory_tuning_evidence_valid(bt)?bt.centered_rms_rad:b.metrics.rms_error;
        const double resolution=std::max({at.position_floor_rad,bt.position_floor_rad,
            a.feedback_evidence.position_floor_rad,b.feedback_evidence.position_floor_rad,.0002});
        const double margin=std::max(resolution,.05*bcentered);
        // One badly worsened axis cannot be hidden by improvements elsewhere.
        // This selects an entire measured gain set, never an unmeasured mix
        if(acentered>bcentered+margin ||
           a.metrics.rms_error>b.metrics.rms_error+std::max(resolution,.05*b.metrics.rms_error) ||
           a.metrics.peak_error>b.metrics.peak_error+std::max(resolution,.03*b.metrics.peak_error) ||
           a.metrics.tail_error_rad>b.metrics.tail_error_rad+std::max(resolution,.05*b.metrics.tail_error_rad))return false;
        candidate_square+=acentered*acentered;
        incumbent_square+=bcentered*bcentered;noise_square+=resolution*resolution;
        candidate_peak_square+=a.metrics.peak_error*a.metrics.peak_error;incumbent_peak_square+=b.metrics.peak_error*b.metrics.peak_error;
        candidate_cost+=candidate.kp[i]/std::max(1e-9,seed.kp[i])+candidate.kd[i]/std::max(1e-9,seed.kd[i]);
        incumbent_cost+=incumbent.kp[i]/std::max(1e-9,seed.kp[i])+incumbent.kd[i]/std::max(1e-9,seed.kd[i]);
    }
    if(candidate_rings<incumbent_rings)return true;
    const double old_peak=std::sqrt(incumbent_peak_square/n),new_peak=std::sqrt(candidate_peak_square/n);
    if(new_peak<old_peak-std::max(std::sqrt(noise_square/n),.03*old_peak))return true;
    const double previous=std::sqrt(incumbent_square/n),current=std::sqrt(candidate_square/n);
    const double margin=std::max(std::sqrt(noise_square/n),.05*previous);
    if(current<previous-margin)return true;
    // Indistinguishable tracking quality favors the less aggressive measured
    // choice, normalized to the authored seed rather than the Safety ceiling
    return std::abs(current-previous)<=margin && candidate_cost<incumbent_cost-1e-9;
}

struct ResidualCorrectionBudget {
    bool fits{false};double remaining_s{0},return_cost_s{0},comparison_cost_s{0},validation_reserve_s{0},exit_reserve_s{0},required_s{0},deficit_s{0};
    std::string reason;
};
// The caller supplies only validation still genuinely required. Reusing an
// existing experiment never extends the original deadline or grants a pass.
inline ResidualCorrectionBudget plan_residual_correction_budget(double remaining_s,double return_cost_s,double comparison_cost_s,
    double remaining_validation_reserve_s,double exit_reserve_s=2) {
    for(double value:{remaining_s,return_cost_s,comparison_cost_s,remaining_validation_reserve_s,exit_reserve_s})
        if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid residual correction cost");
    ResidualCorrectionBudget out;out.remaining_s=remaining_s;out.return_cost_s=return_cost_s;out.comparison_cost_s=comparison_cost_s;
    out.validation_reserve_s=remaining_validation_reserve_s;out.exit_reserve_s=exit_reserve_s;
    out.required_s=return_cost_s+comparison_cost_s+remaining_validation_reserve_s+exit_reserve_s;
    out.deficit_s=std::max(0.0,out.required_s-remaining_s);out.fits=out.deficit_s==0;
    out.reason=out.fits?"comparable_residual_correction_and_remaining_validation_fit_original_budget":
        "insufficient_remaining_budget_for_comparable_residual_correction";
    return out;
}

struct TrajectorySchedule {
    std::size_t native_search_passes{0},compliant_search_passes{0},full_pass_limit{0};
    double native_pass_duration_s{0},compliant_pass_duration_s{0},estimated_total_s{0},deadline_s{0};
    bool fits{false};
};

inline constexpr double recorded_duration_budget_multiplier=12.0;
inline double recorded_duration_budget(double recorded_duration_s) {
    if(!std::isfinite(recorded_duration_s) || recorded_duration_s<=0)
        throw std::invalid_argument("gain calibration needs the original recorded duration");
    return recorded_duration_budget_multiplier*recorded_duration_s;
}

// Compare release, peak and settled speed using the same position evidence.
// A single motor velocity code is retained in diagnostics, not substituted for
// a windowed speed; the Core's independent state-velocity limits still apply
inline double drag_position_speed(const std::vector<Sample>& rows,const NoiseEstimate& noise,double window_s=.6) {
    if(rows.size()<3)return 0;
    const double end=rows.back().time_s;double lo=1e30,hi=-1e30;std::size_t count=0;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=end-window_s) {
        lo=std::min(lo,row.actual);hi=std::max(hi,row.actual);++count;
    }
    if(count<3 || hi-lo<=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,.0002}))return 0;
    return std::max(std::abs(position_window_velocity(rows,noise,window_s)),(hi-lo)/window_s);
}

inline TrajectorySchedule plan_trajectory_budget(double trajectory_duration_s,const std::string& mode,
                                                  double budget_s,double fixed_overhead_s=30,
                                                  double tail_s=1.5,double handover_s=.4,bool admittance_required=false) {
    TrajectorySchedule out;out.deadline_s=budget_s;
    if(!std::isfinite(trajectory_duration_s) || trajectory_duration_s<=0 || !std::isfinite(budget_s) || budget_s<=0 ||
       !std::isfinite(fixed_overhead_s) || fixed_overhead_s<0 || !std::isfinite(tail_s) || tail_s<0 ||
       !std::isfinite(handover_s) || handover_s<0)
        throw std::invalid_argument("invalid trajectory gain calibration budget");
    const bool all=mode=="all",base=mode!="compliant",soft=all || mode=="compliant";
    const bool admittance=admittance_required && (all || mode=="servo_tracking");
    if(!all && mode!="rigid" && mode!="compliant" && mode!="servo_tracking" && mode!="drag")
        throw std::invalid_argument("unknown trajectory gain calibration mode");
    out.native_search_passes=base?1:0;out.compliant_search_passes=soft?1:0;
    out.full_pass_limit=(base?2:0)+(soft?2:0)+(admittance?1:0);
    out.native_pass_duration_s=trajectory_duration_s+tail_s+handover_s;
    out.compliant_pass_duration_s=trajectory_duration_s/.65+tail_s+handover_s;
    // Parallel noise/hold and the two simultaneous all-axis drag-release
    // windows belong to fixed_overhead_s; they are not additional path passes
    out.estimated_total_s=fixed_overhead_s+(base?2:0)*out.native_pass_duration_s+
        (soft?2:0)*out.compliant_pass_duration_s+(admittance?out.native_pass_duration_s:0);
    if(out.estimated_total_s>budget_s+1e-9)return out;
    const auto extra=[&](bool wanted,std::size_t& count,double cost) {
        if(wanted && out.estimated_total_s+cost<=budget_s+1e-9) {
            ++count;++out.full_pass_limit;out.estimated_total_s+=cost;
        }
    };
    extra(base,out.native_search_passes,out.native_pass_duration_s);
    extra(soft,out.compliant_search_passes,out.compliant_pass_duration_s);
    extra(base,out.native_search_passes,out.native_pass_duration_s);
    out.fits=true;
    return out;
}

} // namespace serial_arm::servo_tuning
