#pragma once

#include "serial_arm/dynamics/servo_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

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

struct TrajectoryAssessment {
    Metrics metrics;
    MotionObservation motion;
    TrajectoryTuningEvidence tuning;
    double reference_positive_travel_rad{0},reference_negative_travel_rad{0};
    double steady_error_rad{0},dynamic_overshoot_rad{0};
    std::size_t tail_position_reversals{0};
    bool reference_coverage{false},responsive{false},passed{false};
    std::string reason{"insufficient_measurements"};
};

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
    // A motor velocity code can alternate at rest after a monotone approach.
    // Count reversals only when fresh positions actually move back across the
    // measured encoder/noise floor; keep every tracking-error limit unchanged
    out.tail_position_reversals=tail_position_reversals(rows,excitation_end_s,
        std::max({3*out.motion.encoder_step_rad,6*noise.rms_rad,.0002}));
    out.metrics.oscillating=out.tail_position_reversals>=4 && out.metrics.tail_drift>req.settling_error_rad;
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
    // Raw overshoot remains part of strict acceptance. Only the candidate
    // direction separates a stationary load offset from dynamic overshoot
    const double bias=stationary_bias?out.steady_error_rad:0.0;
    out.dynamic_overshoot_rad=std::max({0.0,actual_high-reference_high-std::max(0.0,bias),
        reference_low-actual_low+std::min(0.0,bias)});
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
    out.reference_coverage=out.reference_positive_travel_rad>=floor && out.reference_negative_travel_rad>=floor;
    const double required=std::max(floor,.2*std::min(out.reference_positive_travel_rad,out.reference_negative_travel_rad));
    out.motion.minimum_motion_span_rad=required;
    out.motion.bidirectional=out.motion.positive_travel_rad>=required && out.motion.negative_travel_rad>=required;
    out.responsive=out.reference_coverage && out.motion.bidirectional && out.metrics.response_rms_ratio>=.2;
    const auto& m=out.metrics;
    out.motion.qualified=out.responsive && m.measurement_samples>=100 && m.continuous && m.limited_samples==0;
    // Global linear correlation is useful diagnostic context, but Coulomb
    // friction can lower it before tuning. It is neither FRF nor a first-pass
    // rejection gate for a general path
    out.metrics.observable=out.responsive;
    out.passed=out.motion.qualified && m.samples>=100 && m.rms_error<=req.rms_error_rad &&
        m.peak_error<=req.peak_error_rad && m.velocity_rms<=req.velocity_rms_rad_s &&
        m.overshoot<=req.overshoot_rad && m.settled && m.settling_s<=req.settling_time_s && !m.oscillating;
    out.metrics.passed=out.passed;
    if(m.measurement_samples<100 || !m.continuous)out.reason="insufficient_measurements";
    else if(m.limited_samples)out.reason="limited";
    else if(!out.reference_coverage)out.reason="trajectory_coverage_insufficient";
    else if(std::max(out.motion.positive_travel_rad,out.motion.negative_travel_rad)<required)out.reason="no_motion";
    else if(!out.motion.bidirectional)out.reason="one_sided_response";
    else if(!out.responsive)out.reason="weak_response";
    else if(m.oscillating || out.dynamic_overshoot_rad>req.overshoot_rad)out.reason="resonance";
    else out.reason=out.passed?"qualified":"tracking_not_qualified";
    out.motion.reason=out.motion.qualified?"motion_established":out.reason;
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
        if(a.passed || !trajectory_tuning_measurement(a) || !a.reference_coverage)continue;
        const bool ring=m.oscillating || a.dynamic_overshoot_rad>req.overshoot_rad;
        // A failed communication path or reversed mapping is not evidence for
        // more actuator stiffness. Low linear correlation alone is not used
        if(!a.responsive && !ring)continue;
        if(ring) {
            next.kp[i]=std::clamp(.9*current.kp[i],0.0,kp_max[i]);
            next.kd[i]=std::clamp(1.1*current.kd[i],0.0,kd_max[i]);
            continue;
        }
        const auto& t=a.tuning;if(!trajectory_tuning_evidence_valid(t))continue;
        if(t.in_phase_gain<1 && t.amplitude_error_rms_rad>t.position_floor_rad &&
           t.amplitude_error_rms_rad>t.velocity_error_rms_rad+t.position_floor_rad) {
            const double step=std::clamp(1-t.in_phase_gain,.02,.10);
            const double damping_ratio=current.kd[i]>0?kd_max[i]/current.kd[i]:1e30;
            const double allowed=std::min(1+step,damping_ratio*damping_ratio);
            if(allowed<=1+1e-12)continue;
            next.kp[i]=std::min(current.kp[i]*allowed,kp_max[i]);
            const double ratio=current.kp[i]>0?next.kp[i]/current.kp[i]:1;
            next.kd[i]=std::min(current.kd[i]*std::sqrt(ratio),kd_max[i]);
        } else if(t.lag_s>0 && t.velocity_error_rms_rad>t.position_floor_rad &&
                  t.velocity_error_rms_rad>t.amplitude_error_rms_rad+t.position_floor_rad &&
                  a.dynamic_overshoot_rad<=t.position_floor_rad) {
            // Velocity-correlated lag does not establish an amplitude deficit.
            // Trial a small damping reduction, never during measured ringing;
            // the next real traversal must justify retaining it
            next.kd[i]=.95*current.kd[i];
        }
    }
    return next;
}

inline bool trajectory_trial_preferred(const Gains& candidate,const std::vector<TrajectoryAssessment>& measured,
                                        const Gains& incumbent,const std::vector<TrajectoryAssessment>& incumbent_measured,
                                        const Gains& seed,const Requirements& req) {
    const auto n=seed.kp.size();
    if(!n || seed.kd.size()!=n || candidate.kp.size()!=n || candidate.kd.size()!=n ||
       incumbent.kp.size()!=n || incumbent.kd.size()!=n || measured.size()!=n || incumbent_measured.size()!=n)
        throw std::invalid_argument("trajectory gain comparison dimension mismatch");
    double candidate_square=0,incumbent_square=0,noise_square=0,candidate_cost=0,incumbent_cost=0;
    std::size_t candidate_rings=0,incumbent_rings=0;
    for(std::size_t i=0;i<n;++i) {
        for(const double value:{seed.kp[i],seed.kd[i],candidate.kp[i],candidate.kd[i],incumbent.kp[i],incumbent.kd[i]})
            if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid trajectory comparison gain");
        const auto& a=measured[i];const auto& b=incumbent_measured[i];
        if(!trajectory_tuning_measurement(a) || !trajectory_tuning_measurement(b))return false;
        const bool ar=a.metrics.oscillating || a.dynamic_overshoot_rad>req.overshoot_rad;
        const bool br=b.metrics.oscillating || b.dynamic_overshoot_rad>req.overshoot_rad;
        if(ar && !br)return false;
        candidate_rings+=ar;incumbent_rings+=br;
        const auto& at=a.tuning;const auto& bt=b.tuning;
        if(!trajectory_tuning_evidence_valid(at) || !trajectory_tuning_evidence_valid(bt))return false;
        const double resolution=std::max(at.position_floor_rad,bt.position_floor_rad);
        const double margin=std::max(resolution,.05*bt.centered_rms_rad);
        // One badly worsened axis cannot be hidden by improvements elsewhere.
        // This selects an entire measured gain set, never an unmeasured mix
        if(at.centered_rms_rad>bt.centered_rms_rad+margin ||
           a.metrics.rms_error>b.metrics.rms_error+std::max(resolution,.05*b.metrics.rms_error) ||
           a.dynamic_overshoot_rad>b.dynamic_overshoot_rad+resolution)return false;
        candidate_square+=at.centered_rms_rad*at.centered_rms_rad;
        incumbent_square+=bt.centered_rms_rad*bt.centered_rms_rad;noise_square+=resolution*resolution;
        candidate_cost+=candidate.kp[i]/std::max(1e-9,seed.kp[i])+candidate.kd[i]/std::max(1e-9,seed.kd[i]);
        incumbent_cost+=incumbent.kp[i]/std::max(1e-9,seed.kp[i])+incumbent.kd[i]/std::max(1e-9,seed.kd[i]);
    }
    if(candidate_rings<incumbent_rings)return true;
    const double previous=std::sqrt(incumbent_square/n),current=std::sqrt(candidate_square/n);
    const double margin=std::max(std::sqrt(noise_square/n),.05*previous);
    if(current<previous-margin)return true;
    // Indistinguishable tracking quality favors the less aggressive measured
    // choice, normalized to the authored seed rather than the Safety ceiling
    return std::abs(current-previous)<=margin && candidate_cost<incumbent_cost-1e-9;
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
