#pragma once

#include "serial_arm/dynamics/gain_candidate_admission.hpp"
#include <optional>

namespace serial_arm::servo_tuning {

// Bounded, opportunistic coordinate polling on an actually repeated taught
// segment. Method background: Kolda/Lewis/Torczon, SIAM Review 45 (2003),
// https://doi.org/10.1137/S003614450242889. The empirical comparison below
// does not claim an identified plant, convergence, or hardware stability.
struct TaughtLocalOptions {
    double segment_s{2},minimum_segment_s{.3},tail_s{.5},maximum_relative_step{.12};
    std::optional<std::size_t> nearby_source_index;
    double maximum_source_distance_s{0};
};
struct TaughtLocalPlan {
    bool valid{false},reverse{false};std::size_t start_index{0},end_index{0};
    double duration_s{0},measurement_cost_s{0},comparison_cost_s{0},score{0};
    std::size_t focus_axis{0};double focus_reference_span_rad{0};
    std::string reason{"taught_local_segment_not_planned"};
};
struct TaughtLocalConditions {
    std::size_t start_index{0},end_index{0};bool reverse{false};
    double playback_rate{1},source_dt_s{.01},tail_s{.5};
    std::string feedforward_scope,load_scope;
};
struct TaughtLocalAxisAssessment {
    bool complete{false},constant_gains{false},limited{false},static_measured{false},sustained_oscillation{false};
    std::size_t fresh_samples{0},residual_reversals{0},reference_reversals{0};
    double rms_error_rad{0},peak_error_rad{0},static_bias_rad{0},static_error_rad{0};
    double same_direction_lag_s{0},same_direction_error_rad{0},dynamic_overshoot_rad{0};
    double raw_directional_excess_rad{0};
    double velocity_error_peak_rad_s{0},motion_floor_rad{0},comparison_uncertainty_rad{0},reference_span_rad{0},actual_span_rad{0};
    double measured_kp{0},measured_kd{0},last_error_rad{0},last_velocity_error_rad_s{0};
    double initial_reference_rad{0},initial_actual_rad{0},initial_error_rad{0};
    double initial_position_span_rad{0},initial_speed_rad_s{0};
};
struct TaughtLocalAssessment {
    bool complete{false},model_identified{false};Gains gains;TaughtLocalConditions conditions;
    std::vector<TaughtLocalAxisAssessment> axes;
    std::vector<std::vector<Sample>> rows;
    std::string reason{"taught_local_not_measured"};
};
struct TaughtLocalCandidate {
    Gains gains;bool changed{false},model_identified{false};std::size_t axis{0};
    std::string coordinate,reason{"taught_local_no_supported_poll"};std::vector<std::string> reasons;
};
struct TaughtLocalComparisonReport {
    bool preferred{false};
    std::string reason{"taught_local_comparison_not_measured"};
    bool retry_recommended{false};
};

namespace taught_local_detail {
inline bool same(double a,double b){return std::isfinite(a) && std::isfinite(b) && std::abs(a-b)<=1e-9*std::max({1.0,std::abs(a),std::abs(b)});}
inline double source_time(const Sample& s){return s.reference_sample_time_known?s.reference_sample_time_s:s.time_s;}
inline double duration(const TaughtLocalConditions& c){return std::abs(static_cast<double>(c.end_index)-static_cast<double>(c.start_index))*c.source_dt_s/c.playback_rate;}
inline bool valid(const TaughtLocalConditions& c) {
    return c.start_index!=c.end_index && c.reverse==(c.start_index>c.end_index) &&
        std::isfinite(c.playback_rate) && c.playback_rate>0 && std::isfinite(c.source_dt_s) && c.source_dt_s>0 &&
        std::isfinite(c.tail_s) && c.tail_s>=.25 && !c.feedforward_scope.empty() && !c.load_scope.empty();
}
inline bool same(const TaughtLocalConditions& a,const TaughtLocalConditions& b) {
    return valid(a) && valid(b) && a.start_index==b.start_index && a.end_index==b.end_index && a.reverse==b.reverse &&
        same(a.playback_rate,b.playback_rate) && same(a.source_dt_s,b.source_dt_s) && same(a.tail_s,b.tail_s) &&
        a.feedforward_scope==b.feedforward_scope && a.load_scope==b.load_scope;
}
inline bool at(const std::vector<Sample>& rows,double t,double& r,double& v,double& velocity_span) {
    if(rows.empty())return false;
    const auto hi=std::lower_bound(rows.begin(),rows.end(),t,[](const Sample& s,double time){return source_time(s)<time;});
    // Compare only measured clock coverage. Clamping two different first
    // samples to t=0 manufactures a velocity mismatch on a curved reference.
    if(hi==rows.end())return false;
    if(hi==rows.begin() || source_time(*hi)==t) {
        if(t<source_time(*hi))return false;
        r=hi->reference;v=hi->reference_velocity;velocity_span=0;return true;
    }
    const auto lo=hi-1;const double dt=source_time(*hi)-source_time(*lo);if(dt<=0 || dt>.06)return false;
    const double a=std::clamp((t-source_time(*lo))/dt,0.0,1.0);
    r=lo->reference+a*(hi->reference-lo->reference);v=lo->reference_velocity+a*(hi->reference_velocity-lo->reference_velocity);
    velocity_span=std::abs(hi->reference_velocity-lo->reference_velocity);return true;
}
inline bool initial_error_at(const std::vector<Sample>& rows,double t,double& error) {
    if(rows.empty())return false;
    const auto hi=std::lower_bound(rows.begin(),rows.end(),t,[](const Sample& s,double time){return source_time(s)<time;});
    if(hi==rows.end() || t<source_time(rows.front()))return false;
    if(hi==rows.begin() || source_time(*hi)==t){error=hi->reference-hi->actual;return true;}
    const auto lo=hi-1;const double dt=source_time(*hi)-source_time(*lo);if(dt<=0 || dt>.06)return false;
    const double u=(t-source_time(*lo))/dt;
    error=(1-u)*(lo->reference-lo->actual)+u*(hi->reference-hi->actual);return true;
}
}

inline TaughtLocalPlan plan_taught_local_correction(const std::vector<std::vector<Sample>>& rows,
    const std::vector<NoiseEstimate>& noise,std::size_t source_samples,double source_dt_s,bool reverse,
    double playback_rate,double available_s,const TaughtLocalOptions& options={}) {
    TaughtLocalPlan out;out.reverse=reverse;
    if(rows.empty() || rows.size()!=noise.size() || source_samples<4 || !std::isfinite(source_dt_s) || source_dt_s<=0 ||
       !std::isfinite(playback_rate) || playback_rate<=0 || !std::isfinite(available_s) || available_s<=0 ||
       !std::isfinite(options.segment_s) || options.segment_s<=0 || !std::isfinite(options.minimum_segment_s) || options.minimum_segment_s<=0 ||
       !std::isfinite(options.tail_s) || options.tail_s<.25){out.reason="taught_local_source_noise_or_budget_missing";return out;}
    const double full=(source_samples-1)*source_dt_s/playback_rate;
    const double span=std::min({options.segment_s,.4*full,(available_s-2*options.tail_s-1.2)/3});
    if(span<options.minimum_segment_s){out.reason="taught_local_complete_comparison_does_not_fit_budget";return out;}
    const auto width=std::min(source_samples-2,std::max<std::size_t>(2,static_cast<std::size_t>(std::floor(span*playback_rate/source_dt_s))));
    const auto n=rows.size();
    std::vector<double> floors(n);
    std::vector<std::vector<double>> energy(n,std::vector<double>(source_samples,0)),count=energy;
    std::vector<std::vector<double>> ref_low(n,std::vector<double>(source_samples,1e30)),ref_high(n,std::vector<double>(source_samples,-1e30));
    std::vector<double> total_low(n,1e30),total_high(n,-1e30);
    for(std::size_t axis=0;axis<rows.size();++axis) {
        if(!std::isfinite(noise[axis].encoder_step_rad) || noise[axis].encoder_step_rad<0 ||
           !std::isfinite(noise[axis].rms_rad) || noise[axis].rms_rad<0){out.reason="taught_local_noise_evidence_invalid";return out;}
        const double floor=std::max({3*noise[axis].encoder_step_rad,6*noise[axis].rms_rad,.0002});
        floors[axis]=floor;
        std::uint64_t sequence=0;
        for(const auto& r:rows[axis])if(r.new_measurement && r.actual_gains_known && std::isfinite(r.actual) && std::isfinite(r.reference) &&
            std::isfinite(r.actual_kp) && r.actual_kp>=0 && std::isfinite(r.actual_kd) && r.actual_kd>=0) {
            if(r.feedback_sequence && r.feedback_sequence==sequence)continue;
            sequence=r.feedback_sequence;
            const double time=taught_local_detail::source_time(r);if(!std::isfinite(time) || time<0 || time>full)continue;
            const auto index=std::min(source_samples-1,static_cast<std::size_t>(std::floor(time*playback_rate/source_dt_s)));
            const double error=(r.reference-r.actual)/floor;energy[axis][index]+=error*error;++count[axis][index];
            ref_low[axis][index]=std::min(ref_low[axis][index],r.reference);
            ref_high[axis][index]=std::max(ref_high[axis][index],r.reference);
            total_low[axis]=std::min(total_low[axis],r.reference);total_high[axis]=std::max(total_high[axis],r.reference);
        }
    }
    std::vector<std::vector<double>> sums(n,std::vector<double>(source_samples+1,0)),counts=sums;
    for(std::size_t a=0;a<n;++a)for(std::size_t i=0;i<source_samples;++i){
        sums[a][i+1]=sums[a][i]+energy[a][i];counts[a][i+1]=counts[a][i]+count[a][i];}
    std::size_t best=0,best_axis=0,fallback=0,fallback_axis=0;
    double best_score=-1,best_span=0,fallback_score=-1;
    for(std::size_t i=0;i+width<source_samples;++i) {
        if(options.nearby_source_index && options.maximum_source_distance_s>0) {
            const auto from=reverse?source_samples-1-i:i,to=reverse?source_samples-1-i-width:i+width;
            if(std::abs(static_cast<double>(from)-*options.nearby_source_index)*source_dt_s>options.maximum_source_distance_s+1e-10 ||
               std::abs(static_cast<double>(to)-*options.nearby_source_index)*source_dt_s>options.maximum_source_distance_s+1e-10)continue;
        }
        for(std::size_t a=0;a<n;++a) {
            const double samples=counts[a][i+width+1]-counts[a][i];if(samples<10)continue;
            const double score=(sums[a][i+width+1]-sums[a][i])/samples;
            if(score>fallback_score){fallback_score=score;fallback=i;fallback_axis=a;}
            double lo=1e30,hi=-1e30;
            for(std::size_t k=i;k<=i+width;++k){lo=std::min(lo,ref_low[a][k]);hi=std::max(hi,ref_high[a][k]);}
            // A large static error on an unchanging taught plateau used to
            // win every local search, even though the segment could not show
            // whether greater P improved movement. Prefer resolved reference
            // travel of the affected axis; retain pure hold support when no
            // moving source segment exists at all.
            const double span=hi-lo;if(span<=3*floors[a])continue;
            const double activity=span/std::max(span,total_high[a]-total_low[a]);
            const double informative=score*std::min(1.0,span/(8*floors[a]))*activity*activity;
            if(informative>best_score){best_score=informative;best=i;best_axis=a;best_span=span;}
        }
    }
    if(best_score<0 && fallback_score>=0){best=fallback;best_axis=fallback_axis;best_score=fallback_score;}
    if(best_score<0){out.reason="taught_local_fresh_tracking_evidence_missing";return out;}
    if(best_score<=1){out.reason="taught_local_no_tracking_problem_resolved_above_measurement_floor";return out;}
    out.start_index=reverse?source_samples-1-best:best;
    out.end_index=reverse?source_samples-1-best-width:best+width;
    out.duration_s=width*source_dt_s/playback_rate;
    // Entry from the caller's current source position is not included. The
    // comparison includes one same-source return and both measured tails.
    out.measurement_cost_s=out.duration_s+options.tail_s+.6;
    out.comparison_cost_s=3*out.duration_s+2*options.tail_s+1.2;
    out.score=best_score;out.focus_axis=best_axis;out.focus_reference_span_rad=best_span;
    out.valid=true;out.reason="observed_taught_error_selected_short_same_direction_segment";return out;
}

inline TaughtLocalAssessment assess_taught_local_segment(const std::vector<std::vector<Sample>>& rows,
    const std::vector<NoiseEstimate>& noise,const Gains& gains,const TaughtLocalConditions& conditions) {
    using namespace taught_local_detail;TaughtLocalAssessment out;out.gains=gains;out.conditions=conditions;
    const auto n=gains.kp.size();out.axes.resize(n);out.rows.resize(n);
    if(!n || gains.kd.size()!=n || rows.size()!=n || noise.size()!=n || !valid(conditions)) {
        out.reason="taught_local_mapping_or_execution_conditions_missing";return out;
    }
    out.complete=true;const double motion_end=duration(conditions),end=motion_end+conditions.tail_s;
    for(std::size_t axis=0;axis<n;++axis) {
        if(!std::isfinite(noise[axis].encoder_step_rad) || noise[axis].encoder_step_rad<0 ||
           !std::isfinite(noise[axis].rms_rad) || noise[axis].rms_rad<0){out.complete=false;out.reason="taught_local_noise_evidence_invalid";return out;}
        auto& a=out.axes[axis];a.measured_kp=gains.kp[axis];a.measured_kd=gains.kd[axis];a.constant_gains=true;
        a.motion_floor_rad=std::max({3*noise[axis].encoder_step_rad,6*noise[axis].rms_rad,.0002});
        // Quantization uncertainty does not average away at a stuck encoder
        // code. Noise averaging is limited to 0.1-s blocks, not UI samples.
        a.comparison_uncertainty_rad=std::max({noise[axis].encoder_step_rad,
            2*noise[axis].rms_rad/std::sqrt(std::max(1.0,end/.1)),1e-6});
        bool continuous=true,inputs=true,timing=true;std::uint64_t sequence=0;
        double qlo=1e30,qhi=-1e30,actual_lo=1e30,actual_hi=-1e30,errlo=1e30,errhi=-1e30,square=0,lag_ev=0,lag_vv=0,along=0,static_sum=0;
        std::size_t moving=0,statics=0;int last_error_sign=0,last_reference_sign=0;
        double direction_origin_reference=0,direction_origin_actual=0,minimum_directional_excess=0;
        const double speed_floor=a.motion_floor_rad/.25;
        for(const auto& r:rows[axis]) {
            a.limited=a.limited || r.limited;
            inputs=inputs && r.independent_disturbance_known && std::isfinite(r.independent_disturbance_nm) &&
                std::abs(r.independent_disturbance_nm)<=1e-10 && r.feedforward_known && std::isfinite(r.applied_feedforward_nm);
            a.constant_gains=a.constant_gains && r.actual_gains_known && same(r.actual_kp,gains.kp[axis]) && same(r.actual_kd,gains.kd[axis]);
            timing=timing && std::isfinite(r.command_time_s) && std::isfinite(r.feedback_time_s) &&
                r.command_time_s-r.feedback_time_s<=.06 && r.feedback_time_s<=r.command_time_s+.005;
            if(r.reference_sample_time_known)timing=timing && std::isfinite(r.reference_sample_time_s) &&
                r.reference_sample_time_s<=r.command_time_s+.005 && r.command_time_s-r.reference_sample_time_s<=.06;
            if(!r.new_measurement || (r.feedback_sequence && r.feedback_sequence==sequence))continue;
            sequence=r.feedback_sequence;
            if(!std::isfinite(r.time_s) || !std::isfinite(r.actual) || !std::isfinite(r.velocity) ||
               !std::isfinite(r.reference) || !std::isfinite(r.reference_velocity)){continuous=false;continue;}
            auto& kept=out.rows[axis];if(!kept.empty() && (r.time_s<=kept.back().time_s || r.time_s-kept.back().time_s>.06 ||
                source_time(r)<source_time(kept.back()) || r.command_time_s<kept.back().command_time_s ||
                r.feedback_time_s<=kept.back().feedback_time_s))continuous=false;
            kept.push_back(r);++a.fresh_samples;const double error=r.reference-r.actual;
            if(a.fresh_samples==1) {
                a.initial_reference_rad=r.reference;a.initial_actual_rad=r.actual;a.initial_error_rad=error;
            }
            if(a.fresh_samples<=3) {
                a.initial_position_span_rad=std::max(a.initial_position_span_rad,std::abs(r.actual-a.initial_actual_rad));
                const double dt=r.time_s-kept.front().time_s;
                if(dt>0)a.initial_speed_rad_s=(r.actual-a.initial_actual_rad)/dt;
            }
            square+=error*error;a.peak_error_rad=std::max(a.peak_error_rad,std::abs(error));
            a.velocity_error_peak_rad_s=std::max(a.velocity_error_peak_rad_s,std::abs(r.reference_velocity-r.velocity));
            a.last_error_rad=error;a.last_velocity_error_rad_s=r.reference_velocity-r.velocity;
            qlo=std::min(qlo,r.reference);qhi=std::max(qhi,r.reference);errlo=std::min(errlo,error);errhi=std::max(errhi,error);
            actual_lo=std::min(actual_lo,r.actual);actual_hi=std::max(actual_hi,r.actual);
            if(std::abs(r.reference_velocity)>speed_floor) {
                const int sign=r.reference_velocity>0?1:-1;
                if(last_reference_sign && sign!=last_reference_sign)++a.reference_reversals;
                const double excess=std::max(0.0,-sign*error);
                a.raw_directional_excess_rad=std::max(a.raw_directional_excess_rad,excess);
                if(sign!=last_reference_sign) {
                    direction_origin_reference=r.reference;direction_origin_actual=r.actual;
                    minimum_directional_excess=excess;
                }
                minimum_directional_excess=std::min(minimum_directional_excess,excess);
                // Existing entry bias is still present in the raw peak and
                // final static error. Call only newly developed motion past
                // the moving reference an overshoot, after both coordinates
                // have actually moved in this reference-direction run.
                if(sign*(r.reference-direction_origin_reference)>a.motion_floor_rad &&
                   sign*(r.actual-direction_origin_actual)>a.motion_floor_rad)
                    a.dynamic_overshoot_rad=std::max(a.dynamic_overshoot_rad,excess-minimum_directional_excess);
                last_reference_sign=sign;
                lag_ev+=error*r.reference_velocity;lag_vv+=r.reference_velocity*r.reference_velocity;
                along+=std::max(0.0,sign*error);++moving;
            } else if(source_time(r)>=motion_end || std::abs(r.reference_velocity)<=.25*speed_floor) {
                static_sum+=error;++statics;
            }
            const int sign=error>a.motion_floor_rad?1:error<-a.motion_floor_rad?-1:0;
            if(sign){if(last_error_sign && sign!=last_error_sign)++a.residual_reversals;last_error_sign=sign;}
        }
        a.reference_span_rad=qhi-qlo;a.actual_span_rad=actual_hi-actual_lo;
        a.rms_error_rad=a.fresh_samples?std::sqrt(square/a.fresh_samples):0;
        a.same_direction_lag_s=lag_vv>0?std::max(0.0,lag_ev/lag_vv):0;
        a.same_direction_error_rad=moving?along/moving:0;
        a.static_measured=statics>=10;a.static_bias_rad=statics?static_sum/statics:0;a.static_error_rad=std::abs(a.static_bias_rad);
        a.sustained_oscillation=a.residual_reversals>=4 && a.residual_reversals>a.reference_reversals+2 && errhi-errlo>4*a.motion_floor_rad;
        const auto& kept=out.rows[axis];a.complete=continuous && inputs && timing && a.constant_gains && a.fresh_samples>=30 &&
            !kept.empty() && source_time(kept.front())<=.03 && source_time(kept.back())>=end-.025;
        out.complete=out.complete && a.complete;
    }
    out.reason=out.complete?"measured_same_taught_segment_without_dynamic_model_identification":"taught_local_measurement_incomplete";return out;
}

inline TaughtLocalCandidate taught_local_candidate(const Gains& current,const std::vector<ModelAxisLimits>& limits,
    const std::vector<NoiseEstimate>& noise,const TaughtLocalAssessment& baseline,
    const std::vector<bool>& model_supported={},std::size_t poll_index=0,const TaughtLocalOptions& options={}) {
    using namespace taught_local_detail;TaughtLocalCandidate out;out.gains=current;const auto n=current.kp.size();out.reasons.assign(n,"taught_local_axis_unchanged");
    if(!n || current.kd.size()!=n || limits.size()!=n || noise.size()!=n || baseline.axes.size()!=n || !baseline.complete ||
       (!model_supported.empty() && model_supported.size()!=n) || !std::isfinite(options.maximum_relative_step) ||
       options.maximum_relative_step<=0 || options.maximum_relative_step>.12) {out.reason="taught_local_complete_baseline_or_mapping_missing";return out;}
    double worst=-1;std::size_t selected=n;
    for(std::size_t i=0;i<n;++i) {
        const auto& a=baseline.axes[i];if((!model_supported.empty() && model_supported[i]) || !a.complete || a.limited ||
            !same(a.measured_kp,current.kp[i]) || !same(a.measured_kd,current.kd[i]) || current.kp[i]<=0)continue;
        const double problem=std::max({a.rms_error_rad,a.static_measured?a.static_error_rad:0,a.same_direction_error_rad,a.dynamic_overshoot_rad})/
            std::max(a.motion_floor_rad,1e-6)+(a.sustained_oscillation?4:0);
        if(problem>worst && problem>1){worst=problem;selected=i;}
    }
    if(selected==n){out.reason="taught_local_no_unresolved_axis_with_resolved_tracking_error";return out;}
    const auto& a=baseline.axes[selected];const auto& bound=limits[selected];out.axis=selected;
    // Poll directions are empirical trials; no sign is interpreted as a
    // mechanical cause or as a successful fit. Each comparison changes one
    // coordinate of an otherwise unchanged complete PD group.
    const bool ringing=a.sustained_oscillation;
    const bool overshoot_only=!ringing && a.static_error_rad<=a.motion_floor_rad &&
        a.dynamic_overshoot_rad>a.same_direction_error_rad+a.motion_floor_rad;
    const bool unresolved_static=!ringing && a.static_measured && a.static_error_rad>3*a.motion_floor_rad &&
        a.reference_span_rad<=3*a.motion_floor_rad && a.actual_span_rad<=3*a.motion_floor_rad;
    const std::array<std::pair<bool,int>,4> ordinary{{{true,1},{false,-1},{false,1},{true,-1}}};
    const std::array<std::pair<bool,int>,4> overshoot{{{true,-1},{false,1},{false,-1},{true,1}}};
    const std::array<std::pair<bool,int>,4> transient{{{false,1},{false,-1},{true,1},{true,-1}}};
    const auto direction=unresolved_static?std::pair<bool,int>{true,1}:
        (ringing?overshoot:overshoot_only?transient:ordinary)[poll_index%4];
    const bool P=direction.first;const int sign=direction.second;
    const double old=P?current.kp[selected]:current.kd[selected],grid=P?bound.grid.kp_step:bound.grid.kd_step,maximum=P?bound.kp_max:bound.kd_max;
    if(!std::isfinite(old) || !std::isfinite(grid) || grid<0 || !std::isfinite(maximum) || maximum<=0){out.reason="taught_local_protocol_mapping_invalid";return out;}
    // A second unresolved static experiment enlarges this local P poll only;
    // it does not switch to Kd, which cannot remove zero-velocity bias. The
    // larger trial still needs the same complete-group effort admission and
    // measured comparison, and never becomes an adopted gain by inference.
    const double relative=unresolved_static?std::min(.25,options.maximum_relative_step*(1+std::min<std::size_t>(poll_index,1))):options.maximum_relative_step;
    const double delta=std::max(relative*old,grid);
    double next=quantize_gain(std::clamp(old+sign*delta,0.0,maximum),grid,maximum);
    if(P && next<=0){out.reason="taught_local_no_lower_positive_restoring_code";return out;}
    if(same(next,old)){out.reason="taught_local_poll_coordinate_at_protocol_boundary";return out;}
    if(std::abs(next-old)>delta+1e-10)next=quantize_gain(std::clamp(old+sign*grid,0.0,maximum),grid,maximum);
    if(P)out.gains.kp[selected]=next;else out.gains.kd[selected]=next;
    std::vector<double> E(n),V(n),freshE(n),freshV(n);
    for(std::size_t i=0;i<n;++i){E[i]=baseline.axes[i].peak_error_rad;V[i]=baseline.axes[i].velocity_error_peak_rad_s;
        freshE[i]=baseline.axes[i].last_error_rad;freshV[i]=baseline.axes[i].last_velocity_error_rad_s;}
    const auto admitted=admit_complete_gain_candidate(current,out.gains,limits,E,V,freshE,freshV);
    if(!admitted.allowed){out.gains=current;out.reason="taught_local_complete_poll_group_outside_existing_effort_or_protocol_domain";out.reasons=admitted.axis_reasons;return out;}
    out.changed=true;out.coordinate=P?"Kp":"Kd";out.reason=ringing?"empirical_bounded_coordinate_poll_for_measured_sustained_residual_oscillation":overshoot_only?
        "empirical_bounded_coordinate_poll_for_dynamic_overshoot_without_resonance_claim":
        "empirical_bounded_coordinate_poll_for_measured_static_bias_or_same_direction_lag";
    out.reasons[selected]=out.reason;return out;
}

inline TaughtLocalComparisonReport taught_local_comparison_report(const TaughtLocalAssessment& candidate,const TaughtLocalAssessment& baseline,
    const Gains& candidate_gains,const Gains& baseline_gains) {
    using namespace taught_local_detail;const auto n=baseline.axes.size();
    if(!n || !candidate.complete || !baseline.complete || candidate.axes.size()!=n ||
       candidate.rows.size()!=n || baseline.rows.size()!=n ||
       candidate_gains.kp.size()!=n || candidate_gains.kd.size()!=n || baseline_gains.kp.size()!=n || baseline_gains.kd.size()!=n ||
       !same(candidate.conditions,baseline.conditions))return {false,"taught_local_complete_same_condition_groups_required"};
    bool improved=false;std::size_t changed=0;
    for(std::size_t i=0;i<n;++i)if(candidate.axes[i].sustained_oscillation && !baseline.axes[i].sustained_oscillation)
        return {false,"taught_local_new_sustained_residual_oscillation"};
    for(std::size_t i=0;i<n;++i) {
        const auto& a=candidate.axes[i];const auto& b=baseline.axes[i];
        if(!a.complete || !b.complete || a.limited || b.limited || !a.constant_gains || !b.constant_gains ||
           !same(a.measured_kp,candidate_gains.kp[i]) || !same(a.measured_kd,candidate_gains.kd[i]) ||
           !same(b.measured_kp,baseline_gains.kp[i]) || !same(b.measured_kd,baseline_gains.kd[i]) ||
           (b.static_measured && !a.static_measured))return {false,"taught_local_actual_fixed_group_or_measurement_scope_invalid"};
        changed+=!same(candidate_gains.kp[i],baseline_gains.kp[i]);changed+=!same(candidate_gains.kd[i],baseline_gains.kd[i]);
        const double uncertainty=std::max(a.comparison_uncertainty_rad,b.comparison_uncertainty_rad);
        const double length=duration(baseline.conditions)+baseline.conditions.tail_s;
        if(candidate.rows[i].empty() || baseline.rows[i].empty())return {false,"taught_local_common_reference_clock_coverage_missing"};
        const double first=std::max(source_time(candidate.rows[i].front()),source_time(baseline.rows[i].front()));
        const double last=std::min({length,source_time(candidate.rows[i].back()),source_time(baseline.rows[i].back())});
        if(first>.03 || last<length-.025 || last-first<.25)return {false,"taught_local_common_reference_clock_coverage_missing"};
        double a_initial_error,b_initial_error;
        if(!initial_error_at(candidate.rows[i],first,a_initial_error) || !initial_error_at(baseline.rows[i],first,b_initial_error))
            return {false,"taught_local_common_initial_state_missing"};
        const double initial_error_difference=std::abs(a_initial_error-b_initial_error);
        const double initial_state_limit=std::max(a.motion_floor_rad,b.motion_floor_rad)+uncertainty;
        for(double t=first;t<=last;t+=.02) {
            double ar,av,aspan,br,bv,bspan;
            if(!at(candidate.rows[i],t,ar,av,aspan) || !at(baseline.rows[i],t,br,bv,bspan))
                return {false,"taught_local_common_reference_clock_gap"};
            if(std::abs(ar-br)>std::max(a.motion_floor_rad,b.motion_floor_rad))
                return {false,"taught_local_actual_reference_position_not_comparable"};
            // Piecewise-linear reconstruction of a curved velocity has an
            // uncertainty set by the actual bracketing commands. This is
            // zero on a plateau; a genuinely different speed still fails.
            const double interpolation_uncertainty=.5*(aspan+bspan);
            if(std::abs(av-bv)>.02*std::max({std::abs(av),std::abs(bv),.01})+
                uncertainty/std::max(.25,length)+interpolation_uncertainty)
                return {false,"taught_local_actual_reference_velocity_not_comparable"};
        }
        if(initial_error_difference>initial_state_limit)
            return {false,"taught_local_initial_state_not_comparable_retry_same_segment",true};
        const bool raw_worsened=a.rms_error_rad>b.rms_error_rad+uncertainty || a.peak_error_rad>b.peak_error_rad+2*uncertainty ||
            (a.static_measured && b.static_measured && a.static_error_rad>b.static_error_rad+uncertainty);
        const bool dynamic_worsened=a.dynamic_overshoot_rad>b.dynamic_overshoot_rad+2*uncertainty;
        const bool unchanged=same(candidate_gains.kp[i],baseline_gains.kp[i]) && same(candidate_gains.kd[i],baseline_gains.kd[i]);
        // A resolved entry offset on an untouched axis can explain a small
        // raw-error shift, but it cannot be subtracted to make the new group
        // pass. Request a matched repeat; keep all original error values.
        if(unchanged && raw_worsened && !dynamic_worsened && initial_error_difference>uncertainty &&
           a.rms_error_rad<=b.rms_error_rad+initial_error_difference+uncertainty &&
           a.peak_error_rad<=b.peak_error_rad+initial_error_difference+2*uncertainty &&
           (!a.static_measured || !b.static_measured || a.static_error_rad<=b.static_error_rad+initial_error_difference+uncertainty))
            return {false,"taught_local_unchanged_axis_entry_bias_requires_matched_repeat",true};
        if(raw_worsened || dynamic_worsened)
            return {false,"taught_local_measured_complete_group_response_worsened"};
        improved=improved || a.rms_error_rad<b.rms_error_rad-uncertainty ||
            (a.static_measured && b.static_measured && a.static_error_rad<b.static_error_rad-uncertainty) ||
            a.same_direction_error_rad<b.same_direction_error_rad-uncertainty ||
            a.dynamic_overshoot_rad<b.dynamic_overshoot_rad-2*uncertainty || (b.sustained_oscillation && !a.sustained_oscillation);
    }
    if(changed!=1)return {false,"taught_local_requires_one_actually_measured_coordinate_change"};
    return improved?TaughtLocalComparisonReport{true,"taught_local_measured_same_segment_complete_group_improved"}:
        TaughtLocalComparisonReport{false,"taught_local_no_improvement_resolved_above_measurement_uncertainty"};
}

inline bool taught_local_preferred(const TaughtLocalAssessment& candidate,const TaughtLocalAssessment& baseline,
    const Gains& candidate_gains,const Gains& baseline_gains) {
    return taught_local_comparison_report(candidate,baseline,candidate_gains,baseline_gains).preferred;
}

} // namespace serial_arm::servo_tuning
