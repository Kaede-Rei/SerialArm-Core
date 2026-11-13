#include "serial_arm/dynamics/gain_taught_local_correction.hpp"
#include <iostream>
#include <numeric>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
TaughtLocalConditions conditions(double duration=2) {
    TaughtLocalConditions c;c.start_index=0;c.end_index=static_cast<std::size_t>(std::round(duration/.01));
    c.source_dt_s=.01;c.tail_s=.5;c.feedforward_scope="same_none_MIT_reference_velocity_enabled";c.load_scope="same_declared_no_payload";return c;
}
std::pair<double,double> reference(double time,double duration,bool stationary=false,bool reverse=false) {
    if(stationary)return {.05,0};
    const double x=std::clamp(time/duration,0.0,1.0),position=.12*x*x*(3-2*x);
    const double speed=time>=0 && time<duration?.72*x*(1-x)/duration:0;
    return reverse?std::pair<double,double>{.12-position,-speed}:std::pair<double,double>{position,speed};
}
std::vector<std::vector<Sample>> simulate(const Gains& gains,const std::vector<double>& inertia,
    const std::vector<NoiseEstimate>& noise,double duration=2,bool stationary=false,bool reverse=false) {
    const auto n=inertia.size();std::vector<std::vector<Sample>> rows(n);
    constexpr double dt=.0005;
    for(std::size_t axis=0;axis<n;++axis) {
        double q=reference(0,duration,stationary,reverse).first,v=0;
        const double M=inertia[axis],friction=.18*M,bias=-.8*M,delay=.01+.2*M;
        for(unsigned k=0;k<=static_cast<unsigned>((duration+.5)/dt);++k) {
            const double t=k*dt;const auto r=reference(t-delay,duration,stationary,reverse);
            const double quantum=noise[axis].encoder_step_rad;
            const double sensed=quantum>0?std::round(q/quantum)*quantum:q;
            double drive=gains.kp[axis]*(r.first-sensed)+gains.kd[axis]*(r.second-v);
            // A finite actuator dead zone and torque code are part of this
            // plant; no fitted coefficient or desired feedback is injected.
            const double dead=.02*M,torque_code=.00002*M;
            drive=std::abs(drive)<=dead?0:drive-std::copysign(dead,drive);
            drive=std::round(drive/torque_code)*torque_code;
            double force=drive+bias-.3*M*v;
            if(std::abs(v)<1e-5 && std::abs(force)<=friction)v=0;
            else {force-=friction*(std::abs(v)>1e-5?std::copysign(1.0,v):std::copysign(1.0,force));v+=dt*force/M;q+=dt*v;}
            if(k%10)continue;
            const auto nominal=reference(t,duration,stationary,reverse);Sample s;
            s.time_s=t+.002;s.command_time_s=t+.002;s.feedback_time_s=t-.003;
            s.reference_sample_time_known=true;s.reference_sample_time_s=t;
            s.actual=quantum>0?std::round(q/quantum)*quantum:q;s.velocity=v;
            s.reference=nominal.first;s.reference_velocity=nominal.second;s.actual_kp=gains.kp[axis];s.actual_kd=gains.kd[axis];
            s.actual_gains_known=true;s.independent_disturbance_known=true;s.feedforward_known=true;s.applied_feedforward_nm=0;
            s.new_measurement=true;s.feedback_sequence=k+1;rows[axis].push_back(s);
        }
    }
    return rows;
}
std::vector<ModelAxisLimits> limits(std::size_t n) {
    std::vector<ModelAxisLimits> out(n);
    for(auto& a:out){a.effort_budget=2;a.kp_max=500;a.kd_max=5;a.max_velocity=2;a.max_acceleration=20;}
    return out;
}
void real_plant_improvement_and_mapping() {
    for(std::size_t n:{1,3,7}) {
        Gains gains;std::vector<double> inertia(n);std::vector<NoiseEstimate> noise(n);
        for(std::size_t i=0;i<n;++i){inertia[i]=.015+.012*i;gains.kp.push_back(50*inertia[i]);
            gains.kd.push_back(2*.8*std::sqrt(inertia[i]*gains.kp.back()));noise[i].encoder_step_rad=.0001;}
        auto rs=simulate(gains,inertia,noise);const auto c=conditions();
        const auto base=assess_taught_local_segment(rs,noise,gains,c);
        check(base.complete && !base.model_identified,"a real delayed nonlinear plant must produce complete fixed-PD empirical baseline evidence");
        const auto proposal=taught_local_candidate(gains,limits(n),noise,base);
        check(proposal.changed && proposal.coordinate=="Kp" && !proposal.model_identified,"unresolved measured lag/static bias must generate a local P poll without claiming an identified model");
        const auto candidate=assess_taught_local_segment(simulate(proposal.gains,inertia,noise),noise,proposal.gains,c);
        if(!taught_local_preferred(candidate,base,proposal.gains,gains)) {
            std::cerr<<"n="<<n<<" selected="<<proposal.axis<<" P="<<gains.kp[proposal.axis]<<" -> "<<proposal.gains.kp[proposal.axis]<<'\n';
            for(std::size_t i=0;i<n;++i)std::cerr<<i<<" rms "<<base.axes[i].rms_error_rad<<" -> "<<candidate.axes[i].rms_error_rad
                <<" peak "<<base.axes[i].peak_error_rad<<" -> "<<candidate.axes[i].peak_error_rad<<" static "<<base.axes[i].static_error_rad<<" -> "<<candidate.axes[i].static_error_rad
                <<" over "<<base.axes[i].dynamic_overshoot_rad<<" -> "<<candidate.axes[i].dynamic_overshoot_rad<<'\n';
        }
        check(taught_local_preferred(candidate,base,proposal.gains,gains),"measured bounded P poll must actually improve this delayed friction/dead-zone/quantized taught traversal");
        check(candidate.axes[proposal.axis].rms_error_rad<base.axes[proposal.axis].rms_error_rad &&
            proposal.gains.kp[proposal.axis]<=1.12*gains.kp[proposal.axis]+1e-12,
            "actual improvement must use at most a 12 percent P change rather than an unexplained gain jump");
        const auto plan=plan_taught_local_correction(simulate(gains,inertia,noise,8),noise,801,.01,false,1,15);
        check(plan.valid && plan.start_index<plan.end_index && plan.duration_s<=2 && plan.end_index-plan.start_index<800 && plan.comparison_cost_s<=15,
            "worst measured tracking errors must select a budgeted proper source subsegment rather than another full traversal");
        const auto reverse=plan_taught_local_correction(simulate(gains,inertia,noise,8),noise,801,.01,true,1,15);
        check(reverse.valid && reverse.start_index==800-plan.start_index && reverse.end_index==800-plan.end_index && reverse.reverse,
            "reverse source indices must preserve the measured traversal direction");
        check(!plan_taught_local_correction(rs,noise,201,.01,false,1,.5).valid,"insufficient budget must not start an incomplete local comparison");
        check(!taught_local_candidate(gains,limits(n),noise,base,std::vector<bool>(n,true)).changed,
            "the model-failure fallback must leave axes with supported online models to their existing tuner");
        auto permuted_gains=gains;auto permuted_rows=rs;auto permuted_noise=noise;auto permuted_inertia=inertia;
        std::reverse(permuted_gains.kp.begin(),permuted_gains.kp.end());std::reverse(permuted_gains.kd.begin(),permuted_gains.kd.end());
        std::reverse(permuted_rows.begin(),permuted_rows.end());std::reverse(permuted_noise.begin(),permuted_noise.end());std::reverse(permuted_inertia.begin(),permuted_inertia.end());
        const auto perm_base=assess_taught_local_segment(permuted_rows,permuted_noise,permuted_gains,c);
        const auto perm_poll=taught_local_candidate(permuted_gains,limits(n),permuted_noise,perm_base);
        check(perm_poll.changed && perm_poll.axis==n-1-proposal.axis &&
            taught_local_preferred(assess_taught_local_segment(simulate(perm_poll.gains,permuted_inertia,permuted_noise),permuted_noise,perm_poll.gains,c),perm_base,perm_poll.gains,permuted_gains),
            "permuting the mapped inertias, gains and rows must preserve which physical axis improves");
        std::cout<<"taught local "<<n<<" axes RMS "<<base.axes[proposal.axis].rms_error_rad<<" -> "<<candidate.axes[proposal.axis].rms_error_rad<<'\n';
    }
}
void completeness_conditions_and_whole_group() {
    const Gains gains{{2,3},{.3,.45}};const std::vector<double> M{.04,.06};std::vector<NoiseEstimate> noise(2);for(auto& x:noise)x.encoder_step_rad=.0001;
    const auto c=conditions();const auto rows=simulate(gains,M,noise);const auto base=assess_taught_local_segment(rows,noise,gains,c);
    const auto proposal=taught_local_candidate(gains,limits(2),noise,base);
    check(proposal.changed,"comparison refusal checks require a real empirical proposal");
    const auto actual=simulate(proposal.gains,M,noise);const auto candidate=assess_taught_local_segment(actual,noise,proposal.gains,c);
    check(taught_local_preferred(candidate,base,proposal.gains,gains),"same-condition empirical candidate must be retained before refusal checks");
    auto changed=c;changed.playback_rate=.9;
    check(!taught_local_preferred(assess_taught_local_segment(actual,noise,proposal.gains,changed),base,proposal.gains,gains),"different speed cannot masquerade as gain improvement");
    changed=c;changed.feedforward_scope="different_feedforward";
    check(!taught_local_preferred(assess_taught_local_segment(actual,noise,proposal.gains,changed),base,proposal.gains,gains),"different declared feedforward must not be compared");
    changed=c;changed.load_scope="different_load";
    check(!taught_local_preferred(assess_taught_local_segment(actual,noise,proposal.gains,changed),base,proposal.gains,gains),"different declared load must not be compared");
    changed=c;changed.start_index=c.end_index;changed.end_index=c.start_index;changed.reverse=true;
    check(!taught_local_preferred(assess_taught_local_segment(actual,noise,proposal.gains,changed),base,proposal.gains,gains),"opposite path direction cannot be credited as controller improvement");
    auto bad=actual;bad[0].back().actual_kp+=.1;
    check(!assess_taught_local_segment(bad,noise,proposal.gains,c).complete,"mixed actual PD cannot qualify a measured complete group");
    bad=actual;bad[0].resize(bad[0].size()/2);
    check(!assess_taught_local_segment(bad,noise,proposal.gains,c).complete,"an uncovered portion of the segment cannot become a complete comparison");
    bad=actual;for(auto& r:bad[0])r.feedback_sequence=1;
    check(!assess_taught_local_segment(bad,noise,proposal.gains,c).complete,"repeated feedback cannot replace missing traversal evidence");
    bad=actual;for(auto& r:bad[0]){r.feedback_time_s=0;r.command_time_s=0;r.reference_sample_time_known=false;}
    check(!assess_taught_local_segment(bad,noise,proposal.gains,c).complete,
        "invented fresh flags and host row times cannot replace a repeated actual feedback clock");
    bad=actual;for(auto& r:bad[0])r.limited=true;
    check(!taught_local_preferred(assess_taught_local_segment(bad,noise,proposal.gains,c),base,proposal.gains,gains),"limited candidate cannot hide behind a lower tracking RMS");
    bad=actual;for(auto& r:bad[0])r.reference+=.01;
    check(!taught_local_preferred(assess_taught_local_segment(bad,noise,proposal.gains,c),base,proposal.gains,gains),"changed actual accepted reference must not bypass the same-segment contract");
    bad=actual;for(auto& r:bad[0])r.actual-=.03;
    check(!taught_local_preferred(assess_taught_local_segment(bad,noise,proposal.gains,c),base,proposal.gains,gains),"worsening another axis must reject the entire candidate group");
    auto both=gains;both.kp[0]*=1.1;both.kp[1]*=1.1;
    check(!taught_local_preferred(assess_taught_local_segment(simulate(both,M,noise),noise,both,c),base,both,gains),"two coordinate winners cannot be assembled into an unpolled group");
    auto tight=limits(2);for(auto& b:tight)b.effort_budget=.0001;
    check(!taught_local_candidate(gains,tight,noise,base).changed,"a local poll cannot bypass the existing measured whole-group effort admission");
    const auto held=assess_taught_local_segment(simulate(gains,M,noise,2,true),noise,gains,c);
    const auto static_proposal=taught_local_candidate(gains,limits(2),noise,held);
    check(held.complete && held.axes[0].static_measured && static_proposal.changed && static_proposal.coordinate=="Kp" &&
        static_proposal.gains.kp[static_proposal.axis]>gains.kp[static_proposal.axis],"slow/static taught bias must propose a measured P direction even without a dynamic fit");
    bad=rows;for(auto& axis:bad)for(auto& row:axis)row.actual=row.reference;
    check(!plan_taught_local_correction(bad,noise,201,.01,false,1,15).valid,
        "an already unresolved-small tracking error must not consume another local measurement loop");
    auto unknown_noise=noise;unknown_noise[0].rms_rad=std::numeric_limits<double>::quiet_NaN();
    check(!assess_taught_local_segment(rows,unknown_noise,gains,c).complete,
        "non-finite uncertainty evidence cannot become a zero-noise successful comparison");
}
void coarse_code() {
    const double pstep=500.0/4095,dstep=5.0/4095;const Gains gains{{3*pstep},{16*dstep}};
    std::vector<NoiseEstimate> noise(1);noise[0].encoder_step_rad=.0001;auto bound=limits(1);bound[0].grid={pstep,dstep};
    const auto c=conditions();const auto base=assess_taught_local_segment(simulate(gains,{.001},noise,2,true),noise,gains,c);
    const auto poll=taught_local_candidate(gains,bound,noise,base);
    check(poll.changed && std::abs(poll.gains.kp[0]-gains.kp[0]-pstep)<1e-12,
        "low positive gain codes need one adjacent encoded poll, not a permanently rounded zero step");
    const auto after=assess_taught_local_segment(simulate(poll.gains,{.001},noise,2,true),noise,poll.gains,c);
    check(taught_local_preferred(after,base,poll.gains,gains),"adjacent low-code polling must improve actual dead-zone/quantized plant bias before retention");
}

std::vector<std::vector<Sample>> irregular_samples(const std::vector<std::vector<Sample>>& dense,
    const std::vector<NoiseEstimate>& noise,double first,bool alternate) {
    auto out=dense;for(auto& axis:out)axis.clear();
    const double intervals[]={.002,.004,.001,.003,.0025};
    for(std::size_t axis=0;axis<dense.size();++axis) {
        const auto& source=dense[axis];std::size_t right=1,index=0;
        for(double t=first;t<=2.5;t+=intervals[(index+++(alternate?2:0))%5]) {
            while(right+1<source.size() && source[right].reference_sample_time_s<t)++right;
            const auto& a=source[right-1];const auto& b=source[right];
            const double u=(t-a.reference_sample_time_s)/(b.reference_sample_time_s-a.reference_sample_time_s);
            auto row=a;row.time_s=row.command_time_s=t+.002;row.feedback_time_s=t-.003;
            row.reference_sample_time_s=t;row.feedback_sequence=index+1;
            row.actual=a.actual+u*(b.actual-a.actual);row.velocity=a.velocity+u*(b.velocity-a.velocity);
            if(noise[axis].encoder_step_rad>0)row.actual=std::round(row.actual/noise[axis].encoder_step_rad)*noise[axis].encoder_step_rad;
            const auto nominal=reference(t,2);row.reference=nominal.first;row.reference_velocity=nominal.second;
            out[axis].push_back(row);
        }
    }
    return out;
}

void irregular_clock_same_path_and_real_speed_refusal() {
    for(std::size_t n:{1,3,7}) {
        Gains gains;std::vector<double> inertia(n);std::vector<NoiseEstimate> noise(n);
        for(std::size_t i=0;i<n;++i) {
            inertia[i]=.018+.011*i;gains.kp.push_back(50*inertia[i]);
            gains.kd.push_back(1.6*std::sqrt(inertia[i]*gains.kp.back()));noise[i].encoder_step_rad=.0001;
        }
        const auto c=conditions();const auto dense=simulate(gains,inertia,noise);
        const auto baseline=assess_taught_local_segment(irregular_samples(dense,noise,.0007,false),noise,gains,c);
        const auto proposal=taught_local_candidate(gains,limits(n),noise,baseline);
        check(proposal.changed,"irregular sampled nonlinear plant must still propose a bounded measured poll");
        const auto sampled=irregular_samples(simulate(proposal.gains,inertia,noise),noise,.0043,true);
        const auto candidate=assess_taught_local_segment(sampled,noise,proposal.gains,c);
        const auto report=taught_local_comparison_report(candidate,baseline,proposal.gains,gains);
        if(!report.preferred)std::cerr<<"irregular "<<n<<" axes: "<<report.reason<<'\n';
        check(report.preferred,"different actual command grids on the same source curve must retain a real complete-group improvement");
        // At t=0 the old comparator clamped these distinct first samples.
        // Their velocity difference exceeds its old relative allowance even
        // though both are exact evaluations of the same curved source.
        const auto& a=sampled[0].front();const auto& b=baseline.rows[0].front();
        check(std::abs(a.reference_velocity-b.reference_velocity)>.02*std::max({std::abs(a.reference_velocity),std::abs(b.reference_velocity),.01})+
            baseline.axes[0].comparison_uncertainty_rad/2.5,
            "the irregular-clock fixture must expose the former first-sample clamping rejection");
        auto changed=sampled;for(auto& axis:changed)for(auto& row:axis)row.reference_velocity*=1.3;
        const auto wrong=assess_taught_local_segment(changed,noise,proposal.gains,c);
        const auto refused=taught_local_comparison_report(wrong,baseline,proposal.gains,gains);
        check(!refused.preferred && refused.reason=="taught_local_actual_reference_velocity_not_comparable",
            "sampling uncertainty cannot excuse a genuinely different emitted speed under identical labels");
        changed=sampled;for(auto& axis:changed)for(auto& row:axis)row.reference+=.01;
        const auto shifted=taught_local_comparison_report(assess_taught_local_segment(changed,noise,proposal.gains,c),baseline,proposal.gains,gains);
        check(!shifted.preferred && shifted.reason=="taught_local_actual_reference_position_not_comparable",
            "irregular clock handling cannot excuse a different actual source path");
    }
}

void entry_bias_is_not_motion_overshoot() {
    for(std::size_t n:{1,3,7}) {
        Gains gains;std::vector<double> inertia(n,.04);std::vector<NoiseEstimate> noise(n);
        for(std::size_t i=0;i<n;++i){gains.kp.push_back(2+.5*i);gains.kd.push_back(.3+.05*i);noise[i].encoder_step_rad=.0001;}
        auto rows=simulate(gains,inertia,noise);
        for(auto& axis:rows)for(auto& row:axis) {row.actual=row.reference+.01;row.velocity=row.reference_velocity;}
        const auto biased=assess_taught_local_segment(rows,noise,gains,conditions());
        check(biased.complete,"entry-offset metric checks require a complete known controller record");
        for(const auto& axis:biased.axes)check(axis.peak_error_rad>.0099 && axis.static_error_rad>.0099 &&
            axis.raw_directional_excess_rad>.0099 && axis.dynamic_overshoot_rad<1e-10,
            "constant entry offset must remain raw peak/static error without becoming dynamic motion overshoot");
        for(auto& axis:rows)for(auto& row:axis) {
            const double t=row.reference_sample_time_s;
            const double bump=t>.5 && t<1.5?.005*std::sin(3.141592653589793*(t-.5)):0;
            row.actual=row.reference+.01+bump;
        }
        const auto moving=assess_taught_local_segment(rows,noise,gains,conditions());
        for(const auto& axis:moving.axes)check(axis.dynamic_overshoot_rad>.0049 && axis.peak_error_rad>.0149 && axis.static_error_rad>.0099,
            "new motion beyond an existing offset must remain measured as real dynamic overshoot");
    }
}

void initial_state_retry_and_real_coupled_regression() {
    for(std::size_t n:{1,3,7}) {
        Gains gains;std::vector<double> inertia(n);std::vector<NoiseEstimate> noise(n);
        for(std::size_t i=0;i<n;++i){inertia[i]=.015+.01*i;gains.kp.push_back(50*inertia[i]);
            gains.kd.push_back(1.6*std::sqrt(inertia[i]*gains.kp.back()));noise[i].encoder_step_rad=.0001;}
        const auto c=conditions();const auto base=assess_taught_local_segment(simulate(gains,inertia,noise),noise,gains,c);
        const auto proposal=taught_local_candidate(gains,limits(n),noise,base);
        check(proposal.changed,"initial-state comparison requires an actual bounded nonlinear-plant poll");
        const auto actual=simulate(proposal.gains,inertia,noise);
        const auto candidate=assess_taught_local_segment(actual,noise,proposal.gains,c);
        check(taught_local_comparison_report(candidate,base,proposal.gains,gains).preferred,
            "a matched nonlinear-plant comparison must remain retainable");
        auto shifted=actual;for(auto& row:shifted[0])row.actual-=.01;
        const auto shifted_assessment=assess_taught_local_segment(shifted,noise,proposal.gains,c);
        const auto unmatched=taught_local_comparison_report(shifted_assessment,base,proposal.gains,gains);
        check(!unmatched.preferred && unmatched.retry_recommended &&
            unmatched.reason=="taught_local_initial_state_not_comparable_retry_same_segment",
            "different measured entry states require a matched repeat rather than bias subtraction or acceptance");
        check(shifted_assessment.axes[0].peak_error_rad>candidate.axes[0].peak_error_rad,
            "initial-state diagnosis must preserve the actual uncorrected peak");
        if(n>1) {
            const std::size_t untouched=(proposal.axis+1)%n;
            shifted=actual;for(auto& row:shifted[untouched])row.actual-=.0003;
            const auto small=assess_taught_local_segment(shifted,noise,proposal.gains,c);
            const auto retry=taught_local_comparison_report(small,base,proposal.gains,gains);
            check(!retry.preferred && retry.retry_recommended && retry.reason=="taught_local_unchanged_axis_entry_bias_requires_matched_repeat",
                "a small resolved entry bias on an unchanged axis must request a repeat rather than erase a useful complete-group poll");
            shifted=actual;
            for(auto& row:shifted[untouched])if(row.reference_sample_time_s>.4)row.actual-=.006;
            const auto coupled=taught_local_comparison_report(assess_taught_local_segment(shifted,noise,proposal.gains,c),base,proposal.gains,gains);
            check(!coupled.preferred && !coupled.retry_recommended && coupled.reason=="taught_local_measured_complete_group_response_worsened",
                "real post-entry degradation on an unchanged coupled axis must reject the whole group");
            shifted=actual;
            for(auto& row:shifted[untouched])if(row.reference_sample_time_s>.3)
                row.actual=row.reference+.02*std::sin(20*3.141592653589793*row.reference_sample_time_s);
            const auto unstable=taught_local_comparison_report(assess_taught_local_segment(shifted,noise,proposal.gains,c),base,proposal.gains,gains);
            check(!unstable.preferred && !unstable.retry_recommended && unstable.reason=="taught_local_new_sustained_residual_oscillation",
                "new persistent residual oscillation must remain a rejection and must not become an entry-bias retry");
        }
    }
}
void choose_motion_instead_of_uninformative_plateau() {
    for(std::size_t n:{1,3,7}) {
        std::vector<std::vector<Sample>> rows(n);std::vector<NoiseEstimate> noise(n);
        for(auto& x:noise)x.encoder_step_rad=.0001;
        for(std::size_t axis=0;axis<n;++axis)for(std::size_t k=0;k<=800;++k) {
            const double t=.01*k;Sample s;s.time_s=s.command_time_s=t;s.feedback_time_s=t-.003;
            s.reference_sample_time_known=true;s.reference_sample_time_s=t;
            // A natural taught move followed by a long friction-stuck hold.
            // The plateau has the largest raw error but contains no change
            // of reference from which to compare tracking improvement.
            s.reference=t<3?.04*t:.12;s.reference_velocity=t<3?.04:0;
            s.actual=t<3?s.reference-.01:.07;
            s.actual_gains_known=true;s.actual_kp=2;s.actual_kd=.1;
            s.new_measurement=true;s.feedback_sequence=k+1;rows[axis].push_back(s);
        }
        const auto plan=plan_taught_local_correction(rows,noise,801,.01,false,1,15);
        check(plan.valid && plan.focus_reference_span_rad>.01 && plan.start_index<300,
            "local correction must choose an informative naturally moving taught part instead of the worst stationary plateau");
        TaughtLocalOptions nearby;nearby.nearby_source_index=250;nearby.maximum_source_distance_s=1.4;
        const auto bounded=plan_taught_local_correction(rows,noise,801,.01,false,1,15,nearby);
        check(bounded.valid && std::abs(static_cast<double>(bounded.start_index)-250)*.01<=1.4+1e-9 &&
            std::abs(static_cast<double>(bounded.end_index)-250)*.01<=1.4+1e-9,
            "the planner must enforce both actual source endpoints against the caller's nearby range");
        Gains g;g.kp.assign(n,2);g.kd.assign(n,.1);
        auto held_rows=simulate(g,std::vector<double>(n,.01),noise,2,true);
        for(auto& axis:held_rows)for(auto& s:axis){s.actual=.01;s.velocity=0;}
        const auto held=assess_taught_local_segment(held_rows,noise,g,conditions());
        const auto first=taught_local_candidate(g,limits(n),noise,held,{},0);
        const auto next=taught_local_candidate(g,limits(n),noise,held,{},1);
        check(first.changed && next.changed && first.coordinate=="Kp" && next.coordinate=="Kp" &&
            next.gains.kp[next.axis]>first.gains.kp[first.axis] && next.gains.kp[next.axis]<=1.25*g.kp[next.axis],
            "an unresolved static P poll must use its next bounded P trial rather than waste the comparison on zero-velocity D");
    }
}
}
int main(){try{real_plant_improvement_and_mapping();completeness_conditions_and_whole_group();coarse_code();irregular_clock_same_path_and_real_speed_refusal();entry_bias_is_not_motion_overshoot();initial_state_retry_and_real_coupled_regression();
    choose_motion_instead_of_uninformative_plateau();
    std::cout<<"taught local empirical correction: passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
