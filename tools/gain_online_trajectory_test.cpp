// Sampled MIT plants run their motor feedback at 1 kHz, commands/telemetry at
// about 200 Hz, with a torque delay and independent actuator-side friction. The
// identification code sees only the recorded host command proxy and feedback.
#include "serial_arm/dynamics/gain_online_trajectory_tuning.hpp"
#include <iostream>
#include <numeric>
using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Run {
    Gains last;std::vector<std::vector<Sample>> rows;
    std::size_t models{0},updates{0},first_update_tick{0};
    std::size_t nonuniform_windows{0};
    double first_p_code_step{0};
    double error_square{0},late_error_square{0};std::size_t measured{0},late_measured{0};
    std::vector<OnlineTrajectoryAxisModel> last_models;
    std::vector<OnlineTrajectoryReport> reports;
};
constexpr double duration=18,host_dt=.005,plant_dt=.001;
constexpr double position_quantum=25.0/65536,velocity_quantum=90.0/4095;
OnlineTrajectoryReference reference(double t,std::size_t n) {
    OnlineTrajectoryReference f;f.time_s=t;
    for(std::size_t i=0;i<n;++i){
        const double omega=2*pi/(9+.35*i),phase=omega*t;
        f.position.push_back(.28*(1-std::cos(phase)));
        f.velocity.push_back(.28*omega*std::sin(phase));
        const double I=.035+.021*i;f.feedforward_nm.push_back(.2*I);
    }
    return f;
}
Run run(std::size_t n,bool adaptive,double inertia_scale=1,bool known_input=true,double viscosity=.25,double coupling=0,bool protocol_grid=false,bool jittered_feedback=false,std::size_t initial_kp_codes=0,const Gains* frozen_gains=nullptr,bool excite=true) {
    std::vector<double> inertias;std::vector<ModelAxisLimits> bounds(n);std::vector<NoiseEstimate> noise(n);
    Gains actual,target;std::vector<double> q(n,0),v(n,0),accepted_r(n,0),accepted_rv(n,0),injection(n,0),ff(n,0);
    std::vector<std::deque<double>> delayed(n,std::deque<double>(9,0));
    for(std::size_t i=0;i<n;++i){
        const double I=.035+.021*i;inertias.push_back(I*inertia_scale);
        actual.kp.push_back(14*I);actual.kd.push_back(8*I);
        bounds[i].kp_max=500;bounds[i].kd_max=5;bounds[i].effort_budget=4;
        bounds[i].grid=protocol_grid?GainCommandGrid{500.0/4095,5.0/4095}:GainCommandGrid{0,0};
        if(protocol_grid){actual.kp.back()=quantize_gain(actual.kp.back(),bounds[i].grid.kp_step,500,true);
            actual.kd.back()=quantize_gain(actual.kd.back(),bounds[i].grid.kd_step,5);}
        if(protocol_grid && initial_kp_codes)actual.kp.back()=initial_kp_codes*bounds[i].grid.kp_step;
        noise[i].encoder_step_rad=jittered_feedback?position_quantum:1e-5;noise[i].rms_rad=1e-6;
    }
    Eigen::MatrixXd mass(n,n);
    for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)
        mass(i,j)=i==j?.035+.021*i:coupling*std::sqrt((.035+.021*i)*(.035+.021*j));
    const Eigen::LDLT<Eigen::MatrixXd> inverse_mass(mass);
    if(frozen_gains)actual=*frozen_gains;
    target=actual;Gains ramp_source=actual;double ramp_started=0;Run out;out.rows.resize(n);
    std::deque<std::vector<double>> q_history,v_history;
    std::vector<double> measured_q(n,0),measured_v(n,0);
    std::size_t next_host_tick=0,host_count=0,feedback_sequence=0;
    double last_feedback_time=0,next_update=4;
    for(std::size_t tick=0;tick<=static_cast<std::size_t>(duration/plant_dt);++tick){
        const double t=tick*plant_dt;
        q_history.push_back(q);v_history.push_back(v);
        if(q_history.size()>4){q_history.pop_front();v_history.pop_front();}
        if(tick==next_host_tick){
            ++host_count;
            const bool new_feedback=!jittered_feedback || host_count%137!=0;
            if(new_feedback){
                ++feedback_sequence;last_feedback_time=std::max(0.0,t-.003);
                for(std::size_t i=0;i<n;++i){
                    const double quantum=jittered_feedback?position_quantum:1e-5;
                    const double physical_q=jittered_feedback?q_history.front()[i]:q[i];
                    const double physical_v=jittered_feedback?v_history.front()[i]:v[i];
                    measured_q[i]=std::round((physical_q+1e-6*std::sin(43*t+i))/quantum)*quantum;
                    const double noisy_v=physical_v+2e-5*std::sin(39*t+.7*i);
                    measured_v[i]=jittered_feedback?std::round(noisy_v/velocity_quantum)*velocity_quantum:noisy_v;
                }
            }
            // Generic scheduling jitter independent of axis, controller and
            // observations. Long (but healthy) host intervals occur sparsely.
            constexpr std::size_t jitter_ticks[]{5,4,6,5,3,7,5,5,4,6,5,5,6,4,5,5,7,3,5};
            const std::size_t interval=jittered_feedback?(host_count%109==0?12:host_count%61==0?10:
                jitter_ticks[(host_count-1)%(sizeof(jitter_ticks)/sizeof(jitter_ticks[0]))]):5;
            next_host_tick=tick+interval;
            const auto f=reference(t,n);
            for(std::size_t i=0;i<n;++i){
                const double blend=online_trajectory_detail::smooth((t-ramp_started)/.12);
                actual.kp[i]=ramp_source.kp[i]+blend*(target.kp[i]-ramp_source.kp[i]);
                actual.kd[i]=ramp_source.kd[i]+blend*(target.kd[i]-ramp_source.kd[i]);
                if(protocol_grid){
                    actual.kp[i]=online_trajectory_detail::nearest_gain(actual.kp[i],bounds[i].grid.kp_step,500);
                    actual.kd[i]=online_trajectory_detail::nearest_gain(actual.kd[i],bounds[i].grid.kd_step,5);
                }
                accepted_r[i]=f.position[i];accepted_rv[i]=f.velocity[i];ff[i]=f.feedforward_nm[i];
                injection[i]=online_trajectory_injection(t,i,.035+.021*i,target.kp[i],target.kd[i],3,
                    std::max(0.0,f.position[i]),std::max(0.0,.6-f.position[i]),host_dt,duration);
                if(!excite)injection[i]=0;
                Sample s;s.time_s=s.command_time_s=t;s.feedback_time_s=jittered_feedback?last_feedback_time:t-.003;
                s.actual=measured_q[i];s.velocity=measured_v[i];
                s.reference=accepted_r[i];s.reference_velocity=accepted_rv[i];
                s.actual_kp=actual.kp[i];s.actual_kd=actual.kd[i];s.actual_gains_known=true;
                s.applied_feedforward_nm=ff[i];s.feedforward_known=true;
                s.independent_disturbance_nm=injection[i];s.independent_disturbance_known=known_input;
                s.planned_independent_disturbance_nm=injection[i];s.independent_input_planned_known=true;
                s.new_measurement=new_feedback;s.feedback_sequence=feedback_sequence;out.rows[i].push_back(s);
                if(t>4){out.error_square+=std::pow(s.actual-s.reference,2);++out.measured;}
                if(t>12){out.late_error_square+=std::pow(s.actual-s.reference,2);++out.late_measured;}
            }
            if(adaptive && t>=next_update && t<duration-.8){
                next_update=t+1;
                std::vector<OnlineTrajectoryReference> future;
                for(std::size_t k=0;k<=43;++k)future.push_back(reference(t-.06+.02*k,n));
                const auto report=online_trajectory_candidate(actual,out.rows,future,inertias,bounds,noise);
                out.reports.push_back(report);
                if(protocol_grid && !out.first_p_code_step)for(std::size_t i=0;i<n;++i)
                    if(std::abs(report.gains.kp[i]-actual.kp[i])>1e-10){out.first_p_code_step=std::abs(report.gains.kp[i]-actual.kp[i])/bounds[i].grid.kp_step;break;}
                ramp_source=actual;ramp_started=t;target=report.gains;out.last_models=report.models;
                for(const auto& m:report.models){out.models+=m.valid;out.nonuniform_windows+=m.reason=="nonuniform_model_time_grid";}
                if(report.changed){++out.updates;if(!out.first_update_tick)out.first_update_tick=tick;}
            }
        }
        Eigen::VectorXd force(n);
        for(std::size_t i=0;i<n;++i){
            const double I=.035+.021*i;
            // The embedded motor sees its present internal state, not the
            // quantized/noisy host telemetry used to reconstruct the input.
            const double command=actual.kp[i]*(accepted_r[i]-q[i])+actual.kd[i]*(accepted_rv[i]-v[i])+ff[i]+injection[i];
            delayed[i].push_back(command);const double torque=delayed[i].front();delayed[i].pop_front();
            const double load=.2*I+viscosity*I*v[i]+.12*I*std::tanh(v[i]/.02);
            force[i]=torque-load;
        }
        const Eigen::VectorXd acceleration=inverse_mass.solve(force);
        for(std::size_t i=0;i<n;++i){v[i]+=acceleration[i]*plant_dt;q[i]+=v[i]*plant_dt;
            check(std::isfinite(q[i]) && std::isfinite(v[i]) && std::abs(q[i])<2 && std::abs(v[i])<10,
                "sampled MIT trajectory must remain numerically and dynamically bounded with the actual 120ms gain ramp");}
    }
    out.last=actual;return out;
}
void tied_input_model_and_compliant_objective() {
    using namespace online_trajectory_detail;
    std::vector<Point> p;double q=0,previous=0,old_u=0;
    constexpr double dt=.02,inertia=.1;
    for(std::size_t k=0;k<240;++k) {
        const double t=k*dt,r=.1*std::sin(.7*t),rv=.07*std::cos(.7*t);
        const double d=.04*code(k/12,0),u=2*(r-q)+.3*(rv-(q-previous)/dt)+d;
        const double next=1.96*q-.96*previous+.002*u+.002*old_u;
        Point point;point.t=t;point.q=std::round(next/.0001)*.0001;point.v=(next-q)/dt;
        point.u=u;point.d=d;point.r=r;point.rv=rv;point.P=2;point.D=.3;p.push_back(point);
        previous=q;q=next;old_u=u;
    }
    NoiseEstimate noise;noise.encoder_step_rad=.0001;
    OnlineTrajectoryAxisModel seed;seed.dt_s=dt;seed.observed_encoder_step_rad=.0001;
    const auto fitted=fit(p,inertia,noise,{},seed);
    check(fitted.valid && fitted.parameter_count==4 && fitted.structure=="arx2_tied_zoh_input_coefficient" &&
          fitted.arx_b0==fitted.arx_b1 && fitted.iv_rank==4,
        "a measured tied-input plant must retain the supported lower-degree structure rather than demand separable adjacent numerator coefficients");
    auto damaged=p;for(std::size_t k=180;k<damaged.size();++k)damaged[k].q+=.15*std::sin(21*damaged[k].t);
    check(!fit(damaged,inertia,noise,{},seed).valid,
        "the lower-degree model must still reject a new held-out output disturbance");
    OnlineTrajectoryAxisModel model;model.valid=true;model.dt_s=.02;model.arx_a1=2;model.arx_a2=-1;
    model.arx_b0=model.arx_b1=.002;model.independent_input_rms_nm=.04/std::sqrt(2.0);model.position_noise_rad=.00001;
    p.clear();for(std::size_t k=0;k<100;++k){Point point;point.t=k*.02;point.d=(k%2?-.04:.04);point.u=point.d;p.push_back(point);}
    std::vector<OnlineTrajectoryReference> future;
    for(std::size_t k=0;k<50;++k)future.push_back({p.back().t-.06+k*.02,{0},{0},{0}});
    OnlineTrajectoryOptions tracking,soft;soft.use=OnlineTrajectoryUse::Compliant;soft.compliant_displacement_rad=.04;
    const auto rigid=rollout(model,p,future,0,10,.5,.04,.1,.04,tracking);
    const auto compliant=rollout(model,p,future,0,10,.5,.04,.1,.04,soft);
    check(rigid.size()>0 && rigid.squaredNorm()<1e-12 && compliant.size()>rigid.size() && compliant.squaredNorm()>0,
        "compliant optimization must include a known-input yield/release prediction even when nominal trajectory tracking is perfect");
    const auto lower=rollout(model,p,future,0,9,.45,.04,.1,.04,soft);
    const auto higher=rollout(model,p,future,0,11,.55,.04,.1,.04,soft);
    check(lower.size()==compliant.size() && higher.size()==compliant.size() && lower.squaredNorm()<higher.squaredNorm(),
        "an insufficient-yield compliant predictor must favor the softer bounded comparison instead of maximizing rigid tracking stiffness");
}
}
int main(){
    tied_input_model_and_compliant_objective();
    for(std::size_t n:{1,3,7}){
        const auto baseline=run(n,false),adaptive=run(n,true);
        check(adaptive.models>0,"independent taught-path measurements must identify supported local models");
        check(adaptive.updates>0 && adaptive.first_update_tick<static_cast<std::size_t>((duration-1)/plant_dt),
            "PD must change before the ongoing taught trajectory finishes");
        check(adaptive.last.kp!=baseline.last.kp || adaptive.last.kd!=baseline.last.kd,
            "the saved controller must contain actual in-motion updates, not only the URDF initializer");
        const double old_rms=std::sqrt(baseline.error_square/baseline.measured),new_rms=std::sqrt(adaptive.error_square/adaptive.measured);
        std::cout<<n<<" axes: supported_models="<<adaptive.models<<" online_updates="<<adaptive.updates
                 <<" tracking RMS="<<old_rms<<" -> "<<new_rms<<'\n';
        check(new_rms<old_rms,"online measured-model updates must improve this sampled delayed MIT plant, not just change gains");
        for(std::size_t i=0;i<n;++i)check(std::isfinite(adaptive.last.kp[i]) && std::isfinite(adaptive.last.kd[i]) &&
            adaptive.last.kp[i]>=0 && adaptive.last.kp[i]<500 && adaptive.last.kd[i]>=0 && adaptive.last.kd[i]<5,
            "online gains must remain within the real protocol capability");
    }
    for(std::size_t n:{1,3,7}){
        const auto baseline=run(n,false,1,true,.25,0,true,true);
        const auto adaptive=run(n,true,1,true,.25,0,true,true);
        const auto original=baseline.rows;
        OnlineTrajectoryAxisModel clock_model;
        const auto points=online_trajectory_detail::points(baseline.rows[0],{},clock_model);
        check(points.size()>=50,"healthy irregular feedback must supply a usable uniformly sampled model window");
        const auto moving_resolution=online_trajectory_detail::fit(points,.035,NoiseEstimate{},OnlineTrajectoryOptions{},clock_model);
        check(std::abs(clock_model.observed_encoder_step_rad-position_quantum)<1e-9 &&
            moving_resolution.position_noise_rad>=position_quantum/std::sqrt(12.0)-1e-12,
            "a static window with no observed encoder transitions must use raw moving code spacing, not assume infinite feedback resolution");
        for(std::size_t k=1;k<points.size();++k)check(std::abs(points[k].t-points[k-1].t-clock_model.dt_s)<1e-9,
            "jittered telemetry must be resampled in time rather than selected into an irregular ARX grid");
        for(std::size_t axis=0;axis<n;++axis)for(std::size_t k=0;k<original[axis].size();++k){
            const auto& before=original[axis][k];const auto& after=baseline.rows[axis][k];
            check(before.time_s==after.time_s && before.command_time_s==after.command_time_s &&
                before.feedback_time_s==after.feedback_time_s && before.feedback_sequence==after.feedback_sequence &&
                before.actual==after.actual && before.velocity==after.velocity && before.reference==after.reference &&
                before.reference_velocity==after.reference_velocity && before.actual_kp==after.actual_kp &&
                before.actual_kd==after.actual_kd && before.independent_disturbance_nm==after.independent_disturbance_nm,
                "resampling must never modify raw feedback, timestamps, references, actual gains or known input");
            check(std::abs(after.actual/position_quantum-std::round(after.actual/position_quantum))<1e-8 &&
                std::abs(after.velocity/velocity_quantum-std::round(after.velocity/velocity_quantum))<1e-8,
                "the jittered plant must expose coarse real protocol position and velocity feedback");
        }
        std::size_t duplicates=0,long_intervals=0;
        for(std::size_t k=1;k<baseline.rows[0].size();++k){
            duplicates+=!baseline.rows[0][k].new_measurement;
            long_intervals+=baseline.rows[0][k].time_s-baseline.rows[0][k-1].time_s>.009;
        }
        check(duplicates>0 && long_intervals>0,"the physical regression must include duplicate feedback and genuine 10-12ms scheduling intervals");
        check(adaptive.models>0 && adaptive.updates>0 && adaptive.first_update_tick<static_cast<std::size_t>((duration-1)/plant_dt),
            "coarse, jittered physical feedback must produce supported PD updates during the original trajectory");
        check(adaptive.nonuniform_windows==0,"normal scheduling jitter must not silently disable all online learning windows");
        check(adaptive.last.kp!=baseline.last.kp || adaptive.last.kd!=baseline.last.kd,
            "real-protocol jittered learning must reach actual motor commands, not just emit update diagnostics");
        const double old_rms=std::sqrt(baseline.error_square/baseline.measured),new_rms=std::sqrt(adaptive.error_square/adaptive.measured);
        std::cout<<n<<" coarse jittered axes: supported_models="<<adaptive.models<<" online_updates="<<adaptive.updates
                 <<" tracking RMS="<<old_rms<<" -> "<<new_rms<<" duplicate_frames="<<duplicates<<" long_intervals="<<long_intervals<<'\n';
        // Learning changes the independent input amplitude and chip frequency.
        // Compare the resulting controllers on fresh complete frozen replays
        // with identical path, initial state, load, clock and no identification
        // input. Keep the raw online RMS above visible rather than attributing
        // a different exploration waveform to controller improvement.
        const auto old_fixed=run(n,false,1,true,.25,0,true,true,0,&baseline.last,false);
        const auto new_fixed=run(n,false,1,true,.25,0,true,true,0,&adaptive.last,false);
        double old_fixed_square=0,new_fixed_square=0;std::size_t fixed_samples=0;
        for(std::size_t axis=0;axis<n;++axis){
            check(old_fixed.rows[axis].size()==new_fixed.rows[axis].size(),"frozen comparison must have the same acquisition schedule");
            check(old_fixed.rows[axis].front().actual==new_fixed.rows[axis].front().actual &&
                old_fixed.rows[axis].front().velocity==new_fixed.rows[axis].front().velocity,
                "frozen comparison must start at the same physical position and velocity");
            for(std::size_t k=0;k<old_fixed.rows[axis].size();++k){
                const auto& a=old_fixed.rows[axis][k];const auto& b=new_fixed.rows[axis][k];
                check(a.time_s==b.time_s && a.reference==b.reference && a.reference_velocity==b.reference_velocity &&
                    a.applied_feedforward_nm==b.applied_feedforward_nm,
                    "frozen controller comparison must preserve the entire reference path, speed, feedforward and evaluation clock");
                check(a.independent_disturbance_nm==0 && b.independent_disturbance_nm==0 &&
                    a.planned_independent_disturbance_nm==0 && b.planned_independent_disturbance_nm==0,
                    "frozen comparison must contain no independent identification excitation");
                check(a.actual_kp==baseline.last.kp[axis] && a.actual_kd==baseline.last.kd[axis] &&
                    b.actual_kp==adaptive.last.kp[axis] && b.actual_kd==adaptive.last.kd[axis],
                    "the complete frozen path must actually execute its selected whole P/D group");
                check(std::abs(a.actual)<2 && std::abs(b.actual)<2 && std::abs(a.velocity)<10 && std::abs(b.velocity)<10,
                    "both measured frozen controller responses must remain bounded throughout the full path");
                old_fixed_square+=std::pow(a.reference-a.actual,2);new_fixed_square+=std::pow(b.reference-b.actual,2);++fixed_samples;
            }
            for(const auto& s:adaptive.rows[axis])check(std::abs(s.actual_kp/(500.0/4095)-std::round(s.actual_kp/(500.0/4095)))<1e-8 &&
                std::abs(s.actual_kd/(5.0/4095)-std::round(s.actual_kd/(5.0/4095)))<1e-8,
                "all actually ramped moving gains must remain legal protocol codes");
        }
        const double old_frozen_rms=std::sqrt(old_fixed_square/fixed_samples),new_frozen_rms=std::sqrt(new_fixed_square/fixed_samples);
        std::cout<<n<<" frozen complete same-path no-excitation RMS="<<old_frozen_rms<<" -> "<<new_frozen_rms<<'\n';
        if(new_frozen_rms>=old_frozen_rms)for(std::size_t window=0;window<adaptive.reports.size();++window){
            const auto& report=adaptive.reports[window];
            for(std::size_t axis=0;axis<n;++axis){const auto& m=report.models[axis];
                std::cout<<"window="<<window<<" axis="<<axis<<" changed="<<report.changed<<" P="<<report.gains.kp[axis]<<" D="<<report.gains.kd[axis]
                    <<" reason="<<report.reasons[axis]<<" model="<<m.reason<<" delay="<<m.delay_s<<" dt="<<m.dt_s
                    <<" a="<<m.arx_a1<<','<<m.arx_a2<<" b="<<m.arx_b0<<','<<m.arx_b1<<" bias="<<m.bias
                    <<" validation="<<m.validation_rms_rad<<" prior_validation="<<m.prior_validation_rms_rad
                    <<" predicted_cost="<<m.predicted_cost_before<<" -> "<<m.predicted_cost_after<<'\n';
            }
        }
        check(new_frozen_rms<old_frozen_rms,"the final actual controller must strictly improve the comparable full frozen path with coarse telemetry and scheduling jitter");
        auto gap=baseline.rows;
        for(auto& axis:gap)axis.erase(std::remove_if(axis.begin(),axis.end(),[](const auto& s){return s.time_s>duration-1.7 && s.time_s<duration-.8;}),axis.end());
        OnlineTrajectoryAxisModel gap_model;
        check(online_trajectory_detail::points(gap[0],{},gap_model).empty(),
            "a real telemetry outage must invalidate the old causal window, not be interpolated across as healthy jitter");
    }
    const auto three_code_baseline=run(1,false,1,true,.25,0,true,true,3);
    const auto three_code=run(1,true,1,true,.25,0,true,true,3);
    check(three_code.models>0 && three_code.updates>0 && three_code.last.kp!=three_code_baseline.last.kp,
        "a supported three-code initial P must reach a measured adjacent-code change rather than round every relative update back to the initializer");
    check(std::abs(three_code.first_p_code_step-1)<1e-8,
        "the first low-P discrete correction must explore one adjacent legal code, not a large gain jump");
    std::cout<<"three-code P: "<<three_code_baseline.last.kp[0]<<" -> "<<three_code.last.kp[0]
             <<" first_step_codes="<<three_code.first_p_code_step<<" online_updates="<<three_code.updates<<'\n';
    const auto encoded_baseline=run(1,false,1,true,.25,0,true),encoded=run(1,true,1,true,.25,0,true);
    check(encoded.updates>0 && (encoded.last.kp!=encoded_baseline.last.kp || encoded.last.kd!=encoded_baseline.last.kd),
        "legal encoded initial P/D must obtain an in-motion update on the real protocol grid");
    const double pcode=500.0/4095,dcode=5.0/4095;
    for(const auto& s:encoded.rows[0])check(std::abs(s.actual_kp/pcode-std::round(s.actual_kp/pcode))<1e-8 &&
        std::abs(s.actual_kd/dcode-std::round(s.actual_kd/dcode))<1e-8,
        "every applied protocol-ramped P/D sample must remain a legal decoded code");
    std::cout<<"protocol grid: online_updates="<<encoded.updates<<" P="<<encoded_baseline.last.kp[0]<<" -> "<<encoded.last.kp[0]
             <<" D="<<encoded_baseline.last.kd[0]<<" -> "<<encoded.last.kd[0]<<'\n';
    // A genuinely coupled inertia matrix is outside an independently fitted
    // two-state diagonal plant family. Its held-out failures must remain visible.
    const auto coupled=run(3,false,1,true,.25,.8,true,true);
    std::vector<OnlineTrajectoryReference> coupled_future;
    for(unsigned k=0;k<=180;++k)coupled_future.push_back(reference(duration+k*host_dt,3));
    std::vector<ModelAxisLimits> coupled_bounds(3);std::vector<NoiseEstimate> coupled_noise(3);
    for(auto& x:coupled_bounds){x.kp_max=500;x.kd_max=5;x.effort_budget=4;}
    for(auto& x:coupled_noise)x.encoder_step_rad=position_quantum;
    const auto coupled_report=online_trajectory_candidate(coupled.last,coupled.rows,coupled_future,
        {.035,.056,.077},coupled_bounds,coupled_noise);
    std::size_t unsupported=0;for(const auto& m:coupled_report.models)unsupported+=!m.valid;
    check(unsupported>0,"strong coupled inertia must expose unsupported diagonal prediction instead of claiming full MIMO identification");
    auto permuted_rows=coupled.rows;auto permuted_future=coupled_future;auto permuted_gain=coupled.last;
    std::reverse(permuted_rows.begin(),permuted_rows.end());
    std::reverse(permuted_gain.kp.begin(),permuted_gain.kp.end());std::reverse(permuted_gain.kd.begin(),permuted_gain.kd.end());
    for(auto& f:permuted_future){std::reverse(f.position.begin(),f.position.end());std::reverse(f.velocity.begin(),f.velocity.end());std::reverse(f.feedforward_nm.begin(),f.feedforward_nm.end());}
    const auto permuted=online_trajectory_candidate(permuted_gain,permuted_rows,permuted_future,
        {.077,.056,.035},coupled_bounds,coupled_noise);
    for(std::size_t i=0;i<3;++i)check(std::abs(permuted.gains.kp[i]-coupled_report.gains.kp[2-i])<1e-9 &&
        std::abs(permuted.gains.kd[i]-coupled_report.gains.kd[2-i])<1e-9,
        "joint permutation may only permute online candidates, never change their computation");
    const auto base=run(1,false),unknown=run(1,true,1,false);
    check(unknown.models==0 && unknown.updates==0,"missing independent input must never be filled as successful identification");
    check(unknown.last.kp==base.last.kp && unknown.last.kd==base.last.kd,"insufficient evidence preserves measured gains while path continues");
    const auto coarse_unknown=run(3,true,1,false,.25,0,true,true);
    check(coarse_unknown.models==0 && coarse_unknown.updates==0,
        "resampling irregular coarse feedback must not manufacture a measured input response when the independent input is unknown");
    auto rows=base.rows;const auto start=reference(duration,1);std::vector<OnlineTrajectoryReference> future;
    for(unsigned k=0;k<=180;++k)future.push_back(reference(duration+k*host_dt,1));
    ModelAxisLimits bound;bound.kp_max=500;bound.kd_max=5;bound.effort_budget=4;NoiseEstimate noise;noise.encoder_step_rad=1e-5;
    const auto good=online_trajectory_candidate(base.last,rows,future,{.035},{bound},{noise});
    check(good.models[0].valid,"the final stream must retain a usable local model");
    std::vector<OnlineTrajectoryReference> worker_future,late_future;
    const double unaligned_host=duration+.007;
    for(unsigned k=0;k<=43;++k)worker_future.push_back(reference(unaligned_host-.06+.02*k,1));
    for(unsigned k=1;k<=40;++k)late_future.push_back(reference(unaligned_host+.02*k,1));
    const auto aligned=online_trajectory_candidate(base.last,rows,worker_future,{.035},{bound},{noise});
    const auto missing=online_trajectory_candidate(base.last,rows,late_future,{.035},{bound},{noise});
    check(aligned.models[0].valid && aligned.reasons[0]!="insufficient_future_taught_reference_model_updates_continue",
        "worker time 7ms off the telemetry grid must retain a covered first prediction with time-60ms future data");
    check(missing.models[0].valid && missing.reasons[0]=="insufficient_future_taught_reference_model_updates_continue",
        "future data beginning after the first prediction must be reported missing rather than falsely counting as tuning");
    const auto viscous=run(1,false,1,true,3);
    const auto different=online_trajectory_candidate(base.last,viscous.rows,future,{.035},{bound},{noise});
    check(different.models[0].valid,"different physical damping must still obtain an identified local model");
    check(std::abs(different.models[0].arx_a1-good.models[0].arx_a1)>1e-4 ||
          std::abs(different.models[0].arx_a2-good.models[0].arx_a2)>1e-4,
          "different real trajectory dynamics must change the measured model, despite identical URDF prior");
    check(different.gains.kp!=good.gains.kp || different.gains.kd!=good.gains.kd,
          "actual measured plant changes must affect the computed P/D candidate");
    auto weak=rows;for(auto& s:weak[0]){s.independent_disturbance_nm*=1e-14;s.planned_independent_disturbance_nm=s.independent_disturbance_nm;}
    check(!online_trajectory_candidate(base.last,weak,future,{.035},{bound},{noise}).models[0].valid,
          "vanishing independent excitation must not be promoted to an identified model");
    auto unmodeled=rows;
    // An unresolved independent cross-axis disturbance starts in the holdout
    // portion. A diagonal local model must expose its prediction failure.
    for(auto& s:unmodeled[0])if(s.time_s>duration-.9){
        const double x=s.time_s-(duration-.9);s.actual+=.35*(1-std::cos(13*x));s.velocity+=4.55*std::sin(13*x);
    }
    check(!online_trajectory_candidate(base.last,unmodeled,future,{.035},{bound},{noise}).models[0].valid,
          "unmodeled coupling/disturbance in held-out data must not count as a validated diagonal model");
    auto unknown_ff=rows;for(auto& s:unknown_ff[0])s.feedforward_known=false;
    check(!online_trajectory_candidate(base.last,unknown_ff,future,{.035},{bound},{noise}).models[0].valid,
          "unknown executed feedforward must not become an identified commanded-input response");
    auto duplicated=rows;for(auto& s:duplicated[0])s.feedback_sequence=1;
    const auto repeated=online_trajectory_candidate(base.last,duplicated,future,{.035},{bound},{noise});
    check(!repeated.changed && !repeated.models[0].valid,"repeated feedback cannot create online information");
    auto clipped=rows;for(auto& s:clipped[0])if(s.time_s>duration-2)s.limited=true;
    const auto limited=online_trajectory_candidate(base.last,clipped,future,{.035},{bound},{noise});
    check(!limited.changed && !limited.models[0].valid,"clipped excitation and its causal history cannot enter the IV fit");
    auto suppressed=rows;
    for(auto& s:suppressed[0])if(s.time_s>duration-2)s.independent_disturbance_nm=0;
    check(!online_trajectory_candidate(base.last,suppressed,future,{.035},{bound},{noise}).models[0].valid,
          "accepted excitation altered by a boundary/headroom guard must be removed with its causal history");
    auto stale=rows;for(auto& s:stale[0])s.feedback_time_s=s.command_time_s-.2;
    check(!online_trajectory_candidate(base.last,stale,future,{.035},{bound},{noise}).changed,"stale feedback cannot tune gains");
    for(double time:{.1,.7,1.5,3.8}){
        const double inward=online_trajectory_injection(time,0,.1,10,1,2,0,.2,.005,10);
        const double opposite=online_trajectory_injection(time,0,.1,10,1,2,.2,0,.005,10);
        check(inward>=0 && opposite<=0,"an endpoint only admits inward known excitation without symmetric room");
    }
    check(online_trajectory_injection(1,0,.1,10,1,2,0,0,.005,10)==0,"no physical position room disables injection without fault");
    check(online_trajectory_injection(0,0,.1,10,1,2,.2,.2,.005,10)==0,"known input fades in without a command step");
    std::cout<<"PASS online trajectory STR: 1/3/7 sampled MIT plants, coarse protocol feedback and irregular host timing, in-motion real updates, raw evidence preserved, gaps/unknown/limited/stale/duplicate evidence, single-sided endpoint input\n";
}
