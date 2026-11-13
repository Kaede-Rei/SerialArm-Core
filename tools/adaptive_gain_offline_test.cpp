// Deterministic tests of the header used by the real calibration call chain.
// No hardware libraries, robot commands, or user Profiles are involved.
#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include "serial_arm/dynamics/gain_trajectory_tuning.hpp"
#include "serial_arm/dynamics/calibration_retime.hpp"
#include <iostream>
#include <functional>
#include <fstream>
#include <sstream>
#include <stdexcept>
using namespace serial_arm::servo_tuning;
void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
void close(double a,double b,double tolerance,const char* message){check(std::abs(a-b)<=tolerance,message);}

bool independent_motion_damping_preserved(const Gains& previous,const Gains& next,
    const std::vector<ModelAxisLimits>& limits) {
    if(previous.kp.size()!=next.kp.size() || previous.kd.size()!=next.kd.size() || limits.size()!=next.kp.size())return false;
    for(std::size_t i=0;i<limits.size();++i) {
        if(next.kd[i]>limits[i].kd_max+1e-12)return false;
        // A new P comparison does not authorize unmeasured D growth.
        if(next.kd[i]!=previous.kd[i])return false;
    }
    return true;
}

void check_shape_preserving_retime() {
    namespace retime=serial_arm::calibration_retime;
    const std::vector<std::vector<double>> dwell{{.02},{.01},{0},{0},{0}};
    const auto stationary=retime::interpolate(dwell,2.5,.01,1);
    close(stationary.position[0],0,1e-14,"nonnegative zero-endpoint dwell must not interpolate through the physical lower stop");
    close(stationary.velocity[0],0,1e-14,"a genuine zero platform must have zero reference velocity");
    for(double value:{.02,-.02,.1,-.1}) {
        const std::vector<std::vector<double>> plateau{{value+.01},{value},{value},{value},{value+.01}};
        for(unsigned step=0;step<=10000;++step) {
            const auto exact=retime::interpolate(plateau,1+step*.0001,.01,1);
            check(exact.position[0]==value && exact.velocity[0]==0,
                "a nonzero constant platform must retain its exact endpoint value and zero velocity at strict Safety boundaries");
        }
    }
    for(const auto& knots:std::vector<std::vector<std::vector<double>>>{dwell,{{0},{0},{.01},{.02},{.02}},{{0},{.01},{.02},{.01},{0}},{{.02},{.019},{.001},{0},{0}}}) {
        for(std::size_t k=0;k+1<knots.size();++k)for(unsigned step=0;step<=100;++step) {
            const double progress=k+step*.01;
            const auto sample=retime::interpolate(knots,progress,.01,.65);
            check(sample.position[0]>=std::min(knots[k][0],knots[k+1][0])-1e-14 &&
                  sample.position[0]<=std::max(knots[k][0],knots[k+1][0])+1e-14,
                "plateaus, extrema and unequal same-direction slopes must remain within each pair of demonstrated values");
            if(progress>1e-5 && progress<knots.size()-1-1e-5) {
                constexpr double h=1e-7;
                const double numeric=(retime::interpolate(knots,progress+h,.01,.65).position[0]-
                    retime::interpolate(knots,progress-h,.01,.65).position[0])/(2*h)*.65/.01;
                close(sample.velocity[0],numeric,2e-6,"bounded Hermite playback velocity must remain the true position derivative");
            }
        }
        close(retime::interpolate(knots,0,.01,1).velocity[0],0,1e-14,"retimed playback must start at rest");
        close(retime::interpolate(knots,knots.size()-1,.01,1).velocity[0],0,1e-14,"retimed playback must end at rest");
    }
    std::vector<std::vector<double>> input;
    for(unsigned k=0;k<=400;++k){const double t=k/400.0;input.push_back({.2*std::max(0.0,1-2*t),.1*t});}
    const std::vector<double> max_v{.25,.25},max_a{.6,.6};
    const auto planned=retime::plan(input,max_v,max_a,200);
    for(const auto& q:planned.positions)check(q[0]>=-1e-14 && q[0]<=.2+1e-14 && q[1]>=-1e-14 && q[1]<=.1+1e-14,
        "the geometric smoothing and resampling stage must preserve nonnegative demonstrated endpoint envelopes");
    for(std::size_t k=0;k+1<planned.positions.size();++k)for(unsigned step=0;step<=20;++step) {
        const double progress=k+step*.05;const auto q=retime::interpolate(planned.positions,progress,planned.sample_dt,1);
        for(unsigned axis=0;axis<2;++axis) {
            check(q.position[axis]>=std::min(planned.positions[k][axis],planned.positions[k+1][axis])-1e-14 &&
                  q.position[axis]<=std::max(planned.positions[k][axis],planned.positions[k+1][axis])+1e-14,
                "production interpolation must stay within the same bounded time-sample geometry used for planning");
            check(std::abs(q.velocity[axis])<=max_v[axis]+1e-8,"shape-preserving retiming must retain the configured actual velocity limit");
            if(progress>1e-4 && progress<planned.positions.size()-1-1e-4) {
                constexpr double h=1e-5;
                const auto before=retime::interpolate(planned.positions,progress-h,planned.sample_dt,1);
                const auto after=retime::interpolate(planned.positions,progress+h,planned.sample_dt,1);
                const double acc=(after.velocity[axis]-before.velocity[axis])/(2*h*planned.sample_dt);
                check(std::abs(acc)<=max_a[axis]+1e-5,"actual bounded Hermite derivatives must retain the configured acceleration limit");
            }
        }
    }
    std::vector<std::vector<double>> fixed_axis;
    for(unsigned k=0;k<=100;++k)fixed_axis.push_back({.02,.001*k});
    const auto fixed_plan=retime::plan(fixed_axis,max_v,max_a,200);
    for(const auto& q:fixed_plan.positions)check(q[0]==.02,"geometric smoothing and cubic resampling must preserve an exact nonzero stationary axis");
    for(std::size_t k=0;k+1<fixed_plan.positions.size();++k) {
        const auto q=retime::interpolate(fixed_plan.positions,k+.0034,fixed_plan.sample_dt,1);
        check(q.position[0]==.02 && q.velocity[0]==0,"production playback must preserve exact nonzero platforms after geometric retiming");
    }
    std::cout<<"shape-preserving geometric/production Hermite: exact zero dwell, local node bounds, true derivatives and unchanged v/a limits\n";
}

std::vector<Sample> spectral_experiment(const Requirements& req,const MitExperiment& model,
    double damping,double delay=0,bool inverse_dynamics=false,double noise=0) {
    std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>((req.probe_duration_s+req.tail_s)*500);++k) {
        Sample s;s.time_s=k*.002;s.new_measurement=true;s.feedforward_known=true;
        s.independent_disturbance_known=true;s.feedback_sequence=k+1;s.command_time_s=s.feedback_time_s=s.time_s;
        for(std::size_t j=0;j<3;++j) {
            const double w=2*pi*req.frequencies_hz[j],amplitude=req.amplitude_rad*(j==0?.4:j==1?.35:.25);
            const std::complex<double> reference=model.kind==ExcitationKind::ReferenceTracking?amplitude:0;
            const std::complex<double> disturbance=model.kind==ExcitationKind::IndependentTorqueDisturbance?amplitude:0;
            const std::complex<double> ff=inverse_dynamics?std::complex<double>{-model.inertia_prior*w*w, damping*w}*reference:0;
            const auto denominator=std::complex<double>{model.kp+model.gravity_gradient-model.executed_state_feedforward_gradient-model.inertia_prior*w*w,(model.kd+damping)*w};
            const auto output=(model.kp*reference+std::complex<double>{0,model.kd*w}*reference+ff+disturbance)/denominator*std::exp(std::complex<double>{0,-w*delay});
            const auto at=[&](std::complex<double> x){return (x*std::exp(std::complex<double>{0,w*s.time_s})).imag();};
            s.reference+=at(reference);s.reference_velocity+=at(std::complex<double>{0,w}*reference);
            s.actual+=at(output);s.velocity+=at(std::complex<double>{0,w}*output);
            if(j==0)s.applied_feedforward_nm=0;
            s.applied_feedforward_nm+=at(ff+model.executed_state_feedforward_gradient*output);s.independent_disturbance_nm+=at(disturbance);
        }
        s.actual+=noise*std::sin(83.7*s.time_s);rows.push_back(s);
    }
    return rows;
}

// Integrate an MIT rotary plant with friction, sample/command delay and encoder
// noise, then compare complete gain sets under identical exogenous conditions.
std::vector<Sample> simulate_mit(const Requirements& req,const Gains& gains,double inertia=.2) {
    std::vector<Sample> rows;std::vector<double> commands(12,0);double q=0,v=0;
    constexpr double dt=.001,friction=.006,viscous=.15;
    for(unsigned k=0;k<=static_cast<unsigned>((req.probe_duration_s+req.tail_s)/dt);++k) {
        const double time=k*dt;const auto reference=probe(time,req);
        const double measured=q+1e-6*std::sin(83.7*time);
        const double raw=gains.kp[0]*(reference.first-measured)+gains.kd[0]*(reference.second-v);
        const double command=std::clamp(raw,-8.0,8.0);
        const double applied=commands[k%commands.size()];commands[k%commands.size()]=command;
        const double resistance=viscous*v+friction*std::tanh(v/.001);
        v+=(applied-resistance)/inertia*dt;q+=v*dt;
        Sample row;row.time_s=time;row.reference=reference.first;row.reference_velocity=reference.second;
        row.actual=measured;row.velocity=v;row.new_measurement=true;row.limited=raw!=command;
        row.applied_feedforward_nm=0;row.feedforward_known=true;row.feedback_sequence=k+1;
        row.command_time_s=row.feedback_time_s=time;rows.push_back(row);
    }
    return rows;
}
double interior_error(const std::vector<Sample>& rows,double end) {
    double square=0,count=0;for(const auto& row:rows)if(row.time_s>.7 && row.time_s<end-.7) {
        square+=std::pow(row.actual-row.reference,2);++count;
    }
    return std::sqrt(square/count);
}

// Actual zero-velocity sticking: applied torque below static friction produces
// exactly zero travel; slipping uses kinetic friction and stops on reversal.
struct StickSlipAxis {double inertia,static_friction,viscous,negative_static_friction{0},q{0},v{0};std::vector<double> delay=std::vector<double>(12,0);std::size_t tick{0};
    double physical_lower_stop{-std::numeric_limits<double>::infinity()};};
std::vector<Sample> stick_slip_trial(StickSlipAxis& axis,const Requirements& req,double kp,double kd,
    const ModelAxisLimits& limit,double hardware_effort,double independent_amplitude=0) {
    std::vector<Sample> rows;constexpr double dt=.001,encoder=25.0/65536;
    for(unsigned k=0;k<=static_cast<unsigned>((req.probe_duration_s+req.tail_s+.5)/dt);++k) {
        const double time=k*dt-.5;const auto excitation=probe(time,req);
        const std::pair<double,double> r=independent_amplitude>0?std::pair<double,double>{0,0}:excitation;
        const double independent=independent_amplitude>0?independent_amplitude*excitation.first/req.amplitude_rad:0;
        const double measured=encoder*std::round(axis.q/encoder);
        const double torque=kp*(r.first-measured)+kd*(r.second-axis.v)+independent;
        check(std::abs(torque)<=hardware_effort,"motion establishment must never exceed actual hardware/Safety effort");
        const double applied=axis.delay[axis.tick%axis.delay.size()];axis.delay[axis.tick++%axis.delay.size()]=torque;
        const double negative=axis.negative_static_friction>0?axis.negative_static_friction:axis.static_friction;
        const double stopping_threshold=applied>=0?axis.static_friction:negative;
        if(std::abs(axis.v)<1e-5 && std::abs(applied)<=stopping_threshold)axis.v=0;
        else {
            const double direction=std::abs(axis.v)>1e-5?std::copysign(1.0,axis.v):std::copysign(1.0,applied);
            const double kinetic=.75*(direction>=0?axis.static_friction:negative);
            const double next=axis.v+(applied-kinetic*direction-axis.viscous*axis.v)/axis.inertia*dt;
            if(next*axis.v<0 && std::abs(applied)<=stopping_threshold)axis.v=0;else axis.v=next;
        }
        axis.q+=axis.v*dt;
        if(axis.q<axis.physical_lower_stop){axis.q=axis.physical_lower_stop;axis.v=std::max(0.0,axis.v);}
        if(std::isfinite(limit.reference_min_offset_rad) || std::isfinite(limit.reference_max_offset_rad))
            check(axis.q>=limit.reference_min_offset_rad-1e-8 && axis.q<=limit.reference_max_offset_rad+1e-8,"actual motion must remain in the observed fixed-origin interval");
        else check(std::abs(axis.q)<=limit.position_margin_rad,"motion establishment must remain in its fixed measured-center envelope");
        if(time<0 || k%5)continue;
        Sample row;row.time_s=time;row.reference=r.first;row.reference_velocity=r.second;row.actual=measured;row.velocity=axis.v;
        row.new_measurement=true;row.feedforward_known=true;row.applied_feedforward_nm=0;row.independent_disturbance_known=true;
        row.independent_disturbance_nm=independent;
        row.feedback_sequence=k+1;row.command_time_s=row.feedback_time_s=time;rows.push_back(row);
    }
    return rows;
}
void check_six_axis_motion_establishment() {
    // 1794213550334 mapped model envelope and unchanged actuator/Safety scales.
    const std::vector<double> inertia{.0918663659,.4419690902,.2448401088,.02407264437,.007283483283,.001123771029};
    const std::vector<double> gradients{0,10.469,5.922,1.2286,.45805,.0545968};
    const std::vector<double> hardware{28,28,28,10,10,10},kp_max(6,500),kd_max(6,5);
    std::vector<ModelAxisLimits> limits;for(unsigned i=0;i<6;++i)limits.push_back({.04,1.62,3.24,.25*hardware[i],kp_max[i],kd_max[i],{500.0/4095,5.0/4095}});
    std::vector<std::vector<double>> path;for(unsigned k=0;k<1001;++k){std::vector<double> q;for(unsigned i=0;i<6;++i)q.push_back(.005*std::sin(2*pi*(.25+.08*i)*k*.01));path.push_back(q);}
    const auto demand=path_response_demand(path,.01,pi/1800);
    // Gravity is supplied by the actual execution path; its nominal residual
    // stiffness is zero, rather than the uncompensated |dG/dq| envelope.
    const auto initialized=bootstrap_model_initialization({inertia},std::vector<double>(6,0),limits,demand);Gains current=initialized.gains;
    const std::vector<double> static_friction{.46,.84,1.27,.152,.168,.162},negative_friction{.40,.22,1.67,.143,.140,.134};
    std::vector<StickSlipAxis> plant;for(unsigned i=0;i<6;++i)plant.push_back({inertia[i],static_friction[i],i<3?.15:.01,negative_friction[i]});
    // As in 1794213550334, stationary samples initially expose one code only;
    // resolution is learned from actual subsequent movement, not invented.
    Requirements req;std::vector<NoiseEstimate> noise(6);
    std::vector<Requirements> plans;for(unsigned i=0;i<6;++i){const auto p=short_probe_plan(req,limits[i],current.kp[i],current.kd[i],noise[i],false,220);check(p.valid,"initial six-axis probe must be bounded");plans.push_back(p.requirements);}
    double elapsed=0;bool established=false,passed_weak_phase=false;unsigned measured_trials=0;std::vector<bool> initially_stuck(6,false);
    CalibrationBudget budget(28.22,CalibrationMode::QuickAll);constexpr double reserved_verification=153;
    MotionEstablishmentState establishment_state;
    while(budget.can_start(elapsed,short_trial_cost(plans,req.tail_s),reserved_verification)) {
        std::vector<ResponseModel> models;std::vector<Metrics> metrics;std::vector<MotionObservation> motion;
        for(unsigned i=0;i<6;++i) {
            const auto rows=stick_slip_trial(plant[i],plans[i],current.kp[i],current.kd[i],limits[i],hardware[i]);
            auto a=assess_trajectory(rows,plans[i],noise[i],plans[i].probe_duration_s);a.metrics.applied_input_peak_nm=applied_pd_input_peak(rows,current.kp[i],current.kd[i]);
            auto m=plans[i].motion_probe?motion_probe_evidence(rows,plans[i],current.kp[i],current.kd[i]):
                identify_mit(rows,plans[i],{current.kp[i],current.kd[i],inertia[i],0,ExcitationKind::ReferenceTracking});
            m.sustained_oscillation=a.metrics.oscillating;m.dynamic_overshoot=a.dynamic_overshoot_rad>req.overshoot_rad;
            if(!measured_trials)initially_stuck[i]=a.metrics.actual_span==0;
            passed_weak_phase=passed_weak_phase || m.reason=="low_spectral_fit_quality" ||
                (a.metrics.actual_span>0 && (!m.tracking_valid || !a.motion.qualified));
            models.push_back(m);metrics.push_back(a.metrics);motion.push_back(a.motion);
        }
        elapsed+=short_trial_cost(plans,req.tail_s);++measured_trials;
        const auto step=motion_establishment_step(current,inertia,limits,noise,models,metrics,motion,plans,req,false,
            budget.remaining(elapsed)-reserved_verification-budget.exit_reserve_s,&establishment_state);
        check(!step.feedback_fault,"static friction and low coherence must not be misclassified as a feedback fault");
        if(step.motion_established) {
            established=true;
            if(step.spectral_confirmation_required) {
                plans=comparable_probe_plans(current,current,establishment_state.spectral_plans,limits,noise,req,false,
                    budget.remaining(elapsed)-reserved_verification-budget.exit_reserve_s);
                for(unsigned i=0;i<6;++i) {
                    check(!plans[i].motion_probe,"established single-tone motion must restore a separate actual three-frequency experiment");
                    const auto rows=stick_slip_trial(plant[i],plans[i],current.kp[i],current.kd[i],limits[i],hardware[i]);
                    (void)identify_mit(rows,plans[i],{current.kp[i],current.kd[i],inertia[i],0,ExcitationKind::ReferenceTracking});
                }
                elapsed+=short_trial_cost(plans,req.tail_s);++measured_trials;
            }
            break;
        }
        if(!step.retry) {
            check(!step.established && !step.motion_established && !step.feedback_fault,
                "an unresolved optional symmetric probe must preserve honest insufficiency instead of a fabricated pass or Safety failure");
            break;
        }
        for(unsigned i=0;i<6;++i)check(step.gains.kp[i]<=kp_max[i] && step.gains.kd[i]<=kd_max[i],"establishment must preserve actual gain Safety ceilings");
        current=step.gains;plans=step.plans;
    }
    check(initially_stuck[0] && initially_stuck[2] && initially_stuck[5],"regression must start with truly stuck heavy and wrist axes");
    check(measured_trials>4 && elapsed<budget.hard_limit_s-reserved_verification,
        "the optional legacy short-probe scheduler must retain verification and exit time even if it cannot establish all axes");
    check(passed_weak_phase,"regression must traverse weak nonlinear response rather than jump directly to a passing plant");
    if(!established) {
        std::cout<<"optional legacy symmetric probe: honest incomplete motion after "<<measured_trials<<" trials, "<<elapsed
            <<" s; no automatic D escalation, no false pass, original validation/exit reserve retained\n";
        return;
    }

    const double establishment_elapsed=elapsed;
    // Independent torque uses the current established whole gain set, not the
    // tiny unmeasured model seed. Supplement only missing responses, bounded
    // by the existing Robot 5% overlay ceiling and the original wall budget.
    std::vector<double> torque(6),torque_ceiling(6);
    std::vector<StickSlipAxis> disturbance_plant;
    for(unsigned i=0;i<6;++i) {
        const auto plan=short_probe_plan(req,limits[i],current.kp[i],current.kd[i],noise[i],false,150,plans[i].amplitude_rad,plans[i].frequencies_hz.front());
        torque_ceiling[i]=std::min(.05*hardware[i],limits[i].effort_budget);torque[i]=std::min(torque_ceiling[i],plan.torque_amplitude_nm);
        disturbance_plant.push_back({inertia[i],static_friction[i],i<3?.15:.01,negative_friction[i]});
    }
    const double legacy_wrist_torque=.5*initialized.gains.kp[5]*std::min(plans[5].amplitude_rad,2*req.hold_error_rad);
    check(torque[5]>10*legacy_wrist_torque,"current measured wrist gains must plan independent torque far above the obsolete seed-normalized input");
    bool wrist_observable=false,heavy_unobservable_at_cap=false;unsigned disturbance_trials=0;
    while(budget.can_start(elapsed,short_trial_cost(plans,req.tail_s),reserved_verification)) {
        std::vector<ResponseModel> models;std::vector<Metrics> metrics;DisturbanceAssessment heavy;
        for(unsigned i=0;i<6;++i) {
            const auto rows=stick_slip_trial(disturbance_plant[i],plans[i],current.kp[i],current.kd[i],limits[i],hardware[i],torque[i]);
            for(const auto& row:rows){check(row.reference==0 && row.reference_velocity==0,"known independent disturbance must retain its fixed reference definition");check(std::abs(row.independent_disturbance_nm)<=.05*hardware[i],"actual injected torque must preserve Robot five-percent Safety ceiling");}
            auto model=identify_mit(rows,plans[i],{current.kp[i],current.kd[i],inertia[i],0,ExcitationKind::IndependentTorqueDisturbance});
            const auto a=assess_trajectory(rows,plans[i],noise[i],plans[i].probe_duration_s);
            models.push_back(model);metrics.push_back(a.metrics);
            if(i==5)wrist_observable=wrist_observable || model.disturbance_valid;
            if(i==2)heavy=assess_disturbance(rows,plans[i],noise[i],plans[i].probe_duration_s,req.hold_error_rad);
        }
        elapsed+=short_trial_cost(plans,req.tail_s);++disturbance_trials;
        const auto next=revise_disturbance_input(torque,torque_ceiling,models,metrics,budget.remaining(elapsed),short_trial_cost(plans,req.tail_s),reserved_verification+budget.exit_reserve_s);
        check(!next.feedback_fault,"static friction locking must remain missing response evidence, not communication failure");
        if(torque[2]==torque_ceiling[2])heavy_unobservable_at_cap=!models[2].disturbance_valid && !heavy.motion_observed && heavy.static_measurement_complete && !heavy.passed &&
            heavy.reason=="independent_response_below_motion_floor_static_behavior_only";
        if(!next.retry)break;
        torque=next.amplitudes;
    }
    check(wrist_observable,"known independent wrist torque must produce observable response after actual motion establishment");
    check(heavy_unobservable_at_cap,"heavy-axis stiction above five-percent overlay must preserve insufficient stiffness evidence, never a fake high-stiffness pass");
    std::cout<<"independent stick/slip torque: "<<disturbance_trials<<" bounded trials; wrist response observed, heavy-axis 5% cap insufficient and retained as missing evidence\n";
    // Reproduce the recorded low-coherence reason without promoting it to an
    // identification success. Health and response validity are different.
    std::vector<ResponseModel> weak(6);for(auto& m:weak)m.reason="response_below_encoder_noise_floor";weak[1].reason="low_spectral_fit_quality";
    std::vector<Metrics> healthy(6);for(auto& m:healthy){m.samples=m.measurement_samples=800;m.continuous=true;}
    std::vector<MotionObservation> quiet(6);
    const auto recorded=motion_establishment_step(initialized.gains,inertia,limits,noise,weak,healthy,quiet,plans,req,false,200);
    check(recorded.retry && !recorded.established && !recorded.feedback_fault,"recorded joint2 low_spectral_fit_quality must schedule bounded establishment, not fail or fabricate identification");
    auto invalid_input=weak;invalid_input[1].reason="invalid_command_time";
    const auto invalid_decision=motion_establishment_step(current,inertia,limits,noise,invalid_input,healthy,quiet,plans,req,false,200);
    check(invalid_decision.feedback_fault && !invalid_decision.retry,"bad experiment timestamps must not justify stronger commands even if count summaries look healthy");
    auto lost=healthy;lost[2].continuous=false;
    check(motion_establishment_step(current,inertia,limits,noise,weak,lost,quiet,plans,req,false,200).feedback_fault,"lost feedback must still terminate establishment");
    check(!motion_establishment_step(current,inertia,limits,noise,weak,healthy,quiet,plans,req,false,0).retry,"expired measurement budget must not launch another establishment trial");
    auto first=current,second=current;first.kp[2]=96.8;second.kp[2]=193.6;
    auto wider=plans;wider[2].amplitude_rad=.028;
    const auto common=comparable_probe_plans(first,second,wider,limits,noise,req,false,60);
    check(common[2].amplitude_rad<wider[2].amplitude_rad,"stronger candidate may require a new shared comparison excitation");
    check(comparable_probe_plans(second,first,wider,limits,noise,req,false,60)[2].amplitude_rad==common[2].amplitude_rad,"common plan must be independent of candidate/incumbent order");

    // An actuator that cannot break static friction within its original bounds
    // ends with insufficient evidence; reaching a cap never becomes a pass.
    const ModelAxisLimits blocked_limits{.04,1.62,3.24,2.5,500,5,{500.0/4095,5.0/4095}};
    const auto blocked_seed=model_initialization({{inertia[5]}},{gradients[5]},{blocked_limits});
    Gains blocked_gain=blocked_seed.gains;StickSlipAxis blocked{inertia[5],12,.01,12};
    std::vector<Requirements> blocked_plans{short_probe_plan(req,blocked_limits,blocked_gain.kp[0],blocked_gain.kd[0],NoiseEstimate{},false,100).requirements};
    CalibrationBudget blocked_budget(10,CalibrationMode::QuickAll);double blocked_elapsed=0;bool retried=true,blocked_passed=false;unsigned blocked_trials=0;
    while(blocked_budget.can_start(blocked_elapsed,short_trial_cost(blocked_plans,req.tail_s),40)) {
        const auto rows=stick_slip_trial(blocked,blocked_plans[0],blocked_gain.kp[0],blocked_gain.kd[0],blocked_limits,10);
        const auto a=assess_trajectory(rows,blocked_plans[0],NoiseEstimate{},blocked_plans[0].probe_duration_s);
        const auto m=identify_mit(rows,blocked_plans[0],{blocked_gain.kp[0],blocked_gain.kd[0],inertia[5],0,ExcitationKind::ReferenceTracking});
        blocked_elapsed+=short_trial_cost(blocked_plans,req.tail_s);++blocked_trials;
        const auto next=motion_establishment_step(blocked_gain,{inertia[5]},{blocked_limits},{NoiseEstimate{}},{m},{a.metrics},{a.motion},blocked_plans,req,false,blocked_budget.remaining(blocked_elapsed)-42);
        blocked_passed=next.established;check(!blocked_passed,"unreachable stiction must never be promoted to an established response");
        retried=next.retry;if(!retried)break;blocked_gain=next.gains;blocked_plans=next.plans;
    }
    check(blocked_trials>4 && blocked_elapsed+40<blocked_budget.hard_limit_s &&
        (!retried || !blocked_budget.can_start(blocked_elapsed,short_trial_cost(blocked_plans,req.tail_s),40)),
        "unreachable static friction must stop at capability or original time budget with verification/exit reserve");
    std::cout<<"six-axis static stick/slip establishment: "<<measured_trials<<" complete trials, "<<establishment_elapsed<<" s; all axes established, limits unchanged, verification reserved\n";
}

void check_protocol_demand_and_working_domain() {
    const double kp_step=500.0/4095,kd_step=5.0/4095;
    for(unsigned code=0;code<=4095;++code)for(const auto maximum:{5.0,500.0}) {
        const double step=maximum/4095,value=quantize_gain(code*step,step,maximum,true);
        check(gain_grid_code(value,step,maximum)==code,"every mapped MIT gain code must survive actual float normalize/nearest-code encoding");
        check(quantize_gain(value,step,maximum,true)==value && quantize_gain(value,step,maximum)==value,
            "gain grid application must be idempotent, never lose or accumulate a code");
    }
    check(static_cast<unsigned>(static_cast<float>(.0829971086)/500*4095)==0,"1794217032269 wrist seed must reproduce the historical truncate backend's zero position-gain code");
    const auto positive=quantize_gain(.0829971086,kp_step,500,true);
    check(gain_grid_code(positive,kp_step,500)==1,"positive model seed below one code must use a real positive command code");
    check(quantize_gain(.0829971086,0,500)==.0829971086,"unknown-driver continuous model preview must remain supported");
    const ModelAxisLimits axis{.04,1.62,3.24,7,500,5,{kp_step,kd_step}};
    const auto domain=gain_working_domain(axis,.02,.05);
    close(domain.kp_max,350,1e-12,"working stiffness ceiling must derive from effort/error, independent of old PD settings");
    const auto bounded=constrain_candidate(500,5,axis,.02,.05);
    check(bounded.first*.02+bounded.second*.05<=7,"combined proportional and derivative demand must fit feedback effort");
    check(bounded.first<500 && bounded.second<=5,"automatic work domain and protocol maxima must remain distinct");
    const auto paired=initial_model_pair(.09117253,0,500,.7,axis,.016,1.62);
    check(paired.first*.016+paired.second*1.62<=7 && paired.second/(2*std::sqrt(.09117253*paired.first))>=.7,
        "initial effort projection must preserve damping instead of producing the observed high-Kp/tiny-Kd Profile candidate");
    const ModelAxisLimits initial_damping_cap{.04,1.62,3.24,7,500,.4,{}};
    const auto start=initial_model_pair(.001123771,0,200,.7,initial_damping_cap,.016,.1);
    check(start.second/(2*std::sqrt(.001123771*start.first))>=.7-1e-12,"protocol Kd cap may lower only the initial response frequency to preserve nominal damping");
    ResponseModel continuing;continuing.tracking_valid=continuing.identifiable=true;continuing.inertia_estimate=.001123771;continuing.tracking_peak_error_rad=.02;continuing.reference_lag_s=.05;
    check(adaptive_candidate(Gains{{start.first},{start.second}},{continuing},{initial_damping_cap}).gains.kp[0]>start.first,
        "the initialization damping curve must never freeze later evidence-driven Kp search at the Kd protocol ceiling");
    std::vector<std::vector<double>> slow,fast;
    for(unsigned k=0;k<2001;++k){const double t=k*.005;slow.push_back({.005*std::sin(2*pi*.25*t),.005*std::sin(2*pi*.25*t)});
        fast.push_back({.005*std::sin(2*pi*1.5*t),.005*std::sin(2*pi*1.5*t)});}
    auto slow_demand=path_response_demand(slow,.005,pi/1800),fast_demand=path_response_demand(fast,.005,pi/1800);
    const std::vector<ModelAxisLimits> limits{axis,axis};
    const auto first=model_initialization({{.2,.001123771}},{0,0},limits,slow_demand);
    const auto second=model_initialization({{.2,.001123771}},{0,0},limits,fast_demand);
    check(second.gains.kp[0]>first.gains.kp[0] && second.gains.kp[1]>first.gains.kp[1],"actual path response demand must affect heavy and wrist model candidates");
    const auto boot_slow=bootstrap_model_initialization({{.2,.001123771}},{0,0},limits,slow_demand);
    const auto boot_fast=bootstrap_model_initialization({{.2,.001123771}},{0,0},limits,fast_demand);
    check(boot_slow.gains.kp==boot_fast.gains.kp && boot_slow.gains.kd==boot_fast.gains.kd && boot_fast.gains.kp[1]<second.gains.kp[1],
        "high path acceleration/error demand must remain an offline target, never become the unmeasured motion bootstrap gain");
    check(gain_grid_code(first.gains.kp[1],kp_step,500)>0,"light-axis response initialization must not silently encode zero");
    const auto uncompensated=model_initialization({{.2,.001123771}},{2,.0546},limits,slow_demand);
    check(uncompensated.gains.kp[0]>first.gains.kp[0],"only uncompensated gravity-gradient residual belongs in stiffness demand");
    slow_demand.coupled_acceleration_torque={.10,.03};
    slow_demand.mass_matrix_samples={{{.2,.005},{.005,.001123771}}};
    const auto coupled=model_initialization({{.2,.001123771}},{0,0},limits,slow_demand);
    check(coupled.gains.kp[1]>first.gains.kp[1] && coupled.nominal_mode_frequency_hz.size()==2 && coupled.nominal_mode_damping_ratio.size()==2,
        "off-diagonal inertia and acceleration of other joints must enter candidates and complete-matrix modal diagnostics");
    slow_demand.mass_matrix_samples={{{.2,.2},{.2,.001123771}}};bool invalid=false;
    try{model_initialization({{.2,.001123771}},{0,0},limits,slow_demand);}catch(const std::invalid_argument&){invalid=true;}
    check(invalid,"nonpositive coupled inertia must not produce an initialized candidate");
    auto modal_demand=path_response_demand(slow,.005,pi/1800);
    const auto diagonal_start=model_initialization({{.2,.2}},{0,0},limits,modal_demand);
    modal_demand.mass_matrix_samples={{{.2,.15},{.15,.2}}};
    const auto filtered=model_initialization({{.2,.2}},{0,0},limits,modal_demand);
    check(*std::min_element(filtered.nominal_mode_damping_ratio.begin(),filtered.nominal_mode_damping_ratio.end())>=modal_demand.damping_ratio*(1-1e-6),
        "complete coupled inertia must actively preserve the initial modal damping target");
    check(filtered.gains.kd!=diagonal_start.gains.kd || filtered.gains.kp!=diagonal_start.gains.kp,
        "coupled low damping must alter the actual initial gain set rather than only emit a warning");
    auto loose=axis;loose.max_velocity=100;loose.max_acceleration=1000;
    auto strict=axis;strict.max_velocity=.01;strict.max_acceleration=.02;
    strict.effort_budget=.1;
    const auto recovery=recovery_response_demand(2,pi/1800,.7);
    const auto unbounded_prior=model_initialization({{.243,.001124}},{0,0},{loose,loose},recovery);
    const auto artificial_constraint=model_initialization({{.243,.001124}},{0,0},{strict,strict},recovery);
    check(unbounded_prior.unconstrained_kp==artificial_constraint.unconstrained_kp && unbounded_prior.unconstrained_kd==artificial_constraint.unconstrained_kd,
        "artificial velocity/acceleration/effort limits may constrain execution but must never change the physical/task model candidate");
    check(unbounded_prior.gains.kp!=artificial_constraint.gains.kp,"an actual effort constraint must be reported through the constrained candidate, not by modifying its physical prior");
    check(unbounded_prior.unconstrained_kp[0]>unbounded_prior.unconstrained_kp[1],"the same recovery objective must preserve the physical shoulder/wrist inertia scale");
    const auto recovery3=recovery_response_demand(3,pi/1800,.7);
    const std::vector<ModelAxisLimits> three_limits{axis,axis,axis};
    const auto independent=model_initialization({{.2,.2,.001124}},{0,0,0},three_limits,recovery3);
    auto block=recovery3;block.mass_matrix_samples={{{.2,.15,0},{.15,.2,0},{0,0,.001124}}};
    const auto coupled_block=model_initialization({{.2,.2,.001124}},{0,0,0},three_limits,block);
    check(coupled_block.gains.kp[2]==independent.gains.kp[2] && coupled_block.gains.kd[2]==independent.gains.kd[2],
        "a limiting mode in another independent inertia block must not scale the wrist seed");
    check(!coupled_block.modal_adjustments.empty(),"mode-local adjustments must preserve their participating-axis evidence");
    std::cout<<"protocol/task/work domain checks: all 4096 MIT codes, light-axis positive gain, actual task demand, FF residual, full coupled M, joint effort constraint\n";
}

void check_recorded_quantization_boundary() {
    // Reproduce the actual terminal decision after joint6 established on the
    // eighth complete group: joint3 fell one code below its motion threshold.
    const ModelAxisLimits limit{.04,1.62,3.24,7,220,4.2,{}};
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    auto plan=short_probe_plan(req,limit,220,4.2,noise,false,100,.028,.2).requirements;
    const Gains gains{{220},{4.2}};ResponseModel model;model.tracking_valid=true;model.reason="closed_loop_model_not_identifiable";
    Metrics healthy;healthy.measurement_samples=healthy.samples=1230;healthy.actual_span=.00457763671875;healthy.response_rms_ratio=.24769;healthy.applied_input_peak_nm=2.752906;
    MotionObservation motion;motion.positive_travel_rad=.00457763671875;motion.negative_travel_rad=.0030517578125;
    motion.minimum_motion_span_rad=.0031692131;motion.encoder_step_rad=noise.encoder_step_rad;motion.response_rms_ratio=.24769;motion.quantization_boundary=true;
    MotionEstablishmentState state;state.context_gains=gains;state.context_plans={plan};state.established_axes={true};state.boundary_retests={0};
    auto decision=motion_establishment_step(gains,{.24482097},{limit},{noise},{model},{healthy},{motion},{plan},req,false,200,&state);
    check(!decision.established && !decision.motion_established && !decision.feedback_fault,
        "a current encoder-boundary miss must not inherit a previous axis pass or become a feedback fault");
    check(decision.retry || !decision.blocked_axes.empty(),
        "an unresolved boundary must request a legal complete-set measurement or identify the actual input constraint");
    check(decision.gains.kp[0]<=limit.kp_max && decision.gains.kd[0]<=limit.kd_max &&
        (!decision.retry || short_trial_cost(decision.plans,req.tail_s)<=200),
        "boundary supplementation must obey protocol limits and the unchanged measurement budget");
    MotionEstablishmentState expired;
    check(!motion_establishment_step(gains,{.24482097},{limit},{noise},{model},{healthy},{motion},{plan},req,false,0,&expired).retry,
        "boundary remeasurement must retain actual verification/exit budget reservation");
    motion.negative_travel_rad=.0034332275390625;motion.qualified=motion.bidirectional=true;motion.quantization_boundary=false;
    check(motion_establishment_step(gains,{.24482097},{limit},{noise},{model},{healthy},{motion},{plan},req,false,200,&state).established,
        "only a newly measured complete parameter group may establish final motion");
}

void check_unidentified_response_and_execution_guard(const std::string& recorded_frames) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const ModelAxisLimits limit{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    const Gains seed{{138*500.0/4095},{5}};
    auto plan=short_probe_plan(req,limit,seed.kp[0],5,noise,false,200,.00980991226876,.45).requirements;
    ResponseModel weak;weak.reason="low_spectral_fit_quality";
    Metrics metric;metric.measurement_samples=metric.samples=1377;metric.actual_span=noise.encoder_step_rad;metric.applied_input_peak_nm=.34209276;
    MotionObservation motion;motion.response_rms_ratio=.03;motion.encoder_step_rad=noise.encoder_step_rad;
    MotionEstablishmentState state;
    const auto initial=motion_establishment_step(seed,{.44196909},{limit},{noise},{weak},{metric},{motion},{plan},req,false,200,&state);
    check(initial.retry && initial.gains.kd==seed.kd && !initial.motion_established &&
        initial.gains.kp[0]<=1.4*seed.kp[0]+limit.grid.kp_step,
        "1794233944825 acceleration-bounded weak response must request legal drive without inventing damping or motion");
    const auto smaller=short_probe_plan(req,limit,seed.kp[0],5,noise,false,200,.0049,.45).requirements;
    MotionEstablishmentState amplitude_state;
    const auto amplitude_first=motion_establishment_step(seed,{.44196909},{limit},{noise},{weak},{metric},{motion},{smaller},req,false,200,&amplitude_state);
    check(amplitude_first.retry && amplitude_first.gains.kp==seed.kp && amplitude_first.gains.kd==seed.kd &&
        amplitude_first.plans[0].amplitude_rad>smaller.amplitude_rad+noise.encoder_step_rad,
        "genuine reference-amplitude headroom must be used before a controller gain increase");
    const auto before_next=state;
    const auto next=motion_establishment_step(initial.gains,{.44196909},{limit},{noise},{weak},{metric},{motion},initial.plans,req,false,190,&state);
    check(next.retry && !next.motion_established && !next.feedback_fault && next.gains.kp[0]>=initial.gains.kp[0] &&
        next.gains.kp[0]<=1.4*initial.gains.kp[0]+limit.grid.kp_step && next.gains.kd==seed.kd &&
        prescribed_pd_input_peak(next.plans[0],next.gains.kp[0],next.gains.kd[0])>
            prescribed_pd_input_peak(initial.plans[0],initial.gains.kp[0],initial.gains.kd[0]),
        "continued sticking must retain a growing known input through legal amplitude or bounded stiffness without inventing damping evidence");
    check(short_trial_cost(next.plans,req.tail_s)<=190 &&
        prescribed_pd_input_peak(next.plans[0],next.gains.kp[0],next.gains.kd[0])<=.5*limit.effort_budget+1e-9,
        "weak-response revisions must retain the original measurement budget and calibration-effort headroom");
    auto huge_pd=metric;huge_pd.applied_input_peak_nm=700;
    auto state_copy=before_next;
    const auto noisy_pd=motion_establishment_step(initial.gains,{.44196909},{limit},{noise},{weak},{huge_pd},{motion},initial.plans,req,false,190,&state_copy);
    check(noisy_pd.gains.kp==next.gains.kp && noisy_pd.gains.kd==next.gains.kd,"D times noisy velocity peak must not act as a breakaway-force estimator or Kp normalization");
    auto moving_metric=metric;moving_metric.actual_span=.012;auto moving=motion;moving.response_rms_ratio=.7;moving.bidirectional=true;
    MotionEstablishmentState moving_state;
    for(unsigned trial=0;trial<4;++trial){const auto retry=motion_establishment_step(seed,{.44196909},{limit},{noise},{weak},{moving_metric},{moving},{plan},req,false,190,&moving_state);
        check(retry.gains.kp==seed.kp && retry.gains.kd==seed.kd,"poor spectral fit with substantial bidirectional motion must never be treated as evidence of insufficient Kp");plan=retry.plans[0];}
    weak.tracking_valid=true;weak.tracking_peak_error_rad=.02;weak.reference_lag_s=.3;weak.reason="closed_loop_model_not_identifiable";
    const auto candidate=adaptive_candidate(seed,{weak},{limit},req);
    check(candidate.gains.kp==seed.kp && candidate.gains.kd==seed.kd,"tracking lag without identifiable feedback dynamics must not automatically increase Kp");

    for(unsigned kind=0;kind<3;++kind) {
        CommandConditionedMotionGuard guard(req,noise);bool triggered=false;
        for(unsigned k=0;k<=1000;++k){Sample row;row.time_s=.005*k;row.new_measurement=true;row.feedback_sequence=k+1;
            if(kind==0){row.reference=.01*std::sin(2*pi*5*row.time_s);row.actual=row.reference;}
            else if(kind==1){row.actual=noise.encoder_step_rad*std::sin(2*pi*70*row.time_s);row.velocity=100*std::sin(2*pi*70*row.time_s);}
            else {row.actual=.01*std::exp(-8*row.time_s)*std::sin(2*pi*row.time_s);}
            triggered=triggered || guard.observe(row).triggered;
        }
        check(!triggered,"a commanded reversal, one-code position noise or a single overshoot cannot trigger the uncommanded-position guard");
    }
    CommandConditionedMotionGuard unstable(req,noise);bool triggered=false;
    for(unsigned k=0;k<=200;++k){Sample row;row.time_s=.005*k;row.new_measurement=true;row.feedback_sequence=k+1;
        row.reference=.02*row.time_s;row.actual=row.reference+.008*std::sin(2*pi*25*row.time_s);triggered=triggered || unstable.observe(row).triggered;}
    check(triggered,"repeated rapid position excursions absent from the accepted reference must stop a trial before tail-only assessment");

    if(!recorded_frames.empty()) {
        std::ifstream input(recorded_frames);check(bool(input),"recorded frame CSV must be readable");
        const auto split=[](const std::string& line){std::vector<std::string> out;std::stringstream stream(line);std::string value;while(std::getline(stream,value,','))out.push_back(value);return out;};
        std::string line;std::getline(input,line);const auto header=split(line);
        const auto column=[&](const std::string& key){const auto it=std::find(header.begin(),header.end(),key);check(it!=header.end(),"recorded CSV lacks required actual evidence field");return std::size_t(it-header.begin());};
        const auto stage=column("gain_stage"),time=column("monotonic_ns"),q=column("q:joint4"),reference=column("ref_q:joint4"),sequence=column("feedback_received_ns:joint4");
        CommandConditionedMotionGuard guard(req,noise);double start=-1,first=-1,last=0;MotionGuardResult observed;
        while(std::getline(input,line)){const auto values=split(line);if(values[stage]!="native-frozen")continue;
            const double now=std::stod(values[time])*1e-9;if(start<0)start=now;Sample row;row.time_s=now-start;row.actual=std::stod(values[q]);row.reference=std::stod(values[reference]);
            row.feedback_sequence=std::stoull(values[sequence]);row.new_measurement=true;last=row.time_s;const auto result=guard.observe(row);
            if(result.triggered && first<0){first=row.time_s;observed=result;}}
        check(first>0 && first<last-.01,"actual 1794233944825 joint4 oscillation must trigger before the final governor failure frame");
        std::cout<<"1794233944825 actual frame replay: first joint4 guard="<<first<<" s, final="<<last<<" s, reversals="<<observed.position_reversals<<", reference reversals="<<observed.reference_reversals<<"\n";
        const auto directory=recorded_frames.substr(0,recorded_frames.find_last_of('/')+1);
        for(const std::string name:{"servo-probe-native-establish-1.csv","servo-probe-native-establish-2.csv","servo-probe-native-establish-3.csv","servo-probe-native-compare-4.csv"}) {
            std::ifstream probe_rows(directory+name);check(bool(probe_rows),"actual short-probe evidence must be readable");
            std::getline(probe_rows,line);const auto fields=split(line);
            const auto index=[&](const std::string& key){const auto it=std::find(fields.begin(),fields.end(),key);check(it!=fields.end(),"short probe CSV lacks actual evidence");return std::size_t(it-fields.begin());};
            const auto ti=index("time_s"),ji=index("joint"),qi=index("actual"),ri=index("reference"),si=index("feedback_sequence"),ni=index("new_measurement");
            std::vector<CommandConditionedMotionGuard> guards(6,CommandConditionedMotionGuard(req,noise));
            while(std::getline(probe_rows,line)){const auto values=split(line);Sample row;row.time_s=std::stod(values[ti]);row.actual=std::stod(values[qi]);row.reference=std::stod(values[ri]);
                row.new_measurement=values[ni]=="1";row.feedback_sequence=std::stoull(values[si]);const auto axis=std::stoul(values[ji].substr(5))-1;
                check(axis<guards.size() && !guards[axis].observe(row).triggered,"the recorded slow short probes must not be retrospectively misreported as uncommanded oscillation");}
        }
    }
    std::cout<<"weak-response trust steps, unidentified tracking deficit, noise/command conditioned position guard: passed\n";
}

void check_establishment_input_plateaus(const std::string& recorded_directory) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const ModelAxisLimits limit{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    Gains gain{{1977*500.0/4095},{2284*5.0/4095}}; // actual trial 10: 241.3919 / 2.78877
    auto plan=short_probe_plan(req,limit,gain.kp[0],gain.kd[0],noise,false,200,.01226239033595,.3825).requirements;
    Metrics metric;metric.measurement_samples=metric.samples=1377;metric.actual_span=5*noise.encoder_step_rad;metric.applied_input_peak_nm=2.55291345;
    MotionObservation motion;motion.positive_travel_rad=5*noise.encoder_step_rad;motion.negative_travel_rad=4*noise.encoder_step_rad;
    motion.minimum_motion_span_rad=.002783361;motion.encoder_step_rad=noise.encoder_step_rad;motion.response_rms_ratio=.1369;
    ResponseModel weak;weak.reason="response_below_encoder_noise_floor";
    MotionEstablishmentState state;state.excitation_retests={1};bool stopped=false;double elapsed=0;
    for(unsigned trial=0;trial<12;++trial) {
        const auto decision=motion_establishment_step(gain,{.2441513156},{limit},{noise},{weak},{metric},{motion},{plan},req,false,200-elapsed,&state);
        const double previous_input=prescribed_pd_input_peak(plan,gain.kp[0],gain.kd[0]);
        const double next_input=prescribed_pd_input_peak(decision.plans[0],decision.gains.kp[0],decision.gains.kd[0]);
        check(independent_motion_damping_preserved(gain,decision.gains,{limit}) && decision.gains.kp[0]<=500 && next_input<=.5*limit.effort_budget+1e-9,
            "healthy weak-response calibration must preserve independently measured damping and actual protocol/known-drive effort bounds");
        if(decision.gains.kp[0]>gain.kp[0])check(next_input>=previous_input+2*noise.encoder_step_rad*gain.kp[0]-1e-9,
            "1794236700517 cannot raise stiffness by inverse amplitude scaling without a resolvable increase in actual known reference drive");
        elapsed+=short_trial_cost({plan},req.tail_s);
        check(!decision.motion_established && !decision.established,"a hypothetical repeated weak response cannot become a measured motion pass");
        if(!decision.retry){stopped=true;break;}
        check(next_input>previous_input || (!plan.motion_probe && decision.plans[0].motion_probe),
            "additional healthy trials need a genuinely different useful excitation, not an endless gain-only search");
        gain=decision.gains;plan=decision.plans[0];
    }
    check(stopped && elapsed<190,"exhausted useful input or reserved budget must stop without spending the exit reserve");
    gain={{1977*500.0/4095},{2284*5.0/4095}};
    const auto saturated_plan=short_probe_plan(req,limit,gain.kp[0],gain.kd[0],noise,false,200,.01226239033595,.3825).requirements;
    const Gains group{{gain.kp[0],8},{gain.kd[0],.2}};
    const auto second_plan=short_probe_plan(req,limit,8,.2,noise,false,200).requirements;
    MotionEstablishmentState group_state;group_state.excitation_retests={1,1};
    const auto same_group=motion_establishment_step(group,{.2441513156,.2},{limit,limit},{noise,noise},{weak,weak},{metric,metric},{motion,motion},{saturated_plan,second_plan},req,false,200,&group_state);
    check(same_group.retry && independent_motion_damping_preserved(group,same_group.gains,{limit,limit}) &&
        prescribed_pd_input_peak(same_group.plans[1],same_group.gains.kp[1],same_group.gains.kd[1])>
        prescribed_pd_input_peak(second_plan,group.kp[1],group.kd[1]),
        "one high-input axis must not freeze another weak axis's legal reference-drive calibration");
    auto ringing=weak;ringing.sustained_oscillation=true;MotionEstablishmentState unstable_state;unstable_state.excitation_retests={1,1};
    const auto reduced=motion_establishment_step(group,{.2441513156,.2},{limit,limit},{noise,noise},{weak,ringing},{metric,metric},{motion,motion},{saturated_plan,second_plan},req,false,200,&unstable_state);
    check(reduced.gains.kp[0]==group.kp[0] && reduced.gains.kp[1]<group.kp[1] && reduced.gains.kd[1]==group.kd[1] && reduced.plans[0].frequencies_hz==saturated_plan.frequencies_hz,
        "measured ringing must reduce stiffness while retaining damping before complete-group remeasurement and may not strengthen another axis");
    // One-sided drift is not a hardware-capacity certificate. Even after a
    // previous single-tone trial, legal small-input calibration may continue.
    const ModelAxisLimits roomy{.04,1.62,3.24,7,500,5,{}};
    Gains elastic{{8},{.2}};auto elastic_plan=short_probe_plan(req,roomy,8,.2,noise,false,180).requirements;
    auto elastic_metric=metric;elastic_metric.actual_span=.004;elastic_metric.applied_input_peak_nm=.1;
    auto elastic_motion=motion;elastic_motion.positive_travel_rad=.004;elastic_motion.negative_travel_rad=.001;
    elastic_motion.minimum_motion_span_rad=.003;elastic_motion.response_rms_ratio=.15;
    MotionEstablishmentState elastic_state;elastic_state.excitation_retests={1};
    auto first=motion_establishment_step(elastic,{.2},{roomy},{noise},{weak},{elastic_metric},{elastic_motion},{elastic_plan},req,false,180,&elastic_state);
    check(first.retry && first.blocked_axes.empty() && independent_motion_damping_preserved(elastic,first.gains,{roomy}) &&
        prescribed_pd_input_peak(first.plans[0],first.gains.kp[0],first.gains.kd[0])>prescribed_pd_input_peak(elastic_plan,8,.2),
        "coarse one-sided response must first retain a legal useful next reference-drive measurement");
    elastic=first.gains;elastic_plan=first.plans[0];elastic_metric.applied_input_peak_nm=.2;
    auto confirm=motion_establishment_step(elastic,{.2},{roomy},{noise},{weak},{elastic_metric},{elastic_motion},{elastic_plan},req,false,170,&elastic_state);
    check(confirm.retry && confirm.blocked_axes.empty() && !confirm.motion_established && independent_motion_damping_preserved(elastic,confirm.gains,{roomy}),
        "unchanged one-sided displacement must not permanently block a low-input controller while legal input can still grow");

    auto saturated=roomy;saturated.kp_max=8;saturated.kd_max=.2;
    const auto maximum=symmetric_motion_probe_plan(req,saturated,8,.2,noise,200,elastic_plan,.028,.2);
    check(maximum.valid,"the true saturation fixture must have an executable full input window");
    MotionEstablishmentState maximum_state;
    const auto terminal=motion_establishment_step(Gains{{8},{.2}},{.2},{saturated},{noise},{weak},{elastic_metric},{elastic_motion},
        {maximum.requirements},req,false,140,&maximum_state);
    check(!terminal.retry && !terminal.motion_established && terminal.blocked_axes==std::vector<std::size_t>{0} &&
        terminal.gains.kp[0]==8 && prescribed_pd_input_peak(terminal.plans[0],8,.2)<=prescribed_pd_input_peak(maximum.requirements,8,.2)+1e-12,
        "actual amplitude/protocol/frequency saturation must not create fictitious input growth or a motion pass");

    if(!recorded_directory.empty()) {
        const auto split=[](const std::string& line){std::vector<std::string> out;std::stringstream stream(line);std::string value;while(std::getline(stream,value,','))out.push_back(value);return out;};
        MotionEstablishmentState replay;unsigned first_diverged=0;double next_kp=0,next_frequency=0,next_amplitude=0;
        for(unsigned trial=1;trial<=15;++trial) {
            std::ifstream input(recorded_directory+"/servo-probe-native-establish-"+std::to_string(trial)+".csv");check(bool(input),"all fifteen actual establishment CSVs must be readable");
            std::string line;std::getline(input,line);const auto header=split(line);
            const auto column=[&](const std::string& key){const auto it=std::find(header.begin(),header.end(),key);check(it!=header.end(),"actual short CSV lacks evidence field");return std::size_t(it-header.begin());};
            const auto ti=column("time_s"),ji=column("joint"),ri=column("reference"),vi=column("reference_velocity"),qi=column("actual"),di=column("velocity"),ni=column("new_measurement"),ki=column("actual_kp"),kdi=column("actual_kd");
            std::vector<Sample> rows;Gains current;double center=0,numerator=0,denominator=0;
            Requirements actual_plan=req;actual_plan.probe_duration_s=5.5;actual_plan.amplitude_rad=1;
            const double frequency=trial==1?.45:.3825;actual_plan.frequencies_hz={frequency,2*frequency,frequency*1.2/.35};
            while(std::getline(input,line)){const auto v=split(line);if(v[ji]!="joint3")continue;Sample row;row.time_s=std::stod(v[ti]);row.reference=std::stod(v[ri]);row.reference_velocity=std::stod(v[vi]);
                row.actual=std::stod(v[qi]);row.velocity=std::stod(v[di]);row.new_measurement=v[ni]=="1";row.actual_gains_known=true;row.actual_kp=std::stod(v[ki]);row.actual_kd=std::stod(v[kdi]);
                if(rows.empty()){center=row.reference;current={{row.actual_kp},{row.actual_kd}};}const double unit=probe(row.time_s,actual_plan).first;
                numerator+=(row.reference-center)*unit;denominator+=unit*unit;rows.push_back(row);}
            actual_plan.amplitude_rad=numerator/denominator;
            auto a=assess_trajectory(rows,actual_plan,noise,5.5);a.metrics.applied_input_peak_nm=applied_pd_input_peak(rows,current.kp[0],current.kd[0]);
            ResponseModel recorded;recorded.reason=trial==5 || trial==9?"low_spectral_fit_quality":"response_below_encoder_noise_floor";
            // After the first policy divergence the later original CSVs are
            // evidence of the OLD controller only. Never feed them back as if
            // the proposed slower experiment had actually been executed.
            if(!first_diverged) {
                const auto decision=motion_establishment_step(current,{.2441513156},{limit},{noise},{recorded},{a.metrics},{a.motion},{actual_plan},req,false,250,&replay);
                const auto& proposed=decision.plans[0];
                if(decision.gains.kp!=current.kp || decision.gains.kd!=current.kd || proposed.motion_probe ||
                    std::abs(proposed.amplitude_rad-actual_plan.amplitude_rad)>1e-8 || proposed.frequencies_hz!=actual_plan.frequencies_hz) {
                    first_diverged=trial;next_kp=decision.gains.kp[0];next_frequency=proposed.frequencies_hz.front();next_amplitude=proposed.amplitude_rad;
                    const double old_input=prescribed_pd_input_peak(actual_plan,current.kp[0],current.kd[0]);
                    const double new_input=prescribed_pd_input_peak(proposed,next_kp,decision.gains.kd[0]);
                    check(decision.retry && independent_motion_damping_preserved(current,decision.gains,{limit}) &&
                        (next_kp<=current.kp[0] || new_input>=old_input+2*noise.encoder_step_rad*current.kp[0]-1e-9),
                        "the first real-data policy divergence must request useful bounded drive, not an input-neutral stiffness escalation");
                }
            }
            if(trial>=11)check(a.metrics.actual_span<=6*noise.encoder_step_rad+1e-10,"actual late response must preserve its six-code plateau evidence");
        }
        check(first_diverged>0 && first_diverged<=10 && next_kp<300,"the real 15-CSV failure must diverge before the input-neutral 337/473/500 escalation");
        std::cout<<"1794236700517 actual 15-CSV audit, replay only to first divergence: trial "<<first_diverged<<", next Kp="<<next_kp<<", frequency="<<next_frequency<<", amplitude="<<next_amplitude<<" (unexecuted)\n";
    }
    std::cout<<"planned excitation saturation and measured input/response plateau checks passed\n";
}

void check_symmetric_motion_establishment(const std::string& recorded_directory) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const ModelAxisLimits limit{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    const Gains gains{{1977*500.0/4095},{2284*5.0/4095}};
    auto original=short_probe_plan(req,limit,gains.kp[0],gains.kd[0],noise,false,250,.02,.2).requirements;
    Metrics recorded;recorded.samples=recorded.measurement_samples=1800;recorded.actual_span=6*noise.encoder_step_rad;recorded.applied_input_peak_nm=2.7746612771;
    MotionObservation motion;motion.positive_travel_rad=6*noise.encoder_step_rad;motion.negative_travel_rad=5*noise.encoder_step_rad;
    motion.minimum_motion_span_rad=.00303513248;motion.encoder_step_rad=noise.encoder_step_rad;motion.response_rms_ratio=.151;
    ResponseModel model;model.tracking_valid=true;model.reason="closed_loop_model_not_identifiable";
    MotionEstablishmentState state;state.excitation_retests={1};
    if(!recorded_directory.empty()) {
        std::ifstream input(recorded_directory+"/servo-probe-native-establish-12.csv");check(bool(input),"actual 1794238166406 twelfth spectral CSV must be readable");
        const auto split=[](const std::string& line){std::vector<std::string> out;std::stringstream stream(line);std::string value;while(std::getline(stream,value,','))out.push_back(value);return out;};
        std::string line;std::getline(input,line);const auto fields=split(line);
        const auto column=[&](const std::string& key){const auto it=std::find(fields.begin(),fields.end(),key);check(it!=fields.end(),"actual twelfth CSV lacks required evidence");return std::size_t(it-fields.begin());};
        const auto ti=column("time_s"),ji=column("joint"),ri=column("reference"),vi=column("reference_velocity"),qi=column("actual"),di=column("velocity"),ni=column("new_measurement");
        std::vector<Sample> actual_rows;
        while(std::getline(input,line)){const auto v=split(line);if(v[ji]!="joint3")continue;Sample row;row.time_s=std::stod(v[ti]);row.reference=std::stod(v[ri]);row.reference_velocity=std::stod(v[vi]);
            row.actual=std::stod(v[qi]);row.velocity=std::stod(v[di]);row.new_measurement=v[ni]=="1";actual_rows.push_back(row);}
        const auto actual=assess_trajectory(actual_rows,original,noise,original.probe_duration_s);recorded=actual.metrics;motion=actual.motion;
        recorded.applied_input_peak_nm=applied_pd_input_peak(actual_rows,gains.kp[0],gains.kd[0]);
        check(!motion.qualified && motion.negative_travel_rad==5*noise.encoder_step_rad,"the real final multisine window must preserve its unmet five-code directional response");
    }
    const auto switched=motion_establishment_step(gains,{.2441513156},{limit},{noise},{model},{recorded},{motion},{original},req,false,240,&state);
    check(switched.retry && !switched.feedback_fault && !switched.motion_established && !switched.established &&
        switched.gains.kp[0]<=limit.kp_max && switched.gains.kd[0]<=limit.kd_max &&
        short_trial_cost(switched.plans,req.tail_s)<=240,
        "1794238166406 healthy unresolved window must retain a bounded next measurement without inventing established motion or exhausted actuator capability");
    // Test the distinct waveform explicitly. The scheduler may choose another
    // legal amplitude/controller revision before this mechanism experiment.
    const auto symmetric=symmetric_motion_probe_plan(req,limit,gains.kp[0],gains.kd[0],noise,240,original);
    check(symmetric.valid && symmetric.requirements.motion_probe,"the explicit bounded symmetric motion experiment must remain available");
    const auto plan=symmetric.requirements;
    check((plan.probe_duration_s-1.4)*plan.frequencies_hz.front()>=2-1e-9 &&
        plan.amplitude_rad*(gains.kp[0]+gains.kd[0]*(2*pi*plan.frequencies_hz.front()+3))<=.5*limit.effort_budget+1e-10,
        "symmetric motion must contain two complete interior cycles within the unchanged half calibration-effort allowance");
    double positive=0,negative=0;
    for(double t=.7;t<plan.probe_duration_s-.7;t+=.005){const auto r=probe(t,plan);const double u=gains.kp[0]*r.first+gains.kd[0]*r.second;positive=std::max(positive,u);negative=std::max(negative,-u);}
    close(positive,negative,.001,"the distinct motion experiment must provide equal exogenous drive in both directions");
    check(positive>3.2,"same controller and effort allowance must remove the recorded positive-direction multisine drive starvation");

    // Synthetic asymmetric friction is an offline mechanism experiment, not a
    // claim that the original arm would pass this counterfactual excitation.
    StickSlipAxis mobile{.2441513156,2.3,.15,2.7};
    const auto rows=stick_slip_trial(mobile,plan,gains.kp[0],gains.kd[0],limit,28);
    auto observed=assess_trajectory(rows,plan,noise,plan.probe_duration_s);observed.metrics.applied_input_peak_nm=applied_pd_input_peak(rows,gains.kp[0],gains.kd[0]);
    const auto evidence=motion_probe_evidence(rows,plan,gains.kp[0],gains.kd[0]);
    check(evidence.reason=="motion_establishment_only_no_spectral_fit" && !evidence.tracking_valid && !evidence.identifiable && !evidence.missing_evidence.empty(),
        "a valid single-tone motion experiment must never fabricate three-tone tracking or pole identification evidence");
    MotionEstablishmentState motion_state;motion_state.spectral_plans={original};
    const auto established=motion_establishment_step(gains,{.2441513156},{limit},{noise},{evidence},{observed.metrics},{observed.motion},{plan},req,false,200,&motion_state);
    check(established.motion_established && established.spectral_confirmation_required && !established.established &&
        !motion_state.spectral_plans[0].motion_probe && !observed.passed,
        "actual motion can establish independently while requiring the original spectral measurement and retaining the strict tracking rejection");
    close(motion_state.progress[0].observed_input_nm,observed.metrics.applied_input_peak_nm,1e-12,"the all-motion-established early return must still refresh this window's observed input evidence");
    close(motion_state.progress[0].observed_span_rad,observed.metrics.actual_span,1e-12,"the early return must refresh this window's actual motion span");
    auto stable_rows=rows;for(auto& row:stable_rows){row.actual_gains_known=true;row.actual_kp=gains.kp[0];row.actual_kd=gains.kd[0];}
    stable_rows[200].actual_kp*=.9;const auto transition=motion_probe_evidence(stable_rows,plan,gains.kp[0],gains.kd[0]);
    check(transition.reason=="changing_feedback_gains","single-tone establishment must retain the actual executed-gain stability check");
    const auto wait=motion_establishment_step(gains,{.2441513156},{limit},{noise},{transition},{observed.metrics},{observed.motion},{plan},req,false,180,&motion_state);
    check(wait.retry && !wait.motion_established && wait.gains.kp==gains.kp,"gain transition cannot bypass stable complete-controller remeasurement merely because visible motion exists");
    auto missing=rows;for(std::size_t i=200;i<240;++i)missing[i].new_measurement=false;
    check(motion_probe_evidence(missing,plan,gains.kp[0],gains.kd[0]).reason=="feedback_time_gap_or_repeat",
        "single-tone motion cannot hide a feedback gap by skipping the physical frequency fitter");
    StickSlipAxis blocked{.2441513156,12,.15,12};
    const auto blocked_rows=stick_slip_trial(blocked,plan,gains.kp[0],gains.kd[0],limit,28);
    auto blocked_observed=assess_trajectory(blocked_rows,plan,noise,plan.probe_duration_s);
    blocked_observed.metrics.applied_input_peak_nm=applied_pd_input_peak(blocked_rows,gains.kp[0],gains.kd[0]);
    const auto blocked_evidence=motion_probe_evidence(blocked_rows,plan,gains.kp[0],gains.kd[0]);
    MotionEstablishmentState immobile_state;immobile_state.spectral_plans={original};
    const auto continued=motion_establishment_step(gains,{.2441513156},{limit},{noise},{blocked_evidence},{blocked_observed.metrics},{blocked_observed.motion},{plan},req,false,180,&immobile_state);
    check(!continued.feedback_fault && !continued.motion_established && !continued.established &&
        continued.gains.kp[0]<=limit.kp_max && continued.gains.kd[0]<=limit.kd_max,
        "a healthy immobile single-tone window remains an insufficient measurement without an invented motion pass or execution fault");
    if(continued.retry)check(prescribed_pd_input_peak(continued.plans[0],continued.gains.kp[0],continued.gains.kd[0])>
            prescribed_pd_input_peak(plan,gains.kp[0],gains.kd[0]),
        "another immobile-window trial must offer a genuinely larger known drive");
    else check(continued.blocked_axes==std::vector<std::size_t>{0} && continued.reasons[0].find("no_legal_input_growth")!=std::string::npos,
        "the high-input immobile axis may stop only for its actual legal-drive limit, never merely because a single tone has been attempted");
    check(short_trial_cost(continued.plans,req.tail_s)<=180 &&
        prescribed_pd_input_peak(continued.plans[0],continued.gains.kp[0],continued.gains.kd[0])<=.5*limit.effort_budget+1e-9,
        "continued single-tone input calibration must retain the original budget and effort allowance");
    auto normal=model;normal.tracking_valid=true;auto qualified=observed.motion;qualified.qualified=true;
    auto fresh=observed.metrics;fresh.applied_input_peak_nm+=.123;fresh.actual_span+=noise.encoder_step_rad;
    const auto complete=motion_establishment_step(gains,{.2441513156},{limit},{noise},{normal},{fresh},{qualified},{original},req,false,150,&motion_state);
    check(complete.established && complete.motion_established,"a complete ordinary spectral/motion measurement must preserve its original establishment decision");
    close(motion_state.progress[0].observed_input_nm,fresh.applied_input_peak_nm,1e-12,"an already qualified axis must never serialize the preceding window's observed input");
    close(motion_state.progress[0].observed_span_rad,fresh.actual_span,1e-12,"the ordinary all-established return must refresh its observed position span");
    std::cout<<"1794238166406 recorded-window counterfactual: explicit symmetric +/-"<<positive<<" Nm reference drive; separate asymmetric-friction plant established motion, immobile plant retained legal input calibration\n";
}

std::vector<std::vector<Sample>> read_joint_evidence_csv(const std::string& path) {
    std::ifstream input(path);check(bool(input),"recorded calibration CSV must be readable");
    const auto split=[](const std::string& line){std::vector<std::string> out;std::stringstream stream(line);std::string value;while(std::getline(stream,value,','))out.push_back(value);return out;};
    std::string line;std::getline(input,line);const auto fields=split(line);
    const auto column=[&](const std::string& key){const auto it=std::find(fields.begin(),fields.end(),key);check(it!=fields.end(),"recorded calibration CSV lacks required evidence");return std::size_t(it-fields.begin());};
    const auto ti=column("time_s"),ji=column("joint"),ri=column("reference"),vi=column("reference_velocity"),qi=column("actual"),di=column("velocity"),ni=column("new_measurement"),li=column("limited");
    const auto ki=column("actual_kp"),kdi=column("actual_kd"),gi=column("actual_gains_known"),fi=column("feedforward_nm"),ii=column("independent_disturbance_nm");
    const auto ci=column("command_time_s"),fti=column("feedback_time_s"),si=column("feedback_sequence");
    std::vector<std::vector<Sample>> rows(6);
    while(std::getline(input,line)){const auto v=split(line);if(v[ji].size()<6)continue;const auto axis=std::stoul(v[ji].substr(5))-1;check(axis<rows.size(),"recorded joint mapping must match six-axis fixture");
        Sample row;row.time_s=std::stod(v[ti]);row.reference=std::stod(v[ri]);row.reference_velocity=std::stod(v[vi]);row.actual=std::stod(v[qi]);row.velocity=std::stod(v[di]);
        row.new_measurement=v[ni]=="1";row.limited=v[li]=="1";row.actual_gains_known=v[gi]=="1";row.actual_kp=std::stod(v[ki]);row.actual_kd=std::stod(v[kdi]);
        row.applied_feedforward_nm=std::stod(v[fi]);row.feedforward_known=true;row.independent_disturbance_nm=std::stod(v[ii]);row.independent_disturbance_known=true;
        row.command_time_s=std::stod(v[ci]);row.feedback_time_s=std::stod(v[fti]);row.feedback_sequence=std::stoull(v[si]);rows[axis].push_back(row);
    }
    return rows;
}

void check_observed_reference_domain(const std::string& recorded_directory) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const ModelAxisLimits original{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    auto inward=original;inward.position_margin_rad=.02;inward.reference_min_offset_rad=0;inward.reference_max_offset_rad=.04;
    const double kp=50,kd=3;
    const auto legacy=short_probe_plan(req,original,kp,kd,noise,false,200);
    const auto planned=short_probe_plan(req,inward,kp,kd,noise,false,200);
    check(legacy.valid && planned.valid && legacy.requirements.reference_bias_rad==0 && planned.requirements.reference_bias_rad>0,
        "a fresh position at an observed endpoint must produce an inward excitation without changing unbounded legacy plans");
    close(planned.requirements.reference_bias_rad,planned.requirements.amplitude_rad,1e-12,"an endpoint experiment must remain on its known side");
    const auto verify=[&](const ShortProbePlan& p,const ModelAxisLimits& bounds,double P,double D) {
        const auto& r=p.requirements;const double high_w=2*pi*r.frequencies_hz[r.motion_probe?0:2];
        const auto b=bound_reference_excitation(bounds,P,D,r.amplitude_rad,high_w);
        for(double t=0;t<r.probe_duration_s;t+=.001) {
            const auto sample=probe(t,r);const double h=1e-6;
            const double numerical=(probe(t+h,r).first-probe(t-h,r).first)/(2*h);
            check(sample.first>=bounds.reference_min_offset_rad-1e-12 && sample.first<=bounds.reference_max_offset_rad+1e-12,
                "the interior and both fade transitions must remain inside actual demonstrated positions");
            close(sample.second,numerical,3e-7,"MIT reference velocity must differentiate the actual biased position during both fades");
            check(std::abs(sample.second)<=b.velocity_bound_rad_s+1e-9 && std::abs(P*sample.first+D*sample.second)<=b.effort_bound_nm+1e-9,
                "biased reference velocity and actual exogenous effort must retain the original bounded planner headroom");
        }
        check(probe(0,r)==std::pair<double,double>{0,0} && probe(r.probe_duration_s,r)==std::pair<double,double>{0,0},
            "a biased bounded experiment must start and end at the same fresh measured origin with zero velocity");
        check(prescribed_pd_input_peak(r,P,D)<=.5*bounds.effort_budget+1e-9,
            "input-platform comparisons must include the actual reference bias and transition velocity");
    };
    verify(planned,inward,kp,kd);
    auto upper=inward;upper.reference_min_offset_rad=-.04;upper.reference_max_offset_rad=0;
    const auto return_inward=symmetric_motion_probe_plan(req,upper,kp,kd,noise,200,planned.requirements);
    check(return_inward.valid && return_inward.requirements.reference_bias_rad<0,"the upper observed endpoint must mirror the inward plan");
    verify(return_inward,upper,kp,kd);
    const auto common=comparable_probe_plans({{kp},{kd}},{{kp*1.1},{kd}}, {planned.requirements},{inward},{noise},req,false,200);
    check(common.size()==1 && common[0].reference_bias_rad>0,"candidate remeasurement must use one common biased domain and direction for both full gain sets");
    auto invalid=inward;invalid.reference_min_offset_rad=.001;bool rejected=false;
    try{short_probe_plan(req,invalid,kp,kd,noise,false,200);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"a domain excluding its fresh actual origin cannot silently move that origin or stretch a limit");

    StickSlipAxis stop{.2,.12,.15,.15};stop.physical_lower_stop=0;
    const auto rows=stick_slip_trial(stop,planned.requirements,kp,kd,inward,28);
    auto assessment=assess_trajectory(rows,planned.requirements,noise,planned.requirements.probe_duration_s);
    check(assessment.motion.qualified && assessment.metrics.continuous && !assessment.metrics.limited_samples,
        "a physical lower stop and static friction must still allow measured inward and return movement under the unchanged controller/Safety limits");
    check(!assessment.passed,"inward motion evidence cannot relax the strict original tracking-error acceptance");
    const auto linear=simulate_mit(planned.requirements,{{kp},{kd}});
    const auto identified=identify_mit(linear,planned.requirements,{kp,kd,.2,0,ExcitationKind::ReferenceTracking});
    check(identified.tracking_valid && identified.coherence>.65,
        "closed-loop identification must fit actual biased reference, derivative and feedback using its existing intercept instead of an assumed unbiased input");

    Metrics healthy;healthy.measurement_samples=healthy.samples=1500;
    ResponseModel missing;missing.reason="response_below_encoder_noise_floor";
    const auto supplemented=revise_disturbance_input({0,.02},{0,.10},{missing,missing},{healthy,healthy},100,10);
    check(!supplemented.feedback_fault && supplemented.retry && supplemented.amplitudes[0]==0 && supplemented.amplitudes[1]==.04 &&
        supplemented.reasons[0]=="independent_disturbance_not_requested_without_bidirectional_demonstrated_room",
        "a deliberately unexcited boundary axis cannot reject or grow the independently measurable axes' disturbance input");
    auto lost=healthy;lost.continuous=false;
    check(revise_disturbance_input({0,.02},{0,.1},{missing,missing},{lost,healthy},100,10).feedback_fault,
        "a masked disturbance axis must still retain actual feedback-loss detection");
    rejected=false;try{revise_disturbance_input({0,.02},{.1,.1},{missing,missing},{healthy,healthy},100,10);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"zero input is only an explicit no-room mask when its matching ceiling is also zero");

    if(!recorded_directory.empty()) {
        const auto actual=read_joint_evidence_csv(recorded_directory+"/servo-probe-native-establish-12.csv");
        for(unsigned axis:{1U,2U}) {
            double low=std::numeric_limits<double>::infinity(),high=-low;
            for(const auto& row:actual[axis])if(row.new_measurement){low=std::min(low,row.actual);high=std::max(high,row.actual);}
            check(high>low,"the recorded joint2/joint3 actual interval must contain observed displacement");
            auto domain=original;domain.position_margin_rad=.5*(high-low);domain.reference_min_offset_rad=0;domain.reference_max_offset_rad=high-low;
            const double P=axis==1?147.61904761904762:241.39194139194137,D=axis==1?3.352869352869353:5;
            const auto actual_bounds=bound_reference_excitation(domain,P,D,.015,2*pi*.2);
            auto actual_plan=req;actual_plan.motion_probe=true;actual_plan.amplitude_rad=actual_bounds.amplitude_rad;
            actual_plan.reference_bias_rad=actual_bounds.bias_rad;
            for(double t=0;t<actual_plan.probe_duration_s;t+=.005) {
                const double reference=low+probe(t,actual_plan).first;
                check(reference>=low-1e-12 && reference<=high+1e-12,"counterfactual endpoint references must stay inside the actual recorded joint2/joint3 envelope regardless of edited URDF limits");
            }
        }
    }
    std::cout<<"observed endpoint reference bias, exact MIT velocity, unchanged effort/acceptance, lower-stop/friction plant and masked disturbance checks passed\n";
}

void check_shared_slow_establishment(const std::string& recorded_directory) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const ModelAxisLimits heavy{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    const auto wrist=ModelAxisLimits{.04,1.62,3.24,2.5,500,5,heavy.grid};
    const std::vector<ModelAxisLimits> limits{heavy,wrist,wrist};
    const std::vector<double> inertia{.24458484787,.00728364962,.001123771029};
    Gains gains{{241.39194139194137,6.837606837606837,3.4188034188034186},{5,.1111111111111111,.017094017094017096}};
    std::vector<Requirements> plans;
    for(unsigned i=0;i<3;++i)plans.push_back(short_probe_plan(req,limits[i],gains.kp[i],gains.kd[i],noise,false,200,.02,.2).requirements);
    Metrics weak;weak.samples=weak.measurement_samples=1231;weak.actual_span=6*noise.encoder_step_rad;weak.applied_input_peak_nm=2.566106;
    MotionObservation motion;motion.encoder_step_rad=noise.encoder_step_rad;motion.positive_travel_rad=6*noise.encoder_step_rad;
    motion.negative_travel_rad=5*noise.encoder_step_rad;motion.minimum_motion_span_rad=.0028586244;motion.response_rms_ratio=.15;
    ResponseModel model;model.reason="response_below_encoder_noise_floor";
    MotionEstablishmentState state;state.excitation_retests={1,1,1};
    auto shared=motion_establishment_step(gains,inertia,limits,std::vector<NoiseEstimate>(3,noise),
        std::vector<ResponseModel>(3,model),std::vector<Metrics>(3,weak),std::vector<MotionObservation>(3,motion),plans,req,false,190,&state);
    check(shared.retry && independent_motion_damping_preserved(gains,shared.gains,limits) && !shared.motion_established,
        "1794267281603 weak wrists must retain an independent legal next calibration while the heavy axis lacks a useful measurement");
    for(unsigned i:{1U,2U})check(prescribed_pd_input_peak(shared.plans[i],shared.gains.kp[i],shared.gains.kd[i])>
        prescribed_pd_input_peak(plans[i],gains.kp[i],gains.kd[i]),
        "a heavy-axis limitation must not be inherited as a wrist input limitation");

    // Start at the actually measured late controller, with resolvable asymmetric
    // static friction. These are reproducible mechanism tests, not fitted arm
    // parameters or a claim about the unexecuted hardware window.
    std::vector<StickSlipAxis> plant{{inertia[0],2.3,.15,2.7},{inertia[1],.168,.01,.140},{inertia[2],.162,.01,.134}};
    plans=shared.plans;double elapsed=0;bool established=false,wrist_step=false;unsigned trials=0;
    while(elapsed+short_trial_cost(plans,req.tail_s)<190) {
        std::vector<ResponseModel> models;std::vector<Metrics> metrics;std::vector<MotionObservation> motions;
        for(unsigned i=0;i<3;++i) {
            const auto rows=stick_slip_trial(plant[i],plans[i],gains.kp[i],gains.kd[i],limits[i],i?10:28);
            auto assessment=assess_trajectory(rows,plans[i],noise,plans[i].probe_duration_s);
            assessment.metrics.applied_input_peak_nm=applied_pd_input_peak(rows,gains.kp[i],gains.kd[i]);
            const auto evidence=plans[i].motion_probe?motion_probe_evidence(rows,plans[i],gains.kp[i],gains.kd[i]):
                identify_mit(rows,plans[i],{gains.kp[i],gains.kd[i],inertia[i],0,ExcitationKind::ReferenceTracking});
            models.push_back(evidence);metrics.push_back(assessment.metrics);motions.push_back(assessment.motion);
        }
        elapsed+=short_trial_cost(plans,req.tail_s);
        ++trials;
        const auto next=motion_establishment_step(gains,inertia,limits,std::vector<NoiseEstimate>(3,noise),models,metrics,motions,plans,req,false,190-elapsed,&state);
        check(!next.feedback_fault,"shared slow motion keeps healthy stick/slip data distinct from an execution failure");
        if(next.motion_established){established=true;break;}
        check(next.retry,"shared weak wrists must retain a legal next measurement while a demonstrated input increase remains available");
        wrist_step=wrist_step || next.gains.kp[1]>gains.kp[1] || next.gains.kp[2]>gains.kp[2];
        if(next.gains.kp[0]>gains.kp[0])check(
            prescribed_pd_input_peak(next.plans[0],next.gains.kp[0],next.gains.kd[0])>=
            prescribed_pd_input_peak(plans[0],gains.kp[0],gains.kd[0])+2*noise.encoder_step_rad*gains.kp[0]-1e-9,
            "a heavy-axis gain step must increase resolvable drive rather than reverse-scaling amplitude toward the protocol ceiling");
        gains=next.gains;plans=next.plans;
    }
    check(established && wrist_step && elapsed<190,"shared actual scheduler must establish the frictional wrist group inside its original measurement budget");

    if(!recorded_directory.empty()) {
        const auto rows=read_joint_evidence_csv(recorded_directory+"/servo-probe-native-establish-12.csv");
        const std::vector<double> kp{68.25396825396825,147.61904761904762,241.39194139194137,25.64102564102564,6.837606837606837,3.4188034188034186};
        const std::vector<double> kd{1.5384615384615385,3.352869352869353,5,.4358974358974359,.1111111111111111,.017094017094017096};
        const std::vector<double> nominal_gradient{0,-1.6003974855417213,-1.547205694337972,-.1928056680370327,-.02143952324698908,.05421125345635245};
        const std::vector<double> measured_inertia{.13532696385,.29348798754,.24458484787,.02391338447,.00728364962,.001123771029};
        std::vector<Requirements> executed;std::vector<ResponseModel> models;std::vector<Metrics> metrics;std::vector<MotionObservation> motions;
        for(unsigned i=0;i<6;++i) {
            auto actual_plan=req;actual_plan.amplitude_rad=i==2?.012592904557399497:.012262390335951842;
            const double f=i==2?.2:.3825;actual_plan.frequencies_hz={f,2*f,f*1.2/.35};actual_plan.probe_duration_s=7.65;
            actual_plan.encoder_step_rad=noise.encoder_step_rad;executed.push_back(actual_plan);
            auto a=assess_trajectory(rows[i],actual_plan,noise,7.65);
            a.metrics.applied_input_peak_nm=applied_pd_input_peak(rows[i],kp[i],kd[i]);
            check(a.metrics.continuous && !a.metrics.limited_samples && !a.metrics.oscillating,
                "the recorded final window retains its healthy, unlimited, nonoscillating measurement evidence");
            close(rows[i][rows[i].size()/2].actual_kp,kp[i],1e-4,"record replay must use the actual transmitted late gains");
            close(rows[i][rows[i].size()/2].actual_kd,kd[i],1e-4,"record replay must use the actual transmitted damping");
            MitExperiment actual_model{kp[i],kd[i],measured_inertia[i],nominal_gradient[i],ExcitationKind::ReferenceTracking};
            actual_model.executed_state_feedforward_gradient=nominal_gradient[i];
            models.push_back(identify_mit(rows[i],actual_plan,actual_model));metrics.push_back(a.metrics);motions.push_back(a.motion);
        }
        MotionEstablishmentState replay;replay.excitation_retests=std::vector<unsigned>(6,1);
        const auto next=motion_establishment_step({kp,kd},measured_inertia,{heavy,heavy,heavy,wrist,wrist,wrist},
            std::vector<NoiseEstimate>(6,noise),models,metrics,motions,executed,req,false,200,&replay);
        check(next.retry && !next.feedback_fault && independent_motion_damping_preserved({kp,kd},next.gains,{heavy,heavy,heavy,wrist,wrist,wrist}) && !next.motion_established,
            "actual 1794267281603 final data must schedule legal calibration without inventing successful execution");
        for(unsigned i:{3U,4U,5U})check(
            prescribed_pd_input_peak(next.plans[i],next.gains.kp[i],next.gains.kd[i])>
            prescribed_pd_input_peak(executed[i],kp[i],kd[i]),
            "recorded weak wrist axes need their own larger known drive instead of another axis's plateau conclusion");
    }
    std::cout<<"shared heavy-axis input plateau and weak frictional wrist establishment: "<<trials<<" trials, "<<elapsed<<" s, full gain-set measurement and unchanged constraints\n";
}

void check_low_input_wrist_calibration(const std::string& recorded_directory) {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    noise.rms_rad=.00019072632178811585; // full stationary CSV, not a friction fit
    const ModelAxisLimits wrist{.04,1.62,3.24,2.5,500,5,{500.0/4095,5.0/4095}};
    const Gains current{{2.4420024420024418},{.1111111111111111}};
    auto plan=req;plan.motion_probe=true;plan.amplitude_rad=.028;plan.frequencies_hz={.2,.4,.6857142857142857};plan.probe_duration_s=11.4;
    Metrics metric;metric.samples=metric.measurement_samples=2541;metric.actual_span=2*noise.encoder_step_rad;
    metric.applied_input_peak_nm=.0783786748736684;
    MotionObservation motion;motion.encoder_step_rad=noise.encoder_step_rad;motion.positive_travel_rad=noise.encoder_step_rad;
    motion.negative_travel_rad=2*noise.encoder_step_rad;motion.minimum_motion_span_rad=.01119997767141106;motion.response_rms_ratio=.008128583072283167;
    ResponseModel model;model.reason="motion_establishment_only_no_spectral_fit";
    MotionEstablishmentState state;state.excitation_retests={8};state.progress.resize(1);
    state.progress[0].blocked=true;state.progress[0].plateau_motion_confirmation=true;
    state.progress[0].anchor_input_nm=.04020272590547512;state.progress[0].anchor_span_rad=8*noise.encoder_step_rad;
    state.progress[0].anchor_minimum_travel_rad=noise.encoder_step_rad;
    const auto next=motion_establishment_step(current,{.007283656926727555},{wrist},{noise},{model},{metric},{motion},{plan},req,false,250,&state);
    const double old_input=prescribed_pd_input_peak(plan,current.kp[0],current.kd[0]);
    const double new_input=prescribed_pd_input_peak(next.plans[0],next.gains.kp[0],next.gains.kd[0]);
    check(next.retry && next.blocked_axes.empty() && !next.motion_established && !next.established && !next.feedback_fault,
        "1794278968578 joint5's fully sampled small-input single tone must retain legal calibration instead of a permanent insufficiency latch");
    check(independent_motion_damping_preserved(current,next.gains,{wrist}) && next.gains.kd[0]==current.kd[0] && next.gains.kp[0]>current.kp[0] &&
        next.gains.kp[0]<=1.4*current.kp[0]+wrist.grid.kp_step && new_input>old_input+2*noise.encoder_step_rad*current.kp[0],
        "the amplitude-saturated weak wrist needs a bounded resolvable P-only comparison without unmeasured D growth or identification");
    check(new_input<=.5*wrist.effort_budget+1e-9 && short_trial_cost(next.plans,req.tail_s)<250,
        "the new small-input proposal must retain the original force and reserved-time constraints");

    // Axis order and joint count must not convert one saturated axis into a
    // stop for all useful experiments. The final decision still needs every
    // axis's CURRENT measurement, not accumulated historical passes.
    auto saturated=wrist;saturated.kp_max=65*wrist.grid.kp_step;saturated.kd_max=164*wrist.grid.kd_step;
    const auto capped=symmetric_motion_probe_plan(req,saturated,saturated.kp_max,saturated.kd_max,noise,250,plan,.028,.2).requirements;
    auto ready_model=model;ready_model.tracking_valid=true;ready_model.reason="closed_loop_fit_insufficient";
    auto ready_motion=motion;ready_motion.qualified=ready_motion.bidirectional=true;
    for(unsigned count:{1U,3U,7U})for(unsigned weak_axis=0;weak_axis<count;++weak_axis) {
        Gains group;std::vector<ModelAxisLimits> limits;std::vector<double> inertia(count,.007283656926727555);
        std::vector<NoiseEstimate> noises(count,noise);std::vector<ResponseModel> models(count,model);
        std::vector<Metrics> metrics(count,metric);std::vector<MotionObservation> motions(count,motion);
        std::vector<Requirements> plans;
        for(unsigned i=0;i<count;++i){const bool active=i==weak_axis;group.kp.push_back(active?current.kp[0]:saturated.kp_max);
            group.kd.push_back(active?current.kd[0]:saturated.kd_max);limits.push_back(active?wrist:saturated);plans.push_back(active?plan:capped);}
        MotionEstablishmentState grouped;
        const auto step=motion_establishment_step(group,inertia,limits,noises,models,metrics,motions,plans,req,false,250,&grouped);
        check(step.retry && !step.motion_established && !step.established && !step.feedback_fault,
            "any N-axis permutation must continue its legal weak-axis measurement without promoting the incomplete full group");
        check(step.gains.kp[weak_axis]>group.kp[weak_axis] && independent_motion_damping_preserved(group,step.gains,limits) &&
            std::find(step.blocked_axes.begin(),step.blocked_axes.end(),weak_axis)==step.blocked_axes.end(),
            "the small-input wrist retains the same bounded continuation under axis permutation");
        for(unsigned i=0;i<count;++i)if(i!=weak_axis)check(step.gains.kp[i]==group.kp[i] && step.gains.kd[i]==group.kd[i] &&
            prescribed_pd_input_peak(step.plans[i],step.gains.kp[i],step.gains.kd[i])<=prescribed_pd_input_peak(plans[i],group.kp[i],group.kd[i])+1e-12,
            "a truly protocol/amplitude-saturated axis cannot invent input growth while another axis continues");
        std::fill(models.begin(),models.end(),ready_model);std::fill(motions.begin(),motions.end(),ready_motion);
        motions[weak_axis].qualified=false;
        check(!motion_establishment_step(group,inertia,limits,noises,models,metrics,motions,plans,req,false,250,&grouped).motion_established,
            "all other axes passing does not waive the weak axis's current complete-group motion requirement");
    }
    if(!recorded_directory.empty()) {
        const auto rows=read_joint_evidence_csv(recorded_directory+"/servo-probe-native-establish-11.csv");
        const std::vector<double> inertia{.11973876183976842,.3622863615136868,.24412967086252443,.023678950172938876,.007283656926727555,.0011237710292219616};
        std::vector<ModelAxisLimits> limits;std::vector<Requirements> plans;std::vector<ResponseModel> models;
        std::vector<Metrics> metrics;std::vector<MotionObservation> motions;Gains actual;
        const std::vector<double> amplitudes{.028,.014267028808593749,.024814666748046877,.028,.028,.028};
        const double common_frequency=1/(6*req.settling_time_s);
        for(unsigned i=0;i<6;++i) {
            check(rows[i].size()==2541,"the original last actual CSV must preserve all 2541 samples for every joint");
            auto bounds=wrist;bounds.effort_budget=i<3?7:2.5;
            if(i==1){bounds.reference_min_offset_rad=-.000762939453125;bounds.reference_max_offset_rad=.04;bounds.position_margin_rad=.5*(.04+.000762939453125);}
            if(i==2){bounds.reference_min_offset_rad=-.0308990478515625;bounds.reference_max_offset_rad=.04;bounds.position_margin_rad=.5*(.04+.0308990478515625);}
            limits.push_back(bounds);auto executed=plan;executed.amplitude_rad=amplitudes[i];
            if(i==1)executed.reference_bias_rad=.013504089355468749;
            const double f=i==3 || i==4?.2:common_frequency;executed.frequencies_hz={f,2*f,f*1.2/.35};plans.push_back(executed);
            actual.kp.push_back(rows[i].front().actual_kp);actual.kd.push_back(rows[i].front().actual_kd);
            auto observed=assess_trajectory(rows[i],executed,noise,executed.probe_duration_s);
            observed.metrics.applied_input_peak_nm=applied_pd_input_peak(rows[i],actual.kp[i],actual.kd[i]);
            check(observed.metrics.continuous && !observed.metrics.limited_samples && !observed.metrics.oscillating,
                "the original final low-input CSV remains healthy unlimited evidence, not a fabricated execution fault");
            models.push_back(motion_probe_evidence(rows[i],executed,actual.kp[i],actual.kd[i]));metrics.push_back(observed.metrics);motions.push_back(observed.motion);
        }
        close(metrics[4].applied_input_peak_nm,.0783786748736684,1e-10,"replay must reconstruct the actual transmitted joint5 PD command");
        check(!motions[4].qualified && motions[4].positive_travel_rad==noise.encoder_step_rad && motions[4].negative_travel_rad==2*noise.encoder_step_rad,
            "the original one/two-code wrist response remains insufficient under unchanged acceptance");
        MotionEstablishmentState replay;const auto proposal=motion_establishment_step(actual,inertia,limits,std::vector<NoiseEstimate>(6,noise),
            models,metrics,motions,plans,req,false,250,&replay);
        check(proposal.retry && !proposal.feedback_fault && !proposal.motion_established && independent_motion_damping_preserved(actual,proposal.gains,limits) &&
            proposal.gains.kp[4]>actual.kp[4] && std::find(proposal.blocked_axes.begin(),proposal.blocked_axes.end(),4)==proposal.blocked_axes.end(),
            "the actual 1794278968578 complete window must propose a legal unexecuted wrist measurement instead of ending the group");
        std::cout<<"1794278968578 actual joint5: PD="<<metrics[4].applied_input_peak_nm<<", next P="<<proposal.gains.kp[4]
            <<", next known drive="<<prescribed_pd_input_peak(proposal.plans[4],proposal.gains.kp[4],proposal.gains.kd[4])<<" (unexecuted)\n";
    }
    std::cout<<"low-input wrist calibration, real protocol saturation, axis permutations and incomplete-group rejection passed\n";
}

void check_disturbance_resolution_and_residual_correction(const std::string& recorded_directory) {
    Requirements req;req.probe_duration_s=5.5;const double encoder=25.0/65536;
    for(unsigned kind=0;kind<4;++kind) {
        std::vector<Sample> rows;
        for(unsigned k=0;k<=3500;++k){Sample row;row.time_s=.002*k;row.new_measurement=true;row.independent_disturbance_known=true;
            row.independent_disturbance_nm=row.time_s<5.5?.7*std::sin(2*pi*.45*row.time_s):0;
            if(kind<2)row.actual=row.time_s>.7?(k/10%2)*encoder:0;
            else if(row.time_s<5.5){row.actual=8*encoder*std::sin(2*pi*.45*row.time_s);row.velocity=8*encoder*2*pi*.45*std::cos(2*pi*.45*row.time_s);}
            else if(kind==2){row.actual=8*encoder*std::sin(2*pi*.45*5.5)*std::exp(-9*(row.time_s-5.5));row.velocity=-9*row.actual;}
            else {row.actual=8*encoder*std::sin(2*pi*10*(row.time_s-5.5));row.velocity=8*encoder*20*pi*std::cos(2*pi*10*(row.time_s-5.5));}
            rows.push_back(row);
        }
        auto plan=req;NoiseEstimate baseline;
        if(kind==1)plan.encoder_step_rad=2*encoder;
        if(kind>=2)baseline.encoder_step_rad=encoder;
        const auto measured=assess_disturbance(rows,plan,baseline,5.5,.004);
        if(kind<2)check(!measured.motion_observed && !measured.sustained_oscillation && !measured.measurement_complete && measured.static_measurement_complete &&
            measured.encoder_step_rad>0 && measured.motion_floor_rad>=3*encoder,
            "an initially constant baseline followed by one actual position code cannot identify stiffness or sustained release oscillation");
        else if(kind==2)check(measured.motion_observed && measured.measurement_complete && measured.passed,
            "a genuinely resolved disturbance response and release must retain the original physical acceptance");
        else check(measured.motion_observed && measured.sustained_oscillation && !measured.passed,
            "resolving encoder grain must not hide genuine multi-code release oscillation");
    }
    const ModelAxisLimits limit{.04,1.62,3.24,7,500,5,{500.0/4095,5.0/4095}};
    const Gains incumbent{{1977*500.0/4095},{5}},requested{{1.1*incumbent.kp[0]},{5}};
    const double E=1.25*.018803036847175658,V=1.25*.202103;
    const auto local=constrain_measured_gain_step(incumbent,requested,{limit},{E},{V});
    check(local.changed && local.gains.kp[0]>incumbent.kp[0] && local.gains.kp[0]<requested.kp[0] && local.gains.kd==incumbent.kd && local.predicted_effort_nm[0]<=7+1e-10,
        "the real full-path local step must preserve measured damping and constrain the requested increase without reversing its direction");
    const auto unsupported=constrain_measured_gain_step(incumbent,requested,{limit},{.04},{V});
    check(!unsupported.changed && unsupported.gains.kp==incumbent.kp && unsupported.gains.kd==incumbent.kd,
        "an unsupported robust error envelope must report no increase rather than project a stiffness trial into an unrequested P/D decrease");
    const auto old_budget=plan_residual_correction_budget(159.15,34.74481,34.74481,93.2997,2);
    const auto remaining_costs=plan_residual_correction_budget(159.15,32.7464,32.7464,90.2997,2);
    check(!old_budget.fits && old_budget.deficit_s>5 && remaining_costs.fits && remaining_costs.required_s<159.15,
        "correction scheduling must expose real return/comparison/reserve costs, not silently disappear over a few seconds or extend the original deadline");

    // Retention considers full-cycle peaks before the auxiliary RMS objective.
    TrajectoryAssessment good;good.reference_coverage=good.responsive=true;good.metrics.samples=good.metrics.measurement_samples=1000;
    good.metrics.rms_error=.01;good.metrics.peak_error=.02;good.tuning.valid=true;good.tuning.samples=1000;
    good.tuning.reference_rms_rad=.1;good.tuning.centered_rms_rad=.01;good.tuning.position_floor_rad=.0002;
    auto peak_improvement=good;peak_improvement.metrics.peak_error=.015;
    auto rms_only=good;rms_only.metrics.rms_error=.007;rms_only.tuning.centered_rms_rad=.007;rms_only.metrics.peak_error=.025;
    check(trajectory_trial_preferred(Gains{{11},{1}},{peak_improvement},Gains{{10},{1}},{good},Gains{{1},{.1}},req),
        "a resolved all-cycle peak improvement may retain a complete candidate without requiring an unrelated RMS improvement");
    check(!trajectory_trial_preferred(Gains{{11},{1}},{rms_only},Gains{{10},{1}},{good},Gains{{1},{.1}},req),
        "RMS improvement must never hide a worsened all-cycle peak");

    if(!recorded_directory.empty()) {
        const auto small=read_joint_evidence_csv(recorded_directory+"/servo-probe-native-established-disturbance.csv");
        const auto large=read_joint_evidence_csv(recorded_directory+"/servo-probe-native-established-disturbance-supplement.csv");
        const auto unresolved=assess_disturbance(small[0],req,NoiseEstimate{},5.5,.004);
        const auto resolved=assess_disturbance(large[0],req,NoiseEstimate{},5.5,.004);
        check(!unresolved.motion_observed && !unresolved.sustained_oscillation && !unresolved.measurement_complete && unresolved.static_measurement_complete,
            "1794241207035 actual one-code joint1 disturbance must remain static evidence, not a false motion/release-oscillation measurement");
        close(unresolved.encoder_step_rad,encoder,1e-12,"the actual disturbance window must recover the encoder grain absent from the initial stationary baseline");
        check(resolved.motion_observed && resolved.measurement_complete,"actual multi-code supplemented disturbance must retain its resolved measurement");
        const auto path=read_joint_evidence_csv(recorded_directory+"/gain-trajectory-native-frozen.csv");
        std::ifstream result(recorded_directory+"/result.json");const std::string json((std::istreambuf_iterator<char>(result)),std::istreambuf_iterator<char>());
        const auto key=json.find("\"gain_trajectory_duration_s\"");check(key!=std::string::npos,"actual task metadata must supply its executed path duration");
        const double duration=std::stod(json.substr(json.find(':',key)+1));
        Gains current;std::vector<TrajectoryAssessment> assessments;std::vector<double> error_bounds,velocity_bounds;std::vector<ModelAxisLimits> limits;
        for(std::size_t i=0;i<path.size();++i) {
            check(!path[i].empty(),"actual native path needs all six joint rows");current.kp.push_back(path[i][0].actual_kp);current.kd.push_back(path[i][0].actual_kd);
            NoiseEstimate n;n.encoder_step_rad=encoder;auto a=assess_trajectory(path[i],req,n,duration);assessments.push_back(a);
            double velocity_error=0;for(const auto& row:path[i])velocity_error=std::max(velocity_error,std::abs(row.reference_velocity-row.velocity));
            error_bounds.push_back(std::max(1.25*a.metrics.peak_error,a.metrics.peak_error+3*encoder));velocity_bounds.push_back(std::max(.01,1.25*velocity_error));
            auto x=limit;x.effort_budget=i<3?7:2.5;limits.push_back(x);
            check(a.tuning.valid && a.tuning.velocity_error_rms_rad>a.tuning.amplitude_error_rms_rad+a.tuning.position_floor_rad && !a.metrics.limited_samples && !a.metrics.oscillating && !a.passed,
                "the actual full path must retain its valid velocity-related residual, strict failure, and absence of saturation/oscillation");
        }
        const auto proposal=trajectory_candidate_report(current,std::vector<double>(6,500),std::vector<double>(6,5),assessments,req);
        const auto bounded=constrain_measured_gain_step(current,proposal.gains,limits,error_bounds,velocity_bounds);
        check(proposal.changed && bounded.changed,"actual failed native tracking must generate an evidence-driven bounded local comparison without a successful physical fit");
        std::cout<<"1794241207035 actual disturbance resolution="<<unresolved.encoder_step_rad<<", floor="<<unresolved.motion_floor_rad<<"; one-code false motion/oscillation removed\n";
        for(std::size_t i=0;i<path.size();++i) {
            check(bounded.gains.kp[i]>=current.kp[i] && bounded.gains.kp[i]<=1.1*current.kp[i]+1e-10 && bounded.gains.kd[i]>=current.kd[i] && bounded.predicted_effort_nm[i]<=limits[i].effort_budget+1e-9,
                "every actual-path suggestion must remain a small local step within its measured joint-effort envelope");
            std::cout<<"path axis "<<i+1<<": lag="<<assessments[i].tuning.lag_s<<", E="<<error_bounds[i]<<", V="<<velocity_bounds[i]
                <<", proposed="<<bounded.gains.kp[i]<<'/'<<bounded.gains.kd[i]<<", predicted envelope="<<bounded.predicted_effort_nm[i]<<" Nm (unexecuted)\n";
        }
    }
    std::cout<<"disturbance encoder floor, measured local-step direction, peak-first retention and explicit correction cost checks passed\n";
}

int main(int argc,char** argv) {
    try {
        const bool motion_latest=argc>2 && std::string(argv[1])=="--motion-task";
        const bool wrist_latest=argc>2 && std::string(argv[1])=="--wrist-task";
        const bool latest=argc>2 && (std::string(argv[1])=="--latest-task" || motion_latest || wrist_latest);
        check_shape_preserving_retime();
        check_protocol_demand_and_working_domain();check_recorded_quantization_boundary();
        check_unidentified_response_and_execution_guard(!latest && argc>1?argv[1]:"");
        check_establishment_input_plateaus(!latest && argc>2?argv[2]:"");
        check_symmetric_motion_establishment(!latest && argc>3?argv[3]:"");
        check_observed_reference_domain(motion_latest?argv[2]:"");
        check_shared_slow_establishment(motion_latest?argv[2]:"");
        check_low_input_wrist_calibration(wrist_latest?argv[2]:"");
        check_disturbance_resolution_and_residual_correction(latest && !motion_latest && !wrist_latest?argv[2]:"");
        check_six_axis_motion_establishment();
        const ModelAxisLimits limit{.2,1.5,4,8,150,4,{}};
        ModelResponseDemand model_demand;model_demand.frequency_hz={.4,.4};model_demand.velocity_peak={.02,.02};model_demand.acceleration_peak={.02,.02};
        const auto initialized=model_initialization({{.2,.05},{.3,.08}},{.1,.02},{limit,limit},model_demand);
        check(initialized.source=="urdf_task_response_and_protocol_grid" && !initialized.measured,"model source must remain unmeasured");
        check(initialized.gains.kp[0]>initialized.gains.kp[1],"axis inertia must determine its model gains");
        const Gains old_low{{1,2},{.1,.2}},old_high{{100,120},{3,4}};
        (void)old_low;(void)old_high;
        const auto unchanged=model_initialization({{.2,.05},{.3,.08}},{.1,.02},{limit,limit},model_demand);
        check(unchanged.gains.kp==initialized.gains.kp && unchanged.gains.kd==initialized.gains.kd,"configured gains may not enter initialization");
        const auto seven=model_initialization({{.1,.2,.3,.4,.5,.6,.7}},{0,0,0,0,0,0,0},std::vector<ModelAxisLimits>(7,limit));
        check(seven.gains.kp.size()==7,"joint count must be generic");
        const ModelAxisLimits limited{.02,.1,.3,.02,10,.1,{}};
        const auto bounded=model_initialization({{.3}},{0},{limited});
        check(bounded.gains.kp[0]<=limited.kp_max && bounded.gains.kd[0]<=limited.kd_max && !bounded.constraints[0].empty(),"hardware and motion limits must constrain model gains");
        bool invalid=false;try{model_initialization({{0}},{0},{limit});}catch(const std::invalid_argument&){invalid=true;}
        check(invalid,"invalid inertia must not silently fall back to constants");
        invalid=false;try{model_initialization({{.2,.3}},{0},{limit});}catch(const std::invalid_argument&){invalid=true;}
        check(invalid,"mapping mismatch must fail");

        Requirements req;req.probe_duration_s=8;req.amplitude_rad=.012;req.frequencies_hz={.35,.7,1.2};
        std::vector<Sample> boundary_rows;
        for(unsigned k=0;k<=4750;++k) {
            Sample row;row.time_s=k*.002;const auto reference=probe(row.time_s,req);
            row.reference=reference.first;row.reference_velocity=reference.second;row.actual=row.reference+pi/1800;
            row.velocity=row.reference_velocity;row.new_measurement=true;boundary_rows.push_back(row);
        }
        auto exact=req;exact.peak_error_rad=assess(boundary_rows,req,req.probe_duration_s).endpoint_peak_error_rad;
        auto below=exact;below.peak_error_rad=std::nextafter(exact.peak_error_rad,std::numeric_limits<double>::infinity());
        check(assess(boundary_rows,below,req.probe_duration_s).passed && !assess(boundary_rows,exact,req.probe_duration_s).passed,
            "final-target maximum error equal to the configured target must fail the strict less-than boundary");
        check(assess_trajectory(boundary_rows,below,NoiseEstimate{},req.probe_duration_s).passed &&
            !assess_trajectory(boundary_rows,exact,NoiseEstimate{},req.probe_duration_s).passed,
            "the real trajectory acceptance entry must preserve the same strict final-target boundary");

        MitExperiment mit{8,1.0,.2,0,ExcitationKind::ReferenceTracking};
        auto rows=spectral_experiment(req,mit,.15,.012,false,1e-6);
        auto measured=identify_mit(rows,req,mit);
        check(measured.tracking_valid && measured.identifiable,"MIT tracking experiment must identify a known linear response");
        close(measured.inertia_estimate,.2,.025,"MIT derivative-reference zero must not bias inertia");
        close(measured.effective_delay_s,.012,.008,"effective command/response delay must be recovered");
        check(!measured.disturbance_valid,"good tracking must not fabricate stiffness evidence");
        const MitExperiment gravity_ff{8,1,.2,2,ExcitationKind::ReferenceTracking,2};
        const auto gravity_rows=spectral_experiment(req,gravity_ff,.15);
        const auto gravity_identified=identify_mit(gravity_rows,req,gravity_ff);
        check(gravity_identified.identifiable && gravity_identified.tracking_valid,"known gravity(actual) feedforward must fit using raw executed FF and the physical gravity denominator");
        close(gravity_identified.physical_stiffness_nm_rad,10,.01,"physical fitted stiffness must retain Kp plus actual gravity gradient");
        close(gravity_identified.effective_stiffness_nm_rad,8,.01,"executed state-FF gradient must not be counted as feedback restoring stiffness");
        close(gravity_identified.natural_frequency_rad_s,std::sqrt(8/.2),.01,"interpreted closed-loop natural frequency must subtract known state-FF gradient");
        close(gravity_identified.damping_ratio,1.15/(2*std::sqrt(8*.2)),.01,"interpreted damping ratio must use effective restoring stiffness");
        auto wrong_nominal=gravity_ff;wrong_nominal.gravity_gradient=4;
        const auto actual_fit=identify_mit(gravity_rows,req,wrong_nominal);
        close(actual_fit.physical_stiffness_nm_rad,10,.01,"a nominal gravity prior must not overwrite actual measured physical stiffness");
        close(actual_fit.nominal_physical_stiffness_nm_rad,12,1e-12,"the nominal gravity prior must remain explicitly distinguishable from the fitted physical value");
        auto actual_gain_rows=rows;for(auto& row:actual_gain_rows){row.actual_gains_known=true;row.actual_kp=mit.kp;row.actual_kd=mit.kd;}
        check(identify_mit(actual_gain_rows,req,mit).identifiable,"stable actual executed gains must retain valid MIT identification");
        actual_gain_rows[800].actual_kp*=.95;
        const auto transitioning=identify_mit(actual_gain_rows,req,mit);
        check(!transitioning.identifiable && !transitioning.tracking_valid && transitioning.reason=="changing_feedback_gains",
            "a transition in actual executed gain cannot be fit as a successful constant-gain MIT experiment");
        Metrics transition_health;transition_health.samples=transition_health.measurement_samples=800;
        const auto waiting=motion_establishment_step(Gains{{mit.kp},{mit.kd}},{mit.inertia_prior},{limit},{NoiseEstimate{}},
            {transitioning},{transition_health},{MotionObservation{}},{req},req,false,60);
        check(waiting.retry && !waiting.feedback_fault && waiting.gains.kp[0]==mit.kp && waiting.gains.kd[0]==mit.kd,
            "healthy gain handover must wait for stable remeasurement instead of increasing input or claiming response success");
        Sample actual_input;actual_input.reference=.01;actual_input.actual_gains_known=true;actual_input.actual_kp=2;actual_input.actual_kd=.2;
        close(applied_pd_input_peak({actual_input},100,5),.02,1e-12,"actual input evidence must use executed gain, not the requested candidate context");
        const auto reference_only=identify(rows,req);
        check(reference_only.tracking_valid && !reference_only.identifiable,"legacy zero-free fit must not report MIT poles");

        auto ff_rows=spectral_experiment(req,MitExperiment{1,.1,.2,0,ExcitationKind::ReferenceTracking},.1,0,true);
        const auto feedforward_fit=identify_mit(ff_rows,req,MitExperiment{1,.1,.2,0,ExcitationKind::ReferenceTracking});
        check(feedforward_fit.tracking_valid && !feedforward_fit.disturbance_valid,"inverse dynamics tracking cannot validate feedback stiffness");
        auto unknown_ff=rows;for(auto& row:unknown_ff)row.feedforward_known=false;
        auto unknown=identify_mit(unknown_ff,req,mit);
        check(unknown.tracking_valid && !unknown.identifiable && !unknown.missing_evidence.empty(),"unknown feedforward must preserve tracking evidence and withhold physical fit");
        auto saturated=rows;saturated[700].limited=true;
        check(!identify_mit(saturated,req,mit).tracking_valid,"limited actual input cannot pass spectral evidence");
        auto gaps=rows;for(std::size_t k=800;k<900;++k)gaps[k].new_measurement=false;
        check(!identify_mit(gaps,req,mit).tracking_valid,"missing feedback must not pass");
        auto repeated=rows;for(auto& row:repeated)row.feedback_sequence=1;
        check(!identify_mit(repeated,req,mit).tracking_valid,"duplicate feedback must not count as independent samples");
        auto noisy=rows;for(auto& row:noisy)row.actual+=.03*std::sin(83.7*row.time_s);
        check(!identify_mit(noisy,req,mit).tracking_valid,"noise-dominated short test must remain insufficient");
        MitExperiment disturbance=mit;disturbance.kind=ExcitationKind::IndependentTorqueDisturbance;
        auto disturbed=identify_mit(spectral_experiment(req,disturbance,.15,.012),req,disturbance);
        check(disturbed.disturbance_valid && disturbed.identifiable && !disturbed.tracking_valid,"independent disturbance must produce its own evidence");
        check(std::isfinite(disturbed.low_frequency_compliance_rad_nm),"disturbance compliance must be measured");

        Gains capped{{8},{4}};ResponseModel lag=measured;lag.tracking_peak_error_rad=.02;lag.reference_lag_s=.05;
        ResponseModel stiff_goal;stiff_goal.tracking_valid=true;stiff_goal.disturbance_valid=true;
        stiff_goal.tracking_peak_error_rad=.1*req.peak_error_rad;stiff_goal.low_frequency_compliance_rad_nm=1.0/capped.kp[0];
        stiff_goal.disturbance_goal_evaluated=true;stiff_goal.disturbance_goal_met=false;
        check(adaptive_candidate(capped,{stiff_goal},{limit},req).gains.kp[0]>capped.kp[0],"failed independent disturbance goal must revise gains even with good tracking");
        auto proposal=adaptive_candidate(capped,{lag},{limit},req);
        check(proposal.changed && proposal.gains.kp[0]>8 && proposal.gains.kd[0]<=4,"damping ceiling must not freeze stiffness response to measured lag");
        auto quiet=lag;quiet.tracking_peak_error_rad=0;quiet.reference_lag_s=0;quiet.identifiable=false;
        check(adaptive_candidate(capped,{quiet},{limit},req).gains.kp[0]!=proposal.gains.kp[0],"real identification must affect generated candidate");
        lag.sustained_oscillation=true;
        check(adaptive_candidate(capped,{lag},{limit},req).gains.kp[0]<8,"sustained oscillation must lower stiffness");
        lag.sustained_oscillation=false;lag.dynamic_overshoot=true;
        auto overshoot=adaptive_candidate(capped,{lag},{limit},req);
        check(overshoot.reasons[0].find("dynamic_overshoot")!=std::string::npos && overshoot.reasons[0].find("resonance")==std::string::npos,"single overshoot is not mechanical resonance");
        TrajectoryAssessment a;a.reference_coverage=a.responsive=true;a.metrics.samples=a.metrics.measurement_samples=1000;
        a.tuning.valid=true;a.tuning.samples=1000;a.tuning.reference_rms_rad=.01;a.tuning.position_floor_rad=.0002;
        a.tuning.lag_s=.1;a.tuning.velocity_error_rms_rad=.01;a.tuning.centered_rms_rad=.01;a.tuning.in_phase_gain=.99;
        auto path_proposal=trajectory_candidate(capped,{150},{4},{a},req);
        check(path_proposal.kp[0]>8,"velocity-correlated lag must allow stiffness correction");
        a.tuning.amplitude_error_rms_rad=.02;a.tuning.velocity_error_rms_rad=0;a.tuning.in_phase_gain=.7;
        check(trajectory_candidate(capped,{150},{4},{a},req).kp[0]>8,"amplitude deficit must permit Kp correction at Kd ceiling");

        Requirements recovery_req=req;recovery_req.hold_error_rad=.001;recovery_req.settling_time_s=.7;
        std::vector<Sample> release_rows;
        for(unsigned k=0;k<=2250;++k) {
            Sample row;row.time_s=k*.002;row.new_measurement=true;row.independent_disturbance_known=true;
            row.reference=0;row.actual=.0001;
            if(row.time_s>=.2 && row.time_s<3) {row.independent_disturbance_nm=.01;row.actual+=.0007;}
            if(row.time_s>=3){row.actual+=.0007*std::exp(-7*(row.time_s-3));row.velocity=-.0049*std::exp(-7*(row.time_s-3));}
            release_rows.push_back(row);
        }
        NoiseEstimate recovery_noise;recovery_noise.encoder_step_rad=1e-6;
        auto recovery=assess_disturbance(release_rows,recovery_req,recovery_noise,3,.001);
        check(recovery.measurement_complete && recovery.passed,"bounded independent torque and actual release recovery must qualify separately");
        close(recovery.peak_displacement_rad,.0007,1e-8,"forced displacement must exclude initial static error");
        auto too_soft=release_rows;for(auto& row:too_soft)if(row.time_s>=.2 && row.time_s<3)row.actual=.003;
        auto soft_result=assess_disturbance(too_soft,recovery_req,recovery_noise,3,.001);
        check(soft_result.measurement_complete && !soft_result.passed && soft_result.reason=="disturbance_displacement_above_goal",
            "good final tracking cannot validate insufficient disturbance stiffness");
        auto bias_release=release_rows;for(auto& row:bias_release)row.actual+=.003;
        check(assess_disturbance(bias_release,recovery_req,recovery_noise,3,.001).reason=="final_static_error_above_goal",
            "post-disturbance static error must be separate from displacement and dynamic overshoot");
        auto no_input=release_rows;for(auto& row:no_input)row.independent_disturbance_nm=0;
        check(!assess_disturbance(no_input,recovery_req,recovery_noise,3,.001).measurement_complete,"missing disturbance cannot become successful evidence");
        auto moving_reference=release_rows;moving_reference[900].reference=.01;
        check(!assess_disturbance(moving_reference,recovery_req,recovery_noise,3,.001).measurement_complete,"moving reference invalidates independent disturbance comparison");

        const Gains weak{{3},{.4}};
        auto simulated=simulate_mit(req,weak);
        auto simulation_evidence=identify_mit(simulated,req,MitExperiment{3,.4,.2,0,ExcitationKind::ReferenceTracking});
        check(simulation_evidence.tracking_valid,"integrated plant short test must enter the same identification API");
        simulation_evidence.tracking_peak_error_rad=std::max(simulation_evidence.tracking_peak_error_rad,2*req.peak_error_rad);
        const auto simulated_proposal=adaptive_candidate(weak,{simulation_evidence},{limit},req);
        const auto compared=simulate_mit(req,simulated_proposal.gains);
        std::cout<<"delayed MIT proposal="<<simulated_proposal.gains.kp[0]<<'/'<<simulated_proposal.gains.kd[0]
            <<", reason="<<simulated_proposal.reasons[0]<<", zeta="<<simulation_evidence.damping_ratio
            <<", delay="<<simulation_evidence.effective_delay_s<<", errors="<<interior_error(simulated,req.probe_duration_s)
            <<'/'<<interior_error(compared,req.probe_duration_s)<<'\n';
        check(interior_error(compared,req.probe_duration_s)<.99*interior_error(simulated,req.probe_duration_s),
            "model and short-test generated complete candidate must improve reproducible delayed/frictional MIT plant");
        std::cout<<"delayed MIT plant RMS: "<<interior_error(simulated,req.probe_duration_s)
            <<" -> "<<interior_error(compared,req.probe_duration_s)<<" rad\n";

        CalibrationBudget quick(25,CalibrationMode::QuickAll),fine(25,CalibrationMode::FineSingle);
        close(quick.target_s,300,0,"quick target must be 12T");close(quick.hard_limit_s,375,0,"quick hard cap must be 15T");
        close(fine.target_s,300,0,"fine target must be 12T");close(fine.hard_limit_s,600,0,"fine hard cap must be 24T");
        check(!quick.can_start(370,5) && quick.can_start(300,20,20),"whole-trial and exit/validation reservation must fit");
        check(quick.expired(375) && !quick.expired(374),"deadline must be strict");
        NoiseEstimate noise;noise.encoder_step_rad=1e-5;noise.rms_rad=1e-6;
        auto plan=short_probe_plan(req,limit,8,1,noise,false,30);
        check(plan.valid && plan.requirements.probe_duration_s<8,"quick identification must have bounded short experiment");
        check(!short_probe_plan(req,limit,8,1,noise,false,2).valid,"short test must honor remaining budget");
        noise.rms_rad=.1;
        check(!short_probe_plan(req,limit,8,1,noise,false,30).valid,"unobservable probe cannot become a successful default estimate");
        std::cout<<"adaptive gain offline checks passed: mapped model seeds, MIT input definition, tracking/disturbance evidence, feedforward, noise, timing, ceilings, candidate direction, 12T/15T/24T budgets\n";
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
