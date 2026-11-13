// Deterministic closed-loop MIT frequency evidence and sampled PD design.
// All excitation/feedback is generated offline; no device or Profile is used.
#include "serial_arm/dynamics/frequency_response_gain_tuning.hpp"
#include "serial_arm/dynamics/mit_closed_loop_autotune.hpp"
#include <iostream>
#include <stdexcept>
using namespace serial_arm::servo_tuning;
void check(bool good,const char* why){if(!good)throw std::runtime_error(why);}
struct Plant {double inertia{.2},damping{.15},stiffness{.5},delay{.012};};
std::complex<double> physical_plant(const Plant& p,double hz) {
    const double w=2*pi*hz;return std::exp(std::complex<double>{0,-w*p.delay})/
        std::complex<double>{p.stiffness-p.inertia*w*w,p.damping*w};
}
std::vector<Sample> experiment(const Requirements& req,double kp,double kd,const Plant& plant,
    bool disturbance=false,bool inverse_feedforward=false,double position_noise=0,double state_ff_gradient=0,
    double state_velocity_ff=0,double velocity_filter_s=0) {
    std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>(req.probe_duration_s/.002);++k) {
        Sample row;row.time_s=k*.002;row.command_time_s=row.feedback_time_s=row.time_s;
        row.new_measurement=true;row.feedback_sequence=k+1;row.actual_gains_known=true;row.actual_kp=kp;row.actual_kd=kd;
        row.feedforward_known=row.independent_disturbance_known=true;row.applied_feedforward_nm=0;row.independent_disturbance_nm=0;
        for(unsigned j=0;j<3;++j) {
            const double w=2*pi*req.frequencies_hz[j];const auto P=physical_plant(plant,req.frequencies_hz[j]);
            const auto Hv=std::complex<double>{0,w}/std::complex<double>{1,w*velocity_filter_s};const auto C=kp+kd*Hv;
            const std::complex<double> R=disturbance?0:req.amplitude_rad*(j==0?1:.8);
            const std::complex<double> D=disturbance?.08*(j==0?1:.8):0;
            const auto state_ff=state_ff_gradient+state_velocity_ff*Hv;
            const auto F=inverse_feedforward?R/P:std::complex<double>{0,0};const auto Y=P*(C*R+F+D)/(1.0+P*(C-state_ff));
            const auto at=[&](std::complex<double> value){return (value*std::exp(std::complex<double>{0,w*row.time_s})).imag();};
            row.reference+=at(R);row.reference_velocity+=at(std::complex<double>{0,w}*R);
            row.actual+=at(Y);row.velocity+=at(Hv*Y);
            row.applied_feedforward_nm+=at(F+state_ff*Y);row.independent_disturbance_nm+=at(D);
        }
        row.actual+=position_noise*std::sin(91.7*row.time_s);rows.push_back(row);
    }return rows;
}
EmpiricalMitFrequencyResponse isolated_response(const std::vector<Sample>& rows,const Requirements& req,const MitExperiment& mit) {
    return estimate_mit_frequency_response(rows,req,mit,true,true);
}

namespace {
void controller_failure_reason_survives_incomplete_sequence() {
    MitClosedLoopTrialEstimate measured_failure;
    measured_failure.reason="mit_independent_position_tone_below_noise";
    measured_failure.missing_evidence={"resolved_raw_position_response_at_each_tone_and_half"};
    const auto stopped=combine_mit_closed_loop_trials(std::vector<MitClosedLoopTrialEstimate>{measured_failure});
    check(!stopped.valid && !stopped.gain_holdout_verified && stopped.reason==measured_failure.reason &&
          stopped.response.reason==measured_failure.reason && stopped.missing_evidence==measured_failure.missing_evidence &&
          stopped.response.missing_evidence==measured_failure.missing_evidence,
        "a recorded controller-0 failure was overwritten by the number of unperformed later experiments");
    auto later_failure=measured_failure;later_failure.reason="mit_independent_tone_fit_quality_insufficient";
    later_failure.missing_evidence={"repeatable_three_tone_position_and_known_input_R_squared"};
    const auto several=combine_mit_closed_loop_trials(std::vector<MitClosedLoopTrialEstimate>{measured_failure,later_failure});
    check(several.reason==measured_failure.reason && several.missing_evidence==measured_failure.missing_evidence,
        "aggregation lost the first concrete failure in favor of a later failed experiment");
    const auto after_unmeasured=combine_mit_closed_loop_trials(
        std::vector<MitClosedLoopTrialEstimate>{MitClosedLoopTrialEstimate{},later_failure});
    check(after_unmeasured.reason==later_failure.reason,"an unmeasured default hid actual subsequent failure evidence");
    check(combine_mit_closed_loop_trials(std::vector<MitClosedLoopTrialEstimate>{}).reason=="mit_controller_experiments_insufficient" &&
          combine_mit_closed_loop_trials(std::vector<MitClosedLoopTrialEstimate>{MitClosedLoopTrialEstimate{}}).reason==
              "mit_controller_experiments_insufficient",
        "missing experiments without a concrete measured failure must retain the existing insufficient-evidence reason");
}

struct InternalMitPort {
    Plant mechanical;
    double command_transport_s{.013},return_transport_s{.012},host_feedback_age_s{.016};
    double internal_position_filter_s{0},internal_velocity_filter_s{0},reported_velocity_filter_s{.080};
    double fixed_position_feedforward{0},fixed_velocity_feedforward{0};
    bool coupled_two_axis{false};double held_other_kp{10},held_other_kd{.5};
};
std::complex<double> mechanical_port(const InternalMitPort& p,double hz) {
    if(!p.coupled_two_axis)return physical_plant(p.mechanical,hz);
    const double w=2*pi*hz;const std::complex<double> z11{.5-.2*w*w,.15*w},z22{.7-.12*w*w,.12*w},z12{-.1-.06*w*w,.01*w};
    const auto delay=std::exp(std::complex<double>{0,-w*p.mechanical.delay}),det=z11*z22-z12*z12;
    const auto G11=delay*z22/det,G22=delay*z11/det,G12=-delay*z12/det;
    const std::complex<double> held_controller{p.held_other_kp,w*p.held_other_kd};
    return G11-G12*held_controller*G12/(1.0+G22*held_controller);
}
std::complex<double> position_channel(const InternalMitPort& p,double hz) {
    return 1.0/std::complex<double>{1,2*pi*hz*p.internal_position_filter_s};
}
std::complex<double> velocity_channel(const InternalMitPort& p,double hz) {
    const double w=2*pi*hz;
    return std::complex<double>{0,w}/std::complex<double>{1,w*p.internal_velocity_filter_s};
}
std::complex<double> port_feedforward(const InternalMitPort& p,double hz) {
    return p.fixed_position_feedforward+p.fixed_velocity_feedforward*velocity_channel(p,hz);
}
std::complex<double> effective_loop_plant(const InternalMitPort& p,double hz) {
    const auto G=mechanical_port(p,hz);
    return G*position_channel(p,hz)/(1.0-G*port_feedforward(p,hz));
}
std::complex<double> internal_closed_loop(const InternalMitPort& p,double P,double D,double hz) {
    const auto G=mechanical_port(p,hz);
    const double w=2*pi*hz;
    const auto E=std::exp(std::complex<double>{0,-w*(p.command_transport_s+p.return_transport_s)});
    return E*G/(1.0+G*(P*position_channel(p,hz)+D*velocity_channel(p,hz)-port_feedforward(p,hz)));
}
// These fixtures evaluate the independent commanded torque and the returned
// position on their own clocks. The internal MIT feedback is never replaced
// by returned q/v; in particular the reported velocity filter is different.
std::vector<Sample> internal_port_experiment(const Requirements& req,const InternalMitPort& p,
    double P,double D,double torque_amplitude=.06,double input_start_s=.25) {
    std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>((req.probe_duration_s+input_start_s+req.tail_s)/.002);++k) {
        Sample row;row.time_s=k*.002;row.command_time_s=row.time_s;row.feedback_time_s=row.time_s-p.host_feedback_age_s;
        row.new_measurement=true;row.feedback_sequence=k+1;row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;
        row.feedforward_known=row.independent_disturbance_known=true;row.applied_feedforward_nm=0;
        const auto envelope=[&](double time) {
            if(time<0 || time>=req.probe_duration_s)return 0.;
            const double edge=std::clamp(std::min(time,req.probe_duration_s-time)/.7,0.,1.);
            return edge*edge*(3-2*edge);
        };
        for(unsigned j=0;j<3;++j) {
            const double hz=req.frequencies_hz[j],w=2*pi*hz,weight=j==0?.4:j==1?.35:.25;
            const std::complex<double> instrument=torque_amplitude*weight;
            const auto H=internal_closed_loop(p,P,D,hz),Y=H*instrument;
            const auto sampled=[&](std::complex<double> value,double clock) {
                return (value*std::exp(std::complex<double>{0,w*(clock-input_start_s)})).imag()*envelope(clock-input_start_s);
            };
            row.independent_disturbance_nm+=sampled(instrument,row.command_time_s);
            row.actual+=sampled(Y,row.feedback_time_s);
            const auto reported_velocity=std::complex<double>{0,w}/std::complex<double>{1,w*p.reported_velocity_filter_s};
            row.velocity+=sampled(reported_velocity*Y,row.feedback_time_s);
            row.applied_feedforward_nm+=sampled(port_feedforward(p,hz)*Y*
                std::exp(std::complex<double>{0,w*p.return_transport_s}),row.command_time_s);
        }
        rows.push_back(row);
    }
    return rows;
}

void three_controller_internal_port_identification() {
    Requirements req;req.probe_duration_s=16;req.amplitude_rad=.01;req.frequencies_hz={.35,.7,1.2};
    req.encoder_step_rad=1e-6;req.noise_rms_rad=1e-6;
    InternalMitPort port;port.mechanical.delay=.004;
    MitAutotuneOperatingPoint point;point.actual_gains={{4,10},{.3,.5}};point.fixed_reference_rad={0,0};
    point.excited_axis=0;point.condition_id="same_measured_interior_posture_load_feedforward_and_other_axis_hold";
    point.isolated_axis_excitation=true;
    const auto measure=[&](const InternalMitPort& physical,const MitAutotuneOperatingPoint& op) {
        return estimate_mit_closed_loop_trial(internal_port_experiment(req,physical,op.actual_gains.kp[0],op.actual_gains.kd[0]),req,op);
    };
    const auto triplet=[&](const InternalMitPort& physical) {
        auto p=point,d=point,pd=point;p.actual_gains.kp[0]=5;d.actual_gains.kd[0]=.6;
        pd.actual_gains.kp[0]=5;pd.actual_gains.kd[0]=.6;
        return std::array<MitClosedLoopTrialEstimate,4>{measure(physical,point),measure(physical,p),measure(physical,d),measure(physical,pd)};
    };
    auto trials=triplet(port);
    for(const auto& trial:trials)check(trial.valid,
        "each independently commanded-torque/returned-position experiment must be valid without pretending host reconstructed PD is actual torque");
    const auto identified=combine_mit_closed_loop_trials(trials);
    check(identified.valid && identified.response.points.size()==3,
        "three noncollinear measured internal MIT controllers plus an independent fourth controller must identify and validate the local equivalent feedback port");
    for(const auto& p:identified.response.points) {
        check(std::abs(p.plant/effective_loop_plant(port,p.hz)-1.0)<1e-6,
            "the common command and receive transport must cancel from the internal-feedback loop plant");
        const auto true_velocity=velocity_channel(port,p.hz)/position_channel(port,p.hz);
        check(std::abs(p.feedback_velocity_per_position/true_velocity-1.0)<1e-6,
            "the internal damping channel must be identified from gain variation instead of the unrelated returned velocity filter");
    }
    auto filtered=port;filtered.internal_position_filter_s=.025;filtered.internal_velocity_filter_s=.012;
    filtered.fixed_position_feedforward=.15;filtered.fixed_velocity_feedforward=.02;
    const auto compensated=combine_mit_closed_loop_trials(triplet(filtered));
    check(compensated.valid,"fixed state-feedforward and internal feedback filtering must retain the affine inverse closed-loop relation");
    for(const auto& p:compensated.response.points)check(
        std::abs(p.plant/effective_loop_plant(filtered,p.hz)-1.0)<1e-6 &&
        std::abs(p.feedback_velocity_per_position/(velocity_channel(filtered,p.hz)/position_channel(filtered,p.hz))-1.0)<1e-6,
        "equivalent plant and relative internal velocity channel must remain distinct from physical inertia and reported feedback");
    auto different_delay=port;different_delay.command_transport_s=.035;different_delay.return_transport_s=.020;
    const auto delayed=combine_mit_closed_loop_trials(triplet(different_delay));
    check(delayed.valid,"a different common external transport must not be reinterpreted as changed mechanical damping");
    for(std::size_t i=0;i<identified.response.points.size();++i)check(
        std::abs(delayed.response.points[i].plant/identified.response.points[i].plant-1.0)<1e-6 &&
        std::abs(delayed.response.points[i].feedback_velocity_per_position/identified.response.points[i].feedback_velocity_per_position-1.0)<1e-6,
        "transport cancellation must retain the same internal loop design data");
    auto coupled=port;coupled.coupled_two_axis=true;
    const auto coupled_identified=combine_mit_closed_loop_trials(triplet(coupled));
    check(coupled_identified.valid,"single-axis probing with the other coupled axis under fixed hold must identify its equivalent port");
    for(const auto& p:coupled_identified.response.points)check(std::abs(p.plant/effective_loop_plant(coupled,p.hz)-1.0)<1e-6,
        "the identified single-axis plant must include the actual held-other-axis interaction");
    auto changed_hold=coupled;changed_hold.held_other_kp=25;
    check(std::abs(effective_loop_plant(changed_hold,.7)/effective_loop_plant(coupled,.7)-1.0)>.01,
        "a coupled counterexample must expose why changing another axis's controller makes the earlier equivalent loop data stale");
    auto collinear=trials;collinear[2]=collinear[3]=trials[1];
    check(!combine_mit_closed_loop_trials(collinear).valid,
        "repeated actual protocol gains cannot manufacture a third independent controller experiment");
    auto failed_validation=trials;auto different_mechanics=port;different_mechanics.mechanical.inertia*=1.5;
    auto fourth_point=point;fourth_point.actual_gains.kp[0]=5;fourth_point.actual_gains.kd[0]=.6;
    failed_validation[3]=measure(different_mechanics,fourth_point);
    check(failed_validation[3].valid,"the changed-load fourth experiment must itself contain valid local frequency evidence");
    check(!combine_mit_closed_loop_trials(failed_validation).valid,
        "a fourth independent PD experiment that disagrees with the affine inverse response must reject a deceptively exact three-point interpolation");
    auto held_changed=trials;held_changed[2].operating_point.actual_gains.kp[1]=20;
    check(!combine_mit_closed_loop_trials(held_changed).valid,
        "changing a held-other-axis controller invalidates the common equivalent plant instead of appearing to identify the active axis");
    auto posture_changed=trials;posture_changed[2].operating_point.fixed_reference_rad[0]=.02;
    check(!combine_mit_closed_loop_trials(posture_changed).valid,
        "different operating-point references must not be merged into a single local MIT identification");
    auto condition_changed=trials;condition_changed[2].operating_point.condition_id="different_load_or_feedforward";
    check(!combine_mit_closed_loop_trials(condition_changed).valid,
        "a declared load/feedforward change must not be hidden in a gain-varying affine fit");
    auto clipped=internal_port_experiment(req,port,4,.3);clipped[2000].limited=true;
    check(!estimate_mit_closed_loop_trial(clipped,req,point).valid,
        "a limited independent-torque experiment cannot qualify the internal MIT response");
    auto unknown=internal_port_experiment(req,port,4,.3);unknown[2000].independent_disturbance_known=false;
    check(!estimate_mit_closed_loop_trial(unknown,req,point).valid,
        "missing independent commanded-input evidence cannot be filled by host PD reconstruction");
    auto actual_gain_changed=internal_port_experiment(req,port,4,.3);actual_gain_changed[2000].actual_kd=.5;
    check(!estimate_mit_closed_loop_trial(actual_gain_changed,req,point).valid,
        "a controller handover cannot be treated as one frozen internal MIT experiment");
    auto absent_report_velocity=internal_port_experiment(req,port,4,.3);
    for(auto& row:absent_report_velocity)row.velocity=std::numeric_limits<double>::quiet_NaN();
    check(estimate_mit_closed_loop_trial(absent_report_velocity,req,point).valid,
        "unknown reported velocity must not invalidate independent torque-to-position evidence or become a fabricated internal damping channel");
    auto short_req=req;short_req.probe_duration_s=5.5;
    const auto insufficient=estimate_mit_closed_loop_trial(internal_port_experiment(short_req,port,4,.3),short_req,point);
    check(!insufficient.valid && !insufficient.missing_evidence.empty(),
        "a quick window too short for the actual lowest tone must report missing frequency evidence instead of inventing valid poles or gains");
    auto duplicate=internal_port_experiment(req,port,4,.3);duplicate.insert(duplicate.begin()+1001,duplicate[1000]);
    const auto deduplicated=estimate_mit_closed_loop_trial(duplicate,req,point);
    check(deduplicated.valid && deduplicated.duplicate_feedback==1,
        "one repeated feedback frame must be counted once in a multigain MIT experiment");
    auto gapped=internal_port_experiment(req,port,4,.3);
    gapped.erase(std::remove_if(gapped.begin(),gapped.end(),[](const Sample& row){return row.time_s>4 && row.time_s<4.1;}),gapped.end());
    check(!estimate_mit_closed_loop_trial(gapped,req,point).valid,
        "missing receive-time coverage cannot be repaired by a high-quality tone fit");
    const ModelAxisLimits limits{.04,2,4,7,500,5,{500.0/4095,5.0/4095}};
    PdLoopShapingRequest target;target.target_crossover_hz=.7;target.error_bound_rad=.025;target.velocity_error_bound_rad=.15;
    const auto design=shape_mit_pd(4,.3,identified.response,limits,target);
    check(design.valid && design.candidate_loop.crossover_covered && design.candidate_loop.phase_margin_deg>=50,
        "the correctly identified internal-feedback port must support a bounded sampled-band PD design");
    std::cout<<"three-controller MIT port: noncollinear gains, actual commanded torque, unknown common transport, internal/report velocity distinction, internal position filtering, fixed FF and held-other conditions passed\n";
}

std::vector<Sample> integrated_internal_mit(const Requirements& req,double P,double D,
    bool inject,double initial_q=0) {
    constexpr double dt=.0005,input_start=.25,host_age=.006;
    constexpr unsigned command_steps=20,inner_steps=8,return_steps=12,age_steps=12,host_steps=4;
    const double end=req.probe_duration_s+input_start+req.tail_s;
    std::vector<double> command_queue(command_steps,0),inner_queue(inner_steps,0),position_history,velocity_history;
    std::vector<Sample> rows;double q=initial_q,v=0,reported_velocity=0,host_torque=0;
    for(unsigned k=0;k<=static_cast<unsigned>(end/dt);++k) {
        const double time=k*dt;
        if(k%host_steps==0)host_torque=inject?.06*probe(time-input_start,req).first/req.amplitude_rad:0;
        const double applied_input=command_queue[k%command_steps];command_queue[k%command_steps]=host_torque;
        const double requested=-P*q-D*v+applied_input;
        const double actual_torque=inner_queue[k%inner_steps];inner_queue[k%inner_steps]=requested;
        const double friction=.025*v+.0002*std::tanh(v/.005);
        v+=(actual_torque-.15*v-.5*q-friction)/.2*dt;q+=v*dt;
        reported_velocity+=dt/(.08+dt)*(v-reported_velocity);
        position_history.push_back(q);velocity_history.push_back(reported_velocity);
        if(k%host_steps)continue;
        const unsigned feedback_k=k>=age_steps?k-age_steps:0;
        const unsigned measured_k=feedback_k>=return_steps?feedback_k-return_steps:0;
        Sample row;row.time_s=time;row.command_time_s=time;row.feedback_time_s=std::max(0.,time-host_age);
        row.actual=std::round(position_history[measured_k]/.000002)*.000002;
        row.velocity=std::round(velocity_history[measured_k]/(20.0/4095))*(20.0/4095);
        row.new_measurement=k>=age_steps;row.feedback_sequence=feedback_k+1;
        row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;
        row.independent_disturbance_known=true;row.independent_disturbance_nm=host_torque;
        row.feedforward_known=true;row.applied_feedforward_nm=0;rows.push_back(row);
    }
    return rows;
}

void integrated_mit_design_effect() {
    Requirements req;req.probe_duration_s=24;req.amplitude_rad=.01;req.frequencies_hz={.35,.7,1.2};
    req.encoder_step_rad=.000002;req.noise_rms_rad=.000001;
    MitAutotuneOperatingPoint point;point.actual_gains={{4},{.3}};point.fixed_reference_rad={0};
    point.excited_axis=0;point.isolated_axis_excitation=true;point.condition_id="same_integrated_MIT_plant_posture_and_load";
    auto pp=point,dd=point,pd=point;pp.actual_gains.kp[0]=2;dd.actual_gains.kd[0]=1.2;pd.actual_gains.kp[0]=2;pd.actual_gains.kd[0]=1.2;
    const auto trial=[&](const MitAutotuneOperatingPoint& p) {
        return estimate_mit_closed_loop_trial(integrated_internal_mit(req,p.actual_gains.kp[0],p.actual_gains.kd[0],true),req,p);
    };
    const std::array<MitClosedLoopTrialEstimate,4> measured{trial(point),trial(pp),trial(dd),trial(pd)};
    for(const auto& m:measured)check(m.valid,
        "time-integrated sampled MIT experiments with friction, torque delay, stale returns and quantization must provide usable independent-input evidence");
    const auto identified=combine_mit_closed_loop_trials(measured);
    if(!identified.valid) {
        std::cerr<<"integrated MIT identification: "<<identified.reason<<'\n';
        for(std::size_t axis=0;axis<measured.size();++axis)for(const auto& p:measured[axis].points)
            std::cerr<<" trial="<<axis<<" hz="<<p.hz<<" H="<<p.closed_loop_response
                <<" half_a="<<p.first_half_response<<" half_b="<<p.second_half_response<<'\n';
    }
    check(identified.valid,"three-controller identification must predict its fourth real integrated-MIT response");
    const ModelAxisLimits limits{.04,2,4,7,500,5,{500.0/4095,5.0/4095}};
    PdLoopShapingRequest target;target.target_crossover_hz=.7;target.error_bound_rad=.025;target.velocity_error_bound_rad=.15;
    const auto design=shape_mit_pd(4,.3,identified.response,limits,target);
    check(design.valid && design.candidate_loop.crossover_covered,
        "a measured internal-loop FRF must generate a constrained protocol-projected PD candidate on the real time-integrated plant");
    const auto measure_return=[&](double P,double D) {
        const auto response=integrated_internal_mit(req,P,D,false,.015);double square=0,opposite=0,last_outside=0;std::size_t count=0;
        for(const auto& row:response)if(row.new_measurement && row.time_s<=4) {
            square+=row.actual*row.actual;opposite=std::max(opposite,-row.actual);++count;
            if(std::abs(row.actual)>.001)last_outside=row.time_s;
        }
        return std::array<double,3>{std::sqrt(square/count),opposite,last_outside};
    };
    const auto before=measure_return(4,.3),after=measure_return(design.kp,design.kd);
    std::cout<<"integrated MIT design: PD 4/.3 -> "<<design.kp<<'/'<<design.kd
        <<"; actual return RMS "<<before[0]<<" -> "<<after[0]<<"; opposite overshoot "<<before[1]<<" -> "<<after[1]
        <<"; settling to .001 rad "<<before[2]<<" -> "<<after[2]<<" s\n";
    check(after[0]<=before[0] && after[1]<.6*before[1] && after[2]<.85*before[2],
        "the measured phase-margin design must substantially reduce actual overshoot and settling time without worsening return RMS in an independent integrated-MIT validation");
    // These individually useful H measurements need not yield a repeatable
    // A/B/C estimate when actual gain changes are small relative to transient
    // and nonlinear uncertainty. Honest rejection requests another experiment.
    auto weak_p=point,weak_d=point,weak_pd=point;
    weak_p.actual_gains.kp[0]=5;weak_d.actual_gains.kd[0]=.6;weak_pd.actual_gains.kp[0]=5;weak_pd.actual_gains.kd[0]=.6;
    const std::array<MitClosedLoopTrialEstimate,4> weak{measured[0],trial(weak_p),trial(weak_d),trial(weak_pd)};
    const auto unresolved=combine_mit_closed_loop_trials(weak);
    check(!unresolved.valid && !unresolved.missing_evidence.empty(),
        "a low parameter-SNR multi-controller experiment must remain missing evidence rather than fabricate internal damping or a gain proposal");
}
}
int main() {
    try {
        controller_failure_reason_survives_incomplete_sequence();
        three_controller_internal_port_identification();
        integrated_mit_design_effect();
        Requirements req;req.probe_duration_s=16;req.amplitude_rad=.01;req.frequencies_hz={.35,.7,1.2};
        const Plant plant;const double kp=4,kd=.3;const MitExperiment mit{kp,kd,plant.inertia,plant.stiffness,ExcitationKind::ReferenceTracking};
        const auto rows=experiment(req,kp,kd,plant);
        check(!estimate_mit_frequency_response(rows,req,mit).valid,
            "simultaneous or undeclared multi-axis excitation must not be interpreted as a SISO joint-port plant");
        const auto response=isolated_response(rows,req,mit);
        check(response.valid && response.tracking_response_valid && !response.disturbance_response_valid && response.points.size()==3,
            "known actual MIT inputs and fresh feedback must produce reference evidence with explicit scope");
        for(const auto& point:response.points)check(std::abs(point.plant/physical_plant(plant,point.hz)-1.0)<1e-8,
            "reconstructed complete MIT effort must recover the actual nonparametric plant including transport delay");
        const ModelAxisLimits limits{.04,2,4,7,500,5,{500.0/4095,5.0/4095}};
        PdLoopShapingRequest target;target.target_crossover_hz=.7;target.error_bound_rad=.025;target.velocity_error_bound_rad=.15;
        const auto proposal=shape_mit_pd(kp,kd,response,limits,target);
        check(proposal.valid && proposal.changed && proposal.candidate_loop.crossover_covered &&
              proposal.candidate_loop.phase_margin_deg>=50 && proposal.candidate_loop.phase_margin_deg<=85 &&
              std::abs(std::log(proposal.candidate_loop.crossover_hz/.7))<.2,
            "analytic PD loop shaping must deliver a protocol-quantized, re-evaluated, bracketed crossover and requested design phase margin");
        check(proposal.kp*target.error_bound_rad+proposal.kd*target.velocity_error_bound_rad<=limits.effort_budget+1e-9,
            "sampled loop design must retain actual joint feedback effort constraints");
        std::vector<EmpiricalMitFrequencyResponse> axes;std::vector<SampledPdGainProposal> proposals;
        for(double inertia:{.05,.2,.7,.1,.35}) {
            auto varied=plant;varied.inertia=inertia;varied.damping=.75*inertia;varied.stiffness=2.5*inertia;
            auto m=mit;m.inertia_prior=inertia;
            const auto r=isolated_response(experiment(req,kp,kd,varied),req,m);
            check(r.valid,"different mapped inertias must retain measured nonparametric evidence");axes.push_back(r);
            proposals.push_back(shape_mit_pd(kp,kd,r,limits,target));check(proposals.back().valid,"each mapped inertia must have its own bounded measured-frequency PD design");
        }
        check(proposals[2].kp>proposals[0].kp && proposals[2].kd>proposals[0].kd,
            "different actual inertias must change the computed PD pair rather than selecting one fixed gain template");
        for(unsigned axis:{4U,2U,0U,3U,1U}) {
            const auto p=shape_mit_pd(kp,kd,axes[axis],limits,target);
            check(p.kp==proposals[axis].kp && p.kd==proposals[axis].kd,"axis permutation must not alter any measured plant's tuning result");
        }
        auto stronger=mit;stronger.kd=1.2;
        const auto zero=isolated_response(experiment(req,kp,stronger.kd,plant),req,stronger);
        check(zero.valid,"MIT reference velocity must remain an explicit actual input term");
        check(std::abs(zero.points[2].reference_response-response.points[2].reference_response)>.05 &&
              std::abs(zero.points[2].plant/response.points[2].plant-1.0)<1e-8,
            "the MIT reference numerator zero changes closed-loop tracking but must not be mistaken for a changed physical plant or zero-free bandwidth");
        const auto fed=isolated_response(experiment(req,kp,kd,plant,false,true),req,mit);
        check(!fed.valid && fed.reason=="reference_feedforward_exceeds_declared_position_only_scope",
            "perfect inverse-dynamics tracking cannot be interpreted as a position-only executed feedforward strategy or stand in for independent disturbance evidence");
        auto gravity=plant;gravity.stiffness=1.5;auto gravity_mit=mit;gravity_mit.executed_state_feedforward_gradient=gravity.stiffness;
        const auto state_fed=isolated_response(experiment(req,kp,kd,gravity,false,false,0,gravity.stiffness),req,gravity_mit);
        check(state_fed.valid,"known actually executed state-dependent feedforward must retain measurable port response");
        for(const auto& point:state_fed.points) {
            const auto physical=physical_plant(gravity,point.hz),effective=physical/(1.0-gravity.stiffness*physical);
            check(std::abs(point.physical_plant/physical-1.0)<1e-8 && std::abs(point.plant/effective-1.0)<1e-8,
                "physical torque plant and the state-feedforward-compensated feedback-loop plant must be distinguished before computing phase margin");
        }
        auto dm=mit;dm.kind=ExcitationKind::IndependentTorqueDisturbance;
        const auto independent=isolated_response(experiment(req,kp,kd,plant,true),req,dm);
        check(independent.valid && independent.disturbance_response_valid && !independent.tracking_response_valid &&
              std::abs(independent.points[1].plant/response.points[1].plant-1.0)<1e-8,
            "known independent disturbance and reference tracking must preserve their different evidence while agreeing on the measured plant");
        auto moving_velocity_reference=experiment(req,kp,kd,plant,true);
        for(auto& row:moving_velocity_reference)row.reference_velocity=.01*std::sin(2*pi*req.frequencies_hz[0]*row.time_s);
        check(!estimate_mit_frequency_response(moving_velocity_reference,req,dm,true).valid,
            "MIT permits independent reference velocity, but a fixed-position disturbance claim must reject its additional external drive");
        auto drifting_reference=experiment(req,kp,kd,plant,true);
        for(auto& row:drifting_reference)row.reference=.0001*row.time_s;
        check(!estimate_mit_frequency_response(drifting_reference,req,dm,true).valid,
            "off-tone reference drift cannot count as a fixed-reference independent disturbance experiment");
        const double filter=.08,velocity_ff=.12;
        const auto dynamic_rows=experiment(req,kp,kd,gravity,true,false,0,gravity.stiffness,velocity_ff,filter);
        const auto dynamic=estimate_mit_frequency_response(dynamic_rows,req,dm,true);
        check(dynamic.valid && dynamic.disturbance_response_valid,
            "fixed-reference independent torque must identify the controlled port with the actual dynamic feedforward strategy inside it");
        for(const auto& point:dynamic.points) {
            const double w=2*pi*point.hz;const auto Hv=std::complex<double>{0,w}/std::complex<double>{1,w*filter};
            const auto F=gravity.stiffness+velocity_ff*Hv,P=physical_plant(gravity,point.hz);
            check(std::abs(point.feedback_velocity_per_position/Hv-1.0)<1e-8 && std::abs(point.executed_feedforward_per_position/F-1.0)<1e-8 &&
                  std::abs(point.physical_plant/P-1.0)<1e-8 && std::abs(point.plant/(P/(1.0-F*P))-1.0)<1e-8,
                "actual velocity filtering and executed state/velocity feedforward must enter measured loop equations without assuming ideal jw or accurate compensation");
        }
        const auto dynamic_loop=evaluate_mit_pd_loop(kp,kd,dynamic);
        check(dynamic_loop.valid && std::abs(dynamic_loop.points[2].magnitude-
            std::abs((kp+kd*dynamic.points[2].feedback_velocity_per_position)*dynamic.points[2].plant))<1e-10,
            "sampled loop evaluation must use measured velocity feedback transfer, not an assumed derivative");
        const double ideal_phase=std::arg(std::complex<double>{kp,2*pi*dynamic.points[2].hz*kd});
        const double actual_phase=std::arg(kp+kd*dynamic.points[2].feedback_velocity_per_position);
        check(std::abs(ideal_phase-actual_phase)*180/pi>5,
            "the velocity-filter counterexample must expose a material phase error in an ideal-jw controller assumption");
        const auto dynamic_design=shape_mit_pd(kp,kd,dynamic,limits,target);
        check(dynamic_design.valid && dynamic_design.candidate_loop.crossover_covered && dynamic_design.candidate_loop.phase_margin_deg>=target.min_phase_margin_deg,
            "PD synthesis with measured filtered velocity must retain the requested sampled phase margin after actual protocol projection");
        check(!estimate_mit_frequency_response(rows,req,mit,true).valid,
            "a reference experiment must explicitly declare its interpretable executed feedforward scope");
        auto dynamic_reference=experiment(req,kp,kd,gravity,false,false,0,gravity.stiffness,velocity_ff,filter);
        check(!estimate_mit_frequency_response(dynamic_reference,req,gravity_mit,true).valid,
            "an undeclared dynamic state-feedback strategy cannot be silently treated as gravity-only during reference identification");
        auto duplicate=rows;duplicate.insert(duplicate.begin()+1001,duplicate[1000]);
        const auto dedup=isolated_response(duplicate,req,mit);
        check(dedup.valid && dedup.duplicate_feedback==1 && dedup.fresh_samples==response.fresh_samples,
            "duplicate received feedback must not create additional identification samples");
        const auto invalid=[&](std::vector<Sample> altered,const char* why){check(!isolated_response(altered,req,mit).valid,why);};
        auto altered=rows;altered[1000].limited=true;invalid(altered,"limited commands must not count as successful frequency evidence");
        altered=rows;altered[1000].actual_gains_known=false;invalid(altered,"unknown executed gains cannot be replaced with intended defaults");
        altered=rows;altered[1000].actual_kp*=1.1;invalid(altered,"a changing controller cannot be fitted as a fixed closed-loop experiment");
        altered=rows;altered[1000].feedforward_known=false;invalid(altered,"unknown execution feedforward must reject physical input estimation");
        altered=rows;altered[1000].command_time_s+=.1;invalid(altered,"a command timestamp in the future cannot produce valid delay/phase evidence");
        altered=rows;altered[1000].feedback_time_s=altered[999].feedback_time_s;invalid(altered,"a repeated receive timestamp with a new sequence is invalid timing evidence");
        altered=rows;for(auto& row:altered)if(row.time_s>8){row.actual*=2;row.velocity*=2;}
        invalid(altered,"inconsistent half-window plants must request further measurement rather than silently average incompatible conditions");
        invalid(experiment(req,kp,kd,plant,false,false,.05),"position noise above the useful excitation must remain missing evidence");
        altered=rows;for(auto& row:altered)row.applied_feedforward_nm=-row.actual_kp*(row.reference-row.actual)-row.actual_kd*(row.reference_velocity-row.velocity);
        invalid(altered,"a known total command that cancels into noise cannot identify a physical plant from tracking alone");
        auto short_req=req;short_req.probe_duration_s=5;
        check(!isolated_response(rows,short_req,mit).valid,"short low-frequency windows cannot fabricate half-window consistency");
        auto motion=req;motion.motion_probe=true;
        check(!isolated_response(rows,motion,mit).valid,"single-tone movement establishment is not a three-frequency response experiment");
        auto outside=target;outside.target_crossover_hz=2;
        check(!shape_mit_pd(kp,kd,response,limits,outside).valid,"an unmeasured intended crossover must request higher-frequency evidence");
        auto constrained=limits;constrained.kp_max=.1;constrained.kd_max=.01;constrained.grid={};
        check(!shape_mit_pd(kp,kd,response,constrained,target).valid,
            "protocol/effort projection must be followed by a new crossing and phase check, never retain the unconstrained design margin");
        EmpiricalMitFrequencyResponse multiple;multiple.valid=true;multiple.isolated_axis_excitation=true;
        for(unsigned k=0;k<4;++k){EmpiricalMitFrequencyPoint p;p.hz=.2*std::pow(2,k);p.plant=std::polar(k%2?.1:10.0,-130*pi/180);multiple.points.push_back(p);}
        const auto ambiguous=evaluate_mit_pd_loop(1,0,multiple);
        check(ambiguous.valid && !ambiguous.crossover_covered && ambiguous.crossovers.size()==3,
            "all visible unity crossings must be examined and a multi-crossing response cannot be reported as one verified design margin");
        std::cout<<"empirical MIT FRF and sampled PD design passed: actual PD/FF/D, reference zero, separate disturbance, five inertias/axis permutation, delay, noise, dedup, clocks, input cancellation, half-window quality, protocol projection and every sampled crossover\n";
        std::cout<<"candidate Kp="<<proposal.kp<<", Kd="<<proposal.kd<<", measured-band crossover="<<proposal.candidate_loop.crossover_hz
                 <<" Hz, design phase margin="<<proposal.candidate_loop.phase_margin_deg<<" deg; hardware comparison remains required\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
