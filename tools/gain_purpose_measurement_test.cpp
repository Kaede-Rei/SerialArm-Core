#include "serial_arm/dynamics/gain_purpose_measurement.hpp"
#include <iostream>
#include <numeric>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
ModelAxisLimits limits(double room=.08) {
    ModelAxisLimits x;x.max_velocity=1;x.max_acceleration=10;x.effort_budget=1;
    x.reference_min_offset_rad=-room;x.reference_max_offset_rad=room;return x;
}
std::vector<Sample> simulate(const PurposePulsePlan& plan,const NoiseEstimate& noise,double M,double P,double D,
    double delay=0,double friction=0,bool locked=false,double bias=0) {
    constexpr double dt=.001;double q=0,v=0;std::vector<Sample> rows;std::vector<double> positions,velocities;
    for(unsigned k=0;k<=std::ceil(plan.duration_s/dt);++k) {
        const double t=k*dt,u=purpose_pulse_input(t-delay,plan);double force=u-P*q-D*v+bias;
        if(locked){q=0;v=0;}
        else if(std::abs(v)<1e-5 && std::abs(force)<=friction)v=0;
        else {force-=friction*(std::abs(v)>1e-5?(v>0?1:-1):(force>0?1:-1));v+=force/M*dt;q+=v*dt;}
        positions.push_back(q);velocities.push_back(v);
        if(k%5)continue;
        const std::size_t sensor=k>=3?k-3:0;const double measured=positions[sensor];
        Sample r;r.time_s=t;r.actual=noise.encoder_step_rad>0?std::round(measured/noise.encoder_step_rad)*noise.encoder_step_rad:measured;
        r.velocity=velocities[sensor];r.actual_kp=P;r.actual_kd=D;r.actual_gains_known=true;
        r.new_measurement=true;r.feedback_sequence=k+1;r.feedback_time_s=t-.003;r.command_time_s=t;
        r.independent_disturbance_known=true;r.independent_disturbance_nm=purpose_pulse_input(t,plan);
        rows.push_back(r);
    }
    return rows;
}
void opposite_direction_after_measured_deadzone() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;
    for(std::size_t n:{1,3,7})for(std::size_t axis=0;axis<n;++axis)for(int sign:{-1,1}) {
        auto bounds=limits(.04);bounds.effort_budget=.5;
        PurposePulseOptions options;options.available_s=20;options.torque_quantum_nm=.001;
        options.direction=sign;options.previous_input_peak_nm=.3;
        const auto plan=plan_purpose_pulse(bounds,noise,.05,10,.8,options);
        const auto measured=assess_purpose_pulse(simulate(plan,noise,.05,10,.8,0,.2,false,-sign*.15),noise,plan);
        check(plan.valid && measured.input_measurement_complete && !measured.yield_observed,
            "a geometrically valid pulse can oppose existing equilibrium effort and remain hidden inside a static dead zone");
        const auto reverse=next_purpose_pulse(plan,measured);
        check(reverse.valid && reverse.direction==-sign && reverse.direction_reversed &&
            reverse.supplement_action=="measure_opposite_legal_direction" && reverse.duration_s==plan.duration_s &&
            reverse.amplitude_nm<=reverse.input_ceiling_nm && reverse.amplitude_nm<=plan.opposite_input_ceiling_nm,
            "complete unresolved input at one directional ceiling must permit exactly one separately bounded reverse measurement");
        const auto response=assess_purpose_pulse(simulate(reverse,noise,.05,10,.8,0,.2,false,-sign*.15),noise,reverse);
        check(response.input_measurement_complete && response.yield_observed,
            "same-law reverse experiment must reveal the directional static response without robot- or joint-specific tuning");
        auto locked=assess_purpose_pulse(simulate(reverse,noise,.05,10,.8,0,0,true),noise,reverse);
        check(!next_purpose_pulse(reverse,locked).valid && locked.compliant.unresolved_both_input_directions_at_ceiling &&
            locked.compliant.unresolved_input_direction_mask==3 && !locked.compliant.measurement_complete,
            "two unresolved directional ceilings must remain insufficient response evidence instead of alternating forever");
        const auto proposal=compliant_candidate_report({{10},{.8}},{100},{5},{locked.compliant});
        check(proposal.changed && proposal.gains.kp.front()<10 && proposal.gains.kd.front()==.8 &&
            proposal.reasons.front()=="both_legal_input_directions_measured_unresolved_small_stiffness_reduction_comparison",
            "both measured directional ceilings may generate a bounded softer trial without inventing yielding or a damping direction");
        const auto new_group=assess_purpose_pulse(simulate(reverse,noise,.05,9.2,.8,0,0,true),noise,reverse);
        check(!new_group.compliant.unresolved_both_input_directions_at_ceiling &&
            !compliant_use_preferred({new_group.compliant},{locked.compliant},{{9.2},{.8}},{{10},{.8}},true,true),
            "a different PD group cannot inherit old opposite-direction evidence or retain unresolved yielding as an improvement");
        auto one_sided=bounds;if(sign>0)one_sided.reference_min_offset_rad=0;else one_sided.reference_max_offset_rad=0;
        const auto edge=plan_purpose_pulse(one_sided,noise,.05,10,.8,options);
        const auto blocked=assess_purpose_pulse(simulate(edge,noise,.05,10,.8,0,0,true),noise,edge);
        check(!next_purpose_pulse(edge,blocked).valid,
            "a reverse supplement cannot leave the demonstrated domain or cross a zero mechanical boundary");
        options.direction=2;
        check(!plan_purpose_pulse(bounds,noise,.05,10,.8,options).valid,
            "unknown directional requests cannot silently select a different experiment");
        (void)axis;
    }
}

void planning_and_waveform() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;auto bound=limits();
    PurposePulseOptions o;o.available_s=20;o.torque_quantum_nm=.001;
    const auto p=plan_purpose_pulse(bound,noise,.05,10,.8,o);
    check(p.valid && p.amplitude_nm>0 && p.duration_s<o.available_s,"valid model/noise plan must fit its declared time");
    check(p.target_displacement_rad>=3*p.motion_floor_rad && p.amplitude_nm<=p.input_ceiling_nm,"pulse must resolve codes within existing effort allowance");
    check(purpose_pulse_input(-1,p)==0 && purpose_pulse_input(p.duration_s,p)==0 && purpose_pulse_input(p.release_s,p)==0,"baseline and released torque must be zero");
    constexpr double eps=1e-6;
    check(std::abs(purpose_pulse_input(p.baseline_s+eps,p)/eps)<1e-4 && std::abs(purpose_pulse_input(p.release_s-eps,p)/eps)<1e-4,"pulse edges have zero torque derivative");
    for(double t=0;t<p.duration_s;t+=.001)check(std::abs(purpose_pulse_input(t,p))<=p.amplitude_nm+1e-12,"entire smooth pulse is bounded");
    o.available_s=p.duration_s-.01;check(!plan_purpose_pulse(bound,noise,.05,10,.8,o).valid,"partial windows cannot be started by pretending to fit the budget");
    o.available_s=20;check(!plan_purpose_pulse(bound,noise,0,10,.8,o).valid,"missing model inertia cannot become a successful default model");
    check(!plan_purpose_pulse(limits(.0002),noise,.05,10,.8,o).valid,"unresolvable measured room cannot be expanded by planning");
    auto negative=bound;negative.reference_max_offset_rad=.0001;
    const auto n=plan_purpose_pulse(negative,noise,.05,10,.8,o);
    check(n.valid && n.direction==-1 && purpose_pulse_input(n.baseline_s+n.rise_s,n)<0,"one-sided measured room selects a smooth torque toward its interior");
    o.previous_input_peak_nm=2*p.amplitude_nm;
    const auto softer=plan_purpose_pulse(bound,noise,.05,5,.4,o);
    check(softer.valid && softer.amplitude_nm>=o.previous_input_peak_nm,"softer gains must not erase the known useful independent input");
    o.previous_input_peak_nm=.7;
    const auto measured_force=plan_purpose_pulse(bound,noise,.001,5,.4,o);
    check(measured_force.valid && measured_force.amplitude_nm>.001*bound.max_acceleration &&
        measured_force.amplitude_nm<=measured_force.input_ceiling_nm && measured_force.amplitude_nm<=.5*5*(.08-p.motion_floor_rad),
        "measured force may exceed M*a but must remain inside the current pose/restoring-displacement allowance");
    PurposePulseOptions coarse;coarse.available_s=20;coarse.torque_quantum_nm=.1;
    const auto low_code=plan_purpose_pulse(limits(.05),noise,.01,.1,.01,coarse);
    check(!low_code.valid && low_code.amplitude_nm==0 && low_code.reason=="purpose_pulse_no_representable_input" &&
        purpose_pulse_input(1,low_code)==0,
        "a coarse first force code cannot command a one-radian predicted displacement in a 0.05-radian measured domain");
    coarse.previous_input_peak_nm=.3;
    const auto historic_code=plan_purpose_pulse(limits(.05),noise,.01,.1,.01,coarse);
    check(!historic_code.valid && historic_code.amplitude_nm==0,
        "historical measured force cannot bypass the current soft-P pose domain or trigger a fixed force fallback");
    coarse.torque_quantum_nm=.0001;coarse.previous_input_peak_nm=0;
    const auto fine_code=plan_purpose_pulse(limits(.05),noise,.01,.1,.01,coarse);
    check(fine_code.valid && fine_code.amplitude_nm>0 && fine_code.amplitude_nm/.1<=.5*(.05-fine_code.motion_floor_rad),
        "low restoring gains remain measurable when the actual force code fits the demonstrated recovery room");
}
void plateau_measurement() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;PurposePulseOptions o;o.available_s=20;
    const auto p=plan_purpose_pulse(limits(),noise,.05,10,.8,o);
    const auto rows=simulate(p,noise,.05,10,.8,.035);
    const auto a=assess_purpose_pulse(rows,noise,p);
    check(a.input_measurement_complete && a.direct_response_complete && a.yield_observed && a.compliant.passed,
        "delayed quantized closed-loop pulse must measure signed plateau yielding and unloading without zero-lag multisine projection");
    check(!a.model_identified && std::isfinite(a.effective_compliance_rad_nm) && a.effective_compliance_rad_nm>0,
        "finite plateau compliance must not manufacture a pole or bandwidth identification");
    auto written_late=rows;
    for(auto& r:written_late) {
        r.reference_sample_time_known=true;r.reference_sample_time_s=r.command_time_s;
        r.command_time_s+=.035;r.time_s+=.035;r.feedback_time_s+=.035;
    }
    const auto late=assess_purpose_pulse(written_late,noise,p);
    check(late.input_measurement_complete && late.compliant.passed && std::abs(late.actual_release_s-a.actual_release_s-.035)<.006,
        "known control sample time must preserve actual waveform comparison and unload timing despite a later write clock");
    const auto friction=assess_purpose_pulse(simulate(p,noise,.05,10,.8,.015,.35*p.amplitude_nm),noise,p);
    check(friction.input_measurement_complete && friction.yield_observed && friction.hold_signed_displacement_rad<a.hold_signed_displacement_rad,
        "static friction reduces measured yielding instead of being replaced by the URDF prediction");
    const auto hard=assess_purpose_pulse(simulate(p,noise,.05,10,.8,0,0,true),noise,p);
    check(hard.input_measurement_complete && hard.direct_response_complete && !hard.yield_observed && !hard.compliant.measurement_complete &&
        !hard.compliant.passed && !std::isfinite(hard.effective_compliance_rad_nm),
        "hard sub-resolution response retains complete known input and displacement upper bound without fake yielding or infinite stiffness");
    auto bad=rows;for(auto& r:bad)r.limited=true;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"limited input cannot qualify a purpose");
    bad=rows;for(auto& r:bad)r.independent_disturbance_known=false;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"unknown independent input must remain missing evidence");
    bad=rows;for(auto& r:bad)r.feedback_sequence=1;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"repeated feedback must not manufacture a complete plateau");
    bad=rows;auto hidden=bad[bad.size()/3];hidden.new_measurement=false;hidden.independent_disturbance_known=false;
    bad.insert(bad.begin()+bad.size()/3,hidden);
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,
        "a repeated feedback frame must not hide an unknown intervening actual command");
    bad=rows;bad.erase(bad.begin()+bad.size()/3,bad.begin()+bad.size()/2);
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"lost feedback interval must remain an incomplete measurement");
    bad=rows;for(auto& r:bad)r.independent_disturbance_nm=0;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"zero or rejected actual pulse cannot be replaced with planned force");
    bad=rows;for(auto& r:bad)r.command_time_s=std::numeric_limits<double>::quiet_NaN();
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"unknown actual command timing cannot be silently filled from host sampling time");
    bad=rows;for(auto& r:bad)r.feedback_time_s=r.command_time_s-.08;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"stale feedback cannot become fresh measurement by setting the new-measurement flag");
    bad=written_late;for(auto& r:bad)r.reference_sample_time_s=r.command_time_s+.02;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"future control sample times cannot supply known actual pulse timing");
    bad=rows;for(auto& r:bad)r.feedback_time_s=r.command_time_s+.02;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"future feedback timing cannot be accepted as current response evidence");
    bad=rows;for(auto& r:bad)if(r.time_s>=p.release_s)r.independent_disturbance_nm=.001;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"small residual input after unloading cannot qualify a release");
    bad=rows;bad.back().actual_kp+=1;
    check(!assess_purpose_pulse(bad,noise,p).input_measurement_complete,"mixed actual PD codes cannot qualify one full group");
    bad=rows;for(auto& r:bad)r.actual=-r.actual;
    const auto opposite=assess_purpose_pulse(bad,noise,p);
    check(opposite.input_measurement_complete && !opposite.yield_observed && opposite.opposite_response &&
        opposite.reason=="purpose_pulse_resolved_response_opposes_known_input",
        "a resolved opposite response must have a distinct reason from sub-resolution movement");
    check(hard.reason=="purpose_pulse_response_below_measured_resolution" && !hard.opposite_response,
        "an unchanged encoder code must never be described as measured reverse motion");
    bad=rows;for(auto& r:bad)r.actual+=.02;
    const auto offset=assess_purpose_pulse(bad,noise,p);
    check(offset.compliant.passed && offset.compliant.final_reference_error_rad>.015 && offset.compliant.final_equilibrium_error_rad<.002,
        "unloaded equilibrium recovery and nominal tracking error must remain separate");
    auto neg_bounds=limits();neg_bounds.reference_max_offset_rad=0;
    const auto neg_plan=plan_purpose_pulse(neg_bounds,noise,.05,10,.8,o);
    const auto neg=assess_purpose_pulse(simulate(neg_plan,noise,.05,10,.8),noise,neg_plan);
    check(neg_plan.direction==-1 && neg.compliant.passed && neg.hold_mean_input_nm<0 && neg.hold_signed_displacement_rad>0,
        "negative measured room must retain the same signed plateau interpretation under joint-axis direction reversal");
}
void mapped_isolation_and_drag() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;
    for(std::size_t n:{1,3,7}) {
        std::vector<std::size_t> order(n);std::iota(order.begin(),order.end(),0);
        std::vector<PurposePulsePlan> plans;std::vector<double> values;
        for(std::size_t i=0;i<n;++i){PurposePulseOptions o;o.available_s=20;plans.push_back(plan_purpose_pulse(limits(),noise,.03+.02*i,8+2*i,.6+.1*i,o));}
        for(unsigned pass=0;pass<2;++pass) {
            for(auto axis:order) {
                const auto& p=plans[axis];check(p.valid,"arbitrary mapped joint count must plan without joint-name cases");
                values.assign(n,0);values[axis]=purpose_pulse_input(p.baseline_s+p.rise_s+.5*p.hold_s,p);
                check(std::count_if(values.begin(),values.end(),[](double x){return x!=0;})==1,"single-axis schedule cannot command collinear multi-joint disturbances");
                const auto a=assess_purpose_pulse(simulate(p,noise,.03+.02*axis,8+2*axis,.6+.1*axis),noise,p);
                check(a.input_measurement_complete && a.yield_observed,"permuting mapped axes must preserve their own actual model and pulse evidence");
            }
            std::reverse(order.begin(),order.end());
        }
    }
    PurposePulseOptions o;o.available_s=20;o.zero_stiffness=true;
    const auto p=plan_purpose_pulse(limits(),noise,.02,0,.1,o);
    check(p.valid,"zero-stiffness purpose must have its own inertia/damping and room bounded pulse");
    const auto rs=simulate(p,noise,.02,0,.1);
    double maxq=0;for(const auto& r:rs)maxq=std::max(maxq,std::abs(r.actual));
    check(maxq<.04 && std::abs(rs.back().actual)>.0003,"zero-stiffness release must stay in measured room while allowing a new stopped position");
    const auto release=assess_drag_release(rs,noise,p.release_s,{},true);
    check(release.motion_observed && release.measurement_complete,"actual P0 pulse must create measurable release after the input is removed");
    check(!assess_purpose_pulse(rs,noise,p).input_measurement_complete,"restoring-purpose assessment must not qualify a zero-stiffness controller");
}
void comparison_initial_pose() {
    for(std::size_t n:{1,3,7}) {
        std::vector<double> baseline(n),nearby(n),released(n);std::vector<NoiseEstimate> noise(n);
        for(std::size_t i=0;i<n;++i) {
            baseline[i]=.2*static_cast<double>(i)-.4;noise[i].encoder_step_rad=.0001+.00001*i;
            noise[i].rms_rad=.00002*(i%3);
            const double floor=std::max({3*noise[i].encoder_step_rad,6*noise[i].rms_rad,.0002});
            nearby[i]=baseline[i]+(i%2?-.7:.7)*floor;
        }
        check(purpose_comparison_pose(baseline,nearby,noise),
            "same initial pose within measured encoder/noise tolerance must permit an optional damping comparison");
        PurposePulseOptions options;options.zero_stiffness=true;options.available_s=20;
        const auto plan=plan_purpose_pulse(limits(),noise.back(),.02,0,.1,options);
        check(plan.valid,"mapped drag release must have an executable model/noise plan");
        const auto rows=simulate(plan,noise.back(),.02,0,.1);
        check(assess_drag_release(rows,noise.back(),plan.release_s).passed,
            "a valid zero-stiffness release may stop at a new pose and still pass its own purpose");
        released=nearby;released.back()=baseline.back()+rows.back().actual;
        check(!purpose_comparison_pose(baseline,released,noise),
            "a successful P0 release that changes the initial pose cannot be used as a same-condition damping comparison");
        auto reverse_baseline=baseline,reverse_nearby=nearby,reverse_released=released;auto reverse_noise=noise;
        std::reverse(reverse_baseline.begin(),reverse_baseline.end());std::reverse(reverse_nearby.begin(),reverse_nearby.end());
        std::reverse(reverse_released.begin(),reverse_released.end());std::reverse(reverse_noise.begin(),reverse_noise.end());
        check(purpose_comparison_pose(reverse_baseline,reverse_nearby,reverse_noise) &&
            !purpose_comparison_pose(reverse_baseline,reverse_released,reverse_noise),
            "permuting every mapped axis must preserve comparable and shifted-pose decisions");
        auto missing=nearby;missing.pop_back();auto missing_noise=noise;missing_noise.pop_back();
        check(!purpose_comparison_pose(baseline,missing,noise) && !purpose_comparison_pose(baseline,nearby,missing_noise),
            "missing position or noise axes must not permit comparison of incomplete gain groups");
        auto invalid=nearby;invalid.back()=std::numeric_limits<double>::quiet_NaN();
        check(!purpose_comparison_pose(baseline,invalid,noise),"non-finite actual pose must remain unavailable for damping comparison");
        invalid=baseline;invalid.front()=std::numeric_limits<double>::infinity();
        check(!purpose_comparison_pose(invalid,nearby,noise),"non-finite baseline pose must remain unavailable for damping comparison");
    }
    check(!purpose_comparison_pose({}, {}, {}),"empty groups cannot produce comparable pose evidence");
}

std::vector<Sample> low_speed_pd_history(double force,double P=1.2,double D=.035) {
    std::vector<Sample> out;
    for(unsigned k=0;k<240;++k) {
        Sample r;r.time_s=r.command_time_s=.005*k;r.feedback_time_s=r.time_s-.004;
        r.new_measurement=true;r.feedback_sequence=k+1;r.actual_gains_known=true;r.actual_kp=P;r.actual_kd=D;
        r.reference=.2;r.actual=r.reference-force/P;r.reference_velocity=r.velocity=0;
        r.independent_disturbance_known=true;r.independent_disturbance_nm=0;out.push_back(r);
    }
    return out;
}
void measured_prior_and_stiction_retry() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    for(std::size_t n:{1,3,7}) {
        std::vector<PurposeInputPrior> priors;
        for(std::size_t axis=0;axis<n;++axis) {
            const double force=.14+.02*axis;
            const auto prior=estimate_purpose_input_prior(low_speed_pd_history(force),noise,"measured-taught-path.csv");
            check(prior.valid && prior.low_speed_pd_samples==240 && prior.independent_samples==0 &&
                std::abs(prior.seed_nm-force)<1e-12 && prior.source=="measured-taught-path.csv" &&
                prior.interpretation.find("not_friction_identification")!=std::string::npos,
                "actual low-speed Pe+Dvel evidence must set a traceable input-scale prior, not a friction identification");
            priors.push_back(prior);
        }
        for(unsigned order=0;order<2;++order) {
            for(const auto& prior:priors) {
                auto bounds=limits(.8);bounds.max_acceleration=3;PurposePulseOptions options;
                options.available_s=20;options.torque_quantum_nm=20.0/4095;options.input_prior=prior;
                const auto planned=plan_purpose_pulse(bounds,noise,.001,1.2,.035,options);
                check(planned.valid && planned.amplitude_nm>=prior.seed_nm && planned.input_prior.valid &&
                    planned.amplitude_nm<=planned.input_ceiling_nm && planned.amplitude_nm<=bounds.effort_budget,
                    "the measured prior must affect a mapped pulse while obeying current restoring-pose and effort bounds");
            }
            std::reverse(priors.begin(),priors.end());
        }
    }
    auto bounds=limits(.4);bounds.max_acceleration=3;PurposePulseOptions options;
    options.available_s=20;options.torque_quantum_nm=20.0/4095;
    const auto old=plan_purpose_pulse(bounds,noise,.001,1.2,.035,options);
    check(old.valid && old.amplitude_nm<.01,"the low-inertia counterfactual must reproduce a millinewton-metre model seed");
    const auto locked=assess_purpose_pulse(simulate(old,noise,.001,1.2,.035,0,.14),noise,old);
    check(locked.input_measurement_complete && !locked.yield_observed,
        "known .005-level force cannot fabricate motion through a .14-level static dead zone");
    auto current=old;auto measured=locked;unsigned trials=1;
    while(!measured.yield_observed && trials<12) {
        const auto next=next_purpose_pulse(current,measured);
        if(!next.valid)break;
        check(next.amplitude_nm>current.amplitude_nm && next.amplitude_nm<=current.input_ceiling_nm &&
            next.duration_s==current.duration_s,"each complete retry must increase actual input without expanding time or hard domains");
        current=next;measured=assess_purpose_pulse(simulate(current,noise,.001,1.2,.035,0,.14),noise,current);++trials;
    }
    check(measured.yield_observed && trials>2 && current.amplitude_nm<=old.input_ceiling_nm,
        "budget-controlled repeated measured input must establish a quantized static-friction response beyond one fixed retry");
    const auto resolved_next=next_purpose_pulse(current,measured);
    check(!resolved_next.valid || (resolved_next.amplitude_nm==current.amplitude_nm &&
        resolved_next.supplement_action=="extend_same_input_observation"),
        "resolved response must stop increasing input; an unsettled plateau may only extend the same bounded force");
    options.input_prior=estimate_purpose_input_prior(low_speed_pd_history(.14),noise,"prior.csv");
    const auto seeded=plan_purpose_pulse(bounds,noise,.001,1.2,.035,options);
    check(seeded.amplitude_nm>10*old.amplitude_nm,"measured trajectory input evidence must change the actual pulse, not only its metadata");
    const auto seeded_measurement=assess_purpose_pulse(simulate(seeded,noise,.001,1.2,.035,0,.14),noise,seeded);
    auto rejected=seeded_measurement;rejected.input_measurement_complete=false;
    check(!next_purpose_pulse(seeded,rejected).valid,"unknown or incomplete input cannot justify escalation");
    rejected=seeded_measurement;rejected.opposite_response=true;
    check(!next_purpose_pulse(seeded,rejected).valid,"measured reverse response must not trigger stronger same-direction input");
    auto narrow=bounds;narrow.reference_min_offset_rad=-.04;narrow.reference_max_offset_rad=.04;
    const auto capped=plan_purpose_pulse(narrow,noise,.001,1.2,.035,options);
    check(capped.valid && capped.amplitude_nm<.14 && capped.amplitude_nm<=capped.input_ceiling_nm,
        "a .14 prior must not bypass the current narrow restoring-pose range");
    const auto no_motion=assess_purpose_pulse(simulate(capped,noise,.001,1.2,.035,0,.14),noise,capped);
    const auto stopped=next_purpose_pulse(capped,no_motion);
    check(stopped.valid && stopped.direction==-capped.direction && stopped.direction_reversed &&
        stopped.reason=="purpose_pulse_unresolved_at_current_ceiling_measure_opposite_direction",
        "one exhausted directional domain may request its separately bounded opposite direction without inventing a hardware limit");
    auto bad=low_speed_pd_history(.14);for(auto& r:bad)r.independent_disturbance_known=false;
    check(!estimate_purpose_input_prior(bad,noise,"unknown.csv").valid,"unknown actual input must not produce a reliable prior");
    bad=low_speed_pd_history(.14);for(auto& r:bad)r.limited=true;
    check(!estimate_purpose_input_prior(bad,noise,"limited.csv").valid,"clipped path input cannot seed a stronger purpose input");
    bad=low_speed_pd_history(.14);for(auto& r:bad){r.independent_disturbance_nm=.07;r.planned_independent_disturbance_nm=.14;r.independent_input_planned_known=true;}
    check(!estimate_purpose_input_prior(bad,noise,"clipped-independent.csv").valid,"planned force must not replace the actually reduced input");
    bad=low_speed_pd_history(0);for(auto& r:bad){r.actual+=noise.encoder_step_rad*(r.feedback_sequence%2?1:-1);r.velocity=.1*(r.feedback_sequence%2?1:-1);}
    check(!estimate_purpose_input_prior(bad,noise,"noise.csv").valid,"sub-resolution position and derivative-code noise cannot become a static force prior");
    bad=low_speed_pd_history(0);for(auto& r:bad)r.independent_disturbance_nm=.12;
    const auto known=estimate_purpose_input_prior(bad,noise,"independent.csv");
    check(known.valid && known.independent_samples==240 && known.independent_peak_nm==.12 && known.low_speed_pd_samples==0,
        "known independent input and low-speed PD evidence retain separate counts and sources");
}
void sampled_zero_stiffness_impulse_domain() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    for(std::size_t n:{1,3,7}) {
        std::vector<std::size_t> order(n);std::iota(order.begin(),order.end(),0);
        for(unsigned permutation=0;permutation<2;++permutation) {
            for(auto axis:order) {
                const double mass=.001+.0002*axis,damping=.004+.002*axis;
                auto bound=limits(.3);PurposePulseOptions options;options.zero_stiffness=true;options.available_s=20;
                options.torque_quantum_nm=20.0/4095;options.input_prior.valid=true;options.input_prior.seed_nm=.14;
                const auto plan=plan_purpose_pulse(bound,noise,mass,0,damping,options);
                check(plan.valid && plan.timing_shortened && plan.amplitude_nm>=options.torque_quantum_nm,
                    "a long P0 pulse below the first force code must shorten within sampled execution limits");
                check(plan.maximum_sampled_input_ceiling_nm+1e-12>=plan.input_ceiling_nm,
                    "the minimum-waveform planning ceiling must not freeze input retries at the current longer pulse ceiling");
                const double room=.3-plan.motion_floor_rad,impulse=plan.amplitude_nm*plan.impulse_bound_area_s;
                check(impulse/damping<=.5*room+1e-12 && impulse/mass<=.5*bound.max_velocity+1e-12 &&
                    plan.amplitude_nm<=bound.effort_budget && plan.rise_s>=2*options.feedback_interval_s &&
                    plan.hold_s>=2*options.feedback_interval_s && plan.fall_s>=2*options.feedback_interval_s,
                    "shortening must preserve free-release J/D travel, J/M speed, effort and minimum sampled ramp bounds");
                const auto data=simulate(plan,noise,mass,0,damping);
                double travel=0,speed=0;
                for(const auto& row:data){travel=std::max(travel,std::abs(row.actual));speed=std::max(speed,std::abs(row.velocity));}
                check(travel<=.5*room+noise.encoder_step_rad && speed<=.5*bound.max_velocity+1e-6,
                    "an actual free inertial MIT plant without friction must remain within the declared impulse bounds");
            }
            std::reverse(order.begin(),order.end());
        }
    }
    auto bound=limits(.4);PurposePulseOptions options;options.zero_stiffness=true;options.available_s=20;
    options.torque_quantum_nm=.001;options.input_prior.valid=true;options.input_prior.seed_nm=.14;
    const auto feasible=plan_purpose_pulse(bound,noise,.02,0,.1,options);
    check(feasible.valid && feasible.timing_shortened && feasible.prior_input_representable && feasible.amplitude_nm>=.14,
        "a measured force prior that fits a shorter sampled impulse must not be crushed by the old long-waveform travel cap");
    auto narrow=limits(.04);options.torque_quantum_nm=.01;
    const auto impossible=plan_purpose_pulse(narrow,noise,.001,0,.00001,options);
    check(!impossible.valid && impossible.amplitude_nm==0 && impossible.reason=="purpose_pulse_no_representable_input",
        "no representable sampled P0 impulse may fit an extremely small damping/range domain; force cannot bypass it");
    options.available_s=1.5;
    check(!plan_purpose_pulse(bound,noise,.02,0,.1,options).valid,
        "short pulses still reserve the full zero-input observation and cannot enlarge the wall budget");
}
void settled_observation_supplements() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;
    for(std::size_t n:{1,3,7})for(std::size_t axis=0;axis<n;++axis) {
        const double scale=1+.1*axis;PurposePulseOptions options;options.available_s=60;
        options.input_prior.valid=true;options.input_prior.seed_nm=.1*scale;
        auto bound=limits(.4);bound.effort_budget=2;
        auto current=plan_purpose_pulse(bound,noise,.01*scale,scale,.2*scale,options);
        check(current.valid && current.restoring_domain_bounded,"restoring input must retain its static recovery-domain bound");
        auto measured=assess_purpose_pulse(simulate(current,noise,.05*scale,scale,2*scale),noise,current);
        check(measured.input_measurement_complete && measured.yield_observed && !measured.hold_settled,
            "unmodelled slow actual response must be retained as moving but unsettled evidence");
        const double amplitude=current.amplitude_nm,tail=current.duration_s-current.release_s;
        unsigned tries=0;
        while(!measured.hold_settled && tries++<6) {
            const auto next=next_purpose_pulse(current,measured);
            check(next.valid && next.supplement_action=="extend_same_input_observation" &&
                next.amplitude_nm==amplitude && next.hold_s>current.hold_s && next.duration_s>current.duration_s &&
                std::abs(next.duration_s-next.release_s-tail)<1e-10 && next.input_ceiling_nm==current.input_ceiling_nm,
                "known moving plateau must extend observation at identical input and preserve room/effort/unloaded-tail constraints");
            check(next.duration_s>current.duration_s+.1,"a caller must reserve the entire longer window, not the old duration");
            current=next;measured=assess_purpose_pulse(simulate(current,noise,.05*scale,scale,2*scale),noise,current);
        }
        check(tries>0 && measured.hold_settled && measured.direct_response_complete,
            "same-force observation supplements must resolve a slow plateau without escalating force");
        auto unknown=measured;unknown.input_measurement_complete=false;
        check(!next_purpose_pulse(current,unknown).valid,"unknown input cannot justify a longer pulse");
        auto free=current;free.restoring_domain_bounded=false;free.impulse_bound_area_s=.1;
        measured.hold_settled=false;
        check(!next_purpose_pulse(free,measured).valid,
            "a free-release impulse must be replanned inside its original J/D and J/M bounds, never silently extended");
    }
}
void model_operating_condition_comparison() {
    for(std::size_t n:{1,3,7}) {
        std::vector<double> mass(n*n),gravity(n),input(n),quantum(n,.001);
        for(std::size_t i=0;i<n;++i) {
            gravity[i]=.1*i;input[i]=.05+.03*i;
            for(std::size_t j=0;j<n;++j)
                mass[i*n+j]=std::sqrt(std::pow(10.0,-static_cast<double>(i+j)))*(.03+(i==j?1:0));
        }
        const auto same=purpose_comparison_condition(mass,mass,gravity,gravity,input,quantum);
        check(same.comparable && same.relative_M_change<1e-10 && same.max_gravity_change==0,
            "same mapped inertia and actually executed gravity must permit comparison without encoder-code coincidence");
        auto nearby_mass=mass,nearby_gravity=gravity;
        for(auto& x:nearby_mass)x*=1.04;
        for(std::size_t i=0;i<n;++i)nearby_gravity[i]+=.05*input[i]+quantum[i];
        const auto near=purpose_comparison_condition(mass,nearby_mass,gravity,nearby_gravity,input,quantum);
        check(near.comparable && std::abs(near.relative_M_change-.04)<1e-10 && near.max_gravity_change>0,
            "small known operating-condition changes must use declared 5-percent inertia and 10-percent-input tolerances plus one force code");
        auto permute=[&](const std::vector<double>& values,bool matrix) {
            std::vector<double> out(values.size());
            for(std::size_t i=0;i<n;++i) {
                if(matrix)for(std::size_t j=0;j<n;++j)out[i*n+j]=values[(n-1-i)*n+(n-1-j)];
                else out[i]=values[n-1-i];
            }
            return out;
        };
        const auto reversed=purpose_comparison_condition(permute(mass,true),permute(nearby_mass,true),
            permute(gravity,false),permute(nearby_gravity,false),permute(input,false),permute(quantum,false));
        check(reversed.comparable && std::abs(reversed.relative_M_change-near.relative_M_change)<1e-10 &&
            std::abs(reversed.max_gravity_change-near.max_gravity_change)<1e-12,
            "joint permutation must preserve the complete physical operating-condition comparison");
        auto changed=mass;
        for(std::size_t i=0;i<n;++i){changed[i*n+n-1]*=std::sqrt(1.12);changed[(n-1)*n+i]*=std::sqrt(1.12);}
        const auto weak_axis=purpose_comparison_condition(mass,changed,gravity,gravity,input,quantum);
        check(!weak_axis.comparable && weak_axis.relative_M_change>.05,
            "a weak-axis inertia change must not be hidden by a heavier joint's ordinary matrix norm");
        auto load=gravity;load.back()+=.1*input.back()+quantum.back()+.00001;
        check(!purpose_comparison_condition(mass,mass,gravity,load,input,quantum).comparable,
            "gravity execution change above the known input-relative tolerance must reject a damping comparison");
        changed=mass;changed.back()=-1;
        check(!purpose_comparison_condition(mass,changed,gravity,gravity,input,quantum).comparable,
            "a nonphysical mass matrix cannot establish comparable inertia");
        changed=mass;changed.front()=std::numeric_limits<double>::quiet_NaN();
        check(!purpose_comparison_condition(mass,changed,gravity,gravity,input,quantum).comparable,
            "unknown current inertia must remain unknown rather than falling back to pose proximity");
        load=gravity;load.back()=std::numeric_limits<double>::quiet_NaN();
        check(!purpose_comparison_condition(mass,mass,gravity,load,input,quantum).comparable,
            "unknown actually executed gravity must not be replaced by an assumed correct model term");
        check(!purpose_comparison_condition(mass,mass,gravity,gravity,std::vector<double>(n,0),quantum).comparable,
            "no known nonzero experiment input cannot establish input-relative load comparability");
    }
    check(!purpose_comparison_condition({}, {}, {}, {}, {}, {}).comparable,
        "empty models cannot authorize an operating-condition comparison");
}
}
int main(){try{planning_and_waveform();plateau_measurement();mapped_isolation_and_drag();comparison_initial_pose();measured_prior_and_stiction_retry();sampled_zero_stiffness_impulse_domain();
    settled_observation_supplements();model_operating_condition_comparison();opposite_direction_after_measured_deadzone();
    std::cout<<"purpose pulse measurement: passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
