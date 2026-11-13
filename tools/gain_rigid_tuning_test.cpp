#include "serial_arm/dynamics/gain_rigid_tuning.hpp"
#include "serial_arm/dynamics/gain_candidate_admission.hpp"

#include <iostream>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void near(double a,double b,const char* message){check(std::abs(a-b)<1e-10*std::max({1.0,std::abs(a),std::abs(b)}),message);}

RigidUseEvidence evidence(double p=10,double d=1) {
    RigidUseEvidence out;out.measured_kp=p;out.measured_kd=d;out.fixed_gain_measurement_complete=true;
    out.input_signature="fixed positive torque envelope + release, 0.1Nm,0.2-1.2s,3.6s";
    out.execution_scope="fixed taught interior reference, declared load, same gravity FF and integral state";
    out.time_resolution_s=.005;
    auto& response=out.disturbance;
    response.input_measurement_complete=true;response.measurement_complete=true;response.input_known=true;
    response.reference_fixed=true;response.gains_constant=true;response.yield_observed=true;
    response.motion_floor_rad=.0002;response.input_peak_nm=.1;response.initial_equilibrium_rad=.203;
    response.initial_reference_error_rad=.003;response.peak_displacement_rad=.01;
    response.final_reference_error_rad=.003;response.final_equilibrium_error_rad=.0001;
    response.final_drift_rad=.0001;response.final_speed_rad_s=.0001;response.release_recovery_s=.3;
    return out;
}
void directions_and_protocol() {
    auto measured=evidence(10,5);
    const auto candidate=rigid_candidate_report({{10},{5}},{500},{5},{measured});
    check(candidate.changed && candidate.gains.kp[0]>10 && candidate.gains.kp[0]<=11.2+1e-12 && candidate.gains.kd[0]==5,
        "a damping protocol ceiling froze an evidence-based independent stiffness comparison");
    check(candidate.gains.kp[0]<20,"one resolved error triggered a direct large-gain jump");
    measured.disturbance.sustained_oscillation=true;measured.disturbance.release_reversals=5;
    const auto ringing=rigid_candidate_report({{10},{5}},{500},{5},{measured});
    near(ringing.gains.kp[0],8.8,"repeated release oscillation did not propose bounded stiffness reduction");
    near(ringing.gains.kd[0],5,"stiffness reduction silently modified damping");
    measured.disturbance.sustained_oscillation=false;measured.disturbance.release_reversals=1;
    check(rigid_candidate_report({{10},{5}},{500},{5},{measured}).gains.kp[0]>10,
        "a single release overshoot was mislabeled as sustained oscillation or resonance");
    auto missing=measured;missing.disturbance.input_known=false;
    check(!rigid_candidate_report({{10},{5}},{500},{5},{missing}).changed,
        "unknown independent input fabricated a measured stiffness revision");
    auto clipped=measured;clipped.disturbance.limited=true;
    check(!rigid_candidate_report({{10},{5}},{500},{5},{clipped}).changed,
        "limited input was used as unqualified stiffness evidence");
    auto mixed=measured;mixed.measured_kp=9;
    check(!rigid_candidate_report({{10},{5}},{500},{5},{mixed}).changed,
        "evidence from a different actual controller drove this candidate");
    const GainCommandGrid grid{500.0/4095,5.0/4095};
    const double p=3*grid.kp_step,d=3*grid.kd_step;
    auto low=evidence(p,d);
    const auto encoded=rigid_candidate_report({{p},{d}},{500},{5},{low},{grid});
    near(encoded.gains.kp[0],4*grid.kp_step,"a sub-code local increase froze instead of proposing one adjacent code");
    near(encoded.gains.kd[0],d,"stiffness protocol rounding changed unchanged damping");
    auto minimum=evidence(grid.kp_step,d);minimum.disturbance.sustained_oscillation=true;
    const auto minimum_report=rigid_candidate_report({{grid.kp_step},{d}},{500},{5},{minimum},{grid});
    near(minimum_report.gains.kp[0],grid.kp_step,"a minimum restoring code was removed");
    near(minimum_report.gains.kd[0],4*grid.kd_step,"ringing at minimum positive stiffness did not offer an adjacent damping comparison");
    const ModelAxisLimits bound{.1,5,20,2,500,5,grid};
    check(admit_complete_gain_candidate({{p},{d}},encoded.gains,{bound},{.01},{.01},{.001},{0}).allowed,
        "a bounded positive-grid rigid comparison did not survive existing unchanged whole-group admission");
    auto strict=bound;strict.effort_budget=.001;
    check(!admit_complete_gain_candidate({{p},{d}},encoded.gains,{strict},{.01},{.01},{.001},{0}).allowed,
        "the rigid helper bypassed existing feedback effort admission");
}

void reference_comparison_and_hold() {
    const Gains old{{10},{1}},next{{11.2},{1}};const auto baseline=evidence();auto improved=baseline;
    improved.measured_kp=11.2;improved.disturbance.peak_displacement_rad=.0088;
    improved.disturbance.final_reference_error_rad=.0026;
    check(rigid_use_preferred(next,{improved},old,{baseline}),"resolved same-input stiffness improvement was not retained");
    auto relabelled=baseline;relabelled.disturbance.passed=true;relabelled.disturbance.stable_recovery=true;
    check(!rigid_use_preferred(old,{relabelled},old,{baseline}),"pass labels alone selected unchanged raw measurements");
    auto weaker=improved;weaker.disturbance.input_peak_nm=.05;
    check(!rigid_use_preferred(next,{weaker},old,{baseline}),"a smaller input fabricated better rigidity");
    weaker=improved;weaker.input_signature="same peak but different waveform";
    check(!rigid_use_preferred(next,{weaker},old,{baseline}),"changed input timing or waveform was compared using only its peak");
    auto center=improved;center.disturbance.initial_equilibrium_rad+=.02;
    check(!rigid_use_preferred(next,{center},old,{baseline}),"a different fixed reference center was silently compared");
    auto ff=improved;ff.execution_scope="changed gravity feedforward";
    check(!rigid_use_preferred(next,{ff},old,{baseline}),"different feedforward or load conditions fabricated a controller improvement");
    auto slow=improved;slow.disturbance.release_recovery_s+=.1;
    check(!rigid_use_preferred(next,{slow},old,{baseline}),"smaller displacement hid worse release recovery");
    auto ringing=improved;ringing.disturbance.sustained_oscillation=true;ringing.disturbance.release_reversals=4;
    check(!rigid_use_preferred(next,{ringing},old,{baseline}),"smaller displacement hid newly repeated oscillation");
    auto limited=improved;limited.disturbance.limited=true;
    check(!rigid_use_preferred(next,{limited},old,{baseline}),"a newly limited candidate was retained");
    auto unresolved=evidence();unresolved.disturbance.peak_displacement_rad=.0001;
    unresolved.disturbance.final_reference_error_rad=.0001;unresolved.disturbance.yield_observed=false;
    unresolved.disturbance.measurement_complete=false;
    check(!rigid_candidate_report(old,{500},{5},{unresolved}).changed,
        "sub-resolution independent motion became either a rigidity failure or a stiffness search target");
    unresolved.hold={true,true,true,true,false,false,.01,.0001,.0001,.0002,.2};
    check(rigid_candidate_report(old,{500},{5},{unresolved}).changed,
        "actual unloaded static hold error did not participate in restoring gain comparison");
    auto settled=unresolved;settled.measured_kp=11.2;settled.hold.static_error_rad=.008;
    check(rigid_use_preferred(next,{settled},old,{unresolved}),
        "a measured static hold improvement was ignored when independent yielding was below resolution");
    settled.hold.reference_position_rad=.3;
    check(!rigid_use_preferred(next,{settled},old,{unresolved}),"a different hold center fabricated static improvement");
    auto unknown_hold=unresolved;unknown_hold.disturbance.input_known=false;unknown_hold.hold.input_known_zero=false;
    check(!rigid_candidate_report(old,{500},{5},{unknown_hold}).changed,"unknown hold load/input fabricated a static correction");
}

// The simulator directly integrates a sampled joint-side MIT port:
// I*qdd = P*(r-q) + D*(rdot-v) + known external input + fixed bias - b*v.
// Nothing in it is copied from the controller's desired response metrics.
RigidUseEvidence sampled_mit(double p,double d,std::size_t axis,bool unknown=false) {
    const double inertia=.05*(1+.15*axis),bias=.035*(1+.2*axis),r=.15+.02*axis;
    constexpr double dt=.001,release=1.2,step=1e-5;
    double q=r+bias/p,v=0;std::vector<Sample> rows;
    for(unsigned k=0;k<=3800;++k) {
        const double time=k*dt;
        double input=0;
        if(time>=.2 && time<release) {
            const double fraction=std::clamp(std::min(time-.2,release-time)/.12,0.0,1.0);
            input=.12*(1+.1*axis)*fraction*fraction*(3-2*fraction);
        }
        const double effort=p*(r-q)-d*v+input+bias-.025*v;
        v+=effort/inertia*dt;q+=v*dt;
        if(k%5)continue;
        Sample row;row.time_s=time;row.command_time_s=time;row.feedback_time_s=time;
        row.reference=r;row.reference_velocity=0;row.actual=step*std::round(q/step);row.velocity=v;
        row.new_measurement=true;row.feedback_sequence=k+1;row.actual_kp=p;row.actual_kd=d;
        row.actual_gains_known=true;row.independent_disturbance_known=!unknown;row.independent_disturbance_nm=input;
        row.feedforward_known=true;row.applied_feedforward_nm=0;rows.push_back(row);
    }
    NoiseEstimate noise;noise.encoder_step_rad=step;
    RigidUseEvidence out;out.measured_kp=p;out.measured_kd=d;
    out.disturbance=assess_compliant_response(rows,noise,release,compliant_use_requirements(.1,.12/p,noise));
    out.fixed_gain_measurement_complete=true;out.time_resolution_s=.005;
    out.input_signature="same smooth finite external torque envelope, normalized axis input, t.2-1.2,release1.2,3.8s";
    out.execution_scope="sampled_MIT_same_declared_bias_inertia_center_and_zero_FF";
    return out;
}
void arbitrary_counts_and_actual_mit_improvement() {
    for(std::size_t n:{1,3,7}) {
        Gains old;std::vector<RigidUseEvidence> baseline;
        for(std::size_t i=0;i<n;++i) {
            old.kp.push_back(10*(1+.2*i));old.kd.push_back(1.6*(1+.2*i));
            baseline.push_back(sampled_mit(old.kp.back(),old.kd.back(),i));
            check(baseline.back().disturbance.input_measurement_complete,"actual sampled MIT baseline did not complete its known-input window");
        }
        const auto proposal=rigid_candidate_report(old,std::vector<double>(n,500),old.kd,baseline);
        check(proposal.changed,"actual measured rigid displacement did not generate a bounded complete proposal");
        std::vector<ModelAxisLimits> bounds(n,ModelAxisLimits{.1,5,20,2,500,5,{0,0}});
        check(admit_complete_gain_candidate(old,proposal.gains,bounds,std::vector<double>(n,.02),
            std::vector<double>(n,.2),std::vector<double>(n,0),std::vector<double>(n,0)).allowed,
            "actual sampled MIT candidate was not admitted as a complete group");
        std::vector<RigidUseEvidence> candidate;
        double peak_before=0,peak_after=0;
        for(std::size_t i=0;i<n;++i) {
            candidate.push_back(sampled_mit(proposal.gains.kp[i],proposal.gains.kd[i],i));
            peak_before+=baseline[i].disturbance.peak_displacement_rad;peak_after+=candidate[i].disturbance.peak_displacement_rad;
            check(proposal.gains.kd[i]==old.kd[i],"damping protocol ceiling forced an invented coupled damping revision");
        }
        check(rigid_use_preferred(proposal.gains,candidate,old,baseline),"true sampled MIT rigid improvement was rejected");
        check(peak_after<peak_before,"candidate gains did not actually reduce sampled plant disturbance displacement");
        auto shuffled_old=old,shuffled_new=proposal.gains;
        for(auto* list:{&shuffled_old.kp,&shuffled_old.kd,&shuffled_new.kp,&shuffled_new.kd})std::reverse(list->begin(),list->end());
        std::reverse(candidate.begin(),candidate.end());std::reverse(baseline.begin(),baseline.end());
        check(rigid_use_preferred(shuffled_new,candidate,shuffled_old,baseline),"axis permutation changed whole-group empirical selection");
        std::cout<<"rigid sampled MIT axes="<<n<<" summed peak "<<peak_before<<" -> "<<peak_after<<'\n';
    }
    auto unknown=sampled_mit(10,1.6,0,true);
    check(!unknown.disturbance.input_measurement_complete &&
        !rigid_candidate_report({{10},{1.6}},{500},{5},{unknown}).changed,
        "actual unknown-input sampled simulation fabricated a rigid tuning result");
    // Unresolved axes can accompany an improved axis only unchanged and with
    // a real complete fixed-group record. They are never spliced from trials.
    const Gains old{{10,10,10},{1,1,1}},next{{11.2,10,10},{1,1,1}};
    std::vector<RigidUseEvidence> previous(3,evidence()),candidate=previous;
    candidate[0].measured_kp=11.2;candidate[0].disturbance.peak_displacement_rad=.008;
    for(std::size_t i=1;i<3;++i) {previous[i].disturbance.input_known=false;candidate[i].disturbance.input_known=false;}
    check(rigid_use_preferred(next,candidate,old,previous),"unchanged unidentifiable axes blocked a measured whole-group improvement");
    auto spliced=next;spliced.kp[2]=11.2;candidate[2].measured_kp=11.2;
    check(!rigid_use_preferred(spliced,candidate,old,previous),"an unidentifiable changed axis was silently spliced into the selected group");
    candidate[2].fixed_gain_measurement_complete=false;candidate[2].measured_kp=10;
    check(!rigid_use_preferred(next,candidate,old,previous),"a missing whole-group measurement was silently filled from the baseline");
}
}
int main(){try{directions_and_protocol();reference_comparison_and_hold();arbitrary_counts_and_actual_mit_improvement();
    std::cout<<"rigid bounded empirical gain checks passed\n";return 0;}
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
