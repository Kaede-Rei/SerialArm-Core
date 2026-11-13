#include "serial_arm/dynamics/gain_compliant_tuning.hpp"
#include "serial_arm/dynamics/gain_disturbance_plan.hpp"
#include <functional>
#include <iostream>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
std::vector<Sample> rows(const std::function<double(double)>& position,double end,double P,double D) {
    std::vector<Sample> result;
    for(unsigned k=0;k<=static_cast<unsigned>(end*200);++k) {
        Sample row;row.time_s=.005*k;row.actual=position(row.time_s);
        row.velocity=(position(row.time_s+.0001)-position(row.time_s-.0001))/.0002;
        row.new_measurement=true;row.feedback_sequence=k+1;
        row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;
        row.independent_disturbance_known=true;row.command_time_s=row.time_s;row.feedback_time_s=row.time_s-.005;
        result.push_back(row);
    }
    return result;
}
void compliant_protocol_rounding() {
    for(std::size_t n:{1,3,8}) {
        std::vector<ModelAxisLimits> limits(n);Gains candidate;
        std::vector<double> pmax(n),dmax(n);
        for(std::size_t i=0;i<n;++i) {
            auto& x=limits[i];x.kp_max=500;x.kd_max=5;x.grid.kp_step=500.0/4095;x.grid.kd_step=5.0/4095;
            const double rigidP=(3+2*i)*x.grid.kp_step,rigidD=(9+2*i)*x.grid.kd_step;
            candidate.kp.push_back(.5*rigidP);candidate.kd.push_back(std::sqrt(.5)*rigidD);
            pmax[i]=.65*rigidP;dmax[i]=.85*rigidD;
        }
        check(encode_compliant_candidate(candidate,pmax,dmax,limits),"small protocol codes must generate a consistent compliant controller");
        for(std::size_t i=0;i<n;++i)check(candidate.kp[i]<=pmax[i] && candidate.kd[i]<=dmax[i] && candidate.kp[i]>0 && candidate.kd[i]>0,
            "encoded restoring gains must not exceed the encoded bounds");
        std::vector<CompliantUseAssessment> missing(n);
        compliant_candidate_report(candidate,pmax,dmax,missing);
        auto once=candidate;
        check(encode_compliant_candidate(candidate,pmax,dmax,limits) && once.kp==candidate.kp && once.kd==candidate.kd,
            "reencoding a complete candidate must be idempotent");
        pmax[0]=.25*limits[0].grid.kp_step;
        check(!encode_compliant_candidate(candidate,pmax,dmax,limits),"no positive restoring code is explicitly unavailable");
    }
}
void discovery_budget_priority() {
    // Recorded task 1794478087644 reached 40.98/43.35 source seconds,
    // then an all-supplements reserve stopped it at 170 seconds. There were
    // still more than four minutes before its unchanged 15T deadline.
    constexpr double hard=15*28.14,elapsed_before=3;
    const double path=42.425426111291245,tail=1.5;
    const double reserve=discovery_required_validation_reserve(path+tail+3,{path/.65+tail+3},6,7.5);
    check(hard-elapsed_before-reserve>250 && 185+reserve<hard,
        "required full paths/holds/drag must fit after discovery without reserving every optional supplement ahead of traversal");
    const double fine=discovery_required_validation_reserve(path+tail+3,{},0,0);
    check(fine==path+tail+5 && 24*28.14-fine>hard-reserve,
        "single-use fine calibration reserves its own actual validation and exit without expanding recorded T");
    bool invalid=false;try{(void)discovery_required_validation_reserve(path,{-1},0,0);}catch(const std::invalid_argument&){invalid=true;}
    check(invalid,"invalid validation costs cannot manufacture a larger discovery budget");
}
void drag_use() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    auto quiet=rows([](double t){return .2+.0001*std::sin(2*pi*40*t);},1,0,.2);
    for(std::size_t i=0;i<quiet.size();++i)quiet[i].velocity=(i%2?1:-1)*.007326;
    const auto stationary=assess_drag_stationarity(quiet,noise);
    check(stationary.measurement_complete && stationary.passed && stationary.raw_peak_velocity_rad_s>.003,
        "zero stiffness quiet position stability must remain usable despite diagnostic velocity-code chatter");
    const auto no_release=assess_drag_release(quiet,noise,.2);
    check(!no_release.measurement_complete && !no_release.passed,
        "quiet dragging is useful evidence but cannot invent a motion-release test");
    const auto quiet_candidate=drag_damping_candidate_report({{0},{.2}},{5},{stationary},{no_release});
    check(!quiet_candidate.changed && quiet_candidate.gains.kp[0]==0 &&
        quiet_candidate.reasons[0]=="quiet_drag_goal_met_release_scope_unmeasured",
        "missing release evidence must not remove the usable quiet scope or increase damping");
    auto nonzero=quiet;for(auto& row:nonzero)row.actual_kp=1;
    check(!assess_drag_stationarity(nonzero,noise).measurement_complete,
        "drag validation requires measured zero stiffness, not merely a zero-stiffness candidate label");
    auto moving=rows([](double t){return t<1?.015*t:.015+.003*(1-std::exp(-(t-1)/.2));},2.6,0,.2);
    const auto release=assess_drag_release(moving,noise,1,{},true);
    check(release.measurement_complete && release.passed && release.final_position_rad-release.release_position_rad>.002,
        "zero stiffness release must decay and may stop at a new position without returning to its original pose");
    check(!assess_drag_release(moving,noise,1).measurement_complete,
        "spontaneous movement without known preparation must not become independent release evidence");
    auto slow=rows([](double t){return t<1?.015*t:.015+.008*(t-1);},2.6,0,.2);
    const auto slow_release=assess_drag_release(slow,noise,1,{},true);
    check(slow_release.measurement_complete && !slow_release.passed,
        "a measured still-moving release is complete but fails its own decay goal");
    const auto increased=drag_damping_candidate_report({{0},{.2}},{5},{stationary},{slow_release});
    check(increased.changed && increased.gains.kp[0]==0 && std::abs(increased.gains.kd[0]-.216)<1e-12,
        "slow release must propose a small independent damping comparison while stiffness remains zero");
    const auto drifting=assess_drag_stationarity(rows([](double t){return .2+.01*t;},1,0,.2),noise);
    check(drifting.measurement_complete && !drifting.passed &&
        !drag_damping_candidate_report({{0},{.2}},{5},{drifting},{slow_release}).changed,
        "persistent zero-stiffness bias drift must request feedforward/load review instead of racing damping to its limit");
    auto blended=moving;blended.back().actual_kd=.3;
    check(!assess_drag_release(blended,noise,1,{},true).measurement_complete,
        "release evidence collected with mixed actual damping cannot qualify a single gain group");
    auto limited=quiet;limited.back().limited=true;
    check(!assess_drag_stationarity(limited,noise).passed,
        "a limited quiet experiment cannot pass its requested use");
    auto duplicate=quiet;for(auto& row:duplicate)row.feedback_sequence=1;
    check(!assess_drag_stationarity(duplicate,noise).measurement_complete,
        "duplicated feedback cannot manufacture quiet measurement completion");
    for(std::size_t n:{1,3,7}) {
        Gains gains;gains.kp.assign(n,0);gains.kd.assign(n,.2);
        const auto group=drag_damping_candidate_report(gains,std::vector<double>(n,5),
            std::vector<DragStationaryAssessment>(n,stationary),std::vector<DragReleaseAssessment>(n,slow_release));
        check(group.gains.kp==gains.kp && group.changed && group.gains.kd.size()==n,
            "drag candidate logic must support arbitrary mapped joint counts without special joint cases");
    }
    check(drag_use_preferred({stationary},{release},{stationary},{slow_release}),
        "a complete measured release improvement must retain its whole damping group");
    check(!drag_use_preferred({stationary},{no_release},{stationary},{slow_release}),
        "a quiet-only candidate cannot erase an already measured failed release and appear preferable");
    check(!drag_use_preferred({stationary},{slow_release},{stationary},{no_release}),
        "newly acquired failed release evidence is not itself a reason to replace an equally quiet baseline");
    auto worse=slow_release;worse.final_speed_rad_s*=1.5;worse.decay_s+=.3;
    check(!drag_use_preferred({stationary},{worse},{stationary},{slow_release}),
        "an available candidate with worse measured decay must not replace a complete unqualified baseline");
    check(!drag_use_preferred({drifting},{release},{stationary},{release}),
        "newly worse quiet stability cannot be hidden by a successful motion release");
}

void disturbance_domain() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto both=independent_disturbance_direction_plan(.04,.04,noise,.1,10);
    check(both.eligible && both.bidirectional && both.direction==0 && both.required_room_rad==.02,
        "demonstrated room on both sides must preserve the original symmetric known torque experiment");
    const auto positive=independent_disturbance_direction_plan(0,.04,noise,.1,10);
    check(positive.eligible && !positive.bidirectional && positive.direction==1 && positive.requested_torque_nm==.1,
        "a measured lower endpoint must use only the demonstrated positive torque direction at unchanged amplitude");
    const auto negative=independent_disturbance_direction_plan(.04,0,noise,.1,10);
    check(negative.eligible && !negative.bidirectional && negative.direction==-1 &&
        negative.validation_scope=="negative_only_known_joint_torque",
        "a measured upper endpoint must report the negative-only scope instead of declaring the entire trial invalid");
    check(!independent_disturbance_direction_plan(.01,.01,noise,.1,10).eligible,
        "insufficient demonstrated room on both sides must remain missing evidence");
    check(!independent_disturbance_direction_plan(.04,.04,noise,0,10).eligible &&
        !independent_disturbance_direction_plan(.04,.04,noise,.1,0).eligible,
        "zero unknown input or zero restoring stiffness cannot be an independent compliant torque plan");
    const auto quantized=independent_disturbance_direction_plan(0,.0005,noise,.00001,10);
    check(!quantized.eligible,
        "one-sided room must still resolve encoder/noise displacement without expanding the observed domain");
    Requirements req;req.probe_duration_s=8;req.amplitude_rad=.015;req.motion_probe=false;req.reference_bias_rad=0;
    for(unsigned k=0;k<=8000;++k) {
        const double time=.001*k;
        const double plus=independent_disturbance_input(positive,time,req),minus=independent_disturbance_input(negative,time,req);
        check(plus>=0 && plus<=.1 && minus<=0 && minus>=-.1 && std::abs(plus+minus)<1e-12,
            "one-sided known torque must retain the chosen direction and requested amplitude bound throughout its waveform");
        check(std::abs(independent_disturbance_input(both,time,req)-probe(time,req).first/req.amplitude_rad*.1)<1e-12,
            "a bidirectional plan must retain its original multisine waveform");
        const double edge=std::clamp(std::min(time,req.probe_duration_s-time)/.7,0.0,1.0);
        const double envelope=edge*edge*(3-2*edge);
        check(std::abs(plus-.05*envelope-.5*independent_disturbance_input(both,time,req))<1e-12,
            "one-sided excitation must contain the same three tones plus the smooth DC envelope rather than a changed rectification spectrum");
    }
    constexpr double delta=1e-6;
    check(independent_disturbance_input(positive,-1,req)==0 && independent_disturbance_input(positive,8,req)==0 &&
        std::abs(independent_disturbance_input(positive,delta,req)/delta)<1e-5 &&
        std::abs(independent_disturbance_input(positive,8-delta,req)/delta)<1e-5,
        "one-sided disturbance must have zero input and first derivative at both ends");
}

void one_sided_mit_response() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto plan=independent_disturbance_direction_plan(0,.04,noise,.1,10);
    Requirements req;req.probe_duration_s=8;req.amplitude_rad=.015;
    constexpr double dt=.001,release=8.25;double q=0,v=0;
    std::vector<Sample> measured;
    for(unsigned k=0;k<=10000;++k) {
        const double time=k*dt,input=independent_disturbance_input(plan,time-.25,req);
        const double acceleration=(input-10*q-.8*v)/.05;
        v+=acceleration*dt;q+=v*dt;
        if(k%5)continue;
        Sample row;row.time_s=time;row.actual=q;row.velocity=v;
        row.new_measurement=true;row.feedback_sequence=k+1;row.actual_gains_known=true;row.actual_kp=10;row.actual_kd=.8;
        row.command_time_s=time;row.feedback_time_s=time;row.independent_disturbance_known=true;row.independent_disturbance_nm=input;
        measured.push_back(row);
    }
    const auto assessment=assess_compliant_response(measured,noise,release,compliant_use_requirements(.04,.01,noise));
    check(assessment.measurement_complete && assessment.passed && assessment.yield_observed && assessment.stable_recovery,
        "a declared MIT plant must produce measurable bounded yielding and unloaded recovery under the actual smooth one-sided input");
}

std::vector<Sample> compliance(double displacement=.015,double nominal_offset=.02,double recovery_time_constant=.12) {
    auto measured=rows([=](double t) {
        if(t<.2)return nominal_offset;
        if(t<1)return nominal_offset+displacement*(1-std::exp(-(t-.2)/.08));
        return nominal_offset+displacement*(1-std::exp(-.8/.08))*std::exp(-(t-1)/recovery_time_constant);
    },2.6,10,.2);
    for(auto& row:measured)row.independent_disturbance_nm=row.time_s>=.2 && row.time_s<1?.1:0;
    return measured;
}
void compliant_use() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto req=compliant_use_requirements(.04,.015,noise);
    check(req.maximum_displacement_rad==.03,
        "compliant displacement acceptance must come from a declared model/local response envelope");
    const auto measured=assess_compliant_response(compliance(),noise,1,req);
    check(measured.measurement_complete && measured.passed && measured.yield_observed && measured.stable_recovery &&
        measured.peak_displacement_rad>.1*pi/180 && measured.final_reference_error_rad>.01 &&
        measured.final_equilibrium_error_rad<.001,
        "bounded compliant yielding above rigid precision must recover its unloaded equilibrium and retain nominal static error separately");
    check(measured.signed_compliance_rad_nm>0 && measured.validation_scope.find("not_contact_force")!=std::string::npos,
        "known joint torque evidence must identify its interaction scope without claiming real contact force acceptance");
    const auto locked=assess_compliant_response(compliance(0),noise,1,req);
    check(locked.input_measurement_complete && !locked.measurement_complete && !locked.passed && !locked.yield_observed,
        "an immobile rigid response to known input is not successful compliant yielding");
    const auto softer=compliant_candidate_report({{10},{.2}},{500},{5},{locked});
    check(!softer.changed && softer.gains.kp[0]==10 && softer.gains.kd[0]==.2 &&
        softer.reasons[0]=="known_input_yield_unresolved_increase_measurement_input_before_gain_revision",
        "unresolved yielding cannot justify blind whole-group softening before an informative input is measured");
    const GainCommandGrid grid{500.0/4095,5.0/4095};
    const auto resolved_small=assess_compliant_response(compliance(.005),noise,1,req);
    const auto low_code=compliant_candidate_report({{3*grid.kp_step},{3*grid.kd_step}},{500},{5},{resolved_small},{grid});
    check(low_code.changed && low_code.gains.kp[0]==2*grid.kp_step && low_code.gains.kd[0]==2*grid.kd_step,
        "coarse restoring/damping channels must retain one adjacent positive protocol code rather than erase every softening trial");
    auto unknown=compliance();for(auto& row:unknown)row.independent_disturbance_known=false;
    const auto missing=assess_compliant_response(unknown,noise,1,req);
    check(!missing.measurement_complete && !missing.passed &&
        !compliant_candidate_report({{10},{.2}},{500},{5},{missing}).changed,
        "unknown interaction input must not qualify compliance or justify gain changes");
    const auto too_far=assess_compliant_response(compliance(.05),noise,1,req);
    check(too_far.measurement_complete && !too_far.passed &&
        too_far.reason=="compliant_yield_above_model_local_envelope",
        "measured yielding outside the model/local envelope must fail compliance without changing Safety limits");
    const auto stiffer=compliant_candidate_report({{10},{.2}},{500},{5},{too_far});
    check(stiffer.changed && stiffer.gains.kp[0]==10.8 && stiffer.gains.kd[0]==.2,
        "excess measured yielding must propose an independent bounded stiffness comparison");
    auto moving_reference=compliance();for(auto& row:moving_reference)row.reference_velocity=.01;
    check(!assess_compliant_response(moving_reference,noise,1,req).measurement_complete,
        "reference-driven motion must remain separate from independent interaction response");
    auto mixed=compliance();mixed.back().actual_kp=11;
    check(!assess_compliant_response(mixed,noise,1,req).measurement_complete,
        "a mixed applied controller is not one jointly validated compliant gain group");
    auto gapped=compliance();for(auto& row:gapped)if(row.time_s>.6)row.time_s+=.1;
    const auto gap=assess_compliant_response(gapped,noise,1,req);
    check(!gap.measurement_complete && !compliant_candidate_report({{10},{.2}},{500},{5},{gap}).changed,
        "incomplete timing evidence must not generate an apparently measured compliant candidate");
    auto limited=compliance();limited.back().limited=true;
    const auto saturation=assess_compliant_response(limited,noise,1,req);
    check(!saturation.passed && !compliant_candidate_report({{10},{.2}},{500},{5},{saturation}).changed,
        "limited interaction evidence must not pass or drive a candidate outside known input conditions");

    const auto slow=assess_compliant_response(compliance(.015,.02,5),noise,1,req);
    const auto recovering=assess_compliant_response(compliance(.015,.02,2),noise,1,req);
    check(slow.measurement_complete && recovering.measurement_complete && !slow.passed && !recovering.passed &&
        compliant_use_preferred({recovering},{slow}),
        "a resolved measured recovery improvement must be retainable even when both controllers miss final compliance goals");
    const auto less_excessive=assess_compliant_response(compliance(.04),noise,1,req);
    check(!too_far.passed && !less_excessive.passed && compliant_use_preferred({less_excessive},{too_far}),
        "reduced actual yielding beyond a shared envelope must be comparable before final acceptance is reached");
    check(compliant_use_preferred({measured},{locked}),
        "new complete bounded yielding may add the missing motion scope to a complete known-input locked response");
    const auto smaller_yield=assess_compliant_response(compliance(.005),noise,1,req);
    const auto larger_yield=assess_compliant_response(compliance(.012),noise,1,req);
    check(smaller_yield.measurement_complete && larger_yield.measurement_complete &&
        compliant_use_preferred({larger_yield},{smaller_yield}),
        "resolved larger yielding inside the same envelope with recovery still qualified is useful compliance, not worse rigid tracking");
    check(!compliant_use_preferred({locked},{measured}) && !compliant_use_preferred({missing},{slow}) &&
        !compliant_use_preferred({gap},{slow}) && !compliant_use_preferred({saturation},{slow}),
        "lost yielding, unknown input, gapped or limited trials cannot replace a complete measured compliant group");
    check(!compliant_use_preferred({slow},{slow}),
        "repeating an unqualified trial is not a measured improvement");
    auto relabelled=slow;relabelled.passed=true;relabelled.stable_recovery=true;
    check(!compliant_use_preferred({relabelled},{slow}),
        "changing final acceptance labels without improving raw purpose metrics must not select a candidate");
    auto changed_input=recovering;changed_input.input_peak_nm*=.5;
    auto changed_scope=recovering;changed_scope.validation_scope="other_input_scope";
    check(!compliant_use_preferred({changed_input},{slow}) && !compliant_use_preferred({changed_scope},{slow}),
        "a weaker disturbance or different purpose scope is not a comparable controller improvement");
    auto loosened=too_far;loosened.displacement_limit_rad=.08;loosened.passed=true;
    check(!compliant_use_preferred({loosened},{too_far}),
        "a larger model envelope alone cannot turn the same excessive displacement into an improvement");
    auto unstable=recovering;unstable.sustained_oscillation=true;
    check(!compliant_use_preferred({unstable},{slow}),
        "better recovery error cannot hide newly sustained compliant oscillation");
    for(std::size_t n:{1,3,7}) {
        std::vector<CompliantUseAssessment> old(n,slow),next=old;
        next.back()=recovering;
        check(compliant_use_preferred(next,old),
            "whole compliant comparisons must follow measured axes for arbitrary joint counts");
        std::reverse(next.begin(),next.end());std::reverse(old.begin(),old.end());
        check(compliant_use_preferred(next,old),
            "joint permutation must preserve the measured compliant improvement");
        if(n>1) {
            next.back().final_equilibrium_error_rad+=.01;
            check(!compliant_use_preferred(next,old),
                "one substantially worse axis cannot be hidden by another axis's recovery improvement");
        }
    }
}
void compliant_partial_scope_group() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto requirements=compliant_use_requirements(.04,.015,noise);
    const auto locked=assess_compliant_response(compliance(0),noise,1,requirements);
    const auto small=assess_compliant_response(compliance(.005),noise,1,requirements);
    auto yielding_rows=compliance(.012);
    for(auto& row:yielding_rows){row.actual_kp=9.2;row.actual_kd=.184;}
    const auto yielding=assess_compliant_response(yielding_rows,noise,1,requirements);
    for(std::size_t n:{1,3,7}) {
        Gains previous;previous.kp.assign(n,10);previous.kd.assign(n,.2);
        std::vector<CompliantUseAssessment> old(n,locked),next=old;
        const auto all_unknown=compliant_candidate_report(previous,std::vector<double>(n,500),std::vector<double>(n,5),old);
        check(!all_unknown.changed && all_unknown.gains.kp==previous.kp && all_unknown.gains.kd==previous.kd,
            "all unresolved axes require more informative measured input rather than a blind whole-group gain reduction");
        check(!compliant_use_preferred(next,old,previous,previous,true,true),
            "unchanged unresolved axes alone cannot manufacture an improvement or a purpose pass");
        if(n==1)continue;
        old.front()=small;next.front()=yielding;
        const auto proposal=compliant_candidate_report(previous,std::vector<double>(n,500),std::vector<double>(n,5),old);
        check(proposal.changed && std::abs(proposal.gains.kp.front()-9.2)<1e-12 && std::abs(proposal.gains.kd.front()-.184)<1e-12,
            "the already measured yielding axis must define a bounded complete-group trial");
        for(std::size_t i=1;i<n;++i)check(proposal.gains.kp[i]==10 && proposal.gains.kd[i]==.2,
            "unresolved axes remain on their actual measured codes before the resolved-subset group is tested");
        check(!compliant_use_preferred(next,old),"legacy comparisons still require measured yielding on every axis");
        check(compliant_use_preferred(next,old,proposal.gains,previous,true,true),
            "an unchanged below-resolution axis with complete constant-PD input evidence must not erase another axis's complete measured improvement");
        check(!compliant_use_preferred(next,old,proposal.gains,previous,false,true),
            "an interrupted complete-group experiment cannot be retained through an unchanged-axis exception");
        auto changed_unknown=proposal.gains;changed_unknown.kp.back()=9.2;
        auto changed_measurement=next;changed_measurement.back().measured_kp=9.2;
        check(!compliant_use_preferred(changed_measurement,old,changed_unknown,previous,true,true),
            "changing an unresolved axis still requires that axis's actual same-input yielding/recovery evidence");
        auto lost_input=next;lost_input.back().input_known=false;
        auto limited=next;limited.back().limited=true;
        auto oscillating=next;oscillating.back().sustained_oscillation=true;
        auto false_parameter_claim=next;false_parameter_claim.back().measured_kp=9.9;
        check(!compliant_use_preferred(lost_input,old,proposal.gains,previous,true,true) &&
            !compliant_use_preferred(limited,old,proposal.gains,previous,true,true) &&
            !compliant_use_preferred(oscillating,old,proposal.gains,previous,true,true) &&
            !compliant_use_preferred(false_parameter_claim,old,proposal.gains,previous,true,true),
            "unchanged-axis handling cannot bypass unknown input, limits, oscillation or gains unlike the actually measured group");
        std::reverse(next.begin(),next.end());std::reverse(old.begin(),old.end());
        auto reversed_next=proposal.gains,reversed_old=previous;
        std::reverse(reversed_next.kp.begin(),reversed_next.kp.end());std::reverse(reversed_next.kd.begin(),reversed_next.kd.end());
        check(compliant_use_preferred(next,old,reversed_next,reversed_old,true,true),
            "partial measurement scope follows axis mapping rather than any preferred joint");
    }
}
}
int main() {
    try {compliant_protocol_rounding();discovery_budget_priority();drag_use();disturbance_domain();one_sided_mit_response();compliant_use();compliant_partial_scope_group();std::cout<<"drag and compliant purpose validation checks passed\n";return 0;}
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
