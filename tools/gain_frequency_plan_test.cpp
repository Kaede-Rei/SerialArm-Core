// Probe scheduling only: no hardware, historical gains or robot identity.
#include "serial_arm/dynamics/gain_frequency_plan.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
constexpr double pi=3.14159265358979323846;
void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
void close(double actual,double expected,const char* message,double tolerance=1e-10) {
    check(std::abs(actual-expected)<=tolerance*std::max({1.,std::abs(actual),std::abs(expected)}),message);
}
MitFrequencyPlanRequest ordinary() {return {.1,10,.4,.005,.005,60,.9};}

void feasible_plan(const MitFrequencyPlanRequest& request,const MitFrequencyPlan& plan) {
    check(plan.valid && plan.eligible,"usable four-controller measurement was rejected");
    check(plan.total_cost_s<=request.measurement_budget_s+1e-10,
        "probe cost exceeded the caller's existing remaining budget");
    close(plan.total_cost_s,4*(plan.probe_duration_s+request.per_controller_overhead_s),
        "all four controller windows and their transition/baseline/tail costs must be reserved");
    const auto tones=plan.frequencies_hz;
    check(tones[0]>0 && tones[0]<tones[1] && tones[1]<tones[2],"planned input frequencies are not positive and ordered");
    close(tones[0],.5*tones[1],"low tone ratio changed");
    close(tones[2],2*tones[1],"high tone ratio changed");
    check(tones[2]*request.feedback_period_s<=.1+1e-12,
        "planned high tone is unresolved at the measured fresh-feedback cadence");
    if(request.feedback_age_s>0)check(tones[2]*request.feedback_age_s<=.125+1e-12,
        "planned high tone exceeded its explicit measured feedback-age phase allowance");
    const double usable=plan.probe_duration_s-1.4-request.feedback_age_s;
    check(usable*tones[0]>=3-1e-10,"smooth edges or feedback age consumed the low-frequency cycle requirement");
    check(usable/request.feedback_period_s>=160-1e-8,
        "planner ignored the existing estimator's selected fresh sample requirement");
    check(usable>=128*request.feedback_period_s+request.feedback_age_s-1e-10,
        "both independent fit halves must contain sufficient fresh samples");
    close(plan.probe_duration_s/request.feedback_period_s,
        std::round(plan.probe_duration_s/request.feedback_period_s),"probe duration does not align to measured feedback cadence");
}

void rigid_body_prior() {
    auto request=ordinary();auto plan=plan_mit_frequency_experiment(request);
    feasible_plan(request,plan);
    check(plan.prior_crossover_covered && plan.missing_evidence.empty(),"ordinary prior should be covered without invented missing evidence");
    const double w=2*pi*plan.rigid_body_crossover_hz;
    close(std::hypot(request.actual_kp,request.actual_kd*w)/(request.inertia_kg_m2*w*w),1,
        "probe prior is not the actual rigid-body PD unity-magnitude frequency");
    auto no_damping=request;no_damping.actual_kd=0;
    const auto proportional=plan_mit_frequency_experiment(no_damping);
    close(proportional.rigid_body_crossover_hz,std::sqrt(request.actual_kp/request.inertia_kg_m2)/(2*pi),
        "P-only rigid-body crossover formula is incorrect");
    auto inertia=request;inertia.inertia_kg_m2*=4;
    check(plan_mit_frequency_experiment(inertia).rigid_body_crossover_hz<plan.rigid_body_crossover_hz,
        "different URDF inertia did not alter the probe prior");
    auto damping=request;damping.actual_kd*=3;
    check(plan_mit_frequency_experiment(damping).rigid_body_crossover_hz>plan.rigid_body_crossover_hz,
        "actual current controller damping did not enter the PD crossover equation");
    auto scaled=request;scaled.inertia_kg_m2*=7;scaled.actual_kp*=7;scaled.actual_kd*=7;
    const auto invariant=plan_mit_frequency_experiment(scaled);
    close(invariant.rigid_body_crossover_hz,plan.rigid_body_crossover_hz,"mechanically equivalent scaling changed the probe prior");
    close(invariant.probe_duration_s,plan.probe_duration_s,"mechanically equivalent scaling changed experiment cost");
}

void generic_axis_mapping() {
    // No axis name/index/count is part of this API. Independent per-axis
    // inputs must produce the same plan in any permutation or arm size.
    for(const unsigned axes:{1U,3U,7U}) {
        std::vector<MitFrequencyPlanRequest> requests;
        std::vector<MitFrequencyPlan> plans;
        for(unsigned j=0;j<axes;++j) {
            auto request=ordinary();request.inertia_kg_m2*=j+1;
            request.actual_kp*=1+.3*j;request.actual_kd*=1+.1*j;
            requests.push_back(request);plans.push_back(plan_mit_frequency_experiment(request));
            feasible_plan(request,plans.back());
        }
        for(unsigned j=0;j<axes;++j) {
            const auto reversed=plan_mit_frequency_experiment(requests[axes-j-1]);
            const auto& original=plans[axes-j-1];
            close(reversed.rigid_body_crossover_hz,original.rigid_body_crossover_hz,"axis ordering changed a model/controller probe prior");
            close(reversed.probe_duration_s,original.probe_duration_s,"axis ordering changed the same experiment cost");
        }
    }
}

void actual_budget_and_partial_observability() {
    auto request=ordinary();const auto ample=plan_mit_frequency_experiment(request);
    request.measurement_budget_s=18;
    const auto short_budget=plan_mit_frequency_experiment(request);feasible_plan(request,short_budget);
    check(short_budget.budget_band_limited && short_budget.prior_crossover_covered &&
        short_budget.selected_center_hz>ample.selected_center_hz && short_budget.probe_duration_s<ample.probe_duration_s,
        "short remaining budget did not schedule shorter observable cycles");
    // A poor rigid-body prior is not a reason to reject real lower-band data:
    // embedded velocity filtering can make D/J very different from bandwidth.
    request=ordinary();request.actual_kd=100;
    const auto above=plan_mit_frequency_experiment(request);feasible_plan(request,above);
    check(!above.prior_crossover_covered && above.feedback_band_limited && !above.missing_evidence.empty(),
        "a high unmeasured prior hid useful partial-band measurement or lost its missing high-frequency evidence");
    request=ordinary();request.actual_kp=.0001;request.actual_kd=0;
    const auto below=plan_mit_frequency_experiment(request);feasible_plan(request,below);
    check(!below.prior_crossover_covered && below.budget_band_limited && !below.missing_evidence.empty(),
        "an unaffordable low-frequency prior must retain useful upper-band data and missing lower-frequency evidence");

    request=ordinary();request.measurement_budget_s=10;
    const auto insufficient=plan_mit_frequency_experiment(request);
    check(insufficient.valid && !insufficient.eligible && !insufficient.missing_evidence.empty(),
        "insufficient full-set budget must remain missing evidence, not a successful experiment");
    request=ordinary();request.feedback_age_s=.05;request.measurement_budget_s=15;
    const auto no_band=plan_mit_frequency_experiment(request);
    check(no_band.valid && !no_band.eligible && no_band.reason=="mit_frequency_plan_budget_and_feedback_band_do_not_overlap",
        "a three-tone band that cannot fit budget and feedback resolution was invented");
}

void feedback_timing_and_invalid_inputs() {
    auto request=ordinary();request.actual_kd=100;
    const auto ordinary_plan=plan_mit_frequency_experiment(request);
    request.feedback_period_s=.01;
    const auto slower=plan_mit_frequency_experiment(request);feasible_plan(request,slower);
    check(slower.observable_high_hz<ordinary_plan.observable_high_hz,"measured slower fresh feedback did not lower the observable cap");
    request.feedback_age_s=.025;
    const auto older=plan_mit_frequency_experiment(request);feasible_plan(request,older);
    check(older.observable_high_hz<slower.observable_high_hz,"measured feedback age did not lower the observable cap");
    for(const double stale:{.060001,.1}) {
        request=ordinary();request.feedback_period_s=stale;
        const auto gap=plan_mit_frequency_experiment(request);
        check(gap.valid && !gap.eligible,"known fresh feedback gaps outside estimator domain were scheduled");
        request=ordinary();request.feedback_age_s=stale;
        const auto age=plan_mit_frequency_experiment(request);
        check(age.valid && !age.eligible,"known stale feedback outside estimator domain was scheduled");
    }
    using Field=double MitFrequencyPlanRequest::*;
    const std::array<Field,3> positive{&MitFrequencyPlanRequest::inertia_kg_m2,&MitFrequencyPlanRequest::actual_kp,&MitFrequencyPlanRequest::feedback_period_s};
    const std::array<Field,4> nonnegative{&MitFrequencyPlanRequest::actual_kd,&MitFrequencyPlanRequest::feedback_age_s,
        &MitFrequencyPlanRequest::measurement_budget_s,&MitFrequencyPlanRequest::per_controller_overhead_s};
    for(const auto field:positive)for(const double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        request=ordinary();request.*field=bad;const auto result=plan_mit_frequency_experiment(request);
        check(!result.valid && !result.eligible && !result.missing_evidence.empty(),"invalid model/position-controller/observed timing input became eligible");
    }
    for(const auto field:nonnegative)for(const double bad:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        request=ordinary();request.*field=bad;const auto result=plan_mit_frequency_experiment(request);
        check(!result.valid && !result.eligible,"invalid damping, feedback age, or existing budget became eligible");
    }
    request=ordinary();request.actual_kd=1e200;
    const auto extreme=plan_mit_frequency_experiment(request);
    check(!extreme.valid && !extreme.eligible,"unresolvable numerical prior must not become a fabricated valid plan");
    request=ordinary();request.measurement_budget_s=0;
    const auto zero_budget=plan_mit_frequency_experiment(request);
    check(zero_budget.valid && !zero_budget.eligible,"zero budget must not start an experiment or invalidate otherwise valid mechanical inputs");
}

MitFrequencyTorquePlanRequest ordinary_torque() {
    MitFrequencyTorquePlanRequest request;
    request.inertia_kg_m2=.2;request.actual_kp={4,2,4,2};request.actual_kd={.3,.3,.9,.9};
    request.frequencies_hz={.5,1,2};request.encoder_step_rad=1e-5;request.noise_rms_rad=1e-6;
    request.negative_room_rad=request.positive_room_rad=.1;request.effort_allowance_nm=1;
    request.torque_quantum_nm=.0001;return request;
}
void feasible_torque(const MitFrequencyTorquePlanRequest& request,const MitFrequencyTorquePlan& plan) {
    check(plan.valid && plan.eligible,"signal-resolved bounded AC input was rejected");
    check(plan.initial_amplitude_nm>0 && plan.initial_amplitude_nm<=plan.maximum_amplitude_nm &&
          plan.maximum_amplitude_nm<=request.effort_allowance_nm,"input exceeded existing effort or dynamic position envelope");
    for(std::size_t k=0;k<4;++k) {
        double sum=0;
        for(std::size_t j=0;j<3;++j) {
            const double predicted=plan.predicted_tone_position_amplitude_rad[k][j];sum+=predicted;
            check(predicted+1e-12>=plan.position_target_rad,"one controller/tone was ignored by the common input SNR target");
            close(predicted,plan.initial_amplitude_nm*plan.predicted_tone_position_per_nm[k][j],
                "reported tone prediction does not use the actual planned input");
            if(request.torque_quantum_nm>0)check(plan.initial_amplitude_nm*MitFrequencyTorquePolicy::tone_weights[j]+1e-12>=
                2*request.torque_quantum_nm,"planned instrument tone is below its known protocol code target");
        }
        close(sum,plan.predicted_peak_excursion_rad[k],"complete three-tone peak bound omitted a component");
        check(sum<=plan.maximum_predicted_excursion_rad+1e-12,"predicted multisine left reserved actual two-sided room");
    }
    close(plan.maximum_predicted_excursion_rad,.5*std::min(request.negative_room_rad,request.positive_room_rad),
        "input planning invented room beyond the actual demonstrated two-sided domain");
    // Existing first-controller supplement can use the remaining amplitude
    // headroom without crossing a separate static P*room gate.
    check(std::min(2*plan.initial_amplitude_nm,plan.maximum_amplitude_nm)<=plan.maximum_amplitude_nm,
        "bounded input supplement exceeded its computed dynamic envelope");
}
void dynamic_input_scale_noise_and_actual_controller_group() {
    auto request=ordinary_torque();const auto plan=plan_mit_frequency_torque_experiment(request);feasible_torque(request,plan);
    for(const unsigned axes:{1U,3U,7U})for(unsigned axis=0;axis<axes;++axis) {
        auto scaled=request;const double factor=axis+1;
        scaled.inertia_kg_m2*=factor;scaled.effort_allowance_nm*=factor;scaled.torque_quantum_nm*=factor;
        for(auto& p:scaled.actual_kp)p*=factor;
        for(auto& d:scaled.actual_kd)d*=factor;
        const auto result=plan_mit_frequency_torque_experiment(scaled);feasible_torque(scaled,result);
        close(result.initial_amplitude_nm,factor*plan.initial_amplitude_nm,"input planning depends on a joint count/index instead of inertia/controller scaling");
        close(result.maximum_amplitude_nm,factor*plan.maximum_amplitude_nm,"dynamic room bound does not scale with physically equivalent parameters");
    }
    auto permuted=request;std::reverse(permuted.actual_kp.begin(),permuted.actual_kp.end());std::reverse(permuted.actual_kd.begin(),permuted.actual_kd.end());
    const auto order=plan_mit_frequency_torque_experiment(permuted);
    close(order.initial_amplitude_nm,plan.initial_amplitude_nm,"ordering four actual controllers changed their common instrument");
    close(order.maximum_amplitude_nm,plan.maximum_amplitude_nm,"controller ordering changed the worst-case displacement envelope");
    auto noisy=request;noisy.noise_rms_rad=.0002;
    check(plan_mit_frequency_torque_experiment(noisy).minimum_required_amplitude_nm>plan.minimum_required_amplitude_nm,
        "measured static noise did not affect required input");
    auto clipped=request;clipped.actual_kd={5,5,5,5};
    auto unencoded=clipped;unencoded.actual_kd={15,15,15,15};
    check(plan_mit_frequency_torque_experiment(clipped).minimum_required_amplitude_nm<
          plan_mit_frequency_torque_experiment(unencoded).minimum_required_amplitude_nm,
        "input plan substituted requested damping for actual protocol-clipped controller values");
    auto narrow=request;narrow.negative_room_rad=1e-6;
    const auto too_small=plan_mit_frequency_torque_experiment(narrow);
    check(too_small.valid && !too_small.eligible && too_small.initial_amplitude_nm==0 &&
          too_small.minimum_required_amplitude_nm>too_small.maximum_amplitude_nm,
        "insufficient demonstrated domain was converted to a resolved measurement");
    auto weak=request;weak.effort_allowance_nm=1e-6;
    check(!plan_mit_frequency_torque_experiment(weak).eligible,"existing effort constraints were enlarged for signal resolution");
    for(const double bad:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=request;invalid.noise_rms_rad=bad;
        check(!plan_mit_frequency_torque_experiment(invalid).valid,"invalid measured noise became a valid input plan");
    }
    auto bad_controller=request;bad_controller.actual_kp[3]=0;
    check(!plan_mit_frequency_torque_experiment(bad_controller).valid,"missing fourth actual position controller was filled with defaults");
    auto bad_tones=request;bad_tones.frequencies_hz={1,2,2};
    check(!plan_mit_frequency_torque_experiment(bad_tones).valid,"inseparable input frequencies became a valid experiment");
}

MitFrequencyCoDesignRequest recorded_model_counterexample() {
    // Fixed read-only 1794494177155 model/controller/resolution fixture. There
    // is no joint name, robot identity or residual-dependent rule in the API.
    MitFrequencyCoDesignRequest request;
    request.frequency={.0011237710292219601,.7326007326007326,.04151404151404151,.00507,.00968,80,1.6};
    auto& t=request.torque;t.inertia_kg_m2=request.frequency.inertia_kg_m2;
    const double P=request.frequency.actual_kp,D=request.frequency.actual_kd,step=.001221001221001221;
    t.actual_kp={P,.5*P,P,.5*P};t.actual_kd={D,D,3*D+step,3*D+step};
    t.encoder_step_rad=.0003814697265625;t.negative_room_rad=t.positive_room_rad=.04;t.effort_allowance_nm=.25;
    return request;
}
void joint_frequency_and_input_plan() {
    auto request=recorded_model_counterexample();
    const auto prior=plan_mit_frequency_experiment(request.frequency);
    request.torque.frequencies_hz=prior.frequencies_hz;
    const auto unresolved=plan_mit_frequency_torque_experiment(request.torque);
    check(unresolved.valid && !unresolved.eligible &&
        unresolved.minimum_required_amplitude_nm>2*unresolved.maximum_amplitude_nm,
        "recorded low-P/high-D prior-band counterexample was not detected");
    const auto combined=plan_mit_frequency_experiment_co_design(request);
    check(combined.valid && combined.eligible && combined.numeric_bands_examined>1 &&
          combined.frequency.selected_center_hz<prior.selected_center_hz && !combined.frequency.prior_crossover_covered,
        "pure numerical co-design did not select an actually resolvable lower partial band");
    feasible_plan(request.frequency,combined.frequency);
    request.torque.frequencies_hz=combined.frequency.frequencies_hz;feasible_torque(request.torque,combined.torque);
    check(combined.frequency.total_cost_s<=request.frequency.measurement_budget_s,
        "lowering the measurement band silently enlarged the existing four-window wall-clock budget");
    // Same prior band becomes feasible only when genuine room is supplied;
    // its amplitude can exceed the old static .4*Pmin*room restriction.
    auto more_room=request.torque;more_room.negative_room_rad=more_room.positive_room_rad=.16;
    more_room.frequencies_hz=prior.frequencies_hz;
    const auto dynamic=plan_mit_frequency_torque_experiment(more_room);feasible_torque(more_room,dynamic);
    check(dynamic.initial_amplitude_nm>.4*.5*request.frequency.actual_kp*.16,
        "finite-frequency input remains locked to a static P*room torque cap");
    auto short_budget=request;short_budget.frequency.measurement_budget_s=30;
    const auto insufficient=plan_mit_frequency_experiment_co_design(short_budget);
    check(insufficient.valid && !insufficient.eligible && !insufficient.missing_evidence.empty(),
        "low-frequency cycles outside the original complete-set budget became a success");
    auto quantized=request;quantized.torque.torque_quantum_nm=20./4095;
    check(!plan_mit_frequency_experiment_co_design(quantized).eligible,
        "known command quantization was ignored when the demonstrated room cannot resolve the required tones");
    auto mismapped=request;mismapped.torque.inertia_kg_m2*=2;
    check(!plan_mit_frequency_experiment_co_design(mismapped).valid,
        "frequency/input plans with different physical/controller mappings were silently combined");
}
}
int main() {
    try {
        rigid_body_prior();generic_axis_mapping();actual_budget_and_partial_observability();feedback_timing_and_invalid_inputs();
        dynamic_input_scale_noise_and_actual_controller_group();joint_frequency_and_input_plan();
        std::cout<<"MIT frequency/input co-design: current four-PD rigid-body prior, measured noise/cadence, dynamic room, actual effort, pure numerical partial-band selection and original four-window budget PASS\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
