#include "serial_arm/dynamics/gain_purpose_measurement.hpp"
#include <iostream>
#include <numeric>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
std::vector<Sample> damped_release(double mass,double damping,double input,double step,
    bool locked=false,double release=.8,double duration=2.5) {
    std::vector<Sample> rows;const double start=.2,tau=mass/damping;
    const double powered=release-start,steady=input/damping;
    const double q_release=steady*(powered-tau*(1-std::exp(-powered/tau)));
    const double v_release=steady*(1-std::exp(-powered/tau));
    for(unsigned k=0;k<=static_cast<unsigned>(duration/.005);++k) {
        const double t=.005*k,sensor=t-.004;double q=0,v=0;
        if(sensor>=start && sensor<release){const double u=sensor-start;v=steady*(1-std::exp(-u/tau));q=steady*(u-tau*(1-std::exp(-u/tau)));}
        else if(sensor>=release){const double u=sensor-release;v=v_release*std::exp(-u/tau);q=q_release+v_release*tau*(1-std::exp(-u/tau));}
        Sample row;row.time_s=row.command_time_s=t;row.feedback_time_s=t-.004;
        row.reference_sample_time_known=true;row.reference_sample_time_s=t;
        row.reference=row.reference_velocity=0;row.actual=locked?0:std::round(q/step)*step;
        row.velocity=locked?(k%2?.04:-.04):v;row.new_measurement=true;row.feedback_sequence=k+1;
        row.actual_kp=0;row.actual_kd=damping;row.actual_gains_known=true;
        row.independent_disturbance_known=true;row.independent_disturbance_nm=t>=start && t<release?input:0;
        row.independent_input_planned_known=true;row.planned_independent_disturbance_nm=row.independent_disturbance_nm;
        rows.push_back(row);
    }
    return rows;
}
void complete_fast_stop_and_missing_motion() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto moving=damped_release(.001,.5,.1,noise.encoder_step_rad);
    const auto fast=assess_drag_release(moving,noise,.8);
    check(fast.input_measurement_complete && fast.preparation_motion_observed && !fast.release_motion_observed &&
        fast.fast_stop_observed && fast.measurement_complete && fast.passed,
        "known prepared motion stopping inside one position code must retain a measured fast-stop scope");
    check(fast.decay_time_is_upper_bound && fast.decay_time_upper_bound_s>=.6 &&
        fast.decay_s==fast.decay_time_upper_bound_s && fast.validation_scope.find("upper_bound")!=std::string::npos,
        "sub-resolution stopping must record a finite observed upper bound rather than fabricate a resolved decay time");
    const auto still=damped_release(.001,.5,.01,noise.encoder_step_rad,true);
    const auto unexcited=assess_drag_release(still,noise,.8);
    check(unexcited.input_measurement_complete && !unexcited.preparation_motion_observed &&
        !unexcited.measurement_complete && !unexcited.fast_stop_observed && !unexcited.motion_observed,
        "a stationary trace and velocity-code chatter cannot manufacture prepared or released motion");
    auto retry=next_drag_release_input(unexcited,.01,.2,.005);
    check(retry.retry && retry.next_input_nm==.02,"complete known sub-resolution input must permit a bounded measured-force retry");
    auto previous=unexcited;previous.input_peak_nm=.08;
    retry=next_drag_release_input(previous,.08,.13,.005);
    check(retry.retry && retry.next_input_nm<=.13 && retry.next_input_nm>.08,"input retries must stop at the current impulse and effort domain");
    previous.input_peak_nm=retry.next_input_nm;
    check(!next_drag_release_input(previous,retry.next_input_nm,.13,.005).retry,
        "reaching a quantized input ceiling is insufficient measurement, not an identified hardware limit");
    check(!next_drag_release_input(fast,.1,.3,.005).retry,"established preparation motion must stop further force increases");
    check(!next_drag_release_input(unexcited,.01,.011,.02).retry,"a larger protocol code cannot bypass a smaller current force domain");
    auto drifting=unexcited;drifting.release_motion_observed=true;
    check(!next_drag_release_input(drifting,.01,.2,.005).retry,
        "motion emerging only after unloading must trigger response review rather than stronger preparation force");
    NoiseEstimate fine;fine.encoder_step_rad=1e-5;
    const auto slow=assess_drag_release(damped_release(.1,.05,.02,fine.encoder_step_rad),fine,.8);
    check(slow.input_measurement_complete && slow.release_motion_observed && slow.measurement_complete &&
        !slow.fast_stop_observed && !slow.decay_time_is_upper_bound && !slow.passed,
        "a resolved slow release must retain its real displacement and cannot use the fast-stop exception");
}
void actual_input_and_timing() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const auto original=damped_release(.001,.5,.1,noise.encoder_step_rad);
    auto rows=original;for(auto& row:rows)if(row.time_s<.8)row.actual_kp=1;
    check(!assess_drag_release(rows,noise,.8).measurement_complete,
        "a stiff preparation cannot be relabelled as a constant actual zero-stiffness release");
    rows=original;for(auto& row:rows)if(row.time_s<.8)row.independent_disturbance_known=false;
    check(!assess_drag_release(rows,noise,.8,{},true).input_measurement_complete,
        "a preparation hint cannot replace unknown executed input evidence");
    rows=original;for(auto& row:rows)if(row.time_s<.8)row.feedback_time_s=row.command_time_s-.08;
    check(!assess_drag_release(rows,noise,.8).input_measurement_complete,"stale preparation feedback cannot establish release input timing");
    rows=original;for(auto& row:rows)if(row.time_s>=.2 && row.time_s<.8)row.planned_independent_disturbance_nm=.2;
    check(!assess_drag_release(rows,noise,.8).input_measurement_complete,"clamped actual input is not the recorded preparation plan");
    rows=original;for(auto& row:rows)if(row.time_s>=.8)row.independent_disturbance_nm=.001;
    check(!assess_drag_release(rows,noise,.8).input_measurement_complete,"unremoved input cannot masquerade as damping decay");
    rows=original;for(auto& row:rows)if(row.time_s<.8)row.reference_velocity=.1;
    check(!assess_drag_release(rows,noise,.8).measurement_complete,"a nonzero MIT reference-velocity drive must not be treated as free drag release");
    rows=original;rows[80].limited=true;
    const auto limited=assess_drag_release(rows,noise,.8);
    check(!limited.input_measurement_complete && !next_drag_release_input(limited,.1,.3,.005).retry,
        "input-phase limiting cannot be hidden by an otherwise quiet zero-input tail");
    rows=original;for(auto& row:rows)row.feedback_sequence=1;
    check(!assess_drag_release(rows,noise,.8).measurement_complete,"repeated feedback cannot manufacture a complete release");
    rows=original;auto hidden=rows[80];hidden.new_measurement=false;hidden.actual_kp=1;rows.insert(rows.begin()+80,hidden);
    check(!assess_drag_release(rows,noise,.8).input_measurement_complete,
        "a repeated feedback frame cannot hide an intervening nonzero actual stiffness command");
}
void demonstrated_motion_free_release() {
    NoiseEstimate noise;noise.encoder_step_rad=.0001;
    for(std::size_t n:{1,3,7})for(std::size_t axis=0;axis<n;++axis)for(double sign:{-1.0,1.0}) {
        const double scale=1+.1*axis;
        auto rows=damped_release(.02*scale,.2*scale,sign*.1*scale,noise.encoder_step_rad);
        for(auto& row:rows) {
            row.independent_disturbance_nm=row.planned_independent_disturbance_nm=0;
            if(row.command_time_s<.8) {
                row.actual_kp=10*(.8-row.command_time_s);row.actual_kd=.4*scale;
                row.reference=row.actual+sign*.01;row.reference_velocity=sign*.1;
            }
        }
        check(!assess_drag_release(rows,noise,.8).measurement_complete,
            "a demonstrated preparation must not masquerade as an isolated P0 torque-pulse experiment");
        const auto free=assess_drag_release(rows,noise,.8,{},true);
        check(free.demonstrated_motion_preparation && free.input_measurement_complete &&
            free.preparation_motion_observed && free.release_motion_observed && free.measurement_complete && free.passed &&
            free.validation_scope.find("demonstrated_motion_then_measured_zero_stiffness")!=std::string::npos,
            "known demonstrated motion and actual PD handover must permit measured P0 decay under constant released damping");
        auto bad=rows;for(auto& r:bad)if(r.command_time_s>=.8)r.reference_velocity=.1;
        check(!assess_drag_release(bad,noise,.8,{},true).measurement_complete,
            "MIT reference velocity after release remains driven motion rather than free damping evidence");
        bad=rows;auto hidden=bad[200];hidden.new_measurement=false;hidden.actual_kp=.1;bad.insert(bad.begin()+200,hidden);
        check(!assess_drag_release(bad,noise,.8,{},true).input_measurement_complete,
            "a repeated encoder sample cannot hide nonzero released stiffness during demonstrated-motion release");
        bad=rows;bad[210].actual_kd+=.01;
        check(!assess_drag_release(bad,noise,.8,{},true).input_measurement_complete,
            "changing released damping invalidates a single-group decay measurement");
        bad=rows;for(auto& r:bad)r.actual=r.velocity=0;
        check(!assess_drag_release(bad,noise,.8,{},true).measurement_complete,
            "a preparation flag cannot manufacture motion from a stationary encoder trace");
    }
}

void mapped_count_and_direction() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    for(std::size_t n:{1,3,7}) {
        std::vector<std::size_t> order(n);std::iota(order.begin(),order.end(),0);
        for(unsigned permutation=0;permutation<2;++permutation) {
            for(auto axis:order)for(double direction:{-1.0,1.0}) {
                const double scale=1+.2*axis;
                const auto result=assess_drag_release(damped_release(.001*scale,.5*scale,direction*.1*scale,
                    noise.encoder_step_rad),noise,.8);
                check(result.fast_stop_observed && result.measurement_complete && result.passed,
                    "fast release evidence must follow 1/3/7 mapped axes and either input direction without joint-specific cases");
            }
            std::reverse(order.begin(),order.end());
        }
    }
}
std::vector<Sample> short_pulse(double scale,double direction,double feedback_age,double write_lag,
    double host_lag,double actuator_delay=0) {
    PurposePulsePlan plan;plan.valid=true;plan.direction=static_cast<int>(direction);plan.amplitude_nm=.08*scale;
    plan.baseline_s=.25;plan.rise_s=plan.hold_s=plan.fall_s=.01;plan.release_s=.28;
    constexpr double h=.00025;double q=0,v=0;std::vector<double> positions{0},velocities{0};std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>(2/h);++k) {
        const double t=h*k,u=purpose_pulse_input(t-write_lag-actuator_delay,plan);
        if(k){v+=(u-.5*scale*v)/(.001*scale)*h;q+=v*h;positions.push_back(q);velocities.push_back(v);}
        if(k%20)continue;
        const double feedback=t-feedback_age;
        const auto index=static_cast<std::size_t>(std::max(0.0,std::round(feedback/h)));
        Sample row;row.time_s=t+host_lag;row.command_time_s=t;row.feedback_time_s=feedback;
        row.reference_sample_time_s=t-write_lag;row.reference_sample_time_known=true;
        row.actual=std::round(positions[index]/(25.0/65536))*(25.0/65536);row.velocity=velocities[index];
        row.actual_kp=0;row.actual_kd=.5*scale;row.actual_gains_known=true;
        row.new_measurement=true;row.feedback_sequence=k+1;row.independent_disturbance_known=true;
        row.independent_disturbance_nm=purpose_pulse_input(t-write_lag,plan);
        row.independent_input_planned_known=true;row.planned_independent_disturbance_nm=row.independent_disturbance_nm;
        rows.push_back(row);
    }
    return rows;
}
void matched_feedback_clock_windows() {
    NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    for(std::size_t n:{1,3,7})for(std::size_t axis=0;axis<n;++axis)for(double direction:{-1.0,1.0}) {
        const double scale=1+.2*axis;
        for(double age:{.004,.018})for(double host:{0.0,.007}) {
            const auto rows=short_pulse(scale,direction,age,.035,host);
            const auto result=assess_drag_release(rows,noise,.315+host);
            check(result.feedback_clock_aligned && result.release_timing_known &&
                std::abs(result.command_release_s-.315)<.001 && result.release_speed_window_s>=.05 &&
                result.speed_comparison_valid && result.preparation_motion_observed && result.measurement_complete && result.passed,
                "short sampled pulse movement must survive host/write/feedback offsets and either mapped direction");
            check(result.peak_speed_rad_s<=result.initial_speed_rad_s &&
                !drag_damping_candidate_report({{0},{.5*scale}},{5},{{}},{result}).changed,
                "matching position windows must not turn deceleration into a false damping increase");
        }
    }
    const auto delayed=assess_drag_release(short_pulse(1,1,.004,0,0,.06),noise,.28);
    check(delayed.input_measurement_complete && delayed.measurement_complete && delayed.pulse_response_motion_observed &&
        !delayed.speed_comparison_valid &&
        !drag_damping_candidate_report({{0},{.5}},{5},{{}},{delayed}).changed,
        "actuator response after command unloading cannot manufacture a measured pre-release speed or damping direction");
    auto bad=short_pulse(1,1,.004,0,0);for(auto& r:bad)r.feedback_time_s=std::numeric_limits<double>::quiet_NaN();
    const auto unknown=assess_drag_release(bad,noise,.28);
    check(!unknown.feedback_clock_aligned && !unknown.measurement_complete &&
        !drag_damping_candidate_report({{0},{.5}},{5},{{}},{unknown}).changed,
        "missing feedback time must remain unknown and cannot reverse damping adaptation");
}
}
int main(){try{complete_fast_stop_and_missing_motion();actual_input_and_timing();mapped_count_and_direction();
    matched_feedback_clock_windows();demonstrated_motion_free_release();
    std::cout<<"drag actual-input and release measurement checks passed\n";return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
