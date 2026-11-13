// Whole taught-path polls against explicit sampled MIT plants. These plant
// assumptions test generic behavior; they are not estimates of a user's motor.
#include "serial_arm/dynamics/gain_trajectory_tuning.hpp"
#include <iostream>
#include <numeric>
using namespace serial_arm::servo_tuning;
namespace {
void check(bool condition,const char* reason) {if(!condition)throw std::runtime_error(reason);}
std::vector<Sample> simulate(double P,double D,double inertia) {
    constexpr double dt=.001,end=12,tail=1.5,pc=25.0/65536;
    double q=0,v=0,command=0;std::vector<double> torque_delay(5,0);std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>((end+tail)/dt);++k) {
        const double time=k*dt,phase=2*pi*std::min(time,end)/end;
        const double r=.25*(1-std::cos(phase));
        const double rv=time<end?.25*2*pi/end*std::sin(phase):0;
        const double measured=std::round(q/pc)*pc;
        if(k%5==0) {
            command=P*(r-measured)+D*(rv-v);
            Sample s;s.time_s=s.command_time_s=time;s.feedback_time_s=time-.004;
            s.reference=r;s.reference_velocity=rv;s.actual=measured;s.velocity=v;
            s.new_measurement=true;s.feedback_sequence=k/5+1;s.actual_gains_known=true;
            s.actual_kp=P;s.actual_kd=D;rows.push_back(s);
        }
        const double applied=torque_delay[k%torque_delay.size()];torque_delay[k%torque_delay.size()]=command;
        // A smooth Coulomb transition keeps the deterministic integration
        // well defined through zero speed; its scale follows each inertia.
        const double friction=.8*inertia*std::tanh(v/.01)+.25*inertia*v;
        v+=(applied-friction)/inertia*dt;q+=v*dt;
    }
    return rows;
}
}
int main() {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const double pcode=500.0/4095,dcode=5.0/4095;
    for(std::size_t n:{1,3,7}) {
        Gains gains;std::vector<ModelAxisLimits> limits;std::vector<TrajectoryAssessment> measured;
        for(std::size_t i=0;i<n;++i) {
            const double inertia=.02+.017*i;
            gains.kp.push_back(quantize_gain(30*inertia,pcode,500));
            gains.kd.push_back(quantize_gain(2*inertia,dcode,5));
            ModelAxisLimits axis;axis.kp_max=500;axis.kd_max=5;axis.effort_budget=5;
            axis.grid={pcode,dcode};limits.push_back(axis);
            measured.push_back(assess_trajectory(simulate(gains.kp.back(),gains.kd.back(),inertia),req,noise,12));
        }
        const auto proposal=budgeted_trajectory_local_candidate(gains,limits,measured,req);
        check(proposal.changed,"complete taught measurements must propose an executable group");
        std::vector<TrajectoryAssessment> compared;
        for(std::size_t i=0;i<n;++i) {
            check(proposal.gains.kp[i]>gains.kp[i] && proposal.gains.kp[i]<=1.5*gains.kp[i] && proposal.gains.kd[i]==gains.kd[i],
                "every resolved deficient axis must receive a bounded P-only trial, not only the worst axis");
            compared.push_back(assess_trajectory(simulate(proposal.gains.kp[i],proposal.gains.kd[i],.02+.017*i),req,noise,12));
        }
        check(trajectory_trial_preferred(proposal.gains,compared,gains,measured,gains,req),
            "a physically simulated complete-group improvement must be retained as a whole");
        auto accepted_endpoint=measured;
        for(auto& a:accepted_endpoint)a.passed=true;
        Requirements relaxed=req;relaxed.peak_error_rad=1;
        const auto by_endpoint=budgeted_trajectory_local_candidate(gains,limits,accepted_endpoint,relaxed);
        check(by_endpoint.gains.kp==proposal.gains.kp && by_endpoint.gains.kd==proposal.gains.kd,
            "endpoint pass and a different final tolerance must not suppress dynamic tracking optimization");
        auto reversed=accepted_endpoint;auto reverse_gains=gains;auto reverse_limits=limits;
        std::reverse(reversed.begin(),reversed.end());std::reverse(reverse_gains.kp.begin(),reverse_gains.kp.end());
        std::reverse(reverse_gains.kd.begin(),reverse_gains.kd.end());std::reverse(reverse_limits.begin(),reverse_limits.end());
        const auto permutation=budgeted_trajectory_local_candidate(reverse_gains,reverse_limits,reversed,req);
        for(std::size_t i=0;i<n;++i)check(permutation.gains.kp[i]==proposal.gains.kp[n-1-i] && permutation.gains.kd[i]==proposal.gains.kd[n-1-i],
            "axis permutation must only permute the candidate");
        auto worse_tail=compared;
        worse_tail.back().metrics.tail_error_rad=measured.back().metrics.tail_error_rad+.03;
        check(!trajectory_trial_preferred(proposal.gains,worse_tail,gains,measured,gains,req),
            "dynamic improvement must not hide a worsened measured final target");
        auto unstable=compared;unstable.back().metrics.oscillating=true;
        check(!trajectory_trial_preferred(proposal.gains,unstable,gains,measured,gains,req),
            "new sustained oscillation must reject the whole candidate");
        auto damaged=measured;damaged.front().feedback_evidence.limited=true;
        const auto partial=budgeted_trajectory_local_candidate(gains,limits,damaged,req);
        check(partial.gains.kp.front()==gains.kp.front() && partial.gains.kd.front()==gains.kd.front(),
            "a limited axis stays on its measured gains while other complete axes remain eligible");
    }
    // An aligned delayed path crosses its reference at a turn; that alone is
    // not a sustained oscillation and must not reverse the supported P trial.
    Gains one{{10},{1}};ModelAxisLimits axis;axis.kp_max=500;axis.kd_max=5;axis.effort_budget=10;
    auto a=assess_trajectory(simulate(10,1,.1),req,noise,12);
    a.reference_coverage=a.responsive=true;a.passed=true;
    auto& t=a.tuning;t.valid=true;t.samples=1000;t.position_floor_rad=.001;t.reference_rms_rad=.2;
    t.centered_rms_rad=.03;t.in_phase_gain=.95;t.lag_s=.12;
    t.amplitude_error_rms_rad=.01;t.velocity_error_rms_rad=.02;
    a.dynamic_overshoot_rad=.05;a.metrics.oscillating=false;a.feedback_evidence.uncommanded_reversals=false;
    a.feedback_evidence.velocity_noise_risk=false;a.metrics.peak_error=a.feedback_evidence.peak_error_rad=.1;
    a.feedback_evidence.velocity_error_peak_rad_s=.2;
    const auto p=budgeted_trajectory_local_candidate(one,{axis},{a},req,0);
    const auto d=budgeted_trajectory_local_candidate(one,{axis},{a},req,1);
    check(p.gains.kp[0]>10 && p.gains.kd[0]==1,"a turn lag crossing must not be labeled oscillation and reduce P");
    check(d.gains.kp==one.kp && d.gains.kd[0]<1 && d.gains.kd[0]>=.9,
        "the alternative lag poll must test a separate small D reduction");
    check(d.reasons[0].find("not_identified_damping_cause")!=std::string::npos,
        "lower D is a measured alternative, not a claimed diagnosis of excessive damping");
    const auto smaller=budgeted_trajectory_local_candidate(one,{axis},{a},req,1,.5);
    check(smaller.gains.kd[0]>d.gains.kd[0] && smaller.gains.kd[0]<1,
        "a contracted continuous poll must reduce its step without changing direction");
    std::cout<<"PASS complete-group taught-path polls: 1/3/7 inertias, permutation, actual simulated improvement, independent endpoint scoring, lag/overshoot separation, alternative D, whole-group tail and oscillation rejection\n";
}
