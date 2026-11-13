// Nominal damping proposals and their real motion-establishment caller.
// All simulated plants are deterministic; no hardware plugin is loaded.
#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include <iostream>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void close(double a,double b,double eps,const char* message){check(std::abs(a-b)<=eps,message);}
ModelAxisLimits roomy() { return {.04,5,20,100,500,5,{500.0/4095,5.0/4095}}; }

void check_mappings() {
    for(std::size_t n:{1,3,7}) {
        Gains current;std::vector<double> target,inertia,E,V;
        std::vector<ModelAxisLimits> limits(n,roomy());
        for(std::size_t i=0;i<n;++i) {
            const double J=.001+.017*i;inertia.push_back(J);
            current.kp.push_back(quantize_gain(25*J,limits[i].grid.kp_step,500,true));
            current.kd.push_back(quantize_gain(1.4*std::sqrt(J*current.kp.back()),limits[i].grid.kd_step,5,true));
            target.push_back(4*current.kp.back());E.push_back(.01+.001*i);V.push_back(.03+.001*i);
        }
        const auto source=current;
        const auto candidate=nominal_damping_gain_step(current,target,inertia,limits,E,V);
        check(candidate.changed && !candidate.measured,"a model candidate must require actual complete-set measurement");
        check(current.kp==source.kp && current.kd==source.kd,"candidate generation must not adopt or mutate the actual measured group");
        for(std::size_t i=0;i<n;++i) {
            check(candidate.gains.kp[i]>current.kp[i] && candidate.gains.kd[i]==current.kd[i],"stiffness progression must retain independently measured damping");
            check(candidate.gains.kp[i]<=std::max(1.25*current.kp[i],current.kp[i]+limits[i].grid.kp_step),"unmeasured P request exceeded one small step or one code");
            check(candidate.requested_kd[i]>=current.kd[i],"nominal damping demand must remain visible as unmeasured diagnostic context");
            check(candidate.predicted_effort_nm[i]<=limits[i].effort_budget,"paired candidate exceeded actual joint effort envelope");
        }
        auto reverse=[](auto& x){std::reverse(x.begin(),x.end());};
        reverse(current.kp);reverse(current.kd);reverse(target);reverse(inertia);reverse(limits);reverse(E);reverse(V);
        const auto permuted=nominal_damping_gain_step(current,target,inertia,limits,E,V);
        for(std::size_t i=0;i<n;++i) {
            close(permuted.gains.kp[i],candidate.gains.kp[n-1-i],0,"joint order changed the nominal P proposal");
            close(permuted.gains.kd[i],candidate.gains.kd[n-1-i],0,"joint order changed the nominal D proposal");
        }
    }
    std::cout<<"PASS model damping progression for 1, 3 and 7 axes and permuted joint mappings\n";
}

void check_constraints() {
    auto cap=roomy();cap.grid={0,0};cap.kd_max=.5;
    const Gains actual{{4},{.5}};
    const auto candidate=nominal_damping_gain_step(actual,{8},{.2},{cap},{.01},{.1},{.7});
    check(candidate.gains.kp[0]>4 && candidate.gains.kd[0]==.5,"D ceiling must still permit an independently measured P comparison");
    check(candidate.reasons[0].find("kd_protocol_ceiling")!=std::string::npos,"D ceiling must record its protocol source");
    cap.effort_budget=.12;
    const auto tight=nominal_damping_gain_step(actual,{8},{.2},{cap},{.01},{.1},{.7});
    check(tight.gains.kp[0]>=actual.kp[0] && tight.gains.kd[0]>=actual.kd[0] && tight.predicted_effort_nm[0]<=.12,
        "effort restriction must limit a local step without radially cutting the measured group");
    cap.kd_max=5;cap.effort_budget=.15;
    const auto noisy=nominal_damping_gain_step(actual,{8},{.2},{cap},{.01},{.25},{.7});
    check(noisy.gains.kp==actual.kp && noisy.gains.kd==actual.kd && !noisy.changed,
        "an observed velocity/noise envelope without damping headroom must keep the measured group");
    check(noisy.reasons[0].find("diagnostic_only")!=std::string::npos,"a constrained design prior must not masquerade as achieved damping");
    const auto zero=nominal_damping_gain_step({{0},{0}},{8},{.2},{roomy()},{.01},{.1});
    check(!zero.changed,"zero-P source without a nominal design prior must not invent a damping ratio");
    auto bounds=roomy();bounds.grid={0,0};
    const auto explicit_prior=nominal_damping_gain_step({{1},{.01}},{4},{.01},{bounds},{.01},{.1},{.7});
    check(explicit_prior.gains.kd[0]==.01 && explicit_prior.requested_kd[0]>.01,"an explicit nominal prior must not command unmeasured high D");
    const auto reduction=nominal_damping_gain_step({{4},{.5}},{3},{.2},{bounds},{.01},{.1});
    close(reduction.gains.kd[0],.5,0,"reducing stiffness must preserve existing damping");
    bool rejected=false;
    try { (void)nominal_damping_gain_step(actual,{8},{.2,.3},{cap},{.01},{.1}); }
    catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"wrong inertia/joint mapping must not silently truncate the group");
    std::cout<<"PASS protocol ceiling, independent P step, effort/noise constraints and explicit design prior\n";
}

void check_motion_caller() {
    auto limit=roomy();limit.reference_min_offset_rad=-.01;limit.reference_max_offset_rad=.01;
    Requirements req;req.amplitude_rad=.01;req.motion_probe=true;req.frequencies_hz={.25,.5,.857142857};req.probe_duration_s=9.4;
    NoiseEstimate noise;noise.encoder_step_rad=1e-5;noise.velocity_peak=.001;
    const Gains actual{{4},{.28}};
    Metrics metric;metric.samples=metric.measurement_samples=1800;metric.continuous=true;
    metric.peak_error=.01;metric.actual_span=0;metric.applied_input_peak_nm=.04;
    ResponseModel model;model.reason="motion_establishment_only_no_spectral_fit";
    MotionObservation motion;motion.bidirectional=false;motion.response_rms_ratio=0;
    MotionEstablishmentState state;
    const auto proposed=motion_establishment_step(actual,{.01},{limit},{noise},{model},{metric},{motion},{req},req,false,180,&state,{.7});
    check(proposed.retry && proposed.gains.kp[0]>actual.kp[0] && proposed.gains.kd[0]==actual.kd[0],
        "real motion-establishment P escalation must not command unmeasured D growth");
    check(!proposed.established && !proposed.motion_established && !proposed.feedback_fault,
        "an unexecuted paired candidate cannot turn missing movement into established response");
    check(proposed.reasons[0].find("bounded_pd_step")!=std::string::npos,"the motion caller did not expose its model damping basis");
    check(prescribed_pd_input_peak(proposed.plans[0],proposed.gains.kp[0],proposed.gains.kd[0])>
        prescribed_pd_input_peak(req,actual.kp[0],actual.kd[0]),"the new candidate does not offer a useful actual reference drive");
    model.sustained_oscillation=true;
    const auto ring=motion_establishment_step(actual,{.01},{limit},{noise},{model},{metric},{motion},{req},req,false,180);
    check(ring.gains.kp[0]<actual.kp[0] && ring.gains.kd[0]==actual.kd[0],
        "observed ringing must not lower damping together with stiffness");
    check(!ring.established && !ring.motion_established,"ringing feedback cannot become qualified because a reduction was proposed");
    const auto deadline=motion_establishment_step(actual,{.01},{limit},{noise},{model},{metric},{motion},{req},req,false,0);
    check(!deadline.retry,"a damping candidate must obey the original remaining trial budget");
    metric.limited_samples=1;
    const auto limited=motion_establishment_step(actual,{.01},{limit},{noise},{model},{metric},{motion},{req},req,false,180);
    check(limited.feedback_fault && limited.gains.kp==actual.kp && limited.gains.kd==actual.kd,
        "limited feedback cannot justify a stronger candidate or erase the executed group");
    std::cout<<"PASS actual motion-establishment caller, retained measured group, ringing and hard budget\n";
}

struct PlantResult {double overshoot{0},tail_rms{0};};
PlantResult step_response(double J,double P,double D) {
    double q=0,v=0,sum=0;std::size_t count=0;PlantResult out;
    constexpr double dt=.0002;
    for(unsigned k=0;k<10000;++k) {
        const double t=k*dt,target=t<.1?0:.01;
        const double tau=P*(target-q)-D*v;
        v+=tau/J*dt;q+=v*dt;
        if(t>=.1)out.overshoot=std::max(out.overshoot,q-.01);
        if(t>=1){sum+=(q-target)*(q-target);++count;}
    }
    out.tail_rms=std::sqrt(sum/count);return out;
}
void check_reproducible_plant() {
    auto limits=roomy();limits.grid={0,0};
    for(double inertia:{.001,.02,.2}) {
        const Gains original{{25*inertia},{7*inertia}};
        const auto proposal=nominal_damping_gain_step(original,{16*original.kp[0]},{inertia},{limits},{.01},{.1});
        const auto jumped=step_response(inertia,16*original.kp[0],original.kd[0]);
        const auto bounded=step_response(inertia,proposal.gains.kp[0],proposal.gains.kd[0]);
        check(bounded.overshoot<.2*jumped.overshoot,
            "an unmeasured large P jump must be replaced by a small actual comparison");
        const double dt=.005;const auto radius=[&](double P,double D) {
            Eigen::Matrix2d F;F<<1-.5*P/inertia*dt*dt,dt-.5*D/inertia*dt*dt,-P/inertia*dt,1-D/inertia*dt;
            return Eigen::EigenSolver<Eigen::Matrix2d>(F).eigenvalues().cwiseAbs().maxCoeff();
        };
        check(radius(25*inertia,7*inertia)<1 && radius(25*inertia,600*inertia)>1,
            "a high continuous-time D can destabilize an explicitly sampled plant; it must not be treated as verified stability");
    }
    std::cout<<"PASS reproducible inertial MIT plants: small independent feedback comparisons, sampled high-D instability\n";
}
}

void check_bootstrap_coupling_and_timing() {
    auto cap=roomy();cap.grid={0,0};
    ModelResponseDemand demand=recovery_response_demand(3,pi/1800,.7);
    demand.mass_matrix_samples={{{.2,.199,0},{.199,.2,0},{0,0,.001}}};
    const std::vector<ModelAxisLimits> limits(3,cap);
    const auto unknown=bootstrap_model_initialization({{.2,.2,.001}},{0,0,0},limits,demand);
    check(unknown.gains.kd[0]<unknown.unconstrained_kd[0] && unknown.bootstrap_damping_inertia_min[0]<.0021,
        "bootstrap damping must use coupled free inertia instead of allocating D until a minimum zeta passes");
    check(!unknown.measured && unknown.nominal_mode_damping_ratio.size()==3,
        "lower nominal damping remains a diagnostic, not a fabricated measured pass or model failure");
    demand.observation_interval_s=.005;demand.feedback_age_s=.005;
    const auto timed=bootstrap_model_initialization({{.2,.2,.001}},{0,0,0},limits,demand);
    check(timed.modeled_fastest_damping_rate_s_inv<=50*(1+1e-8),
        "bootstrap fastest damping mode must respect its explicitly supplied observation timing budget");
    check(timed.gains.kd[2]==unknown.gains.kd[2],
        "timing restriction in a different coupled block must not shrink an independent wrist damping");
    demand.observation_interval_s=-1;bool rejected=false;
    try{(void)bootstrap_model_initialization({{.2,.2,.001}},{0,0,0},limits,demand);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"invalid observation timing cannot silently become a model prior");
    std::cout<<"PASS coupled free-inertia bootstrap and explicit timing; modal zeta is diagnostic, independent blocks preserved\n";
}

int main() {
    try {check_mappings();check_constraints();check_motion_caller();check_reproducible_plant();check_bootstrap_coupling_and_timing();}
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
