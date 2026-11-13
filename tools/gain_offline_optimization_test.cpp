#include "serial_arm/dynamics/gain_offline_optimization.hpp"
#include <iostream>

using namespace serial_arm::servo_tuning;
namespace {
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}

struct Fixture {OfflineGainPath path;OfflineGainModel model;OfflineGainLimits limits;};

Fixture fixture(std::size_t n,double inertia=.1,double duration=1,double period=.005) {
    Fixture f;const auto count=static_cast<Eigen::Index>(n);
    auto zeros=Eigen::VectorXd::Zero(count).eval();
    f.path.period_s=period;f.path.initial_position=zeros;f.path.initial_velocity=zeros;
    f.path.lower=Eigen::VectorXd::Constant(count,-2);f.path.upper=Eigen::VectorXd::Constant(count,2);
    f.limits.lower=f.path.lower;f.limits.upper=f.path.upper;
    f.limits.velocity=Eigen::VectorXd::Constant(count,100);f.limits.effort=Eigen::VectorXd::Constant(count,100);
    f.limits.position_step=Eigen::VectorXd::Constant(count,.0004);
    f.limits.velocity_step=Eigen::VectorXd::Constant(count,.01);f.limits.torque_step=zeros;
    ModelAxisLimits limit;limit.kp_max=500;limit.kd_max=5;limit.effort_budget=100;limit.grid={.01,.001};
    f.limits.axes.assign(n,limit);
    f.model.coulomb=zeros;f.model.viscous=zeros;f.model.bias=zeros;f.model.rotor=zeros;
    f.model.gravity_error=zeros;f.model.available=true;
    for(double time=0;time<=duration+1e-10;time+=period) {
        OfflineGainStep row;row.time_s=time;
        row.reference=Eigen::VectorXd::Constant(count,.05*std::sin(2*pi*time));
        row.reference_velocity=Eigen::VectorXd::Constant(count,.1*pi*std::cos(2*pi*time));
        row.actual=row.reference;row.velocity=row.reference_velocity;
        row.torque=zeros;row.feedforward=zeros;row.gravity=zeros;
        row.mass=Eigen::MatrixXd::Identity(count,count)*inertia;
        row.coriolis=Eigen::MatrixXd::Zero(count,count);row.gravity_gradient=row.coriolis;
        f.path.steps.push_back(row);
    }
    return f;
}

OfflineGainPath quiet(OfflineGainPath path,double displacement=.01) {
    path.initial_position.setConstant(displacement);path.initial_velocity.setZero();
    for(auto& row:path.steps){row.reference.setZero();row.reference_velocity.setZero();row.actual.setZero();row.velocity.setZero();}
    return path;
}

double difference(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
    require(a.rows()==b.rows() && a.cols()==b.cols(),"simulation dimensions must match");
    return (a-b).cwiseAbs().maxCoeff();
}

void trust_region_and_search() {
    for(std::size_t n:{1,3,7}) {
        auto f=fixture(n);Gains seed;seed.kp.assign(n,2);seed.kd.assign(n,.3);
        const auto region=offline_gain_local_trust_region(OfflineGainUse::Tracking,seed,f.limits);
        for(std::size_t i=0;i<n;++i) {
            require(region.lower.kp[i]==1.5 && region.upper.kp[i]==3 &&
                region.lower.kd[i]==.225 && region.upper.kd[i]==.375,
                "trust bounds must remain anchored to the original complete seed for every mapped axis");
        }
        f.model.bias.setConstant(.02);
        const auto optimized=optimize_offline_gain_use(OfflineGainUse::Tracking,f.path,f.path,f.model,seed,{},
            f.limits,Requirements{},Eigen::VectorXd::Zero(n),.5);
        require(optimized.evaluations>1 && optimized.gains.kp.size()==n,"local search must execute actual closed-loop comparisons");
        require(optimized.available && optimized.changed && optimized.selected.cost<optimized.baseline.cost,
            "the trust-region test must exercise retained model improvements rather than only an unchanged seed");
        for(std::size_t i=0;i<n;++i) {
            require(optimized.gains.kp[i]>=region.lower.kp[i] && optimized.gains.kp[i]<=region.upper.kp[i] &&
                optimized.gains.kd[i]>=region.lower.kd[i] && optimized.gains.kd[i]<=region.upper.kd[i],
                "moving iteration centers cannot accumulate beyond the original seed trust region");
            require(optimized.trust_region.upper.kp[i]==3,"the reported search region cannot follow a moving center");
        }
        Gains high;high.kp.assign(n,100);high.kd.assign(n,2);
        const auto ideal=simulate_offline_gain_path(f.path,f.model,high,f.limits);
        require(ideal.complete && ideal.safe,"the declared ideal plant must reproduce the formerly admissible large-gain extrapolation");
        require(optimized.gains.kp[0]<high.kp[0],"an ideally feasible large gain must still remain outside an unmeasured local search");
        const auto drag_region=offline_gain_local_trust_region(OfflineGainUse::Drag,seed,f.limits);
        for(std::size_t i=0;i<n;++i)require(drag_region.lower.kp[i]==0 && drag_region.upper.kp[i]==0,
            "all drag stiffness channels must remain exactly zero for arbitrary mappings");
    }
    auto f=fixture(1);const double p=f.limits.axes[0].grid.kp_step,d=f.limits.axes[0].grid.kd_step;
    const auto adjacent=offline_gain_local_trust_region(OfflineGainUse::Tracking,{{p},{d}},f.limits);
    require(adjacent.lower.kp[0]==0 && adjacent.upper.kp[0]==2*p && adjacent.lower.kd[0]==0 && adjacent.upper.kd[0]==2*d,
        "a sub-code percentage interval may gain only one adjacent executable code");
    const auto zero=offline_gain_local_trust_region(OfflineGainUse::Drag,{{0},{0}},f.limits);
    require(zero.lower.kp[0]==0 && zero.upper.kp[0]==0 && zero.lower.kd[0]==0 && zero.upper.kd[0]==d,
        "zero damping may explore one known code while drag stiffness remains zero");
    const auto capped=offline_gain_local_trust_region(OfflineGainUse::Tracking,{{500},{5}},f.limits);
    require(capped.upper.kp[0]==500 && capped.upper.kd[0]==5,"a numerical trust region cannot enlarge protocol capabilities");
    const auto drag=optimize_offline_gain_use(OfflineGainUse::Drag,quiet(f.path,0),quiet(f.path,0),f.model,{{0},{.3}}, {},
        f.limits,Requirements{},Eigen::VectorXd::Zero(1),.15);
    require(drag.gains.kp[0]==0,"the actual drag optimizer must preserve zero stiffness");
    const auto stopped=optimize_offline_gain_use(OfflineGainUse::Tracking,f.path,f.path,f.model,{{2},{.3}}, {},
        f.limits,Requirements{},Eigen::VectorXd::Zero(1),1,[]{return false;});
    require(!stopped.available && !stopped.selected.complete,"cancellation cannot manufacture a complete model comparison");
    std::cout<<"PASS immutable seed trust region, former large-gain extrapolation, adjacent codes, 1/3/7 axes, drag and cancellation\n";
}

void inner_feedback_scenarios() {
    auto f=fixture(1,.01,.2,.001);auto path=quiet(f.path);const Gains gains{{100},{.5}};
    const auto run=[&](const OfflineGainSimulationOptions& options) {
        return simulate_offline_gain_path(path,f.model,gains,f.limits,{},.0001,.0001,1,10,{}, {},options);
    };
    const auto ideal=run({});require(ideal.complete,"the ideal comparison must complete");
    OfflineGainSimulationOptions sampled;sampled.inner_pd_period_s=.01;
    const auto held=run(sampled);
    require(held.complete && held.feedback.block(1,0,9,1).maxCoeff()==held.feedback.block(1,0,9,1).minCoeff(),
        "an explicit inner PD period must hold actual feedback torque between update instants");
    require(difference(ideal.position,held.position)>1e-5,"inner ZOH must change the physical closed-loop response");
    OfflineGainSimulationOptions delayed_position;delayed_position.position_feedback_delay_s=.005;
    const auto pq=run(delayed_position);
    require(pq.complete && difference(pq.feedback,ideal.feedback)>.001,
        "position feedback delay must enter the P term, including with zero gravity and no feedforward");
    OfflineGainSimulationOptions delayed_velocity;delayed_velocity.velocity_feedback_delay_s=.005;
    const auto pv=run(delayed_velocity);
    require(pv.complete && difference(pv.feedback,ideal.feedback)>.001,
        "velocity feedback delay must independently enter the D term");
    f.limits.position_step.setConstant(.001);path=quiet(f.path,0);
    for(auto& row:path.steps)row.reference.setConstant(.0001);
    OfflineGainSimulationOptions quantized;quantized.quantize_position_feedback=true;
    const auto continuous_position=run({}),quantized_position=run(quantized);
    require(quantized_position.complete && difference(quantized_position.feedback,continuous_position.feedback)>1e-5,
        "position encoder quantization must affect actual PD feedback rather than just reported curves");
    path=quiet(f.path,0);path.initial_velocity.setConstant(.004);
    quantized={};quantized.quantize_velocity_feedback=true;
    const auto continuous_velocity=run({}),quantized_velocity=run(quantized);
    require(quantized_velocity.complete && difference(quantized_velocity.feedback,continuous_velocity.feedback)>1e-5,
        "velocity quantization must reach the damping feedback input");
    quantized.position_feedback_delay_s=-1;
    require(!run(quantized).complete,"invalid internal timing cannot become a successful simulation");
    std::cout<<"PASS actual inner PD ZOH, separate position/velocity delays and optional feedback quantization\n";
}

void inertia_uncertainty() {
    for(std::size_t n:{1,3,7}) {
        auto f=fixture(n,.01,.3);f.model.rotor.setConstant(.09);
        Gains gains;gains.kp.assign(n,20);gains.kd.assign(n,.5);
        OfflineGainSimulationOptions with;with.inner_pd_period_s=.001;
        with.position_feedback_delay_s=with.velocity_feedback_delay_s=.001;
        with.quantize_position_feedback=with.quantize_velocity_feedback=true;
        auto without=with;without.include_fitted_rotor_inertia=false;
        const auto added=simulate_offline_gain_path(f.path,f.model,gains,f.limits,{},.003,.001,1,10,{}, {},with);
        const auto urdf=simulate_offline_gain_path(f.path,f.model,gains,f.limits,{},.003,.001,1,10,{}, {},without);
        require(added.complete && urdf.complete && difference(added.position,urdf.position)>.001,
            "URDF-only inertia must really remove the fitted rotor instead of merely scaling their sum");
        const auto a=evaluate_offline_gain_use(OfflineGainUse::Tracking,f.path,f.path,f.model,gains,f.limits,Requirements{},
            Eigen::VectorXd::Zero(n),{},false,{},with);
        const auto b=evaluate_offline_gain_use(OfflineGainUse::Tracking,f.path,f.path,f.model,gains,f.limits,Requirements{},
            Eigen::VectorXd::Zero(n),{},false,{},without);
        const auto both=evaluate_offline_gain_use(OfflineGainUse::Tracking,f.path,f.path,f.model,gains,f.limits,Requirements{},
            Eigen::VectorXd::Zero(n),{},true);
        require(both.simulation_scenarios==2 && both.includes_urdf_only_inertia && !both.stability_identified,
            "uncertainty revalidation must include a separate zero-extra-J case without claiming identified stability");
        require(both.feasible==(a.feasible && b.feasible) && std::abs(both.cost-std::max(a.cost,b.cost))<1e-9,
            "a failed or worse URDF-only case cannot be hidden by a successful added-inertia case");
        for(std::size_t i=0;i<n;++i)require(std::abs(both.peak_error_rad[i]-std::max(a.peak_error_rad[i],b.peak_error_rad[i]))<1e-12,
            "all mapped axes must report the actual worse simulated error across inertia scenarios");
    }
    std::cout<<"PASS explicit zero-extra-inertia revalidation and honest simulation-only evidence across 1/3/7 axes\n";
}

void fit_evidence() {
    auto f=fixture(1);auto stationary=quiet(f.path,0);const Gains gains{{2},{.3}};
    const auto none=fit_offline_gain_model(stationary,gains,f.limits,1);
    require(!none.available && none.axis_fitted_samples.size()==1 && none.axis_fitted_samples[0]==0 &&
        none.reason.find("insufficient_dynamic")!=std::string::npos,
        "a finite zero-parameter stationary replay must not count as a fitted dynamic model");
    auto constant=stationary;
    for(auto& row:constant.steps){row.velocity.setConstant(.1);row.actual.setConstant(.1*row.time_s);}
    require(!fit_offline_gain_model(constant,gains,f.limits,1).available,
        "many constant-speed samples without dynamic variation cannot identify a dynamic model");
    auto missing=f.path;for(auto& row:missing.steps)row.torque.resize(0);
    require(!fit_offline_gain_model(missing,gains,f.limits,1).available,"missing actual torque cannot become default successful identification");
    for(auto& row:f.path.steps) {
        const auto acceleration=Eigen::VectorXd::Constant(1,-.2*pi*pi*std::sin(2*pi*row.time_s)).eval();
        row.measured_inverse_dynamics=row.mass*acceleration;
        row.torque=row.measured_inverse_dynamics;row.measured_gravity=Eigen::VectorXd::Zero(1);
    }
    const auto fitted=fit_offline_gain_model(f.path,gains,f.limits,2);
    require(fitted.available && fitted.axis_fitted_samples[0]>=30 && !fitted.stability_identified &&
        fitted.reason=="same_record_low_frequency_prediction_fit_only_no_identified_stability",
        "a finite dynamic fit must report its limited prediction scope without upgrading it to measured stability");
    std::cout<<"PASS insufficient dynamic/torque evidence and same-record fit scope\n";
}
}

int main() {
    try {trust_region_and_search();inner_feedback_scenarios();inertia_uncertainty();fit_evidence();}
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
