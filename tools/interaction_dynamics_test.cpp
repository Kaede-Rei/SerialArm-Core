#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/runtime/interaction_controller.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace serial_arm;

int main() {
    GeneralizedMomentumObserver observer;
    assert(observer.configure({1,{25},{0}}));
    GeneralizedMomentumInput in{{-0.7},{0},{0},{{1}},{0},0.005,{0.7}};
    for(int i=0;i<1000;++i) { auto out=observer.update(in);assert(out);assert(std::abs(out->tau_ext_hat[0])<1e-10); }
    in.measured_torque[0]=-1.0;
    double ext=0;
    for(int i=0;i<1000;++i) {auto out=observer.update(in);assert(out);ext=out->tau_ext_hat[0];}
    assert(std::abs(ext-0.3)<1e-9);

    JointAdmittanceController admittance;
    assert(admittance.configure({1,{1},{1},{3},{8},{10},{10}}));
    JointAdmittanceInput cmd{{1},0.005,{}, {}, {}, {},true};
    for(int i=0;i<100;++i) assert(admittance.update(cmd));
    cmd.tau_ext_hat[0]=0;
    double final_q=0,final_v=0;
    for(int i=0;i<2000;++i) {auto out=admittance.update(cmd);assert(out);final_q=out->delta_q[0];final_v=out->delta_q_dot[0];}
    assert(final_q>0.05 && std::abs(final_v)<1e-10);

    AdmittanceFrictionCalibrationCfg cfg;
    cfg.joints_count=1;cfg.momentum_gain={25};cfg.min_samples_per_direction=20;
    auto samples=[](bool reverse) {
        std::vector<AdmittanceFrictionSample> result;
        double filtered=0;
        for(int i=0;i<1600;++i) {
            const double v=(reverse?-1:1)*0.22*std::sin(i*0.025);
            const double speed=std::abs(v),x=std::min(speed/0.03,1.0),blend=x*x*(3-2*x);
            const double friction=blend*(v>=0 ? 0.8+0.4*speed : -0.6-0.3*speed);
            const double dt=i==0?0.0:0.005;
            if(dt>0) filtered=(filtered+25*dt*friction)/(1+25*dt);
            // An unavailable aligned acceleration must not enter fitting or validation.
            result.push_back({{v},{0},{i==300 ? 100.0 : filtered},dt,i!=300});
        }
        return result;
    };
    const auto result=calibrate_admittance_friction_cross_validated(samples(false),samples(true),cfg);
    assert(result && result->observable[0] && result->validation_pass[0]);
    assert(std::abs(result->positive_coulomb[0]-0.8)<1e-7);
    assert(std::abs(result->positive_viscous[0]-0.4)<1e-7);
    assert(std::abs(result->negative_coulomb[0]+0.6)<1e-7);
    assert(std::abs(result->negative_viscous[0]+0.3)<1e-7);

    // Sensor reports 1.5 / 2.5 velocity LSB at an unchanged measured pose.
    // These observed stationary values must not turn the fitted Coulomb model on.
    InteractionControllerCfg ic;ic.enabled=true;ic.admittance_enabled=false;ic.momentum={1,{25},{0}};
    ic.external_torque.joints_count=1;ic.external_torque.torque_bias={0};ic.external_torque.torque_threshold={.1};
    auto& friction=ic.external_torque.friction;friction.enabled=true;friction.velocity_transition=.001;
    friction.velocity_resolution={60.0/4095};friction.positive_coulomb={1};friction.negative_coulomb={-1};
    friction.positive_viscous={0};friction.negative_viscous={0};
    InteractionController interaction;assert(interaction.configure(ic));
    JointCtrlCmd nominal;nominal.pos={0};nominal.vel={0};nominal.tor={0};nominal.kp={0};nominal.kd={0};
    const JointVector measured_torque{0},full_model;
    for(double velocity:{-1.5*60/4095,-2.5*60/4095,1.5*60/4095,2.5*60/4095}) {
        for(int sample=0;sample<8;++sample) {
            GeneralizedMomentumInput mi{{0},{0},{0},{{1}},{velocity},.005,{}};
            auto out=interaction.update({measured_torque,full_model,nominal,.005,{0},{0},{0},{0},{velocity},mi,false,true,true,.005,false,false,{0}});
            assert(out && out->state.valid);
            assert(std::abs(out->friction_residual_hat[0])<1e-12);
            assert(std::abs(out->tau_ext_hat[0])<=std::abs(out->friction_compensated[0])+1e-12);
        }
    }
    GeneralizedMomentumInput moving{{0},{0},{0},{{1}},{2.5*60/4095},.005,{}};
    auto real_motion=interaction.update({measured_torque,full_model,nominal,.005,{0},{0},{0},{0},{2.5*60/4095},moving,false,true,true,.005,false,false,{.001}});
    assert(real_motion && real_motion->friction_residual_hat[0]>0);
    ExternalTorqueObserver deadband;ExternalTorqueObserverCfg dc;dc.joints_count=1;dc.torque_threshold={.1};
    assert(deadband.configure(dc));auto estimate=deadband.update(TorqueResidualEstimate{{.3},{}},JointVector{0});
    assert(estimate && std::abs(estimate->friction_compensated[0]-.3)<1e-12 && std::abs(estimate->tau_ext_hat[0]-.2)<1e-12);
    // External-force zeroing must keep an already validated gravity model,
    // even when these poses would be sufficient to fit a different scale.
    std::vector<AdmittanceStaticPoseSamples> static_poses;
    for(double gravity:{-1.0,0.0,1.0}) {
        AdmittanceStaticPoseSamples pose;
        for(int sample=0;sample<8;++sample)pose.samples.push_back({{gravity},{1.6*gravity-.4}});
        static_poses.push_back(std::move(pose));
    }
    AdmittanceStaticCalibrationCfg static_cfg;static_cfg.joints_count=1;static_cfg.fallback_gravity_scale={1.0};
    const auto fitted=calibrate_admittance_static(static_poses,static_cfg);
    assert(fitted && std::abs(fitted->gravity_scale[0]-1.6)<1e-12 && fitted->gravity_scale_observable[0]);
    static_cfg.fit_gravity_scale=false;
    const auto fixed=calibrate_admittance_static(static_poses,static_cfg);
    assert(fixed && fixed->gravity_scale[0]==1.0 && !fixed->gravity_scale_observable[0]);
    assert(std::abs(fixed->torque_bias[0]-.4)<1e-12 && fixed->torque_threshold[0]>.5);
    assert(fixed->lopo_residual_max[0]>.5 && fixed->within_pose_residual_max[0]<1e-12);
    std::cout << "interaction dynamics passed\n";
}
