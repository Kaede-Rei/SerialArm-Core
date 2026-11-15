#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/runtime/interaction_controller.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace serial_arm;

namespace {

// Independent synthetic no-contact plant: a cubic Hermite ramp joins a
// zero-friction velocity interval to the known signed Coulomb/viscous law.
// Its half-ramp value is exactly 1/2, which is also checked below at velocities
// absent from the identification and cross-validation trajectories.
double synthetic_friction(double velocity, double resolution) {
    const long double speed=std::abs(velocity);
    const long double end=std::max(0.03,4.0*resolution);
    if(speed<=resolution) return 0.0;
    const long double u=std::min((speed-resolution)/(end-resolution),1.0L);
    const long double blend=std::pow(u,3)-3.0L*std::pow(u,2)*(u-1.0L);
    return static_cast<double>(blend*(velocity>0 ? 0.8L+0.4L*speed : -0.6L-0.3L*speed));
}

std::vector<AdmittanceFrictionSample> synthetic_friction_run(
    double resolution, bool reverse, bool filtered) {
    std::vector<AdmittanceFrictionSample> samples;
    double residual=0.0;
    for(int i=0;i<2400;++i) {
        const double phase=i*(reverse?0.023:0.019)+(reverse?0.21:0.0);
        const double amplitude=reverse?-0.17:0.18;
        const double velocity=amplitude*std::sin(phase);
        const double acceleration=amplitude*(reverse?0.023:0.019)/0.005*std::cos(phase);
        const double friction=synthetic_friction(velocity,resolution);
        const double dt=i==0?0.0:0.005;
        if(filtered) {
            if(dt>0) residual=(residual+25.0*dt*friction)/(1.0+25.0*dt);
        } else residual=friction;
        samples.push_back({{velocity},{acceleration},{residual},dt,true});
    }
    return samples;
}

ExternalTorqueObserver calibrated_friction_observer(
    const AdmittanceFrictionCalibrationResult& fit, double resolution) {
    ExternalTorqueObserverCfg cfg;cfg.joints_count=1;
    cfg.friction.enabled=true;cfg.friction.velocity_transition=0.03;
    cfg.friction.velocity_resolution={resolution};
    cfg.friction.positive_coulomb=fit.positive_coulomb;
    cfg.friction.positive_viscous=fit.positive_viscous;
    cfg.friction.negative_coulomb=fit.negative_coulomb;
    cfg.friction.negative_viscous=fit.negative_viscous;
    ExternalTorqueObserver observer;assert(observer.configure(cfg));
    return observer;
}

void check_resolution_aware_friction_calibration() {
    for(double resolution:{0.0,0.005,0.02}) {
        AdmittanceFrictionCalibrationCfg cfg;cfg.joints_count=1;
        cfg.velocity_resolution={resolution};
        // The default 0.05 rad/s exclusion would hide most of the low-speed
        // regression; these synthetic trajectories deliberately identify it.
        cfg.min_fit_velocity=0.0005;cfg.min_speed_span=0.05;
        const auto reverse=synthetic_friction_run(resolution,true,false);
        const auto forward=synthetic_friction_run(resolution,false,false);
        const auto fit=calibrate_admittance_friction_cross_validated(reverse,forward,cfg);
        assert(fit && fit->observable[0] && fit->validation_pass[0]);
        assert(std::abs(fit->positive_coulomb[0]-0.8)<1e-8);
        assert(std::abs(fit->positive_viscous[0]-0.4)<1e-8);
        assert(std::abs(fit->negative_coulomb[0]+0.6)<1e-8);
        assert(std::abs(fit->negative_viscous[0]+0.3)<1e-8);
        assert(fit->cross_residual_rms_after[0]<1e-9);

        auto observer=calibrated_friction_observer(*fit,resolution);
        const double end=std::max(0.03,4.0*resolution);
        for(double velocity:{-0.125,-end,-(resolution+end)/2,-resolution,
                             -resolution/2,0.0,resolution/2,resolution,
                             (resolution+end)/2,end,0.125}) {
            const double expected=synthetic_friction(velocity,resolution);
            const auto estimate=observer.update(TorqueResidualEstimate{{expected},{}},{velocity});
            assert(estimate && std::abs(estimate->friction_residual_hat[0]-expected)<1e-9);
            assert(std::abs(estimate->tau_ext_hat[0])<1e-9);
        }
        const double middle=(resolution+end)/2;
        assert(std::abs(synthetic_friction(middle,resolution)-0.5*(0.8+0.4*middle))<1e-12);
        assert(std::abs(synthetic_friction(-middle,resolution)-0.5*(-0.6-0.3*middle))<1e-12);

        if(resolution>0.0) {
            // Omitting resolution reproduces the former calibration basis.
            // A fit can still reduce its own residual while leaving a false
            // external torque when those coefficients run on the real device.
            auto old_cfg=cfg;old_cfg.velocity_resolution.clear();
            const auto old_fit=calibrate_admittance_friction_cross_validated(reverse,forward,old_cfg);
            assert(old_fit && old_fit->observable[0]);
            auto old_observer=calibrated_friction_observer(*old_fit,resolution);
            const auto old_estimate=old_observer.update(
                TorqueResidualEstimate{{synthetic_friction(middle,resolution)},{}},{middle});
            assert(old_estimate && std::abs(old_estimate->tau_ext_hat[0])>1e-4);
            if(resolution==0.02) assert(std::abs(old_estimate->tau_ext_hat[0])>0.1);
        }

        // Calibration must keep the same velocity law when friction is passed
        // through the momentum observer pole, including both direction changes.
        cfg.momentum_gain={25.0};
        const auto filtered_fit=calibrate_admittance_friction_cross_validated(
            synthetic_friction_run(resolution,true,true),
            synthetic_friction_run(resolution,false,true),cfg);
        assert(filtered_fit && filtered_fit->observable[0] && filtered_fit->validation_pass[0]);
        assert(std::abs(filtered_fit->positive_coulomb[0]-0.8)<1e-8);
        assert(std::abs(filtered_fit->positive_viscous[0]-0.4)<1e-8);
        assert(std::abs(filtered_fit->negative_coulomb[0]+0.6)<1e-8);
        assert(std::abs(filtered_fit->negative_viscous[0]+0.3)<1e-8);
        assert(filtered_fit->cross_residual_rms_after[0]<1e-9);
        auto filtered_observer=calibrated_friction_observer(*filtered_fit,resolution);
        for(double velocity:{-0.125,-middle,middle,0.125}) {
            const auto estimate=filtered_observer.update(
                TorqueResidualEstimate{{synthetic_friction(velocity,resolution)},{}},{velocity});
            assert(estimate && std::abs(estimate->tau_ext_hat[0])<1e-9);
        }
    }

    AdmittanceFrictionCalibrationCfg invalid;invalid.joints_count=1;
    const auto samples=synthetic_friction_run(0.0,false,false);
    for(const JointVector& resolution:{JointVector{-0.001},JointVector{0.001,0.002},
          JointVector{std::numeric_limits<double>::infinity()},
          JointVector{std::numeric_limits<double>::quiet_NaN()}}) {
        invalid.velocity_resolution=resolution;
        const auto result=calibrate_admittance_friction(samples,invalid);
        assert(!result && result.error()==AdmittanceStaticCalibrationErr::INVALID_CFG);
    }
    // Existing callers with no hardware resolution keep the zero-resolution
    // behavior, including the filtered legacy fixture below.
    invalid.velocity_resolution.clear();
    const auto legacy=calibrate_admittance_friction(samples,invalid);
    assert(legacy && legacy->observable[0]);
}

} // namespace

int main() {
    check_resolution_aware_friction_calibration();
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
