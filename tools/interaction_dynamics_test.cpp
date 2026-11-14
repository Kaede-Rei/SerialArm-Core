#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
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
    std::cout << "interaction dynamics passed\n";
}
