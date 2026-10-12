#include "serial_arm/dynamics/calibration_replay_supervisor.hpp"
#include "serial_arm/dynamics/calibration_reference_watchdog.hpp"
#include <cassert>
#include <cmath>
#include <vector>
using namespace serial_arm::calibration_replay_supervisor;
int main() {
    // Grounded in the failed trajectory: J2/J3 had already passed reference.
    const std::vector<double> desired{-0.144324,0.43915,0.523244};
    const std::vector<double> actual{-0.003624,0.598344,0.697146};
    const std::vector<double> speed{-.002,0.1074,-.002};
    const std::vector<double> limits{5.4,5.4,5.4};
    auto status=evaluate(desired,actual,speed,limits);
    assert(status.valid && status.state==State::PAUSE_FOR_FEEDBACK);
    assert(status.worst_joint==2 && status.max_error>0.17);
    assert(status.soft_limit[2]<status.hard_limit[2]);
    const std::vector<double> moving{-0.256628,0.832571,0.913171};
    const auto frozen=command_velocity(moving,true);
    assert(frozen==std::vector<double>({0.0,0.0,0.0}));
    assert(command_velocity(moving,false)==moving);
    // In a waiting step, reference velocity must be ZERO, not the
    // demonstrator's interpolated +0.833/+0.913rad/s.
    const double e2=desired[1]-actual[1];
    const double e3=desired[2]-actual[2];
    const double kp2=5.852461,kp3=18.038382,kd2=1.370763,kd3=4.2;
    assert(kp2*e2+kd2*(0.832571-0.0)>0.0);
    assert(kp3*e3+kd3*(0.913171-0.0)>0.0);
    assert(kp2*e2+kd2*(0.0-0.0)<0.0);
    assert(kp3*e3+kd3*(0.0-0.0)<0.0);
    std::vector<double> kp{2.75541,5.852461,18.038382},kd{.79218,1.370763,4.2};
    const std::vector<double> ceiling_kp{100,220,220}, ceiling_kd{1.8,4.2,4.2};
    for(int i=0;i<10;++i) {
        (void)revise_tracking(kp,kd,status,desired,actual,speed,ceiling_kp,ceiling_kd);
        for(int j=0;j<3;++j){assert(kp[j]<=ceiling_kp[j]&&kd[j]<=ceiling_kd[j]);}
    }
    assert(kp[0]>60.0 && kp[1]>100.0 && kp[2]>100.0);
    auto near=evaluate({.0,.0,.0},{.003,-.003,.001},{.01,.01,.01},limits);
    assert(near.valid && near.state==State::FOLLOWING);
    auto too_small=evaluate({0},{0},{0},{});assert(!too_small.valid);
    auto invalid=evaluate({0.0},{std::nan("")},{0},{5.4});assert(!invalid.valid);
    auto initial_kp=kp,initial_kd=kd;
    assert(!revise_tracking(kp,kd,near,{0.0,0.0,0.0},{.003,-.003,.001},
                           {.01,.01,.01},ceiling_kp,ceiling_kd));
    assert(kp==initial_kp && kd==initial_kd);
}
