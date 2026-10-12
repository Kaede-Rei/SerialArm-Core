#include "serial_arm/dynamics/calibration_measurement_quality.hpp"
#include "serial_arm/dynamics/calibration_auto_gain.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using serial_arm::calibration_measurement::Observation;
using serial_arm::calibration_measurement::StableWindow;

int main() {
    StableWindow good;
    for(int k=0;k<125;++k) good.push(Observation{0.005*k,{0.2,0.1}, {0.0,0.0},
        {1.0+0.01*std::sin(k*0.1), 3.0+0.01*std::sin(k*0.2)}});
    assert(good.stable());
    StableWindow slipping;
    for(int k=0;k<125;++k) slipping.push(Observation{0.005*k,{0.2+0.00007*k,0.1}, {0.0,0.0}, {1.0,3.0}});
    assert(!slipping.stable());
    StableWindow drifting_torque;
    for(int k=0;k<125;++k) drifting_torque.push(Observation{0.005*k,{0.2}, {0.0}, {1.0+0.035*k}});
    assert(!drifting_torque.stable());
    StableWindow moving;
    for(int k=0;k<125;++k) moving.push(Observation{0.005*k,{0.2}, {0.07}, {1.0}});
    assert(!moving.stable());
    StableWindow discontinuity;
    for(int k=0;k<125;++k) discontinuity.push(Observation{0.005*k,{0.2}, {0.0}, {1.0}});
    assert(discontinuity.stable());
    discontinuity.push(Observation{3.5,{0.2},{0.0},{1.0}});
    assert(!discontinuity.stable());
    // The physical starting gains depend on inertia, not on a fixed percentage
    // of the user's Kp/Kd maxima. Safety bounds still cap the commands.
    namespace acg=serial_arm::calibration_auto_gain;
    auto gains=acg::from_model({0.025,0.045},{0.50,1.0},{200.0,220.0},{4.0,4.0});
    assert(gains.kp[0]>1.0 && gains.kp[0]<20.0);
    auto same=acg::from_model({0.025,0.045},{0.50,1.0},{300.0,300.0},{6.0,6.0});
    assert(std::abs(gains.kp[0]-same.kp[0])<1e-10);
    auto other=acg::from_model({0.100,0.045},{0.50,1.0},{200.0,220.0},{4.0,4.0});
    assert(other.kp[0]>gains.kp[0]);
    auto initial=gains;
    assert(acg::revise(gains,{0.025,0.045},{200.,220.},{4.,4.},
        {0.12,0.01},{0.006,0.0001},{0.08,0.001}));
    assert(gains.kp[0]<initial.kp[0]);
    assert(gains.kp[1]==initial.kp[1]);
    std::cout << "calibration_measurement_quality PASS\n";
}
