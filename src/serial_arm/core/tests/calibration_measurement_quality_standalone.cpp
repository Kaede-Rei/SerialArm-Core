#include "serial_arm/dynamics/calibration_measurement_quality.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using serial_arm::calibration_measurement::Observation;
using serial_arm::calibration_measurement::StableWindow;

int main() {
    StableWindow good;
    for(int k=0;k<45;++k) good.push(Observation{0.005*k,{0.2,0.1}, {0.0,0.0},
        {1.0+0.01*std::sin(k*0.1), 3.0+0.01*std::sin(k*0.2)}});
    assert(good.stable());
    StableWindow slipping;
    for(int k=0;k<45;++k) slipping.push(Observation{0.005*k,{0.2+0.0005*k,0.1}, {0.0,0.0}, {1.0,3.0}});
    assert(!slipping.stable());
    StableWindow drifting_torque;
    for(int k=0;k<45;++k) drifting_torque.push(Observation{0.005*k,{0.2}, {0.0}, {1.0+0.035*k}});
    assert(!drifting_torque.stable());
    StableWindow moving;
    for(int k=0;k<45;++k) moving.push(Observation{0.005*k,{0.2}, {0.07}, {1.0}});
    assert(!moving.stable());
    StableWindow discontinuity;
    for(int k=0;k<45;++k) discontinuity.push(Observation{0.005*k,{0.2}, {0.0}, {1.0}});
    assert(discontinuity.stable());
    discontinuity.push(Observation{3.5,{0.2},{0.0},{1.0}});
    assert(!discontinuity.stable());
    std::cout << "calibration_measurement_quality PASS\n";
}
