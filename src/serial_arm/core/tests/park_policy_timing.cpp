#include "serial_arm/core/park_policy.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

int main() {
    using namespace serial_arm;
    assert(!park_time_to_rigid(7.49,10.,0.75));
    assert(park_time_to_rigid(7.50,10.,0.75));
    assert(park_time_to_rigid(9.99,10.,0.75));
    assert(!park_time_to_rigid(7.5,10.,0.0));
    assert(!park_time_to_rigid(7.5,0.,0.75));

    std::vector<double> target(6,0.), pos(6,0.066568), vel(6,0.007328);
    assert(park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    // Regression: neither the old 0.20-rad wall nor larger offsets may trap
    // a stopped robot in COMPLIANT_TRACKING after the 75% time boundary.
    pos[5]=0.210383;
    assert(park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    pos[2]=0.692950;
    assert(park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    pos[2]=2.0;
    assert(park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    vel[4]=0.13;
    assert(!park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    vel[4]=0.007328;
    pos[0]=std::numeric_limits<double>::quiet_NaN();
    assert(!park_safe_for_timed_rigid(pos,vel,target,0.1,2.0));
    pos[0]=0.066568;
    assert(!park_safe_for_timed_rigid({},vel,target,0.1,2.0));

    assert(park_rigid_gain_fraction(0.0,0.65)==0.0);
    assert(std::abs(park_rigid_gain_fraction(0.325,0.65)-0.5)<1e-12);
    assert(park_rigid_gain_fraction(0.65,0.65)==1.0);
    assert(park_rigid_gain_fraction(1.0,0.65)==1.0);
    assert(park_rigid_gain_fraction(-1.0,0.65)==0.0);
    assert(park_rigid_gain_fraction(0.1,0.0)==0.0);
    assert(std::isfinite(park_rigid_gain_fraction(0.3,0.65)));
}
