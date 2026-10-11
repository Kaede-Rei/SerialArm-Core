#include "serial_arm/core/park_policy.hpp"
#include <cassert>
#include <vector>
int main() {
    using namespace serial_arm;
    assert(!park_time_to_rigid(7.49,10.,0.75));
    assert(park_time_to_rigid(7.50,10.,0.75));
    assert(park_time_to_rigid(9.99,10.,0.75));
    assert(!park_time_to_rigid(7.5,10.,0.0));
    assert(!park_time_to_rigid(7.5,0.,0.75));
    std::vector<double> target(6,0.), pos(6,0.066568), vel(6,0.007328);
    assert(park_safe_for_timed_rigid(pos,vel,target,0.1,0.1,2.0));
    pos[2]=0.21;
    assert(!park_safe_for_timed_rigid(pos,vel,target,0.1,0.1,2.0));
    pos[2]=0.066568;
    vel[4]=0.13;
    assert(!park_safe_for_timed_rigid(pos,vel,target,0.1,0.1,2.0));
    vel[4]=0.007328;
    assert(!park_safe_for_timed_rigid({},vel,target,0.1,0.1,2.0));
}
