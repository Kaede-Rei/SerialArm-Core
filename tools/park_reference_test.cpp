#include "serial_arm/core/park_policy.hpp"
#include "serial_arm/runtime/control_runtime.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace serial_arm;
namespace {
void require(bool value,const char* message) { if(!value)throw std::runtime_error(message); }
}

int main() {
    for(double lag:{0.0,.5,1.0,1.5,2.0,3.0}) {
        JointVector q{.02*lag,.0002*lag},v{0,0},target{1,.01};
        const JointVector vmax{.5,.005},acc{2,.02},tolerance{.02,.0002};
        for(int k=0;k<80;++k) {
            const JointVector measured{q[0]-.02*lag,q[1]-.0002*lag};
            const double rotary=park_reference_speed_fraction(q[0],measured[0],tolerance[0]);
            const double linear=park_reference_speed_fraction(q[1],measured[1],tolerance[1]);
            require(std::abs(rotary-linear)<1e-12,"parking slowdown changed with joint position units");
            advance_joint_reference(q,v,target,vmax,acc,.2,.005,&measured,&tolerance);
            require(std::abs(q[0]*.01-q[1])<1e-12 && std::abs(v[0]*.01-v[1])<1e-12,
                "rotary and linear parking references progressed differently after SI scaling");
        }
        if(lag>=2)require(v[0]==0 && v[1]==0,"a lagging park reference kept accelerating");
    }
    JointVector q{0},v{0},target{1},vmax{1},acc{1},measured{0};
    bool rejected=false;
    try { advance_joint_reference(q,v,target,vmax,acc,.2,.005,&measured); }
    catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"parking feedback silently used an unconfigured following tolerance");
    std::cout<<"PASS: parking follows per-axis SI tolerances with unit-invariant reference scaling\n";
}
