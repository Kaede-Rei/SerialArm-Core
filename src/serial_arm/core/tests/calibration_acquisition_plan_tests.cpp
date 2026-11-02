#include "serial_arm/dynamics/calibration_acquisition_plan.hpp"
#include <cmath>
#include <stdexcept>
#include <set>

int main() {
    namespace acq=serial_arm::calibration_acquisition;
    std::set<std::pair<bool,int>> covered;
    bool at_back=true;
    double factor=2.;
    for(const auto& pass:acq::dynamic_passes) {
        if(pass.reverse!=at_back)throw std::runtime_error("replay endpoint discontinuity");
        if(!(pass.rate>0 && pass.rate<=1))throw std::runtime_error("unsafe rate");
        at_back=!pass.reverse;
        factor+=1./pass.rate;
        covered.emplace(pass.reverse,pass.speed_tier);
    }
    if(!at_back || covered.size()!=4)throw std::runtime_error("missing direction/speed or wrong final endpoint");
    if(std::abs(factor-acq::traversal_factor)>1e-12)throw std::runtime_error("duration factor");
    if(std::abs(acq::estimated_seconds(10,12,1.15)-109.6)>1e-9)throw std::runtime_error("both static dwells missing");
    if(acq::progress_units(12)!=30)throw std::runtime_error("progress units");
}
