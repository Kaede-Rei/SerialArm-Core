#include "serial_arm/dynamics/calibration_retime.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    using serial_arm::calibration_retime::plan;
    using serial_arm::calibration_retime::interpolate;
    const std::vector<double> vmax(6, 1.35), amax(6, 2.0);
    auto verify = [&](const std::vector<std::vector<double>>& input, const char* label, bool expect_target) {
        const auto p = plan(input, vmax, amax, 200.0);
        assert(!p.positions.empty() && p.positions.size()>input.size()/200);
        assert(std::isfinite(p.sample_dt) && p.sample_dt>0.0);
        double worst_velocity=0.0, worst_acceleration=0.0;
        for(std::size_t k=0;k+1<p.positions.size();++k) {
            for(int sample=0;sample<=8;++sample) {
                const double u=sample/8.0;
                const auto s=interpolate(p.positions,static_cast<double>(k)+u,p.sample_dt,1.0);
                for(std::size_t j=0;j<6;++j) {
                    const double p0=p.positions[k][j],p1=p.positions[k+1][j];
                    const double m0=k==0?0.0:0.5*(p1-p.positions[k-1][j]);
                    const double m1=k+2==p.positions.size()?0.0:0.5*(p.positions[k+2][j]-p0);
                    const double acc=( (12*u-6)*p0+(6*u-4)*m0+(-12*u+6)*p1+(6*u-2)*m1 )/(p.sample_dt*p.sample_dt);
                    worst_velocity=std::max(worst_velocity,std::abs(s.velocity[j])/vmax[j]);
                    worst_acceleration=std::max(worst_acceleration,std::abs(acc)/amax[j]);
                }
            }
        }
        const double estimated=40.0+2*p.single_pass_s+14.0;
        std::cout<<label<<" samples="<<input.size()<<" scheduled_samples="<<p.positions.size()
                 <<" one_way="<<p.single_pass_s<<" total="<<estimated
                 <<" velocity_fraction="<<worst_velocity<<" acceleration_fraction="<<worst_acceleration<<'\n';
        assert(worst_velocity<0.801 && worst_acceleration<0.801);
        if(expect_target) assert(estimated<200.0);
    };
    std::vector<std::vector<double>> gentle;
    for(int k=0;k<=2000;++k) {
        const double t=k/50.0;
        const double s=std::clamp((t-10.0)/20.0,0.0,1.0);
        gentle.push_back({0.5*s,0.25*s,0.7*s,0.1*s,0.3*s,0.2*s});
    }
    verify(gentle,"gentle",true);
    std::vector<std::vector<double>> reversing;
    for(int k=0;k<=2000;++k) {
        const double t=k/50.0;
        const double s=t<10?0:(t<22?(t-10)/12:(t<31?1:1-(t-31)/9));
        reversing.push_back({0.6*s,0.3*std::sin(s*3.141592653589793),0.8*s,0.2*s,0.3*s,0.1*s});
    }
    verify(reversing,"reversing",false);
    // Path with 13k frames, noisy encoder samples and a local corner; this
    // used to inflate the duration of every OTHER knot through a global dt.
    std::vector<std::vector<double>> noisy;
    for(int k=0;k<=13647;++k) {
        const double t=30.0*k/13647.0;
        const double s=t<6?0:t<15?(t-6)/9:t<20?1:std::max(0.0,1-(t-20)/10);
        std::vector<double> q;
        for(int j=0;j<6;++j) {
            const double noise=0.0015*std::sin(k*(1.11+0.013*j));
            q.push_back((0.1+0.08*j)*s+noise+(j==2&&t>14.95&&t<15.05?0.01:0.0));
        }
        noisy.push_back(std::move(q));
    }
    verify(noisy,"noisy_13k",false);
    // Straight path with one distinct corner; local deceleration must not
    // slow all other segments to the worst-case corner's step size.
    std::vector<std::vector<double>> corner;
    for(int k=0;k<2500;++k) {
        const double t=k/50.0;
        const double x=t<25?0.03*t:0.75-0.01*(t-25);
        corner.push_back({x,0.3*x,0.4*x,0.25*x,0.1*x,0.12*x});
    }
    verify(corner,"one_corner",true);
}
