#include "serial_arm/dynamics/servo_gain_tuning.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

using namespace serial_arm::servo_tuning;
namespace {
std::vector<Sample> simulated(double wn,double zeta,double delay=0.0) {
    Requirements req;std::vector<Sample> rows;double q=0,v=0;
    const double dt=.001;std::vector<std::pair<double,double>> commands;
    for(std::size_t k=0;k<11500;++k) {
        const double t=k*dt;const auto u=probe(t,req);commands.push_back(u);
        const auto past=commands[k>=std::size_t(delay/dt)?k-std::size_t(delay/dt):0];
        const double a=wn*wn*(past.first-q)-2*zeta*wn*v;v+=a*dt;q+=v*dt;
        rows.push_back(Sample{t,u.first,u.second,q,v,.12+.005*std::sin(37*t),false,k%5==0,true});
    }
    return rows;
}
}
int main() {
    Requirements req;
    auto rows=simulated(16,.9,.015);const auto model=identify(rows,req);
    assert(model.points.size()==3);
    assert(model.tested_follow_frequency_hz>0);
    for(const auto& point:model.points) {
        const double w=2*pi*point.hz;
        const auto expected=(16.0*16.0/std::complex<double>{16.0*16.0-w*w,2*.9*16*w})*std::exp(std::complex<double>{0,-w*.015});
        assert(std::abs(point.gain-std::abs(expected))<.03);
        assert(std::abs(point.phase_rad-std::arg(expected))<.03);
    }
    assert(!model.identifiable); // Equivalent low-frequency fits must not claim unique poles/delay
    assert(std::isnan(model.observed_cutoff_hz)); // No extrapolated bandwidth
    auto metric=assess(rows,req,req.probe_duration_s);
    assert(metric.passed && metric.measurement_samples>100);
    assert(std::abs(metric.residual_bias-.12)<.002 && metric.residual_noise_rms<.006);
    auto interrupted=rows;interrupted.erase(interrupted.begin()+2000,interrupted.begin()+2100);
    assert(!assess(interrupted,req,8).passed);
    auto limited=rows;limited[1000].limited=true;
    assert(!assess(limited,req,8).passed);
    auto stale=rows;for(auto& row:stale)row.new_measurement=false;
    assert(!assess(stale,req,8).passed && identify(stale,req).points.empty());
    auto inert=rows;for(auto& row:inert){row.actual=0;row.velocity=0;}
    assert(identify(inert,req).points.empty()); // A stationary axis is not a servo model
    auto ringing=simulated(8,.10);const auto ring_metrics=assess(ringing,req,8);
    assert(!ring_metrics.passed);
    Metrics bad;bad.overshoot=.010;bad.oscillating=true;
    const auto rollback=paired_candidate({{100},{1}},{.5},{200},{20},{bad},{ResponseModel{}});
    assert(rollback.kp[0]<100 && rollback.kd[0]>1);
    const auto damping_limited=paired_candidate({{100},{1}},{.5},{200},{1},{bad},{ResponseModel{}});
    assert(damping_limited.kp[0]<=std::pow(1/1.8,2)/.5+1e-12);
    const auto zero_authority=paired_candidate({{0},{0}},{.5},{0},{0},{bad},{ResponseModel{}});
    assert(zero_authority.kp[0]==0 && zero_authority.kd[0]==0);
    const auto start=probe(0,req),end=probe(8,req);
    assert(start.first==0 && start.second==0 && end.first==0 && end.second==0);
    assert(score({metric})<score({ring_metrics}));
    std::cout<<"servo_gain_tuning: measured response, continuity, freshness, damping rollback and torque authority passed\n";
}
