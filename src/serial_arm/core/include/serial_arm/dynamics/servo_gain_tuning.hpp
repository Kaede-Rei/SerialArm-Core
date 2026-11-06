#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <vector>

namespace serial_arm::servo_tuning {
constexpr double pi=3.14159265358979323846;
struct Requirements {
    double peak_error_rad{0.012},rms_error_rad{0.004},velocity_rms_rad_s{0.08};
    double overshoot_rad{0.004},settling_error_rad{0.003},settling_time_s{0.7};
    double hold_error_rad{0.004},hold_velocity_rad_s{0.025};
    double amplitude_rad{0.008},probe_duration_s{8.0},tail_s{1.5};
    std::array<double,3> frequencies_hz{0.35,0.70,1.20};
    std::size_t max_candidates{3};
};
struct Sample {
    double time_s{0},reference{0},reference_velocity{0},actual{0},velocity{0};
    double residual{0};
    bool limited{false},new_measurement{false},residual_valid{false};
};
struct Metrics {
    std::size_t samples{0},measurement_samples{0},limited_samples{0},tail_samples{0};
    double rms_error{0},peak_error{0},velocity_rms{0},overshoot{0},settling_s{0};
    double residual_bias{0},residual_noise_rms{0},tail_drift{0},tail_peak_velocity{0};
    double reference_span{0},actual_span{0},response_rms_ratio{0};
    std::size_t residual_samples{0},tail_reversals{0};
    bool continuous{true},settled{false},oscillating{false},observable{false},passed{false};
};
inline Metrics assess(const std::vector<Sample>& rows,const Requirements& req,double excitation_end_s) {
    Metrics m;if(rows.empty())return m;
    double sum=0,vs=0,lo=rows.front().reference,hi=lo,actual_lo=rows.front().actual,actual_hi=actual_lo;
    double tail_lo=1e30,tail_hi=-1e30,rs=0,rss=0,last_bad=excitation_end_s;
    int last_sign=0;double last_measurement=rows.front().time_s;
    double reference_sum=0,reference_sq=0,actual_sum=0,actual_sq=0;
    std::size_t excitation_samples=0;
    for(std::size_t k=0;k<rows.size();++k) {
        const auto& s=rows[k];
        if(!std::isfinite(s.time_s)||!std::isfinite(s.reference)||!std::isfinite(s.actual)||!std::isfinite(s.velocity))return Metrics{};
        if(k && (s.time_s<=rows[k-1].time_s || s.time_s-rows[k-1].time_s>0.06))m.continuous=false;
        if(s.new_measurement){if(s.time_s-last_measurement>0.06)m.continuous=false;last_measurement=s.time_s;++m.measurement_samples;}
        const double e=s.actual-s.reference;sum+=e*e;vs+=std::pow(s.velocity-s.reference_velocity,2);
        m.peak_error=std::max(m.peak_error,std::abs(e));++m.samples;
        m.limited_samples+=s.limited;
        lo=std::min(lo,s.reference);hi=std::max(hi,s.reference);
        actual_lo=std::min(actual_lo,s.actual);actual_hi=std::max(actual_hi,s.actual);
        if(s.new_measurement && s.time_s>=0.7 && s.time_s<excitation_end_s-0.7) {
            // Remove the posture offset before measuring motion. A stationary
            // encoder must not pass merely because the requested motion is small.
            const double u=s.reference-rows.front().reference,y=s.actual-rows.front().actual;
            reference_sum+=u;reference_sq+=u*u;actual_sum+=y;actual_sq+=y*y;++excitation_samples;
        }
        if(s.time_s>=excitation_end_s) {
            ++m.tail_samples;tail_lo=std::min(tail_lo,s.actual);tail_hi=std::max(tail_hi,s.actual);
            m.tail_peak_velocity=std::max(m.tail_peak_velocity,std::abs(s.velocity));
            if(std::abs(e)>req.settling_error_rad || std::abs(s.velocity)>req.hold_velocity_rad_s)last_bad=s.time_s;
            const int sign=s.velocity>0.003?1:s.velocity<-.003?-1:0;
            if(sign && last_sign && sign!=last_sign)++m.tail_reversals;
            if(sign)last_sign=sign;
            if(s.residual_valid && std::isfinite(s.residual)){rs+=s.residual;rss+=s.residual*s.residual;++m.residual_samples;}
        }
    }
    m.rms_error=std::sqrt(sum/m.samples);m.velocity_rms=std::sqrt(vs/m.samples);
    m.overshoot=std::max({0.0,lo-actual_lo,actual_hi-hi});
    m.settling_s=std::max(0.0,last_bad-excitation_end_s);
    m.settled=m.tail_samples>=20 && rows.back().time_s-last_bad>=0.25;
    m.tail_drift=m.tail_samples ? tail_hi-tail_lo : 1e30;
    m.oscillating=m.tail_reversals>=4 && m.tail_drift>req.settling_error_rad;
    if(m.residual_samples){m.residual_bias=rs/m.residual_samples;m.residual_noise_rms=std::sqrt(std::max(0.0,rss/m.residual_samples-m.residual_bias*m.residual_bias));}
    if(rows.back().time_s-last_measurement>0.06)m.continuous=false;
    m.reference_span=hi-lo;m.actual_span=actual_hi-actual_lo;
    if(excitation_samples>=100) {
        const double count=static_cast<double>(excitation_samples);
        const double input_variance=std::max(0.0,reference_sq/count-std::pow(reference_sum/count,2));
        const double output_variance=std::max(0.0,actual_sq/count-std::pow(actual_sum/count,2));
        if(input_variance>1e-12)m.response_rms_ratio=std::sqrt(output_variance/input_variance);
        m.observable=m.reference_span>=0.2*req.amplitude_rad &&
            m.actual_span>=0.2*m.reference_span && m.response_rms_ratio>=0.2;
    }
    m.passed=m.observable && m.samples>=100 && m.measurement_samples>=100 && m.continuous && m.limited_samples==0 &&
        m.rms_error<=req.rms_error_rad && m.peak_error<=req.peak_error_rad &&
        m.velocity_rms<=req.velocity_rms_rad_s && m.overshoot<=req.overshoot_rad &&
        m.settled && m.settling_s<=req.settling_time_s && !m.oscillating;
    return m;
}
struct FrequencyPoint {double hz{0},gain{0},phase_rad{0};};
struct ResponseModel {
    bool identifiable{false},resonance_observed{false};
    double natural_frequency_rad_s{0},damping_ratio{0},effective_delay_s{0};
    double relative_fit_error{1e30},held_out_error{1e30},tested_follow_frequency_hz{0};
    // A cutoff is reported only when measurements bracket it, never extrapolated.
    double observed_cutoff_hz{std::numeric_limits<double>::quiet_NaN()};
    std::vector<FrequencyPoint> points;
};
inline ResponseModel identify(const std::vector<Sample>& rows,const Requirements& req) {
    ResponseModel out;
    std::vector<const Sample*> selected;
    for(const auto& s:rows)if(s.new_measurement && s.time_s>=0.7 && s.time_s<req.probe_duration_s-0.7 && !s.limited)selected.push_back(&s);
    if(selected.size()<100)return out;
    Eigen::MatrixXd design(selected.size(),7);Eigen::VectorXd u(selected.size()),y(selected.size());
    for(std::size_t i=0;i<selected.size();++i) {
        design(i,0)=1;
        for(std::size_t j=0;j<3;++j){const double a=2*pi*req.frequencies_hz[j]*selected[i]->time_s;design(i,1+2*j)=std::sin(a);design(i,2+2*j)=std::cos(a);}
        u[i]=selected[i]->reference;y[i]=selected[i]->actual;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    if(qr.rank()!=7)return out;
    const Eigen::VectorXd uc=qr.solve(u),yc=qr.solve(y);
    const double signal=(y.array()-y.mean()).matrix().squaredNorm();
    if(signal<1e-8 || (design*yc-y).squaredNorm()>0.35*signal)return out;
    std::array<std::complex<double>,3> response;
    for(std::size_t j=0;j<3;++j) {
        const std::complex<double> a{uc[1+2*j],uc[2+2*j]},b{yc[1+2*j],yc[2+2*j]};
        if(std::abs(a)<req.amplitude_rad*0.04)return out;
        response[j]=b/a;out.points.push_back({req.frequencies_hz[j],std::abs(response[j]),std::arg(response[j])});
        if(std::abs(response[j])>=0.80 && std::abs(response[j])<=1.20 && std::abs(std::arg(response[j]))<0.45)
            out.tested_follow_frequency_hz=req.frequencies_hz[j];
    }
    // Fit two frequencies, retain the third as an independent spectral check.
    struct Fit {double loss,wn,zeta,delay,held;};
    std::vector<Fit> fits;fits.reserve(65*32*17);double best=1e30;
    for(int ni=0;ni<65;++ni)for(int zi=0;zi<32;++zi)for(int di=0;di<17;++di) {
        const double wn=1.0*std::pow(45.0,ni/64.0),zeta=0.15+2.85*zi/31.0,delay=0.005*di;
        double fit=0,held=0;
        for(std::size_t j=0;j<3;++j) {
            const double w=2*pi*req.frequencies_hz[j];
            const auto h=(wn*wn/std::complex<double>{wn*wn-w*w,2*zeta*wn*w})*std::exp(std::complex<double>{0,-w*delay});
            const double e=std::norm(h-response[j])/std::max(0.01,std::norm(response[j]));
            if(j<2)fit+=e/2;else held=e;
        }
        fits.push_back({fit,wn,zeta,delay,held});
        if(fit<best){best=fit;out.natural_frequency_rad_s=wn;out.damping_ratio=zeta;out.effective_delay_s=delay;out.held_out_error=std::sqrt(held);}
    }
    out.relative_fit_error=std::sqrt(best);
    double wn_low=1e30,wn_high=0,z_low=1e30,z_high=0,d_low=1e30,d_high=0;
    for(const auto& fit:fits)if(fit.loss<=best+1e-4 && fit.held<.05*.05) {
        wn_low=std::min(wn_low,fit.wn);wn_high=std::max(wn_high,fit.wn);
        z_low=std::min(z_low,fit.zeta);z_high=std::max(z_high,fit.zeta);
        d_low=std::min(d_low,fit.delay);d_high=std::max(d_high,fit.delay);
    }
    // Similar low-frequency responses can hide different poles and delays
    // Keep the measured FRF but withhold pole parameters when the fit is ambiguous
    out.identifiable=out.relative_fit_error<0.22 && out.held_out_error<0.05 && out.natural_frequency_rad_s<44 &&
        wn_high>0 && wn_high/wn_low<=1.30 && z_high-z_low<=.35 && d_high-d_low<=.015;
    for(std::size_t j=0;j<out.points.size();++j) {
        out.resonance_observed=out.resonance_observed || out.points[j].gain>1.25;
        if(j && out.points[j-1].gain>std::sqrt(.5) && out.points[j].gain<=std::sqrt(.5))
            out.observed_cutoff_hz=out.points[j].hz;
    }
    return out;
}
struct Gains {std::vector<double> kp,kd;};
inline Gains paired_candidate(const Gains& current,const std::vector<double>& inertia,
    const std::vector<double>& kp_max,const std::vector<double>& kd_max,
    const std::vector<Metrics>& metrics,const std::vector<ResponseModel>& models) {
    const auto n=inertia.size();
    if(current.kp.size()!=n||current.kd.size()!=n||kp_max.size()!=n||kd_max.size()!=n||metrics.size()!=n||models.size()!=n)
        throw std::invalid_argument("servo candidate dimension mismatch");
    Gains next=current;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(current.kp[i]) || current.kp[i]<0 || !std::isfinite(current.kd[i]) || current.kd[i]<0 ||
           !std::isfinite(inertia[i]) || inertia[i]<=0 || !std::isfinite(kp_max[i]) || kp_max[i]<=0 ||
           !std::isfinite(kd_max[i]) || kd_max[i]<=0)
            throw std::invalid_argument("invalid servo gain or safety bound");
        // An early failed axis leaves later axes unmeasured. Do not revise them
        // or replace a qualified configured baseline with a theoretical ratio.
        if(!metrics[i].samples || (metrics[i].passed && models[i].tested_follow_frequency_hz>0))continue;
        const bool ring=metrics[i].oscillating || metrics[i].overshoot>0.004 || models[i].resonance_observed ||
            (models[i].identifiable && (models[i].damping_ratio<0.65 || models[i].effective_delay_s*models[i].natural_frequency_rad_s>0.4));
        const double factor=ring || metrics[i].limited_samples ? .72 :
            !metrics[i].observable ? 1.6 : models[i].tested_follow_frequency_hz<=0 ? 1.35 : 1.15;
        const double kp=std::clamp(current.kp[i]*factor,0.0,kp_max[i]);
        // Keep the configured/measured damping baseline. Model inertia supplies
        // a candidate increment, while the actual response and hard limits judge it.
        const double kd=std::min(kd_max[i],std::max(current.kd[i],1.8*std::sqrt(inertia[i]*kp)));
        next.kp[i]=kp;next.kd[i]=std::max(0.0,kd);
    }
    return next;
}
inline double score(const std::vector<Metrics>& metrics) {
    double value=0;for(const auto& m:metrics)value+=m.rms_error+2*m.overshoot+0.01*m.settling_s+
        0.02*m.velocity_rms+(m.oscillating?1.0:0.0)+0.01*m.limited_samples+(m.continuous?0.0:10.0);
    return value;
}
inline std::pair<double,double> probe(double time,const Requirements& req) {
    if(time<0||time>=req.probe_duration_s)return {0,0};
    const double edge=0.7;
    const double x=std::clamp(std::min(time,req.probe_duration_s-time)/edge,0.0,1.0);
    const double envelope=x*x*(3-2*x);
    const double de=x<1 ? 6*x*(1-x)/edge*(time<edge?1.0:-1.0) : 0.0;
    const std::array<double,3> weights{.4,.35,.25};double q=0,v=0;
    for(std::size_t j=0;j<3;++j){const double w=2*pi*req.frequencies_hz[j];q+=weights[j]*std::sin(w*time);v+=weights[j]*w*std::cos(w*time);}
    return {req.amplitude_rad*envelope*q,req.amplitude_rad*(envelope*v+de*q)};
}
} // namespace serial_arm::servo_tuning
