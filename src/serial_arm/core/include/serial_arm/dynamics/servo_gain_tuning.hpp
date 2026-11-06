#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {
constexpr double pi=3.14159265358979323846;
struct Requirements {
    double peak_error_rad{0.012},rms_error_rad{0.004},velocity_rms_rad_s{0.08};
    double overshoot_rad{0.004},settling_error_rad{0.003},settling_time_s{0.7};
    double hold_error_rad{0.004},hold_velocity_rad_s{0.025};
    double amplitude_rad{0.008},probe_duration_s{8.0},tail_s{1.5};
    double local_range_rad{0.04};
    double noise_rms_rad{0},encoder_step_rad{0},velocity_noise_floor{0};
    bool stationary_probe{false},motion_probe{false};
    std::array<double,3> frequencies_hz{0.35,0.70,1.20};
    std::size_t max_candidates{3};
};
inline constexpr std::size_t motion_attempt_limit=5;
struct Sample {
    double time_s{0},reference{0},reference_velocity{0},actual{0},velocity{0};
    double residual{0};
    bool limited{false},new_measurement{false},residual_valid{false};
};
struct Metrics {
    std::size_t samples{0},measurement_samples{0},limited_samples{0},tail_samples{0};
    double rms_error{0},peak_error{0},velocity_rms{0},overshoot{0},settling_s{0};
    double residual_bias{0},residual_noise_rms{0},tail_drift{0},tail_peak_velocity{0},tail_error_rad{0};
    double reference_span{0},actual_span{0},response_rms_ratio{0};
    std::size_t residual_samples{0},tail_reversals{0};
    bool continuous{true},settled{false},oscillating{false},observable{false},passed{false};
};
struct NoiseEstimate {
    double rms_rad{0},encoder_step_rad{0},velocity_rms{0},velocity_peak{0},velocity_step{0};
    double span_rad{0},drift_rad{0};
    std::size_t samples{0};bool stationary{false};
};
inline double observed_encoder_step(std::vector<double> positions) {
    std::sort(positions.begin(),positions.end());double step=std::numeric_limits<double>::infinity();
    for(std::size_t i=1;i<positions.size();++i)if(positions[i]-positions[i-1]>1e-9)
        step=std::min(step,positions[i]-positions[i-1]);
    return std::isfinite(step)?step:0.0;
}
inline NoiseEstimate estimate_noise(const std::vector<Sample>& rows) {
    NoiseEstimate result;std::vector<double> q,v;double sum=0,square=0,time=0,time2=0,product=0,velocity_square=0;
    for(const auto& row:rows)if(row.new_measurement && std::isfinite(row.actual)) {
        const double delta=row.actual-rows.front().actual;q.push_back(row.actual);sum+=delta;square+=delta*delta;
        time+=row.time_s;time2+=row.time_s*row.time_s;product+=row.time_s*delta;
        v.push_back(row.velocity);velocity_square+=row.velocity*row.velocity;result.velocity_peak=std::max(result.velocity_peak,std::abs(row.velocity));
    }
    result.samples=q.size();result.encoder_step_rad=observed_encoder_step(q);
    if(!q.empty())result.span_rad=*std::max_element(q.begin(),q.end())-*std::min_element(q.begin(),q.end());
    if(!q.empty())result.rms_rad=std::sqrt(std::max(0.0,square/q.size()-std::pow(sum/q.size(),2)));
    if(!v.empty())result.velocity_rms=std::sqrt(velocity_square/v.size());
    result.velocity_step=observed_encoder_step(v);
    if(q.size()>=100) {
        const double count=q.size(),denominator=time2-time*time/count;
        const double slope=denominator>0?(product-time*sum/count)/denominator:0;
        const double residual=std::sqrt(std::max(0.0,(square-sum*sum/count-slope*(product-time*sum/count))/count));
        const double travel=std::abs(slope)*(rows.back().time_s-rows.front().time_s);
        result.drift_rad=travel;
        // Encoder jitter is a few stationary codes, never a moving baseline.
        // A symmetric oscillation has zero slope but still spans many codes.
        result.stationary=result.span_rad<=.0015 && result.span_rad<=std::max(3*result.encoder_step_rad,1e-6) &&
            travel<=std::max(result.encoder_step_rad,2*residual+1e-7);
    }
    return result;
}
inline double position_window_velocity(const std::vector<Sample>& rows,const NoiseEstimate& noise,double window=.25) {
    if(rows.size()<2)return 0;
    double lo=1e30,hi=-1e30,st=0,sq=0,stt=0,stq=0,count=0;
    const double end=rows.back().time_s,origin=rows.back().actual;
    for(const auto& row:rows)if(row.new_measurement && row.time_s>=end-window) {
        const double t=row.time_s-end,q=row.actual-origin;
        lo=std::min(lo,q);hi=std::max(hi,q);st+=t;sq+=q;stt+=t*t;stq+=t*q;++count;
    }
    if(count<3 || hi-lo<=std::max(2*noise.encoder_step_rad,6*noise.rms_rad))return 0;
    const double denominator=stt-st*st/count;
    return denominator>1e-12?(stq-st*sq/count)/denominator:0;
}
struct ProbeBounds {double position_range{0},max_velocity{0},max_acceleration{0},effort_budget{0},kp{0},kd{0};};
inline Requirements bounded_plan(Requirements req,double amplitude,double frequency,const ProbeBounds& bounds) {
    if(!std::isfinite(amplitude)||amplitude<=0||!std::isfinite(frequency)||frequency<=0 ||
       !std::isfinite(bounds.position_range)||bounds.position_range<=0 ||
       !std::isfinite(bounds.max_velocity)||bounds.max_velocity<=0 ||
       !std::isfinite(bounds.max_acceleration)||bounds.max_acceleration<=0 ||
       !std::isfinite(bounds.effort_budget)||bounds.effort_budget<=0 || bounds.kp<0 || bounds.kd<0)
        throw std::invalid_argument("invalid bounded gain probe plan");
    req.frequencies_hz={frequency,2.0*frequency,frequency*1.2/.35};
    // Include the envelope's velocity and acceleration, not only the sine terms
    const double edge=.7,rate=1.5/edge,acceleration=6.0/(edge*edge);
    double w=0,w2=0;const std::array<double,3> weights{.4,.35,.25};
    for(std::size_t i=0;i<3;++i){const double x=2*pi*req.frequencies_hz[i];w+=weights[i]*x;w2+=weights[i]*x*x;}
    const double discovery_w=2*pi*frequency;
    const double velocity=std::max(w,discovery_w)+rate;
    const double accel=std::max(w2,discovery_w*discovery_w)+2*rate*std::max(w,discovery_w)+acceleration;
    req.amplitude_rad=std::min({amplitude,std::max(0.0,req.local_range_rad-req.overshoot_rad),std::max(0.0,bounds.position_range-req.overshoot_rad),
        bounds.max_velocity/velocity,bounds.max_acceleration/accel,
        bounds.effort_budget/std::max(1e-8,2*bounds.kp+2*velocity*bounds.kd)});
    req.probe_duration_s=std::max(8.0,2.0/frequency+1.4);
    return req;
}
struct MotionObservation {
    double reference_positive{0},reference_negative{0},actual_positive{0},actual_negative{0};
    double response_rms_ratio{0},coherence{0},encoder_step_rad{0},noise_rms_rad{0},minimum_motion_span_rad{0};
    bool bidirectional{false},qualified{false};std::string reason{"insufficient_measurements"};
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
            m.tail_error_rad=std::max(m.tail_error_rad,std::abs(e));
            m.tail_peak_velocity=std::max(m.tail_peak_velocity,std::abs(s.velocity));
            const double velocity_floor=std::max(req.hold_velocity_rad_s,req.velocity_noise_floor);
            if(std::abs(e)>req.settling_error_rad || std::abs(s.velocity)>velocity_floor)last_bad=s.time_s;
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
    // A stationary velocity code may be nonzero, but its tolerance cannot hide
    // actual position drift or an oscillating tail.
    if(req.velocity_noise_floor>req.hold_velocity_rad_s && m.tail_drift>std::max({3*req.encoder_step_rad,6*req.noise_rms_rad,.0002})) {
        std::vector<Sample> tail;for(const auto& s:rows)if(s.time_s>=excitation_end_s)tail.push_back(s);
        NoiseEstimate noise;noise.encoder_step_rad=req.encoder_step_rad;noise.rms_rad=req.noise_rms_rad;
        if(std::abs(position_window_velocity(tail,noise))>req.hold_velocity_rad_s || m.tail_drift>std::max({3*req.encoder_step_rad,6*req.noise_rms_rad,.0002}))m.settled=false;
    }
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
inline MotionObservation observe_motion(const std::vector<Sample>& rows,const Requirements& req,
                                         const NoiseEstimate& noise) {
    MotionObservation out;out.noise_rms_rad=noise.rms_rad;
    if(rows.empty())return out;
    const auto metrics=assess(rows,req,req.probe_duration_s);
    std::vector<double> u,y,positions;
    const double reference_center=rows.front().reference,actual_center=rows.front().actual;
    for(const auto& row:rows)if(row.new_measurement) {
        out.reference_positive=std::max(out.reference_positive,row.reference-reference_center);
        out.reference_negative=std::max(out.reference_negative,reference_center-row.reference);
        out.actual_positive=std::max(out.actual_positive,row.actual-actual_center);
        out.actual_negative=std::max(out.actual_negative,actual_center-row.actual);
        positions.push_back(row.actual);
        if(row.time_s>=.7 && row.time_s<req.probe_duration_s-.7) {
            u.push_back(row.reference-reference_center);y.push_back(row.actual-actual_center);
        }
    }
    out.encoder_step_rad=std::max(noise.encoder_step_rad,observed_encoder_step(positions));
    out.response_rms_ratio=metrics.response_rms_ratio;
    const double required=std::max({6*out.encoder_step_rad,6*noise.rms_rad,
        .2*std::min(out.reference_positive,out.reference_negative)});
    out.minimum_motion_span_rad=required;
    out.bidirectional=required>0 && out.actual_positive>=required && out.actual_negative>=required;
    // Establish correlated motion without claiming dynamic tracking quality.
    // A small causal shift allows the low-frequency starting response to lag;
    // the later independent FRF still uses the original strict phase criteria.
    for(std::size_t lag=0;lag<std::min<std::size_t>(100,u.size()/4);lag+=4) {
        double su=0,sy=0,suu=0,syy=0,suy=0;const double count=static_cast<double>(u.size()-lag);
        for(std::size_t i=lag;i<u.size();++i){const double a=u[i-lag],b=y[i];su+=a;sy+=b;suu+=a*a;syy+=b*b;suy+=a*b;}
        const double a=suu-su*su/count,b=syy-sy*sy/count,c=suy-su*sy/count;
        if(a>1e-12 && b>1e-12 && c>0)out.coherence=std::max(out.coherence,std::clamp(c*c/(a*b),0.0,1.0));
    }
    if(metrics.measurement_samples<100 || !metrics.continuous)out.reason="insufficient_measurements";
    else if(metrics.limited_samples)out.reason="limited";
    else if(std::max(out.actual_positive,out.actual_negative)<required)out.reason="no_motion";
    else if(!out.bidirectional)out.reason="one_sided_response";
    else if(out.coherence<.65 || metrics.response_rms_ratio<.2)out.reason="low_coherence";
    else {out.reason="motion_established";out.qualified=true;}
    return out;
}
struct FrequencyPoint {double hz{0},gain{0},phase_rad{0};};
struct ResponseModel {
    bool identifiable{false},resonance_observed{false};
    double natural_frequency_rad_s{0},damping_ratio{0},effective_delay_s{0};
    double relative_fit_error{1e30},held_out_error{1e30},tested_follow_frequency_hz{0};
    double coherence{0};
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
    if(signal>1e-12)out.coherence=std::clamp(1.0-(design*yc-y).squaredNorm()/signal,0.0,1.0);
    if(signal<1e-8 || out.coherence<.65)return out;
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
    const std::vector<Metrics>& metrics,const std::vector<ResponseModel>& models,const Requirements& req=Requirements{}) {
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
        const bool ring=metrics[i].oscillating || metrics[i].overshoot>req.overshoot_rad || models[i].resonance_observed ||
            (models[i].identifiable && models[i].damping_ratio<0.65);
        const bool overdamped=models[i].identifiable && models[i].damping_ratio>1.4 &&
            models[i].effective_delay_s*models[i].natural_frequency_rad_s<.2;
        const bool delayed=models[i].identifiable && models[i].effective_delay_s*models[i].natural_frequency_rad_s>.4;
        const double response=std::max(.1,metrics[i].response_rms_ratio);
        const double measured_deficit=std::max({metrics[i].rms_error/std::max(1e-8,req.rms_error_rad),
            metrics[i].tail_error_rad/std::max(1e-8,.8*req.settling_error_rad),
            models[i].tested_follow_frequency_hz<=0?1.0/response:1.0});
        const double factor=ring || metrics[i].limited_samples ? .8 : delayed ? .85 : overdamped ? 1.0 :
            std::clamp(measured_deficit,1.2,2.0);
        const double kp=std::clamp(current.kp[i]*factor,0.0,kp_max[i]);
        // Keep the configured/measured damping baseline. Model inertia supplies
        // a candidate increment, while the actual response and hard limits judge it.
        const double kd=std::clamp(current.kd[i]*(ring?1.2:metrics[i].limited_samples || delayed?.9:overdamped?.8:1.0),0.0,kd_max[i]);
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
    if(req.stationary_probe || time<0||time>=req.probe_duration_s)return {0,0};
    const double edge=0.7;
    const double x=std::clamp(std::min(time,req.probe_duration_s-time)/edge,0.0,1.0);
    const double envelope=x*x*(3-2*x);
    const double de=x<1 ? 6*x*(1-x)/edge*(time<edge?1.0:-1.0) : 0.0;
    const std::array<double,3> weights{.4,.35,.25};double q=0,v=0;
    for(std::size_t j=0;j<(req.motion_probe?1:3);++j){const double w=2*pi*req.frequencies_hz[j],weight=req.motion_probe?1:weights[j];q+=weight*std::sin(w*time);v+=weight*w*std::cos(w*time);}
    return {req.amplitude_rad*envelope*q,req.amplitude_rad*(envelope*v+de*q)};
}
} // namespace serial_arm::servo_tuning
