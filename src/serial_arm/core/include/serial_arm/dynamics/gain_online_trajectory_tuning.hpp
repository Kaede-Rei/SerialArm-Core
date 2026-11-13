#pragma once

#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

// Indirect self tuning: a local, delayed command-to-position model is fitted
// from independent excitation while the original taught path keeps running.
// This is not a motor-torque measurement or an unconditional stability proof.
// Method sources: Astrom/Wittenmark, On Self Tuning Regulators (TFRT-7017);
// Gilson/Van den Hof, Automatica 41 (2005), 241-249 (closed-loop IV);
// constrained local least-squares/trust-region controller redesign.
struct OnlineTrajectoryReference {
    double time_s{0};
    std::vector<double> position,velocity,feedforward_nm;
};
enum class OnlineTrajectoryUse { Tracking, Compliant };
struct OnlineTrajectoryOptions {
    double window_s{4},update_period_s{1},horizon_s{.8};
    double model_sample_s{.02},minimum_window_s{2},maximum_delay_s{.12};
    double maximum_relative_gain_step{.12};
    std::size_t minimum_fresh_samples{100};
    OnlineTrajectoryUse use{OnlineTrajectoryUse::Tracking};
    double compliant_displacement_rad{.04};
    double compliant_recovery_error_rad{.008},compliant_final_speed_rad_s{.01};
};
struct OnlineTrajectoryAxisModel {
    bool valid{false},reference_only{true};
    std::string reason{"insufficient_online_measurements"};
    std::string interpretation{"predictive_local_per_axis_diagonal_host_command_proxy_not_measured_motor_torque_or_identified_coupling"};
    double dt_s{0},delay_s{0},arx_a1{2},arx_a2{-1},arx_b0{0},arx_b1{0},bias{0},position_origin_rad{0};
    double prior_inertia{0},input_rms_nm{0},independent_input_rms_nm{0},iv_condition{0};
    double training_rms_rad{std::numeric_limits<double>::quiet_NaN()};
    double validation_rms_rad{std::numeric_limits<double>::quiet_NaN()},prior_validation_rms_rad{std::numeric_limits<double>::quiet_NaN()};
    double measured_span_rad{0},position_noise_rad{0},feedback_age_max_s{0};
    double training_span_rad{0},validation_error_limit_rad{0};
    double observed_encoder_step_rad{0},feedback_interval_min_s{0},feedback_interval_max_s{0};
    double predicted_cost_before{0},predicted_cost_after{0},applied_step_fraction{0};
    std::size_t delay_samples{0},samples{0},training_samples{0},validation_samples{0},iv_rank{0};
    std::size_t duplicate_feedback{0},rejected_samples{0};
    std::size_t command_samples{0},resampled_samples{0};
    std::size_t fit_attempts{0},rank_rejections{0},plausibility_rejections{0},prediction_rejections{0};
    std::string structure{"arx2_separate_input_coefficients"};
    std::size_t parameter_count{5};
};
struct OnlineTrajectoryReport {
    Gains gains;
    std::vector<OnlineTrajectoryAxisModel> models;
    std::vector<std::string> reasons;
    bool changed{false},reference_only{true};
};

namespace online_trajectory_detail {
inline std::uint64_t mix(std::uint64_t x) {
    x+=0x9e3779b97f4a7c15ULL;x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;
    x=(x^(x>>27))*0x94d049bb133111ebULL;return x^(x>>31);
}
inline double code(std::uint64_t index,std::size_t axis) {
    return (mix(index+0x100000001b3ULL*(axis+1))&1)?1.0:-1.0;
}
inline double smooth(double value) {value=std::clamp(value,0.0,1.0);return value*value*(3-2*value);}
struct Point {double t{0},q{0},v{0},u{0},d{0},r{0},rv{0},ff{0},P{0},D{0};};
inline double rms(const Eigen::VectorXd& x) {return x.size()?std::sqrt(x.squaredNorm()/x.size()):0;}
inline double nearest_gain(double x,double step,double maximum) {
    return quantize_gain(std::max(0.0,x)+(step>0?.5*step:0),step,maximum);
}
inline bool known_row(const Sample& s) {
    return s.actual_gains_known && s.feedforward_known && s.independent_disturbance_known &&
        std::isfinite(s.time_s) && std::isfinite(s.reference) && std::isfinite(s.reference_velocity) &&
        std::isfinite(s.actual) && std::isfinite(s.velocity) &&
        std::isfinite(s.actual_kp) && s.actual_kp>=0 && std::isfinite(s.actual_kd) && s.actual_kd>=0 &&
        std::isfinite(s.applied_feedforward_nm) && std::isfinite(s.independent_disturbance_nm) &&
        std::isfinite(s.command_time_s) && std::isfinite(s.feedback_time_s);
}
inline std::vector<Point> points(const std::vector<Sample>& rows,const OnlineTrajectoryOptions& options,
    OnlineTrajectoryAxisModel& out) {
    std::vector<const Sample*> fresh,commands;
    if(rows.empty())return {};
    const double begin=rows.back().time_s-options.window_s;
    std::uint64_t previous_sequence=0;
    const auto reset=[&](){fresh.clear();commands.clear();previous_sequence=0;};
    for(const auto& s:rows) {
        if(s.time_s<begin)continue;
        const double age=s.command_time_s-s.feedback_time_s;
        // A clipped independent signal is no longer the prescribed exogenous
        // experiment. Start a new causal history after any such interruption.
        const bool input_modified=s.independent_input_planned_known &&
            (!std::isfinite(s.planned_independent_disturbance_nm) ||
             std::abs(s.planned_independent_disturbance_nm-s.independent_disturbance_nm)>1e-10);
        if(!known_row(s) || s.limited || input_modified || age<-.002 || age>.06) {
            ++out.rejected_samples;reset();continue;
        }
        if(!commands.empty() && (s.command_time_s<commands.back()->command_time_s ||
            s.command_time_s-commands.back()->command_time_s>.06)) {
            ++out.rejected_samples;reset();
        }
        // A repeated feedback frame is not a new output measurement, but the
        // host may have sent a different command while holding that feedback.
        // Preserve those command writes in the zero-order-held input history.
        if(!commands.empty() && s.command_time_s==commands.back()->command_time_s)commands.back()=&s;
        else commands.push_back(&s);
        out.feedback_age_max_s=std::max(out.feedback_age_max_s,age);
        if(!s.new_measurement || (s.feedback_sequence && s.feedback_sequence==previous_sequence) ||
            (!fresh.empty() && s.feedback_time_s==fresh.back()->feedback_time_s)) {
            ++out.duplicate_feedback;continue;
        }
        if(!fresh.empty() && (s.feedback_time_s<fresh.back()->feedback_time_s ||
            s.feedback_time_s-fresh.back()->feedback_time_s>.06)) {
            ++out.rejected_samples;reset();commands.push_back(&s);
        }
        if(s.feedback_sequence)previous_sequence=s.feedback_sequence;
        fresh.push_back(&s);
    }
    out.samples=fresh.size();out.command_samples=commands.size();
    if(fresh.size()<options.minimum_fresh_samples || commands.size()<2 ||
        fresh.back()->feedback_time_s-fresh.front()->feedback_time_s<options.minimum_window_s)return {};
    std::vector<double> intervals,positions;
    for(const auto* s:fresh)positions.push_back(s->actual);
    // Resolution must come from raw fresh positions, never interpolated ones;
    // a stationary noise window with one encoder code supplies no resolution.
    out.observed_encoder_step_rad=observed_encoder_step(std::move(positions));
    for(std::size_t i=1;i<fresh.size();++i)intervals.push_back(fresh[i]->feedback_time_s-fresh[i-1]->feedback_time_s);
    std::sort(intervals.begin(),intervals.end());const double observed=intervals[intervals.size()/2];
    if(!std::isfinite(observed) || observed<=0)return {};
    out.feedback_interval_min_s=intervals.front();out.feedback_interval_max_s=intervals.back();
    const double dt=std::max(options.model_sample_s,observed);out.dt_s=dt;
    const double start=std::max(commands.front()->command_time_s,fresh.front()->feedback_time_s);
    const double end=std::min(commands.back()->command_time_s,fresh.back()->feedback_time_s);
    if(end-start<options.minimum_window_s)return {};
    std::vector<Point> result;std::size_t command=0,feedback=1;
    // Only this local model uses a uniform grid. Output interpolation is
    // bracketed by real, distinct feedback timestamps, without extrapolation.
    // Integrate the host command proxy on its actual write intervals; this is
    // still a proxy, not an estimate of the embedded actuator's actual torque.
    for(std::size_t k=1;;++k) {
        const double t=start+k*dt;if(t>end+1e-10)break;
        while(feedback+1<fresh.size() && fresh[feedback]->feedback_time_s<t)++feedback;
        const auto& lo=*fresh[feedback-1];const auto& hi=*fresh[feedback];
        const double alpha=std::clamp((t-lo.feedback_time_s)/(hi.feedback_time_s-lo.feedback_time_s),0.0,1.0);
        Point p;p.t=t;p.q=lo.actual+alpha*(hi.actual-lo.actual);p.v=lo.velocity+alpha*(hi.velocity-lo.velocity);
        double from=t-dt;
        while(command+1<commands.size() && commands[command+1]->command_time_s<=from)++command;
        while(from<t-1e-12) {
            const auto& x=*commands[command];
            const double until=command+1<commands.size()?std::min(t,commands[command+1]->command_time_s):t;
            const double weight=until-from;
            p.u+=weight*(x.actual_kp*(x.reference-x.actual)+x.actual_kd*(x.reference_velocity-x.velocity)+
                x.applied_feedforward_nm+x.independent_disturbance_nm);
            p.d+=weight*x.independent_disturbance_nm;from=until;
            if(command+1<commands.size() && commands[command+1]->command_time_s<=from)++command;
        }
        p.u/=dt;p.d/=dt;const auto& x=*commands[command];
        p.r=x.reference;p.rv=x.reference_velocity;p.ff=x.applied_feedforward_nm;p.P=x.actual_kp;p.D=x.actual_kd;
        result.push_back(p);
    }
    out.resampled_samples=result.size();if(result.size()<50)return {};
    return result;
}
inline double predict_step(const OnlineTrajectoryAxisModel& m,double q,double previous,double u,double old_u) {
    return m.position_origin_rad+m.arx_a1*(q-m.position_origin_rad)+m.arx_a2*(previous-m.position_origin_rad)+
        m.arx_b0*u+m.arx_b1*old_u+m.bias;
}
inline double validation_error(const OnlineTrajectoryAxisModel& model,const std::vector<Point>& p,std::size_t begin) {
    double square=0;std::size_t count=0;
    const auto d=model.delay_samples;
    // Several free-running steps, rather than almost-perfect one-step position
    // persistence, decide whether the data support local controller redesign.
    const std::size_t horizon=std::max<std::size_t>(3,std::min<std::size_t>(12,static_cast<std::size_t>(std::ceil(.16/model.dt_s))));
    for(std::size_t start=std::max(begin,d+2);start+2<p.size();start+=horizon) {
        double previous=p[start-2].q,q=p[start-1].q;
        for(std::size_t k=start;k<std::min(p.size(),start+horizon);++k) {
            const double predicted=predict_step(model,q,previous,p[k-d].u,p[k-d-1].u);
            if(!std::isfinite(predicted) || std::abs(predicted-p[k].q)>10)return std::numeric_limits<double>::infinity();
            previous=q;q=predicted;square+=std::pow(predicted-p[k].q,2);++count;
        }
    }
    return count?std::sqrt(square/count):std::numeric_limits<double>::infinity();
}
inline OnlineTrajectoryAxisModel fit(const std::vector<Point>& p,double inertia,const NoiseEstimate& noise,
    const OnlineTrajectoryOptions& options,OnlineTrajectoryAxisModel result) {
    result.prior_inertia=inertia;result.position_origin_rad=p.front().q;
    result.position_noise_rad=std::max({noise.rms_rad,noise.encoder_step_rad/std::sqrt(12.0),
        result.observed_encoder_step_rad/std::sqrt(12.0)});
    double dlo=p.front().d,dhi=dlo,qlo=p.front().q,qhi=qlo,input_square=0,dmean=0;
    for(const auto& s:p){dlo=std::min(dlo,s.d);dhi=std::max(dhi,s.d);qlo=std::min(qlo,s.q);qhi=std::max(qhi,s.q);input_square+=s.u*s.u;dmean+=s.d;}
    result.measured_span_rad=qhi-qlo;result.input_rms_nm=std::sqrt(input_square/p.size());dmean/=p.size();
    double dsquare=0;for(const auto& s:p)dsquare+=std::pow(s.d-dmean,2);
    result.independent_input_rms_nm=std::sqrt(dsquare/p.size());
    if(!std::isfinite(result.independent_input_rms_nm) || result.independent_input_rms_nm<=1e-10 || dhi-dlo<1e-9) {
        result.reason="no_known_independent_excitation_model_remains_prior";return result;
    }
    const double dt=result.dt_s,prior_b=.5*dt*dt/inertia;
    const std::size_t training_end=std::max<std::size_t>(30,p.size()*3/4);
    double training_low=p.front().q,training_high=training_low;
    for(std::size_t k=0;k<training_end;++k){training_low=std::min(training_low,p[k].q);training_high=std::max(training_high,p[k].q);}
    result.training_span_rad=training_high-training_low;
    const std::size_t max_delay=std::min<std::size_t>(8,static_cast<std::size_t>(std::floor(options.maximum_delay_s/dt)));
    // Auxiliary, measurement-noise-free response to the independent input is
    // an instrument, never an extra physical motion or fitted feedback error.
    std::vector<double> auxiliary_q(p.size(),0),auxiliary_v(p.size(),0),auxiliary_u(p.size(),0);
    for(std::size_t k=1;k<p.size();++k) {
        const double u=p[k].d-p[k].P*auxiliary_q[k-1]-p[k].D*auxiliary_v[k-1];auxiliary_u[k]=u;
        auxiliary_q[k]=auxiliary_q[k-1]+dt*auxiliary_v[k-1]+.5*dt*dt*u/inertia;
        auxiliary_v[k]=auxiliary_v[k-1]+dt*u/inertia;
    }
    double best_score=std::numeric_limits<double>::infinity(),failed_score=best_score;OnlineTrajectoryAxisModel best=result;
    std::size_t maximum_rank=0,attempts=0,rank_rejections=0,plausibility_rejections=0,prediction_rejections=0;
    bool full_rank_structure=false;
    for(std::size_t delay=0;delay<=max_delay;++delay) {
        const std::size_t first=std::max<std::size_t>(delay+10,12);
        if(training_end<=first+20 || p.size()-training_end<10)continue;
        // Adjacent smoothed inputs need not support two independent numerator
        // coefficients. Compare a tied ZOH-shaped structure on the same held
        // out data instead of inventing an identified parameter from a prior.
        for(bool tied_input:{false,true}) {
        const std::size_t count=training_end-first;const Eigen::Index parameters=tied_input?4:5,instruments=13;
        ++attempts;
        Eigen::MatrixXd X(count,parameters),Z(count,instruments);Eigen::VectorXd y(count),weights(count);
        for(std::size_t k=first;k<training_end;++k) {
            const auto row=static_cast<Eigen::Index>(k-first);
            if(tied_input)X.row(row)<<p[k-1].q-p[k-2].q,p[k-1].q-result.position_origin_rad,
                .5*(p[k-delay].u+p[k-delay-1].u),1;
            else X.row(row)<<p[k-1].q-p[k-2].q,p[k-1].q-result.position_origin_rad,p[k-delay].u,p[k-delay-1].u,1;
            y[row]=p[k].q-2*p[k-1].q+p[k-2].q;
            Z(row,0)=auxiliary_q[k-1]-auxiliary_q[k-2];Z(row,1)=auxiliary_q[k-1];
            Z(row,2)=auxiliary_u[k-delay];Z(row,3)=auxiliary_u[k-delay-1];
            for(std::size_t lag=0;lag<8;++lag)Z(row,4+lag)=p[k-delay-lag].d;
            Z(row,12)=1;
            // A finite forgetting horizon, expressed in seconds, prevents a
            // different model sample rate from changing the retained history.
            weights[row]=std::exp((p[k].t-p[training_end-1].t)/options.window_s);
        }
        Eigen::VectorXd xs(parameters),zs(instruments);
        for(Eigen::Index j=0;j<parameters;++j){xs[j]=std::max(1e-12,std::sqrt(X.col(j).squaredNorm()/count));X.col(j)/=xs[j];}
        for(Eigen::Index j=0;j<instruments;++j){zs[j]=std::max(1e-12,std::sqrt(Z.col(j).squaredNorm()/count));Z.col(j)/=zs[j];}
        const Eigen::MatrixXd weightedZ=Z.array().colwise()*weights.array();
        const Eigen::MatrixXd G=weightedZ.transpose()*X/count;
        const Eigen::VectorXd h=weightedZ.transpose()*y/count;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(G,Eigen::ComputeThinU|Eigen::ComputeThinV);
        const auto singular=svd.singularValues();
        const double cutoff=std::max(1e-9,1e-6*singular[0]);std::size_t rank=0;
        for(Eigen::Index j=0;j<singular.size();++j)rank+=singular[j]>cutoff;
        maximum_rank=std::max(maximum_rank,rank);
        if(rank<static_cast<std::size_t>(parameters)) {++rank_rejections;continue;}
        full_rank_structure=true;
        const double condition=singular[0]/singular[parameters-1];
        Eigen::VectorXd prior(parameters);
        if(tied_input)prior<<0,0,2*prior_b,0;else prior<<0,0,prior_b,prior_b,0;
        prior=prior.cwiseProduct(xs);
        // The prior regularizes a local experiment; it is not success evidence.
        const double regularization=1e-6*std::max(1e-12,G.squaredNorm()/parameters);
        const Eigen::MatrixXd normal=G.transpose()*G+regularization*Eigen::MatrixXd::Identity(parameters,parameters);
        const Eigen::VectorXd fitted=normal.ldlt().solve(G.transpose()*h+regularization*prior).cwiseQuotient(xs);
        if(!fitted.allFinite()){++plausibility_rejections;continue;}
        OnlineTrajectoryAxisModel candidate=result;candidate.delay_samples=delay;candidate.delay_s=delay*dt;
        candidate.structure=tied_input?"arx2_tied_zoh_input_coefficient":"arx2_separate_input_coefficients";
        candidate.parameter_count=static_cast<std::size_t>(parameters);
        candidate.iv_rank=rank;candidate.iv_condition=condition;
        candidate.arx_a1=2+fitted[0]+fitted[1];candidate.arx_a2=-1-fitted[0];
        candidate.arx_b0=tied_input?.5*fitted[2]:fitted[2];candidate.arx_b1=tied_input?.5*fitted[2]:fitted[3];
        candidate.bias=fitted[tied_input?3:4];
        candidate.training_samples=count;
        candidate.training_rms_rad=rms(X*(fitted.cwiseProduct(xs))-y);
        // Positive local actuation and bounded sampling coefficients are model
        // plausibility requirements, not servo-accuracy acceptance criteria.
        const double b=candidate.arx_b0+candidate.arx_b1;
        if(b<=0 || candidate.arx_b0<-.2*b || candidate.arx_b1<-.2*b ||
           b<.02*(2*prior_b) || b>50*(2*prior_b) ||
           std::abs(candidate.arx_a1-2)>.8 || std::abs(candidate.arx_a2+1)>.8) {
            ++plausibility_rejections;if(!best.valid && best.training_samples==0)best=candidate;continue;
        }
        candidate.validation_samples=p.size()-training_end;
        candidate.validation_rms_rad=validation_error(candidate,p,training_end);
        OnlineTrajectoryAxisModel baseline=candidate;baseline.arx_a1=2;baseline.arx_a2=-1;
        baseline.arx_b0=baseline.arx_b1=prior_b;
        // A constant residual load is estimated without pretending the URDF
        // gravity/feedforward declaration has been experimentally calibrated.
        double bias=0;for(std::size_t k=first;k<training_end;++k)
            bias+=p[k].q-2*p[k-1].q+p[k-2].q-prior_b*(p[k-delay].u+p[k-delay-1].u);
        baseline.bias=bias/count;candidate.prior_validation_rms_rad=validation_error(baseline,p,training_end);
        const double noise_allowance=6*result.position_noise_rad;
        // A new disturbance in held-out data must not enlarge its own error
        // allowance. Scale the prediction test from the training output only.
        const double usable_error=std::max(noise_allowance,.2*result.training_span_rad);
        candidate.validation_error_limit_rad=usable_error;
        if(!std::isfinite(candidate.validation_rms_rad) || candidate.validation_rms_rad>usable_error ||
           candidate.validation_rms_rad>std::max(noise_allowance,1.1*candidate.prior_validation_rms_rad)) {
            ++prediction_rejections;
            if(!best.valid && candidate.validation_rms_rad<failed_score){best=candidate;failed_score=candidate.validation_rms_rad;}
            continue;
        }
        const double score=candidate.validation_rms_rad+.01*delay*std::max(noise_allowance,candidate.validation_rms_rad);
        const bool equivalent=best.valid && std::abs(score-best_score)<=result.position_noise_rad &&
            candidate.parameter_count<best.parameter_count;
        if(score<best_score || equivalent){candidate.valid=true;candidate.reason="held_out_local_iv_model_supported";best=std::move(candidate);best_score=score;}
        }
    }
    best.fit_attempts=attempts;best.rank_rejections=rank_rejections;
    best.plausibility_rejections=plausibility_rejections;best.prediction_rejections=prediction_rejections;
    if(!best.valid) {
        if(!best.training_samples)best.iv_rank=maximum_rank;
        best.reason=!full_rank_structure?"independent_input_not_informative_for_local_model":
            (prediction_rejections?"local_iv_model_not_supported_by_next_window_prediction":"local_iv_model_not_physically_plausible");
    }
    return best;
}
inline bool future_at(const std::vector<OnlineTrajectoryReference>& future,std::size_t axis,double t,double& r,double& v,double& ff) {
    if(future.empty() || t<future.front().time_s-1e-6 || t>future.back().time_s+1e-6)return false;
    auto it=std::lower_bound(future.begin(),future.end(),t,[](const auto& p,double value){return p.time_s<value;});
    if(it==future.end())it=std::prev(future.end());
    const auto* hi=&*it;const auto* lo=it==future.begin()?hi:&*std::prev(it);
    if(axis>=hi->position.size() || axis>=hi->velocity.size() || axis>=hi->feedforward_nm.size() ||
       axis>=lo->position.size() || axis>=lo->velocity.size() || axis>=lo->feedforward_nm.size())return false;
    const double alpha=hi->time_s>lo->time_s?std::clamp((t-lo->time_s)/(hi->time_s-lo->time_s),0.0,1.0):0;
    r=lo->position[axis]+alpha*(hi->position[axis]-lo->position[axis]);
    v=lo->velocity[axis]+alpha*(hi->velocity[axis]-lo->velocity[axis]);
    ff=lo->feedforward_nm[axis]+alpha*(hi->feedforward_nm[axis]-lo->feedforward_nm[axis]);
    return std::isfinite(r) && std::isfinite(v) && std::isfinite(ff);
}
inline Eigen::VectorXd rollout(const OnlineTrajectoryAxisModel& model,const std::vector<Point>& p,
    const std::vector<OnlineTrajectoryReference>& future,std::size_t axis,double P,double D,double qscale,double vscale,double uscale,
    const OnlineTrajectoryOptions& options) {
    const double available=future.empty()?0:future.back().time_s-p.back().t;
    const std::size_t count=static_cast<std::size_t>(std::floor(std::min(options.horizon_s,available)/model.dt_s));
    if(count<5 || p.size()<model.delay_samples+3)return {};
    const bool compliant=options.use==OnlineTrajectoryUse::Compliant;
    Eigen::VectorXd residual((compliant?5:3)*count);double previous=p[p.size()-2].q,q=p.back().q,v=p.back().v;
    std::deque<double> inputs;for(std::size_t k=p.size()-model.delay_samples-2;k<p.size();++k)inputs.push_back(p[k].u-p[k].d);
    auto forced_inputs=inputs;double forced_previous=previous,forced_q=q,forced_v=v;
    double known_peak=0;for(const auto& s:p)known_peak=std::max(known_peak,std::abs(s.d));
    const double known_amplitude=std::min(known_peak,std::sqrt(2.0)*model.independent_input_rms_nm);
    const double scope=options.compliant_displacement_rad;
    const double motion_floor=std::max(6*model.position_noise_rad,1e-6);
    const double desired_yield=std::max(motion_floor,.25*scope);
    const double recovery_scale=std::max(motion_floor,std::min(options.compliant_recovery_error_rad,.25*scope));
    const std::size_t pulse_samples=std::max<std::size_t>(2,count/2);
    double old_command=inputs.back();
    for(std::size_t k=0;k<count;++k) {
        double r,rv,ff;if(!future_at(future,axis,p.back().t+(k+1)*model.dt_s,r,rv,ff))return {};
        const double command=P*(r-q)+D*(rv-v)+ff;inputs.push_back(command);
        const std::size_t index=inputs.size()-1-model.delay_samples;
        const double next=predict_step(model,q,previous,inputs[index],inputs[index-1]);
        const double nv=(next-q)/model.dt_s;
        if(!std::isfinite(next) || !std::isfinite(nv) || std::abs(next-p.back().q)>10)return {};
        // Moving-reference accuracy remains useful for compliance, but the
        // purpose also has its own independent-input yield/release objective.
        // Its reference is never silently replaced with rigid tracking alone.
        residual[3*k]=(compliant?.25:1.0)*(next-r)/qscale;
        residual[3*k+1]=(compliant?.06:.12)*(nv-rv)/vscale;
        residual[3*k+2]=.025*(command-old_command)/uscale;
        if(compliant) {
            // Paired predictions use identical initial state, path and gains.
            // The extra smooth input is no larger than input already observed
            // in this fitted window; it is a prediction, not a motor command
            // or a claim of actual contact-force/compliance measurement.
            const double pulse=k<pulse_samples?known_amplitude*std::pow(std::sin(pi*(k+.5)/pulse_samples),2):0;
            const double forced_command=P*(r-forced_q)+D*(rv-forced_v)+ff+pulse;
            forced_inputs.push_back(forced_command);const auto fi=forced_inputs.size()-1-model.delay_samples;
            const double forced_next=predict_step(model,forced_q,forced_previous,forced_inputs[fi],forced_inputs[fi-1]);
            const double forced_nv=(forced_next-forced_q)/model.dt_s;
            if(!std::isfinite(forced_next) || !std::isfinite(forced_nv))return {};
            const double displacement=forced_next-next,delta_velocity=forced_nv-nv;
            const auto offset=3*count+2*k;
            if(k<pulse_samples) {
                residual[offset]=std::max(0.0,desired_yield-displacement)/desired_yield;
                residual[offset+1]=std::max(0.0,std::abs(displacement)-scope)/scope;
            } else {
                // Yielding inside the unloaded-recovery band is not a rigid
                // positioning error. Only terminal release speed is scored;
                // ordinary motion during the unloading transient is expected.
                residual[offset]=std::max(0.0,std::abs(displacement)-recovery_scale)/recovery_scale;
                const bool terminal=k+std::max<std::size_t>(3,count/8)>=count;
                residual[offset+1]=terminal?.12*std::max(0.0,std::abs(delta_velocity)-options.compliant_final_speed_rad_s)/
                    std::max(motion_floor/options.horizon_s,scope/options.horizon_s):0;
            }
            forced_previous=forced_q;forced_q=forced_next;forced_v=forced_nv;
        }
        previous=q;q=next;v=nv;old_command=command;
    }
    return residual;
}
} // namespace online_trajectory_detail

// Pure planning helper. Remaining negative/positive room is separate, so a
// mechanical endpoint permits an inward, one-sided known signal. Commands
// remain subject to the existing position/velocity/effort Safety path.
inline double online_trajectory_injection(double time_s,std::size_t axis,double inertia,double kp,double kd,
    double effort_headroom_nm,double lower_room_rad,double upper_room_rad,double feedback_dt_s,double duration_s) {
    for(double value:{time_s,inertia,kp,kd,effort_headroom_nm,lower_room_rad,upper_room_rad,feedback_dt_s,duration_s})
        if(!std::isfinite(value))return 0;
    if(time_s<0 || time_s>=duration_s || inertia<=0 || kp<0 || kd<0 || effort_headroom_nm<=0 ||
       lower_room_rad<0 || upper_room_rad<0 || feedback_dt_s<=0 || duration_s<=0)return 0;
    const double room=std::max(lower_room_rad,upper_room_rad);if(room<=0)return 0;
    const double nominal=kp>0?std::sqrt(kp/inertia):(kd>0?kd/inertia:0);if(nominal<=0)return 0;
    const double omega=std::min(.5*nominal,.15/feedback_dt_s);
    const double chip_s=std::max(4*feedback_dt_s,pi/(2*std::max(omega,1e-9)));
    const auto chip=static_cast<std::uint64_t>(std::floor(time_s/chip_s));
    const double phase=time_s/chip_s-chip;
    const double signal=online_trajectory_detail::code(chip,axis)+(online_trajectory_detail::code(chip+1,axis)-
        online_trajectory_detail::code(chip,axis))*online_trajectory_detail::smooth(phase);
    const double fade_s=std::min(.8,.15*duration_s);
    const double fade=online_trajectory_detail::smooth(time_s/fade_s)*online_trajectory_detail::smooth((duration_s-time_s)/fade_s);
    const double negative=std::min(.08*effort_headroom_nm,.08*(kp+kd*omega)*lower_room_rad);
    const double positive=std::min(.08*effort_headroom_nm,.08*(kp+kd*omega)*upper_room_rad);
    // Interpolate unequal endpoints; the offset is known, recorded, and the
    // IV model estimates its bias. No symmetric-motion prerequisite exists.
    return fade*(.5*(positive-negative)+.5*(positive+negative)*signal);
}

inline OnlineTrajectoryReport online_trajectory_candidate(const Gains& current,
    const std::vector<std::vector<Sample>>& rows_by_axis,const std::vector<OnlineTrajectoryReference>& future,
    const std::vector<double>& inertia,const std::vector<ModelAxisLimits>& limits,const std::vector<NoiseEstimate>& noise,
    const OnlineTrajectoryOptions& options=OnlineTrajectoryOptions{}) {
    using namespace online_trajectory_detail;
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || rows_by_axis.size()!=n || inertia.size()!=n || limits.size()!=n || noise.size()!=n)
        throw std::invalid_argument("online trajectory tuning joint mapping mismatch");
    if(!std::isfinite(options.window_s) || options.window_s<=0 || !std::isfinite(options.update_period_s) || options.update_period_s<=0 ||
       !std::isfinite(options.horizon_s) || options.horizon_s<=0 || !std::isfinite(options.model_sample_s) || options.model_sample_s<=0 ||
       !std::isfinite(options.minimum_window_s) || options.minimum_window_s<=0 || options.minimum_window_s>options.window_s ||
       !std::isfinite(options.maximum_delay_s) || options.maximum_delay_s<0 ||
       !std::isfinite(options.compliant_displacement_rad) || options.compliant_displacement_rad<=0 ||
       !std::isfinite(options.compliant_recovery_error_rad) || options.compliant_recovery_error_rad<=0 ||
       !std::isfinite(options.compliant_final_speed_rad_s) || options.compliant_final_speed_rad_s<=0 ||
       !std::isfinite(options.maximum_relative_gain_step) || options.maximum_relative_gain_step<=0 || options.maximum_relative_gain_step>.25)
        throw std::invalid_argument("invalid online trajectory tuning window or trust region");
    double last_future=-std::numeric_limits<double>::infinity();
    for(const auto& f:future){if(!std::isfinite(f.time_s) || f.time_s<=last_future)throw std::invalid_argument("unordered online future reference");last_future=f.time_s;}
    OnlineTrajectoryReport out;out.gains=current;out.models.resize(n);out.reasons.resize(n);
    for(std::size_t axis=0;axis<n;++axis) {
        const auto& bound=limits[axis];
        for(double value:{inertia[axis],bound.effort_budget,bound.kp_max,bound.kd_max})
            if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid online physical prior or execution bound");
        for(double value:{current.kp[axis],current.kd[axis],bound.grid.kp_step,bound.grid.kd_step,noise[axis].rms_rad,noise[axis].encoder_step_rad})
            if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid online measured gains, grid or noise");
        if(current.kp[axis]>bound.kp_max || current.kd[axis]>bound.kd_max)throw std::invalid_argument("online current gains exceed protocol");
        auto& model=out.models[axis];auto p=points(rows_by_axis[axis],options,model);
        if(p.empty()){out.reasons[axis]=model.reason;continue;}
        model=fit(p,inertia[axis],noise[axis],options,model);
        if(!model.valid){out.reasons[axis]=model.reason;continue;}
        const double P=current.kp[axis],D=current.kd[axis];
        double reflo=p.front().r,refhi=reflo,vscale=0,E=0,V=0;
        for(const auto& s:p){reflo=std::min(reflo,s.r);refhi=std::max(refhi,s.r);vscale=std::max(vscale,std::abs(s.rv));E=std::max(E,std::abs(s.r-s.q));V=std::max(V,std::abs(s.rv-s.v));}
        const double qscale=std::max({refhi-reflo,model.measured_span_rad,6*model.position_noise_rad,1e-5});
        vscale=std::max({vscale,qscale/options.horizon_s,1e-5});const double uscale=std::max(model.input_rms_nm,1e-6);
        const auto r=rollout(model,p,future,axis,P,D,qscale,vscale,uscale,options);
        if(!r.size()){out.reasons[axis]="insufficient_future_taught_reference_model_updates_continue";continue;}
        model.predicted_cost_before=r.squaredNorm()/r.size();
        const double ps=std::max(P,bound.grid.kp_step),ds=std::max({D,.05*2*std::sqrt(std::max(P,bound.grid.kp_step)*inertia[axis]),bound.grid.kd_step,1e-8});
        const double ph=std::max(.01*ps,1e-8),dh=.01*ds;
        const auto rp=rollout(model,p,future,axis,P+ph,D,qscale,vscale,uscale,options);
        const auto rd=rollout(model,p,future,axis,P,D+dh,qscale,vscale,uscale,options);
        if(rp.size()!=r.size() || rd.size()!=r.size()){out.reasons[axis]="local_predictor_outside_supported_neighborhood";continue;}
        Eigen::MatrixXd J(r.size(),2);J.col(0)=(rp-r)*(ps/ph);J.col(1)=(rd-r)*(ds/dh);
        if(P==0)J.col(0).setZero(); // Zero-stiffness drag remains zero stiffness.
        const Eigen::Matrix2d H=J.transpose()*J;
        const double damping=std::max(1e-8,1e-3*H.trace());
        Eigen::Vector2d step=(H+damping*Eigen::Matrix2d::Identity()).ldlt().solve(-J.transpose()*r);
        if(!step.allFinite()){out.reasons[axis]="unresolved_local_gain_sensitivity";continue;}
        const double fraction=options.maximum_relative_gain_step;
        for(Eigen::Index j=0;j<2;++j)step[j]=std::clamp(step[j],-fraction,fraction);
        if(P==0)step[0]=0;
        const double before_envelope=P*E+D*V;
        bool changed=false;double best_cost=r.squaredNorm();
        for(double scale:{1.0,.5,.25,.125}) {
            const auto projected=[&](double value,double delta,double grid,double maximum) {
                double next=nearest_gain(std::clamp(value+delta,0.0,maximum),grid,maximum);
                // Relative steps below half a protocol code used to freeze
                // small gains forever. Evaluate the adjacent code in the
                // computed descent direction, with the same cost and effort
                // checks. A proposal is not automatically an applied change.
                if(next==value && grid>0 && std::abs(delta)>1e-10)
                    next=nearest_gain(std::clamp(value+std::copysign(grid,delta),0.0,maximum),grid,maximum);
                return next;
            };
            const double pp=P==0?0:projected(P,scale*ps*step[0],bound.grid.kp_step,bound.kp_max);
            const double dd=projected(D,scale*ds*step[1],bound.grid.kd_step,bound.kd_max);
            // The two adjacent codes need not improve together. Compare the
            // coupled step and each one-coordinate projection on the same
            // reference, without changing any measured feedback or limits.
            for(const auto& candidate:std::array<std::pair<double,double>,3>{{{pp,dd},{pp,D},{P,dd}}}) {
                const double cp=candidate.first,cd=candidate.second;
                if(cp==P && cd==D)continue;
                if(std::abs(cp-P)>std::max(fraction*ps,bound.grid.kp_step)+1e-10 ||
                   std::abs(cd-D)>std::max(fraction*ds,bound.grid.kd_step)+1e-10)continue;
                if(cp*E+cd*V>std::max(bound.effort_budget,before_envelope)+1e-10)continue;
                const auto trial=rollout(model,p,future,axis,cp,cd,qscale,vscale,uscale,options);
                if(trial.size()!=r.size() || trial.squaredNorm()>=best_cost*(1-1e-8))continue;
                best_cost=trial.squaredNorm();
                out.gains.kp[axis]=cp;out.gains.kd[axis]=cd;out.changed=changed=true;
                model.predicted_cost_after=best_cost/trial.size();model.applied_step_fraction=scale;
                out.reasons[axis]="online_iv_predictive_small_pd_step_reference_only_external_gain_ramp_required";
            }
        }
        if(!changed)out.reasons[axis]="local_model_has_no_resolved_admissible_gain_step_keep_measured_group_and_continue";
    }
    return out;
}
} // namespace serial_arm::servo_tuning
