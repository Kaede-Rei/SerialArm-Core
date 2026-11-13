#pragma once

#include <Eigen/Dense>
#include <serial_arm/dynamics/gain_measurement_center.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {
constexpr double pi=3.14159265358979323846;
struct Requirements {
    double peak_error_rad{pi/1800},rms_error_rad{0.004},velocity_rms_rad_s{0.08};
    double overshoot_rad{0.004},settling_error_rad{0.003},settling_time_s{0.7};
    double hold_error_rad{0.004},hold_velocity_rad_s{0.025};
    double amplitude_rad{0.008},probe_duration_s{8.0},tail_s{1.5};
    double local_range_rad{0.04};
    double noise_rms_rad{0},encoder_step_rad{0},velocity_noise_floor{0};
    bool stationary_probe{false},motion_probe{false};
    std::array<double,3> frequencies_hz{0.35,0.70,1.20};
    std::size_t max_candidates{3};
    // Offset of the excitation's interior center from its fresh measured
    // origin. It shares the waveform's fade, so no position/velocity step is
    // introduced when a bounded experiment must stay on the observed side.
    double reference_bias_rad{0};
};
inline constexpr std::size_t motion_attempt_limit=5;
struct Sample {
    double time_s{0},reference{0},reference_velocity{0},actual{0},velocity{0};
    double residual{0};
    double measured_torque_nm{std::numeric_limits<double>::quiet_NaN()};
    bool limited{false},new_measurement{false},residual_valid{false};
    // reference/velocity are the accepted MIT setpoints. These fields describe
    // the command actually issued, on the same monotonic clock as time_s.
    double applied_feedforward_nm{std::numeric_limits<double>::quiet_NaN()};
    double independent_disturbance_nm{0};
    double planned_independent_disturbance_nm{0};
    bool independent_input_planned_known{false};
    double command_time_s{std::numeric_limits<double>::quiet_NaN()};
    double feedback_time_s{std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t feedback_sequence{0};
    bool feedforward_known{false},independent_disturbance_known{false};
    double actual_kp{0},actual_kd{0};bool actual_gains_known{false};
    double reference_sample_time_s{0};
    bool reference_sample_time_known{false};
};
struct Metrics {
    std::size_t samples{0},measurement_samples{0},limited_samples{0},tail_samples{0};
    double rms_error{0},peak_error{0},velocity_rms{0},overshoot{0},settling_s{0};
    double residual_bias{0},residual_noise_rms{0},tail_drift{0},tail_peak_velocity{0},tail_error_rad{0};
    double reference_span{0},actual_span{0},response_rms_ratio{0};
    double applied_input_peak_nm{0};
    std::size_t residual_samples{0},tail_reversals{0};
    bool continuous{true},settled{false},oscillating{false},observable{false},passed{false};
    // New records explicitly scope the accuracy target to a final, fixed
    // target window. Legacy peak/tail fields keep their original all-cycle
    // meanings; absence of these endpoint fields is not a historical pass.
    double endpoint_peak_error_rad{0},endpoint_rms_error_rad{0},endpoint_bias_error_rad{0};
    double endpoint_window_start_s{0},endpoint_window_end_s{0};
    std::size_t endpoint_measurement_samples{0};
    bool endpoint_window_available{false},endpoint_passed{false};
    std::string endpoint_reason{"final_target_window_not_measured"};
};
struct NoiseEstimate {
    double rms_rad{0},encoder_step_rad{0},velocity_rms{0},velocity_peak{0},velocity_step{0};
    double span_rad{0},drift_rad{0};
    std::size_t samples{0};bool stationary{false};
};
struct MotionGuardResult {
    bool triggered{false};std::string reason{"no_uncommanded_oscillation"};
    std::size_t position_reversals{0},reference_reversals{0};
    double actual_travel_rad{0},reference_travel_rad{0},residual_peak_to_peak_rad{0},window_s{0};
    double effective_encoder_step_rad{0},hysteresis_rad{0},actual_peak_to_peak_rad{0},reference_peak_to_peak_rad{0};
    std::size_t fresh_position_codes{0},encoder_step_evidence_pairs{0};
    bool encoder_step_known{false};
    double completed_alternating_travel_rad{0},repeated_leg_min_rad{0},repeated_leg_max_rad{0};
    bool repeated_comparable_excursions{false};
    std::size_t independent_input_reversals{0},requested_motion_reversals{0};
    double independent_input_span_nm{0};bool independent_input_known{false};
    bool independent_input_frequencies_known{false},input_tone_response_fit_available{false},input_tone_response_explained{false};
    bool input_tone_unexplained_oscillation{false};
    double input_tone_response_r_squared{0},input_tone_response_condition_number{0},input_tone_residual_peak_to_peak_rad{0};
    std::string input_tone_explanation_reason{"not_evaluated"};
};
// Detect repeated position excursions which the accepted reference does not
// request. This is an execution guard, not a mechanical-resonance identifier;
// raw velocity codes and one dynamic overshoot cannot trigger it.
class CommandConditionedMotionGuard {
    Requirements req_;NoiseEstimate noise_;std::deque<Sample> rows_;std::uint64_t sequence_{0};
    ObservedPositionStep observed_position_step_;
    std::array<double,3> independent_input_frequencies_{};
    bool independent_input_frequencies_known_{false},known_input_oscillation_seen_{false};
    double last_known_input_oscillation_s_{0};
    static bool comparable_legs(const std::vector<double>& legs,double hysteresis,double& minimum,double& maximum) {
        for(std::size_t start=0;start+4<=legs.size();++start) {
            const auto first=legs.begin()+static_cast<std::ptrdiff_t>(start);
            const auto bounds=std::minmax_element(first,first+4);
            minimum=*bounds.first;maximum=*bounds.second;
            if(minimum>=std::max(4*hysteresis,.25*maximum))return true;
        }
        return false;
    }
    double unknown_resolution_hysteresis() const {
        // A final accuracy target is not an encoder-resolution measurement.
        // Until raw codes establish their spacing, use the bounded experiment
        // motion scale. It does not change commanded motion or Safety limits.
        double scale=std::numeric_limits<double>::infinity();
        for(const double value:{req_.amplitude_rad,req_.local_range_rad})
            if(std::isfinite(value) && value>0)scale=std::min(scale,value);
        return std::isfinite(scale)?std::max(1e-5,.25*scale):1e-5;
    }
public:
    explicit CommandConditionedMotionGuard(const Requirements& req=Requirements{},const NoiseEstimate& noise=NoiseEstimate{}):req_(req),noise_(noise){}
    // Only the actual independent-input plan may establish this context.
    // Requirements' default tones do not imply that any torque was applied.
    void set_independent_input_frequencies(const std::array<double,3>& frequencies) {
        independent_input_frequencies_known_=true;
        for(std::size_t k=0;k<frequencies.size();++k)
            if(!std::isfinite(frequencies[k]) || frequencies[k]<=0 || (k && frequencies[k]<=frequencies[k-1]))
                independent_input_frequencies_known_=false;
        independent_input_frequencies_=frequencies;known_input_oscillation_seen_=false;
    }
    void update_noise(const NoiseEstimate& measured) {
        if(std::isfinite(measured.encoder_step_rad) && measured.encoder_step_rad>0)
            noise_.encoder_step_rad=noise_.encoder_step_rad>0?std::min(noise_.encoder_step_rad,measured.encoder_step_rad):measured.encoder_step_rad;
        if(std::isfinite(measured.rms_rad) && measured.rms_rad>=0)noise_.rms_rad=measured.rms_rad;
    }
    MotionGuardResult observe(const Sample& row) {
        MotionGuardResult out;
        if(!row.new_measurement || (sequence_ && (!row.feedback_sequence || row.feedback_sequence<=sequence_)))return out;
        if(!std::isfinite(row.time_s) || !std::isfinite(row.actual) || !std::isfinite(row.reference))return out;
        if(std::isfinite(row.command_time_s) && std::isfinite(row.feedback_time_s) &&
           std::abs(row.command_time_s-row.feedback_time_s)>.06){out.reason="stale_feedback_not_motion_evidence";return out;}
        if(!rows_.empty() && row.time_s<=rows_.back().time_s)return out;
        if(!rows_.empty() && row.time_s-rows_.back().time_s>.06)rows_.clear();
        observed_position_step_.observe(row.actual,row.feedback_sequence);
        const double observed_step=observed_position_step_.qualified_step();
        // The first large movement must never become an arbitrarily large
        // noise allowance. Learning an unknown spacing may only reduce the
        // existing motion threshold; a declared/earlier step only decreases.
        if(observed_step>0 && (noise_.encoder_step_rad>0 || observed_step<=unknown_resolution_hysteresis()/3))
            noise_.encoder_step_rad=noise_.encoder_step_rad>0?std::min(noise_.encoder_step_rad,observed_step):observed_step;
        sequence_=row.feedback_sequence;rows_.push_back(row);
        while(!rows_.empty() && rows_.front().time_s<row.time_s-.25)rows_.pop_front();
        out.effective_encoder_step_rad=noise_.encoder_step_rad;
        out.encoder_step_known=noise_.encoder_step_rad>0;
        out.fresh_position_codes=observed_position_step_.distinct_positions();
        out.encoder_step_evidence_pairs=observed_position_step_.step_evidence_pairs();
        out.independent_input_frequencies_known=independent_input_frequencies_known_;
        const double hysteresis=std::max({3*noise_.encoder_step_rad,6*noise_.rms_rad,
            noise_.encoder_step_rad>0?0:unknown_resolution_hysteresis(),1e-5});
        out.hysteresis_rad=hysteresis;
        if(rows_.size()<20 || rows_.back().time_s-rows_.front().time_s<.15)return out;
        out.window_s=rows_.back().time_s-rows_.front().time_s;
        std::vector<double> actual_legs;
        const auto excursions=[&](bool actual,std::vector<double>* legs=nullptr,const Eigen::VectorXd* positions=nullptr) {
            double extreme=positions?(*positions)[0]:actual?rows_.front().actual:rows_.front().reference,anchor=extreme,travel=0;int direction=0;std::size_t reversals=0,index=0;
            for(const auto& s:rows_) {
                const double q=positions?(*positions)[static_cast<Eigen::Index>(index)]:actual?s.actual:s.reference;++index;
                if(!direction){if(std::abs(q-anchor)>=hysteresis){direction=q>anchor?1:-1;extreme=q;}}
                else if((q-extreme)*direction>=0)extreme=q;
                else if((extreme-q)*direction>=hysteresis){const double leg=std::abs(extreme-anchor);travel+=leg;
                    if(legs)legs->push_back(leg);
                    anchor=extreme;extreme=q;direction=-direction;++reversals;}
            }
            return std::pair<std::size_t,double>{reversals,travel+std::abs(extreme-anchor)};
        };
        const auto actual=excursions(true,&actual_legs),reference=excursions(false);
        out.position_reversals=actual.first;out.reference_reversals=reference.first;
        out.actual_travel_rad=actual.second;out.reference_travel_rad=reference.second;
        out.independent_input_known=std::all_of(rows_.begin(),rows_.end(),[](const Sample& s){
            return s.independent_disturbance_known && std::isfinite(s.independent_disturbance_nm);
        });
        if(out.independent_input_known) {
            double low_input=rows_.front().independent_disturbance_nm,high_input=low_input;
            for(const auto& s:rows_){low_input=std::min(low_input,s.independent_disturbance_nm);high_input=std::max(high_input,s.independent_disturbance_nm);}
            out.independent_input_span_nm=high_input-low_input;
            // Known exogenous torque requests motion even with a fixed
            // position reference. Compare only its dimensionless turn count;
            // torque travel is never compared to joint displacement. A zero
            // or DC input cannot excuse recurring position excursions.
            if(out.independent_input_span_nm>1e-12) {
                const double input_hysteresis=std::max(1e-12,.001*out.independent_input_span_nm);
                double anchor=rows_.front().independent_disturbance_nm,extreme=anchor;int direction=0;
                for(const auto& s:rows_) {
                    const double input=s.independent_disturbance_nm;
                    if(!direction){if(std::abs(input-anchor)>=input_hysteresis){direction=input>anchor?1:-1;extreme=input;}}
                    else if((input-extreme)*direction>=0)extreme=input;
                    else if((extreme-input)*direction>=input_hysteresis){extreme=input;direction=-direction;++out.independent_input_reversals;}
                }
            }
        }
        if(out.independent_input_known && out.independent_input_reversals>0) {
            known_input_oscillation_seen_=true;last_known_input_oscillation_s_=row.time_s;
        }
        out.requested_motion_reversals=std::max(out.reference_reversals,out.independent_input_reversals);
        double low=std::numeric_limits<double>::infinity(),high=-low,actual_low=low,actual_high=high,reference_low=low,reference_high=high;
        for(const auto& s:rows_){const double e=s.actual-s.reference;low=std::min(low,e);high=std::max(high,e);
            actual_low=std::min(actual_low,s.actual);actual_high=std::max(actual_high,s.actual);
            reference_low=std::min(reference_low,s.reference);reference_high=std::max(reference_high,s.reference);}
        out.actual_peak_to_peak_rad=actual_high-actual_low;out.reference_peak_to_peak_rad=reference_high-reference_low;
        out.residual_peak_to_peak_rad=high-low;
        for(const double leg:actual_legs)out.completed_alternating_travel_rad+=leg;
        // Four tiny turns around one large transition are not four cycles of
        // that transition's amplitude. Require two completed back-and-forth
        // excursions at a resolved, comparable scale; slow growth/decay within
        // a factor four over these half-cycles is retained as motion evidence.
        out.repeated_comparable_excursions=comparable_legs(actual_legs,hysteresis,out.repeated_leg_min_rad,out.repeated_leg_max_rad);
        out.triggered=actual.first>=4 && actual.first>=out.requested_motion_reversals+3 &&
            actual.second>2.5*reference.second+8*hysteresis && out.actual_peak_to_peak_rad>4*hysteresis &&
            out.completed_alternating_travel_rad>=2*out.actual_peak_to_peak_rad && out.repeated_comparable_excursions;
        // A stable resonant response can keep ringing at an applied tone while
        // its known input fades or is just released. Only when the ordinary
        // guard would trip, test this explicit plan-based explanation. It is
        // neither a stability certificate nor a change to raw acceptance data.
        if(out.triggered && independent_input_frequencies_known_ && out.independent_input_known && known_input_oscillation_seen_) {
            const double release_window=std::isfinite(req_.tail_s)?std::max(.25,req_.tail_s):.25;
            if(row.time_s-last_known_input_oscillation_s_<=release_window) {
                bool clock_known=true;double previous_command=-1e300,previous_feedback=-1e300;
                for(const auto& s:rows_) {
                    if(!std::isfinite(s.command_time_s) || !std::isfinite(s.feedback_time_s) ||
                       s.command_time_s<=previous_command || s.feedback_time_s<=previous_feedback ||
                       std::abs(s.command_time_s-s.feedback_time_s)>.06)clock_known=false;
                    previous_command=s.command_time_s;previous_feedback=s.feedback_time_s;
                }
                out.input_tone_explanation_reason="known_input_tone_clock_evidence_missing";
                if(clock_known) {
                    const auto count=static_cast<Eigen::Index>(rows_.size());
                    Eigen::MatrixXd design(count,8);Eigen::VectorXd actual_positions(count);
                    const double origin=rows_.front().feedback_time_s,span=rows_.back().feedback_time_s-origin;
                    for(Eigen::Index k=0;k<count;++k) {
                        const auto& s=rows_[static_cast<std::size_t>(k)];const double time=s.feedback_time_s-origin;
                        design(k,0)=1;design(k,1)=time/span-.5;actual_positions[k]=s.actual;
                        for(unsigned j=0;j<3;++j){const double angle=2*pi*independent_input_frequencies_[j]*time;
                            design(k,2+2*j)=std::sin(angle);design(k,3+2*j)=std::cos(angle);}
                    }
                    for(Eigen::Index j=0;j<design.cols();++j){const double norm=design.col(j).norm();if(norm>1e-12)design.col(j)/=norm;}
                    const Eigen::JacobiSVD<Eigen::MatrixXd> fit(design,Eigen::ComputeThinU|Eigen::ComputeThinV);
                    const auto& values=fit.singularValues();
                    out.input_tone_response_condition_number=values[values.size()-1]>1e-12?values[0]/values[values.size()-1]:1e30;
                    out.input_tone_explanation_reason="known_input_tone_fit_ill_conditioned";
                    if(out.input_tone_response_condition_number<=1e4) {
                        const Eigen::VectorXd residual=actual_positions-design*fit.solve(actual_positions);
                        const double total=(actual_positions.array()-actual_positions.mean()).matrix().squaredNorm();
                        out.input_tone_response_fit_available=true;
                        out.input_tone_response_r_squared=total>1e-18?std::clamp(1-residual.squaredNorm()/total,0.0,1.0):0;
                        out.input_tone_residual_peak_to_peak_rad=residual.maxCoeff()-residual.minCoeff();
                        std::vector<double> residual_legs;const auto remainder=excursions(true,&residual_legs,&residual);
                        double completed=0,minimum=0,maximum=0;for(const double leg:residual_legs)completed+=leg;
                        out.input_tone_unexplained_oscillation=remainder.first>=4 && remainder.second>8*hysteresis &&
                            out.input_tone_residual_peak_to_peak_rad>4*hysteresis && completed>=2*out.input_tone_residual_peak_to_peak_rad &&
                            comparable_legs(residual_legs,hysteresis,minimum,maximum);
                        out.input_tone_response_explained=out.input_tone_response_r_squared>=.95 && !out.input_tone_unexplained_oscillation;
                        out.input_tone_explanation_reason=out.input_tone_response_explained?"known_input_tones_explain_position_motion_not_an_acceptance_pass":
                            out.input_tone_unexplained_oscillation?"resolved_repeated_motion_remains_outside_known_input_tones":"known_input_tones_do_not_explain_position_motion";
                        if(out.input_tone_response_explained)out.triggered=false;
                    }
                }
            }
        }
        if(out.triggered)out.reason="repeated_position_oscillation_not_requested_by_reference";
        else if(out.input_tone_response_explained)out.reason="position_motion_explained_by_known_independent_input_tones";
        return out;
    }
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
        // A symmetric oscillation has zero slope but still spans many codes
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
    double initial_error_rad{0},positive_travel_rad{0},negative_travel_rad{0},alignment_delay_s{0};
    bool bidirectional{false},qualified{false},quantization_boundary{false};std::string reason{"insufficient_measurements"};
};

inline void assess_endpoint_window(Metrics& m,const std::vector<Sample>& rows,
                                  const Requirements& req,double excitation_end_s) {
    // A positioning score uses the raw maximum error over the final half
    // second, after the planned endpoint dwell. It never waits for the error
    // to meet the score: a complete, inaccurate window is a measured failure.
    constexpr double window_s=.5,maximum_gap_s=.06;
    if(rows.empty() || !std::isfinite(excitation_end_s) || excitation_end_s<0 ||
       !std::isfinite(req.tail_s) || req.tail_s<window_s ||
       !std::isfinite(req.peak_error_rad) || req.peak_error_rad<=0)return;
    const double end=rows.back().time_s,begin=end-window_s;
    if(!std::isfinite(end) || end<excitation_end_s+req.tail_s-maximum_gap_s || begin<excitation_end_s) {
        m.endpoint_reason="final_target_window_incomplete";return;
    }
    m.endpoint_window_start_s=begin;m.endpoint_window_end_s=end;
    const double target=rows.back().reference;
    if(!std::isfinite(target)){m.endpoint_reason="final_target_nonfinite";return;}
    // This is numerical equality of an accepted stationary command, not a
    // noise tolerance or the user's positioning target. It cannot turn a
    // slowly moving reference into an endpoint measurement.
    const double reference_tolerance=1e-9*std::max(1.0,std::abs(target));
    double sum=0,square=0,previous_fresh=begin,last_time=begin;
    std::size_t window_samples=0;std::uint64_t sequence=0;
    bool seen_row=false;
    for(const auto& row:rows) {
        if(row.time_s<begin)continue;
        if(!std::isfinite(row.time_s) || !std::isfinite(row.reference) ||
           !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual)) {
            m.endpoint_reason="final_target_nonfinite";return;
        }
        if((seen_row && row.time_s<=last_time) || row.time_s-last_time>maximum_gap_s) {
            m.endpoint_reason="final_target_feedback_discontinuous";return;
        }
        seen_row=true;last_time=row.time_s;
        if(std::abs(row.reference-target)>reference_tolerance ||
           std::abs(row.reference_velocity)>reference_tolerance) {
            m.endpoint_reason="final_target_reference_not_stationary";return;
        }
        if(row.limited){m.endpoint_reason="final_target_limited";return;}
        const double error=row.actual-target;
        m.endpoint_peak_error_rad=std::max(m.endpoint_peak_error_rad,std::abs(error));
        sum+=error;square+=error*error;++window_samples;
        if(!row.new_measurement)continue;
        if(row.time_s-previous_fresh>maximum_gap_s ||
           (sequence && row.feedback_sequence<=sequence)) {
            m.endpoint_reason="final_target_feedback_discontinuous";return;
        }
        if((std::isfinite(row.command_time_s) || std::isfinite(row.feedback_time_s)) &&
           (!std::isfinite(row.command_time_s) || !std::isfinite(row.feedback_time_s) ||
            std::abs(row.command_time_s-row.feedback_time_s)>maximum_gap_s)) {
            m.endpoint_reason="final_target_feedback_stale";return;
        }
        previous_fresh=row.time_s;sequence=row.feedback_sequence;++m.endpoint_measurement_samples;
    }
    if(m.endpoint_measurement_samples<20 || end-previous_fresh>maximum_gap_s || !window_samples) {
        m.endpoint_reason="final_target_feedback_insufficient";return;
    }
    m.endpoint_rms_error_rad=std::sqrt(square/window_samples);
    m.endpoint_bias_error_rad=sum/window_samples;
    m.endpoint_window_available=true;
    m.endpoint_passed=m.endpoint_peak_error_rad<req.peak_error_rad;
    m.endpoint_reason=m.endpoint_passed?"final_target_window_qualified":"final_target_error_not_qualified";
}

inline Metrics assess(const std::vector<Sample>& rows,const Requirements& req,double excitation_end_s) {
    Metrics m;if(rows.empty())return m;
    double sum=0,vs=0,lo=rows.front().reference,hi=lo,actual_lo=rows.front().actual,actual_hi=actual_lo;
    double tail_lo=1e30,tail_hi=-1e30,rs=0,rss=0,last_bad=excitation_end_s;
    int last_sign=0;double last_measurement=rows.front().time_s;
    double reference_sum=0,reference_sq=0,actual_sum=0,actual_sq=0;
    std::size_t excitation_samples=0;
    for(std::size_t k=0;k<rows.size();++k) {
        const auto& s=rows[k];
        if(!std::isfinite(s.time_s)||!std::isfinite(s.reference)||!std::isfinite(s.reference_velocity)||!std::isfinite(s.actual)||!std::isfinite(s.velocity))return Metrics{};
        if(k && (s.time_s<=rows[k-1].time_s || s.time_s-rows[k-1].time_s>0.06))m.continuous=false;
        if(s.new_measurement){if(s.time_s-last_measurement>0.06)m.continuous=false;last_measurement=s.time_s;++m.measurement_samples;}
        const double e=s.actual-s.reference;sum+=e*e;vs+=std::pow(s.velocity-s.reference_velocity,2);
        m.peak_error=std::max(m.peak_error,std::abs(e));++m.samples;
        m.limited_samples+=s.limited;
        lo=std::min(lo,s.reference);hi=std::max(hi,s.reference);
        actual_lo=std::min(actual_lo,s.actual);actual_hi=std::max(actual_hi,s.actual);
        if(s.new_measurement && s.time_s>=0.7 && s.time_s<excitation_end_s-0.7) {
            // Remove the posture offset before measuring motion. A stationary
            // encoder must not pass merely because the requested motion is small
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
    // actual position drift or an oscillating tail
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
    assess_endpoint_window(m,rows,req,excitation_end_s);
    m.passed=m.observable && m.samples>=100 && m.measurement_samples>=100 && m.continuous &&
        m.limited_samples==0 && m.endpoint_window_available && m.endpoint_passed && !m.oscillating;
    return m;
}
inline MotionObservation observe_motion(const std::vector<Sample>& rows,const Requirements& req,
                                         const NoiseEstimate& noise) {
    MotionObservation out;out.noise_rms_rad=noise.rms_rad;
    if(rows.empty())return out;
    const auto metrics=assess(rows,req,req.probe_duration_s);
    std::vector<double> u,y,positions,selected_times;
    std::vector<const Sample*> fresh;
    const double reference_center=rows.front().reference;
    out.initial_error_rad=rows.front().actual-reference_center;
    for(const auto& row:rows)if(row.new_measurement) {
        out.reference_positive=std::max(out.reference_positive,row.reference-reference_center);
        out.reference_negative=std::max(out.reference_negative,reference_center-row.reference);
        out.actual_positive=std::max(out.actual_positive,row.actual-reference_center);
        out.actual_negative=std::max(out.actual_negative,reference_center-row.actual);
        positions.push_back(row.actual);fresh.push_back(&row);
        if(row.time_s>=.7 && row.time_s<req.probe_duration_s-.7) {
            u.push_back(row.reference-reference_center);y.push_back(row.actual-reference_center);selected_times.push_back(row.time_s);
        }
    }
    out.encoder_step_rad=std::max(noise.encoder_step_rad,observed_encoder_step(positions));
    out.response_rms_ratio=metrics.response_rms_ratio;
    const double required=std::max({6*out.encoder_step_rad,6*noise.rms_rad,
        .2*std::min(out.reference_positive,out.reference_negative)});
    out.minimum_motion_span_rad=required;
    // Establish correlated motion without claiming dynamic tracking quality.
    // A small causal shift allows the low-frequency starting response to lag;
    // the later independent FRF still uses the original strict phase criteria
    for(std::size_t lag=0;lag<std::min<std::size_t>(100,u.size()/4);lag+=4) {
        double su=0,sy=0,suu=0,syy=0,suy=0;const double count=static_cast<double>(u.size()-lag);
        for(std::size_t i=lag;i<u.size();++i){const double a=u[i-lag],b=y[i];su+=a;sy+=b;suu+=a*a;syy+=b*b;suy+=a*b;}
        const double a=suu-su*su/count,b=syy-sy*sy/count,c=suy-su*sy/count;
        if(a>1e-12 && b>1e-12 && c>0) {
            const double coherence=std::clamp(c*c/(a*b),0.0,1.0);
            if(coherence>out.coherence){out.coherence=coherence;out.alignment_delay_s=selected_times[lag]-selected_times.front();}
        }
    }
    // The first actual sample may retain the preceding window's static
    // tracking error. Measure real forward/backward travel on significant
    // monotonic reference segments, using the same small causal alignment as
    // the correlation test. Neither a DC offset nor the initial pose defines
    // whether an encoder has moved in both commanded directions
    const auto record_segment=[&](std::size_t first,std::size_t last,int direction) {
        const double input=fresh[last]->reference-fresh[first]->reference;
        if(std::abs(input)<std::max(2*required,.1*req.amplitude_rad))return;
        const auto at=[&](double time){return std::lower_bound(fresh.begin(),fresh.end(),time,
            [](const Sample* row,double value){return row->time_s<value;});};
        const auto begin=at(fresh[first]->time_s+out.alignment_delay_s),end=at(fresh[last]->time_s+out.alignment_delay_s);
        if(begin==fresh.end() || end==fresh.end())return;
        const double travel=(*end)->actual-(*begin)->actual;
        if(direction>0)out.positive_travel_rad=std::max(out.positive_travel_rad,travel);
        else if(direction<0)out.negative_travel_rad=std::max(out.negative_travel_rad,-travel);
    };
    if(fresh.size()>=2) {
        std::size_t first=0,last=0;int direction=0;
        for(std::size_t k=1;k<fresh.size() && fresh[k]->time_s<=req.probe_duration_s;++k) {
            const double delta=fresh[k]->reference-fresh[k-1]->reference;
            const int next=delta>1e-8?1:delta< -1e-8?-1:0;
            if(next && direction && next!=direction){record_segment(first,k-1,direction);first=k-1;}
            if(next)direction=next;
            last=k;
        }
        record_segment(first,last,direction);
    }
    out.bidirectional=required>0 && out.positive_travel_rad>=required && out.negative_travel_rad>=required;
    out.quantization_boundary=!out.bidirectional && out.encoder_step_rad>0 &&
        std::min(out.positive_travel_rad,out.negative_travel_rad)>0 &&
        std::min(out.positive_travel_rad,out.negative_travel_rad)+out.encoder_step_rad>=required && out.response_rms_ratio>=.2;
    if(metrics.measurement_samples<100 || !metrics.continuous)out.reason="insufficient_measurements";
    else if(metrics.limited_samples)out.reason="limited";
    else if(std::max(out.positive_travel_rad,out.negative_travel_rad)<required)out.reason="no_motion";
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
    // A cutoff is reported only when measurements bracket it, never extrapolated
    double observed_cutoff_hz{std::numeric_limits<double>::quiet_NaN()};
    std::vector<FrequencyPoint> points;
    bool tracking_valid{false},disturbance_valid{false},feedforward_known{false};
    bool disturbance_goal_evaluated{false},disturbance_goal_met{false};
    bool sustained_oscillation{false},dynamic_overshoot{false};
    double inertia_estimate{0},effective_stiffness_nm_rad{0},viscous_damping_nm_s_rad{0};
    double low_frequency_compliance_rad_nm{std::numeric_limits<double>::quiet_NaN()};
    double reference_lag_s{0},direction_bias_rad{0},tracking_peak_error_rad{0};
    double velocity_error_rad_per_rad_s{0};bool direction_evidence_valid{false};
    std::size_t fresh_samples{0},duplicate_feedback{0};
    std::string model_definition{"unidentified"},reason{"insufficient_measurements"};
    std::vector<std::string> missing_evidence;
    double physical_stiffness_nm_rad{0},nominal_physical_stiffness_nm_rad{0};
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
    // This compatibility entry point has no knowledge of the MIT velocity
    // reference or executed feedforward. Its FRF is valid reference-tracking
    // evidence; fitting a zero-free second-order plant would mislabel the MIT
    // numerator zero as physical bandwidth/damping. Use identify_mit for poles.
    out.tracking_valid=true;out.model_definition="reference_tracking_frf";
    out.reason="mit_execution_parameters_required";
    out.missing_evidence.push_back("mit_velocity_reference_and_applied_feedforward");

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
        // or replace a qualified configured baseline with a theoretical ratio
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
        // a candidate increment, while the actual response and hard limits judge it
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
    const double displacement=req.amplitude_rad*q+req.reference_bias_rad;
    return {envelope*displacement,envelope*req.amplitude_rad*v+de*displacement};
}
} // namespace serial_arm::servo_tuning
