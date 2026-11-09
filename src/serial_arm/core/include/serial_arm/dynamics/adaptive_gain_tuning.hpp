#pragma once

#include "serial_arm/dynamics/servo_gain_tuning.hpp"
#include <optional>

namespace serial_arm::servo_tuning {

inline std::optional<std::vector<double>> measured_probe_center(const std::vector<double>& actual,
    const std::vector<std::uint64_t>& received_ns,std::uint64_t now_ns,
    const std::vector<std::uint64_t>& before={}) {
    if(actual.empty() || received_ns.size()!=actual.size() || (!before.empty() && before.size()!=actual.size()))return std::nullopt;
    for(std::size_t i=0;i<actual.size();++i)if(!std::isfinite(actual[i]) || received_ns[i]==0 || received_ns[i]>now_ns ||
        now_ns-received_ns[i]>30000000 || (!before.empty() && received_ns[i]<=before[i]))return std::nullopt;
    return actual;
}

struct GainCommandGrid {double kp_step{0},kd_step{0};};
// Values are mapped joint-side protocol capabilities, never user PD overrides.
struct ModelAxisLimits {
    double position_margin_rad{0},max_velocity{0},max_acceleration{0};
    double effort_budget{0},kp_max{0},kd_max{0};
    GainCommandGrid grid;
};
inline std::uint64_t gain_grid_code(double value,double step,double ceiling) {
    if(step<=0)return 0;
    const auto count=static_cast<std::uint64_t>(std::llround(ceiling/step));
    return static_cast<std::uint64_t>(std::llround(std::clamp(static_cast<float>(value)/static_cast<float>(ceiling),0.0f,1.0f)*static_cast<float>(count)));
}
inline double quantize_gain(double value,double step,double ceiling,bool positive=false) {
    if(!std::isfinite(value) || value<0 || !std::isfinite(step) || step<0 || !std::isfinite(ceiling) || ceiling<=0)
        throw std::invalid_argument("invalid gain protocol grid");
    value=std::min(value,ceiling);if(step==0)return value;
    const auto count=static_cast<std::uint64_t>(std::llround(ceiling/step));
    if(!count)throw std::invalid_argument("gain protocol has no positive code");
    double ratio=value/step;const double nearest=std::round(ratio);
    if(std::abs(ratio-nearest)<=4*std::numeric_limits<float>::epsilon()*std::max(1.0,std::abs(ratio)))ratio=nearest;
    auto code=static_cast<std::uint64_t>(std::floor(ratio));
    if(positive && value>0)code=std::max<std::uint64_t>(1,static_cast<std::uint64_t>(std::ceil(ratio)));
    code=std::min(code,count);if(!code)return 0;
    if(code==count)return ceiling;
    // The transport and Robot both encode the nearest integer code. Returning
    // the decoded code value is idempotent across float transport conversion.
    const double wire=code*step;
    if(wire>ceiling || gain_grid_code(wire,step,ceiling)!=code)throw std::invalid_argument("gain protocol grid cannot be represented");
    return wire;
}
inline Gains quantize_gains(const Gains& gains,const std::vector<ModelAxisLimits>& limits) {
    if(gains.kp.size()!=limits.size() || gains.kd.size()!=limits.size())throw std::invalid_argument("gain grid mapping mismatch");
    Gains out=gains;for(std::size_t i=0;i<limits.size();++i){out.kp[i]=quantize_gain(gains.kp[i],limits[i].grid.kp_step,limits[i].kp_max,gains.kp[i]>0);
        out.kd[i]=quantize_gain(gains.kd[i],limits[i].grid.kd_step,limits[i].kd_max);}
    return out;
}
struct GainWorkingDomain {
    double kp_max{0},kd_max{0},reference_error_bound_rad{0},velocity_error_bound_rad{0},effort_budget{0};
};
inline GainWorkingDomain gain_working_domain(const ModelAxisLimits& bounds,double error_bound,double velocity_error_bound) {
    if(!std::isfinite(error_bound) || error_bound<=0 || !std::isfinite(velocity_error_bound) || velocity_error_bound<0)
        throw std::invalid_argument("gain working domain needs actual experiment error/velocity bounds");
    return {std::min(bounds.kp_max,bounds.effort_budget/error_bound),
        velocity_error_bound>0?std::min(bounds.kd_max,bounds.effort_budget/velocity_error_bound):bounds.kd_max,
        error_bound,velocity_error_bound,bounds.effort_budget};
}
inline std::pair<double,double> constrain_candidate(double kp,double kd,const ModelAxisLimits& bounds,
    double error_bound,double velocity_error_bound) {
    const auto domain=gain_working_domain(bounds,error_bound,velocity_error_bound);
    kp=std::clamp(kp,0.0,domain.kp_max);kd=std::clamp(kd,0.0,domain.kd_max);
    const double effort=kp*error_bound+kd*velocity_error_bound;
    if(effort>bounds.effort_budget){const double scale=bounds.effort_budget/effort;kp*=scale;kd*=scale;}
    kp=quantize_gain(kp,bounds.grid.kp_step,bounds.kp_max);kd=quantize_gain(kd,bounds.grid.kd_step,bounds.kd_max);
    // Guard floating-point arithmetic at an active effort boundary.
    for(unsigned correction=0;correction<4 && kp*error_bound+kd*velocity_error_bound>bounds.effort_budget;++correction) {
        if(kp>0)kp=bounds.grid.kp_step>0?quantize_gain(std::max(0.0,kp-bounds.grid.kp_step),bounds.grid.kp_step,bounds.kp_max):
            std::nextafter(std::max(0.0,(bounds.effort_budget-kd*velocity_error_bound)/error_bound),0.0);
        else if(kd>0 && velocity_error_bound>0)kd=bounds.grid.kd_step>0?quantize_gain(std::max(0.0,kd-bounds.grid.kd_step),bounds.grid.kd_step,bounds.kd_max):
            std::nextafter(bounds.effort_budget/velocity_error_bound,0.0);
    }
    return {kp,kd};
}
struct ConstrainedMeasuredGainStep {
    Gains gains;std::vector<std::string> reasons;
    std::vector<double> predicted_effort_nm;bool changed{false};
};
// Constrain a local comparison around an actually measured complete controller.
// The error bounds describe that experiment, not the local probe's position
// range. A stiffness-increase proposal must not silently turn into a large
// decrease of both P and D through radial projection.
inline ConstrainedMeasuredGainStep constrain_measured_gain_step(const Gains& current,const Gains& requested,
    const std::vector<ModelAxisLimits>& limits,const std::vector<double>& error_bound_rad,
    const std::vector<double>& velocity_error_bound_rad) {
    const auto n=limits.size();
    if(!n || current.kp.size()!=n || current.kd.size()!=n || requested.kp.size()!=n || requested.kd.size()!=n ||
       error_bound_rad.size()!=n || velocity_error_bound_rad.size()!=n)throw std::invalid_argument("measured gain step mapping mismatch");
    ConstrainedMeasuredGainStep out;out.gains=current;out.reasons.resize(n);out.predicted_effort_nm.resize(n);
    for(std::size_t i=0;i<n;++i) {
        const auto& x=limits[i];const double E=error_bound_rad[i],V=velocity_error_bound_rad[i];
        for(double value:{x.effort_budget,x.kp_max,x.kd_max})
            if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid measured gain capability");
        (void)gain_working_domain(x,E,V);
        for(double value:{current.kp[i],current.kd[i],requested.kp[i],requested.kd[i]})
            if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid measured gain step");
        if(current.kp[i]>x.kp_max || current.kd[i]>x.kd_max)throw std::invalid_argument("measured controller outside protocol capability");
        double kp=std::min(requested.kp[i],x.kp_max),kd=std::min(requested.kd[i],x.kd_max);
        if(kp>=current.kp[i] && kd>=current.kd[i]) {
            if(current.kp[i]*E+current.kd[i]*V>x.effort_budget*(1+1e-10)) {
                kp=current.kp[i];kd=current.kd[i];out.reasons[i]="measured_error_envelope_has_no_effort_room_for_increase";
            } else {
                kp=quantize_gain(std::min(kp,std::max(0.0,(x.effort_budget-current.kd[i]*V)/E)),x.grid.kp_step,x.kp_max);
                kp=std::max(current.kp[i],kp);
                kd=quantize_gain(std::min(kd,V>0?std::max(0.0,(x.effort_budget-kp*E)/V):x.kd_max),x.grid.kd_step,x.kd_max);
                kd=std::max(current.kd[i],kd);
                if(kp*E+kd*V>x.effort_budget*(1+1e-10)){kp=current.kp[i];kd=current.kd[i];}
                out.reasons[i]=(kp<requested.kp[i] || kd<requested.kd[i])?
                    "measured_joint_effort_envelope_bounds_local_increase_preserve_measured_damping":"measured_joint_effort_envelope_allows_local_step";
            }
        } else {
            const auto bounded=constrain_candidate(kp,kd,x,E,V);kp=bounded.first;kd=bounded.second;
            out.reasons[i]="evidence_requested_feedback_reduction_within_joint_effort_envelope";
        }
        out.gains.kp[i]=kp;out.gains.kd[i]=kd;out.predicted_effort_nm[i]=kp*E+kd*V;
        out.changed=out.changed || kp!=current.kp[i] || kd!=current.kd[i];
    }
    return out;
}
struct ModelResponseDemand {
    std::vector<double> frequency_hz,velocity_peak,acceleration_peak,coupled_acceleration_torque,error_velocity_bound;
    std::vector<std::vector<std::vector<double>>> mass_matrix_samples;
    double peak_error_rad{pi/1800},damping_ratio{.7},settling_time_s{.7},designed_amplitude_rad{.008};
};
// A no-demonstration preview is a DESIGN TARGET, never an estimate of motor
// capability. Artificial URDF/policy v/a/effort limits do not enter this demand.
inline ModelResponseDemand recovery_response_demand(std::size_t joints,double target_error_rad,
    double settling_time_s,double designed_amplitude_rad=.008) {
    if(!joints || !std::isfinite(target_error_rad) || target_error_rad<=0 || !std::isfinite(settling_time_s) || settling_time_s<=0 ||
       !std::isfinite(designed_amplitude_rad) || designed_amplitude_rad<=0)throw std::invalid_argument("invalid recovery design targets");
    ModelResponseDemand out;out.peak_error_rad=target_error_rad;out.settling_time_s=settling_time_s;out.designed_amplitude_rad=designed_amplitude_rad;
    const double frequency=1/(4*settling_time_s),w=2*pi*frequency;
    out.frequency_hz.assign(joints,frequency);out.velocity_peak.assign(joints,designed_amplitude_rad*w);
    out.acceleration_peak.assign(joints,designed_amplitude_rad*w*w);
    out.error_velocity_bound.assign(joints,std::max(4*designed_amplitude_rad/settling_time_s,2*designed_amplitude_rad*w));
    return out;
}
inline ModelResponseDemand path_response_demand(const std::vector<std::vector<double>>& path,double dt,double target_error_rad) {
    if(path.size()<3 || path.front().empty() || !std::isfinite(dt) || dt<=0 || !std::isfinite(target_error_rad) || target_error_rad<=0)
        throw std::invalid_argument("model response demand needs an original valid path/timebase and error target");
    const auto n=path.front().size();ModelResponseDemand out;out.peak_error_rad=target_error_rad;
    out.frequency_hz.assign(n,0);out.velocity_peak.assign(n,0);out.acceleration_peak.assign(n,0);
    std::vector<double> velocity_square(n,0),acceleration_square(n,0);
    for(const auto& q:path){if(q.size()!=n)throw std::invalid_argument("path response mapping mismatch");for(double x:q)if(!std::isfinite(x))throw std::invalid_argument("nonfinite path response input");}
    for(std::size_t k=1;k+1<path.size();++k)for(std::size_t i=0;i<n;++i) {
        const double v=(path[k+1][i]-path[k-1][i])/(2*dt),a=(path[k+1][i]-2*path[k][i]+path[k-1][i])/(dt*dt);
        out.velocity_peak[i]=std::max(out.velocity_peak[i],std::abs(v));out.acceleration_peak[i]=std::max(out.acceleration_peak[i],std::abs(a));
        velocity_square[i]+=v*v;acceleration_square[i]+=a*a;
    }
    for(std::size_t i=0;i<n;++i)if(velocity_square[i]>1e-16)out.frequency_hz[i]=std::sqrt(acceleration_square[i]/velocity_square[i])/(2*pi);
    return out;
}
struct ModelInitialization {
    Gains gains;
    std::vector<double> inertia_min,inertia_max,frequency_hz;
    std::vector<double> task_frequency_hz,working_kp_max,working_kd_max,reference_error_bound_rad,velocity_error_bound_rad;
    std::vector<double> nominal_mode_frequency_hz,nominal_mode_damping_ratio;
    std::vector<double> unconstrained_kp,unconstrained_kd,pre_modal_kp,pre_modal_kd;
    std::vector<std::string> modal_adjustments;
    std::vector<std::vector<std::string>> constraints;
    std::string source{"urdf_task_response_and_protocol_grid"};
    bool measured{false};
};
inline void validate_model_limits(const ModelAxisLimits& x) {
    for(double v:{x.position_margin_rad,x.max_velocity,x.max_acceleration,x.effort_budget,x.kp_max,x.kd_max})
        if(!std::isfinite(v) || v<=0)throw std::invalid_argument("gain initialization requires finite positive mapped position, motion, effort and gain limits");
}
// Initial gains follow the requested damping curve, rather than radially
// scaling Kp/Kd (which changes zeta by sqrt(scale)). This only selects a safe
// first response experiment; it never becomes a Kp ceiling for later tuning.
inline std::pair<double,double> initial_model_pair(double inertia,double residual_stiffness,double desired_kp,
    double damping_ratio,const ModelAxisLimits& bounds,double error_bound,double velocity_bound) {
    const double available=bounds.effort_budget-velocity_bound*bounds.grid.kd_step;
    if(available<=0)throw std::invalid_argument("initial damping protocol code exceeds feedback effort");
    const double linear=2*damping_ratio*std::sqrt(inertia)*velocity_bound;
    const double root=2*available/(linear+std::sqrt(linear*linear+4*error_bound*available));
    const double kd_kp_limit=bounds.kd_max*bounds.kd_max/(4*damping_ratio*damping_ratio*inertia);
    const double cap=std::min({bounds.kp_max,root*root,kd_kp_limit});
    double kp=quantize_gain(std::min(desired_kp,cap),bounds.grid.kp_step,bounds.kp_max);
    if(kp<=residual_stiffness) {
        const double minimum=residual_stiffness+std::max(bounds.grid.kp_step,1e-9);
        kp=quantize_gain(minimum,bounds.grid.kp_step,bounds.kp_max,true);
    }
    if(kp>cap*(1+1e-10) || kp<=residual_stiffness)
        throw std::invalid_argument("no positive model gain code meets initial damping, residual gravity and effort constraints");
    const double kd=quantize_gain(2*damping_ratio*std::sqrt(inertia*kp),bounds.grid.kd_step,bounds.kd_max,true);
    if(kp*error_bound+kd*velocity_bound>bounds.effort_budget*(1+1e-10))
        throw std::invalid_argument("initial paired gain codes exceed feedback effort");
    return {kp,kd};
}
// The caller evaluates the current URDF at original taught poses using the
// configured controlled-joint mapping. No configured/historical gains enter.
inline ModelInitialization model_initialization(const std::vector<std::vector<double>>& inertia_samples,
    const std::vector<double>& gravity_gradient,const std::vector<ModelAxisLimits>& limits,
    const ModelResponseDemand& demand=ModelResponseDemand{}) {
    const auto n=limits.size();
    if(!n || inertia_samples.empty() || gravity_gradient.size()!=n || !std::isfinite(demand.peak_error_rad) || demand.peak_error_rad<=0 ||
       !std::isfinite(demand.damping_ratio) || demand.damping_ratio<=0 || !std::isfinite(demand.settling_time_s) || demand.settling_time_s<=0 ||
       !std::isfinite(demand.designed_amplitude_rad) || demand.designed_amplitude_rad<=0)
        throw std::invalid_argument("gain initialization requires a valid mapped URDF mass matrix and gravity gradient");
    for(const auto* field:{&demand.frequency_hz,&demand.velocity_peak,&demand.acceleration_peak,&demand.coupled_acceleration_torque,&demand.error_velocity_bound})
        if(!field->empty() && field->size()!=n)throw std::invalid_argument("model response demand mapping mismatch");
    ModelInitialization out;out.inertia_min.assign(n,std::numeric_limits<double>::infinity());out.inertia_max.assign(n,0);
    for(const auto& posture:inertia_samples) {
        if(posture.size()!=n)throw std::invalid_argument("gain initialization controlled joint mapping mismatch");
        for(std::size_t i=0;i<n;++i) {
            if(!std::isfinite(posture[i]) || posture[i]<=1e-9)throw std::invalid_argument("invalid URDF effective joint inertia");
            out.inertia_min[i]=std::min(out.inertia_min[i],posture[i]);out.inertia_max[i]=std::max(out.inertia_max[i],posture[i]);
        }
    }
    out.constraints.resize(n);
    for(std::size_t i=0;i<n;++i) {
        const auto& x=limits[i];validate_model_limits(x);
        if(!std::isfinite(gravity_gradient[i]))throw std::invalid_argument("invalid URDF gravity gradient");
        const double mass=out.inertia_max[i],gradient=std::abs(gravity_gradient[i]);
        const double amplitude=std::min(demand.designed_amplitude_rad,.2*x.position_margin_rad);
        if(amplitude<demand.designed_amplitude_rad)out.constraints[i].emplace_back("user_position_margin_constrains_design_probe");
        const double task_frequency=demand.frequency_hz.empty()?1/(4*demand.settling_time_s):demand.frequency_hz[i];
        const double acceleration=demand.acceleration_peak.empty()?demand.designed_amplitude_rad*std::pow(2*pi*task_frequency,2):demand.acceleration_peak[i];
        const double velocity=demand.velocity_peak.empty()?demand.designed_amplitude_rad*2*pi*task_frequency:demand.velocity_peak[i];
        const double coupled=demand.coupled_acceleration_torque.empty()?mass*acceleration:demand.coupled_acceleration_torque[i];
        for(double v:{acceleration,velocity,task_frequency,coupled})if(!std::isfinite(v) || v<0)throw std::invalid_argument("invalid path response demand");
        // The MIT velocity reference is retained. Demand comes from actual path
        // acceleration/response error, rather than one fixed bandwidth for all arms.
        const double w2=std::max({std::pow(2*pi*task_frequency,2),coupled/(mass*demand.peak_error_rad),
            std::pow(4/(demand.damping_ratio*demand.settling_time_s),2)});
        // Kd multiplies velocity ERROR. An entire axis' maximum speed is not
        // that error and is not a physical motor-response prior.
        const double requested_velocity_error=demand.error_velocity_bound.empty()?
            std::max(4*amplitude/demand.settling_time_s,4*pi*task_frequency*amplitude):demand.error_velocity_bound[i];
        if(!std::isfinite(requested_velocity_error) || requested_velocity_error<0)throw std::invalid_argument("invalid designed velocity-error bound");
        const double velocity_bound=std::min(x.max_velocity,requested_velocity_error);
        if(velocity_bound<requested_velocity_error)out.constraints[i].emplace_back("user_velocity_limit_constrains_error_guard");
        const auto domain=gain_working_domain(x,2*amplitude,velocity_bound);
        const double desired_kp=mass*w2+gradient;
        const double desired_kd=2*demand.damping_ratio*std::sqrt(mass*desired_kp);
        out.unconstrained_kp.push_back(desired_kp);out.unconstrained_kd.push_back(desired_kd);
        const auto gains=initial_model_pair(mass,gradient,desired_kp,demand.damping_ratio,x,2*amplitude,velocity_bound);
        if(x.grid.kp_step>0 && desired_kp<x.grid.kp_step)out.constraints[i].emplace_back("positive_protocol_gain_code");
        if(gains.first<=0)throw std::invalid_argument("model/path demand leaves no positive bounded gain");
        if(gains.first<desired_kp || gains.second<desired_kd)out.constraints[i].emplace_back("initial_response_demand_limited_by_effort_or_protocol");
        if(desired_kd>x.kd_max)out.constraints[i].emplace_back("kd_protocol_ceiling_does_not_freeze_kp");
        out.gains.kp.push_back(gains.first);out.gains.kd.push_back(gains.second);
        out.frequency_hz.push_back(std::sqrt(gains.first/mass)/(2*pi));out.task_frequency_hz.push_back(task_frequency);
        out.working_kp_max.push_back(domain.kp_max);out.working_kd_max.push_back(domain.kd_max);
        out.reference_error_bound_rad.push_back(domain.reference_error_bound_rad);out.velocity_error_bound_rad.push_back(domain.velocity_error_bound_rad);
    }
    std::vector<Eigen::MatrixXd> coupled_matrices;
    for(const auto& matrix:demand.mass_matrix_samples) {
        if(matrix.size()!=n)throw std::invalid_argument("coupled inertia mapping mismatch");
        Eigen::MatrixXd M(n,n);
        for(std::size_t i=0;i<n;++i){if(matrix[i].size()!=n)throw std::invalid_argument("coupled inertia mapping mismatch");for(std::size_t j=0;j<n;++j)M(i,j)=matrix[i][j];}
        if(!M.allFinite() || !M.isApprox(M.transpose(),1e-8) || Eigen::LLT<Eigen::MatrixXd>(M).info()!=Eigen::Success)throw std::invalid_argument("invalid positive-definite coupled inertia");
        coupled_matrices.push_back(std::move(M));
    }
    out.pre_modal_kp=out.gains.kp;out.pre_modal_kd=out.gains.kd;
    struct LimitingMode {double ratio{std::numeric_limits<double>::infinity()},w{0};std::size_t sample{0},mode{0};Eigen::VectorXd v;};
    const auto evaluate_modes=[&]() {
        LimitingMode limiting;out.nominal_mode_frequency_hz.clear();out.nominal_mode_damping_ratio.clear();
        for(std::size_t sample=0;sample<coupled_matrices.size();++sample) {
            const auto& M=coupled_matrices[sample];Eigen::VectorXd kp(n),kd(n);
            for(std::size_t i=0;i<n;++i){kp[i]=out.gains.kp[i];kd[i]=out.gains.kd[i];}
            Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(kp.asDiagonal().toDenseMatrix(),M);
            if(modes.info()!=Eigen::Success || modes.eigenvalues().minCoeff()<=0)throw std::invalid_argument("invalid coupled gain modes");
            for(std::size_t j=0;j<n;++j){const Eigen::VectorXd v=modes.eigenvectors().col(j);const double w=std::sqrt(modes.eigenvalues()[j]);
                const double damping=v.dot(kd.asDiagonal()*v)/(2*w);
                out.nominal_mode_frequency_hz.push_back(w/(2*pi));out.nominal_mode_damping_ratio.push_back(damping);
                if(damping<limiting.ratio)limiting={damping,w,sample,j,v};}
        }
        return limiting;
    };
    auto limiting=evaluate_modes();
    // Coordinate correction acts only through the limiting mode's actual
    // participation. A weak mode in one independent block cannot scale every
    // other joint's stiffness. The bounded iterations are pure model work.
    for(std::size_t iteration=0;iteration<32*n && limiting.ratio<demand.damping_ratio*(1-1e-6);++iteration) {
        const double deficit=2*limiting.w*(demand.damping_ratio-limiting.ratio);
        std::size_t selected=n;double best=0,new_value=0;std::string action;
        // Compare each correction by removed modal deficit per relative change.
        // Tiny participation cannot justify pegging an unrelated wrist's D just
        // because a dominant heavy axis has exhausted its damping allowance.
        for(std::size_t i=0;i<n;++i) {
            const auto& x=limits[i];const double contribution=limiting.v[i]*limiting.v[i],v=out.velocity_error_bound_rad[i];
            const double cap=v>0?std::max(0.0,(x.effort_budget-out.gains.kp[i]*out.reference_error_bound_rad[i])/v):x.kd_max;
            const double available=quantize_gain(std::min(x.kd_max,cap),x.grid.kd_step,x.kd_max);
            const double requested=out.gains.kd[i]+deficit/std::max(1e-30,contribution);
            const double proposed=std::min(available,quantize_gain(requested,x.grid.kd_step,x.kd_max,true));
            const double delta=proposed-out.gains.kd[i];
            const double normalizer=std::max(1e-12,2*demand.damping_ratio*std::sqrt(out.inertia_max[i]*out.gains.kp[i]));
            const double score=delta>0?contribution*delta/(delta/normalizer):0;
            if(delta>1e-12 && score>best+1e-12){selected=i;best=score;new_value=proposed;action="allocate_damping";}
        }
        const double target_lambda=std::pow(limiting.ratio*limiting.w/demand.damping_ratio,2);
        const double reduction=std::max(0.0,limiting.w*limiting.w-target_lambda);
        for(std::size_t i=0;i<n;++i) {
            const auto& x=limits[i];const double participation=limiting.v[i]*limiting.v[i];
            const double floor=quantize_gain(std::abs(gravity_gradient[i])+std::max(x.grid.kp_step,1e-9),x.grid.kp_step,x.kp_max,true);
            const double requested=out.gains.kp[i]-std::max(x.grid.kp_step,reduction/std::max(1e-30,participation));
            const double proposed=std::max(floor,quantize_gain(std::max(0.0,requested),x.grid.kp_step,x.kp_max));
            const double delta=out.gains.kp[i]-proposed;
            const double lowered_w=std::sqrt(std::max(0.0,limiting.w*limiting.w-participation*delta));
            const double improvement=2*demand.damping_ratio*(limiting.w-lowered_w);
            const double score=delta>0?improvement/(delta/out.gains.kp[i]):0;
            if(delta>1e-12 && score>best+1e-12){selected=i;best=score;new_value=proposed;action="reduce_participating_frequency";}
        }
        if(selected==n)break;
        const double participation=out.gains.kp[selected]*limiting.v[selected]*limiting.v[selected]/(limiting.w*limiting.w);
        out.modal_adjustments.push_back("sample="+std::to_string(limiting.sample)+";mode="+std::to_string(limiting.mode)+
            ";axis="+std::to_string(selected)+";stiffness_participation="+std::to_string(participation)+";action="+action);
        if(action=="allocate_damping")out.gains.kd[selected]=new_value;else out.gains.kp[selected]=new_value;
        const std::string reason=action=="allocate_damping"?"coupled_mode_damping_allocation":"coupled_participating_frequency_reduced";
        if(std::find(out.constraints[selected].begin(),out.constraints[selected].end(),reason)==out.constraints[selected].end())out.constraints[selected].push_back(reason);
        limiting=evaluate_modes();
    }
    if(limiting.ratio<demand.damping_ratio*(1-1e-6))
        throw std::invalid_argument("coupled initial damping target cannot fit residual gravity, protocol grid and feedback effort");
    for(std::size_t i=0;i<n;++i)out.frequency_hz[i]=std::sqrt(out.gains.kp[i]/out.inertia_max[i])/(2*pi);
    return out;
}

// Path acceleration/error describes a later performance demand, not evidence
// that a high-bandwidth controller is already stable. Establish the first
// motion from the explicit recovery target and physical inertia instead.
inline ModelInitialization bootstrap_model_initialization(const std::vector<std::vector<double>>& inertia_samples,
    const std::vector<double>& gravity_gradient,const std::vector<ModelAxisLimits>& limits,
    const ModelResponseDemand& task_demand=ModelResponseDemand{}) {
    auto demand=recovery_response_demand(limits.size(),task_demand.peak_error_rad,task_demand.settling_time_s,task_demand.designed_amplitude_rad);
    demand.damping_ratio=task_demand.damping_ratio;demand.mass_matrix_samples=task_demand.mass_matrix_samples;
    auto out=model_initialization(inertia_samples,gravity_gradient,limits,demand);
    out.source="urdf_inertia_and_unmeasured_recovery_bootstrap";
    return out;
}

struct ShortProbePlan {
    Requirements requirements;bool valid{false};std::string reason{"insufficient_budget"};double torque_amplitude_nm{0};
    GainWorkingDomain working_domain;double planned_input_peak_nm{0};
};
inline double applied_pd_input_peak(const std::vector<Sample>& rows,double kp,double kd) {
    double peak=0;for(const auto& row:rows) {
        const double executed_kp=row.actual_gains_known?row.actual_kp:kp,executed_kd=row.actual_gains_known?row.actual_kd:kd;
        const double input=executed_kp*(row.reference-row.actual)+executed_kd*(row.reference_velocity-row.velocity);
        if(!std::isfinite(input))throw std::invalid_argument("nonfinite actual MIT input");
        peak=std::max(peak,std::abs(input));
    }
    return peak;
}
inline ShortProbePlan short_probe_plan(Requirements req,const ModelAxisLimits& bounds,double kp,double kd,
    const NoiseEstimate& noise,bool fine,double remaining_s,double requested_amplitude_rad=.015,double requested_frequency_hz=0) {
    validate_model_limits(bounds);ShortProbePlan out;
    if(!std::isfinite(kp) || kp<=0 || !std::isfinite(kd) || kd<0 || kp>bounds.kp_max || kd>bounds.kd_max)
        throw std::invalid_argument("invalid short probe gains");
    if(!std::isfinite(requested_amplitude_rad) || requested_amplitude_rad<=0 || !std::isfinite(requested_frequency_hz) || requested_frequency_hz<0)
        throw std::invalid_argument("invalid requested probe amplitude or frequency");
    const double floor=std::max({12*noise.encoder_step_rad,8*noise.rms_rad,.25*req.peak_error_rad,1e-5});
    double frequency=requested_frequency_hz>0?requested_frequency_hz:(fine?.65:.45);
    const double minimum_duration=fine?8.0:5.5;
    if(!std::isfinite(remaining_s) || remaining_s<minimum_duration+req.tail_s+.4)return out;
    for(unsigned attempt=0;attempt<5;++attempt) {
        const double duration=std::max(minimum_duration,1.25/frequency+1.4);
        if(duration+req.tail_s+.4>remaining_s){out.reason="insufficient_budget_for_observable_low_frequency_cycles";return out;}
        const double high_w=2*pi*frequency*1.2/.35;
        const double amplitude=std::min({requested_amplitude_rad,.7*bounds.position_margin_rad,
            bounds.max_velocity/(2*(high_w+3)),bounds.max_acceleration/(2*(high_w*high_w+6*high_w+13)),
            // Half of the calibration effort allowance covers nominal PD;
            // the other half remains for response deviations. The full command
            // still passes the unchanged normal total-effort Safety checks.
            bounds.effort_budget/(2*(kp+kd*(high_w+3)))});
        if(amplitude>=floor) {
            req.amplitude_rad=amplitude;req.frequencies_hz={frequency,2*frequency,frequency*1.2/.35};
            req.probe_duration_s=duration;req.noise_rms_rad=noise.rms_rad;req.encoder_step_rad=noise.encoder_step_rad;
            out.requirements=req;out.torque_amplitude_nm=std::min(.1*bounds.effort_budget,kp*amplitude*.5);
            out.working_domain=gain_working_domain(bounds,2*amplitude,2*amplitude*(high_w+3));
            out.planned_input_peak_nm=amplitude*(kp+kd*(high_w+3));
            out.valid=true;out.reason="bounded_model_probe";return out;
        }
        frequency*=.7;
    }
    out.reason="encoder_noise_or_motion_effort_margin_prevents_observable_probe";return out;
}
// A distinct motion-establishment experiment: the same complete controller,
// symmetric single-tone drive, and two full cycles outside the .7 s edges.
// It is not a three-frequency response/physical-identification experiment.
inline ShortProbePlan symmetric_motion_probe_plan(Requirements req,const ModelAxisLimits& bounds,double kp,double kd,
    const NoiseEstimate& noise,double remaining_s,const Requirements& previous) {
    validate_model_limits(bounds);ShortProbePlan out;
    const double frequency=std::min(previous.frequencies_hz.front(),1/(6*req.settling_time_s));
    if(!std::isfinite(frequency) || frequency<=0 || kp<=0 || kd<0 || kp>bounds.kp_max || kd>bounds.kd_max)return out;
    const double duration=2/frequency+1.4,w=2*pi*frequency;
    if(duration+req.tail_s+.6>remaining_s){out.reason="insufficient_budget_for_two_symmetric_motion_cycles";return out;}
    const double requested=std::max(1.25*previous.amplitude_rad,12*noise.encoder_step_rad);
    const double amplitude=std::min({requested,.7*bounds.position_margin_rad,bounds.max_velocity/(2*(w+3)),
        bounds.max_acceleration/(2*(w*w+6*w+13)),bounds.effort_budget/(2*(kp+kd*(w+3)))});
    if(amplitude<std::max({12*noise.encoder_step_rad,8*noise.rms_rad,.25*req.peak_error_rad,1e-5})) {
        out.reason="symmetric_motion_probe_below_encoder_or_noise_floor";return out;
    }
    req.motion_probe=true;req.amplitude_rad=amplitude;req.probe_duration_s=duration;
    req.frequencies_hz={frequency,2*frequency,frequency*1.2/.35};req.noise_rms_rad=noise.rms_rad;req.encoder_step_rad=noise.encoder_step_rad;
    out.requirements=req;out.valid=true;out.reason="symmetric_motion_establishment_only_not_frequency_response";
    out.working_domain=gain_working_domain(bounds,2*amplitude,2*amplitude*(w+3));
    out.planned_input_peak_nm=amplitude*(kp+kd*(w+3));
    return out;
}

inline double short_trial_cost(const std::vector<Requirements>& plans,double tail_s) {
    double duration=0;for(const auto& plan:plans)duration=std::max(duration,plan.probe_duration_s);
    return duration+tail_s+.6;
}
// Exogenous MIT reference drive, excluding the measured -Kp*q-Kd*dq terms.
// Used to compare the actual excitation a proposed gain/plan can produce;
// feedback velocity noise is not an inferred breakaway torque.
inline double prescribed_pd_input_peak(const Requirements& plan,double kp,double kd) {
    double peak=0;
    for(double t=0;t<=plan.probe_duration_s;t+=.005){const auto r=probe(t,plan);peak=std::max(peak,std::abs(kp*r.first+kd*r.second));}
    return peak;
}
inline ResponseModel motion_probe_evidence(const std::vector<Sample>& rows,const Requirements& req,double kp,double kd) {
    ResponseModel out;out.model_definition="motion_establishment_only_no_frequency_response_fit";
    out.missing_evidence.push_back("three_frequency_reference_response_not_measured");
    if(!std::isfinite(kp) || kp<=0 || !std::isfinite(kd) || kd<0){out.reason="invalid_mit_model_prior";return out;}
    if(!req.motion_probe || req.frequencies_hz.front()<=0 || !std::isfinite(req.frequencies_hz.front()) ||
       (req.probe_duration_s-1.4)*req.frequencies_hz.front()<2-1e-9){out.reason="invalid_excitation_plan";return out;}
    double previous=-1,last_feedback=-1;std::uint64_t sequence=0;
    NoiseEstimate guard_noise;guard_noise.encoder_step_rad=req.encoder_step_rad;guard_noise.rms_rad=req.noise_rms_rad;
    CommandConditionedMotionGuard guard(req,guard_noise);
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || row.time_s<previous){out.reason="invalid_command_time";return out;}previous=row.time_s;
        if(row.limited){out.reason="limited_input";return out;}
        if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual) || !std::isfinite(row.velocity)) {
            out.reason="nonfinite_feedback_or_reference";return out;
        }
        if(row.actual_gains_known && (!std::isfinite(row.actual_kp) || !std::isfinite(row.actual_kd) || row.actual_kp<0 || row.actual_kd<0)) {
            out.reason="invalid_executed_feedback_gains";return out;
        }
        if(row.actual_gains_known && (std::abs(row.actual_kp-kp)>1e-6*std::max(1e-9,std::abs(kp)) ||
            std::abs(row.actual_kd-kd)>1e-6*std::max(1e-9,std::abs(kd)))) {
            out.reason="changing_feedback_gains";out.missing_evidence.push_back("stable_executed_gain_window");return out;
        }
        if(!row.new_measurement)continue;
        if(row.feedback_sequence && row.feedback_sequence==sequence){++out.duplicate_feedback;continue;}
        if(row.feedback_sequence)sequence=row.feedback_sequence;
        if(guard.observe(row).triggered)out.sustained_oscillation=true;
        const double feedback=std::isfinite(row.feedback_time_s)?row.feedback_time_s:row.time_s;
        if((last_feedback>=0 && (feedback<=last_feedback || feedback-last_feedback>.06)) ||
            (std::isfinite(row.command_time_s) && row.command_time_s>row.time_s+.001)) {
            out.reason="feedback_time_gap_or_repeat";return out;
        }
        last_feedback=feedback;
        if(row.time_s>=.7 && row.time_s<req.probe_duration_s-.7)++out.fresh_samples;
    }
    out.reason=out.sustained_oscillation?"uncommanded_position_oscillation":
        out.fresh_samples>=100?"motion_establishment_only_no_spectral_fit":"insufficient_fresh_feedback";
    return out;
}
struct MotionEstablishmentDecision {
    Gains gains;std::vector<Requirements> plans;std::vector<std::string> reasons;
    std::vector<std::size_t> blocked_axes;
    bool established{false},retry{false},feedback_fault{false};
    bool motion_established{false},spectral_confirmation_required{false};
};
struct EstablishmentProgress {
    double anchor_input_nm{0},anchor_span_rad{0},anchor_minimum_travel_rad{0};
    double observed_input_nm{0},observed_span_rad{0},observed_minimum_travel_rad{0};
    double planned_input_nm{0},proposed_input_nm{0},confirmation_kp{0},confirmation_frequency_hz{0};
    bool response_plateau_pending{false},input_budget_plateau{false},blocked{false};
    std::string reason;
};
struct MotionEstablishmentState {
    Gains context_gains;std::vector<Requirements> context_plans;
    std::vector<unsigned> boundary_retests;
    std::vector<unsigned> excitation_retests;
    std::vector<bool> established_axes;
    std::vector<EstablishmentProgress> progress;
    std::vector<Requirements> spectral_plans;
};
// The same decision is used by the worker and the stick/slip regression. This
// is response establishment, not identification success or a candidate contest.
// Healthy nonlinear/weak feedback can revise excitation; missing feedback,
// actual limiting and nonfinite measurements cannot justify stronger commands.
inline MotionEstablishmentDecision motion_establishment_step(const Gains& current,const std::vector<double>& inertia,
    const std::vector<ModelAxisLimits>& limits,const std::vector<NoiseEstimate>& noise,
    const std::vector<ResponseModel>& models,const std::vector<Metrics>& metrics,
    const std::vector<MotionObservation>& motion,const std::vector<Requirements>& previous,
    const Requirements& req,bool fine,double remaining_for_measurement_s,MotionEstablishmentState* state=nullptr) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || inertia.size()!=n || limits.size()!=n || noise.size()!=n || models.size()!=n ||
       metrics.size()!=n || motion.size()!=n || previous.size()!=n)throw std::invalid_argument("motion establishment joint mapping mismatch");
    MotionEstablishmentDecision out;out.gains=current;out.plans=previous;out.reasons.assign(n,"response_established");out.established=true;out.motion_established=true;
    std::vector<bool> confirmation_axes(n,false);bool slow_confirmation=false;
    if(state) {
        if(state->boundary_retests.size()!=n){state->boundary_retests.assign(n,0);state->established_axes.assign(n,false);}
        if(state->excitation_retests.size()!=n)state->excitation_retests.assign(n,0);
        if(state->progress.size()!=n)state->progress.resize(n);
        if(state->spectral_plans.size()!=n)state->spectral_plans=previous;
        for(std::size_t i=0;i<n;++i) {
            if(!previous[i].motion_probe)state->spectral_plans[i]=previous[i];
            if(metrics[i].continuous && metrics[i].measurement_samples>=100 && !metrics[i].limited_samples &&
               std::isfinite(metrics[i].applied_input_peak_nm) && std::isfinite(metrics[i].actual_span)) {
                auto& p=state->progress[i];p.observed_input_nm=metrics[i].applied_input_peak_nm;p.observed_span_rad=metrics[i].actual_span;
                p.observed_minimum_travel_rad=std::min(motion[i].positive_travel_rad,motion[i].negative_travel_rad);
            }
            const bool same=state->context_gains.kp.size()==n && state->context_gains.kd.size()==n && state->context_plans.size()==n &&
                state->context_gains.kp[i]==current.kp[i] && state->context_gains.kd[i]==current.kd[i] &&
                state->context_plans[i].amplitude_rad==previous[i].amplitude_rad && state->context_plans[i].frequencies_hz==previous[i].frequencies_hz;
            if(!same){state->boundary_retests[i]=0;state->established_axes[i]=false;}
            if((models[i].tracking_valid || previous[i].motion_probe) && models[i].reason!="changing_feedback_gains" && motion[i].qualified && !models[i].sustained_oscillation && !metrics[i].oscillating) {
                state->established_axes[i]=true;state->progress[i].blocked=false;state->progress[i].reason.clear();
            }
        }
        state->context_gains=current;state->context_plans=previous;
    }
    for(std::size_t i=0;i<n;++i) {
        const auto& m=metrics[i];
        if(!m.continuous || m.measurement_samples<100 || m.limited_samples || !std::isfinite(m.rms_error) ||
           !std::isfinite(m.peak_error) || !std::isfinite(m.actual_span)) {
            out.feedback_fault=true;out.established=false;out.reasons[i]="unhealthy_or_limited_measurement";
        }
        const auto& model=models[i];
        const bool revisable=model.tracking_valid || model.sustained_oscillation ||
            (previous[i].motion_probe && model.reason=="motion_establishment_only_no_spectral_fit") || model.reason=="response_below_encoder_noise_floor" ||
            model.reason=="low_spectral_fit_quality" || model.reason=="insufficient_excitation_at_required_frequencies" || model.reason=="changing_feedback_gains";
        if(!revisable){out.feedback_fault=true;out.reasons[i]="invalid_experiment_or_feedback:"+model.reason;}
        const bool moving=motion[i].qualified && model.reason!="changing_feedback_gains" && !model.sustained_oscillation && !m.oscillating;
        out.motion_established=out.motion_established && moving;
        out.established=out.established && model.tracking_valid && moving && !previous[i].motion_probe;
    }
    out.spectral_confirmation_required=out.motion_established && std::any_of(previous.begin(),previous.end(),[](const Requirements& p){return p.motion_probe;});
    if(out.feedback_fault){out.motion_established=false;return out;}
    if(out.established || out.motion_established)return out;
    for(std::size_t i=0;i<n;++i) {
        if((models[i].tracking_valid || previous[i].motion_probe) && models[i].reason!="changing_feedback_gains" && motion[i].qualified && !models[i].sustained_oscillation && !metrics[i].oscillating) {
            if(state){state->progress[i].blocked=false;state->progress[i].reason.clear();}continue;
        }
        auto* progress=state?&state->progress[i]:nullptr;
        if(progress && progress->blocked){out.blocked_axes.push_back(i);out.reasons[i]=progress->reason;continue;}
        if(models[i].reason=="changing_feedback_gains") {
            out.reasons[i]="gain_transition_requires_stable_complete_set_remeasurement";out.retry=true;continue;
        }
        if(previous[i].motion_probe && !models[i].sustained_oscillation && !metrics[i].oscillating) {
            out.reasons[i]="symmetric_motion_probe_insufficient_at_tested_input_not_hardware_capability";
            if(progress){progress->blocked=true;progress->reason=out.reasons[i];}out.blocked_axes.push_back(i);continue;
        }
        validate_model_limits(limits[i]);
        if(!std::isfinite(inertia[i]) || inertia[i]<=0)throw std::invalid_argument("motion establishment requires valid model inertia");
        auto observed_noise=noise[i];observed_noise.encoder_step_rad=std::max(observed_noise.encoder_step_rad,motion[i].encoder_step_rad);
        const double resolution=std::max({observed_noise.encoder_step_rad,2*observed_noise.rms_rad,1e-6});
        const double minimum_travel=std::min(motion[i].positive_travel_rad,motion[i].negative_travel_rad);
        bool measured_plateau=false,force_symmetric_motion=false;
        if(progress) {
            progress->observed_input_nm=metrics[i].applied_input_peak_nm;progress->observed_span_rad=metrics[i].actual_span;
            progress->observed_minimum_travel_rad=minimum_travel;
            const bool improved=metrics[i].actual_span>progress->anchor_span_rad+resolution ||
                minimum_travel>progress->anchor_minimum_travel_rad+resolution;
            const bool appreciable_motion=metrics[i].actual_span>=4*resolution;
            if(progress->response_plateau_pending) {
                if(improved){progress->response_plateau_pending=false;}
                else if(current.kp[i]==progress->confirmation_kp && previous[i].frequencies_hz.front()<=progress->confirmation_frequency_hz) {
                    force_symmetric_motion=true;progress->response_plateau_pending=false;
                }
            }
            if(appreciable_motion && (progress->anchor_input_nm<=0 || improved)) {
                progress->anchor_input_nm=metrics[i].applied_input_peak_nm;progress->anchor_span_rad=metrics[i].actual_span;
                progress->anchor_minimum_travel_rad=minimum_travel;
            } else if(progress->anchor_input_nm>0 && metrics[i].applied_input_peak_nm>=1.5*progress->anchor_input_nm)
                measured_plateau=true;
        }
        measured_plateau=measured_plateau || force_symmetric_motion;
        if(state && motion[i].quantization_boundary && state->boundary_retests[i]<2) {
            ++state->boundary_retests[i];out.retry=true;
            out.reasons[i]="encoder_boundary_requires_same_complete_gain_set_remeasurement";
            continue;
        }
        double kp=current.kp[i],kd=current.kd[i];
        double requested_amplitude=previous[i].amplitude_rad,frequency=previous[i].frequencies_hz.front();
        const bool ringing=models[i].sustained_oscillation || metrics[i].oscillating;
        if(ringing) {
            // Increasing D cannot be justified by ringing when delay and pole
            // estimates are unknown: it may increase the unstable loop gain.
            kp*=.8;kd*=.8;requested_amplitude*=.8;frequency=std::max(.20,.85*frequency);
            out.reasons[i]="uncommanded_oscillation_reduce_feedback_and_excitation";
        } else {
            const double floor=std::max({4*observed_noise.encoder_step_rad,6*observed_noise.rms_rad,1e-6});
            const bool insufficient_motion=metrics[i].actual_span<=floor || motion[i].response_rms_ratio<.2 ||
                (!motion[i].bidirectional && metrics[i].actual_span<2*motion[i].minimum_motion_span_rad);
            const unsigned retests=state?state->excitation_retests[i]:0;
            if(!insufficient_motion || (state && retests==0)) {
                // A poor spectral fit with appreciable motion may be friction,
                // unmodelled dynamics or noise. It is not evidence for more K.
                if(state)++state->excitation_retests[i];
                requested_amplitude=std::min(.7*limits[i].position_margin_rad,std::max(1.25*requested_amplitude,12*observed_noise.encoder_step_rad));
                frequency=std::max(.20,.85*frequency);
                out.reasons[i]="weak_response_requires_excitation_evidence_before_gain_increase:"+models[i].reason;
            } else {
                // A bounded relative Kp step addresses demonstrated sticking.
                // Actual PD peak includes D*velocity noise and must never be
                // divided by reference amplitude to infer a breakaway Kp.
                kp=std::min(limits[i].kp_max,std::max(kp+limits[i].grid.kp_step,1.4*kp));
                out.reasons[i]="measured_insufficient_motion_bounded_stiffness_step_no_damping_inference";
            }
        }
        kp=quantize_gain(kp,limits[i].grid.kp_step,limits[i].kp_max,kp>0);
        kd=quantize_gain(kd,limits[i].grid.kd_step,limits[i].kd_max);
        auto plan=short_probe_plan(req,limits[i],kp,kd,observed_noise,fine,remaining_for_measurement_s,requested_amplitude,frequency);
        if(!plan.valid){out.reasons[i]+=";"+plan.reason;continue;}
        const double previous_input=prescribed_pd_input_peak(previous[i],current.kp[i],current.kd[i]);
        const double proposed_input=prescribed_pd_input_peak(plan.requirements,kp,kd);
        const double distinguishable_increment=std::max(.1*previous_input,2*resolution*current.kp[i]);
        const bool input_plateau=kp>current.kp[i] && proposed_input<previous_input+distinguishable_increment;
        if(progress){progress->planned_input_nm=previous_input;progress->proposed_input_nm=proposed_input;progress->input_budget_plateau=input_plateau;}
        if(input_plateau || measured_plateau) {
            // More K with inverse amplitude scaling cannot break stiction.
            // Keep this whole gain set and spend remaining measurement time on
            // slower, larger legal excitation before declaring its limitation.
            kp=current.kp[i];kd=current.kd[i];
            frequency=std::max(.20,.7*previous[i].frequencies_hz.front());
            requested_amplitude=std::min(.7*limits[i].position_margin_rad,1.25*previous[i].amplitude_rad);
            plan=short_probe_plan(req,limits[i],kp,kd,observed_noise,fine,remaining_for_measurement_s,requested_amplitude,frequency);
            const bool changed=plan.valid && (plan.requirements.amplitude_rad>previous[i].amplitude_rad+1e-10 ||
                plan.requirements.frequencies_hz.front()<previous[i].frequencies_hz.front()-1e-10);
            out.reasons[i]=measured_plateau?"response_effort_plateau_requires_slower_complete_gain_set_confirmation":"gain_increase_rejected_without_distinguishable_excitation_input_growth";
            if(!changed || force_symmetric_motion) {
                const auto symmetric=symmetric_motion_probe_plan(req,limits[i],kp,kd,observed_noise,remaining_for_measurement_s,previous[i]);
                if(!symmetric.valid) {
                    out.reasons[i]="symmetric_motion_establishment_unavailable:"+symmetric.reason;
                    if(progress){progress->blocked=true;progress->reason=out.reasons[i];}out.blocked_axes.push_back(i);continue;
                }
                plan=symmetric;out.reasons[i]=force_symmetric_motion?"confirmed_response_plateau_requires_symmetric_motion_probe_at_same_complete_gains":
                    "spectral_excitation_boundary_requires_symmetric_motion_probe_at_same_complete_gains";
            }
            slow_confirmation=true;confirmation_axes[i]=true;
            if(measured_plateau && progress && !plan.requirements.motion_probe){progress->response_plateau_pending=true;progress->confirmation_kp=kp;progress->confirmation_frequency_hz=frequency;}
        }
        out.gains.kp[i]=kp;out.gains.kd[i]=kd;
        out.plans[i]=plan.requirements;
        out.retry=out.retry || kp!=current.kp[i] || kd!=current.kd[i] ||
            plan.requirements.amplitude_rad!=previous[i].amplitude_rad || plan.requirements.frequencies_hz!=previous[i].frequencies_hz ||
            plan.requirements.motion_probe!=previous[i].motion_probe;
    }
    const bool any_ringing=std::any_of(models.begin(),models.end(),[](const ResponseModel& m){return m.sustained_oscillation;}) ||
        std::any_of(metrics.begin(),metrics.end(),[](const Metrics& m){return m.oscillating;});
    if(any_ringing) {
        // A measured unstable axis takes priority over a same-group comparison.
        // No other axis is strengthened while feedback is being reduced.
        for(std::size_t i=0;i<n;++i) {
            if(state) {
                auto& p=state->progress[i];p.response_plateau_pending=false;
                p.anchor_input_nm=0;p.anchor_span_rad=0;p.anchor_minimum_travel_rad=0;
            }
            if(models[i].sustained_oscillation || metrics[i].oscillating)continue;
            out.gains.kp[i]=current.kp[i];out.gains.kd[i]=current.kd[i];out.plans[i]=previous[i];
            if(confirmation_axes[i])out.reasons[i]="slow_confirmation_deferred_for_oscillation_feedback_reduction";
        }
    } else if(slow_confirmation) {
        // The comparison is of a complete measured controller, not an axis
        // spliced together while unrelated gains change in the same trial.
        out.gains=current;
        for(std::size_t i=0;i<n;++i)if(!confirmation_axes[i]) {
            out.plans[i]=previous[i];
            if(out.reasons[i]!="response_established" && std::find(out.blocked_axes.begin(),out.blocked_axes.end(),i)==out.blocked_axes.end())
                out.reasons[i]="gain_change_deferred_during_same_complete_gain_set_confirmation";
        }
    }
    double duration=0;for(const auto& plan:out.plans)duration=std::max(duration,plan.probe_duration_s);
    for(auto& plan:out.plans)plan.probe_duration_s=duration;
    if(short_trial_cost(out.plans,req.tail_s)>remaining_for_measurement_s)out.retry=false;
    return out;
}
// A changed excitation is compared only after both complete gain sets have
// been measured with this common plan. The original discovery amplitude is
// never an implicit lower bound on a later legal candidate experiment.
inline std::vector<Requirements> comparable_probe_plans(const Gains& first,const Gains& second,
    const std::vector<Requirements>& previous,const std::vector<ModelAxisLimits>& limits,
    const std::vector<NoiseEstimate>& noise,const Requirements& req,bool fine,double remaining_s) {
    const auto n=first.kp.size();
    if(!n || first.kd.size()!=n || second.kp.size()!=n || second.kd.size()!=n || previous.size()!=n || limits.size()!=n || noise.size()!=n)
        throw std::invalid_argument("comparable probe joint mapping mismatch");
    std::vector<Requirements> plans;double duration=0;
    for(std::size_t i=0;i<n;++i) {
        const auto plan=short_probe_plan(req,limits[i],std::max(first.kp[i],second.kp[i]),std::max(first.kd[i],second.kd[i]),
            noise[i],fine,remaining_s,previous[i].amplitude_rad,previous[i].frequencies_hz.front());
        if(!plan.valid)throw std::invalid_argument("no common bounded candidate excitation: "+plan.reason);
        plans.push_back(plan.requirements);duration=std::max(duration,plan.requirements.probe_duration_s);
    }
    for(auto& plan:plans)plan.probe_duration_s=duration;
    return plans;
}

enum class ExcitationKind {ReferenceTracking,IndependentTorqueDisturbance};
struct MitExperiment {
    double kp{0},kd{0},inertia_prior{0},gravity_gradient{0};
    ExcitationKind kind{ExcitationKind::ReferenceTracking};
    double executed_state_feedforward_gradient{0};
};

struct DisturbanceInputRevision {
    std::vector<double> amplitudes;std::vector<std::string> reasons;bool retry{false},feedback_fault{false};
};
inline DisturbanceInputRevision revise_disturbance_input(const std::vector<double>& current,
    const std::vector<double>& ceiling,const std::vector<ResponseModel>& models,const std::vector<Metrics>& metrics,
    double remaining_s,double complete_trial_cost_s,double reserve_s=0) {
    const auto n=current.size();
    if(!n || ceiling.size()!=n || models.size()!=n || metrics.size()!=n)throw std::invalid_argument("independent disturbance mapping mismatch");
    DisturbanceInputRevision out;out.amplitudes=current;out.reasons.assign(n,"independent_response_measured");
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(current[i]) || current[i]<=0 || !std::isfinite(ceiling[i]) || ceiling[i]<=0 || current[i]>ceiling[i]*(1+1e-9))
            throw std::invalid_argument("independent disturbance exceeds calibrated effort/Safety allowance");
        const auto& model=models[i];const auto& m=metrics[i];
        const bool revisable=model.reason=="response_below_encoder_noise_floor" || model.reason=="low_spectral_fit_quality" ||
            model.reason=="insufficient_excitation_at_required_frequencies" || model.reason=="changing_feedback_gains";
        if(!m.continuous || m.measurement_samples<100 || m.limited_samples || (!model.disturbance_valid && !revisable)) {
            out.feedback_fault=true;out.reasons[i]="invalid_or_limited_disturbance_measurement:"+model.reason;continue;
        }
        if(model.disturbance_valid)continue;
        if(model.reason=="changing_feedback_gains"){out.retry=true;out.reasons[i]="disturbance_requires_stable_feedback_gains";continue;}
        out.amplitudes[i]=std::min(ceiling[i],2*current[i]);
        out.reasons[i]=out.amplitudes[i]>current[i]?"independent_response_insufficient_increase_known_input":"independent_response_unobservable_at_disturbance_safety_ceiling";
        out.retry=out.retry || out.amplitudes[i]>current[i];
    }
    if(out.feedback_fault || !std::isfinite(remaining_s) || complete_trial_cost_s+reserve_s>remaining_s)out.retry=false;
    return out;
}

struct DisturbanceAssessment {
    bool measurement_complete{false},passed{false},limited{false},sustained_oscillation{false},motion_observed{false},static_measurement_complete{false};
    double input_peak_nm{0},initial_error_rad{0},peak_displacement_rad{0},release_recovery_s{0};
    double final_error_rad{0},final_drift_rad{0},final_peak_velocity{0},peak_limit_rad{0},final_window_s{0};
    double encoder_step_rad{0},motion_floor_rad{0};
    std::size_t fresh_samples{0};std::string reason{"insufficient_measurements"};
};
// This assesses a separately commanded torque at a fixed MIT reference. The
// forced displacement is a disturbance metric, never a tracking overshoot.
// Final static error, final drift and the release transient remain separate.
inline DisturbanceAssessment assess_disturbance(const std::vector<Sample>& rows,const Requirements& req,
    const NoiseEstimate& noise,double excitation_end_s,double max_displacement_rad) {
    DisturbanceAssessment out;out.peak_limit_rad=max_displacement_rad;
    if(rows.empty() || !std::isfinite(excitation_end_s) || excitation_end_s<=0 ||
       !std::isfinite(max_displacement_rad) || max_displacement_rad<=0)return out;
    if(rows.front().time_s>.06 || rows.back().time_s<excitation_end_s+req.tail_s-.06)return out;
    std::vector<double> fresh_positions;
    for(const auto& row:rows)if(row.new_measurement && std::isfinite(row.actual))fresh_positions.push_back(row.actual);
    out.encoder_step_rad=std::max({noise.encoder_step_rad,req.encoder_step_rad,observed_encoder_step(fresh_positions)});
    const double floor=std::max({3*out.encoder_step_rad,6*std::max(noise.rms_rad,req.noise_rms_rad),1e-6});
    out.motion_floor_rad=floor;
    double origin=0,initial_reference=0,count=0;
    for(const auto& row:rows)if(row.new_measurement && row.time_s<=.15) {
        origin+=row.actual;initial_reference+=row.reference;++count;
    }
    if(count<3)return out;
    origin/=count;initial_reference/=count;out.initial_error_rad=origin-initial_reference;
    const double final_start=std::max(excitation_end_s+.05,rows.back().time_s-.5);
    out.final_window_s=rows.back().time_s-final_start;
    double final_low=std::numeric_limits<double>::infinity(),final_high=-final_low,last_bad=excitation_end_s;
    double previous=-1,last_fresh=-1,extreme=origin;std::size_t tail_reversals=0,final_samples=0;int direction=0;
    bool known=true,continuous=true,reference_fixed=true;
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || !std::isfinite(row.actual) || !std::isfinite(row.reference) || !std::isfinite(row.velocity))return out;
        if(previous>=0 && (row.time_s<=previous || row.time_s-previous>.06))continuous=false;
        previous=row.time_s;out.limited=out.limited || row.limited;
        reference_fixed=reference_fixed && std::abs(row.reference-initial_reference)<=std::max(1e-7,noise.encoder_step_rad);
        out.peak_displacement_rad=std::max(out.peak_displacement_rad,std::abs(row.actual-origin));
        if(!row.new_measurement)continue;
        if(last_fresh>=0 && row.time_s-last_fresh>.06)continuous=false;
        last_fresh=row.time_s;++out.fresh_samples;
        known=known && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm);
        out.input_peak_nm=std::max(out.input_peak_nm,std::abs(row.independent_disturbance_nm));
        if(row.time_s>=excitation_end_s) {
            if(std::abs(row.actual-row.reference)>req.hold_error_rad || std::abs(row.velocity)>std::max(req.hold_velocity_rad_s,noise.velocity_peak+1e-6))last_bad=row.time_s;
            if(!direction) {
                if(row.actual>extreme+floor){direction=1;extreme=row.actual;}
                else if(row.actual<extreme-floor){direction=-1;extreme=row.actual;}
            } else if(direction>0) {
                if(row.actual>extreme)extreme=row.actual;
                else if(row.actual<extreme-floor){direction=-1;extreme=row.actual;++tail_reversals;}
            } else {
                if(row.actual<extreme)extreme=row.actual;
                else if(row.actual>extreme+floor){direction=1;extreme=row.actual;++tail_reversals;}
            }
        }
        if(row.time_s>=final_start) {
            ++final_samples;final_low=std::min(final_low,row.actual);final_high=std::max(final_high,row.actual);
            out.final_error_rad=std::max(out.final_error_rad,std::abs(row.actual-row.reference));
            out.final_peak_velocity=std::max(out.final_peak_velocity,std::abs(row.velocity));
        }
    }
    if(last_fresh<0 || rows.back().time_s-last_fresh>.06)continuous=false;
    out.sustained_oscillation=tail_reversals>=4 && final_high-final_low>floor;
    out.final_drift_rad=std::isfinite(final_low)?final_high-final_low:std::numeric_limits<double>::infinity();
    out.release_recovery_s=std::max(0.0,last_bad-excitation_end_s);
    out.motion_observed=out.peak_displacement_rad>floor;
    out.static_measurement_complete=continuous && known && reference_fixed && out.fresh_samples>=100 && out.input_peak_nm>1e-8 && out.final_window_s>=.25 && final_samples>=20;
    out.measurement_complete=out.static_measurement_complete && out.motion_observed;
    if(!continuous || out.fresh_samples<100)out.reason="insufficient_fresh_feedback";
    else if(!known || out.input_peak_nm<=1e-8)out.reason="independent_disturbance_input_missing";
    else if(!reference_fixed)out.reason="disturbance_reference_not_fixed";
    else if(out.limited)out.reason="limited_input";
    else if(!out.motion_observed)out.reason="independent_response_below_motion_floor_static_behavior_only";
    else if(out.peak_displacement_rad>max_displacement_rad)out.reason="disturbance_displacement_above_goal";
    else if(out.sustained_oscillation)out.reason="sustained_oscillation_after_release";
    else if(out.final_error_rad>req.hold_error_rad)out.reason="final_static_error_above_goal";
    else if(out.final_drift_rad>req.hold_error_rad)out.reason="final_position_drift_above_goal";
    else if(out.release_recovery_s>req.settling_time_s || rows.back().time_s-last_bad<.25)out.reason="disturbance_release_recovery_above_goal";
    else if(out.final_peak_velocity>std::max(req.hold_velocity_rad_s,noise.velocity_peak+1e-6))out.reason="final_motion_above_goal";
    else if(!out.measurement_complete)out.reason="insufficient_final_window";
    else {out.passed=true;out.reason="disturbance_response_qualified";}
    return out;
}

// Spectral instrumental-response estimation uses a prescribed external
// reference or an independently injected torque. It never regresses raw
// feedback torque against noisy feedback as if this were an open-loop test.
// For MIT, numerator = Kp*R + Kd*Vref + Ffeedforward + Dindependent.
// Raw known state-dependent FF stays in the forcing. Thus the PHYSICAL fitted
// denominator includes gravity; interpretation of restoring stiffness removes
// only the known executed state-FF gradient, not the raw measured FF signal.
inline ResponseModel identify_mit(const std::vector<Sample>& rows,const Requirements& req,const MitExperiment& experiment) {
    ResponseModel out;out.model_definition="mit_closed_loop_known_reference_velocity_and_feedforward";
    if(!std::isfinite(req.probe_duration_s) || req.probe_duration_s<=1.4 || !std::isfinite(req.amplitude_rad) || req.amplitude_rad<=0) {
        out.reason="invalid_excitation_plan";return out;
    }
    for(std::size_t j=0;j<3;++j)if(!std::isfinite(req.frequencies_hz[j]) || req.frequencies_hz[j]<=0 ||
        (j && req.frequencies_hz[j]<=req.frequencies_hz[j-1])) {out.reason="invalid_excitation_frequencies";return out;}
    if((req.probe_duration_s-1.4)*req.frequencies_hz.front()<1) {
        out.reason="insufficient_low_frequency_cycles";out.missing_evidence.push_back("longer_observation_for_slow_excitation");return out;
    }
    if(!std::isfinite(experiment.kp) || experiment.kp<0 || !std::isfinite(experiment.kd) || experiment.kd<0 ||
       !std::isfinite(experiment.inertia_prior) || experiment.inertia_prior<=0 || !std::isfinite(experiment.gravity_gradient) ||
       !std::isfinite(experiment.executed_state_feedforward_gradient)) {
        out.reason="invalid_mit_model_prior";return out;
    }
    out.nominal_physical_stiffness_nm_rad=experiment.kp+experiment.gravity_gradient;
    std::vector<const Sample*> selected;bool known=true,disturbance_known=true;std::uint64_t sequence=0;
    NoiseEstimate guard_noise;guard_noise.encoder_step_rad=req.encoder_step_rad;guard_noise.rms_rad=req.noise_rms_rad;
    CommandConditionedMotionGuard guard(req,guard_noise);
    double previous=-1,last_feedback=-1;bool continuous=true;
    for(const auto& row:rows) {
        if(!std::isfinite(row.time_s) || row.time_s<previous){out.reason="invalid_command_time";return out;}previous=row.time_s;
        if(row.actual_gains_known) {
            if(!std::isfinite(row.actual_kp) || row.actual_kp<0 || !std::isfinite(row.actual_kd) || row.actual_kd<0){out.reason="invalid_executed_feedback_gains";return out;}
            if(std::abs(row.actual_kp-experiment.kp)>1e-6*std::max(1e-9,std::abs(experiment.kp)) ||
               std::abs(row.actual_kd-experiment.kd)>1e-6*std::max(1e-9,std::abs(experiment.kd))) {
                out.reason="changing_feedback_gains";out.missing_evidence.push_back("stable_executed_gain_window");return out;
            }
        }
        out.tracking_peak_error_rad=std::max(out.tracking_peak_error_rad,std::abs(row.actual-row.reference));
        if(row.limited){out.reason="limited_input";out.missing_evidence.push_back("unlimited_excitation");return out;}
        if(!row.new_measurement)continue;
        if(row.feedback_sequence && row.feedback_sequence==sequence){++out.duplicate_feedback;continue;}
        if(row.feedback_sequence)sequence=row.feedback_sequence;
        if(guard.observe(row).triggered)out.sustained_oscillation=true;
        const double feedback=std::isfinite(row.feedback_time_s)?row.feedback_time_s:row.time_s;
        if(last_feedback>=0 && (feedback<=last_feedback || feedback-last_feedback>.06))continuous=false;
        last_feedback=feedback;
        if(std::isfinite(row.command_time_s) && row.command_time_s>row.time_s+.001)continuous=false;
        if(row.time_s<.7 || row.time_s>=req.probe_duration_s-.7)continue;
        if(!std::isfinite(row.reference) || !std::isfinite(row.reference_velocity) || !std::isfinite(row.actual) || !std::isfinite(row.velocity)) {
            out.reason="nonfinite_feedback_or_reference";return out;
        }
        known=known && row.feedforward_known && std::isfinite(row.applied_feedforward_nm);
        disturbance_known=disturbance_known && row.independent_disturbance_known && std::isfinite(row.independent_disturbance_nm);
        selected.push_back(&row);
    }
    if(out.sustained_oscillation){out.reason="uncommanded_position_oscillation";out.missing_evidence.push_back("stable_command_conditioned_response");return out;}
    double positive_error=0,negative_error=0,velocity_product=0,velocity_square=0;
    std::size_t positive_count=0,negative_count=0;
    for(const auto* row:selected) {
        const double error=row->actual-row->reference;
        if(row->reference_velocity>.001){positive_error+=error;++positive_count;}
        if(row->reference_velocity<-.001){negative_error+=error;++negative_count;}
        velocity_product+=row->reference_velocity*error;velocity_square+=row->reference_velocity*row->reference_velocity;
    }
    if(positive_count>=20 && negative_count>=20) {
        out.direction_evidence_valid=true;out.direction_bias_rad=.5*(positive_error/positive_count-negative_error/negative_count);
    } else out.missing_evidence.push_back("bidirectional_velocity_error_evidence");
    if(velocity_square>1e-10)out.velocity_error_rad_per_rad_s=velocity_product/velocity_square;
    out.fresh_samples=selected.size();
    if(selected.size()<100 || !continuous){out.reason=continuous?"insufficient_fresh_feedback":"feedback_time_gap_or_repeat";return out;}
    Eigen::MatrixXd design(selected.size(),7);Eigen::MatrixXd values(selected.size(),6);
    for(std::size_t i=0;i<selected.size();++i) {
        const auto& row=*selected[i];design(i,0)=1;
        for(std::size_t j=0;j<3;++j){const double a=2*pi*req.frequencies_hz[j]*row.time_s;design(i,1+2*j)=std::sin(a);design(i,2+2*j)=std::cos(a);}
        values(i,0)=row.reference;values(i,1)=row.reference_velocity;values(i,2)=row.actual;
        values(i,3)=known?row.applied_feedforward_nm:0;values(i,4)=disturbance_known?row.independent_disturbance_nm:0;values(i,5)=row.velocity;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    if(qr.rank()!=7){out.reason="excitation_frequencies_not_separable";return out;}
    const Eigen::MatrixXd coefficients=qr.solve(values);
    const double signal=(values.col(2).array()-values.col(2).mean()).matrix().squaredNorm();
    if(signal<=selected.size()*std::pow(std::max({req.noise_rms_rad*3,req.encoder_step_rad*2,1e-6}),2)) {
        out.reason="response_below_encoder_noise_floor";return out;
    }
    out.coherence=std::clamp(1-(design*coefficients-values).col(2).squaredNorm()/signal,0.0,1.0);
    if(out.coherence<.65){out.reason="low_spectral_fit_quality";return out;}
    const auto coefficient=[&](std::size_t tone,int col){return std::complex<double>{coefficients(1+2*tone,col),coefficients(2+2*tone,col)};};
    std::array<std::complex<double>,3> inverse_compliance{};double lag_weight=0,lag=0;
    const bool disturbance=experiment.kind==ExcitationKind::IndependentTorqueDisturbance;
    for(std::size_t j=0;j<3;++j) {
        const auto r=coefficient(j,0),v=coefficient(j,1),y=coefficient(j,2),ff=coefficient(j,3),d=coefficient(j,4);
        const auto input=disturbance?d:r;const double w=2*pi*req.frequencies_hz[j];
        if(std::abs(input)<(disturbance?1e-6:req.amplitude_rad*.025) || std::abs(y)<std::max(2*req.noise_rms_rad,1e-7)) {
            out.reason="insufficient_excitation_at_required_frequencies";return out;
        }
        const auto h=y/input;out.points.push_back({req.frequencies_hz[j],std::abs(h),std::arg(h)});
        if(!disturbance) {
            lag+=std::max(0.0,-std::arg(h)/w)*std::norm(r);lag_weight+=std::norm(r);
            if(std::abs(h)>=.8 && std::abs(h)<=1.2 && std::abs(std::arg(h))<.45)
                out.tested_follow_frequency_hz=std::max(out.tested_follow_frequency_hz,req.frequencies_hz[j]);
        } else if(std::abs(experiment.kp*r+experiment.kd*v)>.1*std::abs(d)) {
            out.reason="disturbance_test_reference_not_stationary";out.missing_evidence.push_back("independent_disturbance_with_fixed_reference");return out;
        }
        const auto forcing=experiment.kp*r+experiment.kd*v+ff+d;
        if(std::abs(forcing)<1e-8){out.reason="insufficient_known_command_effort";return out;}
        inverse_compliance[j]=forcing/y;
    }
    out.reference_lag_s=lag_weight>0?lag/lag_weight:0;out.feedforward_known=known;
    if(disturbance) {
        out.missing_evidence.erase(std::remove(out.missing_evidence.begin(),out.missing_evidence.end(),"bidirectional_velocity_error_evidence"),out.missing_evidence.end());
        if(!disturbance_known){out.reason="independent_disturbance_not_measured";out.missing_evidence.push_back("actual_independent_torque_input");return out;}
        out.disturbance_valid=known;out.low_frequency_compliance_rad_nm=out.points.front().gain;
    } else out.tracking_valid=true;
    if(!known) {
        out.reason="executed_feedforward_unknown";out.missing_evidence.push_back("applied_feedforward_for_closed_loop_fit");return out;
    }
    struct PhysicalFit {double cost,delay,mass,stiffness,damping,held;};std::vector<PhysicalFit> fits;
    double best=std::numeric_limits<double>::infinity(),best_delay=0,best_j=0,best_k=0,best_d=0,best_held=1e30;
    for(unsigned index=0;index<=60;++index) {
        const double delay=.002*index;
        Eigen::Matrix<double,4,3> a;Eigen::Matrix<double,4,1> b;
        for(std::size_t j=0;j<2;++j) {
            const double w=2*pi*req.frequencies_hz[j];
            const auto denominator=inverse_compliance[j]*std::exp(std::complex<double>{0,-w*delay});
            a.row(2*j)<<-w*w,1,0;a.row(2*j+1)<<0,0,w;b[2*j]=denominator.real();b[2*j+1]=denominator.imag();
        }
        const Eigen::Vector3d fitted=a.colPivHouseholderQr().solve(b);
        const double mass=fitted[0],stiffness=fitted[1],damping=fitted[2];
        if(mass<.2*experiment.inertia_prior || mass>5*experiment.inertia_prior || stiffness<=0 || damping<=0)continue;
        double loss=0,held=0;
        for(std::size_t j=0;j<3;++j) {
            const double w=2*pi*req.frequencies_hz[j];
            const auto predicted=std::complex<double>{stiffness-mass*w*w,damping*w}*std::exp(std::complex<double>{0,w*delay});
            const double error=std::norm(predicted-inverse_compliance[j])/std::max(1e-8,std::norm(inverse_compliance[j]));
            if(j<2)loss+=error/2;else held=error;
        }
        // The model is a prior, not a replacement for independent held-out data.
        const double cost=loss+held+.002*std::pow(std::log(mass/experiment.inertia_prior),2);
        fits.push_back({cost,delay,mass,stiffness,damping,held});
        if(cost<best){best=cost;best_delay=delay;best_j=mass;best_k=stiffness;best_d=damping;best_held=held;out.relative_fit_error=std::sqrt(loss);}
    }
    if(best_j<=0){out.reason="closed_loop_model_not_identifiable";return out;}
    out.held_out_error=std::sqrt(best_held);out.effective_delay_s=best_delay;out.inertia_estimate=best_j;
    out.physical_stiffness_nm_rad=best_k;
    out.effective_stiffness_nm_rad=best_k-experiment.executed_state_feedforward_gradient;
    out.viscous_damping_nm_s_rad=best_d-experiment.kd;
    if(out.effective_stiffness_nm_rad<=0) {
        out.reason="nonpositive_effective_restoring_stiffness";out.missing_evidence.push_back("positive_effective_closed_loop_stiffness");return out;
    }
    out.natural_frequency_rad_s=std::sqrt(out.effective_stiffness_nm_rad/best_j);
    out.damping_ratio=best_d/(2*std::sqrt(out.effective_stiffness_nm_rad*best_j));
    double mass_low=best_j,mass_high=best_j,delay_low=best_delay,delay_high=best_delay,zeta_low=out.damping_ratio,zeta_high=out.damping_ratio;
    bool positive_stiffness_family=true;
    for(const auto& fit:fits)if(fit.cost<=best+.0004 && fit.held<=std::max(.0025,best_held*1.5+.0004)) {
        mass_low=std::min(mass_low,fit.mass);mass_high=std::max(mass_high,fit.mass);
        delay_low=std::min(delay_low,fit.delay);delay_high=std::max(delay_high,fit.delay);
        const double effective=fit.stiffness-experiment.executed_state_feedforward_gradient;
        if(effective<=0){positive_stiffness_family=false;continue;}
        const double zeta=fit.damping/(2*std::sqrt(fit.mass*effective));zeta_low=std::min(zeta_low,zeta);zeta_high=std::max(zeta_high,zeta);
    }
    const bool unique=positive_stiffness_family && mass_high/mass_low<=1.6 && delay_high-delay_low<=.03 && zeta_high-zeta_low<=.5;
    const bool consistent_damping=out.viscous_damping_nm_s_rad>=-std::max(.02,.1*experiment.kd);
    out.identifiable=out.relative_fit_error<.20 && out.held_out_error<.20 && best_delay<.12 && unique && consistent_damping;
    out.reason=out.identifiable?"mit_closed_loop_response_identified":!unique?"closed_loop_poles_or_delay_ambiguous":!consistent_damping?"closed_loop_damping_inconsistent_with_execution":"closed_loop_fit_insufficient";
    if(!out.identifiable)out.missing_evidence.push_back("additional_frequency_or_delay_evidence");
    // A gain peak is a tested closed-loop response peak. A mechanical mode
    // needs sustained ringing/independent evidence and is not inferred here.
    out.resonance_observed=false;
    return out;
}

struct CandidateProposal {Gains gains;std::vector<std::string> reasons;bool changed{false};};
inline CandidateProposal adaptive_candidate(const Gains& current,const std::vector<ResponseModel>& models,
    const std::vector<ModelAxisLimits>& limits,const Requirements& req=Requirements{}) {
    const auto n=current.kp.size();
    if(!n || current.kd.size()!=n || models.size()!=n || limits.size()!=n)throw std::invalid_argument("adaptive gain candidate mapping mismatch");
    CandidateProposal out;out.gains=current;out.reasons.assign(n,"insufficient_response_evidence");
    for(std::size_t i=0;i<n;++i) {
        validate_model_limits(limits[i]);const auto& m=models[i];double kp=current.kp[i],kd=current.kd[i];
        if(!std::isfinite(kp) || kp<0 || kp>limits[i].kp_max || !std::isfinite(kd) || kd<0 || kd>limits[i].kd_max)
            throw std::invalid_argument("adaptive candidate outside gain protocol limits");
        if(m.sustained_oscillation) {kp*=.8;kd*=.8;out.reasons[i]="uncommanded_oscillation_reduce_feedback_without_unproven_damping_increase";}
        else if(m.dynamic_overshoot) {kp*=.9;kd*=1.1;out.reasons[i]="dynamic_overshoot_reduce_stiffness_raise_damping";}
        else if(m.tracking_valid || m.disturbance_valid) {
            const bool deficit=m.tracking_peak_error_rad>req.peak_error_rad || m.reference_lag_s>.015;
            const bool soft=m.disturbance_valid && ((m.disturbance_goal_evaluated && !m.disturbance_goal_met) ||
                (std::isfinite(m.low_frequency_compliance_rad_nm) && current.kp[i]>0 && m.low_frequency_compliance_rad_nm*current.kp[i]>1.15));
            if(deficit && !m.identifiable && !soft)out.reasons[i]="tracking_deficit_without_identifiable_feedback_dynamics_requires_measurement";
            else if(deficit || soft) {
                kp*=m.identifiable && m.effective_delay_s*m.natural_frequency_rad_s>.4?1.05:1.18;
                if(m.identifiable && m.inertia_estimate>0) {
                    const double physical=2*.9*std::sqrt(m.inertia_estimate*kp)-m.viscous_damping_nm_s_rad;
                    kd=std::max(kd,physical);
                }
                out.reasons[i]=soft?"measured_disturbance_compliance_increase_stiffness":"measured_tracking_deficit_increase_stiffness";
            } else if(m.identifiable && m.damping_ratio>1.4 && m.effective_delay_s*m.natural_frequency_rad_s<.2) {
                kd*=.9;out.reasons[i]="measured_excess_damping_reduce_damping";
            } else out.reasons[i]="measured_response_needs_validation";
        }
        const double high_w=2*pi*req.frequencies_hz.back();
        const auto bounded=constrain_candidate(kp,kd,limits[i],2*req.amplitude_rad,2*req.amplitude_rad*(high_w+3));
        out.gains.kp[i]=bounded.first;out.gains.kd[i]=bounded.second;
        if(kd>limits[i].kd_max)out.reasons[i]+=";kd_protocol_ceiling";
        if(kp>limits[i].kp_max)out.reasons[i]+=";kp_protocol_ceiling";
        if(bounded.first<std::min(kp,limits[i].kp_max) || bounded.second<std::min(kd,limits[i].kd_max))out.reasons[i]+=";joint_effort_working_domain";
        out.changed=out.changed || out.gains.kp[i]!=current.kp[i] || out.gains.kd[i]!=current.kd[i];
    }
    return out;
}

enum class CalibrationMode {QuickAll,FineSingle};
struct CalibrationBudget {
    double recorded_duration_s{0},target_s{0},hard_limit_s{0},exit_reserve_s{0};
    CalibrationMode mode{CalibrationMode::QuickAll};
    explicit CalibrationBudget(double original_recorded_duration_s,CalibrationMode selected):recorded_duration_s(original_recorded_duration_s),mode(selected) {
        if(!std::isfinite(recorded_duration_s) || recorded_duration_s<=0)throw std::invalid_argument("gain budget requires trustworthy original recording duration");
        target_s=12*recorded_duration_s;hard_limit_s=(mode==CalibrationMode::QuickAll?15:24)*recorded_duration_s;
        exit_reserve_s=std::min(2.0,.1*recorded_duration_s);
    }
    double remaining(double elapsed_s)const{return std::max(0.0,hard_limit_s-std::max(0.0,elapsed_s));}
    bool expired(double elapsed_s)const{return !std::isfinite(elapsed_s) || elapsed_s>=hard_limit_s;}
    bool can_start(double elapsed_s,double complete_trial_cost_s,double validation_reserve_s=0)const {
        return std::isfinite(elapsed_s) && elapsed_s>=0 && std::isfinite(complete_trial_cost_s) && complete_trial_cost_s>=0 &&
            std::isfinite(validation_reserve_s) && validation_reserve_s>=0 &&
            complete_trial_cost_s+validation_reserve_s+exit_reserve_s<=remaining(elapsed_s);
    }
};

} // namespace serial_arm::servo_tuning
