#include "serial_arm/interaction/runtime/interaction_controller.hpp"

#include <utility>

namespace serial_arm {

tl::expected<void, InteractionControllerErr>
InteractionController::configure(const InteractionControllerCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(InteractionControllerErr::ALREADY_CONFIGURED);

    if(!cfg.enabled) {
        cfg_ = cfg;
        is_configured_ = true;
        return {};
    }

    GeneralizedMomentumObserver momentum_observer;
    GeneralizedMomentumObserver raw_momentum_observer;
    ExternalTorqueObserver net_external_torque_observer;
    ExternalTorqueObserver external_torque_observer;
    JointAdmittanceController admittance_controller;

    if(!external_torque_observer.configure(cfg.external_torque) ||
        !admittance_controller.configure(cfg.admittance)) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }
    if(!momentum_observer.configure([&]{auto c=cfg.momentum;c.initial_residual.assign(c.joints_count,0.0);return c;}())) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }

    {
        auto net_cfg=cfg.external_torque;
        net_cfg.torque_bias.assign(net_cfg.joints_count,0.0);
        net_cfg.friction.enabled=false;
        if(!raw_momentum_observer.configure(cfg.momentum) || !net_external_torque_observer.configure(net_cfg))
            return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }
    cfg_ = cfg;
    raw_momentum_observer_=std::move(raw_momentum_observer);
    net_external_torque_observer_=std::move(net_external_torque_observer);
    momentum_observer_ = std::move(momentum_observer);
    external_torque_observer_ = std::move(external_torque_observer);
    admittance_controller_ = std::move(admittance_controller);
    is_configured_ = true;
    return {};
}

tl::expected<InteractionOutput, InteractionControllerErr>
InteractionController::update(const InteractionInput& input) {
    if(!is_configured_) return tl::make_unexpected(InteractionControllerErr::NOT_CONFIGURED);

    InteractionOutput output;
    output.corrected_cmd = input.nominal_cmd;
    output.state.valid = false;
    if(!cfg_.enabled) return output;

    const bool usable = input.observation_valid;
    if(!usable || (!input.new_measurement && !observation_initialized_)) {
        // Missing measurement authority never becomes an estimated zero interaction
        if(!usable) { momentum_observer_.reset(); raw_momentum_observer_.reset(); observation_initialized_=false; }
        admittance_controller_.advance_parameters(input.dt);
        auto held=admittance_controller_.hold(input.min_delta_q,input.max_delta_q);
        if(!held) return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
        output.delta_q=held->delta_q;
        output.delta_q_dot=held->delta_q_dot;
        output.delta_q_ddot=held->delta_q_ddot;
        output.delta_q_limited=held->delta_q_limited;
        output.delta_q_dot_limited=held->delta_q_dot_limited;
        output.state.delta_q=output.delta_q;
        output.state.delta_q_dot=output.delta_q_dot;
        if(output.corrected_cmd.pos.size()!=output.delta_q.size() || output.corrected_cmd.vel.size()!=output.delta_q.size())
            return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
        if(input.apply_admittance) for(std::size_t i=0;i<output.delta_q.size();++i) output.corrected_cmd.pos[i]+=output.delta_q[i];
        return output;
    }

    if(input.new_measurement) {
        if(input.full_id_acceleration_valid) {
            if(input.measured_torque.size()!=input.full_id_model_torque.size())
                return tl::make_unexpected(InteractionControllerErr::OBSERVER_FAILED);
            output.full_id_residual_raw.resize(input.measured_torque.size());
            for(std::size_t i=0;i<output.full_id_residual_raw.size();++i)
                output.full_id_residual_raw[i]=input.full_id_model_torque[i]-input.measured_torque[i];
        }
        JointVector measured_velocity=input.measured_velocity;
        if(measured_velocity.empty()) measured_velocity.assign(cfg_.external_torque.joints_count,0.0);
        const auto n=cfg_.external_torque.joints_count;
        TorqueResidualEstimate zero;zero.residual.assign(n,0.0);
        auto correction=external_torque_observer_.update(zero,measured_velocity);
        if(!correction) return tl::make_unexpected(InteractionControllerErr::EXTERNAL_FAILED);
        auto momentum_input=input.momentum_input;
        momentum_input.dt=input.measurement_dt>0?input.measurement_dt:input.dt;
        momentum_input.known_residual.resize(n);
        for(std::size_t i=0;i<n;++i) momentum_input.known_residual[i]=
            (cfg_.external_torque.torque_bias.empty()?0.0:cfg_.external_torque.torque_bias[i])+
            correction->friction_residual_hat[i];
        auto momentum=momentum_observer_.update(momentum_input);
        if(!momentum) return tl::make_unexpected(InteractionControllerErr::MOMENTUM_FAILED);
        auto raw_input=momentum_input;
        raw_input.known_residual.clear();
        auto raw=raw_momentum_observer_.update(raw_input);
        if(!raw) return tl::make_unexpected(InteractionControllerErr::MOMENTUM_FAILED);
        output.residual.residual=raw->tau_ext_hat;
        output.residual.residual_filtered=output.residual.residual;
        auto estimated=external_torque_observer_.update(output.residual,measured_velocity);
        TorqueResidualEstimate cleaned;cleaned.residual=momentum->tau_ext_hat;
        auto thresholded=net_external_torque_observer_.update(cleaned,measured_velocity);
        if(!thresholded || !estimated) return tl::make_unexpected(InteractionControllerErr::EXTERNAL_FAILED);
        estimated->friction_compensated=momentum->tau_ext_hat;
        estimated->tau_ext_hat=std::move(thresholded->tau_ext_hat);
        estimated->threshold_active=std::move(thresholded->threshold_active);
        if(!estimated) return tl::make_unexpected(InteractionControllerErr::EXTERNAL_FAILED);
        output.bias_compensated=std::move(estimated->bias_compensated);
        output.friction_residual_hat=std::move(estimated->friction_residual_hat);
        output.friction_compensated=std::move(estimated->friction_compensated);
        output.tau_ext_hat=std::move(estimated->tau_ext_hat);
        output.threshold_active=std::move(estimated->threshold_active);
        last_observation_=output;
        observation_initialized_=true;
    } else {
        // Keep the last trustworthy estimate between fresh sensor groups; only the
        // reference generator advances on the independent actuator execution clock
        output.residual=last_observation_.residual;
        output.full_id_residual_raw=last_observation_.full_id_residual_raw;
        output.bias_compensated=last_observation_.bias_compensated;
        output.friction_residual_hat=last_observation_.friction_residual_hat;
        output.friction_compensated=last_observation_.friction_compensated;
        output.tau_ext_hat=last_observation_.tau_ext_hat;
        output.threshold_active=last_observation_.threshold_active;
    }

    if(!input.apply_admittance) {
        admittance_controller_.advance_parameters(input.dt);
        output.state.valid=true;
        output.state.tau_ext_hat=output.tau_ext_hat;
        const auto n=input.nominal_cmd.pos.size();
        output.delta_q.assign(n,0.0);output.delta_q_dot.assign(n,0.0);output.delta_q_ddot.assign(n,0.0);
        output.state.delta_q=output.delta_q;output.state.delta_q_dot=output.delta_q_dot;
        return output;
    }

    auto admittance = admittance_controller_.update(JointAdmittanceInput{
        output.tau_ext_hat,
        input.dt,
        input.min_delta_q,
        input.max_delta_q,
        input.min_delta_q_dot,
        input.max_delta_q_dot,
        input.drag,
    });
    if(!admittance) return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    if(output.corrected_cmd.pos.size() != admittance->delta_q.size() ||
        output.corrected_cmd.vel.size() != admittance->delta_q_dot.size()) {
        return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    }

    output.delta_q = std::move(admittance->delta_q);
    output.delta_q_dot = std::move(admittance->delta_q_dot);
    output.delta_q_ddot = std::move(admittance->delta_q_ddot);
    output.state.tau_ext_hat = output.tau_ext_hat;
    output.state.delta_q = output.delta_q;
    output.state.delta_q_dot = output.delta_q_dot;
    output.state.valid = true;
    output.delta_q_limited = std::move(admittance->delta_q_limited);
    output.delta_q_dot_limited = std::move(admittance->delta_q_dot_limited);

    for(std::size_t i = 0; i < output.delta_q.size(); ++i) {
        // Outer admittance only shifts the reference produced by the existing joint controller
        output.corrected_cmd.pos[i] += output.delta_q[i];
        output.corrected_cmd.vel[i] += output.delta_q_dot[i];
    }
    return output;
}

tl::expected<void, InteractionControllerErr>
InteractionController::reconfigure(const InteractionControllerCfg& cfg) {
    if(!is_configured_) return configure(cfg);
    InteractionController validated;
    auto valid=validated.configure(cfg);
    if(!valid) return valid;
    if(!cfg_.enabled || !cfg.enabled) { *this=std::move(validated); return {}; }
    const bool observer_changed=cfg_.momentum.gain!=cfg.momentum.gain ||
        cfg_.external_torque.torque_bias!=cfg.external_torque.torque_bias ||
        cfg_.external_torque.torque_threshold!=cfg.external_torque.torque_threshold ||
        cfg_.external_torque.friction.enabled!=cfg.external_torque.friction.enabled ||
        cfg_.external_torque.friction.velocity_transition!=cfg.external_torque.friction.velocity_transition ||
        cfg_.external_torque.friction.positive_coulomb!=cfg.external_torque.friction.positive_coulomb ||
        cfg_.external_torque.friction.positive_viscous!=cfg.external_torque.friction.positive_viscous ||
        cfg_.external_torque.friction.negative_coulomb!=cfg.external_torque.friction.negative_coulomb ||
        cfg_.external_torque.friction.negative_viscous!=cfg.external_torque.friction.negative_viscous;
    if(!admittance_controller_.reconfigure(cfg.admittance))
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    if(observer_changed) {
        momentum_observer_=std::move(validated.momentum_observer_);
        raw_momentum_observer_=std::move(validated.raw_momentum_observer_);
        net_external_torque_observer_=std::move(validated.net_external_torque_observer_);
        external_torque_observer_=std::move(validated.external_torque_observer_);
        observation_initialized_=false;
    }
    cfg_=cfg;
    return {};
}

tl::expected<void, InteractionControllerErr>
InteractionController::accept_reference(const JointVector& q,const JointVector& dq) {
    if(!admittance_controller_.accept(q,dq))
        return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    return {};
}

void InteractionController::reset_reference() {
    if(is_configured_ && cfg_.enabled) admittance_controller_.reset();
}

void InteractionController::reset() {
    if(!is_configured_ || !cfg_.enabled) return;
    momentum_observer_.reset();
    raw_momentum_observer_.reset();
    external_torque_observer_.reset();
    admittance_controller_.reset();
    observation_initialized_=false;
    last_observation_={};
}

bool InteractionController::is_configured() const noexcept {
    return is_configured_;
}

double InteractionController::parameter_update_progress() const noexcept {
    return admittance_controller_.parameter_update_progress();
}

bool InteractionController::is_enabled() const noexcept {
    return is_configured_ && cfg_.enabled;
}

} // namespace serial_arm
