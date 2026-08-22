#include "serial_arm/interaction/interaction_controller.hpp"

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

    TorqueResidualObserver residual_observer;
    GeneralizedMomentumObserver momentum_observer;
    ExternalTorqueObserver external_torque_observer;
    JointAdmittanceController admittance_controller;

    if(!residual_observer.configure(cfg.residual) ||
        !external_torque_observer.configure(cfg.external_torque) ||
        !admittance_controller.configure(cfg.admittance)) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }
    if(cfg.observer_mode == AdmittanceObserverMode::MOMENTUM &&
        !momentum_observer.configure(cfg.momentum)) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }

    cfg_ = cfg;
    residual_observer_ = std::move(residual_observer);
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
    if(!cfg_.enabled) return output;

    if(input.measured_torque.size() != input.full_id_model_torque.size()) {
        return tl::make_unexpected(InteractionControllerErr::OBSERVER_FAILED);
    }
    output.full_id_residual_raw.resize(input.measured_torque.size());
    for(std::size_t i = 0; i < input.measured_torque.size(); ++i) {
        output.full_id_residual_raw[i] = input.full_id_model_torque[i] - input.measured_torque[i];
    }

    TorqueResidualEstimate selected_residual;
    if(cfg_.observer_mode == AdmittanceObserverMode::MOMENTUM) {
        auto momentum = momentum_observer_.update(input.momentum_input);
        if(!momentum) return tl::make_unexpected(InteractionControllerErr::MOMENTUM_FAILED);
        JointVector zeros(momentum->tau_ext_hat.size(), 0.0);
        auto filtered = residual_observer_.update(zeros, momentum->tau_ext_hat);
        if(!filtered) return tl::make_unexpected(InteractionControllerErr::OBSERVER_FAILED);
        selected_residual = std::move(filtered.value());
    }
    else {
        auto residual = residual_observer_.update(input.measured_torque, input.full_id_model_torque);
        if(!residual) return tl::make_unexpected(InteractionControllerErr::OBSERVER_FAILED);
        selected_residual = std::move(residual.value());
    }
    output.residual = selected_residual;

    JointVector measured_velocity = input.measured_velocity;
    if(measured_velocity.empty()) measured_velocity.assign(cfg_.external_torque.joints_count, 0.0);
    auto tau_ext = external_torque_observer_.update(output.residual, measured_velocity);
    if(!tau_ext) return tl::make_unexpected(InteractionControllerErr::EXTERNAL_FAILED);

    auto admittance = admittance_controller_.update(JointAdmittanceInput{
        tau_ext->tau_ext_hat,
        input.dt,
        input.min_delta_q,
        input.max_delta_q,
        input.min_delta_q_dot,
        input.max_delta_q_dot,
    });
    if(!admittance) return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    if(output.corrected_cmd.pos.size() != admittance->delta_q.size() ||
        output.corrected_cmd.vel.size() != admittance->delta_q_dot.size()) {
        return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    }

    output.bias_compensated = std::move(tau_ext->bias_compensated);
    output.friction_residual_hat = std::move(tau_ext->friction_residual_hat);
    output.friction_compensated = std::move(tau_ext->friction_compensated);
    output.tau_ext_hat = std::move(tau_ext->tau_ext_hat);
    output.threshold_active = std::move(tau_ext->threshold_active);
    output.delta_q = std::move(admittance->delta_q);
    output.delta_q_dot = std::move(admittance->delta_q_dot);
    output.delta_q_limited = std::move(admittance->delta_q_limited);
    output.delta_q_dot_limited = std::move(admittance->delta_q_dot_limited);

    for(std::size_t i = 0; i < output.delta_q.size(); ++i) {
        // Outer admittance only shifts the reference produced by the existing joint controller
        output.corrected_cmd.pos[i] += output.delta_q[i];
        output.corrected_cmd.vel[i] += output.delta_q_dot[i];
    }
    return output;
}

void InteractionController::reset() {
    if(!is_configured_ || !cfg_.enabled) return;
    residual_observer_.reset();
    momentum_observer_.reset();
    external_torque_observer_.reset();
    admittance_controller_.reset();
}

bool InteractionController::is_configured() const noexcept {
    return is_configured_;
}

bool InteractionController::is_enabled() const noexcept {
    return is_configured_ && cfg_.enabled;
}

} // namespace serial_arm
