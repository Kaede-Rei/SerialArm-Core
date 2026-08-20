#include "serial_arm/interaction/external_torque_observer.hpp"

#include <algorithm>
#include <cmath>

namespace serial_arm {
namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool valid_optional_joint_vector(const JointVector& values, std::size_t joints_count) {
    return values.empty() || (values.size() == joints_count && finite_vector(values));
}

JointVector zeros_if_empty(const JointVector& values, std::size_t joints_count) {
    return values.empty() ? JointVector(joints_count, 0.0) : values;
}

double smoothstep01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// Friction compensation is never allowed to manufacture torque in the opposite direction
// or remove more residual than is actually observed in the current sample.
double bounded_same_sign_component(double predicted, double observed) {
    if(predicted * observed <= 0.0) return 0.0;
    return std::copysign(std::min(std::abs(predicted), std::abs(observed)), predicted);
}

} // namespace

tl::expected<void, ExternalTorqueObserverErr> ExternalTorqueObserver::configure(const ExternalTorqueObserverCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0) return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    if(!valid_optional_joint_vector(cfg.torque_bias, cfg.joints_count) ||
        !valid_optional_joint_vector(cfg.torque_threshold, cfg.joints_count)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }
    if(!cfg.torque_threshold.empty() && !std::all_of(cfg.torque_threshold.begin(), cfg.torque_threshold.end(), [](double value) {
        return value >= 0.0;
    })) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }

    const auto& friction = cfg.friction;
    if(!std::isfinite(friction.velocity_transition) || friction.velocity_transition <= 0.0 ||
        !std::isfinite(friction.zero_velocity_adaptation_s) || friction.zero_velocity_adaptation_s <= 0.0 ||
        !std::isfinite(friction.kinetic_feedforward_scale) || friction.kinetic_feedforward_scale < 0.0 ||
        friction.kinetic_feedforward_scale > 0.7 ||
        !valid_optional_joint_vector(friction.positive_coulomb, cfg.joints_count) ||
        !valid_optional_joint_vector(friction.positive_viscous, cfg.joints_count) ||
        !valid_optional_joint_vector(friction.negative_coulomb, cfg.joints_count) ||
        !valid_optional_joint_vector(friction.negative_viscous, cfg.joints_count)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }

    cfg_ = cfg;
    cfg_.torque_bias = zeros_if_empty(cfg_.torque_bias, cfg_.joints_count);
    cfg_.torque_threshold = zeros_if_empty(cfg_.torque_threshold, cfg_.joints_count);
    cfg_.friction.positive_coulomb = zeros_if_empty(cfg_.friction.positive_coulomb, cfg_.joints_count);
    cfg_.friction.positive_viscous = zeros_if_empty(cfg_.friction.positive_viscous, cfg_.joints_count);
    cfg_.friction.negative_coulomb = zeros_if_empty(cfg_.friction.negative_coulomb, cfg_.joints_count);
    cfg_.friction.negative_viscous = zeros_if_empty(cfg_.friction.negative_viscous, cfg_.joints_count);
    last_motion_direction_.assign(cfg_.joints_count, 0);
    static_friction_baseline_.assign(cfg_.joints_count, 0.0);
    static_baseline_initialized_.assign(cfg_.joints_count, 0);
    was_moving_.assign(cfg_.joints_count, 0);
    is_configured_ = true;
    return {};
}

tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr>
ExternalTorqueObserver::update(const TorqueResidualEstimate& residual) {
    return update(residual, JointVector(cfg_.joints_count, 0.0), 0.005);
}

tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr>
ExternalTorqueObserver::update(
    const TorqueResidualEstimate& residual,
    const JointVector& joint_velocity,
    double dt) {
    if(!is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::NOT_CONFIGURED);
    if(residual.residual_filtered.size() != cfg_.joints_count || joint_velocity.size() != cfg_.joints_count) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(residual.residual_filtered) || !finite_vector(joint_velocity)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
    }
    if(!std::isfinite(dt) || dt <= 0.0) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_DT);
    }

    ExternalTorqueEstimate estimate;
    estimate.bias_compensated.resize(cfg_.joints_count);
    estimate.friction_residual_hat.assign(cfg_.joints_count, 0.0);
    estimate.friction_compensated.resize(cfg_.joints_count);
    estimate.tau_ext_hat.resize(cfg_.joints_count);
    estimate.contact_confidence.assign(cfg_.joints_count, 0.0);
    estimate.threshold_active.assign(cfg_.joints_count, 0);

    const double static_adaptation_alpha =
        1.0 - std::exp(-dt / cfg_.friction.zero_velocity_adaptation_s);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double bias_compensated = residual.residual_filtered[i] - cfg_.torque_bias[i];
        if(!std::isfinite(bias_compensated)) {
            return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
        }
        estimate.bias_compensated[i] = bias_compensated;

        double friction_residual_hat = 0.0;
        if(cfg_.friction.enabled) {
            const double velocity = joint_velocity[i];
            const double abs_velocity = std::abs(velocity);
            const double motion_ratio = smoothstep01(abs_velocity / cfg_.friction.velocity_transition);
            const bool direction_reliable = abs_velocity >= cfg_.friction.velocity_transition;

            if(direction_reliable) {
                const std::int8_t direction = velocity > 0.0 ? 1 : -1;
                last_motion_direction_[i] = direction;
                was_moving_[i] = 1;

                const double coulomb = direction > 0 ?
                    cfg_.friction.positive_coulomb[i] : cfg_.friction.negative_coulomb[i];
                const double viscous = direction > 0 ?
                    cfg_.friction.positive_viscous[i] : cfg_.friction.negative_viscous[i];
                const double dynamic_prediction = coulomb + viscous * abs_velocity;
                friction_residual_hat = bounded_same_sign_component(dynamic_prediction, bias_compensated);
            }
            else {
                const std::int8_t direction = last_motion_direction_[i];
                double low_speed_dynamic_prediction = 0.0;
                if(direction != 0) {
                    const double coulomb = direction > 0 ?
                        cfg_.friction.positive_coulomb[i] : cfg_.friction.negative_coulomb[i];
                    const double viscous = direction > 0 ?
                        cfg_.friction.positive_viscous[i] : cfg_.friction.negative_viscous[i];
                    low_speed_dynamic_prediction = coulomb + viscous * abs_velocity;
                }

                // On the first low-speed sample after a real motion, latch only the part that is
                // physically compatible with the learned friction envelope. This keeps post-motion
                // stiction/hysteresis out of tau_ext without ever inventing an opposite external torque.
                if(was_moving_[i] != 0) {
                    static_friction_baseline_[i] =
                        bounded_same_sign_component(low_speed_dynamic_prediction, bias_compensated);
                    static_baseline_initialized_[i] =
                        std::abs(static_friction_baseline_[i]) > 0.0 ? 1 : 0;
                    was_moving_[i] = 0;
                }

                if(static_baseline_initialized_[i] != 0) {
                    const double baseline_error = bias_compensated - static_friction_baseline_[i];
                    const double threshold = cfg_.torque_threshold[i];
                    const bool no_new_external_evidence = threshold > 0.0 ?
                        std::abs(baseline_error) <= threshold : std::abs(baseline_error) <= 1.0e-12;
                    if(no_new_external_evidence) {
                        static_friction_baseline_[i] += static_adaptation_alpha * baseline_error;
                    }
                }

                const double static_prediction = static_baseline_initialized_[i] != 0 ?
                    static_friction_baseline_[i] : 0.0;
                const double blended_prediction =
                    motion_ratio * low_speed_dynamic_prediction + (1.0 - motion_ratio) * static_prediction;
                friction_residual_hat = bounded_same_sign_component(blended_prediction, bias_compensated);
            }
        }

        estimate.friction_residual_hat[i] = friction_residual_hat;
        const double friction_compensated = bias_compensated - friction_residual_hat;
        estimate.friction_compensated[i] = friction_compensated;

        const double threshold = cfg_.torque_threshold[i];
        const double magnitude = std::abs(friction_compensated);
        if(threshold > 0.0) {
            // confidence starts at the calibrated no-contact envelope and reaches 1 at 3*threshold.
            estimate.contact_confidence[i] = smoothstep01((magnitude - threshold) / (2.0 * threshold));
        }
        else {
            estimate.contact_confidence[i] = magnitude > 0.0 ? 1.0 : 0.0;
        }

        if(threshold <= 0.0 || magnitude >= 2.0 * threshold) {
            estimate.tau_ext_hat[i] = friction_compensated;
            continue;
        }

        estimate.threshold_active[i] = 1;
        if(magnitude <= threshold) {
            estimate.tau_ext_hat[i] = 0.0;
            continue;
        }

        const double u = (magnitude - threshold) / threshold;
        const double gain = smoothstep01(u);
        estimate.tau_ext_hat[i] = friction_compensated * gain;
    }
    return estimate;
}

void ExternalTorqueObserver::reset() {
    if(!is_configured_) return;
    std::fill(last_motion_direction_.begin(), last_motion_direction_.end(), 0);
    std::fill(static_friction_baseline_.begin(), static_friction_baseline_.end(), 0.0);
    std::fill(static_baseline_initialized_.begin(), static_baseline_initialized_.end(), 0);
    std::fill(was_moving_.begin(), was_moving_.end(), 0);
}

bool ExternalTorqueObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
