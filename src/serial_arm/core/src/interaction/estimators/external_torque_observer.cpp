#include "serial_arm/interaction/estimators/external_torque_observer.hpp"

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

} // namespace

tl::expected<void, ExternalTorqueObserverErr>
ExternalTorqueObserver::configure(const ExternalTorqueObserverCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0) return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    if(!valid_optional_joint_vector(cfg.torque_bias, cfg.joints_count) ||
        !valid_optional_joint_vector(cfg.torque_threshold, cfg.joints_count)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }
    if(!cfg.torque_threshold.empty() && !std::all_of(
        cfg.torque_threshold.begin(), cfg.torque_threshold.end(), [](double value) {
            return value >= 0.0;
        })) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }

    const auto& friction = cfg.friction;
    if(!std::isfinite(friction.velocity_transition) || friction.velocity_transition <= 0.0 ||
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
    is_configured_ = true;
    return {};
}

tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr>
ExternalTorqueObserver::update(const TorqueResidualEstimate& residual) {
    return update(residual, JointVector(cfg_.joints_count, 0.0));
}

tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr>
ExternalTorqueObserver::update(
    const TorqueResidualEstimate& residual,
    const JointVector& joint_velocity) {
    if(!is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::NOT_CONFIGURED);
    if(residual.residual.size() != cfg_.joints_count ||
        joint_velocity.size() != cfg_.joints_count) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(residual.residual) || !finite_vector(joint_velocity)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
    }

    ExternalTorqueEstimate estimate;
    estimate.bias_compensated.resize(cfg_.joints_count);
    estimate.friction_residual_hat.assign(cfg_.joints_count, 0.0);
    estimate.friction_compensated.resize(cfg_.joints_count);
    estimate.tau_ext_hat.resize(cfg_.joints_count);
    estimate.threshold_active.assign(cfg_.joints_count, 0);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double bias_compensated = residual.residual[i] - cfg_.torque_bias[i];
        if(!std::isfinite(bias_compensated)) {
            return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
        }
        estimate.bias_compensated[i] = bias_compensated;

        double friction_residual_hat = 0.0;
        const double speed = std::abs(joint_velocity[i]);
        if(cfg_.friction.enabled && speed > 0.0) {
            const bool positive = joint_velocity[i] > 0.0;
            const double coulomb = positive ?
                cfg_.friction.positive_coulomb[i] : cfg_.friction.negative_coulomb[i];
            const double viscous = positive ?
                cfg_.friction.positive_viscous[i] : cfg_.friction.negative_viscous[i];
            const double dynamic_prediction = coulomb + viscous * speed;
            const double normalized_speed = std::clamp(speed / cfg_.friction.velocity_transition, 0.0, 1.0);
            const double blend = normalized_speed * normalized_speed * (3.0 - 2.0 * normalized_speed);
            friction_residual_hat = blend * dynamic_prediction;
        }
        estimate.friction_residual_hat[i] = friction_residual_hat;

        const double friction_compensated = bias_compensated - friction_residual_hat;
        estimate.friction_compensated[i] = friction_compensated;

        const double threshold = cfg_.torque_threshold[i];
        if(threshold <= 0.0) {
            estimate.tau_ext_hat[i] = friction_compensated;
            continue;
        }

        const double magnitude = std::abs(friction_compensated);
        if(magnitude <= threshold) {
            estimate.threshold_active[i] = 1;
            estimate.tau_ext_hat[i] = 0.0;
            continue;
        }

        // Continuous deadband removes the calibrated no-contact envelope without a jump at threshold
        estimate.tau_ext_hat[i] = std::copysign(magnitude - threshold, friction_compensated);
    }

    return estimate;
}

void ExternalTorqueObserver::reset() {
    // Stateless after configure; kept for the InteractionController lifecycle contract
}

bool ExternalTorqueObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
