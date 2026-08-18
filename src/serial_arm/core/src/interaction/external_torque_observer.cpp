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

} // namespace

tl::expected<void, ExternalTorqueObserverErr> ExternalTorqueObserver::configure(const ExternalTorqueObserverCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0) return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    if((!cfg.torque_bias.empty() && (cfg.torque_bias.size() != cfg.joints_count || !finite_vector(cfg.torque_bias))) ||
        (!cfg.torque_threshold.empty() && (cfg.torque_threshold.size() != cfg.joints_count || !finite_vector(cfg.torque_threshold)))) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }
    if(!cfg.torque_threshold.empty() && !std::all_of(cfg.torque_threshold.begin(), cfg.torque_threshold.end(), [](double value) {
        return value >= 0.0;
        })) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }

    cfg_ = cfg;
    if(cfg_.torque_bias.empty()) cfg_.torque_bias.assign(cfg_.joints_count, 0.0);
    if(cfg_.torque_threshold.empty()) cfg_.torque_threshold.assign(cfg_.joints_count, 0.0);
    is_configured_ = true;
    return {};
}

tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> ExternalTorqueObserver::update(const TorqueResidualEstimate& residual) const {
    if(!is_configured_) return tl::make_unexpected(ExternalTorqueObserverErr::NOT_CONFIGURED);
    if(residual.residual_filtered.size() != cfg_.joints_count) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(residual.residual_filtered)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
    }

    ExternalTorqueEstimate estimate;
    estimate.bias_compensated.resize(cfg_.joints_count);
    estimate.tau_ext_hat.resize(cfg_.joints_count);
    estimate.threshold_active.assign(cfg_.joints_count, 0);
    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double bias_compensated = residual.residual_filtered[i] - cfg_.torque_bias[i];
        if(!std::isfinite(bias_compensated)) {
            return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
        }

        estimate.bias_compensated[i] = bias_compensated;
        const double threshold = cfg_.torque_threshold[i];
        const double magnitude = std::abs(bias_compensated);
        if(threshold <= 0.0 || magnitude >= 2.0 * threshold) {
            estimate.tau_ext_hat[i] = bias_compensated;
            continue;
        }

        estimate.threshold_active[i] = 1;
        if(magnitude <= threshold) {
            estimate.tau_ext_hat[i] = 0.0;
            continue;
        }

        const double u = (magnitude - threshold) / threshold;
        const double gain = u * u * (3.0 - 2.0 * u);
        estimate.tau_ext_hat[i] = bias_compensated * gain;
    }
    return estimate;
}

bool ExternalTorqueObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
