#include "serial_arm/interaction/estimators/torque_residual_observer.hpp"

#include <algorithm>
#include <cmath>

namespace serial_arm {

namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

bool valid_vector(const JointVector& values, std::size_t expected_size) {
    return values.size() == expected_size && finite_vector(values);
}

} // namespace

tl::expected<void, TorqueResidualObserverErr> TorqueResidualObserver::configure(const TorqueResidualObserverCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(TorqueResidualObserverErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0) {
        return tl::make_unexpected(TorqueResidualObserverErr::INVALID_CFG);
    }

    cfg_ = cfg;
    is_configured_ = true;
    return {};
}

tl::expected<TorqueResidualEstimate, TorqueResidualObserverErr> TorqueResidualObserver::update(
    const JointVector& measured_torque,
    const JointVector& model_torque) {
    if(!is_configured_) return tl::make_unexpected(TorqueResidualObserverErr::NOT_CONFIGURED);
    if(measured_torque.size() != cfg_.joints_count || model_torque.size() != cfg_.joints_count) {
        return tl::make_unexpected(TorqueResidualObserverErr::INVALID_INPUT_SIZE);
    }
    if(!valid_vector(measured_torque, cfg_.joints_count) || !valid_vector(model_torque, cfg_.joints_count)) {
        return tl::make_unexpected(TorqueResidualObserverErr::NON_FINITE_INPUT);
    }

    TorqueResidualEstimate estimate;
    estimate.residual.resize(cfg_.joints_count);
    estimate.residual_filtered.resize(cfg_.joints_count);
    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        estimate.residual[i] = model_torque[i] - measured_torque[i];
        estimate.residual_filtered[i] = estimate.residual[i];
    }
    return estimate;
}

void TorqueResidualObserver::reset() {
    // Stateless residual calculator; kept for lifecycle compatibility
}

bool TorqueResidualObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
