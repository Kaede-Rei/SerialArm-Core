#include "serial_arm/interaction/generalized_momentum_observer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace serial_arm {
namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool valid_matrix(const std::vector<JointVector>& matrix, std::size_t n) {
    if(matrix.size() != n) return false;
    return std::all_of(matrix.begin(), matrix.end(), [n](const JointVector& row) {
        return row.size() == n && finite_vector(row);
    });
}

JointVector mat_vec(const std::vector<JointVector>& matrix, const JointVector& vector) {
    JointVector out(matrix.size(), 0.0);
    for(std::size_t i = 0; i < matrix.size(); ++i) {
        for(std::size_t j = 0; j < vector.size(); ++j) out[i] += matrix[i][j] * vector[j];
    }
    return out;
}

} // namespace

tl::expected<void, GeneralizedMomentumObserverErr> GeneralizedMomentumObserver::configure(
    const GeneralizedMomentumObserverCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(GeneralizedMomentumObserverErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0 || cfg.gain.size() != cfg.joints_count || !finite_vector(cfg.gain) ||
        (!cfg.initial_residual.empty() && (cfg.initial_residual.size() != cfg.joints_count || !finite_vector(cfg.initial_residual))) ||
        std::any_of(cfg.gain.begin(), cfg.gain.end(), [](double value) { return value <= 0.0; })) {
        return tl::make_unexpected(GeneralizedMomentumObserverErr::INVALID_CFG);
    }
    cfg_ = cfg;
    predicted_momentum_.assign(cfg.joints_count, 0.0);
    residual_ = cfg.initial_residual.empty() ? JointVector(cfg.joints_count, 0.0) : cfg.initial_residual;
    previous_mass_matrix_.assign(cfg.joints_count, JointVector(cfg.joints_count, 0.0));
    initialized_ = false;
    is_configured_ = true;
    return {};
}

tl::expected<GeneralizedMomentumOutput, GeneralizedMomentumObserverErr>
GeneralizedMomentumObserver::update(const GeneralizedMomentumInput& input) {
    if(!is_configured_) return tl::make_unexpected(GeneralizedMomentumObserverErr::NOT_CONFIGURED);
    const std::size_t n = cfg_.joints_count;
    if(input.measured_torque.size() != n || input.gravity.size() != n || input.coriolis.size() != n ||
        input.velocity.size() != n || !valid_matrix(input.mass_matrix, n)) {
        return tl::make_unexpected(GeneralizedMomentumObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(input.measured_torque) || !finite_vector(input.gravity) ||
        !finite_vector(input.coriolis) || !finite_vector(input.velocity)) {
        return tl::make_unexpected(GeneralizedMomentumObserverErr::NON_FINITE_INPUT);
    }
    if(!std::isfinite(input.dt) || input.dt <= 0.0) {
        return tl::make_unexpected(GeneralizedMomentumObserverErr::INVALID_DT);
    }

    const JointVector momentum = mat_vec(input.mass_matrix, input.velocity);
    if(!initialized_) {
        residual_ = cfg_.initial_residual.empty() ? JointVector(n, 0.0) : cfg_.initial_residual;
        predicted_momentum_.resize(n);
        JointVector momentum_error(n, 0.0);
        for(std::size_t i = 0; i < n; ++i) {
            // Keep the observer algebra r=K*(p-p_hat) consistent on the very first
            // sample. Initialising p_hat=p would force a calibrated non-zero bias to
            // collapse to zero on sample two and then reconverge, which appears as a
            // false interaction transient at activation/reset.
            momentum_error[i] = residual_[i] / cfg_.gain[i];
            predicted_momentum_[i] = momentum[i] - momentum_error[i];
        }
        previous_mass_matrix_ = input.mass_matrix;
        initialized_ = true;
        return GeneralizedMomentumOutput{ residual_, std::move(momentum_error) };
    }

    JointVector mdot_times_dq(n, 0.0);
    for(std::size_t i = 0; i < n; ++i) {
        for(std::size_t j = 0; j < n; ++j) {
            mdot_times_dq[i] +=
                (input.mass_matrix[i][j] - previous_mass_matrix_[i][j]) / input.dt * input.velocity[j];
        }
    }

    // p_dot = tau + tau_ext - g + C^T*dq,
    // C^T*dq = Mdot*dq - C*dq. The integral state predicts momentum using the
    // previous residual; the current momentum error closes the observer loop.
    for(std::size_t i = 0; i < n; ++i) {
        const double c_transpose_dq = mdot_times_dq[i] - input.coriolis[i];
        const double known_momentum_rate = input.measured_torque[i] - input.gravity[i] + c_transpose_dq;
        predicted_momentum_[i] += input.dt * (known_momentum_rate + residual_[i]);
    }

    GeneralizedMomentumOutput output;
    output.momentum_error.resize(n);
    output.tau_ext_hat.resize(n);
    for(std::size_t i = 0; i < n; ++i) {
        output.momentum_error[i] = momentum[i] - predicted_momentum_[i];
        residual_[i] = cfg_.gain[i] * output.momentum_error[i];
        output.tau_ext_hat[i] = residual_[i];
    }
    previous_mass_matrix_ = input.mass_matrix;
    return output;
}

void GeneralizedMomentumObserver::reset() {
    if(!is_configured_) return;
    std::fill(predicted_momentum_.begin(), predicted_momentum_.end(), 0.0);
    residual_ = cfg_.initial_residual.empty() ? JointVector(cfg_.joints_count, 0.0) : cfg_.initial_residual;
    for(auto& row : previous_mass_matrix_) std::fill(row.begin(), row.end(), 0.0);
    initialized_ = false;
}

bool GeneralizedMomentumObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
