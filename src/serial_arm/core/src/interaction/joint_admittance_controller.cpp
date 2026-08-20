#include "serial_arm/interaction/joint_admittance_controller.hpp"

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

bool valid_cfg_vector(const JointVector& values, std::size_t expected_size, bool allow_zero) {
    if(values.size() != expected_size || !finite_vector(values)) return false;
    return std::all_of(values.begin(), values.end(), [allow_zero](double value) {
        return allow_zero ? value >= 0.0 : value > 0.0;
    });
}

double smoothstep01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

} // namespace

tl::expected<AdmittanceDampingMetrics, JointAdmittanceControllerErr>
compute_admittance_damping_metrics(double mass, double damping, double stiffness) {
    if(!std::isfinite(mass) || !std::isfinite(damping) || !std::isfinite(stiffness) ||
        mass <= 0.0 || damping < 0.0 || stiffness <= 0.0) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }

    const double critical_damping = 2.0 * std::sqrt(mass * stiffness);
    const double natural_frequency = std::sqrt(stiffness / mass);
    return AdmittanceDampingMetrics{
        critical_damping,
        damping / critical_damping,
        natural_frequency,
        4.74 / natural_frequency,
    };
}

tl::expected<void, JointAdmittanceControllerErr>
JointAdmittanceController::configure(const JointAdmittanceControllerCfg& cfg) {
    if(is_configured_) return tl::make_unexpected(JointAdmittanceControllerErr::ALREADY_CONFIGURED);
    if(cfg.joints_count == 0 ||
        cfg.enabled.size() != cfg.joints_count ||
        !valid_cfg_vector(cfg.mass, cfg.joints_count, false) ||
        !valid_cfg_vector(cfg.damping, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.stiffness, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.max_delta_q, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.max_delta_q_dot, cfg.joints_count, true)) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }
    if(!std::all_of(cfg.enabled.begin(), cfg.enabled.end(), [](std::uint8_t value) {
        return value == 0 || value == 1;
    })) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }
    if(cfg.variable.enabled &&
        (!std::isfinite(cfg.variable.engage_time_s) || cfg.variable.engage_time_s <= 0.0 ||
         !std::isfinite(cfg.variable.release_time_s) || cfg.variable.release_time_s <= 0.0 ||
         !std::isfinite(cfg.variable.soft_velocity_ratio) || cfg.variable.soft_velocity_ratio <= 0.0 ||
         cfg.variable.soft_velocity_ratio >= 1.0 ||
         !std::isfinite(cfg.variable.max_damping_multiplier) || cfg.variable.max_damping_multiplier < 1.0 ||
         (!cfg.variable.soft_velocity.empty() &&
          !valid_cfg_vector(cfg.variable.soft_velocity, cfg.joints_count, false)))) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }
    if(cfg.variable.enabled && !cfg.variable.soft_velocity.empty()) {
        for(std::size_t i = 0; i < cfg.joints_count; ++i) {
            if(cfg.variable.soft_velocity[i] >= cfg.max_delta_q_dot[i]) {
                return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
            }
        }
    }

    cfg_ = cfg;
    delta_q_.assign(cfg_.joints_count, 0.0);
    delta_q_dot_.assign(cfg_.joints_count, 0.0);
    contact_blend_.assign(cfg_.joints_count, 0.0);
    is_configured_ = true;
    return {};
}

tl::expected<JointAdmittanceOutput, JointAdmittanceControllerErr>
JointAdmittanceController::update(const JointAdmittanceInput& input) {
    if(!is_configured_) return tl::make_unexpected(JointAdmittanceControllerErr::NOT_CONFIGURED);
    if(input.tau_ext_hat.size() != cfg_.joints_count ||
        (!input.contact_confidence.empty() && input.contact_confidence.size() != cfg_.joints_count)) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(input.tau_ext_hat) ||
        (!input.contact_confidence.empty() && !finite_vector(input.contact_confidence))) {
        return tl::make_unexpected(JointAdmittanceControllerErr::NON_FINITE_INPUT);
    }
    if(!std::isfinite(input.dt) || input.dt <= 0.0) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_DT);
    }

    const bool has_dynamic_limits = !input.min_delta_q.empty() || !input.max_delta_q.empty() ||
        !input.min_delta_q_dot.empty() || !input.max_delta_q_dot.empty();
    if(has_dynamic_limits) {
        if(input.min_delta_q.size() != cfg_.joints_count ||
            input.max_delta_q.size() != cfg_.joints_count ||
            input.min_delta_q_dot.size() != cfg_.joints_count ||
            input.max_delta_q_dot.size() != cfg_.joints_count) {
            return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_INPUT_SIZE);
        }
        if(!finite_vector(input.min_delta_q) || !finite_vector(input.max_delta_q) ||
            !finite_vector(input.min_delta_q_dot) || !finite_vector(input.max_delta_q_dot)) {
            return tl::make_unexpected(JointAdmittanceControllerErr::NON_FINITE_INPUT);
        }
    }

    std::vector<std::uint8_t> delta_q_limited(cfg_.joints_count, 0);
    std::vector<std::uint8_t> delta_q_dot_limited(cfg_.joints_count, 0);
    JointVector effective_damping(cfg_.joints_count, 0.0);
    JointVector effective_stiffness(cfg_.joints_count, 0.0);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        if(cfg_.enabled[i] == 0) {
            delta_q_[i] = 0.0;
            delta_q_dot_[i] = 0.0;
            contact_blend_[i] = 0.0;
            continue;
        }

        double min_delta_q = -cfg_.max_delta_q[i];
        double max_delta_q = cfg_.max_delta_q[i];
        double min_delta_q_dot = -cfg_.max_delta_q_dot[i];
        double max_delta_q_dot = cfg_.max_delta_q_dot[i];
        if(has_dynamic_limits) {
            if(input.min_delta_q[i] > input.max_delta_q[i] ||
                input.min_delta_q_dot[i] > input.max_delta_q_dot[i]) {
                return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_DYNAMIC_LIMITS);
            }
            min_delta_q = std::max(min_delta_q, input.min_delta_q[i]);
            max_delta_q = std::min(max_delta_q, input.max_delta_q[i]);
            min_delta_q_dot = std::max(min_delta_q_dot, input.min_delta_q_dot[i]);
            max_delta_q_dot = std::min(max_delta_q_dot, input.max_delta_q_dot[i]);
            if(min_delta_q > max_delta_q || min_delta_q_dot > max_delta_q_dot) {
                return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_DYNAMIC_LIMITS);
            }
        }

        double d_eff = cfg_.damping[i];
        double k_eff = cfg_.stiffness[i];
        if(cfg_.variable.enabled) {
            const double target = input.contact_confidence.empty() ? 0.0 :
                std::clamp(input.contact_confidence[i], 0.0, 1.0);
            const double time_constant = target > contact_blend_[i] ?
                cfg_.variable.engage_time_s : cfg_.variable.release_time_s;
            const double blend_alpha = 1.0 - std::exp(-input.dt / time_constant);
            contact_blend_[i] += blend_alpha * (target - contact_blend_[i]);
            contact_blend_[i] = std::clamp(contact_blend_[i], 0.0, 1.0);

            k_eff = cfg_.stiffness[i] * (1.0 - contact_blend_[i]);
            const double return_damping = cfg_.stiffness[i] > 0.0 ?
                2.0 * std::sqrt(cfg_.mass[i] * cfg_.stiffness[i]) : cfg_.damping[i];
            d_eff = contact_blend_[i] * cfg_.damping[i] +
                (1.0 - contact_blend_[i]) * return_damping;

            const double hard_speed = cfg_.max_delta_q_dot[i];
            const double soft_speed = cfg_.variable.soft_velocity.empty() ?
                cfg_.variable.soft_velocity_ratio * hard_speed : cfg_.variable.soft_velocity[i];
            if(hard_speed > soft_speed && std::abs(delta_q_dot_[i]) > soft_speed) {
                const double ratio = (std::abs(delta_q_dot_[i]) - soft_speed) /
                    (hard_speed - soft_speed);
                const double wall = smoothstep01(ratio);
                d_eff *= 1.0 + (cfg_.variable.max_damping_multiplier - 1.0) * wall;
            }
        }
        else {
            contact_blend_[i] = 0.0;
        }
        effective_damping[i] = d_eff;
        effective_stiffness[i] = k_eff;

        const double delta_q_ddot = (input.tau_ext_hat[i] -
            d_eff * delta_q_dot_[i] - k_eff * delta_q_[i]) / cfg_.mass[i];
        const double unclamped_delta_q_dot = delta_q_dot_[i] + delta_q_ddot * input.dt;
        delta_q_dot_[i] = std::clamp(unclamped_delta_q_dot, min_delta_q_dot, max_delta_q_dot);
        if(delta_q_dot_[i] != unclamped_delta_q_dot) delta_q_dot_limited[i] = 1;

        const double unclamped_delta_q = delta_q_[i] + delta_q_dot_[i] * input.dt;
        delta_q_[i] = std::clamp(unclamped_delta_q, min_delta_q, max_delta_q);
        if(delta_q_[i] != unclamped_delta_q) delta_q_limited[i] = 1;

        const bool at_upper_boundary = delta_q_[i] >= max_delta_q;
        const bool at_lower_boundary = delta_q_[i] <= min_delta_q;
        if((at_upper_boundary && delta_q_dot_[i] > 0.0) ||
            (at_lower_boundary && delta_q_dot_[i] < 0.0)) {
            delta_q_dot_[i] = 0.0;
        }

        if(!std::isfinite(delta_q_[i]) || !std::isfinite(delta_q_dot_[i]) ||
            !std::isfinite(d_eff) || !std::isfinite(k_eff)) {
            return tl::make_unexpected(JointAdmittanceControllerErr::NON_FINITE_INPUT);
        }
    }

    return JointAdmittanceOutput{
        delta_q_,
        delta_q_dot_,
        std::move(delta_q_limited),
        std::move(delta_q_dot_limited),
        contact_blend_,
        std::move(effective_damping),
        std::move(effective_stiffness),
    };
}

void JointAdmittanceController::reset() {
    std::fill(delta_q_.begin(), delta_q_.end(), 0.0);
    std::fill(delta_q_dot_.begin(), delta_q_dot_.end(), 0.0);
    std::fill(contact_blend_.begin(), contact_blend_.end(), 0.0);
}

bool JointAdmittanceController::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
