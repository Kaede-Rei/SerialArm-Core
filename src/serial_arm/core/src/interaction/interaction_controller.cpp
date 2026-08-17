#include "serial_arm/interaction/interaction_controller.hpp"

#include <utility>

namespace serial_arm {

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 controller
 * @param cfg controller 配置
 * @return 成功时返回空值；失败时返回 InteractionControllerErr
 */
tl::expected<void, InteractionControllerErr> InteractionController::configure(const InteractionControllerCfg& cfg) {
    if(is_configured_) {
        return tl::make_unexpected(InteractionControllerErr::ALREADY_CONFIGURED);
    }

    TorqueResidualObserver residual_observer;
    ExternalTorqueObserver external_torque_observer;
    JointAdmittanceController admittance_controller;

    const auto observer_result = residual_observer.configure(cfg.residual);
    if(!observer_result) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }

    if(cfg.admittance_enabled) {
        const auto external_result = external_torque_observer.configure(cfg.external_torque);
        const auto admittance_result = admittance_controller.configure(cfg.admittance);
        if(!external_result || !admittance_result) {
            return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
        }
    }

    cfg_ = cfg;
    residual_observer_ = std::move(residual_observer);
    external_torque_observer_ = std::move(external_torque_observer);
    admittance_controller_ = std::move(admittance_controller);
    is_configured_ = true;
    return {};
}

/**
 * @brief 计算 interaction 输出
 * @param input 单周期输入
 * @return 成功时返回 interaction 输出；失败时返回 InteractionControllerErr
 */
tl::expected<InteractionOutput, InteractionControllerErr> InteractionController::update(const InteractionInput& input) {
    if(!is_configured_) {
        return tl::make_unexpected(InteractionControllerErr::NOT_CONFIGURED);
    }

    auto residual = residual_observer_.update(input.dynamics);
    if(!residual) {
        return tl::make_unexpected(InteractionControllerErr::OBSERVER_FAILED);
    }

    InteractionOutput output;
    output.corrected_cmd = input.nominal_cmd;
    output.residual = std::move(residual.value());
    output.tau_ext_hat.assign(output.residual.gravity_residual_filtered.size(), 0.0);
    output.delta_q.assign(output.residual.gravity_residual_filtered.size(), 0.0);
    output.delta_q_dot.assign(output.residual.gravity_residual_filtered.size(), 0.0);

    if(!cfg_.admittance_enabled) {
        return output;
    }

    auto tau_ext = external_torque_observer_.update(output.residual);
    if(!tau_ext) {
        return tl::make_unexpected(InteractionControllerErr::EXTERNAL_FAILED);
    }

    auto admittance = admittance_controller_.update(JointAdmittanceInput{ tau_ext->tau_ext_hat, input.dt });
    if(!admittance) {
        return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    }

    if(output.corrected_cmd.pos.size() != admittance->delta_q.size() ||
        output.corrected_cmd.vel.size() != admittance->delta_q_dot.size()) {
        return tl::make_unexpected(InteractionControllerErr::ADMITTANCE_FAILED);
    }

    output.tau_ext_hat = std::move(tau_ext->tau_ext_hat);
    output.delta_q = std::move(admittance->delta_q);
    output.delta_q_dot = std::move(admittance->delta_q_dot);

    for(std::size_t i = 0; i < output.delta_q.size(); ++i) {
        output.corrected_cmd.pos[i] += output.delta_q[i];
        output.corrected_cmd.vel[i] += output.delta_q_dot[i];
    }
    return output;
}

/**
 * @brief 清除内部滤波历史
 */
void InteractionController::reset() {
    residual_observer_.reset();
    admittance_controller_.reset();
}

/**
 * @brief 查询 controller 是否已经完成配置
 */
bool InteractionController::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
