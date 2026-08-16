#include "serial_arm/interaction/interaction_controller.hpp"

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

    const auto observer_result = residual_observer_.configure(cfg.residual);
    if(!observer_result) {
        return tl::make_unexpected(InteractionControllerErr::INVALID_CFG);
    }

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
    return output;
}

/**
 * @brief 清除内部滤波历史
 */
void InteractionController::reset() {
    residual_observer_.reset();
}

/**
 * @brief 查询 controller 是否已经完成配置
 */
bool InteractionController::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
