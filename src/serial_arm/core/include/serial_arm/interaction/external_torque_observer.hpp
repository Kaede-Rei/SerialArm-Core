#pragma once

#include <cstddef>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 外力矩估计 source
 */
enum class ExternalTorqueSource {
    GRAVITY,     ///< 使用 filtered gravity residual
    NONLINEAR,  ///< 使用 filtered nonlinear residual
};

/**
 * @brief 外力矩 observer 错误类型
 */
enum class ExternalTorqueObserverErr {
    NOT_CONFIGURED,         ///< Observer 尚未完成配置
    ALREADY_CONFIGURED,     ///< Observer 已经配置，不能重复配置
    INVALID_CFG,            ///< Observer 配置内容无效
    INVALID_INPUT_SIZE,     ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,       ///< 输入包含 NaN 或无穷值
};

/**
 * @brief 外力矩 observer 配置
 */
struct ExternalTorqueObserverCfg {
    std::size_t joints_count{ 0 };                          ///< 受控关节数量
    ExternalTorqueSource source{ ExternalTorqueSource::GRAVITY };  ///< residual source
};

/**
 * @brief 外力矩估计结果
 */
struct ExternalTorqueEstimate {
    JointVector tau_ext_hat;    ///< 关节侧外力矩估计
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 从 TorqueResidualEstimate 中选择经过 G1 验证的 residual source
 */
class ExternalTorqueObserver {
public:
    /**
     * @brief 配置 observer
     * @param cfg observer 配置
     * @return 成功时返回空值；失败时返回 ExternalTorqueObserverErr
     */
    tl::expected<void, ExternalTorqueObserverErr> configure(const ExternalTorqueObserverCfg& cfg);
    /**
     * @brief 使用 residual 估计外力矩
     * @param residual residual observer 输出
     * @return 成功时返回外力矩估计；失败时返回 ExternalTorqueObserverErr
     */
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(const TorqueResidualEstimate& residual) const;
    /**
     * @brief 查询 observer 是否已经完成配置
     * @return 已成功配置时返回 true，否则返回 false
     */
    bool is_configured() const noexcept;

private:
    ExternalTorqueObserverCfg cfg_;     ///< observer 配置
    bool is_configured_{ false };       ///< 是否已经完成配置
};

} // namespace serial_arm
