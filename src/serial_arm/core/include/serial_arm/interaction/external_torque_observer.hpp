#pragma once

#include <cstddef>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class ExternalTorqueObserverErr {
    NOT_CONFIGURED,         ///< Observer 尚未完成配置
    ALREADY_CONFIGURED,     ///< Observer 已经配置，不能重复配置
    INVALID_CFG,            ///< Observer 配置内容无效
    INVALID_INPUT_SIZE,     ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,       ///< 输入包含 NaN 或无穷值
};

/**
 * @brief 外力矩估计配置
 */
struct ExternalTorqueObserverCfg {
    std::size_t joints_count{ 0 };  ///< 受控关节数量
    JointVector torque_bias;        ///< 每关节固定零偏；空向量等价于全 0
    JointVector torque_threshold;   ///< 小力矩忽略阈值；空向量等价于全 0
};

struct ExternalTorqueEstimate {
    JointVector tau_ext_hat;    ///< 关节侧外力矩估计
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 对滤波 residual 执行零偏修正与小信号阈值处理
 */
class ExternalTorqueObserver {
public:
    tl::expected<void, ExternalTorqueObserverErr> configure(const ExternalTorqueObserverCfg& cfg);
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(const TorqueResidualEstimate& residual) const;
    bool is_configured() const noexcept;

private:
    ExternalTorqueObserverCfg cfg_;     ///< observer 配置
    bool is_configured_{ false };       ///< 是否已经完成配置
};

} // namespace serial_arm
