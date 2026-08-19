#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"
#include "serial_arm/interaction/friction_residual_model.hpp"

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
    FrictionResidualModelCfg friction; ///< 速度相关摩擦 residual 模型
};

struct ExternalTorqueEstimate {
    JointVector bias_compensated;                 ///< residual_filtered - torque_bias
    JointVector friction_residual_hat;             ///< 当前摩擦模型预测的 residual 分量
    JointVector friction_compensated;              ///< bias_compensated - friction_residual_hat
    JointVector tau_ext_hat;                      ///< 平滑阈值处理后的关节侧外力矩估计
    std::vector<std::uint8_t> threshold_active;   ///< 1 表示该轴当前处于 threshold 过渡/抑制区
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 对滤波 residual 执行零偏修正与小信号阈值处理
 */
class ExternalTorqueObserver {
public:
    tl::expected<void, ExternalTorqueObserverErr> configure(const ExternalTorqueObserverCfg& cfg);
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(const TorqueResidualEstimate& residual);
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(
        const TorqueResidualEstimate& residual, const JointVector& joint_velocity);
    void reset();
    bool is_configured() const noexcept;

private:
    ExternalTorqueObserverCfg cfg_;     ///< observer 配置
    std::vector<std::int8_t> last_motion_direction_; ///< -1/0/+1；低速静摩擦补偿使用最近可靠运动方向
    bool is_configured_{ false };       ///< 是否已经完成配置
};

} // namespace serial_arm
