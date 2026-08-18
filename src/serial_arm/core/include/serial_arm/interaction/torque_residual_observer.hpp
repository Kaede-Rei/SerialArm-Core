#pragma once

#include <cstddef>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节力矩 residual observer 错误类型
 */
enum class TorqueResidualObserverErr {
    NOT_CONFIGURED,         ///< Observer 尚未完成配置
    ALREADY_CONFIGURED,     ///< Observer 已经配置，不能重复配置
    INVALID_CFG,            ///< Observer 配置内容无效
    INVALID_INPUT_SIZE,     ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,       ///< 输入包含 NaN 或无穷值
};

/**
 * @brief 关节力矩 residual observer 配置
 */
struct TorqueResidualObserverCfg {
    std::size_t joints_count{ 0 };  ///< 受控关节数量
    double filter_alpha{ 0.1 };     ///< 一阶低通滤波系数
};

/**
 * @brief 关节力矩 residual 估计结果
 */
struct TorqueResidualEstimate {
    JointVector residual;           ///< model_torque - measured_torque
    JointVector residual_filtered;  ///< 滤波后的 residual
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 基于当前模式对应的内部动力学模型力矩与反馈力矩计算关节 residual
 */
class TorqueResidualObserver {
public:
    tl::expected<void, TorqueResidualObserverErr> configure(const TorqueResidualObserverCfg& cfg);
    tl::expected<TorqueResidualEstimate, TorqueResidualObserverErr> update(
        const JointVector& measured_torque,
        const JointVector& model_torque);
    void reset();
    bool is_configured() const noexcept;

private:
    TorqueResidualObserverCfg cfg_;  ///< observer 配置
    JointVector filtered_;           ///< residual 滤波历史
    bool is_configured_{ false };    ///< 是否已经完成配置
    bool has_sample_{ false };       ///< 是否已经接收过有效 sample
};

} // namespace serial_arm
