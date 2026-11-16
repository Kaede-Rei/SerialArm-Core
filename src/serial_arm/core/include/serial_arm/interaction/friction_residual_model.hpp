#pragma once

#include <algorithm>
#include <cmath>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

// Calibration and execution must use the same low-speed friction basis.
// A zero resolution preserves the continuous, unquantized model.
inline double friction_residual_blend(double velocity, double transition, double resolution = 0.0) {
    const double full_speed = std::max(transition, 4.0 * resolution);
    const double x = std::clamp((std::abs(velocity) - resolution) / (full_speed - resolution), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

/**
 * @brief 关节速度相关的 signed friction residual 模型
 *
 * 该模型直接描述摩擦在 sensorless residual 中的分量
 *   residual_friction(dq>0) = positive_coulomb + positive_viscous * |dq|
 *   residual_friction(dq<0) = negative_coulomb + negative_viscous * |dq|
 *
 * 四组系数由无外力双向回放标定得到
 * 零速附近不使用动态摩擦模型，由 torque_bias + torque_threshold 处理
 */
struct FrictionResidualModelCfg {
    bool enabled{ false };                 ///< 是否启用动态摩擦 residual 补偿
    JointVector velocity_transition;      ///< 各关节 SI 渐入速度；不低于 4 个反馈速度步进
    JointVector positive_coulomb;          ///< dq>0 时 signed Coulomb residual Nm
    JointVector positive_viscous;          ///< dq>0 时 signed viscous residual Nm/(rad/s)
    JointVector negative_coulomb;          ///< dq<0 时 signed Coulomb residual Nm
    JointVector negative_viscous;          ///< dq<0 时 signed viscous residual Nm/(rad/s)
    JointVector velocity_resolution;       ///< 内部：关节侧反馈量化步进，非 YAML 参数
};

} // namespace serial_arm
