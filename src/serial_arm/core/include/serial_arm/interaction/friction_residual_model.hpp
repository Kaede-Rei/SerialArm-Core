#pragma once

#include "serial_arm/core/types.hpp"

namespace serial_arm {

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
    double velocity_transition{ 0.03 };    ///< rad/s；从零速连续渐入，并在达到该速度时完整使用动态摩擦模型
    JointVector positive_coulomb;          ///< dq>0 时 signed Coulomb residual Nm
    JointVector positive_viscous;          ///< dq>0 时 signed viscous residual Nm/(rad/s)
    JointVector negative_coulomb;          ///< dq<0 时 signed Coulomb residual Nm
    JointVector negative_viscous;          ///< dq<0 时 signed viscous residual Nm/(rad/s)
};

} // namespace serial_arm
