#pragma once

#include "serial_arm/core/types.hpp"

namespace serial_arm {

/**
 * @brief 由关节内部摩擦在 sensorless residual 中产生的速度相关模型
 *
 * 这里直接拟合 residual 中的摩擦分量，而不是强行假定电机/关节力矩正负号：
 *   residual_friction(v>0) = positive_coulomb + positive_viscous * |v|
 *   residual_friction(v<0) = negative_coulomb + negative_viscous * |v|
 *
 * 四个系数均允许带符号，由无外力双向回放自动辨识
 */
struct FrictionResidualModelCfg {
    bool enabled{ false };                 ///< 是否启用摩擦 residual 补偿
    double velocity_transition{ 0.03 };    ///< rad/s；低于该速度时保留最近可靠运动方向处理静摩擦
    JointVector positive_coulomb;          ///< dq>0 时 v->0+ 的 signed residual Nm
    JointVector positive_viscous;          ///< dq>0 时相对 |dq| 的 signed residual 斜率 Nm/(rad/s)
    JointVector negative_coulomb;          ///< dq<0 时 v->0- 的 signed residual Nm
    JointVector negative_viscous;          ///< dq<0 时相对 |dq| 的 signed residual 斜率 Nm/(rad/s)
};

} // namespace serial_arm
