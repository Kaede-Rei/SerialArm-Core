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
    double velocity_transition{ 0.03 };    ///< rad/s；高于该速度认为方向可靠
    double zero_velocity_adaptation_s{ 0.60 }; ///< s；零速静摩擦 baseline 在无新外力时的慢速自适应时间常数
    double kinetic_feedforward_scale{ 0.0 }; ///< [0,0.7]；执行器滑动摩擦助力比例，默认关闭
    JointVector positive_coulomb;          ///< dq>0 时 v->0+ 的 signed residual Nm
    JointVector positive_viscous;          ///< dq>0 时相对 |dq| 的 signed residual 斜率 Nm/(rad/s)
    JointVector negative_coulomb;          ///< dq<0 时 v->0- 的 signed residual Nm
    JointVector negative_viscous;          ///< dq<0 时相对 |dq| 的 signed residual 斜率 Nm/(rad/s)
};

} // namespace serial_arm
