#pragma once

#include <cstddef>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

enum class GeneralizedMomentumObserverErr {
    NOT_CONFIGURED,
    ALREADY_CONFIGURED,
    INVALID_CFG,
    INVALID_INPUT_SIZE,
    NON_FINITE_INPUT,
    INVALID_DT,
};

struct GeneralizedMomentumObserverCfg {
    std::size_t joints_count{ 0 };
    JointVector gain; ///< rad/s；observer 一阶收敛带宽
    JointVector initial_residual; ///< 可选初始 residual；用于用已标定 torque_bias 无冲击启动
};

struct GeneralizedMomentumInput {
    JointVector measured_torque;             ///< 执行器反馈关节力矩 Nm
    JointVector gravity;                     ///< 校准后的重力广义力 Nm
    JointVector coriolis;                    ///< C(q,dq)*dq Nm
    std::vector<JointVector> mass_matrix;    ///< M(q)，N×N
    JointVector velocity;                    ///< dq rad/s
    double dt{ 0.0 };
};

struct GeneralizedMomentumOutput {
    JointVector tau_ext_hat;                 ///< 广义外力矩估计 Nm
    JointVector momentum_error;              ///< p - p_hat
};

/**
 * @brief 不显式使用 qdd 的 generalized momentum observer
 *
 * 使用 p=M(q)dq 与
 *   p_dot = tau + tau_ext - g + C(q,dq)^T dq
 * 构造积分 observerC^T dq 通过 Mdot*dq - C*dq 计算，Mdot 使用相邻质量矩阵差分；
 * 该差分只依赖 q 的缓变模型矩阵，不对实测 dq 做二次数值微分
 */
class GeneralizedMomentumObserver {
public:
    tl::expected<void, GeneralizedMomentumObserverErr> configure(
        const GeneralizedMomentumObserverCfg& cfg);
    tl::expected<GeneralizedMomentumOutput, GeneralizedMomentumObserverErr> update(
        const GeneralizedMomentumInput& input);
    void reset();
    bool is_configured() const noexcept;

private:
    GeneralizedMomentumObserverCfg cfg_;
    JointVector predicted_momentum_;
    JointVector residual_;
    std::vector<JointVector> previous_mass_matrix_;
    bool initialized_{ false };
    bool is_configured_{ false };
};

} // namespace serial_arm
