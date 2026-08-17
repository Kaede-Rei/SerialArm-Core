#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief InteractionController 错误类型
 */
enum class InteractionControllerErr {
    NOT_CONFIGURED,     ///< Controller 尚未完成配置
    ALREADY_CONFIGURED, ///< Controller 已经配置，不能重复配置
    INVALID_CFG,        ///< Controller 配置内容无效
    OBSERVER_FAILED,    ///< residual observer 更新失败
    EXTERNAL_FAILED,    ///< external torque observer 更新失败
    ADMITTANCE_FAILED,  ///< admittance controller 更新失败
};

/**
 * @brief InteractionController 配置
 */
struct InteractionControllerCfg {
    TorqueResidualObserverCfg residual;          ///< residual observer 配置
    bool admittance_enabled{ false };            ///< 是否启用导纳路径
    ExternalTorqueObserverCfg external_torque;   ///< external torque observer 配置
    JointAdmittanceControllerCfg admittance;     ///< joint admittance controller 配置
};

/**
 * @brief InteractionController 单周期输入
 */
struct InteractionInput {
    const DynamicsState& dynamics;       ///< 当前动力学状态
    const JointCtrlCmd& nominal_cmd;     ///< 名义关节控制命令
    double dt{ 0.0 };                    ///< 控制周期
};

/**
 * @brief InteractionController 单周期输出
 */
struct InteractionOutput {
    JointCtrlCmd corrected_cmd;          ///< 修正后的关节控制命令
    TorqueResidualEstimate residual;     ///< residual 观测结果
    JointVector tau_ext_hat;             ///< 关节侧外力矩估计
    JointVector delta_q;                 ///< 导纳位置偏移
    JointVector delta_q_dot;             ///< 导纳速度偏移
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief P1 阶段 interaction 控制器
 */
class InteractionController {
public:
    /**
     * @brief 配置 controller
     * @param cfg controller 配置
     * @return 成功时返回空值；失败时返回 InteractionControllerErr
     */
    tl::expected<void, InteractionControllerErr> configure(const InteractionControllerCfg& cfg);
    /**
     * @brief 计算 interaction 输出
     * @param input 单周期输入
     * @return 成功时返回 interaction 输出；失败时返回 InteractionControllerErr
     */
    tl::expected<InteractionOutput, InteractionControllerErr> update(const InteractionInput& input);
    /**
     * @brief 清除内部滤波历史
     */
    void reset();

    /**
     * @brief 查询 controller 是否已经完成配置
     * @return 已成功配置时返回 true，否则返回 false
     */
    bool is_configured() const noexcept;

private:
    TorqueResidualObserver residual_observer_;          ///< residual observer
    ExternalTorqueObserver external_torque_observer_;    ///< external torque observer
    JointAdmittanceController admittance_controller_;    ///< joint admittance controller
    InteractionControllerCfg cfg_;                       ///< controller 配置
    bool is_configured_{ false };                        ///< 是否已经完成配置
};

} // namespace serial_arm
