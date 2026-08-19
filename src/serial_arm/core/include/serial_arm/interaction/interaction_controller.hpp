#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class InteractionControllerErr {
    NOT_CONFIGURED,     ///< Controller 尚未完成配置
    ALREADY_CONFIGURED, ///< Controller 已经配置，不能重复配置
    INVALID_CFG,        ///< Controller 配置内容无效
    OBSERVER_FAILED,    ///< residual observer 更新失败
    EXTERNAL_FAILED,    ///< external torque observer 更新失败
    ADMITTANCE_FAILED,  ///< admittance controller 更新失败
};

struct InteractionControllerCfg {
    bool enabled{ false };                          ///< 导纳能力是否启用
    TorqueResidualObserverCfg residual;             ///< residual observer 配置
    ExternalTorqueObserverCfg external_torque;      ///< external torque observer 配置
    JointAdmittanceControllerCfg admittance;        ///< joint admittance controller 配置
};

struct InteractionInput {
    const JointVector& measured_torque;  ///< 当前关节反馈力矩
    const JointVector& model_torque;     ///< 实际 q/dq/qdd 对应的完整内部动力学模型力矩
    const JointCtrlCmd& nominal_cmd;     ///< 名义关节控制命令
    double dt{ 0.0 };                    ///< 控制周期
    JointVector min_delta_q;             ///< Safety 剩余空间给出的最小位置修正
    JointVector max_delta_q;             ///< Safety 剩余空间给出的最大位置修正
    JointVector min_delta_q_dot;         ///< Safety 剩余空间给出的最小速度修正
    JointVector max_delta_q_dot;         ///< Safety 剩余空间给出的最大速度修正
    JointVector measured_velocity;       ///< 当前实测关节速度；摩擦 residual 补偿使用
};

struct InteractionOutput {
    JointCtrlCmd corrected_cmd;                       ///< 修正后的关节控制命令
    TorqueResidualEstimate residual;                  ///< residual 观测结果
    JointVector bias_compensated;                     ///< residual_filtered - torque_bias
    JointVector friction_residual_hat;                 ///< 摩擦模型预测的 residual 分量
    JointVector friction_compensated;                  ///< bias 后再减去摩擦 residual
    JointVector tau_ext_hat;                          ///< 阈值处理后的关节侧外力矩估计
    std::vector<std::uint8_t> threshold_active;       ///< 1 表示当前处于 threshold 抑制/过渡区
    JointVector delta_q;                              ///< 导纳位置偏移
    JointVector delta_q_dot;                          ///< 导纳速度偏移
    std::vector<std::uint8_t> delta_q_limited;        ///< 1 表示位置偏移触及限幅
    std::vector<std::uint8_t> delta_q_dot_limited;    ///< 1 表示速度偏移触及限幅
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class InteractionController {
public:
    tl::expected<void, InteractionControllerErr> configure(const InteractionControllerCfg& cfg);
    tl::expected<InteractionOutput, InteractionControllerErr> update(const InteractionInput& input);
    void reset();
    bool is_configured() const noexcept;
    bool is_enabled() const noexcept;

private:
    TorqueResidualObserver residual_observer_;           ///< residual observer
    ExternalTorqueObserver external_torque_observer_;    ///< external torque observer
    JointAdmittanceController admittance_controller_;    ///< joint admittance controller
    InteractionControllerCfg cfg_;                       ///< controller 配置
    bool is_configured_{ false };                        ///< 是否已经完成配置
};

} // namespace serial_arm
