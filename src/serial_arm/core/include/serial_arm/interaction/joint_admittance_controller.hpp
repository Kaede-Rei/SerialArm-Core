#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

enum class JointAdmittanceControllerErr {
    NOT_CONFIGURED,
    ALREADY_CONFIGURED,
    INVALID_CFG,
    INVALID_INPUT_SIZE,
    NON_FINITE_INPUT,
    INVALID_DT,
    INVALID_DYNAMIC_LIMITS,
};

/**
 * @brief 连续 variable admittance 的少量高级参数
 *
 * mass/damping/stiffness 仍由 JointAdmittanceControllerCfg 提供：
 * - damping：接触跟随阶段的 M-D 阻尼
 * - stiffness：松手回中阶段的目标刚度
 */
struct VariableAdmittanceCfg {
    bool enabled{ false };
    double engage_time_s{ 0.03 };              ///< 接触置信度上升时间常数 s
    double release_time_s{ 0.12 };             ///< 松手后回中权重恢复时间常数 s
    JointVector soft_velocity;                  ///< 每轴 Q 弹软速度墙起点 rad/s；为空时兼容 soft_velocity_ratio
    double soft_velocity_ratio{ 0.70 };         ///< 兼容旧配置：soft_velocity 为空时按 max_delta_q_dot 比例计算
    double max_damping_multiplier{ 4.0 };       ///< 到达硬速度上限前最大阻尼倍率 >=1
};

struct JointAdmittanceControllerCfg {
    std::size_t joints_count{ 0 };
    std::vector<std::uint8_t> enabled;
    JointVector mass;
    JointVector damping;
    JointVector stiffness;
    JointVector max_delta_q;
    JointVector max_delta_q_dot;
    VariableAdmittanceCfg variable;
};

struct JointAdmittanceInput {
    JointVector tau_ext_hat;
    double dt{ 0.0 };
    JointVector min_delta_q;
    JointVector max_delta_q;
    JointVector min_delta_q_dot;
    JointVector max_delta_q_dot;
    JointVector contact_confidence;             ///< [0,1]；空向量按 0 处理
};

struct JointAdmittanceOutput {
    JointVector delta_q;
    JointVector delta_q_dot;
    std::vector<std::uint8_t> delta_q_limited;
    std::vector<std::uint8_t> delta_q_dot_limited;
    JointVector contact_blend;                  ///< 经过时间平滑的交互权重
    JointVector effective_damping;              ///< 本周期实际使用的 D
    JointVector effective_stiffness;            ///< 本周期实际使用的 K
};

struct AdmittanceDampingMetrics {
    double critical_damping{ 0.0 };
    double damping_ratio{ 0.0 };
    double natural_frequency{ 0.0 };
    double settling_time_95_critical{ 0.0 };
};

tl::expected<AdmittanceDampingMetrics, JointAdmittanceControllerErr>
compute_admittance_damping_metrics(double mass, double damping, double stiffness);

class JointAdmittanceController {
public:
    tl::expected<void, JointAdmittanceControllerErr> configure(const JointAdmittanceControllerCfg& cfg);
    tl::expected<JointAdmittanceOutput, JointAdmittanceControllerErr> update(const JointAdmittanceInput& input);
    void reset();
    bool is_configured() const noexcept;

private:
    JointAdmittanceControllerCfg cfg_;
    JointVector delta_q_;
    JointVector delta_q_dot_;
    JointVector contact_blend_;
    bool is_configured_{ false };
};

} // namespace serial_arm
