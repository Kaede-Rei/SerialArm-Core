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
 * @brief 固定参数关节空间导纳控制器配置
 *
 * 每个关节独立执行
 *   M * delta_q_ddot + D * delta_q_dot + K * delta_q = tau_ext_hat
 */
struct JointAdmittanceControllerCfg {
    std::size_t joints_count{ 0 };
    std::vector<std::uint8_t> enabled;
    JointVector mass;                 ///< M > 0
    JointVector damping;              ///< D >= 0
    JointVector stiffness;            ///< K >= 0
    JointVector max_delta_q;          ///< 最大位置修正绝对值 rad
    JointVector max_delta_q_dot;      ///< 最大速度修正绝对值 rad/s
};

struct JointAdmittanceInput {
    JointVector tau_ext_hat;
    double dt{ 0.0 };
    JointVector min_delta_q;
    JointVector max_delta_q;
    JointVector min_delta_q_dot;
    JointVector max_delta_q_dot;
};

struct JointAdmittanceOutput {
    JointVector delta_q;
    JointVector delta_q_dot;
    std::vector<std::uint8_t> delta_q_limited;
    std::vector<std::uint8_t> delta_q_dot_limited;
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
    bool is_configured_{ false };
};

} // namespace serial_arm
