#pragma once

#include <cstddef>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节导纳 controller 错误类型
 */
enum class JointAdmittanceControllerErr {
    NOT_CONFIGURED,         ///< Controller 尚未完成配置
    ALREADY_CONFIGURED,     ///< Controller 已经配置，不能重复配置
    INVALID_CFG,            ///< Controller 配置内容无效
    INVALID_INPUT_SIZE,     ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,       ///< 输入包含 NaN 或无穷值
    INVALID_DT,             ///< dt 非有限或不大于 0
    INVALID_DYNAMIC_LIMITS, ///< 单周期动态位置/速度限幅无效
};

/**
 * @brief 关节导纳 controller 配置
 */
struct JointAdmittanceControllerCfg {
    std::size_t joints_count{ 0 };      ///< 受控关节数量
    std::vector<std::uint8_t> enabled;  ///< 每个关节是否启用导纳
    JointVector mass;                   ///< 虚拟质量
    JointVector damping;                ///< 虚拟阻尼
    JointVector stiffness;              ///< 虚拟刚度
    JointVector max_delta_q;            ///< 位置偏移限幅
    JointVector max_delta_q_dot;        ///< 速度偏移限幅
};

/**
 * @brief 关节导纳单周期输入
 */
struct JointAdmittanceInput {
    JointVector tau_ext_hat;       ///< 关节侧外力矩估计
    double dt{ 0.0 };              ///< 控制周期
    JointVector min_delta_q;       ///< 单周期允许的最小位置修正；空向量表示仅使用配置限幅
    JointVector max_delta_q;       ///< 单周期允许的最大位置修正；空向量表示仅使用配置限幅
    JointVector min_delta_q_dot;   ///< 单周期允许的最小速度修正；空向量表示仅使用配置限幅
    JointVector max_delta_q_dot;   ///< 单周期允许的最大速度修正；空向量表示仅使用配置限幅
};

/**
 * @brief 关节导纳单周期输出
 */
struct JointAdmittanceOutput {
    JointVector delta_q;                              ///< 位置偏移
    JointVector delta_q_dot;                          ///< 速度偏移
    std::vector<std::uint8_t> delta_q_limited;        ///< 1 表示本周期位置偏移触及限幅
    std::vector<std::uint8_t> delta_q_dot_limited;    ///< 1 表示本周期速度偏移触及限幅
};

/**
 * @brief 单轴导纳二阶系统阻尼指标
 */
struct AdmittanceDampingMetrics {
    double critical_damping{ 0.0 };  ///< 临界阻尼 Dcrit = 2 * sqrt(mass * stiffness)
    double damping_ratio{ 0.0 };     ///< 阻尼比 zeta = damping / Dcrit
};

/**
 * @brief 根据 mass / damping / stiffness 计算临界阻尼与阻尼比
 * @return 参数有限且 mass、stiffness 大于 0、damping 不小于 0 时返回指标
 */
tl::expected<AdmittanceDampingMetrics, JointAdmittanceControllerErr>
compute_admittance_damping_metrics(double mass, double damping, double stiffness);

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 关节空间导纳 controller
 */
class JointAdmittanceController {
public:
    /**
     * @brief 配置 controller
     * @param cfg controller 配置
     * @return 成功时返回空值；失败时返回 JointAdmittanceControllerErr
     */
    tl::expected<void, JointAdmittanceControllerErr> configure(const JointAdmittanceControllerCfg& cfg);
    /**
     * @brief 更新导纳状态
     * @param input 单周期输入
     * @return 成功时返回导纳输出；失败时返回 JointAdmittanceControllerErr
     */
    tl::expected<JointAdmittanceOutput, JointAdmittanceControllerErr> update(const JointAdmittanceInput& input);
    /**
     * @brief 清除导纳状态
     */
    void reset();
    /**
     * @brief 查询 controller 是否已经完成配置
     * @return 已成功配置时返回 true，否则返回 false
     */
    bool is_configured() const noexcept;

private:
    JointAdmittanceControllerCfg cfg_;   ///< controller 配置
    JointVector delta_q_;                ///< 位置偏移状态
    JointVector delta_q_dot_;            ///< 速度偏移状态
    bool is_configured_{ false };        ///< 是否已经完成配置
};

} // namespace serial_arm
