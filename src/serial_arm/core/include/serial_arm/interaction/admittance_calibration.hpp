#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

namespace serial_arm {

/**
 * @brief 一帧无外力静态标定样本
 */
struct AdmittanceStaticSample {
    JointVector gravity;          ///< 当前姿态未缩放的模型重力广义力 Nm
    JointVector measured_torque;  ///< 当前关节反馈力矩 Nm
};

/**
 * @brief 一个静态姿态内连续采集的样本
 */
struct AdmittanceStaticPoseSamples {
    std::vector<AdmittanceStaticSample> samples;
};

/**
 * @brief 一次性多姿态静态标定配置
 */
struct AdmittanceStaticCalibrationCfg {
    std::size_t joints_count{ 0 };             ///< 关节数量
    JointVector fallback_gravity_scale;        ///< 重力变化不可观测时保留的 gravity_scale
    double gravity_observability_span{ 0.25 }; ///< 至少覆盖该重力变化范围 Nm 才拟合 gravity_scale
    double threshold_margin{ 1.2 };            ///< P99 静态残差外的安全倍率
    double threshold_max_margin{ 1.05 };       ///< 已观测静态最大残差外的安全倍率
};

/**
 * @brief 一次性多姿态静态标定结果
 */
struct AdmittanceStaticCalibrationResult {
    JointVector gravity_scale;                       ///< 每关节重力模型缩放
    JointVector torque_bias;                         ///< 每关节固定 residual 零偏 Nm
    JointVector torque_threshold;                    ///< 每关节小力矩阈值 Nm
    JointVector residual_p99;                        ///< 标定后未滤波静态 |residual| P99 Nm
    JointVector residual_max;                        ///< 标定后静态 |residual| 最大值 Nm
    std::vector<std::uint8_t> gravity_scale_observable; ///< 1 表示该轴本次数据足以拟合 gravity_scale
};

enum class AdmittanceStaticCalibrationErr {
    INVALID_CFG,
    EMPTY_SAMPLES,
    INVALID_SAMPLE_SIZE,
    NON_FINITE_SAMPLE,
};

/**
 * @brief 使用一次多姿态无外力数据联合标定 gravity_scale、torque_bias 和 torque_threshold
 *
 * 每个关节拟合关系：
 *   gravity_scale * gravity - measured_torque - torque_bias ~= 0
 *
 * 对重力变化不足的关节不强行拟合 gravity_scale，而使用 fallback_gravity_scale
 * torque_threshold 同时覆盖统计 P99 与本次标定实际观测到的静态最大残差：
 *   max(threshold_margin * P99, threshold_max_margin * max_abs_residual)
 * 这样一次静态标定优先保证“无外力不触发导纳”，而不依赖后续 filter_alpha
 */
tl::expected<AdmittanceStaticCalibrationResult, AdmittanceStaticCalibrationErr>
calibrate_admittance_static(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    const AdmittanceStaticCalibrationCfg& cfg);

} // namespace serial_arm
