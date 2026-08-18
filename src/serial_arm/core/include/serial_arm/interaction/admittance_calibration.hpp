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
    JointVector within_pose_residual_p99;            ///< 单姿态内未滤波静态噪声 |residual| P99 Nm
    JointVector within_pose_residual_max;            ///< 单姿态内未滤波静态噪声 |residual| 最大值 Nm
    JointVector lopo_residual_p99;                   ///< Leave-One-Pose-Out 跨姿态泛化 |residual| P99 Nm
    JointVector lopo_residual_max;                   ///< Leave-One-Pose-Out 跨姿态泛化 |residual| 最大值 Nm
    JointVector residual_p99;                        ///< threshold 使用的保守 P99 envelope Nm
    JointVector residual_max;                        ///< threshold 使用的保守最大 residual envelope Nm
    std::vector<std::uint8_t> gravity_scale_observable; ///< 1 表示该轴本次数据足以拟合 gravity_scale
};


/**
 * @brief 静态标定验证配置；只描述 observer 标定参数，不包含任何 M/D/K 导纳动态参数
 */
struct AdmittanceStaticValidationCfg {
    std::size_t joints_count{ 0 };      ///< 关节数量
    JointVector gravity_scale;          ///< 已标定重力缩放
    JointVector torque_bias;            ///< 已标定 residual 零偏 Nm
    JointVector torque_threshold;       ///< 已标定静态阈值 Nm
};

/**
 * @brief 静态标定验证结果
 */
struct AdmittanceStaticValidationResult {
    JointVector gravity_span;            ///< 验证姿态覆盖的未缩放模型重力范围 Nm
    JointVector residual_rms;             ///< 未滤波、bias 后静态 residual RMS Nm
    JointVector residual_p99;             ///< 未滤波、bias 后静态 |residual| P99 Nm
    JointVector residual_max;             ///< 未滤波、bias 后静态 |residual| 最大值 Nm
    JointVector feedback_quantization_step; ///< 验证样本中识别出的重复反馈量化步长 Nm；未识别时为 0
    JointVector guarded_max_limit;        ///< 允许的单帧最大 residual：threshold + 1 个量化步长 Nm
    JointVector threshold_utilization;    ///< residual_P99 / torque_threshold；越小静态裕量越大
    JointVector guarded_max_utilization;  ///< residual_max / guarded_max_limit；越小单帧量化裕量越大
    std::vector<std::uint8_t> pass;       ///< 1 表示 P99 在 threshold 内且 max 未超过一个额外量化步长
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
 */
tl::expected<AdmittanceStaticCalibrationResult, AdmittanceStaticCalibrationErr>
calibrate_admittance_static(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    const AdmittanceStaticCalibrationCfg& cfg);


/**
 * @brief 验证一次标定后的静态 residual envelope
 */
tl::expected<AdmittanceStaticValidationResult, AdmittanceStaticCalibrationErr>
evaluate_admittance_static_validation(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    const AdmittanceStaticValidationCfg& cfg);

} // namespace serial_arm
