#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/dynamics/dynamics.hpp"

namespace serial_arm {

/**
 * @brief 模型校正控制周期原始帧
 */
struct ModelCalibrationFrame {
    std::uint64_t monotonic_ns{ 0 };
    std::uint64_t cycle{ 0 };
    JointVector position;
    JointVector velocity;
    JointVector torque;
    JointVector acceleration;
    JointVector reference_position;
    JointVector reference_velocity;
    std::string acceleration_source;
    std::string impedance_mode;
    double feedback_age_ms{ 0.0 };
    bool valid{ false };
    std::string phase;
    std::string direction;
    std::string speed_tier;
    std::size_t pose_group{ 0 };
    bool validation{ false };
};

/**
 * @brief 一次模型校正任务的固定元数据
 */
struct ModelCalibrationMetadata {
    std::string task_id;
    std::string profile;
    std::string core_config_path;
    std::string core_fingerprint;
    std::string urdf_path;
    std::string urdf_fingerprint;
    std::vector<std::string> resource_paths;
    std::vector<std::string> joint_names;
    std::string base_frame;
    std::string tool_frame;
    std::string locked_joint_reference{ "pinocchio_neutral" };
    std::array<double, 3> gravity{ 0.0, 0.0, -9.81 };
    JointVector original_gravity_scale;
    JointVector mapping_pos_ratio;
    JointVector mapping_tor_ratio;
    std::vector<int> mapping_direction;
    JointVector mapping_joint_zero_offset;
    JointVector mapping_actuator_zero_offset;
    std::string position_unit{ "rad" };
    std::string velocity_unit{ "rad/s" };
    std::string torque_unit{ "Nm" };
    std::string load_description;
    std::string identification_mode{ "global" };
    std::vector<std::string> locked_links;
    std::vector<std::string> identifiable_links;
    std::size_t pose_budget{ 8 };
    double validation_fraction{ 0.30 };
    double max_com_offset_m{ 0.05 };
    double regularization{ 1.0e-2 };
    double svd_relative_threshold{ 1.0e-4 };
    double minimum_information_score{ 1.0e-4 };
    double max_task_duration_s{ 600.0 };
};

/**
 * @brief 原始数据记录状态
 */
struct ModelCalibrationRecorderStatus {
    std::size_t accepted_frames{ 0 };
    std::size_t written_frames{ 0 };
    std::size_t dropped_frames{ 0 };
    bool data_gap{ false };
    bool write_failed{ false };
    std::string error;
    std::string directory;
};

/**
 * @brief 控制周期到写盘线程的有界记录器
 */
class ModelCalibrationRecorder {
public:
    explicit ModelCalibrationRecorder(std::size_t capacity = 8192);
    ~ModelCalibrationRecorder();
    ModelCalibrationRecorder(const ModelCalibrationRecorder&) = delete;
    ModelCalibrationRecorder& operator=(const ModelCalibrationRecorder&) = delete;

    tl::expected<void, std::string> start(
        const std::filesystem::path& directory,
        const ModelCalibrationMetadata& metadata,
        const std::vector<JointVector>& trajectory = {});
    void push(ModelCalibrationFrame frame);
    void stop();
    ModelCalibrationRecorderStatus status() const;
    std::vector<ModelCalibrationFrame> frames() const;
    tl::expected<void, std::string> write_trajectory(const std::vector<JointVector>& trajectory, double sample_dt);
    tl::expected<void, std::string> write_trajectory_checkpoint(const std::vector<JointVector>& trajectory, double sample_dt);

private:
    void writer_loop();
    std::string frame_line(const ModelCalibrationFrame& frame) const;
    tl::expected<void, std::string> write_metadata() const;

    std::size_t capacity_{ 8192 };
    ModelCalibrationMetadata metadata_;
    std::filesystem::path directory_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<ModelCalibrationFrame> queue_;
    std::vector<ModelCalibrationFrame> frames_;
    std::thread writer_;
    std::atomic<bool> running_{ false };
    std::atomic<bool> stop_requested_{ false };
    ModelCalibrationRecorderStatus status_;
};

/**
 * @brief 静态姿态组
 */
struct GravityCalibrationPoseGroup {
    std::size_t pose_group{ 0 };
    bool validation{ false };
    std::vector<ModelCalibrationFrame> samples;
};

/**
 * @brief 单个原始 Link 的质心约束
 */
struct GravityCalibrationLinkConstraint {
    std::string link_name;
    Eigen::Vector3d max_abs_offset{ Eigen::Vector3d::Constant(0.05) };
};

/**
 * @brief 静态重力校正配置
 */
struct GravityCalibrationOptions {
    std::size_t minimum_training_groups{ 4 };
    double regularization{ 1.0e-2 };
    double svd_relative_threshold{ 1.0e-4 };
    double minimum_observable_energy{ 0.20 };
    double default_max_com_offset_m{ 0.05 };
    double absolute_rms_target_nm{ 0.25 };
    double minimum_rms_improvement_nm{ 0.03 };
    double maximum_joint_degradation_nm{ 0.03 };
    std::vector<GravityCalibrationLinkConstraint> constraints;
    std::string identification_mode{ "global" };
    std::vector<std::string> locked_links;
};

/**
 * @brief 一组模型的逐关节误差指标
 */
struct GravityValidationMetrics {
    JointVector rms;
    JointVector p99;
    JointVector maximum;
    JointVector bias;
};

/**
 * @brief 候选静态重力校正结果
 */
struct GravityCalibrationResult {
    std::vector<GravityFirstMoment> first_moments;
    std::vector<GravityFirstMoment> com_offsets_m; // Per-link ΔCOM relative to source URDF, for human review
    JointVector torque_bias;
    std::vector<std::uint8_t> parameter_observable;
    std::vector<double> singular_values;
    std::size_t numerical_rank{ 0 };
    double regression_reconstruction_rms{ 0.0 };
    double selected_regularization{ 0.0 };
    bool prior_preferred{ false };
    std::string identification_mode{ "global" };
    std::vector<std::string> locked_links;
    std::vector<std::string> fitted_links;
    bool constraints_ok{ false };
    bool static_pass{ false };
    std::string status;
    std::string failure_reason;
    GravityValidationMetrics training_original;
    GravityValidationMetrics training_scaled;
    GravityValidationMetrics training_candidate;
    GravityValidationMetrics validation_original;
    GravityValidationMetrics validation_scaled;
    GravityValidationMetrics validation_candidate;
    JointVector noise_rms;
};

/**
 * @brief 使用按整姿态组划分的数据拟合固定质量的一阶矩校正
 */
tl::expected<GravityCalibrationResult, std::string> fit_gravity_calibration(
    Dynamics& dynamics,
    const std::vector<GravityCalibrationPoseGroup>& groups,
    const JointVector& original_gravity_scale,
    const GravityCalibrationOptions& options);

/**
 * @brief 从原始控制周期帧按 pose_group 构造静态数据组
 */
std::vector<GravityCalibrationPoseGroup> group_static_calibration_frames(
    const std::vector<ModelCalibrationFrame>& frames);

/**
 * @brief 从任务目录重载控制周期原始帧
 */
tl::expected<std::vector<ModelCalibrationFrame>, std::string> load_model_calibration_frames(
    const std::filesystem::path& directory,
    std::size_t joints_count);

/**
 * @brief 生成适合 JSON 协议的候选结果文本
 */
std::string gravity_calibration_result_json(const GravityCalibrationResult& result);

/**
 * @brief 对文件内容生成稳定 FNV-1a 指纹
 */
std::string model_calibration_file_fingerprint(const std::string& path);

} // namespace serial_arm
