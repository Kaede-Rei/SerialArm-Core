#pragma once

#include <tl/expected.hpp>

#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 单个执行器物理范围与协议能力
 */
struct ActuatorCapability {
    std::string actuator_name; ///< 执行器名称
    double min_pos{ 0.0 };     ///< 执行器最小位置
    double max_pos{ 0.0 };     ///< 执行器最大位置
    double max_vel{ 0.0 };     ///< 执行器最大速度
    double max_effort{ 0.0 };  ///< 执行器最大力矩
    double max_kp{ 0.0 };      ///< 执行器侧最大 kp
    double max_kd{ 0.0 };      ///< 执行器侧最大 kd
    std::string joint_name;    ///< 具名关节身份，用于命令、反馈与能力的双向对齐
    double kp_resolution{ 0.0 }; ///< 执行器协议 kp 步进，0 表示连续或未声明
    double kd_resolution{ 0.0 }; ///< 执行器协议 kd 步进，0 表示连续或未声明
    double effort_resolution{ 0.0 }; ///< 执行器协议力矩步进，0 表示连续或未声明
};

using HardwareCapabilities = std::vector<ActuatorCapability>;

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
