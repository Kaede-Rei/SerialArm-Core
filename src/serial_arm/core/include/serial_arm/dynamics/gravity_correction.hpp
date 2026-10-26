#pragma once

#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/interaction/friction_residual_model.hpp"

namespace serial_arm {

struct GravityCorrectionPackage {
    std::string urdf_fingerprint;
    std::vector<std::string> joint_names;
    std::vector<GravityFirstMoment> first_moments;
    JointVector torque_bias;
    JointVector torque_threshold;
    bool static_pass{ false };
    bool friction_pass{ false };
    FrictionResidualModelCfg friction;
};

GravityCorrectionPackage load_gravity_correction_package(
    const std::string& path,
    const std::vector<std::string>& expected_joint_names,
    const std::string& expected_urdf_path);

tl::expected<void, DynamicsErr> configure_robot_dynamics(RobotCfg& cfg, Dynamics& dynamics);

} // namespace serial_arm
