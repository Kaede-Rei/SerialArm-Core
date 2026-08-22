#include "serial_arm/config/limit_resolver.hpp"
#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/model/model_loader.hpp"
#include "serial_arm/robot.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using namespace serial_arm;
namespace fs = std::filesystem;

std::string fixture_path(const std::string& name) {
    return std::string(SERIAL_ARM_TEST_FIXTURE_DIR) + "/" + name;
}

void write_text(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    stream << text;
}

void write_test_package(const fs::path& root, const std::string& package) {
    write_text(root / package / "package.xml", "<package><name>" + package + "</name></package>\n");
}

std::string test_profile_yaml(const std::string& profile_name, const std::string& plugin, const std::string& core_package, const std::string& hardware_package) {
    return "profiles:\n  " + profile_name + ":\n    core:\n      package: " + core_package + "\n      config: config/core.yaml\n    hardware:\n      plugin: " + plugin + "\n      config_package: " + hardware_package + "\n      config: config/hardware.yaml\n";
}

JointState joint_state(std::size_t n) {
    return JointState{
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 0.0),
    };
}

ActuatorState actuator_state(std::size_t n) {
    return ActuatorState{
        ActuatorVector(n, 0.0),
        ActuatorVector(n, 0.0),
        ActuatorVector(n, 0.0),
        std::vector<std::uint8_t>(n, 1),
        std::vector<std::uint8_t>(n, 1),
        std::vector<int>(n, 0),
    };
}

JointCtrlCmd joint_cmd(std::size_t n) {
    return JointCtrlCmd{
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 1.0),
        JointVector(n, 0.1),
    };
}

DynamicsCfg dynamics_cfg(const std::vector<std::string>& names) {
    DynamicsCfg cfg;
    cfg.urdf_path = fixture_path("simple_4dof_revolute_arm.urdf");
    cfg.joint_names = names;
    cfg.base_frame = "base_link";
    cfg.tool_frame = "tool0";
    cfg.gravity_scale.assign(names.size(), 1.0);
    return cfg;
}

SafetyCfg safety_cfg(bool continuous) {
    SafetyCfg cfg;
    cfg.joints_count = 1;
    cfg.limits.has_position_limit = { static_cast<std::uint8_t>(continuous ? 0 : 1) };
    cfg.limits.min_pos = { -1.0 };
    cfg.limits.max_pos = { 1.0 };
    cfg.limits.max_vel = { 2.0 };
    cfg.limits.max_acc = { 100.0 };
    cfg.limits.max_effort = { 5.0 };
    cfg.limits.max_kp = { 20.0 };
    cfg.limits.max_kd = { 2.0 };
    cfg.limits.pos_margin = { 0.1 };
    cfg.fault_recovery.compliant_recovery.kp = { 1.0 };
    cfg.fault_recovery.compliant_recovery.kd = { 0.1 };
    cfg.fault_recovery.compliant_recovery.max_vel = { 1.0 };
    return cfg;
}

RobotCfg robot_cfg_for_validation(bool continuous) {
    RobotCfg cfg;
    cfg.joint_names = { "joint1" };
    cfg.runtime.ctrl_frequency_hz = 100.0;
    cfg.runtime.joint_acc_filter_alpha = 0.2;
    cfg.shutdown.park_before_disable = true;
    cfg.shutdown.park_pos = { continuous ? 100.0 : 0.0 };
    cfg.shutdown.speed_scale = 0.2;
    cfg.shutdown.position_tolerance = 0.01;
    cfg.shutdown.velocity_tolerance = 0.01;
    cfg.shutdown.settle_time_s = 0.0;
    cfg.shutdown.relaxed_tolerance_ratio = 2.0;
    cfg.shutdown.timeout_s = 1.0;
    cfg.ctrller.joints_count = 1;
    cfg.ctrller.allow_full_cmd = true;
    cfg.ctrller.rigid_hold_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.rigid_tracking_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.compliant_hold_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.compliant_drag_gains = { { 0.0 }, { 0.1 } };
    cfg.ctrller.compliant_tracking_gains = { { 1.0 }, { 0.1 } };
    cfg.mapper.joints_count = 1;
    cfg.mapper.pos_ratio = { 1.0 };
    cfg.mapper.tor_ratio = { 1.0 };
    cfg.mapper.direction = { 1 };
    cfg.mapper.joint_zero_offset = { 0.0 };
    cfg.mapper.actuator_zero_offset = { 0.0 };
    cfg.safety = safety_cfg(continuous);
    cfg.safety.max_dt_s = 0.02;
    cfg.dynamics.urdf_path = fixture_path("simple_4dof_revolute_arm.urdf");
    cfg.dynamics.joint_names = { "joint1" };
    cfg.dynamics.base_frame = "base_link";
    cfg.dynamics.tool_frame = "tool0";
    cfg.dynamics.gravity_scale = { 1.0 };
    return cfg;
}

void configure_test_admittance(
    AdmittanceCapabilityCfg& admittance,
    double mass = 1.0,
    double damping = 1.0,
    double stiffness = 1.0,
    double max_delta_q = 1.0,
    double max_delta_q_dot = 1.0,
    double torque_threshold = 0.0) {
    constexpr double kMinDynamicTerm = 1.0e-6;
    const double d = std::max(damping, kMinDynamicTerm);
    const double k = std::max(stiffness, kMinDynamicTerm);

    admittance.enabled = true;
    admittance.joint_enabled = { 1 };
    admittance.observer.mode = AdmittanceObserverMode::FULL_ID;
    admittance.observer.momentum_gain = { 25.0 };
    admittance.calibration.torque_bias = { 0.0 };
    admittance.calibration.torque_threshold = { torque_threshold };
    admittance.calibration.friction.enabled = false;
    admittance.calibration.friction.velocity_transition = 0.03;
    admittance.calibration.friction.positive_coulomb = { 0.0 };
    admittance.calibration.friction.positive_viscous = { 0.0 };
    admittance.calibration.friction.negative_coulomb = { 0.0 };
    admittance.calibration.friction.negative_viscous = { 0.0 };
    admittance.controller.mass = { mass };
    admittance.controller.damping = { d };
    admittance.controller.stiffness = { k };
    admittance.controller.max_delta_q = { max_delta_q };
    admittance.controller.max_delta_q_dot = { max_delta_q_dot };
}

class FakeMotorBus final : public MotorBus {
public:
    tl::expected<void, MotorBusErr> configure(const std::string&) override { return {}; }
    tl::expected<void, MotorBusErr> connect() override { return {}; }
    tl::expected<ActuatorState, MotorBusErr> read() override { return state; }
    tl::expected<void, MotorBusErr> activate() override { return {}; }
    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        last_cmd = cmd;
        return {};
    }
    tl::expected<void, MotorBusErr> stop() override { return {}; }
    tl::expected<void, MotorBusErr> deactivate() override { return {}; }
    tl::expected<void, MotorBusErr> recover() override { return {}; }
    const HardwareCapabilities& capabilities() const noexcept override { return caps_; }
    void cleanup() noexcept override {}
    std::size_t size() const noexcept override { return caps_.size(); }

    HardwareCapabilities caps_{ { "actuator1", -10.0, 10.0, 10.0, 10.0, 100.0, 10.0 } };
    ActuatorState state{ actuator_state(1) };
    ActuatorCtrlCmd last_cmd;
};

} // namespace

TEST(ContinuousJointSafety, PositionLimitsAreSkippedForContinuousJoints) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    state.pos[0] = 10.0;
    EXPECT_TRUE(safety.check_state(state, actuator_state(1), 0.0));

    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = -10.0;
    cmd.kp[0] = 0.0;
    EXPECT_TRUE(safety.check_joint_cmd(state, cmd, 0.001));
}

TEST(ContinuousJointSafety, VelocityEffortAndFiniteChecksStillApply) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    state.vel[0] = 4.0;
    auto state_result = safety.check_state(state, actuator_state(1), 0.0);
    ASSERT_FALSE(state_result);
    EXPECT_EQ(state_result.error().code, SafetyErr::JOINT_VEL_LIMIT);

    state = joint_state(1);
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.tor[0] = 6.0;
    auto effort_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(effort_result);
    EXPECT_EQ(effort_result.error().code, SafetyErr::CMD_EFFORT_LIMIT);

    cmd = joint_cmd(1);
    cmd.pos[0] = std::numeric_limits<double>::infinity();
    auto finite_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(finite_result);
    EXPECT_EQ(finite_result.error().code, SafetyErr::NON_FINITE_CMD);
}

TEST(ContinuousJointSafety, GainsAndTimeoutsStillApply) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.kp[0] = 21.0;
    auto kp_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(kp_result);
    EXPECT_EQ(kp_result.error().code, SafetyErr::CMD_KP_LIMIT);

    cmd = joint_cmd(1);
    cmd.kd[0] = 3.0;
    auto kd_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(kd_result);
    EXPECT_EQ(kd_result.error().code, SafetyErr::CMD_KD_LIMIT);

    auto state_timeout = safety.check_state(state, actuator_state(1), 1.0);
    ASSERT_FALSE(state_timeout);
    EXPECT_EQ(state_timeout.error().code, SafetyErr::STATE_TIMEOUT);

    auto cmd_timeout = safety.check_cmd_age(1.0);
    ASSERT_FALSE(cmd_timeout);
    EXPECT_EQ(cmd_timeout.error().code, SafetyErr::CMD_TIMEOUT);
}

TEST(ContinuousJointSafety, ParkPositionValidationRespectsPositionLimitFlag) {
    RobotCfg continuous = robot_cfg_for_validation(true);
    EXPECT_TRUE(validate_robot_core_cfg(continuous));

    RobotCfg revolute = robot_cfg_for_validation(false);
    revolute.shutdown.park_pos = { 100.0 };
    auto result = validate_robot_core_cfg(revolute);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ConfigErr::INVALID_VALUE);
}

TEST(ContinuousJointSafety, RevolutePositionLimitsRemainActive) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(false)));

    JointState state = joint_state(1);
    state.pos[0] = 2.0;
    auto state_result = safety.check_state(state, actuator_state(1), 0.0);
    ASSERT_FALSE(state_result);
    EXPECT_EQ(state_result.error().code, SafetyErr::JOINT_POS_LIMIT);

    state.pos[0] = 0.0;
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = 2.0;
    auto cmd_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(cmd_result);
    EXPECT_EQ(cmd_result.error().code, SafetyErr::CMD_POS_LIMIT);
}

TEST(ContinuousJointSafety, CommandStepUsesExistingPositionRepresentation) {
    Safety safety;
    SafetyCfg cfg = safety_cfg(true);
    cfg.limits.max_acc[0] = 1000.0;
    ASSERT_TRUE(safety.configure(cfg));

    JointState state = joint_state(1);
    state.pos[0] = M_PI - 0.001;
    ASSERT_TRUE(safety.reset_cmd_history(state));

    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = -M_PI + 0.001;
    auto result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, SafetyErr::CMD_POS_STEP_LIMIT);
}

TEST(FaultRecoverySafety, PositionRecoverySkipsOnlyPositionRelatedChecks) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(false)));

    JointState state = joint_state(1);
    state.pos[0] = 1.2;
    ActuatorState actuators = actuator_state(1);

    auto normal_state = safety.check_state(state, actuators, 0.0);
    ASSERT_FALSE(normal_state);
    EXPECT_EQ(normal_state.error().code, SafetyErr::JOINT_POS_LIMIT);
    EXPECT_TRUE(safety.check_state_for_position_recovery(state, actuators, 0.0));

    state.vel[0] = 4.0;
    auto fast_state = safety.check_state_for_position_recovery(state, actuators, 0.0);
    ASSERT_FALSE(fast_state);
    EXPECT_EQ(fast_state.error().code, SafetyErr::JOINT_VEL_LIMIT);
    state.vel[0] = 0.0;

    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = 1.2;
    auto normal_cmd = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(normal_cmd);
    EXPECT_EQ(normal_cmd.error().code, SafetyErr::CMD_POS_LIMIT);
    EXPECT_TRUE(safety.check_joint_cmd_for_position_recovery(state, cmd, 0.001));

    cmd.tor[0] = 6.0;
    auto effort_cmd = safety.check_joint_cmd_for_position_recovery(state, cmd, 0.001);
    ASSERT_FALSE(effort_cmd);
    EXPECT_EQ(effort_cmd.error().code, SafetyErr::CMD_EFFORT_LIMIT);
}

TEST(RobotFaultRecovery, ActivatePositionLimitFaultCanUseGravityCompensatedCompliantDragRecovery) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    cfg.safety.fault_recovery.compliant_recovery.effort_scale = 1.0;

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.pos[0] = 1.2;

    ModelFeedforwardFn gravity = [](
        ModelFeedforwardMode mode, const JointState& state,
        const JointVector&, const JointVector&, double) {
            return tl::expected<JointVector, ModelFeedforwardErr>(
                JointVector(state.pos.size(), mode == ModelFeedforwardMode::GRAVITY ? 0.7 : 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    auto activated = robot.activate();
    ASSERT_FALSE(activated);
    ASSERT_EQ(activated.error().code, RobotErr::SAFETY_FAILED);
    ASSERT_EQ(activated.error().safety_fault.code, SafetyErr::JOINT_POS_LIMIT);
    EXPECT_EQ(robot.get_state(), RobotState::FAULT);

    ASSERT_TRUE(robot.enter_fault_compliant_recovery());
    EXPECT_EQ(robot.get_fault_hold_mode(), FaultHoldMode::COMPLIANT_RECOVERY);
    ASSERT_FALSE(bus_raw->last_cmd.tor.empty());
    EXPECT_NEAR(bus_raw->last_cmd.tor[0], 0.7, 1e-12);

    bus_raw->state.pos[0] = 1.1;
    ASSERT_TRUE(robot.maintain_fault_hold());
    ASSERT_FALSE(bus_raw->last_cmd.pos.empty());
    EXPECT_NEAR(bus_raw->last_cmd.pos[0], 1.1, 1e-12);
    EXPECT_NEAR(bus_raw->last_cmd.tor[0], 0.7, 1e-12);

    ASSERT_TRUE(robot.return_to_fault_rigid_hold());
    EXPECT_EQ(robot.get_fault_hold_mode(), FaultHoldMode::RIGID_HOLD);
    ASSERT_TRUE(robot.enter_fault_compliant_recovery());

    bus_raw->state.pos[0] = 0.8;
    ASSERT_TRUE(robot.maintain_fault_hold());
    ASSERT_TRUE(robot.maintain_fault_hold());
    ASSERT_TRUE(robot.maintain_fault_hold());
    ASSERT_TRUE(robot.clear_fault());
    EXPECT_EQ(robot.get_state(), RobotState::ACTIVE);
}

TEST(ModelLoaderLimitResolver, ContinuousJointResolvesWithoutPositionLimits) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    ModelLoader loader;
    auto model = loader.load(fixture_path("simple_4dof_arm.urdf"), names);
    ASSERT_TRUE(model);
    EXPECT_TRUE(model->joint_limits[0].has_position_limit);
    EXPECT_FALSE(model->joint_limits[1].has_position_limit);

    JointActuatorMapCfg mapper;
    mapper.joints_count = names.size();
    mapper.pos_ratio.assign(names.size(), 1.0);
    mapper.tor_ratio.assign(names.size(), 1.0);
    mapper.direction.assign(names.size(), 1);
    mapper.joint_zero_offset.assign(names.size(), 0.0);
    mapper.actuator_zero_offset.assign(names.size(), 0.0);

    HardwareCapabilities caps;
    for(std::size_t i = 0; i < names.size(); ++i) {
        caps.push_back({ "actuator" + std::to_string(i + 1), -10.0, 10.0, 10.0, 10.0, 100.0, 10.0 });
    }

    SafetyPolicyCfg policy;
    policy.position_margin = 0.05;
    policy.max_acc.assign(names.size(), 20.0);
    policy.max_dt_s = 0.01;
    policy.state_timeout_s = 0.05;
    policy.cmd_timeout_s = 0.1;

    LimitResolver resolver;
    auto resolved = resolver.resolve(*model, mapper, caps, policy);
    ASSERT_TRUE(resolved);
    EXPECT_FALSE(resolved->joints[1].has_position_limit);

    Safety safety;
    ASSERT_TRUE(safety.configure(to_safety_cfg(*resolved)));

    JointState state = joint_state(names.size());
    ActuatorState actuators = actuator_state(names.size());
    EXPECT_TRUE(safety.check_state(state, actuators, 0.0));
    auto safe_cmd = safety.check_joint_cmd(state, joint_cmd(names.size()), 0.001);
    EXPECT_TRUE(safe_cmd) << static_cast<int>(safe_cmd.error().code) << " index=" << safe_cmd.error().index << " value=" << safe_cmd.error().value << " limit=" << safe_cmd.error().limit;

    JointActuatorMapper mapper_instance;
    ASSERT_TRUE(mapper_instance.configure(mapper));
    auto actuator_cmd = mapper_instance.to_actuator_cmd(joint_cmd(names.size()));
    ASSERT_TRUE(actuator_cmd);
    EXPECT_EQ(actuator_cmd->pos.size(), names.size());
    auto joint_back = mapper_instance.to_joint_state(actuators);
    ASSERT_TRUE(joint_back);
    EXPECT_EQ(joint_back->pos.size(), names.size());
}

TEST(DynamicsMandatory, PlaceholderInertialFixtureComputesFiniteOutputs) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    ModelLoader loader;
    ASSERT_TRUE(loader.load(fixture_path("simple_4dof_revolute_arm.urdf"), names));

    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(dynamics_cfg(names)));

    JointState state = joint_state(names.size());
    JointVector acc(names.size(), 0.0);
    JointVector ref_acc(names.size(), 0.0);
    ASSERT_TRUE(dynamics.update(state, acc, ref_acc));

    EXPECT_EQ(dynamics.get_info().joints_count, names.size());
    EXPECT_EQ(dynamics.get_mass_matrix().rows(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_mass_matrix().cols(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_tool_jacobian().rows(), 6);
    EXPECT_EQ(dynamics.get_tool_jacobian().cols(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_gravity().size(), names.size());
    EXPECT_EQ(dynamics.get_inverse_dynamics().size(), names.size());
    EXPECT_TRUE(dynamics.get_mass_matrix().allFinite());
    EXPECT_TRUE(dynamics.get_tool_jacobian().allFinite());
}

TEST(DynamicsMandatory, FullInverseDynamicsUsesCalibratedGravityScale) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    DynamicsCfg cfg = dynamics_cfg(names);
    cfg.gravity_scale = { 0.5, 0.6, 0.7, 0.8 };

    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(cfg));

    JointState state = joint_state(names.size());
    state.pos = { 0.3, -0.4, 0.5, -0.2 };
    state.vel.assign(names.size(), 0.0);
    const JointVector zero(names.size(), 0.0);
    ASSERT_TRUE(dynamics.update(state, zero, zero));

    // dq=0 and ddq=0 => inverse dynamics must reduce to the same calibrated
    // gravity term used by the quasi-static HOLD residual.
    for(std::size_t i = 0; i < names.size(); ++i) {
        EXPECT_NEAR(dynamics.get_inverse_dynamics()[i], dynamics.get_gravity_compensation()[i], 1e-10);
    }
}

TEST(DynamicsTwoStage, LegacyUpdateMatchesStateThenReferenceUpdate) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    JointState state = joint_state(names.size());
    state.pos = { 0.1, -0.2, 0.3, -0.4 };
    state.vel = { 0.01, -0.02, 0.03, -0.04 };
    state.tor = { 0.5, -0.6, 0.7, -0.8 };
    const JointVector acc{ 0.02, -0.01, 0.03, -0.02 };
    const JointVector ref_acc{ 0.04, -0.03, 0.02, -0.01 };

    Dynamics legacy;
    ASSERT_TRUE(legacy.configure(dynamics_cfg(names)));
    ASSERT_TRUE(legacy.update(state, acc, ref_acc));

    Dynamics staged;
    ASSERT_TRUE(staged.configure(dynamics_cfg(names)));
    ASSERT_TRUE(staged.update_state(state, acc));
    ASSERT_TRUE(staged.update_reference(ref_acc));

    const DynamicsState& legacy_state = legacy.get_state();
    const DynamicsState& staged_state = staged.get_state();
    EXPECT_EQ(staged_state.pos, state.pos);
    EXPECT_EQ(staged_state.vel, state.vel);
    EXPECT_EQ(staged_state.acc, acc);
    EXPECT_EQ(staged_state.tor, state.tor);
    EXPECT_EQ(staged_state.ref_acc, ref_acc);
    EXPECT_EQ(staged_state.gravity.size(), names.size());
    EXPECT_EQ(staged_state.inverse_dynamics.size(), names.size());
    EXPECT_TRUE(staged_state.mass_matrix.isApprox(legacy_state.mass_matrix, 1e-12));
    EXPECT_TRUE(staged_state.tool_jacobian.isApprox(legacy_state.tool_jacobian, 1e-12));
    for(std::size_t i = 0; i < names.size(); ++i) {
        EXPECT_NEAR(staged_state.gravity[i], legacy_state.gravity[i], 1e-12);
        EXPECT_NEAR(staged_state.gravity_compensation[i], legacy_state.gravity_compensation[i], 1e-12);
        EXPECT_NEAR(staged_state.nonlinear[i], legacy_state.nonlinear[i], 1e-12);
        EXPECT_NEAR(staged_state.coriolis[i], legacy_state.coriolis[i], 1e-12);
        EXPECT_NEAR(staged_state.forward_dynamics[i], legacy_state.forward_dynamics[i], 1e-12);
        EXPECT_NEAR(staged_state.inverse_dynamics[i], legacy_state.inverse_dynamics[i], 1e-12);
    }
}

TEST(DynamicsTwoStage, StateUpdateWorksWithoutReferenceAndClearsReferenceCache) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(dynamics_cfg(names)));

    JointState state = joint_state(names.size());
    const JointVector acc(names.size(), 0.0);
    ASSERT_TRUE(dynamics.update(state, acc, JointVector(names.size(), 0.5)));
    ASSERT_TRUE(dynamics.update_state(state, acc));

    const DynamicsState& updated = dynamics.get_state();
    EXPECT_TRUE(dynamics.is_updated());
    EXPECT_EQ(updated.ref_acc, JointVector(names.size(), 0.0));
    EXPECT_EQ(updated.inverse_dynamics, JointVector(names.size(), 0.0));
    EXPECT_EQ(updated.gravity.size(), names.size());
    EXPECT_EQ(updated.forward_dynamics.size(), names.size());
    EXPECT_TRUE(updated.mass_matrix.allFinite());
}

TEST(DynamicsTwoStage, ReferenceUpdateRequiresStateUpdate) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(dynamics_cfg(names)));

    auto result = dynamics.update_reference(JointVector(names.size(), 0.0));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), DynamicsErr::NOT_UPDATED);
}

TEST(DynamicsTwoStage, StateUpdateValidatesInputs) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(dynamics_cfg(names)));

    JointState state = joint_state(names.size());
    auto size_result = dynamics.update_state(state, JointVector(names.size() - 1, 0.0));
    ASSERT_FALSE(size_result);
    EXPECT_EQ(size_result.error(), DynamicsErr::INVALID_INPUT_SIZE);

    JointVector acc(names.size(), 0.0);
    state.pos[1] = std::numeric_limits<double>::quiet_NaN();
    auto nan_result = dynamics.update_state(state, acc);
    ASSERT_FALSE(nan_result);
    EXPECT_EQ(nan_result.error(), DynamicsErr::NON_FINITE_INPUT);

    state = joint_state(names.size());
    state.tor[2] = std::numeric_limits<double>::infinity();
    auto inf_result = dynamics.update_state(state, acc);
    ASSERT_FALSE(inf_result);
    EXPECT_EQ(inf_result.error(), DynamicsErr::NON_FINITE_INPUT);
}

TEST(DynamicsTwoStage, ReferenceUpdateValidatesInputs) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    Dynamics dynamics;
    ASSERT_TRUE(dynamics.configure(dynamics_cfg(names)));
    ASSERT_TRUE(dynamics.update_state(joint_state(names.size()), JointVector(names.size(), 0.0)));

    auto size_result = dynamics.update_reference(JointVector(names.size() - 1, 0.0));
    ASSERT_FALSE(size_result);
    EXPECT_EQ(size_result.error(), DynamicsErr::INVALID_INPUT_SIZE);

    JointVector ref_acc(names.size(), 0.0);
    ref_acc[0] = std::numeric_limits<double>::quiet_NaN();
    auto nan_result = dynamics.update_reference(ref_acc);
    ASSERT_FALSE(nan_result);
    EXPECT_EQ(nan_result.error(), DynamicsErr::NON_FINITE_INPUT);

    ref_acc.assign(names.size(), 0.0);
    ref_acc[1] = std::numeric_limits<double>::infinity();
    auto inf_result = dynamics.update_reference(ref_acc);
    ASSERT_FALSE(inf_result);
    EXPECT_EQ(inf_result.error(), DynamicsErr::NON_FINITE_INPUT);
}

TEST(MitBackendContract, MapperPassesFullCommandToMotorBus) {
    JointActuatorMapCfg mapper_cfg;
    mapper_cfg.joints_count = 1;
    mapper_cfg.pos_ratio = { 2.0 };
    mapper_cfg.tor_ratio = { 3.0 };
    mapper_cfg.direction = { 1 };
    mapper_cfg.joint_zero_offset = { 0.1 };
    mapper_cfg.actuator_zero_offset = { -0.2 };

    JointActuatorMapper mapper;
    ASSERT_TRUE(mapper.configure(mapper_cfg));

    JointCtrlCmd joint;
    joint.pos = { 1.0 };
    joint.vel = { 2.0 };
    joint.tor = { 3.0 };
    joint.kp = { 4.0 };
    joint.kd = { 5.0 };
    auto actuator = mapper.to_actuator_cmd(joint);
    ASSERT_TRUE(actuator);

    FakeMotorBus bus;
    ASSERT_TRUE(bus.write(*actuator));
    EXPECT_DOUBLE_EQ(bus.last_cmd.pos[0], 2.0 * (1.0 - 0.1) - 0.2);
    EXPECT_DOUBLE_EQ(bus.last_cmd.vel[0], 4.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.tor[0], 1.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.kp[0], 4.0 / 6.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.kd[0], 5.0 / 6.0);
}

TEST(RobotProfile, ResolvesCoreFieldsWithoutRosEnvironment) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");
    unsetenv("SERIAL_ARM_RESOURCE_PATH");

    RobotProfileLoadOptions options;
    options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto profile = load_robot_profile_core("dm_arm_gray", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_EQ(profile->name, "dm_arm_gray");
    EXPECT_NE(profile->core_config_path.find("config/core/gray.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_plugin.find("serial_arm_hardware_damiao"), std::string::npos);
    EXPECT_NE(profile->hardware_config_path.find("config/hardware.yaml"), std::string::npos);
}

TEST(RobotProfile, MissingAndInvalidProfilesReportContext) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");

    RobotProfileLoadOptions options;
    options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto missing = load_robot_profile_core("missing_profile", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("missing_profile"), std::string::npos);

    const std::string invalid_file = std::string(SERIAL_ARM_TEST_TMP_DIR) + "/invalid_robot_profiles.yaml";
    {
        std::ofstream stream(invalid_file);
        stream << "profiles:\n  broken:\n    core:\n      package: dm_arm_description\n      config: config/core/gray.yaml\n";
    }
    RobotProfileLoadOptions invalid_options;
    invalid_options.profile_file = invalid_file;
    invalid_options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto invalid = load_robot_profile_core("broken", invalid_options);
    ASSERT_FALSE(invalid);
    EXPECT_EQ(invalid.error().code, RobotProfileErr::MISSING_FIELD);
    EXPECT_NE(invalid.error().message.find("broken"), std::string::npos);
}

TEST(RobotProfile, MultipleResourceRootsContinueUntilProfileMatches) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "multi_resource_roots";
    fs::remove_all(root);
    const fs::path resource_a = root / "resource_a";
    const fs::path resource_b = root / "resource_b";

    write_test_package(resource_a, "robot_a_description");
    write_text(resource_a / "robot_a_description" / "config" / "core.yaml", "robot_a core\n");
    write_text(resource_a / "robot_a_description" / "config" / "hardware.yaml", "robot_a hardware\n");
    write_text(resource_a / "robot_profiles.yaml", test_profile_yaml("robot_a", "serial_arm_hardware_a", "robot_a_description", "robot_a_description"));

    write_test_package(resource_b, "robot_b_description");
    write_text(resource_b / "robot_b_description" / "config" / "core.yaml", "robot_b core\n");
    write_text(resource_b / "robot_b_description" / "config" / "hardware.yaml", "robot_b hardware\n");
    write_text(resource_b / "robot_profiles.yaml", test_profile_yaml("robot_b", "serial_arm_hardware_b", "robot_b_description", "robot_b_description"));

    RobotProfileLoadOptions options;
    options.resource_paths = { resource_a.string(), resource_b.string() };
    auto profile = load_robot_profile_core("robot_b", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->profile_file.find("resource_b"), std::string::npos);
    EXPECT_NE(profile->core_config_path.find("robot_b_description"), std::string::npos);

    auto missing = load_robot_profile_core("missing_robot", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("missing_robot"), std::string::npos);
    EXPECT_NE(missing.error().message.find("resource_a"), std::string::npos);
    EXPECT_NE(missing.error().message.find("resource_b"), std::string::npos);
}

TEST(RobotProfile, ExplicitProfileFileDoesNotFallback) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "explicit_profile_file";
    fs::remove_all(root);
    const fs::path explicit_root = root / "explicit";
    const fs::path fallback_root = root / "fallback";

    write_test_package(explicit_root, "explicit_description");
    write_text(explicit_root / "explicit_description" / "config" / "core.yaml", "explicit core\n");
    write_text(explicit_root / "explicit_description" / "config" / "hardware.yaml", "explicit hardware\n");
    write_text(explicit_root / "robot_profiles.yaml", test_profile_yaml("same_robot", "serial_arm_hardware_explicit", "explicit_description", "explicit_description"));

    write_test_package(fallback_root, "fallback_description");
    write_text(fallback_root / "fallback_description" / "config" / "core.yaml", "fallback core\n");
    write_text(fallback_root / "fallback_description" / "config" / "hardware.yaml", "fallback hardware\n");
    write_text(fallback_root / "robot_profiles.yaml",
        test_profile_yaml("same_robot", "serial_arm_hardware_fallback", "fallback_description", "fallback_description") +
        "  only_in_fallback:\n"
        "    core:\n"
        "      package: fallback_description\n"
        "      config: config/core.yaml\n"
        "    hardware:\n"
        "      plugin: serial_arm_hardware_fallback\n"
        "      config_package: fallback_description\n"
        "      config: config/hardware.yaml\n");

    RobotProfileLoadOptions options;
    options.profile_file = (explicit_root / "robot_profiles.yaml").string();
    options.resource_paths = { explicit_root.string(), fallback_root.string() };
    auto profile = load_robot_profile_core("same_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->core_config_path.find("explicit_description"), std::string::npos);
    EXPECT_EQ(profile->hardware_plugin, "serial_arm_hardware_explicit");

    auto missing = load_robot_profile_core("only_in_fallback", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("only_in_fallback"), std::string::npos);
}

TEST(RobotProfile, GenericBackendPluginResolutionUsesPluginName) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "generic_backend_plugin";
    fs::remove_all(root);
    const std::string plugin = "serial_arm_hardware_test";
    write_test_package(root, "test_description");
    write_text(root / "test_description" / "config" / "core.yaml", "test core\n");
    write_text(root / "test_description" / "config" / "hardware.yaml", "test hardware\n");
    write_text(root / "robot_profiles.yaml", test_profile_yaml("test_robot", plugin, "test_description", "test_description"));
    write_text(root / "install" / plugin / "lib" / ("lib" + plugin + ".so"), "");

    RobotProfileLoadOptions options;
    options.resource_paths = { root.string() };
    auto profile = load_robot_profile_core("test_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->hardware_plugin.find("libserial_arm_hardware_test.so"), std::string::npos);
    EXPECT_EQ(profile->hardware_plugin.find("serial_arm_hardware_damiao"), std::string::npos);
}

TEST(RobotProfile, StandaloneInstalledResourceLayoutResolvesWithoutRosEnvironment) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");

    const fs::path prefix = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "standalone_install_prefix";
    fs::remove_all(prefix);
    const fs::path profiles_share = prefix / "share" / "serial_arm_robot_profiles";
    const fs::path description_share = prefix / "share" / "test_robot_description";
    const std::string plugin = "serial_arm_hardware_test";

    write_test_package(prefix / "share", "test_robot_description");
    write_text(description_share / "config" / "core.yaml", "test core\n");
    write_text(description_share / "config" / "hardware.yaml", "test hardware\n");
    write_text(profiles_share / "config" / "robot_profiles.yaml", test_profile_yaml("test_robot", plugin, "test_robot_description", "test_robot_description"));
    write_text(prefix / "lib" / ("lib" + plugin + ".so"), "");

    RobotProfileLoadOptions options;
    options.resource_paths = { prefix.string() };
    auto profile = load_robot_profile_core("test_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->profile_file.find("share/serial_arm_robot_profiles/config/robot_profiles.yaml"), std::string::npos);
    EXPECT_NE(profile->core_config_path.find("share/test_robot_description/config/core.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_config_path.find("share/test_robot_description/config/hardware.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_plugin.find("libserial_arm_hardware_test.so"), std::string::npos);
}

TEST(AdmittanceCapabilityValidation, DisabledCapabilityDoesNotRequireJointParameters) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.capability.admittance.enabled = false;
    EXPECT_TRUE(validate_robot_core_cfg(cfg));
}

TEST(AdmittanceControllerConfig, DirectMDKIsPreservedWithoutSemanticDerivation) {
    AdmittanceCapabilityCfg cfg;
    configure_test_admittance(cfg, 0.4, 3.6, 8.0, 1.0, 1.0, 0.0);

    ASSERT_EQ(cfg.controller.mass.size(), 1u);
    EXPECT_NEAR(cfg.controller.mass[0], 0.4, 1e-12);
    EXPECT_NEAR(cfg.controller.damping[0], 3.6, 1e-12);
    EXPECT_NEAR(cfg.controller.stiffness[0], 8.0, 1e-12);
    EXPECT_DOUBLE_EQ(cfg.controller.max_delta_q[0], 1.0);
    EXPECT_DOUBLE_EQ(cfg.controller.max_delta_q_dot[0], 1.0);
}

TEST(AdmittanceCapabilityValidation, EnabledCapabilityRequiresValidPerJointParameters) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 5.0, 8.0, 20.0, 0.005, 0.01, 0.0);
    EXPECT_TRUE(validate_robot_core_cfg(cfg));

    admittance.controller.mass[0] = 0.0;
    auto invalid_controller = validate_robot_core_cfg(cfg);
    ASSERT_FALSE(invalid_controller);
    EXPECT_EQ(invalid_controller.error().code, ConfigErr::INVALID_VALUE);

    admittance.controller.mass[0] = 5.0;
    admittance.calibration.torque_threshold[0] = -0.1;
    auto invalid_threshold = validate_robot_core_cfg(cfg);
    ASSERT_FALSE(invalid_threshold);
    EXPECT_EQ(invalid_threshold.error().code, ConfigErr::INVALID_VALUE);
}

TEST(RobotAdmittanceCapability, EnabledCapabilityRequiresGravityModelCallback) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 5.0, 8.0, 20.0, 0.005, 0.01, 0.0);

    Robot robot;
    auto result = robot.configure(cfg, std::make_unique<FakeMotorBus>());
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, RobotErr::INVALID_CFG);
}

TEST(RobotAdmittanceCapability, DisabledCapabilityKeepsExistingRobotBehavior) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    cfg.capability.admittance.enabled = false;

    Robot robot;
    EXPECT_TRUE(robot.configure(cfg, std::make_unique<FakeMotorBus>()));
}

TEST(RobotAdmittanceCapability, EnabledCapabilityCorrectsNominalCommandBeforeSafety) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 0.1, 0.1, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.pos[0] = -0.999;
    bus_raw->state.tor[0] = 100.0;

    ModelFeedforwardFn gravity = [](ModelFeedforwardMode mode, const JointState& state, const JointVector&, const JointVector&, double) {
        if(mode == ModelFeedforwardMode::GRAVITY) return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    ASSERT_TRUE(robot.activate());
    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    EXPECT_LT(output->joint_cmd.pos[0], 0.0);
    EXPECT_LT(output->joint_cmd.vel[0], 0.0);
    EXPECT_EQ(bus_raw->last_cmd.pos, output->actuator_cmd.pos);
}

TEST(RobotAdmittanceCapability, CompliantDragBypassesAdmittanceCorrection) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.pos[0] = -0.999;
    bus_raw->state.tor[0] = 100.0;

    ModelFeedforwardFn gravity = [](ModelFeedforwardMode, const JointState& state, const JointVector&, const JointVector&, double) {
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    ASSERT_TRUE(robot.activate());
    ASSERT_TRUE(robot.set_impedance_mode(JointImpedanceMode::COMPLIANT_DRAG));

    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    EXPECT_DOUBLE_EQ(output->joint_cmd.pos[0], bus_raw->state.pos[0]);
    EXPECT_DOUBLE_EQ(output->joint_cmd.vel[0], 0.0);
    EXPECT_EQ(robot.get_state(), RobotState::ACTIVE);
}



TEST(RobotAdmittanceCapability, HoldUsesDynamicModelTorqueForResidual) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    cfg.runtime.model_feedforward_mode = ModelFeedforwardMode::GRAVITY;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.tor[0] = 3.0;

    int gravity_calls = 0;
    int full_inverse_calls = 0;
    ModelFeedforwardFn model = [&](ModelFeedforwardMode mode, const JointState& state, const JointVector&, const JointVector&, double) {
        if(mode == ModelFeedforwardMode::GRAVITY) {
            ++gravity_calls;
            return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 1.0));
        }
        if(mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            ++full_inverse_calls;
            return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 3.0));
        }
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), model));
    ASSERT_TRUE(robot.activate());

    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->admittance_active);
    ASSERT_EQ(output->residual_raw.size(), 1u);
    EXPECT_NEAR(output->residual_raw[0], 0.0, 1e-12);
    EXPECT_GT(gravity_calls, 0);
    EXPECT_GT(full_inverse_calls, 0);
}

TEST(RobotAdmittanceCapability, TrackingUsesDynamicModelTorqueForResidual) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    cfg.runtime.model_feedforward_mode = ModelFeedforwardMode::GRAVITY;
    cfg.safety.require_continuous_cmd = false;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.tor[0] = 3.0;

    int gravity_calls = 0;
    int full_inverse_calls = 0;
    JointVector dynamic_acc;
    JointVector dynamic_ref_acc;
    ModelFeedforwardFn model = [&](ModelFeedforwardMode mode, const JointState& state, const JointVector& acc, const JointVector& ref_acc, double) {
        if(mode == ModelFeedforwardMode::GRAVITY) {
            ++gravity_calls;
            return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 1.0));
        }
        if(mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            ++full_inverse_calls;
            dynamic_acc = acc;
            dynamic_ref_acc = ref_acc;
            return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 3.0));
        }
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), model));
    ASSERT_TRUE(robot.activate());
    ASSERT_TRUE(robot.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING));
    ASSERT_TRUE(robot.set_cmd(JointPosVelCmd{ { 0.0 }, { 1.0 } }));

    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->admittance_active);
    ASSERT_EQ(output->residual_raw.size(), 1u);
    EXPECT_NEAR(output->residual_raw[0], 0.0, 1e-12);
    EXPECT_GT(gravity_calls, 0);
    EXPECT_GT(full_inverse_calls, 0);
    ASSERT_EQ(dynamic_acc.size(), 1u);
    ASSERT_EQ(dynamic_ref_acc.size(), 1u);
    EXPECT_NEAR(dynamic_acc[0], 0.0, 1e-12);
    EXPECT_NEAR(dynamic_ref_acc[0], dynamic_acc[0], 1e-12);
}

TEST(RobotAdmittanceCapability, RuntimeSuspensionBypassesAndResetsAdmittance) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.tor[0] = 1.0;
    ModelFeedforwardFn gravity = [](ModelFeedforwardMode, const JointState& state, const JointVector&, const JointVector&, double) {
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    ASSERT_TRUE(robot.activate());

    robot.set_admittance_suspended(true);
    auto suspended = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(suspended);
    EXPECT_FALSE(suspended->admittance_active);
    EXPECT_DOUBLE_EQ(suspended->joint_cmd.pos[0], bus_raw->state.pos[0]);

    robot.set_admittance_suspended(false);
    auto resumed = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(resumed);
    EXPECT_TRUE(resumed->admittance_active);
    EXPECT_LT(resumed->joint_cmd.pos[0], bus_raw->state.pos[0]);
}

TEST(RobotAdmittanceCapability, RuntimeConfigUpdateResetsStateAndExposesTelemetry) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 0.1, 0.1, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.tor[0] = 1.0;
    ModelFeedforwardFn gravity = [](ModelFeedforwardMode, const JointState& state, const JointVector&, const JointVector&, double) {
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    ASSERT_TRUE(robot.activate());

    auto updated = admittance;
    updated.controller.mass[0] = 0.5;
    updated.controller.damping[0] = 2.0;
    updated.controller.stiffness[0] = 4.0;
    updated.controller.max_delta_q_dot[0] = 0.2;
    ASSERT_TRUE(robot.set_admittance_cfg(updated));
    const auto& applied = robot.get_admittance_cfg();
    EXPECT_NEAR(applied.controller.mass[0], 0.5, 1e-12);
    EXPECT_NEAR(applied.controller.damping[0], 2.0, 1e-12);
    EXPECT_NEAR(applied.controller.stiffness[0], 4.0, 1e-12);

    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    EXPECT_TRUE(output->admittance_active);
    ASSERT_EQ(output->bias_compensated.size(), 1u);
    ASSERT_EQ(output->tau_ext_hat.size(), 1u);
    ASSERT_EQ(output->delta_q.size(), 1u);
    ASSERT_EQ(output->delta_q_dot.size(), 1u);
    EXPECT_NEAR(output->bias_compensated[0], -1.0, 1e-12);
    EXPECT_NEAR(output->tau_ext_hat[0], -1.0, 1e-12);
    EXPECT_EQ(output->torque_threshold_active.size(), 1u);
    EXPECT_EQ(output->delta_q_limited.size(), 1u);
    EXPECT_EQ(output->delta_q_dot_limited.size(), 1u);
    EXPECT_EQ(output->safety_position_margin_active.size(), 1u);
    EXPECT_EQ(output->safety_velocity_margin_active.size(), 1u);
}

TEST(RobotAdmittanceCapability, DynamicAdmittanceLimitUsesRemainingSafetyPositionSpace) {
    RobotCfg cfg = robot_cfg_for_validation(false);
    cfg.runtime.write_enabled = true;
    auto& admittance = cfg.capability.admittance;
    configure_test_admittance(admittance, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0);

    auto bus = std::make_unique<FakeMotorBus>();
    FakeMotorBus* bus_raw = bus.get();
    bus_raw->state.pos[0] = -0.899;
    bus_raw->state.tor[0] = 100.0;

    ModelFeedforwardFn gravity = [](ModelFeedforwardMode, const JointState& state, const JointVector&, const JointVector&, double) {
        return tl::expected<JointVector, ModelFeedforwardErr>(JointVector(state.pos.size(), 0.0));
        };

    Robot robot;
    ASSERT_TRUE(robot.configure(cfg, std::move(bus), gravity));
    ASSERT_TRUE(robot.activate());
    auto output = robot.cycle(Robot::Clock::now());
    ASSERT_TRUE(output);
    EXPECT_NEAR(output->joint_cmd.pos[0], -0.9, 1e-12);
    EXPECT_DOUBLE_EQ(output->joint_cmd.vel[0], 0.0);
    EXPECT_EQ(robot.get_state(), RobotState::ACTIVE);
}

