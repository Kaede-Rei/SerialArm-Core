#include <gtest/gtest.h>

#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/interaction_controller.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

#include <cmath>

using namespace serial_arm;

namespace {

JointCtrlCmd nominal_cmd(double pos = 1.0) {
    JointCtrlCmd cmd;
    cmd.pos = { pos };
    cmd.vel = { 0.0 };
    cmd.tor = { 0.0 };
    cmd.kp = { 10.0 };
    cmd.kd = { 1.0 };
    return cmd;
}

InteractionControllerCfg enabled_cfg() {
    InteractionControllerCfg cfg;
    cfg.enabled = true;
    cfg.residual = TorqueResidualObserverCfg{ 1, 1.0 };
    cfg.external_torque.joints_count = 1;
    cfg.external_torque.torque_bias = { 0.2 };
    cfg.external_torque.torque_threshold = { 0.3 };
    cfg.admittance.joints_count = 1;
    cfg.admittance.enabled = { 1 };
    cfg.admittance.mass = { 1.0 };
    cfg.admittance.damping = { 0.0 };
    cfg.admittance.stiffness = { 0.0 };
    cfg.admittance.max_delta_q = { 1.0 };
    cfg.admittance.max_delta_q_dot = { 1.0 };
    return cfg;
}

} // namespace

TEST(TorqueResidualObserverTests, FiltersGravityMinusMeasuredTorque) {
    TorqueResidualObserver observer;
    ASSERT_TRUE(observer.configure(TorqueResidualObserverCfg{ 1, 0.5 }));

    auto first = observer.update(JointVector{ 2.0 }, JointVector{ 1.0 });
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->residual[0], -1.0);
    EXPECT_DOUBLE_EQ(first->residual_filtered[0], -1.0);

    auto second = observer.update(JointVector{ 0.0 }, JointVector{ 1.0 });
    ASSERT_TRUE(second);
    EXPECT_DOUBLE_EQ(second->residual[0], 1.0);
    EXPECT_NEAR(second->residual_filtered[0], 0.0, 1e-12);
}

TEST(ExternalTorqueObserverTests, AppliesBiasThenHardThresholdWithoutSubtractingThreshold) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { 0.2 };
    cfg.torque_threshold = { 0.3 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate residual;
    residual.residual = { 0.4 };
    residual.residual_filtered = { 0.4 };
    auto below = observer.update(residual);
    ASSERT_TRUE(below);
    EXPECT_DOUBLE_EQ(below->tau_ext_hat[0], 0.0);

    residual.residual = { 0.8 };
    residual.residual_filtered = { 0.8 };
    auto above = observer.update(residual);
    ASSERT_TRUE(above);
    EXPECT_NEAR(above->tau_ext_hat[0], 0.6, 1e-12);
}

TEST(JointAdmittanceControllerTests, BoundaryStopsOutwardVelocityAndAllowsReverseRecovery) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 1.0 };
    cfg.damping = { 0.0 };
    cfg.stiffness = { 0.0 };
    cfg.max_delta_q = { 0.03 };
    cfg.max_delta_q_dot = { 0.2 };
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput saturation_input;
    saturation_input.tau_ext_hat = { 100.0 };
    saturation_input.dt = 1.0;
    auto saturated = controller.update(saturation_input);
    ASSERT_TRUE(saturated);
    EXPECT_DOUBLE_EQ(saturated->delta_q[0], 0.03);
    EXPECT_DOUBLE_EQ(saturated->delta_q_dot[0], 0.0);

    JointAdmittanceInput recovery_input;
    recovery_input.tau_ext_hat = { -1.0 };
    recovery_input.dt = 0.1;
    auto recovery = controller.update(recovery_input);
    ASSERT_TRUE(recovery);
    EXPECT_LT(recovery->delta_q[0], 0.03);
    EXPECT_LT(recovery->delta_q_dot[0], 0.0);
}

TEST(JointAdmittanceControllerTests, DynamicPositionLimitStopsCorrectionAtRemainingSafetySpace) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 1.0 };
    cfg.damping = { 0.0 };
    cfg.stiffness = { 0.0 };
    cfg.max_delta_q = { 1.0 };
    cfg.max_delta_q_dot = { 1.0 };
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { -100.0 };
    input.dt = 0.01;
    input.min_delta_q = { -0.0001 };
    input.max_delta_q = { 1.0 };
    input.min_delta_q_dot = { -1.0 };
    input.max_delta_q_dot = { 1.0 };

    auto output = controller.update(input);
    ASSERT_TRUE(output);
    EXPECT_NEAR(output->delta_q[0], -0.0001, 1e-12);
    EXPECT_DOUBLE_EQ(output->delta_q_dot[0], 0.0);
}

TEST(JointAdmittanceControllerTests, DynamicVelocityLimitRestrictsCorrectionBeforeIntegration) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 1.0 };
    cfg.damping = { 0.0 };
    cfg.stiffness = { 0.0 };
    cfg.max_delta_q = { 1.0 };
    cfg.max_delta_q_dot = { 1.0 };
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { -100.0 };
    input.dt = 0.01;
    input.min_delta_q = { -1.0 };
    input.max_delta_q = { 1.0 };
    input.min_delta_q_dot = { -0.02 };
    input.max_delta_q_dot = { 1.0 };

    auto output = controller.update(input);
    ASSERT_TRUE(output);
    EXPECT_NEAR(output->delta_q_dot[0], -0.02, 1e-12);
    EXPECT_NEAR(output->delta_q[0], -0.0002, 1e-12);
}

TEST(InteractionControllerTests, DisabledCapabilityPassesNominalCommandThroughWithoutObserverWork) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.enabled = false;
    ASSERT_TRUE(controller.configure(cfg));

    const auto nominal = nominal_cmd();
    InteractionInput input{ JointVector{}, JointVector{}, nominal, 0.005, {}, {}, {}, {} };
    auto output = controller.update(input);
    ASSERT_TRUE(output);
    EXPECT_EQ(output->corrected_cmd.pos, nominal.pos);
    EXPECT_EQ(output->corrected_cmd.vel, nominal.vel);
    EXPECT_TRUE(output->tau_ext_hat.empty());
    EXPECT_TRUE(output->delta_q.empty());
    EXPECT_TRUE(output->delta_q_dot.empty());
}

TEST(InteractionControllerTests, EnabledCapabilityAppliesThresholdedExternalTorqueToNominalCommand) {
    InteractionController controller;
    ASSERT_TRUE(controller.configure(enabled_cfg()));

    const auto nominal = nominal_cmd(0.0);
    InteractionInput input{ JointVector{ 0.8 }, JointVector{ 0.0 }, nominal, 0.1, {}, {}, {}, {} };
    auto output = controller.update(input);
    ASSERT_TRUE(output);
    // residual = gravity - measured = -0.8; bias -0.2 => -1.0, above threshold and preserved.
    EXPECT_NEAR(output->tau_ext_hat[0], -1.0, 1e-12);
    EXPECT_LT(output->corrected_cmd.pos[0], 0.0);
    EXPECT_LT(output->corrected_cmd.vel[0], 0.0);
}
