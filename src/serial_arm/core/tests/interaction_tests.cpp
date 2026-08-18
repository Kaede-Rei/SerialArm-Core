#include <gtest/gtest.h>

#include "serial_arm/interaction/admittance_calibration.hpp"
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

TEST(TorqueResidualObserverTests, FiltersModelTorqueMinusMeasuredTorque) {
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


TEST(TorqueResidualObserverTests, BiasPrimedResetFiltersFirstSampleInsteadOfPassingRawResidual) {
    TorqueResidualObserver observer;
    TorqueResidualObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.filter_alpha = 0.1;
    cfg.initial_filtered_residual = { 0.2 };
    ASSERT_TRUE(observer.configure(cfg));

    auto first = observer.update(JointVector{ 0.0 }, JointVector{ 0.5 });
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->residual[0], 0.5);
    EXPECT_NEAR(first->residual_filtered[0], 0.23, 1e-12);

    observer.reset();
    auto after_reset = observer.update(JointVector{ 0.0 }, JointVector{ 0.5 });
    ASSERT_TRUE(after_reset);
    EXPECT_NEAR(after_reset->residual_filtered[0], 0.23, 1e-12);
}

TEST(InteractionControllerTests, UsesTorqueBiasAsResidualFilterResetBaseline) {
    auto cfg = enabled_cfg();
    cfg.residual.filter_alpha = 0.1;
    ASSERT_DOUBLE_EQ(cfg.external_torque.torque_bias[0], 0.2);

    InteractionController controller;
    ASSERT_TRUE(controller.configure(cfg));

    const auto cmd = nominal_cmd();
    auto output = controller.update(InteractionInput{
        JointVector{ 0.0 },
        JointVector{ 0.5 },
        cmd,
        0.01,
    });
    ASSERT_TRUE(output);
    ASSERT_EQ(output->residual.residual_filtered.size(), 1u);
    EXPECT_NEAR(output->residual.residual_filtered[0], 0.23, 1e-12);
}

TEST(JointAdmittanceControllerTests, ComputesCriticalDampingAndDampingRatio) {
    const auto metrics = compute_admittance_damping_metrics(0.5, 5.5, 15.0);
    ASSERT_TRUE(metrics);
    EXPECT_NEAR(metrics->critical_damping, 2.0 * std::sqrt(7.5), 1e-12);
    EXPECT_NEAR(metrics->damping_ratio, 5.5 / (2.0 * std::sqrt(7.5)), 1e-12);

    EXPECT_FALSE(compute_admittance_damping_metrics(0.5, 1.0, 0.0));
}


TEST(AdmittanceStaticCalibrationTests, FitsGravityScaleBiasAndThresholdAcrossPoses) {
    std::vector<AdmittanceStaticPoseSamples> poses(3);
    const std::vector<double> gravity_values{ -2.0, 0.0, 2.0 };
    for(std::size_t p = 0; p < poses.size(); ++p) {
        for(int k = 0; k < 20; ++k) {
            const double g = gravity_values[p];
            const double noise = (k % 2 == 0) ? 0.01 : -0.01;
            // production residual = gravity_scale * gravity - measured_torque - torque_bias
            // target parameters: gravity_scale=0.8, torque_bias=0.2
            const double measured = 0.8 * g - 0.2 + noise;
            poses[p].samples.push_back(AdmittanceStaticSample{ JointVector{ g }, JointVector{ measured } });
        }
    }

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;
    cfg.threshold_margin = 1.2;

    auto result = calibrate_admittance_static(poses, cfg);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->gravity_scale_observable, std::vector<std::uint8_t>{ 1 });
    EXPECT_NEAR(result->gravity_scale[0], 0.8, 1e-6);
    EXPECT_NEAR(result->torque_bias[0], 0.2, 1e-6);
    // Threshold is derived from the unfiltered static residual envelope, so later
    // filter_alpha tuning does not change the one-time calibration semantics.
    EXPECT_NEAR(result->residual_p99[0], 0.01, 1e-12);
    EXPECT_NEAR(result->torque_threshold[0], 0.012, 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, ThresholdAlsoCoversObservedStaticMaximum) {
    AdmittanceStaticPoseSamples pose;
    for(int k = 0; k < 199; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ 0.0 } });
    }
    pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.1 } });

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;
    cfg.threshold_margin = 1.2;

    auto result = calibrate_admittance_static({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->residual_p99[0], 0.0, 1e-12);
    EXPECT_NEAR(result->residual_max[0], 0.1, 1e-12);
    EXPECT_NEAR(result->torque_threshold[0], 0.105, 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, KeepsFallbackScaleWhenGravityIsUnobservable) {
    AdmittanceStaticPoseSamples pose;
    for(int k = 0; k < 20; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.35 } });
    }

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 0.9 };
    cfg.gravity_observability_span = 0.25;
    cfg.threshold_margin = 1.2;

    auto result = calibrate_admittance_static({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->gravity_scale_observable, std::vector<std::uint8_t>{ 0 });
    EXPECT_NEAR(result->gravity_scale[0], 0.9, 1e-12);
    EXPECT_NEAR(result->torque_bias[0], 0.35, 1e-12);
}

TEST(ExternalTorqueObserverTests, AppliesBiasThenSmoothThresholdWithoutSubtractingThreshold) {
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

    // bias compensated = 0.45 lies halfway through [threshold, 2*threshold].
    residual.residual = { 0.65 };
    residual.residual_filtered = { 0.65 };
    auto transition = observer.update(residual);
    ASSERT_TRUE(transition);
    EXPECT_GT(transition->tau_ext_hat[0], 0.0);
    EXPECT_LT(transition->tau_ext_hat[0], 0.45);
    EXPECT_EQ(transition->threshold_active[0], 1);

    // Above 2*threshold preserves the full bias-compensated torque.
    residual.residual = { 0.9 };
    residual.residual_filtered = { 0.9 };
    auto above = observer.update(residual);
    ASSERT_TRUE(above);
    EXPECT_NEAR(above->tau_ext_hat[0], 0.7, 1e-12);
    EXPECT_EQ(above->threshold_active[0], 0);
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
    EXPECT_NEAR(output->bias_compensated[0], -1.0, 1e-12);
    EXPECT_NEAR(output->tau_ext_hat[0], -1.0, 1e-12);
    EXPECT_EQ(output->threshold_active[0], 0);
    EXPECT_LT(output->corrected_cmd.pos[0], 0.0);
    EXPECT_LT(output->corrected_cmd.vel[0], 0.0);
    EXPECT_EQ(output->delta_q_limited.size(), 1u);
    EXPECT_EQ(output->delta_q_dot_limited.size(), 1u);
}
