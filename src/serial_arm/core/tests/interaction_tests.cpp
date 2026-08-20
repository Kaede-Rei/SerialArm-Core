#include <gtest/gtest.h>

#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/generalized_momentum_observer.hpp"
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
    cfg.residual = TorqueResidualObserverCfg{ 1, 1.0, {} };
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
    ASSERT_TRUE(observer.configure(TorqueResidualObserverCfg{ 1, 0.5, {} }));

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
        {}, {}, {}, {}, {}, {},
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
    EXPECT_NEAR(metrics->natural_frequency, std::sqrt(30.0), 1e-12);
    EXPECT_NEAR(metrics->settling_time_95_critical, 4.74 / std::sqrt(30.0), 1e-12);

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
    // The synthetic feedback occupies two repeated levels 0.02 Nm apart, so the
    // quantization guard reserves one further level beyond the 0.01 Nm envelope.
    EXPECT_NEAR(result->torque_threshold[0], 0.03 * (1.0 + 1e-6), 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, FitsEachStaticPoseWithEqualWeightRegardlessOfFrameCount) {
    std::vector<AdmittanceStaticPoseSamples> poses(3);
    poses[0].samples.push_back(AdmittanceStaticSample{ JointVector{ -1.0 }, JointVector{ -1.0 } });
    poses[1].samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.2 } });
    for(int k = 0; k < 100; ++k) {
        poses[2].samples.push_back(AdmittanceStaticSample{ JointVector{ 1.0 }, JointVector{ 0.5 } });
    }

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;

    auto result = calibrate_admittance_static(poses, cfg);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->gravity_scale[0], 0.75, 1e-12);
    EXPECT_NEAR(result->torque_bias[0], 0.25, 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, ThresholdCoversLeaveOnePoseOutGeneralizationError) {
    std::vector<AdmittanceStaticPoseSamples> poses(3);
    poses[0].samples.push_back(AdmittanceStaticSample{ JointVector{ -1.0 }, JointVector{ -1.0 } });
    poses[1].samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ 0.0 } });
    poses[2].samples.push_back(AdmittanceStaticSample{ JointVector{ 1.0 }, JointVector{ 0.6 } });

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;
    cfg.threshold_margin = 1.2;
    cfg.threshold_max_margin = 1.05;

    auto result = calibrate_admittance_static(poses, cfg);
    ASSERT_TRUE(result);

    // The all-pose fit only sees a 0.2 Nm in-sample residual, but leaving either
    // endpoint pose out exposes a 0.4 Nm cross-pose generalization error.
    // The calibration threshold must protect the latter rather than overfit the
    // poses that were used to estimate gravity_scale / torque_bias.
    EXPECT_NEAR(result->within_pose_residual_p99[0], 0.0, 1e-12);
    EXPECT_NEAR(result->within_pose_residual_max[0], 0.0, 1e-12);
    EXPECT_NEAR(result->lopo_residual_p99[0], 0.4, 1e-12);
    EXPECT_NEAR(result->lopo_residual_max[0], 0.4, 1e-12);
    EXPECT_NEAR(result->residual_p99[0], 0.4, 1e-12);
    EXPECT_NEAR(result->residual_max[0], 0.4, 1e-12);
    EXPECT_NEAR(result->torque_threshold[0], 0.48, 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, ThresholdAddsOneMeasuredTorqueQuantizationStepBeyondObservedEnvelope) {
    std::vector<AdmittanceStaticPoseSamples> poses(3);
    for(int k = 0; k < 20; ++k) {
        poses[0].samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ (k % 2 == 0) ? 0.00 : 0.01 } });
        poses[1].samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ (k % 2 == 0) ? 0.01 : 0.02 } });
        poses[2].samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ (k % 2 == 0) ? 0.02 : 0.03 } });
    }

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;
    cfg.threshold_margin = 1.2;
    cfg.threshold_max_margin = 1.05;

    auto result = calibrate_admittance_static(poses, cfg);
    ASSERT_TRUE(result);

    // The observed LOPO/static envelope ends at 0.02 Nm, while measured torque
    // itself changes only in 0.01 Nm discrete levels. One further feedback LSB
    // must therefore be reserved beyond the observed envelope so a static
    // validation sample landing on the next quantization bin is not rejected.
    EXPECT_NEAR(result->residual_max[0], 0.02, 1e-12);
    EXPECT_GT(result->torque_threshold[0], 0.03);
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

TEST(AdmittanceStaticCalibrationTests, ValidationChecksOnlyStaticObserverEnvelope) {
    AdmittanceStaticPoseSamples pose;
    pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 1.0 }, JointVector{ 0.79 } });
    pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 1.0 }, JointVector{ 0.78 } });

    AdmittanceStaticValidationCfg cfg;
    cfg.joints_count = 1;
    cfg.gravity_scale = { 1.0 };
    cfg.torque_bias = { 0.2 };
    cfg.torque_threshold = { 0.05 };

    auto result = evaluate_admittance_static_validation({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->residual_max[0], 0.02, 1e-12);
    EXPECT_NEAR(result->threshold_utilization[0], 0.4, 1e-12);
    EXPECT_EQ(result->pass, std::vector<std::uint8_t>{ 1 });
}

TEST(AdmittanceStaticCalibrationTests, ValidationAllowsOneQuantizationBinBeyondThresholdWhenP99IsInside) {
    AdmittanceStaticPoseSamples pose;
    for(int k = 0; k < 196; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.04 } });
    }
    for(int k = 0; k < 3; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.05 } });
    }
    pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.06 } });

    AdmittanceStaticValidationCfg cfg;
    cfg.joints_count = 1;
    cfg.gravity_scale = { 1.0 };
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.05 };

    auto result = evaluate_admittance_static_validation({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->residual_p99[0], 0.05, 1e-12);
    EXPECT_NEAR(result->residual_max[0], 0.06, 1e-12);
    EXPECT_NEAR(result->feedback_quantization_step[0], 0.01, 1e-12);
    EXPECT_GT(result->guarded_max_limit[0], 0.06);
    EXPECT_EQ(result->pass, std::vector<std::uint8_t>{ 1 });
}

TEST(AdmittanceStaticCalibrationTests, ValidationStillFailsWhenPersistentResidualExceedsThreshold) {
    AdmittanceStaticPoseSamples pose;
    for(int k = 0; k < 200; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.06 } });
    }

    AdmittanceStaticValidationCfg cfg;
    cfg.joints_count = 1;
    cfg.gravity_scale = { 1.0 };
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.05 };

    auto result = evaluate_admittance_static_validation({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->residual_p99[0], 0.06, 1e-12);
    EXPECT_NEAR(result->feedback_quantization_step[0], 0.0, 1e-12);
    EXPECT_EQ(result->pass, std::vector<std::uint8_t>{ 0 });
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
    InteractionInput input{ JointVector{}, JointVector{}, nominal, 0.005, {}, {}, {}, {}, {}, {} };
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
    InteractionInput input{ JointVector{ 0.8 }, JointVector{ 0.0 }, nominal, 0.1, {}, {}, {}, {}, {}, {} };
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

TEST(AdmittanceFrictionCalibrationTests, FitsSignedForwardAndReverseResidualModel) {
    std::vector<AdmittanceFrictionSample> samples;
    for(int k = 0; k < 80; ++k) {
        const double speed = 0.08 + 0.004 * static_cast<double>(k);
        samples.push_back(AdmittanceFrictionSample{
            JointVector{ speed }, JointVector{ 0.2 }, JointVector{ -0.18 - 0.12 * speed }
        });
        samples.push_back(AdmittanceFrictionSample{
            JointVector{ -speed }, JointVector{ -0.2 }, JointVector{ 0.24 + 0.08 * speed }
        });
    }

    AdmittanceFrictionCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.min_fit_velocity = 0.05;
    cfg.max_fit_acceleration = 1.0;
    cfg.min_samples_per_direction = 20;

    const auto result = calibrate_admittance_friction(samples, cfg);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->observable, std::vector<std::uint8_t>{ 1 });
    EXPECT_NEAR(result->positive_coulomb[0], -0.18, 1e-6);
    EXPECT_NEAR(result->positive_viscous[0], -0.12, 1e-6);
    EXPECT_NEAR(result->negative_coulomb[0], 0.24, 1e-6);
    EXPECT_NEAR(result->negative_viscous[0], 0.08, 1e-6);
    EXPECT_LT(result->residual_rms_after[0], 1e-8);
    EXPECT_GT(result->residual_rms_before[0], 0.1);
}

TEST(AdmittanceFrictionCalibrationTests, RejectsViscousFitWhenSpeedSpanIsTooSmall) {
    std::vector<AdmittanceFrictionSample> samples;
    for(int k = 0; k < 80; ++k) {
        const double speed = 0.10 + 1.0e-5 * static_cast<double>(k);
        samples.push_back(AdmittanceFrictionSample{
            JointVector{ speed }, JointVector{ 0.1 }, JointVector{ -0.20 - 0.10 * speed }
        });
        samples.push_back(AdmittanceFrictionSample{
            JointVector{ -speed }, JointVector{ -0.1 }, JointVector{ 0.25 + 0.08 * speed }
        });
    }

    AdmittanceFrictionCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.min_fit_velocity = 0.05;
    cfg.max_fit_acceleration = 1.0;
    cfg.min_speed_span = 0.03;
    cfg.min_samples_per_direction = 30;

    const auto result = calibrate_admittance_friction(samples, cfg);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->observable, std::vector<std::uint8_t>{ 0 });
    EXPECT_LT(result->positive_speed_span[0], cfg.min_speed_span);
    EXPECT_LT(result->negative_speed_span[0], cfg.min_speed_span);
}

TEST(ExternalTorqueObserverTests, RemovesCalibratedMovingFrictionResidualBeforeThreshold) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.0 };
    cfg.friction.enabled = true;
    cfg.friction.velocity_transition = 0.03;
    cfg.friction.positive_coulomb = { -0.18 };
    cfg.friction.positive_viscous = { -0.12 };
    cfg.friction.negative_coulomb = { 0.24 };
    cfg.friction.negative_viscous = { 0.08 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate residual;
    residual.residual = { -0.128 };
    residual.residual_filtered = { -0.128 }; // friction -0.228 + true external +0.10 at dq=+0.4
    const auto estimate = observer.update(residual, JointVector{ 0.4 });
    ASSERT_TRUE(estimate);
    EXPECT_NEAR(estimate->friction_residual_hat[0], -0.228, 1e-12);
    EXPECT_NEAR(estimate->friction_compensated[0], 0.10, 1e-12);
    EXPECT_NEAR(estimate->tau_ext_hat[0], 0.10, 1e-12);
}

TEST(ExternalTorqueObserverTests, RetainsLastMotionDirectionNearZeroWithoutOverSubtractingObservedResidual) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.0 };
    cfg.friction.enabled = true;
    cfg.friction.velocity_transition = 0.03;
    cfg.friction.positive_coulomb = { -0.20 };
    cfg.friction.positive_viscous = { 0.0 };
    cfg.friction.negative_coulomb = { 0.20 };
    cfg.friction.negative_viscous = { 0.0 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate moving;
    moving.residual = { -0.20 };
    moving.residual_filtered = { -0.20 };
    ASSERT_TRUE(observer.update(moving, JointVector{ 0.2 }));

    TorqueResidualEstimate stopped;
    stopped.residual = { -0.15 };
    stopped.residual_filtered = { -0.15 };
    const auto estimate = observer.update(stopped, JointVector{ 0.0 });
    ASSERT_TRUE(estimate);
    EXPECT_NEAR(estimate->friction_residual_hat[0], -0.15, 1e-12);
    EXPECT_NEAR(estimate->friction_compensated[0], 0.0, 1e-12);

    // If the static residual is larger than the learned friction component, keep the excess as external torque.
    stopped.residual = { -0.35 };
    stopped.residual_filtered = { -0.35 };
    const auto with_external = observer.update(stopped, JointVector{ 0.0 });
    ASSERT_TRUE(with_external);
    // Zero-speed memory decays continuously, so the second stopped sample is
    // slightly smaller than the full Coulomb value instead of persisting forever.
    EXPECT_GT(with_external->friction_residual_hat[0], -0.20);
    EXPECT_LT(with_external->friction_residual_hat[0], -0.19);
    EXPECT_GT(std::abs(with_external->friction_compensated[0]), 0.15);
    EXPECT_LT(std::abs(with_external->friction_compensated[0]), 0.16);
}


TEST(GeneralizedMomentumObserverTests, ConvergesToConstantExternalTorqueWithoutAccelerationInput) {
    GeneralizedMomentumObserver observer;
    GeneralizedMomentumObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.gain = { 25.0 };
    ASSERT_TRUE(observer.configure(cfg));

    // M=1, C=0, g=0. A constant +1 Nm external torque produces dq=t.
    GeneralizedMomentumInput input;
    input.measured_torque = { 0.0 };
    input.gravity = { 0.0 };
    input.coriolis = { 0.0 };
    input.mass_matrix = { JointVector{ 1.0 } };
    input.dt = 0.001;

    double velocity = 0.0;
    for(int k = 0; k < 2000; ++k) {
        velocity += input.dt;
        input.velocity = { velocity };
        ASSERT_TRUE(observer.update(input));
    }
    const auto output = observer.update(input);
    ASSERT_TRUE(output);
    EXPECT_NEAR(output->tau_ext_hat[0], 1.0, 0.03);
}

TEST(GeneralizedMomentumObserverTests, BiasPrimedInitializationDoesNotCreateStaticTransient) {
    GeneralizedMomentumObserver observer;
    GeneralizedMomentumObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.gain = { 25.0 };
    cfg.initial_residual = { 0.20 };
    ASSERT_TRUE(observer.configure(cfg));

    GeneralizedMomentumInput input;
    input.measured_torque = { 0.0 };
    input.gravity = { 0.20 };
    input.coriolis = { 0.0 };
    input.mass_matrix = { JointVector{ 1.0 } };
    input.velocity = { 0.0 };
    input.dt = 0.005;

    for(int k = 0; k < 20; ++k) {
        const auto output = observer.update(input);
        ASSERT_TRUE(output);
        EXPECT_NEAR(output->tau_ext_hat[0], 0.20, 1e-12);
    }
}

TEST(GeneralizedMomentumObserverTests, RejectsNoExternalTorqueDuringChangingMassWithoutQdd) {
    GeneralizedMomentumObserver observer;
    GeneralizedMomentumObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.gain = { 30.0 };
    ASSERT_TRUE(observer.configure(cfg));

    GeneralizedMomentumInput input;
    input.measured_torque = { 0.0 };
    input.gravity = { 0.0 };
    input.coriolis = { 0.0 };
    input.velocity = { 0.0 };
    input.mass_matrix = { JointVector{ 1.0 } };
    input.dt = 0.005;
    ASSERT_TRUE(observer.update(input));

    for(int k = 0; k < 200; ++k) {
        input.mass_matrix[0][0] = 1.0 + 0.001 * static_cast<double>(k);
        input.velocity[0] = 0.0;
        const auto output = observer.update(input);
        ASSERT_TRUE(output);
        EXPECT_NEAR(output->tau_ext_hat[0], 0.0, 1e-9);
    }
}

TEST(AdmittanceFrictionCalibrationTests, CrossValidationRejectsReplaySpecificModel) {
    std::vector<AdmittanceFrictionSample> reverse;
    std::vector<AdmittanceFrictionSample> forward;
    for(int k = 0; k < 80; ++k) {
        const double speed = 0.08 + 0.004 * static_cast<double>(k);
        reverse.push_back({ { speed }, { 0.1 }, { -0.20 - 0.10 * speed } });
        reverse.push_back({ { -speed }, { -0.1 }, { 0.20 + 0.10 * speed } });
        // Deliberately different replay-specific offset: a model fit on one pass must not validate on the other.
        forward.push_back({ { speed }, { 0.1 }, { -0.55 - 0.10 * speed } });
        forward.push_back({ { -speed }, { -0.1 }, { 0.55 + 0.10 * speed } });
    }
    AdmittanceFrictionCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.min_fit_velocity = 0.05;
    cfg.max_fit_acceleration = 1.0;
    cfg.min_speed_span = 0.03;
    cfg.min_samples_per_direction = 20;
    cfg.cross_validation_max_rms_ratio = 0.8;

    const auto result = calibrate_admittance_friction_cross_validated(reverse, forward, cfg);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->observable[0], 1);
    EXPECT_EQ(result->validation_pass[0], 0);
    EXPECT_GT(result->cross_residual_rms_after[0], 0.8 * result->cross_residual_rms_before[0]);
}

TEST(ExternalTorqueObserverTests, ZeroSpeedStaticBaselinePersistsAndExposesNewExternalChange) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.05 };
    cfg.friction.enabled = true;
    cfg.friction.velocity_transition = 0.03;
    cfg.friction.zero_velocity_adaptation_s = 0.10;
    cfg.friction.positive_coulomb = { -0.20 };
    cfg.friction.positive_viscous = { 0.0 };
    cfg.friction.negative_coulomb = { 0.20 };
    cfg.friction.negative_viscous = { 0.0 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate residual;
    residual.residual = { -0.20 };
    residual.residual_filtered = { -0.20 };
    ASSERT_TRUE(observer.update(residual, JointVector{ 0.2 }, 0.01));

    // The post-motion static residual is stiction/hysteresis. Latch it as a baseline and
    // keep cancelling it instead of letting a time decay recreate steady-state error.
    residual.residual = { -0.15 };
    residual.residual_filtered = { -0.15 };
    auto stopped = observer.update(residual, JointVector{ 0.0 }, 0.01);
    ASSERT_TRUE(stopped);
    for(int k = 0; k < 200; ++k) {
        stopped = observer.update(residual, JointVector{ 0.0 }, 0.01);
        ASSERT_TRUE(stopped);
    }
    EXPECT_NEAR(stopped->friction_residual_hat[0], -0.15, 0.01);
    EXPECT_NEAR(stopped->friction_compensated[0], 0.0, 0.01);

    // A new residual step larger than the calibrated no-contact envelope must not be
    // absorbed by the adaptive baseline; the excess remains visible as external torque.
    residual.residual = { -0.30 };
    residual.residual_filtered = { -0.30 };
    stopped = observer.update(residual, JointVector{ 0.0 }, 0.01);
    ASSERT_TRUE(stopped);
    EXPECT_NEAR(stopped->friction_residual_hat[0], -0.15, 0.01);
    EXPECT_LT(stopped->friction_compensated[0], -0.10);
    EXPECT_LT(stopped->tau_ext_hat[0], -0.10);
}

TEST(ExternalTorqueObserverTests, StaticBaselineNeverManufacturesOppositeTorque) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.02 };
    cfg.friction.enabled = true;
    cfg.friction.velocity_transition = 0.03;
    cfg.friction.zero_velocity_adaptation_s = 0.10;
    cfg.friction.positive_coulomb = { -0.20 };
    cfg.friction.positive_viscous = { 0.0 };
    cfg.friction.negative_coulomb = { 0.20 };
    cfg.friction.negative_viscous = { 0.0 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate residual;
    residual.residual = { -0.20 };
    residual.residual_filtered = { -0.20 };
    ASSERT_TRUE(observer.update(residual, JointVector{ 0.2 }, 0.01));
    residual.residual = { -0.15 };
    residual.residual_filtered = { -0.15 };
    ASSERT_TRUE(observer.update(residual, JointVector{ 0.0 }, 0.01));

    // If an opposite external torque cancels the stored friction baseline, compensation
    // is bounded by the currently observed residual and may never create a false opposite torque.
    residual.residual = { 0.0 };
    residual.residual_filtered = { 0.0 };
    const auto cancelled = observer.update(residual, JointVector{ 0.0 }, 0.01);
    ASSERT_TRUE(cancelled);
    EXPECT_NEAR(cancelled->friction_residual_hat[0], 0.0, 1e-12);
    EXPECT_NEAR(cancelled->friction_compensated[0], 0.0, 1e-12);
}

TEST(JointAdmittanceControllerTests, VariableModeUsesMDWhileContactAndRestoresSpringOnRelease) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 0.5 };
    cfg.damping = { 1.0 };      // follow damping
    cfg.stiffness = { 8.0 };    // return stiffness
    cfg.max_delta_q = { 1.0 };
    cfg.max_delta_q_dot = { 2.0 };
    cfg.variable.enabled = true;
    cfg.variable.engage_time_s = 0.02;
    cfg.variable.release_time_s = 0.08;
    cfg.variable.soft_velocity_ratio = 0.7;
    cfg.variable.max_damping_multiplier = 4.0;
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { 0.6 };
    input.contact_confidence = { 1.0 };
    input.dt = 0.005;
    JointAdmittanceOutput out;
    for(int k = 0; k < 200; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
    }
    EXPECT_GT(out.contact_blend[0], 0.95);
    EXPECT_LT(out.effective_stiffness[0], 0.5);
    const double displaced = out.delta_q[0];
    EXPECT_GT(displaced, 0.1);

    input.tau_ext_hat = { 0.0 };
    input.contact_confidence = { 0.0 };
    for(int k = 0; k < 300; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
    }
    EXPECT_LT(out.contact_blend[0], 0.05);
    EXPECT_GT(out.effective_stiffness[0], 7.5);
    EXPECT_LT(std::abs(out.delta_q[0]), 0.02);
}

TEST(JointAdmittanceControllerTests, VariableModeBuildsSoftVelocityWallBeforeHardLimit) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 0.2 };
    cfg.damping = { 0.4 };
    cfg.stiffness = { 5.0 };
    cfg.max_delta_q = { 2.0 };
    cfg.max_delta_q_dot = { 1.0 };
    cfg.variable.enabled = true;
    cfg.variable.engage_time_s = 0.001;
    cfg.variable.release_time_s = 0.1;
    cfg.variable.soft_velocity_ratio = 0.5;
    cfg.variable.max_damping_multiplier = 5.0;
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { 2.0 };
    input.contact_confidence = { 1.0 };
    input.dt = 0.002;
    JointAdmittanceOutput out;
    for(int k = 0; k < 500; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
        if(std::abs(out.delta_q_dot[0]) > 0.55) break;
    }
    EXPECT_GT(out.effective_damping[0], cfg.damping[0]);
    EXPECT_LT(std::abs(out.delta_q_dot[0]), cfg.max_delta_q_dot[0]);
}


TEST(JointAdmittanceControllerTests, PerJointSoftVelocityOverridesLegacyRatio) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 0.2 };
    cfg.damping = { 0.4 };
    cfg.stiffness = { 5.0 };
    cfg.max_delta_q = { 2.0 };
    cfg.max_delta_q_dot = { 1.0 };
    cfg.variable.enabled = true;
    cfg.variable.engage_time_s = 0.001;
    cfg.variable.release_time_s = 0.1;
    cfg.variable.soft_velocity = { 0.2 };
    cfg.variable.soft_velocity_ratio = 0.9; // would start much later if used
    cfg.variable.max_damping_multiplier = 5.0;
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { 2.0 };
    input.contact_confidence = { 1.0 };
    input.dt = 0.002;
    JointAdmittanceOutput out;
    bool wall_seen = false;
    for(int k = 0; k < 500; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
        if(std::abs(out.delta_q_dot[0]) > 0.25 && out.effective_damping[0] > cfg.damping[0]) {
            wall_seen = true;
            break;
        }
    }
    EXPECT_TRUE(wall_seen);
    EXPECT_LT(std::abs(out.delta_q_dot[0]), cfg.max_delta_q_dot[0]);
}
