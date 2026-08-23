#include <gtest/gtest.h>

#include "serial_arm/interaction/admittance_calibration.hpp"
#include "serial_arm/interaction/estimators/external_torque_observer.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/runtime/interaction_controller.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/estimators/torque_residual_observer.hpp"

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
    cfg.residual = TorqueResidualObserverCfg{ 1 };
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

TEST(TorqueResidualObserverTests, ComputesModelTorqueMinusMeasuredTorqueWithoutExtraFiltering) {
    TorqueResidualObserver observer;
    ASSERT_TRUE(observer.configure(TorqueResidualObserverCfg{ 1 }));

    auto first = observer.update(JointVector{ 2.0 }, JointVector{ 1.0 });
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->residual[0], -1.0);
    EXPECT_DOUBLE_EQ(first->residual_filtered[0], -1.0);

    auto second = observer.update(JointVector{ 0.0 }, JointVector{ 1.0 });
    ASSERT_TRUE(second);
    EXPECT_DOUBLE_EQ(second->residual[0], 1.0);
    EXPECT_DOUBLE_EQ(second->residual_filtered[0], 1.0);
}

TEST(InteractionControllerTests, AppliesTorqueBiasAfterDirectResidual) {
    auto cfg = enabled_cfg();
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
    EXPECT_DOUBLE_EQ(output->residual.residual_filtered[0], 0.5);
    EXPECT_DOUBLE_EQ(output->bias_compensated[0], 0.3);
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
    // Threshold is derived directly from the static residual envelope.
    EXPECT_NEAR(result->residual_p99[0], 0.01, 1e-12);
    // The synthetic feedback occupies two repeated levels 0.02 Nm apart, so the
    // quantization guard reserves one further level beyond the 0.01 Nm envelope.
    EXPECT_NEAR(result->torque_threshold[0], 0.03 * (1.0 + 1e-6), 1e-12);
}

TEST(AdmittanceStaticCalibrationTests, AllowsGravityScaleAboveOneWhenModelUnderestimatesLoad) {
    std::vector<AdmittanceStaticPoseSamples> poses(3);
    const std::vector<double> gravity_values{ -2.0, 0.0, 2.0 };
    for(std::size_t p = 0; p < poses.size(); ++p) {
        const double g = gravity_values[p];
        for(int k = 0; k < 10; ++k) {
            const double measured = 1.4 * g - 0.15;
            poses[p].samples.push_back(AdmittanceStaticSample{ JointVector{ g }, JointVector{ measured } });
        }
    }

    AdmittanceStaticCalibrationCfg cfg;
    cfg.joints_count = 1;
    cfg.fallback_gravity_scale = { 1.0 };
    cfg.gravity_observability_span = 0.25;

    auto result = calibrate_admittance_static(poses, cfg);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->gravity_scale_observable, std::vector<std::uint8_t>{ 1 });
    EXPECT_NEAR(result->gravity_scale[0], 1.4, 1e-12);
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

TEST(AdmittanceStaticCalibrationTests, ValidationTreatsSingleFrameMaxOutlierAsWarningWhenP99Passes) {
    AdmittanceStaticPoseSamples pose;
    for(int k = 0; k < 199; ++k) {
        pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.04 } });
    }
    pose.samples.push_back(AdmittanceStaticSample{ JointVector{ 0.0 }, JointVector{ -0.20 } });

    AdmittanceStaticValidationCfg cfg;
    cfg.joints_count = 1;
    cfg.gravity_scale = { 1.0 };
    cfg.torque_bias = { 0.0 };
    cfg.torque_threshold = { 0.05 };

    auto result = evaluate_admittance_static_validation({ pose }, cfg);
    ASSERT_TRUE(result);
    EXPECT_LT(result->residual_p99[0], cfg.torque_threshold[0]);
    EXPECT_GT(result->residual_max[0], result->guarded_max_limit[0]);
    EXPECT_EQ(result->pass, std::vector<std::uint8_t>{ 1 });
}

TEST(ExternalTorqueObserverTests, AppliesBiasAndContinuousDeadband) {
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
    EXPECT_EQ(below->threshold_active[0], 1);

    // bias compensated = 0.45, continuous deadband removes threshold 0.30
    residual.residual = { 0.65 };
    residual.residual_filtered = { 0.65 };
    auto above = observer.update(residual);
    ASSERT_TRUE(above);
    EXPECT_NEAR(above->tau_ext_hat[0], 0.15, 1e-12);
    EXPECT_EQ(above->threshold_active[0], 0);

    residual.residual = { 0.9 };
    residual.residual_filtered = { 0.9 };
    auto large = observer.update(residual);
    ASSERT_TRUE(large);
    EXPECT_NEAR(large->tau_ext_hat[0], 0.4, 1e-12);
    EXPECT_EQ(large->threshold_active[0], 0);
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
    // residual = gravity - measured = -0.8; bias compensation gives -1.0; deadband 0.3 leaves -0.7.
    EXPECT_NEAR(output->bias_compensated[0], -1.0, 1e-12);
    EXPECT_NEAR(output->tau_ext_hat[0], -0.7, 1e-12);
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

TEST(ExternalTorqueObserverTests, RampsDynamicFrictionContinuouslyUpToVelocityTransition) {
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

    TorqueResidualEstimate residual;
    residual.residual = { -0.15 };
    residual.residual_filtered = { -0.15 };

    const auto stopped = observer.update(residual, JointVector{ 0.0 });
    ASSERT_TRUE(stopped);
    EXPECT_NEAR(stopped->friction_residual_hat[0], 0.0, 1e-12);

    const auto halfway = observer.update(residual, JointVector{ 0.015 });
    ASSERT_TRUE(halfway);
    EXPECT_NEAR(halfway->friction_residual_hat[0], -0.10, 1e-12);

    const auto full = observer.update(residual, JointVector{ 0.03 });
    ASSERT_TRUE(full);
    EXPECT_NEAR(full->friction_residual_hat[0], -0.20, 1e-12);
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

TEST(ExternalTorqueObserverTests, ZeroSpeedUsesBiasAndDeadbandWithoutFrictionStateMachine) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 1;
    cfg.torque_bias = { -0.12 };
    cfg.torque_threshold = { 0.05 };
    cfg.friction.enabled = true;
    cfg.friction.velocity_transition = 0.03;
    cfg.friction.positive_coulomb = { -0.20 };
    cfg.friction.positive_viscous = { 0.0 };
    cfg.friction.negative_coulomb = { 0.20 };
    cfg.friction.negative_viscous = { 0.0 };
    ASSERT_TRUE(observer.configure(cfg));

    TorqueResidualEstimate residual;
    residual.residual = { -0.12 };
    residual.residual_filtered = { -0.12 };
    auto quiet = observer.update(residual, JointVector{ 0.0 });
    ASSERT_TRUE(quiet);
    EXPECT_NEAR(quiet->bias_compensated[0], 0.0, 1e-12);
    EXPECT_NEAR(quiet->friction_residual_hat[0], 0.0, 1e-12);
    EXPECT_NEAR(quiet->tau_ext_hat[0], 0.0, 1e-12);

    residual.residual = { -0.27 };
    residual.residual_filtered = { -0.27 };
    auto external = observer.update(residual, JointVector{ 0.0 });
    ASSERT_TRUE(external);
    EXPECT_NEAR(external->bias_compensated[0], -0.15, 1e-12);
    EXPECT_NEAR(external->tau_ext_hat[0], -0.10, 1e-12);
}

TEST(ExternalTorqueObserverTests, MovingFrictionSubtractionPreservesOpposingExternalTorque) {
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

    // positive motion friction is -0.20 while an opposing external torque is +0.35
    // observed residual is therefore +0.15 and full model subtraction must recover +0.35
    TorqueResidualEstimate residual;
    residual.residual = { 0.15 };
    residual.residual_filtered = { 0.15 };
    const auto estimate = observer.update(residual, JointVector{ 0.2 });
    ASSERT_TRUE(estimate);
    EXPECT_NEAR(estimate->friction_residual_hat[0], -0.20, 1e-12);
    EXPECT_NEAR(estimate->friction_compensated[0], 0.35, 1e-12);
    EXPECT_NEAR(estimate->tau_ext_hat[0], 0.35, 1e-12);
}

TEST(JointAdmittanceControllerTests, FixedMDKReturnsDisplacementToZeroAfterExternalTorqueRemoval) {
    JointAdmittanceController controller;
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = 1;
    cfg.enabled = { 1 };
    cfg.mass = { 0.5 };
    cfg.damping = { 4.0 };
    cfg.stiffness = { 8.0 };
    cfg.max_delta_q = { 1.0 };
    cfg.max_delta_q_dot = { 2.0 };
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { 0.6 };
    input.dt = 0.005;
    JointAdmittanceOutput out;
    for(int k = 0; k < 200; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
    }
    const double displaced = out.delta_q[0];
    EXPECT_GT(displaced, 0.03);

    input.tau_ext_hat = { 0.0 };
    for(int k = 0; k < 600; ++k) {
        auto step = controller.update(input);
        ASSERT_TRUE(step);
        out = *step;
    }
    EXPECT_LT(std::abs(out.delta_q[0]), 1e-3);
    EXPECT_LT(std::abs(out.delta_q_dot[0]), 1e-3);
}

