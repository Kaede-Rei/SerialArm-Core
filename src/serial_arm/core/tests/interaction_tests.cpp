#include "serial_arm/interaction/interaction_controller.hpp"
#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace {

using namespace serial_arm;

DynamicsState dynamics_state(std::size_t n) {
    DynamicsState state;
    state.tor.assign(n, 0.0);
    state.gravity.assign(n, 0.0);
    state.nonlinear.assign(n, 0.0);
    return state;
}

JointCtrlCmd joint_cmd(std::size_t n) {
    JointCtrlCmd cmd;
    cmd.pos.assign(n, 0.0);
    cmd.vel.assign(n, 0.0);
    cmd.tor.assign(n, 0.0);
    cmd.kp.assign(n, 1.0);
    cmd.kd.assign(n, 0.1);
    return cmd;
}

TorqueResidualEstimate residual_estimate(std::size_t n) {
    TorqueResidualEstimate estimate;
    estimate.gravity_residual.assign(n, 0.0);
    estimate.nonlinear_residual.assign(n, 0.0);
    estimate.gravity_residual_filtered.assign(n, 0.0);
    estimate.nonlinear_residual_filtered.assign(n, 0.0);
    return estimate;
}

JointAdmittanceControllerCfg admittance_cfg(std::size_t n) {
    JointAdmittanceControllerCfg cfg;
    cfg.joints_count = n;
    cfg.enabled.assign(n, 1);
    cfg.mass.assign(n, 1.0);
    cfg.damping.assign(n, 0.0);
    cfg.stiffness.assign(n, 0.0);
    cfg.max_delta_q.assign(n, 1.0);
    cfg.max_delta_q_dot.assign(n, 1.0);
    return cfg;
}

void expect_vector_near(const JointVector& actual, const JointVector& expected) {
    ASSERT_EQ(actual.size(), expected.size());
    for(std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], 1e-12);
    }
}

void expect_cmd_eq(const JointCtrlCmd& actual, const JointCtrlCmd& expected) {
    EXPECT_EQ(actual.pos, expected.pos);
    EXPECT_EQ(actual.vel, expected.vel);
    EXPECT_EQ(actual.tor, expected.tor);
    EXPECT_EQ(actual.kp, expected.kp);
    EXPECT_EQ(actual.kd, expected.kd);
}

} // namespace

TEST(TorqueResidualObserverTests, ComputesGravityAndNonlinearResiduals) {
    TorqueResidualObserver observer;
    TorqueResidualObserverCfg cfg;
    cfg.joints_count = 3;
    cfg.filter_alpha = 0.5;
    ASSERT_TRUE(observer.configure(cfg));

    DynamicsState state = dynamics_state(3);
    state.tor = { 1.0, -2.0, 0.5 };
    state.gravity = { 2.5, -1.5, 0.0 };
    state.nonlinear = { 3.0, -4.0, 1.5 };

    auto estimate = observer.update(state);
    ASSERT_TRUE(estimate);
    expect_vector_near(estimate->gravity_residual, { 1.5, 0.5, -0.5 });
    expect_vector_near(estimate->nonlinear_residual, { 2.0, -2.0, 1.0 });
    expect_vector_near(estimate->gravity_residual_filtered, { 1.5, 0.5, -0.5 });
    expect_vector_near(estimate->nonlinear_residual_filtered, { 2.0, -2.0, 1.0 });
}

TEST(TorqueResidualObserverTests, LowPassFiltersAfterFirstSampleAndResetClearsHistory) {
    TorqueResidualObserver observer;
    TorqueResidualObserverCfg cfg;
    cfg.joints_count = 2;
    cfg.filter_alpha = 0.25;
    ASSERT_TRUE(observer.configure(cfg));

    DynamicsState first = dynamics_state(2);
    first.gravity = { 4.0, -2.0 };
    first.nonlinear = { 5.0, -1.0 };
    ASSERT_TRUE(observer.update(first));

    DynamicsState second = dynamics_state(2);
    second.gravity = { 8.0, 2.0 };
    second.nonlinear = { 1.0, 3.0 };
    auto filtered = observer.update(second);
    ASSERT_TRUE(filtered);
    expect_vector_near(filtered->gravity_residual_filtered, { 5.0, -1.0 });
    expect_vector_near(filtered->nonlinear_residual_filtered, { 4.0, 0.0 });

    observer.reset();
    auto reset = observer.update(second);
    ASSERT_TRUE(reset);
    expect_vector_near(reset->gravity_residual_filtered, { 8.0, 2.0 });
    expect_vector_near(reset->nonlinear_residual_filtered, { 1.0, 3.0 });
}

TEST(TorqueResidualObserverTests, RejectsInvalidConfiguration) {
    TorqueResidualObserver observer;

    TorqueResidualObserverCfg cfg;
    cfg.joints_count = 0;
    cfg.filter_alpha = 0.5;
    auto zero_result = observer.configure(cfg);
    ASSERT_FALSE(zero_result);
    EXPECT_EQ(zero_result.error(), TorqueResidualObserverErr::INVALID_CFG);

    cfg.joints_count = 1;
    cfg.filter_alpha = 0.0;
    auto zero_alpha_result = observer.configure(cfg);
    ASSERT_FALSE(zero_alpha_result);
    EXPECT_EQ(zero_alpha_result.error(), TorqueResidualObserverErr::INVALID_CFG);

    cfg.filter_alpha = 1.1;
    auto large_alpha_result = observer.configure(cfg);
    ASSERT_FALSE(large_alpha_result);
    EXPECT_EQ(large_alpha_result.error(), TorqueResidualObserverErr::INVALID_CFG);

    cfg.filter_alpha = std::numeric_limits<double>::quiet_NaN();
    auto nan_alpha_result = observer.configure(cfg);
    ASSERT_FALSE(nan_alpha_result);
    EXPECT_EQ(nan_alpha_result.error(), TorqueResidualObserverErr::INVALID_CFG);
}

TEST(TorqueResidualObserverTests, RejectsInvalidState) {
    TorqueResidualObserver observer;
    TorqueResidualObserverCfg cfg;
    cfg.joints_count = 2;
    ASSERT_TRUE(observer.configure(cfg));

    DynamicsState state = dynamics_state(2);
    state.tor.pop_back();
    auto tor_size = observer.update(state);
    ASSERT_FALSE(tor_size);
    EXPECT_EQ(tor_size.error(), TorqueResidualObserverErr::INVALID_INPUT_SIZE);

    state = dynamics_state(2);
    state.gravity.pop_back();
    auto gravity_size = observer.update(state);
    ASSERT_FALSE(gravity_size);
    EXPECT_EQ(gravity_size.error(), TorqueResidualObserverErr::INVALID_INPUT_SIZE);

    state = dynamics_state(2);
    state.nonlinear.pop_back();
    auto nonlinear_size = observer.update(state);
    ASSERT_FALSE(nonlinear_size);
    EXPECT_EQ(nonlinear_size.error(), TorqueResidualObserverErr::INVALID_INPUT_SIZE);

    state = dynamics_state(2);
    state.tor[0] = std::numeric_limits<double>::quiet_NaN();
    auto nan_state = observer.update(state);
    ASSERT_FALSE(nan_state);
    EXPECT_EQ(nan_state.error(), TorqueResidualObserverErr::NON_FINITE_INPUT);
}

TEST(InteractionControllerTests, PassesNominalCommandThroughAndReportsResidual) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.residual.joints_count = 2;
    cfg.residual.filter_alpha = 0.5;
    ASSERT_TRUE(controller.configure(cfg));

    DynamicsState state = dynamics_state(2);
    state.tor = { 1.0, 2.0 };
    state.gravity = { 3.0, 1.0 };
    state.nonlinear = { 4.0, 5.0 };

    JointCtrlCmd nominal = joint_cmd(2);
    nominal.pos = { 0.1, -0.2 };
    nominal.vel = { 0.3, -0.4 };
    nominal.tor = { 0.5, -0.6 };
    nominal.kp = { 7.0, 8.0 };
    nominal.kd = { 0.7, 0.8 };

    auto output = controller.update(InteractionInput{ state, nominal });
    ASSERT_TRUE(output);
    expect_cmd_eq(output->corrected_cmd, nominal);
    expect_vector_near(output->residual.gravity_residual, { 2.0, -1.0 });
    expect_vector_near(output->residual.nonlinear_residual, { 3.0, 3.0 });
}

TEST(InteractionControllerTests, ResidualMatchesDirectObserverAndResetClearsHistory) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.residual.joints_count = 1;
    cfg.residual.filter_alpha = 0.25;
    ASSERT_TRUE(controller.configure(cfg));

    TorqueResidualObserver observer;
    ASSERT_TRUE(observer.configure(cfg.residual));

    DynamicsState first = dynamics_state(1);
    first.gravity = { 4.0 };
    first.nonlinear = { 6.0 };
    ASSERT_TRUE(controller.update(InteractionInput{ first, joint_cmd(1) }));
    ASSERT_TRUE(observer.update(first));

    DynamicsState second = dynamics_state(1);
    second.gravity = { 8.0 };
    second.nonlinear = { 2.0 };
    auto controller_output = controller.update(InteractionInput{ second, joint_cmd(1) });
    auto observer_output = observer.update(second);
    ASSERT_TRUE(controller_output);
    ASSERT_TRUE(observer_output);
    expect_vector_near(controller_output->residual.gravity_residual_filtered, observer_output->gravity_residual_filtered);
    expect_vector_near(controller_output->residual.nonlinear_residual_filtered, observer_output->nonlinear_residual_filtered);

    controller.reset();
    auto reset_output = controller.update(InteractionInput{ second, joint_cmd(1) });
    ASSERT_TRUE(reset_output);
    expect_vector_near(reset_output->residual.gravity_residual_filtered, { 8.0 });
    expect_vector_near(reset_output->residual.nonlinear_residual_filtered, { 2.0 });
}

TEST(ExternalTorqueObserverTests, SelectsGravityFilteredResidual) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 3;
    cfg.source = ExternalTorqueSource::GRAVITY;
    ASSERT_TRUE(observer.configure(cfg));

    auto residual = residual_estimate(3);
    residual.gravity_residual_filtered = { 1.0, -2.0, 0.5 };
    residual.nonlinear_residual_filtered = { 4.0, 5.0, 6.0 };

    auto estimate = observer.update(residual);
    ASSERT_TRUE(estimate);
    expect_vector_near(estimate->tau_ext_hat, { 1.0, -2.0, 0.5 });
}

TEST(ExternalTorqueObserverTests, SelectsNonlinearFilteredResidual) {
    ExternalTorqueObserver observer;
    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 2;
    cfg.source = ExternalTorqueSource::NONLINEAR;
    ASSERT_TRUE(observer.configure(cfg));

    auto residual = residual_estimate(2);
    residual.gravity_residual_filtered = { 1.0, 2.0 };
    residual.nonlinear_residual_filtered = { -3.0, 4.0 };

    auto estimate = observer.update(residual);
    ASSERT_TRUE(estimate);
    expect_vector_near(estimate->tau_ext_hat, { -3.0, 4.0 });
}

TEST(ExternalTorqueObserverTests, RejectsInvalidConfigurationAndInput) {
    ExternalTorqueObserver observer;
    auto not_configured = observer.update(residual_estimate(1));
    ASSERT_FALSE(not_configured);
    EXPECT_EQ(not_configured.error(), ExternalTorqueObserverErr::NOT_CONFIGURED);

    ExternalTorqueObserverCfg cfg;
    cfg.joints_count = 0;
    cfg.source = ExternalTorqueSource::GRAVITY;
    auto zero_joints = observer.configure(cfg);
    ASSERT_FALSE(zero_joints);
    EXPECT_EQ(zero_joints.error(), ExternalTorqueObserverErr::INVALID_CFG);

    cfg.joints_count = 1;
    cfg.source = static_cast<ExternalTorqueSource>(99);
    auto invalid_source = observer.configure(cfg);
    ASSERT_FALSE(invalid_source);
    EXPECT_EQ(invalid_source.error(), ExternalTorqueObserverErr::INVALID_CFG);

    cfg.source = ExternalTorqueSource::GRAVITY;
    ASSERT_TRUE(observer.configure(cfg));
    auto repeat_configure = observer.configure(cfg);
    ASSERT_FALSE(repeat_configure);
    EXPECT_EQ(repeat_configure.error(), ExternalTorqueObserverErr::ALREADY_CONFIGURED);

    auto residual = residual_estimate(1);
    residual.gravity_residual_filtered.clear();
    auto invalid_size = observer.update(residual);
    ASSERT_FALSE(invalid_size);
    EXPECT_EQ(invalid_size.error(), ExternalTorqueObserverErr::INVALID_INPUT_SIZE);

    residual = residual_estimate(1);
    residual.gravity_residual_filtered[0] = std::numeric_limits<double>::infinity();
    auto non_finite = observer.update(residual);
    ASSERT_FALSE(non_finite);
    EXPECT_EQ(non_finite.error(), ExternalTorqueObserverErr::NON_FINITE_INPUT);
}

TEST(JointAdmittanceControllerTests, ZeroInputKeepsZeroOffsetAndSingleJointTorqueMoves) {
    JointAdmittanceController controller;
    ASSERT_TRUE(controller.configure(admittance_cfg(2)));

    JointAdmittanceInput zero;
    zero.tau_ext_hat = { 0.0, 0.0 };
    zero.dt = 0.1;
    auto zero_output = controller.update(zero);
    ASSERT_TRUE(zero_output);
    expect_vector_near(zero_output->delta_q, { 0.0, 0.0 });
    expect_vector_near(zero_output->delta_q_dot, { 0.0, 0.0 });

    JointAdmittanceInput input;
    input.tau_ext_hat = { 2.0, 0.0 };
    input.dt = 0.1;
    auto output = controller.update(input);
    ASSERT_TRUE(output);
    expect_vector_near(output->delta_q_dot, { 0.2, 0.0 });
    expect_vector_near(output->delta_q, { 0.02, 0.0 });
}

TEST(JointAdmittanceControllerTests, DisabledJointLimitsAndResetAreApplied) {
    JointAdmittanceControllerCfg cfg = admittance_cfg(2);
    cfg.enabled = { 1, 0 };
    cfg.max_delta_q = { 0.03, 1.0 };
    cfg.max_delta_q_dot = { 0.2, 1.0 };

    JointAdmittanceController controller;
    ASSERT_TRUE(controller.configure(cfg));

    JointAdmittanceInput input;
    input.tau_ext_hat = { 100.0, 100.0 };
    input.dt = 0.1;
    auto output = controller.update(input);
    ASSERT_TRUE(output);
    expect_vector_near(output->delta_q_dot, { 0.2, 0.0 });
    expect_vector_near(output->delta_q, { 0.02, 0.0 });

    output = controller.update(input);
    ASSERT_TRUE(output);
    expect_vector_near(output->delta_q_dot, { 0.2, 0.0 });
    expect_vector_near(output->delta_q, { 0.03, 0.0 });

    controller.reset();
    JointAdmittanceInput zero;
    zero.tau_ext_hat = { 0.0, 0.0 };
    zero.dt = 0.1;
    auto reset_output = controller.update(zero);
    ASSERT_TRUE(reset_output);
    expect_vector_near(reset_output->delta_q, { 0.0, 0.0 });
    expect_vector_near(reset_output->delta_q_dot, { 0.0, 0.0 });
}

TEST(JointAdmittanceControllerTests, RejectsInvalidConfigurationAndInput) {
    JointAdmittanceController controller;
    auto not_configured = controller.update(JointAdmittanceInput{ { 0.0 }, 0.1 });
    ASSERT_FALSE(not_configured);
    EXPECT_EQ(not_configured.error(), JointAdmittanceControllerErr::NOT_CONFIGURED);

    JointAdmittanceControllerCfg cfg = admittance_cfg(1);
    cfg.joints_count = 0;
    auto invalid_count = controller.configure(cfg);
    ASSERT_FALSE(invalid_count);
    EXPECT_EQ(invalid_count.error(), JointAdmittanceControllerErr::INVALID_CFG);

    cfg = admittance_cfg(1);
    cfg.mass[0] = 0.0;
    auto invalid_mass = controller.configure(cfg);
    ASSERT_FALSE(invalid_mass);
    EXPECT_EQ(invalid_mass.error(), JointAdmittanceControllerErr::INVALID_CFG);

    cfg = admittance_cfg(1);
    cfg.damping[0] = -1.0;
    auto invalid_damping = controller.configure(cfg);
    ASSERT_FALSE(invalid_damping);
    EXPECT_EQ(invalid_damping.error(), JointAdmittanceControllerErr::INVALID_CFG);

    cfg = admittance_cfg(1);
    cfg.stiffness[0] = -1.0;
    auto invalid_stiffness = controller.configure(cfg);
    ASSERT_FALSE(invalid_stiffness);
    EXPECT_EQ(invalid_stiffness.error(), JointAdmittanceControllerErr::INVALID_CFG);

    cfg = admittance_cfg(1);
    cfg.max_delta_q[0] = -1.0;
    auto invalid_limit = controller.configure(cfg);
    ASSERT_FALSE(invalid_limit);
    EXPECT_EQ(invalid_limit.error(), JointAdmittanceControllerErr::INVALID_CFG);

    cfg = admittance_cfg(1);
    ASSERT_TRUE(controller.configure(cfg));
    auto repeat_configure = controller.configure(cfg);
    ASSERT_FALSE(repeat_configure);
    EXPECT_EQ(repeat_configure.error(), JointAdmittanceControllerErr::ALREADY_CONFIGURED);

    auto invalid_size = controller.update(JointAdmittanceInput{ {}, 0.1 });
    ASSERT_FALSE(invalid_size);
    EXPECT_EQ(invalid_size.error(), JointAdmittanceControllerErr::INVALID_INPUT_SIZE);

    auto invalid_dt = controller.update(JointAdmittanceInput{ { 0.0 }, 0.0 });
    ASSERT_FALSE(invalid_dt);
    EXPECT_EQ(invalid_dt.error(), JointAdmittanceControllerErr::INVALID_DT);

    auto non_finite = controller.update(JointAdmittanceInput{ { std::numeric_limits<double>::quiet_NaN() }, 0.1 });
    ASSERT_FALSE(non_finite);
    EXPECT_EQ(non_finite.error(), JointAdmittanceControllerErr::NON_FINITE_INPUT);
}

TEST(InteractionControllerTests, AdmittanceDisabledKeepsPassthroughAndReportsZeroCorrection) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.residual.joints_count = 1;
    cfg.residual.filter_alpha = 1.0;
    cfg.admittance_enabled = false;
    ASSERT_TRUE(controller.configure(cfg));

    DynamicsState state = dynamics_state(1);
    state.gravity = { 3.0 };
    JointCtrlCmd nominal = joint_cmd(1);
    nominal.pos = { 0.4 };
    nominal.vel = { -0.2 };
    nominal.tor = { 0.3 };
    nominal.kp = { 4.0 };
    nominal.kd = { 0.5 };

    auto output = controller.update(InteractionInput{ state, nominal, 0.1 });
    ASSERT_TRUE(output);
    expect_cmd_eq(output->corrected_cmd, nominal);
    expect_vector_near(output->tau_ext_hat, { 0.0 });
    expect_vector_near(output->delta_q, { 0.0 });
    expect_vector_near(output->delta_q_dot, { 0.0 });
}

TEST(InteractionControllerTests, AdmittanceEnabledCorrectsPositionAndVelocityOnly) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.residual.joints_count = 1;
    cfg.residual.filter_alpha = 1.0;
    cfg.admittance_enabled = true;
    cfg.external_torque.joints_count = 1;
    cfg.external_torque.source = ExternalTorqueSource::GRAVITY;
    cfg.admittance = admittance_cfg(1);
    ASSERT_TRUE(controller.configure(cfg));

    DynamicsState state = dynamics_state(1);
    state.gravity = { 2.0 };
    JointCtrlCmd nominal = joint_cmd(1);
    nominal.pos = { 0.5 };
    nominal.vel = { 0.1 };
    nominal.tor = { 0.7 };
    nominal.kp = { 8.0 };
    nominal.kd = { 0.8 };

    auto output = controller.update(InteractionInput{ state, nominal, 0.1 });
    ASSERT_TRUE(output);
    expect_vector_near(output->tau_ext_hat, { 2.0 });
    expect_vector_near(output->delta_q_dot, { 0.2 });
    expect_vector_near(output->delta_q, { 0.02 });
    expect_vector_near(output->corrected_cmd.pos, { 0.52 });
    expect_vector_near(output->corrected_cmd.vel, { 0.3 });
    expect_vector_near(output->corrected_cmd.tor, nominal.tor);
    expect_vector_near(output->corrected_cmd.kp, nominal.kp);
    expect_vector_near(output->corrected_cmd.kd, nominal.kd);

    controller.reset();
    state.gravity = { 0.0 };
    auto reset_output = controller.update(InteractionInput{ state, nominal, 0.1 });
    ASSERT_TRUE(reset_output);
    expect_vector_near(reset_output->delta_q, { 0.0 });
    expect_vector_near(reset_output->delta_q_dot, { 0.0 });
}

TEST(InteractionControllerTests, FailedAdmittanceConfigurationCanBeRetried) {
    InteractionController controller;
    InteractionControllerCfg cfg;
    cfg.residual.joints_count = 1;
    cfg.residual.filter_alpha = 1.0;
    cfg.admittance_enabled = true;
    cfg.external_torque.joints_count = 1;
    cfg.external_torque.source = ExternalTorqueSource::GRAVITY;
    cfg.admittance = admittance_cfg(1);
    cfg.admittance.mass[0] = 0.0;

    auto invalid_result = controller.configure(cfg);
    ASSERT_FALSE(invalid_result);
    EXPECT_EQ(invalid_result.error(), InteractionControllerErr::INVALID_CFG);

    cfg.admittance.mass[0] = 1.0;
    ASSERT_TRUE(controller.configure(cfg));
}
