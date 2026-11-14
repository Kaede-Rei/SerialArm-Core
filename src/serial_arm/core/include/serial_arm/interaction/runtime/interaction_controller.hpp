#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/estimators/external_torque_observer.hpp"
#include "serial_arm/interaction/estimators/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/controllers/joint_admittance_controller.hpp"
#include "serial_arm/interaction/runtime/interaction_state.hpp"
#include "serial_arm/interaction/estimators/torque_residual_observer.hpp"

namespace serial_arm {

enum class InteractionControllerErr {
    NOT_CONFIGURED,
    ALREADY_CONFIGURED,
    INVALID_CFG,
    OBSERVER_FAILED,
    MOMENTUM_FAILED,
    EXTERNAL_FAILED,
    ADMITTANCE_FAILED,
};

struct InteractionControllerCfg {
    bool enabled{ false };
    GeneralizedMomentumObserverCfg momentum;
    ExternalTorqueObserverCfg external_torque;
    JointAdmittanceControllerCfg admittance;
};

struct InteractionInput {
    const JointVector& measured_torque;
    const JointVector& full_id_model_torque;
    const JointCtrlCmd& nominal_cmd;
    double dt{ 0.0 };
    JointVector min_delta_q;
    JointVector max_delta_q;
    JointVector min_delta_q_dot;
    JointVector max_delta_q_dot;
    JointVector measured_velocity;
    GeneralizedMomentumInput momentum_input;
    bool full_id_acceleration_valid{ false };
    bool new_measurement{ true };
    bool observation_valid{ true };
    double measurement_dt{ 0.0 };
    bool apply_admittance{ true };
    bool drag{ false };
};

struct InteractionOutput {
    InteractionState state;
    JointCtrlCmd corrected_cmd;
    TorqueResidualEstimate residual;                  ///< 当前 observer residual；兼容字段 residual_filtered 与 raw 相同
    JointVector full_id_residual_raw;                ///< FULL_ID model - measured 对照 residual
    JointVector bias_compensated;
    JointVector friction_residual_hat;
    JointVector friction_compensated;
    JointVector tau_ext_hat;
    std::vector<std::uint8_t> threshold_active;
    JointVector delta_q;
    JointVector delta_q_dot;
    JointVector delta_q_ddot;
    std::vector<std::uint8_t> delta_q_limited;
    std::vector<std::uint8_t> delta_q_dot_limited;
};

class InteractionController {
public:
    tl::expected<void, InteractionControllerErr> configure(const InteractionControllerCfg& cfg);
    tl::expected<InteractionOutput, InteractionControllerErr> update(const InteractionInput& input);
    tl::expected<void, InteractionControllerErr> reconfigure(const InteractionControllerCfg& cfg);
    tl::expected<void, InteractionControllerErr> accept_reference(const JointVector& delta_q, const JointVector& delta_q_dot);
    void reset_reference();
    void reset();
    bool is_configured() const noexcept;
    bool is_enabled() const noexcept;
    double parameter_update_progress() const noexcept;

private:
    GeneralizedMomentumObserver momentum_observer_;
    GeneralizedMomentumObserver raw_momentum_observer_;
    ExternalTorqueObserver net_external_torque_observer_;
    ExternalTorqueObserver external_torque_observer_;
    JointAdmittanceController admittance_controller_;
    InteractionControllerCfg cfg_;
    bool is_configured_{ false };
    InteractionOutput last_observation_;
    bool observation_initialized_{ false };
};

} // namespace serial_arm
