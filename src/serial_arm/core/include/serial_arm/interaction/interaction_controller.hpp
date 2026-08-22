#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/admittance_observer_mode.hpp"
#include "serial_arm/interaction/external_torque_observer.hpp"
#include "serial_arm/interaction/generalized_momentum_observer.hpp"
#include "serial_arm/interaction/joint_admittance_controller.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"

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
    AdmittanceObserverMode observer_mode{ AdmittanceObserverMode::FULL_ID };
    TorqueResidualObserverCfg residual;
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
};

struct InteractionOutput {
    JointCtrlCmd corrected_cmd;
    TorqueResidualEstimate residual;                  ///< 当前 observer 的 raw + filtered residual
    JointVector full_id_residual_raw;                ///< FULL_ID model - measured 对照 residual
    JointVector bias_compensated;
    JointVector friction_residual_hat;
    JointVector friction_compensated;
    JointVector tau_ext_hat;
    std::vector<std::uint8_t> threshold_active;
    JointVector delta_q;
    JointVector delta_q_dot;
    std::vector<std::uint8_t> delta_q_limited;
    std::vector<std::uint8_t> delta_q_dot_limited;
};

class InteractionController {
public:
    tl::expected<void, InteractionControllerErr> configure(const InteractionControllerCfg& cfg);
    tl::expected<InteractionOutput, InteractionControllerErr> update(const InteractionInput& input);
    void reset();
    bool is_configured() const noexcept;
    bool is_enabled() const noexcept;

private:
    TorqueResidualObserver residual_observer_;
    GeneralizedMomentumObserver momentum_observer_;
    ExternalTorqueObserver external_torque_observer_;
    JointAdmittanceController admittance_controller_;
    InteractionControllerCfg cfg_;
    bool is_configured_{ false };
};

} // namespace serial_arm
