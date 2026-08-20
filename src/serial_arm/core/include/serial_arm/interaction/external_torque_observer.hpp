#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/interaction/torque_residual_observer.hpp"
#include "serial_arm/interaction/friction_residual_model.hpp"

namespace serial_arm {

enum class ExternalTorqueObserverErr {
    NOT_CONFIGURED,
    ALREADY_CONFIGURED,
    INVALID_CFG,
    INVALID_INPUT_SIZE,
    NON_FINITE_INPUT,
    INVALID_DT,
};

struct ExternalTorqueObserverCfg {
    std::size_t joints_count{ 0 };
    JointVector torque_bias;
    JointVector torque_threshold;
    FrictionResidualModelCfg friction;
};

struct ExternalTorqueEstimate {
    JointVector bias_compensated;
    JointVector friction_residual_hat;
    JointVector friction_compensated;
    JointVector tau_ext_hat;
    JointVector contact_confidence;                 ///< [0,1] 连续交互置信度，供 variable admittance 使用
    std::vector<std::uint8_t> threshold_active;
};

class ExternalTorqueObserver {
public:
    tl::expected<void, ExternalTorqueObserverErr> configure(const ExternalTorqueObserverCfg& cfg);
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(const TorqueResidualEstimate& residual);
    tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> update(
        const TorqueResidualEstimate& residual,
        const JointVector& joint_velocity,
        double dt = 0.005);
    void reset();
    bool is_configured() const noexcept;

private:
    ExternalTorqueObserverCfg cfg_;
    std::vector<std::int8_t> last_motion_direction_;
    JointVector static_friction_baseline_;
    std::vector<std::uint8_t> static_baseline_initialized_;
    std::vector<std::uint8_t> was_moving_;
    bool is_configured_{ false };
};

} // namespace serial_arm
