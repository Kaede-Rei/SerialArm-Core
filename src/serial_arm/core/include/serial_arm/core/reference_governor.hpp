#pragma once

#include "serial_arm/core/safety.hpp"

namespace serial_arm {

enum class ReferenceGovernorErr { INVALID_INPUT, INFEASIBLE };

struct ReferenceGovernorInput {
    JointCtrlCmd requested;
    JointState measured;
    JointCtrlCmd previous;
    JointVector requested_acceleration;
    std::vector<JointVector> mass_matrix;
    JointVector nonlinear;
    JointVector external_torque;
    JointLimitCfg limits;
    double dt{0.0};
    bool model_admittance{false};
};

struct ReferenceGovernorOutput {
    JointCtrlCmd accepted;
    JointVector acceleration;
    JointVector predicted_total_torque;
    std::vector<std::uint8_t> reference_limited;
    std::vector<std::uint8_t> torque_limited;
};

// Project one backward-Euler reference step onto motion and coupled torque
// constraints. Force limits include the local MIT PD contribution, and any
// acceleration change is reflected in the model feedforward before dispatch.
tl::expected<ReferenceGovernorOutput, ReferenceGovernorErr>
govern_reference(const ReferenceGovernorInput& input);

} // namespace serial_arm
