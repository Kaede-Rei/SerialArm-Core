#pragma once

#include "serial_arm/core/types.hpp"

namespace serial_arm {

/**
 * Runtime state exchanged by interaction capability components
 *
 * This structure only describes controller state. It does not contain
 * task-level meaning or application constraints.
 */
struct InteractionState {
    JointVector tau_ext_hat{};
    JointVector delta_q{};
    JointVector delta_q_dot{};
    bool valid{false};
};

} // namespace serial_arm
