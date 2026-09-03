#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"

#include <Eigen/Dense>

namespace serial_arm {

enum class TaskConstraintMode {
    ORIENTATION_ONLY,
    LATERAL_COMPLIANCE,
    BLOCKED_RETREAT,
};

enum class TaskConstraintProjectorErr {
    NOT_CONFIGURED,
    INVALID_CFG,
    INVALID_INPUT,
    PROJECTION_FAILED,
};

struct TaskConstraintProjectorCfg {
    std::size_t joints_count{ 0 };
    bool enabled{ false };
    TaskConstraintMode mode{ TaskConstraintMode::ORIENTATION_ONLY };
    double damping{ 0.0 };
    double nominal_translation_epsilon{ 1.0e-6 };
};

struct TaskConstraintProjectorInput {
    const JointVector& delta_q_raw;
    const JointVector& delta_q_dot_raw;
    const Eigen::MatrixXd& tool_jacobian;
    const JointVector& dq_nominal;
};

struct TaskConstraintProjectorOutput {
    JointVector delta_q_cmd;
    JointVector delta_q_dot_cmd;
};

class TaskConstraintProjector {
public:
    tl::expected<void, TaskConstraintProjectorErr> configure(const TaskConstraintProjectorCfg& cfg);
    tl::expected<TaskConstraintProjectorOutput, TaskConstraintProjectorErr> project(
        const TaskConstraintProjectorInput& input) const;
    bool is_configured() const noexcept;
    bool is_enabled() const noexcept;
    const TaskConstraintProjectorCfg& get_cfg() const noexcept;

private:
    TaskConstraintProjectorCfg cfg_;
    bool is_configured_{ false };
};

} // namespace serial_arm
