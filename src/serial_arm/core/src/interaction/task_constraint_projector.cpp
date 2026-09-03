#include "serial_arm/interaction/task_constraint_projector.hpp"

#include <algorithm>
#include <cmath>

namespace serial_arm {
namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool finite_matrix(const Eigen::MatrixXd& matrix) {
    for(Eigen::Index row = 0; row < matrix.rows(); ++row) {
        for(Eigen::Index col = 0; col < matrix.cols(); ++col) {
            if(!std::isfinite(matrix(row, col))) return false;
        }
    }
    return true;
}

JointVector eigen_to_joint_vector(const Eigen::VectorXd& values) {
    JointVector result(static_cast<std::size_t>(values.size()), 0.0);
    for(Eigen::Index i = 0; i < values.size(); ++i) result[static_cast<std::size_t>(i)] = values(i);
    return result;
}

Eigen::VectorXd joint_vector_to_eigen(const JointVector& values) {
    Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
    for(std::size_t i = 0; i < values.size(); ++i) result(static_cast<Eigen::Index>(i)) = values[i];
    return result;
}

bool valid_projector_cfg(const TaskConstraintProjectorCfg& cfg) {
    return cfg.joints_count > 0 &&
        std::isfinite(cfg.damping) &&
        cfg.damping >= 0.0 &&
        std::isfinite(cfg.nominal_translation_epsilon) &&
        cfg.nominal_translation_epsilon > 0.0;
}

bool valid_input(const TaskConstraintProjectorCfg& cfg, const TaskConstraintProjectorInput& input) {
    const auto n = cfg.joints_count;
    if(input.delta_q_raw.size() != n ||
        input.delta_q_dot_raw.size() != n ||
        input.dq_nominal.size() != n) {
        return false;
    }
    if(input.tool_jacobian.rows() != 6 || input.tool_jacobian.cols() != static_cast<Eigen::Index>(n)) {
        return false;
    }
    return finite_vector(input.delta_q_raw) &&
        finite_vector(input.delta_q_dot_raw) &&
        finite_vector(input.dq_nominal) &&
        finite_matrix(input.tool_jacobian);
}

tl::expected<Eigen::VectorXd, TaskConstraintProjectorErr> project_vector(
    const Eigen::MatrixXd& a,
    const Eigen::VectorXd& raw,
    double damping)
{
    if(a.rows() == 0) return raw;

    Eigen::MatrixXd regularized = a * a.transpose();
    regularized.diagonal().array() += damping * damping;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(regularized);
    if(ldlt.info() != Eigen::Success) {
        return tl::make_unexpected(TaskConstraintProjectorErr::PROJECTION_FAILED);
    }

    const Eigen::VectorXd constrained_component = a.transpose() * ldlt.solve(a * raw);
    if(ldlt.info() != Eigen::Success || !constrained_component.allFinite()) {
        return tl::make_unexpected(TaskConstraintProjectorErr::PROJECTION_FAILED);
    }

    Eigen::VectorXd projected = raw - constrained_component;
    if(!projected.allFinite()) return tl::make_unexpected(TaskConstraintProjectorErr::PROJECTION_FAILED);
    return projected;
}

Eigen::MatrixXd orientation_constraint(const Eigen::MatrixXd& tool_jacobian) {
    return tool_jacobian.block(3, 0, 3, tool_jacobian.cols());
}

Eigen::MatrixXd lateral_constraint(
    const Eigen::MatrixXd& tool_jacobian,
    const JointVector& dq_nominal,
    double nominal_translation_epsilon)
{
    const Eigen::MatrixXd jv = tool_jacobian.block(0, 0, 3, tool_jacobian.cols());
    const Eigen::MatrixXd jw = orientation_constraint(tool_jacobian);
    const Eigen::VectorXd dq = joint_vector_to_eigen(dq_nominal);
    const Eigen::Vector3d nominal_translation = jv * dq;
    const double speed = nominal_translation.norm();
    if(speed <= nominal_translation_epsilon) return jw;

    Eigen::MatrixXd a(4, tool_jacobian.cols());
    a.block(0, 0, 3, tool_jacobian.cols()) = jw;
    a.row(3) = (nominal_translation / speed).transpose() * jv;
    return a;
}

} // namespace

tl::expected<void, TaskConstraintProjectorErr> TaskConstraintProjector::configure(
    const TaskConstraintProjectorCfg& cfg)
{
    if(!valid_projector_cfg(cfg)) return tl::make_unexpected(TaskConstraintProjectorErr::INVALID_CFG);
    cfg_ = cfg;
    is_configured_ = true;
    return {};
}

tl::expected<TaskConstraintProjectorOutput, TaskConstraintProjectorErr> TaskConstraintProjector::project(
    const TaskConstraintProjectorInput& input) const
{
    if(!is_configured_) return tl::make_unexpected(TaskConstraintProjectorErr::NOT_CONFIGURED);
    if(!valid_input(cfg_, input)) return tl::make_unexpected(TaskConstraintProjectorErr::INVALID_INPUT);

    if(!cfg_.enabled || cfg_.mode == TaskConstraintMode::BLOCKED_RETREAT) {
        return TaskConstraintProjectorOutput{ input.delta_q_raw, input.delta_q_dot_raw };
    }

    Eigen::MatrixXd a;
    switch(cfg_.mode) {
        case TaskConstraintMode::ORIENTATION_ONLY:
            a = orientation_constraint(input.tool_jacobian);
            break;
        case TaskConstraintMode::LATERAL_COMPLIANCE:
            a = lateral_constraint(input.tool_jacobian, input.dq_nominal, cfg_.nominal_translation_epsilon);
            break;
        case TaskConstraintMode::BLOCKED_RETREAT:
            break;
    }

    const auto delta_q = project_vector(a, joint_vector_to_eigen(input.delta_q_raw), cfg_.damping);
    if(!delta_q) return tl::make_unexpected(delta_q.error());
    const auto delta_q_dot = project_vector(a, joint_vector_to_eigen(input.delta_q_dot_raw), cfg_.damping);
    if(!delta_q_dot) return tl::make_unexpected(delta_q_dot.error());

    return TaskConstraintProjectorOutput{
        eigen_to_joint_vector(*delta_q),
        eigen_to_joint_vector(*delta_q_dot),
    };
}

bool TaskConstraintProjector::is_configured() const noexcept {
    return is_configured_;
}

bool TaskConstraintProjector::is_enabled() const noexcept {
    return is_configured_ && cfg_.enabled;
}

const TaskConstraintProjectorCfg& TaskConstraintProjector::get_cfg() const noexcept {
    return cfg_;
}

} // namespace serial_arm
