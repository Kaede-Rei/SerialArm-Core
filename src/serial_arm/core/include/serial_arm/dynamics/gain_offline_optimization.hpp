#pragma once

#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include <functional>

namespace serial_arm::servo_tuning {

enum class OfflineGainUse { Tracking, RigidHold, Compliant, Drag };

struct OfflineGainStep {
    double time_s{0};
    Eigen::VectorXd reference,reference_velocity,actual,velocity,torque,feedforward,gravity;
    Eigen::VectorXd measured_inverse_dynamics,measured_gravity;
    Eigen::VectorXd reference_acceleration,coriolis_force;
    Eigen::MatrixXd mass,coriolis,gravity_gradient;
    Eigen::MatrixXd feedforward_gradient;
};

struct OfflineGainPath {
    std::vector<OfflineGainStep> steps;
    Eigen::VectorXd initial_position,initial_velocity,lower,upper;
    double period_s{.005};
};

struct OfflineGainLimits {
    std::vector<ModelAxisLimits> axes;
    Eigen::VectorXd lower,upper,velocity,effort,torque_step,position_step,velocity_step;
    std::vector<double> kp_ceiling,kd_ceiling;
};

struct OfflineGainModel {
    Eigen::VectorXd coulomb,viscous,bias,rotor,gravity_error,prediction_rms_rad;
    bool available{false};
    std::size_t fitted_samples{0},refinement_evaluations{0};
    // A finite same-record prediction is usable for local candidate design;
    // it does not identify actuator timing or establish physical stability.
    bool stability_identified{false};
    std::vector<std::size_t> axis_fitted_samples;
    std::string reason{"dynamic_model_not_fitted"};
};

struct OfflineGainSimulationOptions {
    // Zero preserves the original continuous-feedback integration assumption.
    // Positive values are explicit, unmeasured inner-controller scenarios;
    // none of these values is inferred from host feedback age.
    double inner_pd_period_s{0};
    double position_feedback_delay_s{0},velocity_feedback_delay_s{0};
    bool quantize_position_feedback{false},quantize_velocity_feedback{false};
    bool include_fitted_rotor_inertia{true};
};

struct OfflineGainTrace {
    Eigen::MatrixXd position,velocity,torque,feedback;
    bool complete{false},safe{true};
};

using OfflineGainCheck=std::function<bool()>;
using OfflineGainDynamics=std::function<bool(const Eigen::VectorXd&,const Eigen::VectorXd&,
    const Eigen::VectorXd&,OfflineGainStep&)>;

struct OfflineGainEvaluation {
    double cost{std::numeric_limits<double>::infinity()},violation{std::numeric_limits<double>::infinity()};
    std::vector<double> peak_error_rad;
    bool complete{false},feasible{false};
    std::size_t simulation_scenarios{0};
    bool includes_urdf_only_inertia{false},stability_identified{false};
};

struct OfflineGainTrustRegion {Gains lower,upper;};

// A numerical search domain anchored to one original executable seed. It is
// not a user PD limit or permission to enlarge the domain after an iteration.
OfflineGainTrustRegion offline_gain_local_trust_region(OfflineGainUse use,const Gains& seed,
    const OfflineGainLimits& limits);

struct OfflineGainOptimization {
    Gains gains;
    OfflineGainEvaluation baseline,selected;
    std::size_t evaluations{0};
    double elapsed_s{0};
    bool available{false},changed{false},budget_exhausted{false};
    OfflineGainTrustRegion trust_region;
    std::string method{"original_seed_local_trust_region_protocol_grid_search_on_unvalidated_closed_loop_model"};
};

OfflineGainTrace simulate_offline_gain_path(const OfflineGainPath& path,const OfflineGainModel& model,
    const Gains& gains,const OfflineGainLimits& limits,const Eigen::MatrixXd& independent_input={},
    double actuator_lag_s=.001,double velocity_lag_s=.0005,double inertia_scale=1,
    unsigned subdivisions=10,const OfflineGainCheck& check={},const OfflineGainDynamics& dynamics={},
    const OfflineGainSimulationOptions& simulation_options={});

OfflineGainModel fit_offline_gain_model(const OfflineGainPath& measured,const Gains& actual_gains,
    const OfflineGainLimits& limits,double budget_s,const OfflineGainCheck& check={});

OfflineGainEvaluation evaluate_offline_gain_use(OfflineGainUse use,const OfflineGainPath& tracking,
    const OfflineGainPath& holding,const OfflineGainModel& model,const Gains& gains,
    const OfflineGainLimits& limits,const Requirements& requirements,const Eigen::VectorXd& amplitudes,
    const OfflineGainCheck& check={},bool perturbations=false,const OfflineGainDynamics& dynamics={},
    const OfflineGainSimulationOptions& simulation_options={});

OfflineGainOptimization optimize_offline_gain_use(OfflineGainUse use,const OfflineGainPath& tracking,
    const OfflineGainPath& holding,const OfflineGainModel& model,const Gains& seed,
    const std::vector<std::vector<double>>& mass_basis,const OfflineGainLimits& limits,
    const Requirements& requirements,const Eigen::VectorXd& amplitudes,double budget_s,
    const OfflineGainCheck& check={});

}
