#include "serial_arm/dynamics/gravity_correction.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace serial_arm {


GravityCorrectionPackage load_gravity_correction_package(
    const std::string& path,
    const std::vector<std::string>& expected_joint_names,
    const std::string& expected_urdf_path) {
    const YAML::Node root = YAML::LoadFile(path);
    if(!root || !root.IsMap() || !root["schema"] || root["schema"].as<std::string>() != "serial_arm_gravity_correction") {
        throw std::runtime_error("invalid gravity correction schema");
    }
    GravityCorrectionPackage package;
    package.urdf_fingerprint = root["urdf_fingerprint"] ? root["urdf_fingerprint"].as<std::string>() : std::string{ };
    if(package.urdf_fingerprint.empty() || package.urdf_fingerprint != model_calibration_file_fingerprint(expected_urdf_path)) {
        throw std::runtime_error("gravity correction URDF fingerprint mismatch");
    }
    if(!root["joint_names"] || !root["joint_names"].IsSequence()) throw std::runtime_error("gravity correction joint_names are missing");
    package.joint_names = root["joint_names"].as<std::vector<std::string>>();
    if(package.joint_names != expected_joint_names) throw std::runtime_error("gravity correction joint order mismatch");
    package.static_pass = root["static_pass"] && root["static_pass"].as<bool>();
    package.friction_pass = root["friction_pass"] && root["friction_pass"].as<bool>();
    if(!package.static_pass) throw std::runtime_error("gravity correction static validation did not pass");
    const YAML::Node moments = root["first_moments"];
    if(!moments || !moments.IsMap()) throw std::runtime_error("gravity correction first_moments are missing");
    const YAML::Node masses = root["candidate_masses"];
    if(masses && !masses.IsMap()) throw std::runtime_error("gravity correction masses must be a mapping");
    for(auto it = moments.begin(); it != moments.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const JointVector values = it->second.as<JointVector>();
        if(values.size() != 3 || !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
            throw std::runtime_error("invalid gravity correction first moment for " + name);
        }
        double mass = 0.0;
        if(masses && masses[name]) {
            mass = masses[name].as<double>();
            if(!std::isfinite(mass) || mass <= 0.0)
                throw std::runtime_error("invalid candidate mass for " + name);
        }
        package.first_moments.push_back(GravityFirstMoment{ name, Eigen::Vector3d(values[0], values[1], values[2]), mass });
    }
    if(masses) {
        for(auto it = masses.begin(); it != masses.end(); ++it) {
            const auto name = it->first.as<std::string>();
            if(!moments[name]) throw std::runtime_error("candidate mass missing matching first moment: " + name);
        }
    }
    auto required_vector = [&](const YAML::Node& parent, const char* key) {
        if(!parent || !parent[key]) throw std::runtime_error(std::string("gravity correction missing ") + key);
        JointVector values = parent[key].as<JointVector>();
        if(values.size() != expected_joint_names.size() || !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
            throw std::runtime_error(std::string("invalid gravity correction vector ") + key);
        }
        return values;
        };
    const YAML::Node residual = root["residual"];
    package.torque_bias = required_vector(residual, "torque_bias");
    package.torque_threshold = required_vector(residual, "torque_threshold");
    package.friction.enabled = false;
    if(package.friction_pass && residual["friction"] && residual["friction"].IsMap()) {
        const YAML::Node friction = residual["friction"];
        package.friction.enabled = true;
        package.friction.velocity_transition = friction["velocity_transition"] ? friction["velocity_transition"].as<double>() : 0.03;
        package.friction.positive_coulomb = required_vector(friction, "positive_coulomb");
        package.friction.positive_viscous = required_vector(friction, "positive_viscous");
        package.friction.negative_coulomb = required_vector(friction, "negative_coulomb");
        package.friction.negative_viscous = required_vector(friction, "negative_viscous");
    }
    if(std::any_of(package.torque_threshold.begin(), package.torque_threshold.end(), [](double x) { return x < 0.0; }) ||
        (package.friction.enabled && (!std::isfinite(package.friction.velocity_transition) || package.friction.velocity_transition <= 0.0)))
        throw std::runtime_error("invalid gravity correction residual limits");
    return package;
}


tl::expected<void, DynamicsErr> configure_robot_dynamics(RobotCfg& cfg, Dynamics& dynamics) {
    try {
        if(!cfg.dynamics.gravity_correction_path.empty()) {
            if(cfg.runtime.model_feedforward_mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS ||
                cfg.capability.admittance.observer.mode == AdmittanceObserverMode::FULL_ID)
                return tl::make_unexpected(DynamicsErr::INVALID_CFG);
            const auto package = load_gravity_correction_package(
                cfg.dynamics.gravity_correction_path, cfg.joint_names, cfg.dynamics.urdf_path);
            const auto configured = dynamics.configure(cfg.dynamics);
            if(!configured) return configured;
            cfg.dynamics.gravity_scale.assign(cfg.joint_names.size(), 1.0);
            cfg.capability.admittance.calibration.torque_bias = package.torque_bias;
            cfg.capability.admittance.calibration.torque_threshold = package.torque_threshold;
            cfg.capability.admittance.calibration.friction = package.friction;
            return {};
        }
        return dynamics.configure(cfg.dynamics);
    }
    catch(const std::exception&) {
        return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }
}

} // namespace serial_arm
