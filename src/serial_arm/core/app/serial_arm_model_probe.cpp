#include "serial_arm/config/config.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"
#include "serial_arm/dynamics/adaptive_gain_tuning.hpp"
#include "serial_arm/dynamics/urdf_nominal_gain_design.hpp"
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string json_escape(const std::string& value) {
    std::ostringstream stream;
    for(const unsigned char ch : value) {
        switch(ch) {
        case '\\': stream << "\\\\"; break;
        case '"': stream << "\\\""; break;
        case '\n': stream << "\\n"; break;
        case '\r': stream << "\\r"; break;
        case '\t': stream << "\\t"; break;
        default:
            if(ch < 0x20) {
                stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch)
                       << std::dec << std::setfill(' ');
            }
            else stream << static_cast<char>(ch);
        }
    }
    return stream.str();
}

void print_string(std::ostream& out, const std::string& value) {
    out << '"' << json_escape(value) << '"';
}

void print_vector(std::ostream& out, const serial_arm::JointVector& values) {
    out << '[';
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i) out << ',';
        out << std::setprecision(17) << values[i];
    }
    out << ']';
}

void print_nominal_modes(std::ostream& out,
    const serial_arm::servo_tuning::UrdfNominalModalDiagnostics& modes) {
    const auto finite_vector=[&](const std::vector<double>& values) {
        out<<'[';
        for(std::size_t i=0;i<values.size();++i){if(i)out<<',';
            if(std::isfinite(values[i]))out<<values[i];else out<<"null";}
        out<<']';
    };
    out<<"{\"computed\":true,\"measured\":false,\"gain_basis\":\"unconstrained_nominal_before_protocol_encoding\""
       <<",\"model_definition\":\"M(q0)*qdd+diag(kd)*qd+diag(kp)*q=0\""
       <<",\"frequency_definition\":\"undamped_generalized_eigenfrequencies_of_K_and_M_not_measured_bandwidth\""
       <<",\"damping_definition\":\"phi_transpose_D_phi_over_2omega_phi_transpose_M_phi\""
       <<",\"damping_projection_is_exact\":"<<(modes.damping_projection_is_exact?"true":"false")
       <<",\"nonproportional_damping_interpretation\":\"modal_projection_only_not_independent_damped_modes\""
       <<",\"zero_stiffness_damping_ratio\":\"undefined_null\",\"nominal_mode_frequency_hz\":";
    finite_vector(modes.nominal_mode_frequency_hz);
    out<<",\"nominal_mode_damping_ratio\":";finite_vector(modes.nominal_mode_damping_ratio);
    out<<",\"scope\":\"ideal_local_rigid_body_model_excludes_actuator_delay_filtering_friction_and_flexible_modes\"}";
}

void print_nominal_damping_design(std::ostream& out,
    const serial_arm::servo_tuning::UrdfNominalGainDesign& design) {
    out<<",\"damping_design_method\":";print_string(out,design.damping_design_method);
    out<<",\"damping_interface_projection\":";print_string(out,design.damping_interface_projection);
    out<<",\"damping_design_reference\":\"https://www.robotic.dlr.de/fileadmin/robotic/ott/papers/icra2004_II.pdf\""
       <<",\"damping_design_equation\":\"K*Phi=M*Phi*Omega^2;Phi^T*M*Phi=I;D_full=M*Phi*(2*zeta*Omega)*Phi^T*M;MIT_D=diag(D_full)\""
       <<",\"full_damping_matrix_role\":\"diagnostic_nominal_design_not_executed_cross_axis_velocity_terms\""
       <<",\"full_damping_modal_ratio_target\":"<<design.damping_ratio
       <<",\"damping_stiffness_basis\":";print_vector(out,design.unconstrained_kp);
    out<<",\"full_damping_matrix\":[";
    for(std::size_t i=0;i<design.full_damping_matrix.size();++i){if(i)out<<',';print_vector(out,design.full_damping_matrix[i]);}
    out<<']';
}

void print_vec3(std::ostream& out, const Eigen::Vector3d& values) {
    out << '[' << std::setprecision(17) << values.x() << ',' << values.y() << ',' << values.z() << ']';
}

void print_matrix3(std::ostream& out, const Eigen::Matrix3d& matrix) {
    out << '[';
    for(int row = 0; row < 3; ++row) {
        if(row) out << ',';
        out << '[';
        for(int col = 0; col < 3; ++col) {
            if(col) out << ',';
            out << std::setprecision(17) << matrix(row, col);
        }
        out << ']';
    }
    out << ']';
}

void print_matrix(std::ostream& out, const Eigen::MatrixXd& matrix) {
    out << '[';
    for(Eigen::Index row = 0; row < matrix.rows(); ++row) {
        if(row) out << ',';
        out << '[';
        for(Eigen::Index col = 0; col < matrix.cols(); ++col) {
            if(col) out << ',';
            out << std::setprecision(17) << matrix(row, col);
        }
        out << ']';
    }
    out << ']';
}

serial_arm::JointVector parse_positions(const std::string& value) {
    serial_arm::JointVector positions;
    if(value.empty()) return positions;
    std::stringstream stream(value);
    std::string item;
    while(std::getline(stream, item, ',')) {
        std::size_t parsed = 0;
        const double number = std::stod(item, &parsed);
        if(parsed != item.size() || !std::isfinite(number)) throw std::runtime_error("invalid position value");
        positions.push_back(number);
    }
    return positions;
}

void print_error(const std::string& message) {
    std::cout << "{\"ok\":false,\"error\":\"" << json_escape(message) << "\"}\n" << std::flush;
}
struct GainDesignPath {std::vector<serial_arm::JointVector> positions;double dt{0};};
GainDesignPath read_gain_design_path(const std::string& file,const std::vector<std::string>& joints) {
    GainDesignPath out;if(file.empty())return out;std::ifstream input(file);std::string line;
    if(!input || !std::getline(input,line) || line.rfind("sample_dt,",0)!=0)throw std::runtime_error("invalid gain design trajectory timebase");
    out.dt=std::stod(line.substr(10));if(!std::getline(input,line))throw std::runtime_error("missing gain design trajectory header");
    std::string expected="index";for(const auto& joint:joints)expected+=","+joint;
    if(!line.empty() && line.back()=='\r')line.pop_back();
    if(line!=expected)throw std::runtime_error("gain design trajectory joint order mismatch");
    while(std::getline(input,line)) {
        if(line.empty())continue;
        const auto comma=line.find(',');if(comma==std::string::npos)throw std::runtime_error("invalid gain design trajectory row");
        const auto q=parse_positions(line.substr(comma+1));if(q.size()!=joints.size())throw std::runtime_error("gain design trajectory mapping mismatch");out.positions.push_back(q);
    }
    if(!std::isfinite(out.dt) || out.dt<=0 || out.positions.size()<3)throw std::runtime_error("insufficient gain design trajectory");
    return out;
}

// Read-only local suggestions used when creating a Profile. The live calibration
// evaluates its complete taught path and actual hardware limits independently.
std::string gain_initialization_json(const serial_arm::DynamicsCfg& cfg,
    serial_arm::Dynamics& dynamics, serial_arm::JointVector positions,
    const std::string& limits_json,const std::string& path_csv={}) {
    using namespace serial_arm::servo_tuning;
    if(limits_json.empty()) return {};
    const auto input=YAML::Load(limits_json);
    const auto n=cfg.joint_names.size();
    if(!input.IsSequence() || input.size()!=n)throw std::runtime_error("gain limits must match controlled joint mapping");
    if(positions.empty())positions.assign(n,0.0);
    if(positions.size()!=n)throw std::runtime_error("position vector size must match controlled joint count");
    std::vector<ModelAxisLimits> limits;
    for(std::size_t i=0;i<n;++i) {
        const auto axis=input[i];
        if(axis["joint"].as<std::string>()!=cfg.joint_names[i])throw std::runtime_error("gain limits joint order mismatch");
        limits.push_back({axis["position_margin_rad"].as<double>(),axis["max_velocity"].as<double>(),
            axis["max_acceleration"].as<double>(),axis["effort_budget"].as<double>(),
            axis["kp_max"].as<double>(),axis["kd_max"].as<double>(),
            {axis["protocol_kp_quantum"].as<double>(0),axis["protocol_kd_quantum"].as<double>(0)}});
        validate_model_limits(limits.back());
    }
    serial_arm::JointState state;state.vel.assign(n,0);state.tor.assign(n,0);
    const serial_arm::JointVector zeros(n,0);
    const auto first=input[0];
    ModelResponseDemand demand=recovery_response_demand(n,
        first["gain_design_peak_error_rad"].as<double>(Requirements{}.peak_error_rad),
        first["gain_design_settling_time_s"].as<double>(Requirements{}.settling_time_s),
        first["gain_design_probe_amplitude_rad"].as<double>(Requirements{}.amplitude_rad));
    const auto design_path=read_gain_design_path(path_csv,cfg.joint_names);
    if(!design_path.positions.empty()) {
        auto actual=path_response_demand(design_path.positions,design_path.dt,demand.peak_error_rad);
        actual.settling_time_s=demand.settling_time_s;actual.designed_amplitude_rad=demand.designed_amplitude_rad;demand=std::move(actual);
    }
    demand.coupled_acceleration_torque.assign(n,0);
    const auto sample=[&](const serial_arm::JointVector& q) {
        state.pos=q;
        if(!dynamics.update(state,zeros,zeros))throw std::runtime_error("model gain initialization dynamics update failed");
        std::vector<double> diagonal;
        const auto& matrix=dynamics.get_state().mass_matrix;
        std::vector<std::vector<double>> full(n,std::vector<double>(n));
        for(std::size_t i=0;i<n;++i) {
            double acceleration_torque=0;
            for(std::size_t j=0;j<n;++j) {
                full[i][j]=matrix(static_cast<Eigen::Index>(i),static_cast<Eigen::Index>(j));
                acceleration_torque+=std::abs(full[i][j])*demand.acceleration_peak[j];
            }
            demand.coupled_acceleration_torque[i]=std::max(demand.coupled_acceleration_torque[i],acceleration_torque);
        }
        demand.mass_matrix_samples.push_back(std::move(full));
        // DynamicsState is already projected into controlled-joint order
        for(std::size_t i=0;i<n;++i)diagonal.push_back(matrix(static_cast<Eigen::Index>(i),static_cast<Eigen::Index>(i)));
        return diagonal;
    };
    std::vector<std::vector<double>> inertias;
    std::vector<double> gradient(n,0),nominal_gradient(n,0);
    const auto gradient_at=[&](const serial_arm::JointVector& posture,bool sample_inertia) {
      for(std::size_t i=0;i<n;++i) {
        const double delta=std::min(.01,.2*limits[i].position_margin_rad);
        auto q=posture;q[i]+=delta;
        if(sample_inertia)inertias.push_back(sample(q));else {state.pos=q;if(!dynamics.update(state,zeros,zeros))throw std::runtime_error("gravity-gradient model update failed");}
        const auto plus=dynamics.get_state().gravity;
        const auto plus_compensation=dynamics.get_state().gravity_compensation;
        q[i]-=2*delta;
        if(sample_inertia)inertias.push_back(sample(q));else {state.pos=q;if(!dynamics.update(state,zeros,zeros))throw std::runtime_error("gravity-gradient model update failed");}
        const auto minus=dynamics.get_state().gravity;
        const auto minus_compensation=dynamics.get_state().gravity_compensation;
        const double nominal=(plus[i]-minus[i])/(2*delta);if(std::abs(nominal)>std::abs(nominal_gradient[i]))nominal_gradient[i]=nominal;
        const auto feedforward=input[i]["model_feedforward_mode"].as<std::string>("NONE");
        double residual=nominal;
        if(feedforward=="GRAVITY" || feedforward=="FULL_INVERSE_DYNAMICS")
            residual-=(plus_compensation[i]-minus_compensation[i])/(2*delta);
        if(std::abs(residual)>std::abs(gradient[i]))gradient[i]=residual;
      }
    };
    if(design_path.positions.empty()){inertias.push_back(sample(positions));gradient_at(positions,true);}
    else {
        const auto stride=std::max<std::size_t>(1,design_path.positions.size()/128);
        for(std::size_t k=0;;k=std::min(design_path.positions.size()-1,k+stride)) {
            const auto& q=design_path.positions[k];inertias.push_back(sample(q));gradient_at(q,false);
            if(k==design_path.positions.size()-1)break;
        }
    }
    const auto result=model_initialization(inertias,gradient,limits,demand);
    sample(positions);
    std::ostringstream out;out<<std::setprecision(17);
    out<<"{\"valid\":true,\"measured\":false,\"source\":";print_string(out,result.source);
    out<<",\"scope\":\""<<(design_path.positions.empty()?"unmeasured_recovery_design":"provided_path_offline_design")<<"\",\"task_demand_source\":\""<<(design_path.positions.empty()?"recovery_targets_without_demonstration":"provided_demonstration_path")<<"\",\"path_samples\":"<<design_path.positions.size()<<",\"axes\":[";
    for(std::size_t i=0;i<n;++i) {
        if(i)out<<',';
        out<<"{\"joint\":";print_string(out,cfg.joint_names[i]);
        out<<",\"kp\":"<<result.gains.kp[i]<<",\"kd\":"<<result.gains.kd[i]
           <<",\"inertia_min\":"<<result.inertia_min[i]<<",\"inertia_max\":"<<result.inertia_max[i]
           <<",\"nominal_gravity_gradient\":"<<nominal_gradient[i]<<",\"residual_gravity_gradient\":"<<gradient[i]
           <<",\"frequency_hz\":"<<result.frequency_hz[i]<<",\"task_frequency_hz\":"<<result.task_frequency_hz[i]
           <<",\"unconstrained_kp\":"<<result.unconstrained_kp[i]<<",\"unconstrained_kd\":"<<result.unconstrained_kd[i]
           <<",\"pre_modal_kp\":"<<result.pre_modal_kp[i]<<",\"pre_modal_kd\":"<<result.pre_modal_kd[i]
           <<",\"working_kp_max\":"<<result.working_kp_max[i]<<",\"working_kd_max\":"<<result.working_kd_max[i]
           <<",\"reference_error_bound_rad\":"<<result.reference_error_bound_rad[i]
           <<",\"velocity_error_bound_rad\":"<<result.velocity_error_bound_rad[i]<<",\"gain_domain_source\":";
        print_string(out,input[i]["gain_domain_source"].as<std::string>("legacy_unspecified_domain"));
        out<<",\"constraints\":[";
        for(std::size_t k=0;k<result.constraints[i].size();++k){if(k)out<<',';print_string(out,result.constraints[i][k]);}
        out<<"]}";
    }
    out<<"],\"nominal_mode_frequency_hz\":";print_vector(out,result.nominal_mode_frequency_hz);
    out<<",\"nominal_mode_damping_ratio\":";print_vector(out,result.nominal_mode_damping_ratio);
    out<<",\"modal_adjustments\":[";for(std::size_t i=0;i<result.modal_adjustments.size();++i){if(i)out<<',';print_string(out,result.modal_adjustments[i]);}out<<']';
    out<<"}";return out.str();
}

std::string nominal_gain_design_json(const serial_arm::DynamicsCfg& cfg,serial_arm::Dynamics& dynamics) {
    const auto n=cfg.joint_names.size();
    serial_arm::JointState zero;zero.pos.assign(n,0);zero.vel.assign(n,0);zero.tor.assign(n,0);
    if(!dynamics.update(zero,zero.vel,zero.vel))throw std::runtime_error("URDF nominal design dynamics update failed");
    const auto& M=dynamics.get_state().mass_matrix;
    std::vector<std::vector<double>> matrix(n,std::vector<double>(n));
    for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)matrix[i][j]=M(i,j);
    const auto design=serial_arm::servo_tuning::nominal_urdf_gain_design(matrix);
    std::ostringstream out;out<<std::setprecision(17)<<"{\"valid\":true,\"measured\":false,\"source\":";
    print_string(out,design.source);
    out<<",\"design_frequency_hz\":"<<design.design_frequency_hz<<",\"damping_ratio\":"<<design.damping_ratio;
    out<<",\"design_parameters\":{\"frequency_hz\":"<<design.design_frequency_hz<<",\"damping_ratio\":"<<design.damping_ratio
       <<",\"frequency_role\":\"lower_bound_on_undamped_ideal_model_modes_before_protocol_encoding\""
       <<",\"source\":\"explicit_engineering_parameters_not_urdf_measurements\"}";
    out<<",\"design_target_source\":\"explicit_generic_response_design_not_urdf_measurement\",\"basis_pose\":\"urdf_joint_zero_no_motion\",\"joint_names\":[";
    for(std::size_t i=0;i<n;++i){if(i)out<<',';print_string(out,cfg.joint_names[i]);}
    out<<"],\"positions\":";print_vector(out,zero.pos);
    out<<",\"mass_matrix_basis\":";print_matrix(out,M);
    out<<",\"inertia_basis\":";print_vector(out,design.inertia_basis);
    out<<",\"unconstrained_kp\":";print_vector(out,design.unconstrained_kp);
    out<<",\"unconstrained_kd\":";print_vector(out,design.unconstrained_kd);
    print_nominal_damping_design(out,design);
    out<<",\"modal_diagnostics\":";print_nominal_modes(out,design.modal_diagnostics);
    const auto soft=serial_arm::servo_tuning::nominal_urdf_gain_design(matrix,1,.7);
    out<<",\"purpose_nominal_gain_designs\":{";
    bool first=true;
    for(const auto& role:std::vector<std::string>{"rigid_hold","rigid_tracking","compliant_hold","compliant_tracking","servo_tracking","compliant_drag"}) {
        if(!first)out<<',';
        first=false;print_string(out,role);out<<':';
        const auto& purpose=role=="compliant_hold" || role=="compliant_tracking"?soft:design;
        const bool drag=role=="compliant_drag";
        out<<"{\"measured\":false,\"validated\":false,\"design_frequency_hz\":"<<purpose.design_frequency_hz
           <<",\"damping_ratio\":"<<purpose.damping_ratio<<",\"design_frequency_role\":\""
           <<(drag?"damping_design_coefficient_only_zero_stiffness_has_no_restoring_frequency":
               "lower_bound_on_undamped_ideal_model_modes_before_protocol_encoding")<<"\",\"unconstrained_kp\":";
        print_vector(out,drag?zero.pos:purpose.unconstrained_kp);
        out<<",\"unconstrained_kd\":";print_vector(out,purpose.unconstrained_kd);
        print_nominal_damping_design(out,purpose);
        out<<",\"modal_diagnostics\":";print_nominal_modes(out,drag?
            serial_arm::servo_tuning::ideal_urdf_modal_diagnostics(matrix,zero.pos,purpose.unconstrained_kd):purpose.modal_diagnostics);
        out<<",\"scope\":\""<<(drag?"zero_stiffness_unmeasured_damping_candidate":"unmeasured_nominal_response_design_not_use_validation")<<"\"}";
    }
    out<<'}';
    out<<'}';return out.str();
}

void emit_payload(const serial_arm::DynamicsCfg& cfg,
                  serial_arm::Dynamics& dynamics,
                  serial_arm::JointVector positions,const std::string& nominal_design,
                  const std::string& limits_json = {},const std::string& path_csv={}) {
    const auto initialization=gain_initialization_json(cfg,dynamics,positions,limits_json,path_csv);
    if(positions.empty()) positions.assign(cfg.joint_names.size(), 0.0);
    if(positions.size() != cfg.joint_names.size()) {
        throw std::runtime_error("position vector size must match controlled joint count");
    }

    serial_arm::JointState joint_state;
    joint_state.pos = positions;
    joint_state.vel.assign(positions.size(), 0.0);
    joint_state.tor.assign(positions.size(), 0.0);
    serial_arm::JointVector zeros(positions.size(), 0.0);
    const auto updated = dynamics.update(joint_state, zeros, zeros);
    if(!updated) {
        throw std::runtime_error("Dynamics update failed; DynamicsErr=" +
            std::to_string(static_cast<int>(updated.error())));
    }

    const auto& info = dynamics.get_info();
    const auto& state = dynamics.get_state();
    std::cout << std::setprecision(17);
    std::cout << "{\"ok\":true,\"schema\":\"serial-arm-model\",\"config\":{";
    std::cout << "\"urdf_path\":"; print_string(std::cout, cfg.urdf_path);
    std::cout << ",\"urdf_fingerprint\":"; print_string(std::cout, serial_arm::model_calibration_file_fingerprint(cfg.urdf_path));
    std::cout << ",\"base_frame\":"; print_string(std::cout, cfg.base_frame);
    std::cout << ",\"tool_frame\":"; print_string(std::cout, cfg.tool_frame);
    std::cout << ",\"gravity\":[" << cfg.gravity[0] << ',' << cfg.gravity[1] << ',' << cfg.gravity[2] << ']';
    std::cout << ",\"gravity_scale\":"; print_vector(std::cout, cfg.gravity_scale);
    std::cout << ",\"joint_names\":[";
    for(std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
        if(i) std::cout << ',';
        print_string(std::cout, cfg.joint_names[i]);
    }
    std::cout << "]},\"info\":{";
    std::cout << "\"joints_count\":" << info.joints_count << ",\"nq\":" << info.nq << ",\"nv\":" << info.nv;
    std::cout << ",\"total_mass\":" << info.total_mass;
    std::cout << ",\"reduced_total_mass\":" << info.reduced_total_mass;
    std::cout << ",\"effective_moving_mass\":" << info.effective_moving_mass;
    std::cout << ",\"frame_names\":[";
    for(std::size_t i = 0; i < info.frame_names.size(); ++i) {
        if(i) std::cout << ',';
        print_string(std::cout, info.frame_names[i]);
    }
    std::cout << "],\"effective_inertias\":[";
    for(std::size_t i = 0; i < info.effective_inertias.size(); ++i) {
        if(i) std::cout << ',';
        const auto& item = info.effective_inertias[i];
        std::cout << "{\"joint_name\":"; print_string(std::cout, item.joint_name);
        std::cout << ",\"mass\":" << item.mass << ",\"center_of_mass\":";
        print_vec3(std::cout, item.center_of_mass);
        std::cout << ",\"inertia\":"; print_matrix3(std::cout, item.inertia);
        std::cout << '}';
    }
    std::cout << "]},\"state\":{";
    std::cout << "\"positions\":"; print_vector(std::cout, state.pos);
    std::cout << ",\"gravity\":"; print_vector(std::cout, state.gravity);
    std::cout << ",\"gravity_compensation\":"; print_vector(std::cout, state.gravity_compensation);
    std::cout << ",\"mass_matrix\":"; print_matrix(std::cout, state.mass_matrix);
    std::cout << ",\"center_of_mass\":"; print_vec3(std::cout, state.center_of_mass);
    std::cout << ",\"frames\":[";
    bool first_frame = true;
    for(const auto& frame_name : info.frame_names) {
        const auto pose_result = dynamics.get_frame_pose(frame_name);
        if(!pose_result) continue;
        if(!first_frame) std::cout << ',';
        first_frame = false;
        const Eigen::Isometry3d& pose = pose_result.value();
        const Eigen::Quaterniond q(pose.linear());
        std::cout << "{\"name\":"; print_string(std::cout, frame_name);
        std::cout << ",\"position\":"; print_vec3(std::cout, pose.translation());
        std::cout << ",\"quaternion\":[" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << "]}";
    }
    std::cout << "]}";
    if(!initialization.empty())std::cout<<",\"gain_initialization\":"<<initialization;
    std::cout<<",\"nominal_gain_design\":"<<nominal_design;
    std::cout<<"}\n"<<std::flush;
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::string config_path;
        std::string positions_text;
        std::string gain_limits_json;
        std::string gain_path_csv;
        bool server_mode = false;
        for(int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if(arg == "--config" && i + 1 < argc) config_path = argv[++i];
            else if(arg == "--positions" && i + 1 < argc) positions_text = argv[++i];
            else if(arg == "--server") server_mode = true;
            else if(arg == "--gain-limits-json" && i+1<argc) gain_limits_json=argv[++i];
            else if(arg == "--gain-path-csv" && i+1<argc) gain_path_csv=argv[++i];
            else if(arg == "--help" || arg == "-h") {
                std::cout << "Usage: serial_arm_model_probe --config <core.yaml> [--positions q1,q2,...] [--server] [--gain-limits-json JSON] [--gain-path-csv trajectory.csv]\n";
                return 0;
            }
            else throw std::runtime_error("unknown or incomplete argument: " + arg);
        }
        if(config_path.empty()) throw std::runtime_error("--config is required");
        if(server_mode && !gain_limits_json.empty()) throw std::runtime_error("gain initialization is a one-shot read-only model operation");
        if(!gain_path_csv.empty() && (server_mode || gain_limits_json.empty()))throw std::runtime_error("gain design trajectory requires one-shot gain limits");
        if(server_mode && !positions_text.empty()) throw std::runtime_error("--positions cannot be combined with --server");

        const auto cfg_result = serial_arm::load_dynamics_cfg(config_path);
        if(!cfg_result) throw std::runtime_error(cfg_result.error().message);
        const serial_arm::DynamicsCfg cfg = cfg_result.value();

        serial_arm::Dynamics dynamics;
        const auto configured = dynamics.configure(cfg);
        if(!configured) {
            throw std::runtime_error("Dynamics configure failed; DynamicsErr=" +
                std::to_string(static_cast<int>(configured.error())));
        }

        // Immutable model: viewer poses never change the neutral URDF basis.
        const auto nominal_design=nominal_gain_design_json(cfg,dynamics);
        if(!server_mode) {
            emit_payload(cfg, dynamics, parse_positions(positions_text),nominal_design,gain_limits_json,gain_path_csv);
            return 0;
        }

        std::string line;
        while(std::getline(std::cin, line)) {
            if(line == "quit") break;
            try {
                emit_payload(cfg, dynamics, parse_positions(line),nominal_design);
            }
            catch(const std::exception& error) {
                print_error(error.what());
            }
        }
        return 0;
    }
    catch(const std::exception& error) {
        print_error(error.what());
        return 2;
    }
}
