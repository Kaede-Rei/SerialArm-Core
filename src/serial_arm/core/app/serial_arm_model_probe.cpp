#include "serial_arm/config/config.hpp"
#include "serial_arm/dynamics/dynamics.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
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

void emit_payload(const serial_arm::DynamicsCfg& cfg,
                  serial_arm::Dynamics& dynamics,
                  serial_arm::JointVector positions) {
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
    std::cout << "]}}\n" << std::flush;
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::string config_path;
        std::string positions_text;
        bool server_mode = false;
        for(int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if(arg == "--config" && i + 1 < argc) config_path = argv[++i];
            else if(arg == "--positions" && i + 1 < argc) positions_text = argv[++i];
            else if(arg == "--server") server_mode = true;
            else if(arg == "--help" || arg == "-h") {
                std::cout << "Usage: serial_arm_model_probe --config <core.yaml> [--positions q1,q2,...] [--server]\n";
                return 0;
            }
            else throw std::runtime_error("unknown or incomplete argument: " + arg);
        }
        if(config_path.empty()) throw std::runtime_error("--config is required");
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

        if(!server_mode) {
            emit_payload(cfg, dynamics, parse_positions(positions_text));
            return 0;
        }

        std::string line;
        while(std::getline(std::cin, line)) {
            if(line == "quit") break;
            try {
                emit_payload(cfg, dynamics, parse_positions(line));
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
