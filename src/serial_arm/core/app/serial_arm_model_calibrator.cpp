#include "serial_arm/config/config.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/dynamics/gravity_calibration.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    try {
        std::string config_path;
        std::string dataset_path;
        std::string output_path;
        std::string verify_urdf_path;
        std::string candidate_source="online";
        std::string selected_candidate_path;
        std::string requested_mode;
        std::vector<std::string> requested_locked_links;
        bool locked_override=false;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--config"&&i+1<argc) config_path=argv[++i];
            else if(arg=="--dataset"&&i+1<argc) dataset_path=argv[++i];
            else if(arg=="--output"&&i+1<argc) output_path=argv[++i];
            else if(arg=="--verify-urdf"&&i+1<argc) verify_urdf_path=argv[++i];
            else if(arg=="--candidate-source"&&i+1<argc) candidate_source=argv[++i];
            else if(arg=="--candidate-path"&&i+1<argc) selected_candidate_path=argv[++i];
            else if(arg=="--mode"&&i+1<argc) requested_mode=argv[++i];
            else if(arg=="--lock-link"&&i+1<argc) {requested_locked_links.push_back(argv[++i]);locked_override=true;}
            else if(arg=="--help"||arg=="-h") {
                std::cout << "Usage: serial_arm_model_calibrator --config <core.yaml> --dataset <task-dir> --verify-urdf candidate.urdf [--candidate-source online|offline] [--candidate-path task-dir/recomputed-candidate.json]\n";
                return EXIT_SUCCESS;
            } else throw std::runtime_error("unknown or incomplete argument: "+arg);
        }
        if(verify_urdf_path.empty()) throw std::runtime_error("Offline fitting uses tools/model_calibration_cli.py recompute, shared with the GUI; this tool only verifies a stored candidate");
        if(!output_path.empty()||!requested_mode.empty()||locked_override) throw std::runtime_error("Verification cannot refit or override a stored candidate");
        if(candidate_source!="online"&&candidate_source!="offline") throw std::runtime_error("invalid candidate source");
        if(config_path.empty()||dataset_path.empty()) throw std::runtime_error("--config and --dataset are required");
        const std::filesystem::path task_directory=std::filesystem::absolute(dataset_path);
        const std::filesystem::path metadata_path=task_directory/"metadata.json";
        if(!std::filesystem::is_regular_file(metadata_path)) throw std::runtime_error("model calibration metadata.json is missing");
        const YAML::Node metadata=YAML::LoadFile(metadata_path.string());
        const std::string recorded_core=metadata["core_fingerprint"]?metadata["core_fingerprint"].as<std::string>():std::string{};
        const std::string recorded_urdf=metadata["urdf_fingerprint"]?metadata["urdf_fingerprint"].as<std::string>():std::string{};
        if(recorded_core.empty()||recorded_urdf.empty()) throw std::runtime_error("recorded model fingerprints are missing");

        std::string model_config_path=config_path;
        auto cfg_result=serial_arm::load_dynamics_cfg(model_config_path);
        bool source_matches=cfg_result && recorded_core==serial_arm::model_calibration_file_fingerprint(model_config_path) &&
            recorded_urdf==serial_arm::model_calibration_file_fingerprint(cfg_result->urdf_path);
        if(!source_matches) {
            const std::filesystem::path snapshot_core=task_directory/"source"/"core.yaml";
            const std::filesystem::path snapshot_urdf=task_directory/"source"/"model.urdf";
            if(!std::filesystem::is_regular_file(snapshot_core)||!std::filesystem::is_regular_file(snapshot_urdf)||
                serial_arm::model_calibration_file_fingerprint(snapshot_core.string())!=recorded_core||
                serial_arm::model_calibration_file_fingerprint(snapshot_urdf.string())!=recorded_urdf) {
                throw std::runtime_error("recorded source changed and calibration snapshots are unavailable or invalid");
            }
            model_config_path=snapshot_core.string();
            cfg_result=serial_arm::load_dynamics_cfg(model_config_path);
            if(!cfg_result) throw std::runtime_error(cfg_result.error().message);
            cfg_result->urdf_path=snapshot_urdf.string();
            source_matches=true;
        }
        if(!cfg_result) throw std::runtime_error(cfg_result.error().message);
        if(metadata["joint_names"]&&metadata["joint_names"].IsSequence()) {
            const auto recorded_joints=metadata["joint_names"].as<std::vector<std::string>>();
            if(recorded_joints!=cfg_result->joint_names) throw std::runtime_error("joint order does not match recorded calibration data");
        }
        serial_arm::Dynamics dynamics;
        const auto configured=dynamics.configure(cfg_result.value());
        if(!configured) throw std::runtime_error("Dynamics configure failed");
        const auto frames=serial_arm::load_model_calibration_frames(dataset_path,cfg_result->joint_names.size());
        if(!frames) throw std::runtime_error(frames.error());
        // Verify the selected saved candidate without running any solver
        auto record_path=task_directory/(candidate_source=="offline"?"recomputed-candidate.json":"result.json");
        if(!selected_candidate_path.empty()) {
            if(candidate_source!="offline")throw std::runtime_error("--candidate-path requires offline candidate source");
            const auto root=std::filesystem::canonical(task_directory);
            const auto selected=std::filesystem::canonical(selected_candidate_path);
            const auto relative=selected.lexically_relative(root);
            if(selected.filename()!="recomputed-candidate.json"||relative.empty()||
                *relative.begin()==".."||!std::filesystem::is_regular_file(selected))
                throw std::runtime_error("candidate path must be a recomputed-candidate.json inside the task directory");
            record_path=selected;
        }
        const auto record=YAML::LoadFile(record_path.string());
        if(!record["task_id"] || record["task_id"].as<std::string>()!=metadata["task_id"].as<std::string>())
            throw std::runtime_error("candidate task does not match dataset");
        if(!selected_candidate_path.empty()) {
            if(!record["core_fingerprint"]||!record["urdf_fingerprint"]||
                record["core_fingerprint"].as<std::string>()!=recorded_core||
                record["urdf_fingerprint"].as<std::string>()!=recorded_urdf)
                throw std::runtime_error("selected candidate source fingerprints do not match the recorded dataset");
        }
        const auto candidate_record=candidate_source=="offline"?record:record["gravity_result"];
        if(!candidate_record || !candidate_record["first_moments"] || !candidate_record["first_moments"].IsSequence())
            throw std::runtime_error("saved gravity candidate is missing");
        std::vector<serial_arm::GravityFirstMoment> first_moments;
        for(const auto& item:candidate_record["first_moments"]) {
            serial_arm::GravityFirstMoment moment;
            moment.link_name=item["link_name"].as<std::string>();
            const auto values=item["value"].as<std::vector<double>>();
            if(values.size()!=3) throw std::runtime_error("invalid stored first moment");
            moment.value=Eigen::Vector3d(values[0],values[1],values[2]);
            if(candidate_source=="offline") moment.candidate_mass=candidate_record["candidate_masses"][moment.link_name].as<double>();
            first_moments.push_back(moment);
        }
        if(!verify_urdf_path.empty()) {
            serial_arm::DynamicsCfg candidate_cfg=cfg_result.value();
            candidate_cfg.urdf_path=std::filesystem::absolute(verify_urdf_path).string();
            candidate_cfg.gravity_correction_path.clear();
            candidate_cfg.gravity_scale.assign(candidate_cfg.joint_names.size(),1.0);
            serial_arm::Dynamics candidate;
            const auto candidate_configured=candidate.configure(candidate_cfg);
            if(!candidate_configured) throw std::runtime_error("candidate Dynamics configure failed");
            std::vector<serial_arm::JointVector> poses;
            for(const auto& frame:frames.value()) {
                if(!frame.valid||frame.position.size()!=candidate_cfg.joint_names.size()) continue;
                bool duplicate=false;
                for(const auto& existing:poses) {
                    double maximum=0.0;
                    for(std::size_t i=0;i<existing.size();++i) maximum=std::max(maximum,std::abs(existing[i]-frame.position[i]));
                    if(maximum<1.0e-4){duplicate=true;break;}
                }
                if(!duplicate)poses.push_back(frame.position);
                if(poses.size()>=5)break;
            }
            if(poses.empty())poses.push_back(serial_arm::JointVector(candidate_cfg.joint_names.size(),0.0));
            double gravity_sq=0.0;std::size_t gravity_count=0;double max_position_error=0.0;double max_rotation_error=0.0;
            for(const auto& q:poses) {
                const auto expected=dynamics.compute_gravity_with_first_moments(q,first_moments);
                if(!expected)throw std::runtime_error("candidate gravity prediction failed");
                serial_arm::JointState state;state.pos=q;state.vel.assign(q.size(),0.0);state.tor.assign(q.size(),0.0);
                serial_arm::JointVector zeros(q.size(),0.0);
                const auto original_updated=dynamics.update(state,zeros,zeros);
                const auto candidate_updated=candidate.update(state,zeros,zeros);
                if(!original_updated||!candidate_updated)throw std::runtime_error("candidate verification Dynamics update failed");
                const auto& actual=candidate.get_gravity();
                if(actual.size()!=expected->size())throw std::runtime_error("candidate gravity size mismatch");
                for(std::size_t i=0;i<actual.size();++i){const double error=actual[i]-(*expected)[i];gravity_sq+=error*error;++gravity_count;}
                const Eigen::Isometry3d original_tool=dynamics.get_tool_pose();
                const Eigen::Isometry3d candidate_tool=candidate.get_tool_pose();
                max_position_error=std::max(max_position_error,(original_tool.translation()-candidate_tool.translation()).norm());
                const Eigen::Matrix3d delta=original_tool.rotation().transpose()*candidate_tool.rotation();
                // acos(trace) amplifies roundoff near identity to sqrt(epsilon),
                // falsely rejecting unchanged geometry at the 1e-10 threshold.
                // AngleAxis uses a quaternion/atan2 conversion stable near zero
                max_rotation_error=std::max(max_rotation_error,Eigen::AngleAxisd(delta).angle());
            }
            const double gravity_rms=gravity_count?std::sqrt(gravity_sq/static_cast<double>(gravity_count)):0.0;
            const bool ok=std::isfinite(gravity_rms)&&gravity_rms<1.0e-7&&max_position_error<1.0e-10&&max_rotation_error<1.0e-10;
            std::cout<<std::setprecision(17)<<"{\"ok\":"<<(ok?"true":"false")<<",\"samples\":"<<poses.size()<<",\"gravity_rms_nm\":"<<gravity_rms<<",\"fk_max_position_error_m\":"<<max_position_error<<",\"fk_max_rotation_error_rad\":"<<max_rotation_error<<",\"quality_validation_performed\":false,\"candidate_source\":\""<<candidate_source<<"\",\"scope\":\"stored candidate export consistency only\"}\n";
            return ok?EXIT_SUCCESS:4;
        }
        return EXIT_FAILURE;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
