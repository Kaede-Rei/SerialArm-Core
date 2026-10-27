#include "serial_arm/dynamics/gravity_calibration.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace serial_arm;
void require(bool good,const char* message) { if(!good)throw std::runtime_error(message); }
int main() {
    const auto root=std::filesystem::temp_directory_path()/
        ("serialarm-encoding-metadata-"+std::to_string(::getpid()));
    try {
        ModelCalibrationMetadata metadata;
        metadata.task_id="metadata-limits-test";metadata.joint_names={"joint-b","joint-a"};
        metadata.actuator_capabilities_source="hardware_backend_after_successful_activate";
        metadata.actuator_capabilities={{"motor-b",-6.25,6.25,16,12,500,5,"joint-b"},
                                        {"motor-a",-12.5,12.5,10,28,500,5,"joint-a"}};
        ModelCalibrationRecorder recorder;
        require(bool(recorder.start(root/"resolved",metadata)),"start resolved recorder");recorder.stop();
        auto data=YAML::LoadFile((root/"resolved/metadata.json").string());
        const auto record=data["actuator_capabilities"];
        require(record["source"].as<std::string>()==metadata.actuator_capabilities_source,"readback provenance lost");
        require(record["scope"].as<std::string>().find("not_joint_safety_limits")!=std::string::npos,"scope overstates joint safety");
        require(record["units"]["effort"].as<std::string>()=="actuator_protocol_Nm","effort claim not protocol-specific");
        const auto axes=record["axes"];require(axes.size()==2,"axes missing");
        for(unsigned i=0;i<2;++i) {
            const auto& cap=metadata.actuator_capabilities[i];const auto axis=axes[i];
            require(axis["joint_name"].as<std::string>()==cap.joint_name &&
                    axis["actuator_name"].as<std::string>()==cap.actuator_name,"axis order or identity lost");
            require(axis["min_pos"].as<double>()==cap.min_pos && axis["max_pos"].as<double>()==cap.max_pos &&
                    axis["max_vel"].as<double>()==cap.max_vel && axis["max_effort"].as<double>()==cap.max_effort,
                    "confirmed per-axis ranges not archived");
        }
        ModelCalibrationMetadata unknown;unknown.task_id="old-client";unknown.joint_names={"axis"};
        require(bool(recorder.start(root/"unknown",unknown)),"legacy metadata client rejected");recorder.stop();
        data=YAML::LoadFile((root/"unknown/metadata.json").string());
        require(data["actuator_capabilities"]["source"].as<std::string>()=="unavailable" &&
                data["actuator_capabilities"]["axes"].size()==0,"absent readback fabricated nominal values");
        std::filesystem::remove_all(root);
        std::cout<<"ENCODING_LIMIT_METADATA_PASS\n";return 0;
    }catch(const std::exception& error) {
        std::filesystem::remove_all(root);std::cerr<<error.what()<<'\n';return 1;
    }
}
