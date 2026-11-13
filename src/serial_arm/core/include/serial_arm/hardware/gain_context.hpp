#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <system_error>

namespace serial_arm {

inline std::string gain_hardware_driver_identity(const std::string& path) {
    auto name=std::filesystem::path(path).filename().string();
    if(name.rfind("lib",0)==0)name.erase(0,3);
    const auto suffix=name.find(".so");
    if(suffix!=std::string::npos)name.resize(suffix);
    return name;
}

// Only an existing character device establishes equivalence between serial
// aliases. Missing paths, regular files and arbitrary names retain their text.
inline std::string gain_serial_device_identity(const std::string& value) {
    const std::filesystem::path path(value);
    if(!path.is_absolute())return value;
    std::error_code error;
    if(!std::filesystem::is_character_file(path,error) || error)return value;
    const auto canonical=std::filesystem::canonical(path,error);
    return error?value:canonical.string();
}

// This is a comparison view, never the configuration passed to a driver.
// HardwareLoader materializes the selected named bus into Damiao's flat
// fields only when overrides are present. Damiao itself accepts both forms,
// with explicit flat serial_port/baudrate taking precedence over that bus.
// Preserve all other keys, including bus/backend and actuator declarations.
inline YAML::Node gain_hardware_context(const YAML::Node& config,const std::string& plugin) {
    auto result=YAML::Clone(config);
    if(gain_hardware_driver_identity(plugin)!="serial_arm_hardware_damiao" || !result.IsMap())return result;
    auto target=result["damiao"]?result["damiao"]:result;
    if(!target.IsMap())return result;
    YAML::Node bus;
    if((!target["bus"] || target["bus"].IsScalar()) && result["buses"] && result["buses"].IsMap()) {
        const auto buses=static_cast<const YAML::Node&>(result)["buses"];
        const auto selected=buses[target["bus"].as<std::string>("main_can")];
        if(selected)bus=selected;
    }
    if(bus && bus.IsMap()) {
        const auto port=bus["serial_port"]?bus["serial_port"]:bus["device"];
        if(!target["serial_port"] && port)target["serial_port"]=YAML::Clone(port);
        if(!target["baudrate"] && bus["baudrate"])target["baudrate"]=YAML::Clone(bus["baudrate"]);
        for(const auto* key:{"serial_port","device"})
            if(bus[key] && bus[key].IsScalar())bus[key]=gain_serial_device_identity(bus[key].as<std::string>());
    }
    if(target["serial_port"] && target["serial_port"].IsScalar())
        target["serial_port"]=gain_serial_device_identity(target["serial_port"].as<std::string>());
    return result;
}

} // namespace serial_arm
