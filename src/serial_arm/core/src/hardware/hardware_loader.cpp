#include "serial_arm/hardware/hardware_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <dlfcn.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace serial_arm {

namespace {

using CreateMotorBusFn = MotorBus* (*)();
using DestroyMotorBusFn = void (*)(MotorBus*);

bool has_overrides(const HardwareConfigOverrides& overrides) {
    return overrides.serial_port.has_value() || overrides.baudrate.has_value() || overrides.bus.has_value();
}

bool has_connection_field(const YAML::Node& node) {
    return node && node.IsMap() && (node["serial_port"] || node["baudrate"] || node["bus"]);
}

YAML::Node select_connection_node(const YAML::Node& root) {
    if(!root || !root.IsMap()) return {};
    if(has_connection_field(root)) return root;

    YAML::Node only_map_child;
    std::size_t map_child_count = 0;
    for(const auto& item : root) {
        if(!item.second.IsMap()) continue;
        ++map_child_count;
        only_map_child = item.second;
        if(has_connection_field(item.second)) return item.second;
    }
    if(map_child_count == 1) return only_map_child;
    return {};
}

tl::expected<void, HardwareLoaderErr> validate_overrides(const HardwareConfigOverrides& overrides) {
    if(overrides.serial_port && overrides.serial_port->empty()) return tl::make_unexpected(HardwareLoaderErr::INVALID_OVERRIDE);
    if(overrides.bus && overrides.bus->empty()) return tl::make_unexpected(HardwareLoaderErr::INVALID_OVERRIDE);
    if(overrides.baudrate && *overrides.baudrate <= 0) return tl::make_unexpected(HardwareLoaderErr::INVALID_OVERRIDE);
    return {};
}

std::filesystem::path make_temp_config_path() {
    const auto base = std::filesystem::temp_directory_path();
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return base / ("serial_arm_hardware_override_" + std::to_string(now) + ".yaml");
}

tl::expected<std::filesystem::path, HardwareLoaderErr> write_effective_config(
    const std::string& config_path,
    const HardwareConfigOverrides& overrides) {
    const auto valid = validate_overrides(overrides);
    if(!valid) return tl::make_unexpected(valid.error());

    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);
    }
    catch(const YAML::BadFile&) {
        return tl::make_unexpected(HardwareLoaderErr::CONFIG_OPEN_FAILED);
    }
    catch(const YAML::Exception&) {
        return tl::make_unexpected(HardwareLoaderErr::CONFIG_SYNTAX_ERROR);
    }

    YAML::Node target = select_connection_node(root);
    if(!target || !target.IsMap()) return tl::make_unexpected(HardwareLoaderErr::CONFIG_SYNTAX_ERROR);

    if(overrides.serial_port) target["serial_port"] = *overrides.serial_port;
    if(overrides.baudrate) target["baudrate"] = *overrides.baudrate;
    if(overrides.bus) target["bus"] = *overrides.bus;

    const std::filesystem::path temp_path = make_temp_config_path();
    std::ofstream output(temp_path);
    if(!output) return tl::make_unexpected(HardwareLoaderErr::CONFIG_OPEN_FAILED);
    output << root;
    if(!output) return tl::make_unexpected(HardwareLoaderErr::CONFIG_OPEN_FAILED);
    return temp_path;
}

/**
 * @brief 绑定 Backend 对象和共享库句柄生命周期的 MotorBus wrapper
 *
 * 外部仍只持有 std::unique_ptr<MotorBus>；wrapper 析构时先通过
 * destroy_motor_bus() 销毁真实 Backend，再 dlclose() 对应 shared library
 */
class LoadedMotorBus final : public MotorBus {
public:
    LoadedMotorBus(void* handle, MotorBus* bus, DestroyMotorBusFn destroy) noexcept
        : handle_(handle),
          bus_(bus),
          destroy_(destroy) {}

    ~LoadedMotorBus() override {
        if(bus_ && destroy_) {
            destroy_(std::exchange(bus_, nullptr));
        }
        if(handle_) {
            dlclose(std::exchange(handle_, nullptr));
        }
    }

    LoadedMotorBus(const LoadedMotorBus&) = delete;
    LoadedMotorBus& operator=(const LoadedMotorBus&) = delete;
    LoadedMotorBus(LoadedMotorBus&&) = delete;
    LoadedMotorBus& operator=(LoadedMotorBus&&) = delete;

    tl::expected<void, MotorBusErr> configure(const std::string& config_path) override {
        return bus_->configure(config_path);
    }

    tl::expected<void, MotorBusErr> connect() override {
        return bus_->connect();
    }

    tl::expected<ActuatorState, MotorBusErr> read() override {
        return bus_->read();
    }

    tl::expected<void, MotorBusErr> activate() override {
        return bus_->activate();
    }

    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        return bus_->write(cmd);
    }

    tl::expected<void, MotorBusErr> stop() override {
        return bus_->stop();
    }

    tl::expected<void, MotorBusErr> deactivate() override {
        return bus_->deactivate();
    }

    tl::expected<void, MotorBusErr> recover() override {
        return bus_->recover();
    }

    const HardwareCapabilities& capabilities() const noexcept override {
        return bus_->capabilities();
    }

    void cleanup() noexcept override {
        bus_->cleanup();
    }

    std::size_t size() const noexcept override {
        return bus_->size();
    }

private:
    void* handle_{ nullptr };
    MotorBus* bus_{ nullptr };
    DestroyMotorBusFn destroy_{ nullptr };
};

void close_handle(void* handle) noexcept {
    if(handle) dlclose(handle);
}

} // namespace

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

HardwareLoader::~HardwareLoader() = default;

HardwareLoader::HardwareLoader(HardwareLoader&& other) noexcept = default;

HardwareLoader& HardwareLoader::operator=(HardwareLoader&& other) noexcept = default;

tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> HardwareLoader::load(const std::string& plugin, const std::string& config_path) {
    return load(plugin, config_path, HardwareConfigOverrides{});
}

tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> HardwareLoader::load(
    const std::string& plugin,
    const std::string& config_path,
    const HardwareConfigOverrides& overrides) {
    std::vector<std::string> candidates;
    candidates.push_back(plugin);
    if(plugin.find('/') == std::string::npos) {
        candidates.push_back("lib" + plugin + ".so");
    }

    void* handle{ nullptr };
    for(const auto& candidate : candidates) {
        handle = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
        if(handle) break;
    }
    if(!handle) return tl::make_unexpected(HardwareLoaderErr::OPEN_FAILED);

    auto create = reinterpret_cast<CreateMotorBusFn>(dlsym(handle, "create_motor_bus"));
    auto destroy = reinterpret_cast<DestroyMotorBusFn>(dlsym(handle, "destroy_motor_bus"));
    if(!create || !destroy) {
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::SYMBOL_FAILED);
    }

    std::unique_ptr<MotorBus, DestroyMotorBusFn> raw(create(), destroy);
    if(!raw) {
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::CREATE_FAILED);
    }

    std::optional<std::filesystem::path> effective_config_path;
    std::string effective_config_string = config_path;
    if(has_overrides(overrides)) {
        auto temp_path = write_effective_config(config_path, overrides);
        if(!temp_path) {
            raw.reset();
            close_handle(handle);
            return tl::make_unexpected(temp_path.error());
        }
        effective_config_path = *temp_path;
        effective_config_string = effective_config_path->string();
    }

    const auto configured = raw->configure(effective_config_string);
    if(effective_config_path) {
        std::error_code ec;
        std::filesystem::remove(*effective_config_path, ec);
    }
    if(!configured) {
        raw.reset();
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::CONFIGURE_FAILED);
    }

    return std::unique_ptr<MotorBus>(std::make_unique<LoadedMotorBus>(handle, raw.release(), destroy));
}

} // namespace serial_arm
