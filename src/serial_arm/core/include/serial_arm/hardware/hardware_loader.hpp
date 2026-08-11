#pragma once

#include <tl/expected.hpp>

#include "serial_arm/hardware/motor_bus.hpp"

#include <memory>
#include <optional>
#include <string>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class HardwareLoaderErr {
    OPEN_FAILED,       ///< 无法打开 Hardware Backend 共享库
    SYMBOL_FAILED,     ///< 共享库缺少 create_motor_bus 或 destroy_motor_bus
    CREATE_FAILED,     ///< create_motor_bus 未能创建 MotorBus 实例
    CONFIGURE_FAILED,  ///< MotorBus::configure() 失败
    CONFIG_OPEN_FAILED,      ///< Hardware YAML 无法打开
    CONFIG_SYNTAX_ERROR,     ///< Hardware YAML 语法错误
    INVALID_OVERRIDE,        ///< runtime override 参数非法
};

/**
 * @brief Hardware YAML 运行时覆盖项
 *
 * 空 optional 表示不覆盖对应字段；覆盖只作用于本次 load() 调用，不会写回原始
 * hardware.yaml
 */
struct HardwareConfigOverrides {
    std::optional<std::string> serial_port;  ///< 覆盖 serial_port
    std::optional<int> baudrate;             ///< 覆盖 baudrate
    std::optional<std::string> bus;          ///< 覆盖 bus
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class HardwareLoader {
public:
    /**
     * @brief Hardware Backend 共享库加载器
     *
     * Loader 负责 dlopen() Backend shared library，通过 create_motor_bus()
     * 创建 MotorBus，并将 destroy_motor_bus() 与 dlclose() 生命周期绑定到
     * 返回的 std::unique_ptr<MotorBus> 中
     */
    HardwareLoader() = default;
    ~HardwareLoader();

    HardwareLoader(const HardwareLoader&) = delete;
    HardwareLoader& operator=(const HardwareLoader&) = delete;
    HardwareLoader(HardwareLoader&& other) noexcept;
    HardwareLoader& operator=(HardwareLoader&& other) noexcept;

    /**
     * @brief 加载 Hardware Backend 并使用指定配置完成 configure()
     * @param plugin 共享库路径，或不含路径的插件名
     * @param config_path Backend 专属 YAML 配置路径
     * @return 持有 Backend 对象和共享库句柄生命周期的 MotorBus
     *
     * 当 plugin 不含路径分隔符时，Loader 会额外尝试 lib<plugin>.so
     * 返回对象析构时先调用插件 destroy_motor_bus()，再 dlclose()；Robot、
     * ROS 2、Python 和 Terminal 不需要感知 DSO 生命周期
     */
    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(const std::string& plugin, const std::string& config_path);
    /**
     * @brief 加载 Hardware Backend，并在 configure() 前应用 runtime overrides
     * @param plugin 共享库路径，或不含路径的插件名
     * @param config_path Backend 专属 YAML 配置路径
     * @param overrides 本次运行的硬件连接参数覆盖项
     */
    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(
        const std::string& plugin,
        const std::string& config_path,
        const HardwareConfigOverrides& overrides);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
