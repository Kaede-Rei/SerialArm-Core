#pragma once

#include "serial_arm/transport/bus.hpp"
#include "serial_arm/transport/serial_port.hpp"

#include <functional>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

namespace serial_arm::transport {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 共享串行总线运行配置
 */
struct SerialBusConfig {
    std::string serial_port;          ///< 串口设备路径
    SerialPort::Config port_config{}; ///< POSIX 串口参数
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 同进程共享串行总线
 *
 * SerialBus 唯一持有一个 SerialPort，并通过 transaction() 将串口访问串行化。
 * 协议层在 transaction callback 内完成自己的 write/read/flush/timeout/framing
 * 逻辑；SerialBus 不解析任何具体协议，也不提供绕过仲裁的 public raw write/read
 * 接口。
 */
class SerialBus final {
public:
    using Config = SerialBusConfig;

    /**
     * @brief 创建未打开的共享串行总线
     * @param config 串口设备路径和 POSIX 串口参数
     *
     * 构造函数不打开物理设备；调用 open() 后 SerialBus 成为 SerialPort 唯一 owner。
     * 对同一 SerialBus 实例的 transaction() 调用线程安全并按调用进入顺序互斥执行。
     */
    explicit SerialBus(Config config);

    /**
     * @brief 关闭串口并释放资源
     */
    ~SerialBus() noexcept;

    SerialBus(const SerialBus&) = delete;
    SerialBus& operator=(const SerialBus&) = delete;
    SerialBus(SerialBus&&) = delete;
    SerialBus& operator=(SerialBus&&) = delete;

    /**
     * @brief 打开底层串口
     *
     * 重复调用 open() 不会重复打开已经打开的 SerialPort。open() 与 transaction()
     * 共用同一互斥锁，避免打开或关闭动作与协议事务交叉。
     */
    void open();

    /**
     * @brief 关闭底层串口
     *
     * close() 只由 SerialBus 生命周期或 owner 显式调用；client 不应在 transaction
     * callback 内关闭串口。该函数与 transaction() 互斥。
     */
    void close() noexcept;

    /**
     * @brief 查询串口是否已打开
     * @return 已打开时返回 true
     */
    bool is_open() const noexcept;

    /**
     * @brief 获取共享串行总线配置
     * @return 配置只读引用
     */
    const Config& config() const noexcept;

    /**
     * @brief 获取 physical resource descriptor
     * @return 可传给 BusRegistry 的串口物理资源描述
     */
    BusResourceDescriptor resource_descriptor() const;

    /**
     * @brief 根据配置生成 physical resource descriptor
     * @param config 串口设备路径和 POSIX 串口参数
     * @return 可传给 BusRegistry 的串口物理资源描述
     *
     * 该函数不打开串口，也不创建 SerialBus 实例。调用方可在 Registry 创建 Bus
     * 前用它进行 physical resource 唯一所有权检查。
     */
    static BusResourceDescriptor resource_descriptor(const Config& config);

    /**
     * @brief 执行一次串口事务
     * @param fn 协议 callback，参数为 SerialPort&
     * @return callback 的返回值
     *
     * transaction() 在调用 callback 前获取总线互斥锁，并在 callback 正常返回或抛出
     * 异常时自动释放。callback 独占 SerialPort，可执行 write/read/flush/timeout
     * 处理；其他 client 必须等待本事务结束。该函数不捕获异常，异常会原样传给调用方。
     */
    template<typename Fn>
    auto transaction(Fn&& fn) -> std::invoke_result_t<Fn, SerialPort&> {
        using Result = std::invoke_result_t<Fn, SerialPort&>;
        std::lock_guard<std::mutex> lock(mutex_);
        if constexpr(std::is_void_v<Result>) {
            std::invoke(std::forward<Fn>(fn), serial_);
        }
        else {
            return std::invoke(std::forward<Fn>(fn), serial_);
        }
    }

private:
    Config config_;              ///< 串行总线配置
    SerialPort serial_;          ///< 唯一持有的底层串口
    mutable std::mutex mutex_;   ///< 串口生命周期和事务互斥锁
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
