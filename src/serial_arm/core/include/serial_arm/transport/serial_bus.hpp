#pragma once

#include "serial_arm/transport/bus.hpp"
#include "serial_arm/transport/serial_port.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
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
    SerialPort::Config port_config{}; ///< POSIX 串口参数和默认事务超时
};

/**
 * @brief 单次串行事务超时配置
 */
struct SerialTransactionOptions {
    std::chrono::milliseconds read_timeout{ 2 };    ///< 本事务默认读取超时
    std::chrono::milliseconds write_timeout{ 100 }; ///< 本事务默认写入超时
};

/**
 * @brief 共享串行总线轻量运行统计
 */
struct SerialBusDiagnostics {
    bool is_open{ false };                         ///< 当前串口是否已打开
    std::uint64_t transaction_count{ 0 };          ///< 已进入的 transaction 次数
    std::uint64_t failed_transaction_count{ 0 };   ///< callback 抛出异常的 transaction 次数
    BusResourceDescriptor resource;                ///< physical resource 描述
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 单次共享串行总线事务的受限访问视图
 *
 * SerialTransaction 只暴露协议执行所需的 read/write/flush/drain/available 接口
 * 不暴露 SerialPort 的 open/close/set_config/native_handle 等物理资源管理能力
 * 因此协议 client 无法绕过 SerialBus 修改共享串口生命周期或持久通信配置
 */
class SerialTransaction final {
public:
    using Byte = SerialPort::Byte;                    ///< 串口字节类型
    using Buffer = SerialPort::Buffer;                ///< 串口字节缓冲区
    using FlushDirection = SerialPort::FlushDirection; ///< 串口缓冲区清空方向

    SerialTransaction(const SerialTransaction&) = delete;
    SerialTransaction& operator=(const SerialTransaction&) = delete;
    SerialTransaction(SerialTransaction&&) = delete;
    SerialTransaction& operator=(SerialTransaction&&) = delete;

    /**
     * @brief 最多读取 len 字节
     * @param data 目标缓冲区
     * @param len 目标缓冲区长度
     * @return 实际读取字节数，超时时返回 0
     */
    std::size_t read(Byte* data, std::size_t len);

    /**
     * @brief 使用指定超时最多读取 len 字节
     * @param data 目标缓冲区
     * @param len 目标缓冲区长度
     * @param timeout 本次读取超时
     * @return 实际读取字节数，超时时返回 0
     */
    std::size_t read(Byte* data, std::size_t len, std::chrono::milliseconds timeout);

    /**
     * @brief 最多读取 max_bytes 字节并返回新缓冲区
     * @param max_bytes 最大读取字节数
     * @return 实际读取到的字节
     */
    Buffer read(std::size_t max_bytes);

    /**
     * @brief 使用指定超时最多读取 max_bytes 字节并返回新缓冲区
     * @param max_bytes 最大读取字节数
     * @param timeout 本次读取超时
     * @return 实际读取到的字节
     */
    Buffer read(std::size_t max_bytes, std::chrono::milliseconds timeout);

    /**
     * @brief 在事务默认读取超时内尽量读取 len 字节
     * @param data 目标缓冲区
     * @param len 期望读取字节数
     * @return 实际读取字节数，可能小于 len
     */
    std::size_t read_exact(Byte* data, std::size_t len);

    /**
     * @brief 使用指定超时尽量读取 len 字节
     * @param data 目标缓冲区
     * @param len 期望读取字节数
     * @param timeout 本次读取超时
     * @return 实际读取字节数，可能小于 len
     */
    std::size_t read_exact(Byte* data, std::size_t len, std::chrono::milliseconds timeout);

    /**
     * @brief 在事务默认读取超时内尽量读取 len 字节并返回新缓冲区
     * @param len 期望读取字节数
     * @return 实际读取到的字节
     */
    Buffer read_exact(std::size_t len);

    /**
     * @brief 使用指定超时尽量读取 len 字节并返回新缓冲区
     * @param len 期望读取字节数
     * @param timeout 本次读取超时
     * @return 实际读取到的字节
     */
    Buffer read_exact(std::size_t len, std::chrono::milliseconds timeout);

    /**
     * @brief 在事务默认写入超时内尽量写出 len 字节
     * @param data 待写缓冲区
     * @param len 待写字节数
     * @return 实际写出字节数，可能小于 len
     */
    std::size_t write(const Byte* data, std::size_t len);

    /**
     * @brief 使用指定超时尽量写出 len 字节
     * @param data 待写缓冲区
     * @param len 待写字节数
     * @param timeout 本次写入超时
     * @return 实际写出字节数，可能小于 len
     */
    std::size_t write(const Byte* data, std::size_t len, std::chrono::milliseconds timeout);

    /**
     * @brief 写出缓冲区全部内容
     * @param data 待写缓冲区
     * @return 实际写出字节数
     */
    std::size_t write(const Buffer& data);

    /**
     * @brief 使用指定超时写出缓冲区全部内容
     * @param data 待写缓冲区
     * @param timeout 本次写入超时
     * @return 实际写出字节数
     */
    std::size_t write(const Buffer& data, std::chrono::milliseconds timeout);

    /**
     * @brief 写出初始化列表内容
     * @param data 待写字节列表
     * @return 实际写出字节数
     */
    std::size_t write(std::initializer_list<Byte> data);

    /**
     * @brief 使用指定超时写出初始化列表内容
     * @param data 待写字节列表
     * @param timeout 本次写入超时
     * @return 实际写出字节数
     */
    std::size_t write(std::initializer_list<Byte> data, std::chrono::milliseconds timeout);

    /**
     * @brief 等待输出队列发送完成
     */
    void drain();

    /**
     * @brief 清空串口系统缓冲区
     * @param direction 清空方向
     */
    void flush(FlushDirection direction = FlushDirection::Both);

    /**
     * @brief 查询当前可读字节数
     * @return 内核输入缓冲区中的可读字节数
     */
    std::size_t available() const;

private:
    friend class SerialBus;

    /**
     * @brief 创建受限串行事务视图
     * @param serial SerialBus 唯一持有的底层串口
     * @param options 本事务默认超时
     */
    SerialTransaction(SerialPort& serial, SerialTransactionOptions options) noexcept;

private:
    SerialPort& serial_;               ///< SerialBus 唯一持有的底层串口
    SerialTransactionOptions options_; ///< 本事务默认超时
};

/**
 * @brief 同进程共享串行总线
 *
 * SerialBus 唯一持有一个 SerialPort，并负责串行化完整协议事务
 * 协议层不得直接持有 SerialBus，而应通过 acquire_serial_bus_client() 获取
 * SerialBusClient；总线生命周期由共享引用和 RAII 管理
 */
class SerialBusClient;

class SerialBus final {
public:
    using Config = SerialBusConfig;

    /**
     * @brief 关闭串口并释放资源
     */
    ~SerialBus() noexcept;

    SerialBus(const SerialBus&) = delete;
    SerialBus& operator=(const SerialBus&) = delete;
    SerialBus(SerialBus&&) = delete;
    SerialBus& operator=(SerialBus&&) = delete;

    /**
     * @brief 根据配置生成 physical resource descriptor
     * @param config 串口设备路径和 POSIX 串口参数
     * @return 可传给 BusRegistry 的串口物理资源描述
     *
     * read/write timeout 不属于物理兼容性签名，不同协议 client 可以为每次事务
     * 使用独立 timeout；该函数不打开串口，也不创建 SerialBus 实例
     */
    static BusResourceDescriptor resource_descriptor(const Config& config);

private:
    friend class SerialBusClient;
    friend tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
        const std::string& name,
        const Config& config);

    /**
     * @brief 创建未打开的共享串行总线
     * @param config 串口设备路径、物理串口参数和默认 client 超时
     */
    explicit SerialBus(Config config);

    /**
     * @brief 打开底层串口
     */
    void open();

    /**
     * @brief 关闭底层串口
     */
    void close() noexcept;

    /**
     * @brief 查询串口是否已打开
     * @return 已打开时返回 true
     */
    bool is_open() const noexcept;

    /**
     * @brief 获取共享串行总线轻量运行统计
     * @return 当前 open 状态、transaction 计数、失败计数和 physical resource
     */
    SerialBusDiagnostics diagnostics() const;

    /**
     * @brief 使用指定超时执行一次串口事务
     * @param options 本事务默认读写超时
     * @param fn 协议 callback，参数为受限 SerialTransaction&
     * @return callback 的返回值
     */
    template<typename Fn>
    auto transaction(const SerialTransactionOptions& options, Fn&& fn)
        -> std::invoke_result_t<Fn, SerialTransaction&> {
        using Result = std::invoke_result_t<Fn, SerialTransaction&>;
        validate_transaction_options(options);
        std::lock_guard<std::mutex> lock(mutex_);
        ++transaction_count_;
        SerialTransaction transaction(serial_, options);
        try {
            if constexpr(std::is_void_v<Result>) {
                std::invoke(std::forward<Fn>(fn), transaction);
            }
            else {
                return std::invoke(std::forward<Fn>(fn), transaction);
            }
        }
        catch(...) {
            ++failed_transaction_count_;
            throw;
        }
    }

    /**
     * @brief 校验事务超时配置
     * @param options 待校验事务配置
     */
    static void validate_transaction_options(const SerialTransactionOptions& options);

private:
    Config config_;              ///< 串行总线配置
    SerialPort serial_;          ///< 唯一持有的底层串口
    mutable std::mutex mutex_;   ///< 串口生命周期和事务互斥锁
    std::atomic<std::uint64_t> transaction_count_{ 0 };        ///< 已进入 transaction 次数
    std::atomic<std::uint64_t> failed_transaction_count_{ 0 }; ///< callback 抛出异常次数
};

/**
 * @brief 共享串行总线协议 client
 *
 * SerialBusClient 只持有共享总线引用和当前 client 的默认事务超时
 * 对协议层只暴露 transaction() 和只读运行状态，不暴露 open/close/config/raw port
 * 因此一个协议 client 无法修改共享物理总线生命周期或其他 client 的持久状态
 */
class SerialBusClient final {
public:
    SerialBusClient(const SerialBusClient&) = delete;
    SerialBusClient& operator=(const SerialBusClient&) = delete;
    SerialBusClient(SerialBusClient&&) = delete;
    SerialBusClient& operator=(SerialBusClient&&) = delete;

    /**
     * @brief 使用当前 client 默认超时执行一次串口事务
     * @param fn 协议 callback，参数为受限 SerialTransaction&
     * @return callback 的返回值
     */
    template<typename Fn>
    auto transaction(Fn&& fn) -> std::invoke_result_t<Fn, SerialTransaction&> {
        return bus_->transaction(options_, std::forward<Fn>(fn));
    }

    /**
     * @brief 使用指定超时执行一次串口事务
     * @param options 本事务默认读写超时
     * @param fn 协议 callback，参数为受限 SerialTransaction&
     * @return callback 的返回值
     */
    template<typename Fn>
    auto transaction(const SerialTransactionOptions& options, Fn&& fn)
        -> std::invoke_result_t<Fn, SerialTransaction&> {
        return bus_->transaction(options, std::forward<Fn>(fn));
    }

    /**
     * @brief 查询共享串行总线是否已打开
     * @return 已打开时返回 true
     */
    bool is_open() const noexcept;

    /**
     * @brief 获取共享串行总线轻量运行统计
     * @return 当前 open 状态、transaction 计数、失败计数和 physical resource
     */
    SerialBusDiagnostics diagnostics() const;

private:
    friend tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
        const std::string& name,
        const SerialBusConfig& config);

    /**
     * @brief 创建协议层共享串行 client
     * @param bus 共享物理串行总线
     * @param options 当前 client 默认事务超时
     */
    SerialBusClient(std::shared_ptr<SerialBus> bus, SerialTransactionOptions options) noexcept;

private:
    std::shared_ptr<SerialBus> bus_;    ///< 共享串行总线，仅用于保持 owner 生命周期
    SerialTransactionOptions options_; ///< 当前 client 默认事务超时
};

/**
 * @brief 获取共享串行总线协议 client
 * @param name logical serial bus name
 * @param config 串行总线物理配置和当前 client 默认事务超时
 * @return 成功时返回 transaction-only client；失败时返回 BusRegistry 错误
 *
 * 同名且 physical resource 等价、物理通信参数一致时复用同一个 SerialBus
 * `read_timeout` 和 `write_timeout` 只保存到返回 client，不参与物理兼容性判断
 */
tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
    const std::string& name,
    const SerialBusConfig& config);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
