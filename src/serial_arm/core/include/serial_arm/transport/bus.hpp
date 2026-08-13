#pragma once

#include "serial_arm/transport/can.hpp"
#include "tl/expected.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace serial_arm::transport {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

constexpr std::size_t DEFAULT_CAN_CHANNEL_MAX_PENDING_FRAMES = 256;

/**
 * @brief CAN 通道轻量运行统计
 */
struct CanChannelDiagnostics {
    std::size_t pending_frames{ 0 };      ///< 当前待读帧数
    std::size_t max_pending_frames{ 0 };  ///< 通道队列上限
    std::uint64_t received_frames{ 0 };   ///< 已分发到本通道的总帧数
    std::uint64_t dropped_frames{ 0 };    ///< 因队列达到上限而丢弃的最旧帧数
};

/**
 * @brief 进程内共享总线物理资源类型
 */
enum class BusResourceKind {
    CAN,    ///< CAN 或 CAN 适配器物理端点
    SERIAL, ///< 串口或 RS485 物理端点
};

/**
 * @brief 进程内共享总线物理资源描述
 *
 * `physical_id` 表示 Bus 实现使用的物理端点，例如 `can0` 或 `/dev/ttyACM0`
 * `ownership_key` 表示真正需要进程内唯一持有的底层资源；为空时回退使用 physical_id
 * 不同 Bus 类型如果最终访问同一个 tty，应提供相同 ownership key
 * `config_signature` 必须非空，并包含 provider/backend identity 和会影响多使用者安全共存的物理通信参数
 */
struct BusResourceDescriptor {
    BusResourceKind kind{ BusResourceKind::CAN }; ///< Bus 资源类型
    std::string physical_id;                      ///< Bus 使用的物理端点标识
    std::string config_signature;                 ///< provider identity 与物理通信参数签名
    std::string ownership_key;                    ///< 底层物理资源唯一所有权键
};

/**
 * @brief 共享总线注册表错误类型
 */
enum class BusRegistryErr {
    INVALID_ARGUMENT,           ///< logical name、resource、provider 或 acquisition 配置非法
    CREATE_FAILED,              ///< creator/open 失败、抛出异常或未返回 Bus 实例
    CONFIG_CONFLICT,            ///< 同一 logical bus 或 physical resource 参数冲突
    TYPE_MISMATCH,              ///< 同一 logical bus 已被其他 Bus 类型占用
    PHYSICAL_RESOURCE_CONFLICT, ///< 同一物理资源已由其他 logical bus 持有
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 获取 BusResourceKind 的稳定文本名称
 * @param kind 物理资源类型
 * @return 文本名称
 */
const char* to_string(BusResourceKind kind) noexcept;

/**
 * @brief 获取 BusRegistryErr 的稳定文本名称
 * @param error BusRegistry 错误
 * @return 文本名称
 */
const char* to_string(BusRegistryErr error) noexcept;

/**
 * @brief 构造带上下文的 BusRegistry 错误信息
 * @param error BusRegistry 错误
 * @param name logical bus name
 * @param resource physical resource 描述
 * @return 包含错误类型、logical name、resource kind、physical id 和 config signature 的文本
 */
std::string bus_registry_error_message(
    BusRegistryErr error,
    const std::string& name,
    const BusResourceDescriptor& resource);

/**
 * @brief 构造 tty 物理资源唯一所有权键
 * @param device tty 设备路径
 * @return `tty:` 前缀加规范化设备路径；路径暂不存在时保留原始路径
 */
std::string tty_ownership_key(const std::string& device);

class CanBus;
class CanChannel;
class SerialBusClient;
struct SerialBusConfig;

using CanBusCreator = std::function<std::shared_ptr<CanBus>()>;

/**
 * @brief 获取或创建共享 CAN Bus 并返回独立逻辑通道
 * @param name logical CAN bus name
 * @param resource physical CAN resource 描述
 * @param creator 不存在可复用 CAN Bus 时调用的物理 Bus 创建函数
 * @param filters CAN ID 过滤规则，空列表表示接收所有 CAN 帧
 * @param max_pending_frames 本通道待读队列上限
 * @return 成功时返回 CanChannel；失败时返回 BusRegistry 错误
 *
 * 协议与 Hardware consumer 应只持有返回的 CanChannel，不应持有 physical CanBus
 * CanBus 的创建、复用与 physical resource 唯一所有权由本函数和 BusRegistry 内部协调
 */
tl::expected<std::shared_ptr<CanChannel>, BusRegistryErr> acquire_can_channel(
    const std::string& name,
    const BusResourceDescriptor& resource,
    const CanBusCreator& creator,
    std::vector<CanFilter> filters = {},
    std::size_t max_pending_frames = DEFAULT_CAN_CHANNEL_MAX_PENDING_FRAMES);

/**
 * @brief 获取共享串行总线协议 client
 * @param name logical serial bus name
 * @param config 串行总线配置
 * @return 成功时返回 transaction-only SerialBusClient
 */
tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
    const std::string& name,
    const SerialBusConfig& config);

/**
 * @brief 同进程共享 CAN 总线抽象
 */
class CanBus : public std::enable_shared_from_this<CanBus> {
public:
    using SharedPtr = std::shared_ptr<CanBus>;

    virtual ~CanBus() = default;

    /**
     * @brief 打开底层 CAN 总线
     * @return 成功时返回空结果，否则返回 CAN 错误
     */
    virtual tl::expected<void, CanErr> open() = 0;

    /**
     * @brief 关闭底层 CAN 总线
     */
    virtual void close() noexcept = 0;

    /**
     * @brief 查询底层 CAN 总线是否已打开
     * @return 已打开时返回 true
     */
    virtual bool is_open() const noexcept = 0;

    /**
     * @brief 向底层 CAN 总线发送一帧
     * @param frame 待发送 CAN 帧
     * @return 成功时返回空结果，否则返回 CAN 错误
     */
    virtual tl::expected<void, CanErr> send(const CanFrame& frame) = 0;

    /**
     * @brief 从底层 CAN 总线接收一帧
     * @param timeout 物理接收超时时间
     * @return 成功时返回 CAN 帧，否则返回 CAN 错误
     */
    virtual tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout) = 0;

    /**
     * @brief 清空底层 CAN 总线缓冲区
     */
    virtual void flush() noexcept = 0;

    /**
     * @brief 创建 CAN 通道
     * @param filters CAN ID 过滤规则，空列表表示接收所有 CAN 帧
     * @param max_pending_frames 本通道待读队列上限，达到上限时丢弃最旧帧
     * @return CAN 通道
     * @note 具体 CanBus 必须由 std::shared_ptr 持有后才能调用本函数
     */
    std::shared_ptr<CanChannel> create_channel(
        std::vector<CanFilter> filters = {},
        std::size_t max_pending_frames = DEFAULT_CAN_CHANNEL_MAX_PENDING_FRAMES);

private:
    friend class CanChannel;

    /**
     * @brief 在已经持有 rx_mutex_ 时执行一次物理接收和逻辑分发
     * @param timeout 本次物理接收超时时间
     * @return 物理接收成功或未匹配通道时返回空结果，否则返回 CAN 错误
     */
    tl::expected<void, CanErr> dispatch_once_locked(std::chrono::milliseconds timeout);

    /**
     * @brief 注册逻辑 CAN 通道
     * @param channel 待注册通道
     */
    void register_channel(const std::shared_ptr<CanChannel>& channel);

private:
    std::vector<std::weak_ptr<CanChannel>> channels_;   ///< 已注册 CAN 通道
    std::mutex channels_mutex_;                         ///< 通道列表互斥锁
    std::mutex tx_mutex_;                               ///< 物理 CAN 发送互斥锁
    std::mutex rx_mutex_;                               ///< 物理 CAN 接收互斥锁
};

/**
 * @brief 共享物理 CAN 总线的逻辑通道
 */
class CanChannel {
public:
    /**
     * @brief 发送 CAN 帧
     * @param frame CAN 帧
     * @return 如果发送成功，则返回空结果，否则返回错误类型
     */
    tl::expected<void, CanErr> send(const CanFrame& frame);

    /**
     * @brief 接收属于本通道的 CAN 帧
     * @param timeout 等待本通道 CAN 帧的总超时时间
     * @return 如果接收成功，则返回 CAN 帧，否则返回错误类型
     */
    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout);

    /**
     * @brief 清空本通道待读队列
     * @note 只清理当前逻辑通道，不影响同一物理总线上的其他 CanChannel
     */
    void flush() noexcept;

    /**
     * @brief 获取本通道轻量运行统计
     * @return 当前队列长度、队列上限、累计接收帧数和累计丢帧数
     */
    CanChannelDiagnostics diagnostics() const noexcept;

    /**
     * @brief 判断 CAN 帧是否匹配本通道过滤规则
     * @param frame CAN 帧
     * @return 匹配时返回 true
     */
    bool accepts(const CanFrame& frame) const noexcept;

private:
    friend class CanBus;

    /**
     * @brief 创建逻辑 CAN 通道
     * @param bus 所属共享 CAN 总线
     * @param filters CAN ID 过滤规则
     * @param max_pending_frames 待读队列上限
     */
    CanChannel(
        std::shared_ptr<CanBus> bus,
        std::vector<CanFilter> filters,
        std::size_t max_pending_frames);

    /**
     * @brief 弹出本通道已分发的待读帧
     * @return 有待读帧时返回 CAN 帧，否则返回 TIMEOUT
     */
    tl::expected<CanFrame, CanErr> pop_pending();

    /**
     * @brief 将已匹配的 CAN 帧加入本通道待读队列
     * @param frame 已匹配 CAN 帧
     */
    void enqueue(const CanFrame& frame);

private:
    std::shared_ptr<CanBus> bus_;                   ///< 所属 CAN 总线
    std::vector<CanFilter> filters_;                ///< CAN ID 过滤规则
    std::deque<CanFrame> pending_;                  ///< 已分发到本通道的待读帧
    const std::size_t max_pending_frames_;          ///< 待读队列上限
    std::uint64_t received_frames_{ 0 };            ///< 累计分发帧数
    std::uint64_t dropped_frames_{ 0 };             ///< 累计丢弃的最旧帧数
    mutable std::mutex mutex_;                      ///< 通道待读队列互斥锁
    std::condition_variable pending_cv_;            ///< 通道待读队列通知
};

/**
 * @brief 同进程共享总线注册表
 *
 * BusRegistry 只管理 logical bus name 到 Bus 实例的映射，以及 physical resource
 * 的进程内唯一所有权；它不理解任何具体设备、协议或机器人业务语义
 */
class BusRegistry final {
private:
    template<typename BusT>
    using BusCreator = std::function<std::shared_ptr<BusT>()>;

    using ErasedBusCreator = std::function<std::shared_ptr<void>()>;

    enum class EntryState {
        CREATING, ///< creator 正在 Registry 锁外创建 Bus
        READY,    ///< Bus 已创建并可复用
    };

    struct Entry {
        std::weak_ptr<void> bus;              ///< READY 状态下的 Bus 弱引用
        std::type_index type;                 ///< Bus 具体类型
        BusResourceDescriptor resource;       ///< physical resource 描述
        EntryState state{ EntryState::CREATING }; ///< 当前创建状态
        std::thread::id creator_thread{};     ///< CREATING 状态的 creator 线程
    };

    /**
     * @brief 原子获取或创建指定类型的共享 Bus
     * @param name logical bus name；同名且同类型同配置时返回同一实例
     * @param resource physical resource 描述；同一物理端点在一个进程内只能有一个 owner
     * @param creator 不存在可复用 Bus 时调用的创建函数
     * @return 成功时返回共享 Bus；失败时返回冲突或创建错误
     *
     * Registry 使用创建 reservation 保证同一 logical bus 并发只执行一次 creator
     * creator 在 Registry 全局互斥锁之外执行，避免慢 open 阻塞无关 Bus acquisition 和 creator 重入死锁
     *
     * 本接口仅供 Core acquisition helper 使用，不作为 Protocol / Hardware consumer API
     * consumer 应通过 acquire_can_channel() 或 acquire_serial_bus_client() 获取受限访问对象
     */
    template<typename BusT>
    static tl::expected<std::shared_ptr<BusT>, BusRegistryErr> get_or_create(
        const std::string& name,
        const BusResourceDescriptor& resource,
        const BusCreator<BusT>& creator) {
        auto erased = get_or_create_erased(name, resource, std::type_index(typeid(BusT)), [&]() {
            return std::static_pointer_cast<void>(creator());
            });
        if(!erased) return tl::make_unexpected(erased.error());
        return std::static_pointer_cast<BusT>(*erased);
    }

    static tl::expected<std::shared_ptr<void>, BusRegistryErr> get_or_create_erased(
        const std::string& name,
        const BusResourceDescriptor& resource,
        std::type_index type,
        const ErasedBusCreator& creator);
    static std::unordered_map<std::string, Entry>& buses();
    static std::unordered_map<std::string, std::string>& physical_owners();
    static std::mutex& mutex();
    static std::condition_variable& condition();

    friend tl::expected<std::shared_ptr<CanChannel>, BusRegistryErr> acquire_can_channel(
        const std::string& name,
        const BusResourceDescriptor& resource,
        const CanBusCreator& creator,
        std::vector<CanFilter> filters,
        std::size_t max_pending_frames);
    friend tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
        const std::string& name,
        const SerialBusConfig& config);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
