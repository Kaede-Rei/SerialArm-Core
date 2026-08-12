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
 * `physical_id` 表示同一进程内必须唯一持有的物理端点，例如 `can0`
 * 或 `/dev/ttyACM0`；`config_signature` 由具体 Bus 实现生成，用于描述
 * baudrate、data bits、parity、stop bits、flow control 等会影响共存安全
 * 的通信参数。
 */
struct BusResourceDescriptor {
    BusResourceKind kind{ BusResourceKind::CAN }; ///< 物理资源类型
    std::string physical_id;                      ///< 物理端点标识
    std::string config_signature;                 ///< 通信参数签名
};

/**
 * @brief 共享总线注册表错误类型
 */
enum class BusRegistryErr {
    INVALID_ARGUMENT,           ///< logical bus name 或 physical resource 为空
    CREATE_FAILED,              ///< 创建函数未返回 Bus 实例
    CONFIG_CONFLICT,            ///< 同一 logical bus 或 physical resource 参数冲突
    TYPE_MISMATCH,              ///< 同一 logical bus 已被其他 Bus 类型占用
    PHYSICAL_RESOURCE_CONFLICT, ///< 同一物理资源已由其他 logical bus 持有
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class CanChannel;

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
 * @brief 同进程 CAN 总线共享池
 */
class BusPool final {
public:
    using BusCreator = std::function<std::shared_ptr<CanBus>()>;

    /**
     * @brief 原子获取或创建共享 CAN 总线
     * @param name 总线名称
     * @param creator 总线创建函数
     * @return 如果存在则返回已有总线，否则创建、保存并返回新总线
     */
    static std::shared_ptr<CanBus> get_or_create(const std::string& name, const BusCreator& creator);

private:
    static std::unordered_map<std::string, std::weak_ptr<CanBus>>& buses();
    static std::mutex& mutex();
};

/**
 * @brief 同进程共享总线注册表
 *
 * BusRegistry 只管理 logical bus name 到 Bus 实例的映射，以及 physical resource
 * 的进程内唯一所有权；它不理解 Damiao、Hiwonder、Gripper 或 Tool Button 等业务语义。
 */
class BusRegistry final {
public:
    template<typename BusT>
    using BusCreator = std::function<std::shared_ptr<BusT>()>;

    using CanBusCreator = std::function<std::shared_ptr<CanBus>()>;

    /**
     * @brief 原子获取或创建指定类型的共享 Bus
     * @param name logical bus name；同名且同类型同配置时返回同一实例
     * @param resource physical resource 描述；同一物理端点在一个进程内只能有一个 owner
     * @param creator 不存在可复用 Bus 时调用的创建函数
     * @return 成功时返回共享 Bus；失败时返回冲突或创建错误
     *
     * Registry 只保存弱引用；调用方返回的 `std::shared_ptr` 决定 Bus 生命周期。
     * 最后一个 Bus 引用释放后，下一次访问 Registry 时会清理对应 physical resource
     * 占用记录；该函数内部加锁，多个线程并发获取同一 logical bus 不会创建多个
     * physical owner；创建函数会在锁内执行，创建函数不得递归调用 BusRegistry。
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

    /**
     * @brief 原子获取或创建共享 CAN Bus
     * @param name logical CAN bus name
     * @param resource physical CAN resource 描述
     * @param creator 不存在可复用 CAN Bus 时调用的创建函数
     * @return 成功时返回共享 CanBus；失败时返回冲突或创建错误
     *
     * 返回 Bus 由调用方通过 `std::shared_ptr` 共享持有；函数内部线程安全，并保证
     * 同一 physical CAN resource 不会被不同 logical bus 重复持有。
     */
    static tl::expected<std::shared_ptr<CanBus>, BusRegistryErr> get_or_create_can_bus(
        const std::string& name,
        const BusResourceDescriptor& resource,
        const CanBusCreator& creator);

private:
    using ErasedBusCreator = std::function<std::shared_ptr<void>()>;

    struct Entry {
        std::weak_ptr<void> bus;              ///< Bus 弱引用
        std::type_index type;                 ///< Bus 具体类型
        BusResourceDescriptor resource;       ///< physical resource 描述
    };

    static tl::expected<std::shared_ptr<void>, BusRegistryErr> get_or_create_erased(
        const std::string& name,
        const BusResourceDescriptor& resource,
        std::type_index type,
        const ErasedBusCreator& creator);
    static std::unordered_map<std::string, Entry>& buses();
    static std::unordered_map<std::string, std::string>& physical_owners();
    static std::mutex& mutex();
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
