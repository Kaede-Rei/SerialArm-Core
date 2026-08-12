#include "serial_arm/transport/bus.hpp"

#include <algorithm>

namespace serial_arm::transport {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr auto DISPATCH_SLICE = std::chrono::milliseconds(2);

std::chrono::milliseconds min_timeout(std::chrono::milliseconds lhs, std::chrono::milliseconds rhs) {
    return lhs < rhs ? lhs : rhs;
}

const char* resource_kind_name(BusResourceKind kind) noexcept {
    switch(kind) {
        case BusResourceKind::CAN: return "can";
        case BusResourceKind::SERIAL: return "serial";
    }
    return "unknown";
}

std::string physical_resource_key(const BusResourceDescriptor& resource) {
    return std::string(resource_kind_name(resource.kind)) + '\n' + resource.physical_id;
}

bool resource_matches(const BusResourceDescriptor& lhs, const BusResourceDescriptor& rhs) {
    return lhs.kind == rhs.kind &&
        lhs.physical_id == rhs.physical_id &&
        lhs.config_signature == rhs.config_signature;
}

} // namespace


// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 创建 CAN 通道
 */
std::shared_ptr<CanChannel> CanBus::create_channel(
    std::vector<CanFilter> filters,
    std::size_t max_pending_frames) {
    std::shared_ptr<CanChannel> channel(new CanChannel(
        shared_from_this(),
        std::move(filters),
        std::max<std::size_t>(1, max_pending_frames)));
    register_channel(channel);
    return channel;
}

/**
 * @brief 执行一次物理接收并分发到匹配通道
 */
tl::expected<void, CanErr> CanBus::dispatch_once_locked(std::chrono::milliseconds timeout) {
    auto frame = receive(timeout);
    if(!frame) return tl::make_unexpected(frame.error());

    std::lock_guard<std::mutex> channels_lock(channels_mutex_);
    channels_.erase(std::remove_if(channels_.begin(), channels_.end(),
        [](const std::weak_ptr<CanChannel>& channel) {
            return channel.expired();
        }), channels_.end());

    for(const auto& weak_channel : channels_) {
        auto channel = weak_channel.lock();
        if(channel && channel->accepts(*frame)) {
            channel->enqueue(*frame);
        }
    }

    return {};
}

/**
 * @brief 注册逻辑 CAN 通道
 */
void CanBus::register_channel(const std::shared_ptr<CanChannel>& channel) {
    std::lock_guard<std::mutex> lock(channels_mutex_);
    channels_.push_back(channel);
}

/**
 * @brief 创建逻辑 CAN 通道
 */
CanChannel::CanChannel(
    std::shared_ptr<CanBus> bus,
    std::vector<CanFilter> filters,
    std::size_t max_pending_frames)
    : bus_(std::move(bus)),
      filters_(std::move(filters)),
      max_pending_frames_(max_pending_frames) {
}

/**
 * @brief 发送 CAN 帧
 */
tl::expected<void, CanErr> CanChannel::send(const CanFrame& frame) {
    if(!bus_) return tl::make_unexpected(CanErr::NOT_OPEN);
    std::lock_guard<std::mutex> lock(bus_->tx_mutex_);
    return bus_->send(frame);
}

/**
 * @brief 接收属于本通道的 CAN 帧
 */
tl::expected<CanFrame, CanErr> CanChannel::receive(std::chrono::milliseconds timeout) {
    if(!bus_) return tl::make_unexpected(CanErr::NOT_OPEN);

    auto pending = pop_pending();
    if(pending) return pending;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(true) {
        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) return tl::make_unexpected(CanErr::TIMEOUT);

        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        const auto physical_timeout = min_timeout(remaining, DISPATCH_SLICE);

        if(bus_->rx_mutex_.try_lock()) {
            std::unique_lock<std::mutex> rx_lock(bus_->rx_mutex_, std::adopt_lock);
            pending = pop_pending();
            if(pending) return pending;
            auto dispatched = bus_->dispatch_once_locked(physical_timeout);
            rx_lock.unlock();

            if(!dispatched && dispatched.error() != CanErr::TIMEOUT) return tl::make_unexpected(dispatched.error());
        }
        else {
            std::unique_lock<std::mutex> lock(mutex_);
            pending_cv_.wait_for(lock, physical_timeout, [this]() {
                return !pending_.empty();
                });
        }

        pending = pop_pending();
        if(pending) return pending;
    }
}

/**
 * @brief 清空本通道待读队列
 */
void CanChannel::flush() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
}

/**
 * @brief 获取本通道轻量运行统计
 */
CanChannelDiagnostics CanChannel::diagnostics() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    CanChannelDiagnostics value;
    value.pending_frames = pending_.size();
    value.max_pending_frames = max_pending_frames_;
    value.received_frames = received_frames_;
    value.dropped_frames = dropped_frames_;
    return value;
}

/**
 * @brief 判断 CAN 帧是否匹配本通道过滤规则
 */
bool CanChannel::accepts(const CanFrame& frame) const noexcept {
    if(filters_.empty()) return true;
    for(const auto& filter : filters_) {
        if((frame.id & filter.mask) == (filter.id & filter.mask)) return true;
    }
    return false;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 弹出本通道已分发的待读帧
 */
tl::expected<CanFrame, CanErr> CanChannel::pop_pending() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(pending_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
    CanFrame frame = pending_.front();
    pending_.pop_front();
    return frame;
}

/**
 * @brief 将匹配帧加入本通道待读队列
 */
void CanChannel::enqueue(const CanFrame& frame) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++received_frames_;
        if(pending_.size() >= max_pending_frames_) {
            pending_.pop_front();
            ++dropped_frames_;
        }
        pending_.push_back(frame);
    }
    pending_cv_.notify_one();
}

/**
 * @brief 原子获取或创建共享 CAN 总线
 */
std::shared_ptr<CanBus> BusPool::get_or_create(const std::string& name, const BusCreator& creator) {
    std::lock_guard<std::mutex> lock(mutex());
    auto& pool = buses();
    auto it = pool.find(name);
    if(it != pool.end()) {
        auto bus = it->second.lock();
        if(bus) return bus;
        pool.erase(it);
    }

    auto bus = creator();
    if(bus) pool[name] = bus;
    return bus;
}

/**
 * @brief 获取进程内总线弱引用表
 */
std::unordered_map<std::string, std::weak_ptr<CanBus>>& BusPool::buses() {
    static std::unordered_map<std::string, std::weak_ptr<CanBus>> instance;
    return instance;
}

/**
 * @brief 获取总线共享池互斥锁
 */
std::mutex& BusPool::mutex() {
    static std::mutex instance;
    return instance;
}

/**
 * @brief 原子获取或创建共享 CAN Bus
 */
tl::expected<std::shared_ptr<CanBus>, BusRegistryErr> BusRegistry::get_or_create_can_bus(
    const std::string& name,
    const BusResourceDescriptor& resource,
    const CanBusCreator& creator) {
    return get_or_create<CanBus>(name, resource, creator);
}

/**
 * @brief 原子获取或创建擦除类型后的共享 Bus
 */
tl::expected<std::shared_ptr<void>, BusRegistryErr> BusRegistry::get_or_create_erased(
    const std::string& name,
    const BusResourceDescriptor& resource,
    std::type_index type,
    const ErasedBusCreator& creator) {
    if(name.empty() || resource.physical_id.empty()) {
        return tl::make_unexpected(BusRegistryErr::INVALID_ARGUMENT);
    }

    std::lock_guard<std::mutex> lock(mutex());
    auto& registry = buses();
    auto& resources = physical_owners();

    for(auto it = registry.begin(); it != registry.end();) {
        if(it->second.bus.expired()) {
            const auto key = physical_resource_key(it->second.resource);
            auto resource_it = resources.find(key);
            if(resource_it != resources.end() && resource_it->second == it->first) {
                resources.erase(resource_it);
            }
            it = registry.erase(it);
            continue;
        }
        ++it;
    }

    auto logical_it = registry.find(name);
    if(logical_it != registry.end()) {
        auto existing = logical_it->second.bus.lock();
        if(existing) {
            if(logical_it->second.type != type) return tl::make_unexpected(BusRegistryErr::TYPE_MISMATCH);
            if(!resource_matches(logical_it->second.resource, resource)) {
                return tl::make_unexpected(BusRegistryErr::CONFIG_CONFLICT);
            }
            return existing;
        }
        registry.erase(logical_it);
    }

    const auto key = physical_resource_key(resource);
    auto resource_it = resources.find(key);
    if(resource_it != resources.end()) {
        auto owner_it = registry.find(resource_it->second);
        if(owner_it != registry.end() && !owner_it->second.bus.expired()) {
            if(!resource_matches(owner_it->second.resource, resource)) {
                return tl::make_unexpected(BusRegistryErr::CONFIG_CONFLICT);
            }
            return tl::make_unexpected(BusRegistryErr::PHYSICAL_RESOURCE_CONFLICT);
        }
        resources.erase(resource_it);
    }

    auto bus = creator();
    if(!bus) return tl::make_unexpected(BusRegistryErr::CREATE_FAILED);

    registry.emplace(name, Entry{ bus, type, resource });
    resources[key] = name;
    return bus;
}

/**
 * @brief 获取进程内共享 Bus 弱引用表
 */
std::unordered_map<std::string, BusRegistry::Entry>& BusRegistry::buses() {
    static std::unordered_map<std::string, Entry> instance;
    return instance;
}

/**
 * @brief 获取 physical resource owner 表
 */
std::unordered_map<std::string, std::string>& BusRegistry::physical_owners() {
    static std::unordered_map<std::string, std::string> instance;
    return instance;
}

/**
 * @brief 获取 BusRegistry 互斥锁
 */
std::mutex& BusRegistry::mutex() {
    static std::mutex instance;
    return instance;
}

} // namespace serial_arm::transport
