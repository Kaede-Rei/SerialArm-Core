#include "serial_arm_protocol_damiao_usb2can/bus.hpp"

#include <algorithm>
#include <cstring>
#include <memory>

namespace serial_arm::protocol::damiao_usb2can {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 计算距离 deadline 的剩余毫秒数
 */
std::chrono::milliseconds remaining_time(const std::chrono::steady_clock::time_point& deadline) {
    const auto now = std::chrono::steady_clock::now();
    if(now >= deadline) return std::chrono::milliseconds{ 0 };
    const auto remaining = deadline - now;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    if(ms.count() == 0 && remaining > std::chrono::steady_clock::duration::zero()) {
        ms = std::chrono::milliseconds{ 1 };
    }
    return ms;
}

std::string config_signature(const Config& config) {
    return "damiao_usb2can|baudrate=" + std::to_string(config.baudrate);
}

transport::BusResourceDescriptor resource_descriptor(const Config& config) {
    transport::BusResourceDescriptor resource;
    resource.kind = transport::BusResourceKind::CAN;
    resource.physical_id = config.serial_port;
    resource.config_signature = config_signature(config);
    resource.ownership_key = transport::tty_ownership_key(config.serial_port);
    return resource;
}

Err to_err(transport::BusRegistryErr error, Err create_error) {
    switch(error) {
        case transport::BusRegistryErr::CREATE_FAILED: return create_error;
        case transport::BusRegistryErr::TYPE_MISMATCH: return Err::TYPE_MISMATCH;
        case transport::BusRegistryErr::PHYSICAL_RESOURCE_CONFLICT: return Err::PHYSICAL_RESOURCE_CONFLICT;
        case transport::BusRegistryErr::INVALID_ARGUMENT:
        case transport::BusRegistryErr::CONFIG_CONFLICT: return Err::CONFIG_CONFLICT;
    }
    return Err::CONFIG_CONFLICT;
}

#pragma pack(push, 1)
/**
 * @brief 达妙官方 USB2CAN 模块接收报文
 */
struct RxPacket {
    std::uint8_t frame_header;
    std::uint8_t cmd;
    std::uint8_t can_data_len : 6;
    std::uint8_t can_ide : 1;
    std::uint8_t can_rtr : 1;
    std::uint32_t can_id;
    std::uint8_t can_data[8];
    std::uint8_t frame_end;
};

/**
 * @brief 达妙官方 USB2CAN 模块发送报文
 */
struct TxPacket {
    std::uint8_t frame_header[2] = { 0x55, 0xAA };
    std::uint8_t frame_len = 0x1e;
    std::uint8_t cmd = 0x03;
    std::uint32_t send_times = 1;
    std::uint32_t time_interval = 10;
    std::uint8_t id_type = 0;
    std::uint32_t can_id = 0x01;
    std::uint8_t frame_type = 0;
    std::uint8_t len = 0x08;
    std::uint8_t id_acc = 0;
    std::uint8_t data_acc = 0;
    std::uint8_t data[8] = { 0 };
    std::uint8_t crc = 0;
};
#pragma pack(pop)

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 创建达妙 USB2CAN 总线对象
 */
DamiaoUsbCanBus::DamiaoUsbCanBus(Config config)
    : config_(std::move(config)) {
}

/**
 * @brief 打开底层串口
 */
tl::expected<void, transport::CanErr> DamiaoUsbCanBus::open() {
    try {
        transport::SerialPort::Config config;
        config.baud_rate = static_cast<std::uint32_t>(config_.baudrate);
        config.read_timeout = std::chrono::milliseconds(2);
        config.write_timeout = std::chrono::milliseconds(100);
        serial_.open(config_.serial_port, config);
        read_failed_ = false;
        running_ = true;
        receiver_ = std::thread(&DamiaoUsbCanBus::receive_loop, this);
        return {};
    }
    catch(...) {
        close();
        return tl::make_unexpected(transport::CanErr::OPEN_FAILED);
    }
}

/**
 * @brief 关闭底层串口并清空缓存
 */
void DamiaoUsbCanBus::close() noexcept {
    running_ = false;
    available_.notify_all();
    if(receiver_.joinable()) receiver_.join();
    serial_.close();
    rx_queue_.clear();
    std::lock_guard<std::mutex> lock(queue_mutex_);
    frames_.clear();
    last_read_ = {};
}

/**
 * @brief 查询底层串口是否已打开
 */
bool DamiaoUsbCanBus::is_open() const noexcept {
    return serial_.is_open();
}

/**
 * @brief 编码并发送 CAN 帧
 */
tl::expected<void, transport::CanErr> DamiaoUsbCanBus::send(const transport::CanFrame& frame) {
    if(!serial_.is_open()) return tl::make_unexpected(transport::CanErr::NOT_OPEN);
    if(frame.size > 8 || frame.id > 0x7FF) return tl::make_unexpected(transport::CanErr::INVALID_FRAME);

    TxPacket wire;
    wire.can_id = frame.id;
    wire.len = frame.size;
    std::copy(frame.data.begin(), frame.data.end(), wire.data);
    try {
        const auto sent = serial_.write(reinterpret_cast<const std::uint8_t*>(&wire), sizeof(wire));
        if(sent != sizeof(wire)) return tl::make_unexpected(transport::CanErr::TIMEOUT);
        return {};
    }
    catch(...) {
        return tl::make_unexpected(transport::CanErr::WRITE_FAILED);
    }
}

/**
 * @brief 在总 timeout 内接收并解析完整 CAN 帧
 */
tl::expected<transport::CanFrame, transport::CanErr> DamiaoUsbCanBus::receive(std::chrono::milliseconds timeout) {
    if(!serial_.is_open()) return tl::make_unexpected(transport::CanErr::NOT_OPEN);

    std::unique_lock<std::mutex> lock(queue_mutex_);
    available_.wait_for(lock, timeout, [this] { return !frames_.empty() || !running_; });
    if(read_failed_) return tl::make_unexpected(transport::CanErr::READ_FAILED);
    if(frames_.empty()) return tl::make_unexpected(transport::CanErr::TIMEOUT);
    auto frame = frames_.front();
    frames_.pop_front();
    return frame;
}

/**
 * @brief 清空协议缓存和串口缓冲区
 */
void DamiaoUsbCanBus::flush() noexcept {
    std::lock_guard<std::mutex> wire_lock(wire_mutex_);
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    rx_queue_.clear();
    frames_.clear();
    last_read_ = {};
    if(!serial_.is_open()) return;
    try {
        serial_.flush();
    }
    catch(...) {
    }
}

/**
 * @brief 获取或创建达妙 USB2CAN 逻辑通道
 */
tl::expected<std::shared_ptr<transport::CanChannel>, Err> acquire_channel(
    const std::string& name,
    const Config& config,
    std::vector<transport::CanFilter> filters) {
    Err create_error = Err::OPEN_FAILED;
    auto channel = transport::acquire_can_channel(
        name,
        resource_descriptor(config),
        [&]() -> std::shared_ptr<transport::CanBus> {
            auto candidate = std::make_shared<DamiaoUsbCanBus>(config);
            const auto opened = candidate->open();
            if(!opened) {
                create_error = Err::OPEN_FAILED;
                return nullptr;
            }
            return candidate;
        },
        std::move(filters));

    if(!channel) return tl::make_unexpected(to_err(channel.error(), create_error));
    return *channel;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 从串口补充字节并尝试解析完整帧
 */
void DamiaoUsbCanBus::receive_loop() noexcept {
    try {
        while(running_) {
            // Only this thread reads the tty, independently of control/decode
            // scheduling; queued frames retain this host read timestamp
            std::lock_guard<std::mutex> wire_lock(wire_mutex_);
            const auto n = serial_.read(rx_buf_.data(), rx_buf_.size(), std::chrono::milliseconds(2));
            if(n == 0) continue;
            const auto received = std::chrono::steady_clock::now();
            const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count();
            const bool continuous = last_read_ != std::chrono::steady_clock::time_point{} &&
                received-last_read_ <= std::chrono::milliseconds(20) &&
                received-last_read_ >= std::chrono::microseconds(250);
            last_read_ = received;
            for(std::size_t i=0;i<n;++i) rx_queue_.push_back({rx_buf_[i], static_cast<std::uint64_t>(stamp), continuous});
            std::vector<transport::CanFrame> batch;
            transport::CanFrame frame;
            while(try_parse_frame(frame)) batch.push_back(frame);
            // Multiple feedbacks for one endpoint in a read are a burst, not
            // independently timed observations; retain their age but no qdd
            for(std::size_t i=0;i<batch.size();++i) {
                for(std::size_t j=i+1;j<batch.size();++j) {
                    if(batch[i].id==batch[j].id && (batch[i].id!=0 ||
                       (batch[i].data[0]&0x0f)==(batch[j].data[0]&0x0f))) {
                        batch[i].receive_timing_valid=false;
                        batch[j].receive_timing_valid=false;
                    }
                }
            }
            {
                std::lock_guard<std::mutex> queue_lock(queue_mutex_);
                for(auto& item:batch) {
                    if(frames_.size()>=transport::DEFAULT_CAN_CHANNEL_MAX_PENDING_FRAMES) {
                        frames_.pop_front();
                        for(auto& pending:frames_) pending.receive_timing_valid=false;
                        item.receive_timing_valid=false;
                    }
                    frames_.push_back(item);
                }
            }
            if(!batch.empty()) available_.notify_all();
        }
    } catch(...) {
        read_failed_ = true;
        running_ = false;
        available_.notify_all();
    }
}

/**
 * @brief 从缓存中解析达妙 USB2CAN 接收报文
 */
bool DamiaoUsbCanBus::try_parse_frame(transport::CanFrame& frame) {
    while(rx_queue_.size() >= sizeof(RxPacket)) {
        if(rx_queue_.front().value != 0xAA) {
            rx_queue_.pop_front();
            continue;
        }

        std::array<std::uint8_t, sizeof(RxPacket)> raw{};
        std::array<ReceivedByte, sizeof(RxPacket)> timed{};
        for(std::size_t i = 0; i < sizeof(RxPacket); ++i) {
            timed[i] = rx_queue_.front();
            raw[i] = timed[i].value;
            rx_queue_.pop_front();
        }
        RxPacket wire{};
        std::memcpy(&wire, raw.data(), raw.size());
        if(wire.cmd != 0x11 || wire.frame_end != 0x55 || wire.can_data_len > 8 ||
            wire.can_ide != 0 || wire.can_rtr != 0 || wire.can_id > 0x7FF) {
            for(std::size_t i = raw.size() - 1; i > 0; --i) rx_queue_.push_front(timed[i]);
            continue;
        }

        // Earliest byte gives a conservative age for fragmented wire packets
        frame.received_ns = timed.front().received_ns;
        frame.receive_timing_valid = std::all_of(timed.begin(), timed.end(), [](const ReceivedByte& b) { return b.timing_valid; });
        frame.id = wire.can_id;
        frame.size = wire.can_data_len;
        std::copy(std::begin(wire.can_data), std::end(wire.can_data), frame.data.begin());
        return true;
    }
    return false;
}

} // namespace serial_arm::protocol::damiao_usb2can
