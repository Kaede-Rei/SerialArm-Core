#include "serial_arm/transport/serial_bus.hpp"

#include <stdexcept>

namespace serial_arm::transport {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

std::string to_string(SerialPort::Parity value) {
    switch(value) {
        case SerialPort::Parity::None: return "N";
        case SerialPort::Parity::Even: return "E";
        case SerialPort::Parity::Odd: return "O";
    }
    return "?";
}

std::string to_string(SerialPort::StopBits value) {
    switch(value) {
        case SerialPort::StopBits::One: return "1";
        case SerialPort::StopBits::Two: return "2";
    }
    return "?";
}

std::string to_string(SerialPort::FlowControl value) {
    switch(value) {
        case SerialPort::FlowControl::None: return "none";
        case SerialPort::FlowControl::Software: return "software";
        case SerialPort::FlowControl::Hardware: return "hardware";
    }
    return "unknown";
}

std::string config_signature(const SerialPort::Config& config) {
    return "serial|baudrate=" + std::to_string(config.baud_rate) +
        "|data_bits=" + std::to_string(config.data_bits) +
        "|parity=" + to_string(config.parity) +
        "|stop_bits=" + to_string(config.stop_bits) +
        "|flow_control=" + to_string(config.flow_control);
}

} // namespace


// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 创建受限串行事务视图
 */
SerialTransaction::SerialTransaction(SerialPort& serial, SerialTransactionOptions options) noexcept
    : serial_(serial), options_(options) {
}

/**
 * @brief 最多读取指定字节数
 */
std::size_t SerialTransaction::read(Byte* data, std::size_t len) {
    return serial_.read(data, len, options_.read_timeout);
}

/**
 * @brief 使用指定超时最多读取指定字节数
 */
std::size_t SerialTransaction::read(Byte* data, std::size_t len, std::chrono::milliseconds timeout) {
    return serial_.read(data, len, timeout);
}

/**
 * @brief 最多读取指定字节数并返回新缓冲区
 */
SerialTransaction::Buffer SerialTransaction::read(std::size_t max_bytes) {
    Buffer buffer(max_bytes);
    const std::size_t received = read(buffer.data(), buffer.size());
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 使用指定超时最多读取指定字节数并返回新缓冲区
 */
SerialTransaction::Buffer SerialTransaction::read(
    std::size_t max_bytes,
    std::chrono::milliseconds timeout) {
    Buffer buffer(max_bytes);
    const std::size_t received = read(buffer.data(), buffer.size(), timeout);
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 在事务默认读取超时内尽量读满指定长度
 */
std::size_t SerialTransaction::read_exact(Byte* data, std::size_t len) {
    return serial_.read_exact(data, len, options_.read_timeout);
}

/**
 * @brief 使用指定超时尽量读满指定长度
 */
std::size_t SerialTransaction::read_exact(
    Byte* data,
    std::size_t len,
    std::chrono::milliseconds timeout) {
    return serial_.read_exact(data, len, timeout);
}

/**
 * @brief 在事务默认读取超时内尽量读满指定长度并返回新缓冲区
 */
SerialTransaction::Buffer SerialTransaction::read_exact(std::size_t len) {
    Buffer buffer(len);
    const std::size_t received = read_exact(buffer.data(), buffer.size());
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 使用指定超时尽量读满指定长度并返回新缓冲区
 */
SerialTransaction::Buffer SerialTransaction::read_exact(
    std::size_t len,
    std::chrono::milliseconds timeout) {
    Buffer buffer(len);
    const std::size_t received = read_exact(buffer.data(), buffer.size(), timeout);
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 在事务默认写入超时内尽量写出指定长度
 */
std::size_t SerialTransaction::write(const Byte* data, std::size_t len) {
    return serial_.write(data, len, options_.write_timeout);
}

/**
 * @brief 使用指定超时尽量写出指定长度
 */
std::size_t SerialTransaction::write(
    const Byte* data,
    std::size_t len,
    std::chrono::milliseconds timeout) {
    return serial_.write(data, len, timeout);
}

/**
 * @brief 写出字节缓冲区
 */
std::size_t SerialTransaction::write(const Buffer& data) {
    return write(data.data(), data.size());
}

/**
 * @brief 使用指定超时写出字节缓冲区
 */
std::size_t SerialTransaction::write(const Buffer& data, std::chrono::milliseconds timeout) {
    return write(data.data(), data.size(), timeout);
}

/**
 * @brief 写出初始化列表
 */
std::size_t SerialTransaction::write(std::initializer_list<Byte> data) {
    return write(data.begin(), data.size());
}

/**
 * @brief 使用指定超时写出初始化列表
 */
std::size_t SerialTransaction::write(
    std::initializer_list<Byte> data,
    std::chrono::milliseconds timeout) {
    return write(data.begin(), data.size(), timeout);
}

/**
 * @brief 等待输出队列发送完成
 */
void SerialTransaction::drain() {
    serial_.drain();
}

/**
 * @brief 清空串口缓冲区
 */
void SerialTransaction::flush(FlushDirection direction) {
    serial_.flush(direction);
}

/**
 * @brief 查询内核输入缓冲区可读字节数
 */
std::size_t SerialTransaction::available() const {
    return serial_.available();
}

/**
 * @brief 创建未打开的共享串行总线
 */
SerialBus::SerialBus(Config config)
    : config_(std::move(config)) {
}

/**
 * @brief 关闭串口并释放资源
 */
SerialBus::~SerialBus() noexcept {
    close();
}

/**
 * @brief 打开底层串口
 */
void SerialBus::open() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(serial_.is_open()) return;
    serial_.open(config_.serial_port, config_.port_config);
}

/**
 * @brief 关闭底层串口
 */
void SerialBus::close() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    serial_.close();
}

/**
 * @brief 查询串口是否已打开
 */
bool SerialBus::is_open() const noexcept {
    return serial_.is_open();
}

/**
 * @brief 获取共享串行总线轻量运行统计
 */
SerialBusDiagnostics SerialBus::diagnostics() const {
    SerialBusDiagnostics value;
    value.is_open = serial_.is_open();
    value.transaction_count = transaction_count_.load();
    value.failed_transaction_count = failed_transaction_count_.load();
    value.resource = resource_descriptor(config_);
    return value;
}

/**
 * @brief 根据配置生成 physical resource descriptor
 */
BusResourceDescriptor SerialBus::resource_descriptor(const Config& config) {
    BusResourceDescriptor resource;
    resource.kind = BusResourceKind::SERIAL;
    resource.physical_id = config.serial_port;
    resource.config_signature = config_signature(config.port_config);
    resource.ownership_key = tty_ownership_key(config.serial_port);
    return resource;
}

/**
 * @brief 校验事务超时配置
 */
void SerialBus::validate_transaction_options(const SerialTransactionOptions& options) {
    if(options.read_timeout.count() < 0) {
        throw std::invalid_argument("SerialBus read_timeout must be >= 0 ms");
    }
    if(options.write_timeout.count() < 0) {
        throw std::invalid_argument("SerialBus write_timeout must be >= 0 ms");
    }
}


/**
 * @brief 创建协议层共享串行 client
 */
SerialBusClient::SerialBusClient(
    std::shared_ptr<SerialBus> bus,
    SerialTransactionOptions options) noexcept
    : bus_(std::move(bus)), options_(options) {
}

/**
 * @brief 查询共享串行总线是否已打开
 */
bool SerialBusClient::is_open() const noexcept {
    return bus_ && bus_->is_open();
}

/**
 * @brief 获取共享串行总线轻量运行统计
 */
SerialBusDiagnostics SerialBusClient::diagnostics() const {
    if(!bus_) return {};
    return bus_->diagnostics();
}

/**
 * @brief 获取共享串行总线协议 client
 */
tl::expected<std::shared_ptr<SerialBusClient>, BusRegistryErr> acquire_serial_bus_client(
    const std::string& name,
    const SerialBusConfig& config) {
    if(name.empty() || config.serial_port.empty()) {
        return tl::make_unexpected(BusRegistryErr::INVALID_ARGUMENT);
    }

    SerialTransactionOptions options;
    options.read_timeout = config.port_config.read_timeout;
    options.write_timeout = config.port_config.write_timeout;
    try {
        SerialBus::validate_transaction_options(options);
        SerialPort validator;
        validator.set_config(config.port_config);
    }
    catch(const std::invalid_argument&) {
        return tl::make_unexpected(BusRegistryErr::INVALID_ARGUMENT);
    }

    auto bus = BusRegistry::get_or_create<SerialBus>(
        name,
        SerialBus::resource_descriptor(config),
        [&]() {
            auto value = std::shared_ptr<SerialBus>(new SerialBus(config));
            value->open();
            return value;
        });
    if(!bus) return tl::make_unexpected(bus.error());

    return std::shared_ptr<SerialBusClient>(
        new SerialBusClient(*bus, options));
}

} // namespace serial_arm::transport
