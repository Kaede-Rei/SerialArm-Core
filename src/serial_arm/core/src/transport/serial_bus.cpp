#include "serial_arm/transport/serial_bus.hpp"

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
        "|flow_control=" + to_string(config.flow_control) +
        "|read_timeout_ms=" + std::to_string(config.read_timeout.count()) +
        "|write_timeout_ms=" + std::to_string(config.write_timeout.count());
}

} // namespace


// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

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
    std::lock_guard<std::mutex> lock(mutex_);
    return serial_.is_open();
}

/**
 * @brief 获取共享串行总线配置
 */
const SerialBus::Config& SerialBus::config() const noexcept {
    return config_;
}

/**
 * @brief 获取共享串行总线轻量运行统计
 */
SerialBusDiagnostics SerialBus::diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    SerialBusDiagnostics value;
    value.is_open = serial_.is_open();
    value.transaction_count = transaction_count_;
    value.failed_transaction_count = failed_transaction_count_;
    value.resource = resource_descriptor(config_);
    return value;
}

/**
 * @brief 获取 physical resource descriptor
 */
BusResourceDescriptor SerialBus::resource_descriptor() const {
    return resource_descriptor(config_);
}

/**
 * @brief 根据配置生成 physical resource descriptor
 */
BusResourceDescriptor SerialBus::resource_descriptor(const Config& config) {
    BusResourceDescriptor resource;
    resource.kind = BusResourceKind::SERIAL;
    resource.physical_id = config.serial_port;
    resource.config_signature = config_signature(config.port_config);
    return resource;
}

} // namespace serial_arm::transport
