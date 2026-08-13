#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path write_yaml(const std::string& name, const std::string& text) {
    const auto path = std::filesystem::temp_directory_path() /
        ("serial_arm_damiao_config_" + name + "_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    std::ofstream output(path);
    output << text;
    return path;
}

std::string common_driver_fields() {
    return
        "  refresh_state_in_read: false\n"
        "  feedback_timeout_s: 0.05\n"
        "  activation_retries: 3\n"
        "  startup_read_cycles: 5\n"
        "  stop_kp: 10.0\n"
        "  stop_kd: 0.15\n"
        "  stop_cycles: 5\n"
        "  actuators:\n"
        "    joint1:\n"
        "      name: actuator1\n"
        "      motor_id: 1\n"
        "      master_id: 0\n"
        "      motor_type: DM4310\n";
}

} // namespace

TEST(DamiaoConfigTests, NamedCanBusSuppliesPhysicalConnection) {
    const auto path = write_yaml(
        "named_can",
        "buses:\n"
        "  main_can:\n"
        "    type: can\n"
        "    backend: damiao_usb2can\n"
        "    device: /dev/ttyACM3\n"
        "    baudrate: 1000000\n"
        "damiao:\n"
        "  bus: main_can\n" +
            common_driver_fields());

    serial_arm::DamiaoMotorBus bus;
    auto configured = bus.configure(path.string());
    ASSERT_TRUE(configured);
    EXPECT_EQ(bus.config().bus, "main_can");
    EXPECT_EQ(bus.config().serial_port, "/dev/ttyACM3");
    EXPECT_EQ(bus.config().baudrate, 1000000);
}

TEST(DamiaoConfigTests, LegacyInlineConnectionStillConfigures) {
    const auto path = write_yaml(
        "legacy_inline",
        "damiao:\n"
        "  bus: legacy_can\n"
        "  serial_port: /dev/ttyACM4\n"
        "  baudrate: 921600\n" +
            common_driver_fields());

    serial_arm::DamiaoMotorBus bus;
    auto configured = bus.configure(path.string());
    ASSERT_TRUE(configured);
    EXPECT_EQ(bus.config().bus, "legacy_can");
    EXPECT_EQ(bus.config().serial_port, "/dev/ttyACM4");
    EXPECT_EQ(bus.config().baudrate, 921600);
}

TEST(DamiaoConfigTests, InlinePhysicalFieldsCanOverrideNamedBusForRuntimeEffectiveConfig) {
    const auto path = write_yaml(
        "named_inline_override",
        "buses:\n"
        "  main_can:\n"
        "    type: can\n"
        "    backend: damiao_usb2can\n"
        "    serial_port: /dev/ttyACM5\n"
        "    baudrate: 921600\n"
        "damiao:\n"
        "  bus: main_can\n"
        "  serial_port: /dev/ttyACM6\n"
        "  baudrate: 1152000\n" +
            common_driver_fields());

    serial_arm::DamiaoMotorBus bus;
    auto configured = bus.configure(path.string());
    ASSERT_TRUE(configured);
    EXPECT_EQ(bus.config().bus, "main_can");
    EXPECT_EQ(bus.config().serial_port, "/dev/ttyACM6");
    EXPECT_EQ(bus.config().baudrate, 1152000);
}

TEST(DamiaoConfigTests, RejectsUnsupportedNamedBusDefinition) {
    const auto path = write_yaml(
        "bad_named_can",
        "buses:\n"
        "  main_can:\n"
        "    type: serial\n"
        "    serial_port: /dev/ttyACM7\n"
        "    baudrate: 921600\n"
        "damiao:\n"
        "  bus: main_can\n" +
            common_driver_fields());

    serial_arm::DamiaoMotorBus bus;
    auto configured = bus.configure(path.string());
    ASSERT_FALSE(configured);
    EXPECT_EQ(configured.error(), serial_arm::MotorBusErr::INVALID_CFG);
}
