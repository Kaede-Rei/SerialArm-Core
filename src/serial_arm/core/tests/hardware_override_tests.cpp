#include "serial_arm/hardware/hardware_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path write_hardware_yaml(const std::string& name) {
    const std::filesystem::path path = std::filesystem::path(SERIAL_ARM_TEST_TMP_DIR) / name;
    std::ofstream output(path);
    output << "damiao:\n"
           << "  bus: main_can\n"
           << "  serial_port: /dev/ttyACM0\n"
           << "  baudrate: 921600\n";
    return path;
}

std::filesystem::path write_named_hardware_yaml(const std::string& name) {
    const std::filesystem::path path = std::filesystem::path(SERIAL_ARM_TEST_TMP_DIR) / name;
    std::ofstream output(path);
    output << "buses:\n"
           << "  main_can:\n"
           << "    type: can\n"
           << "    backend: damiao_usb2can\n"
           << "    device: /dev/ttyACM0\n"
           << "    baudrate: 921600\n"
           << "  secondary_can:\n"
           << "    type: can\n"
           << "    backend: damiao_usb2can\n"
           << "    serial_port: /dev/ttyACM2\n"
           << "    baudrate: 1000000\n"
           << "damiao:\n"
           << "  bus: main_can\n";
    return path;
}

std::filesystem::path write_named_serial_hardware_yaml(const std::string& name) {
    const std::filesystem::path path = std::filesystem::path(SERIAL_ARM_TEST_TMP_DIR) / name;
    std::ofstream output(path);
    output << "buses:\n"
           << "  auxiliary_serial:\n"
           << "    type: serial\n"
           << "    serial_port: /dev/ttyUSB0\n"
           << "    baudrate: 1000000\n"
           << "    data_bits: 8\n"
           << "    parity: none\n"
           << "    stop_bits: 1\n"
           << "    flow_control: none\n"
           << "damiao:\n"
           << "  bus: auxiliary_serial\n";
    return path;
}

std::string load_name_from(const std::filesystem::path& config, const serial_arm::HardwareConfigOverrides& overrides = {}) {
    serial_arm::HardwareLoader loader;
    auto bus = loader.load(SERIAL_ARM_TEST_FAKE_HARDWARE_PLUGIN, config.string(), overrides);
    EXPECT_TRUE(bus) << static_cast<int>(bus.error());
    if(!bus) return {};
    EXPECT_EQ(bus.value()->capabilities().size(), 1u);
    return bus.value()->capabilities().front().actuator_name;
}

std::string load_name(const serial_arm::HardwareConfigOverrides& overrides = {}) {
    return load_name_from(write_hardware_yaml("hardware_override_test.yaml"), overrides);
}

} // namespace

TEST(HardwareOverrideTests, NoOverrideKeepsYamlValues) {
    EXPECT_EQ(load_name(), "main_can|/dev/ttyACM0|921600");
}

TEST(HardwareOverrideTests, SerialPortOverrideKeepsOtherYamlValues) {
    serial_arm::HardwareConfigOverrides overrides;
    overrides.serial_port = "/dev/ttyACM1";
    EXPECT_EQ(load_name(overrides), "main_can|/dev/ttyACM1|921600");
}

TEST(HardwareOverrideTests, MultipleOverridesKeepUnspecifiedYamlValues) {
    serial_arm::HardwareConfigOverrides overrides;
    overrides.serial_port = "/dev/ttyACM1";
    overrides.baudrate = 1000000;
    EXPECT_EQ(load_name(overrides), "main_can|/dev/ttyACM1|1000000");
}

TEST(HardwareOverrideTests, BusOverrideDoesNotAffectOtherFields) {
    serial_arm::HardwareConfigOverrides overrides;
    overrides.bus = "secondary_can";
    EXPECT_EQ(load_name(overrides), "secondary_can|/dev/ttyACM0|921600");
}

TEST(HardwareOverrideTests, InvalidBaudrateOverrideIsRejected) {
    const auto config = write_hardware_yaml("hardware_override_invalid_baud.yaml");
    serial_arm::HardwareConfigOverrides overrides;
    overrides.baudrate = 0;

    serial_arm::HardwareLoader loader;
    auto bus = loader.load(SERIAL_ARM_TEST_FAKE_HARDWARE_PLUGIN, config.string(), overrides);
    ASSERT_FALSE(bus);
    EXPECT_EQ(bus.error(), serial_arm::HardwareLoaderErr::INVALID_OVERRIDE);
}

TEST(HardwareOverrideTests, NamedBusLoadsPhysicalConnection) {
    const auto config = write_named_hardware_yaml("hardware_named_bus.yaml");
    EXPECT_EQ(load_name_from(config), "main_can|/dev/ttyACM0|921600");
}

TEST(HardwareOverrideTests, BusOverrideSelectsNamedBusPhysicalConnection) {
    const auto config = write_named_hardware_yaml("hardware_named_bus_override.yaml");
    serial_arm::HardwareConfigOverrides overrides;
    overrides.bus = "secondary_can";
    EXPECT_EQ(load_name_from(config, overrides), "secondary_can|/dev/ttyACM2|1000000");
}

TEST(HardwareOverrideTests, SerialOverridesUpdateReferencedNamedBus) {
    const auto config = write_named_hardware_yaml("hardware_named_bus_serial_override.yaml");
    serial_arm::HardwareConfigOverrides overrides;
    overrides.serial_port = "/dev/ttyACM9";
    overrides.baudrate = 1152000;
    EXPECT_EQ(load_name_from(config, overrides), "main_can|/dev/ttyACM9|1152000");
}

TEST(HardwareOverrideTests, NamedSerialBusPhysicalFieldsCanBeLoaded) {
    const auto config = write_named_serial_hardware_yaml("hardware_named_serial_bus.yaml");
    EXPECT_EQ(load_name_from(config), "auxiliary_serial|/dev/ttyUSB0|1000000");
}

TEST(HardwareOverrideTests, SerialOverrideUpdatesNamedSerialBus) {
    const auto config = write_named_serial_hardware_yaml("hardware_named_serial_bus_override.yaml");
    serial_arm::HardwareConfigOverrides overrides;
    overrides.serial_port = "/dev/ttyUSB9";
    overrides.baudrate = 921600;
    EXPECT_EQ(load_name_from(config, overrides), "auxiliary_serial|/dev/ttyUSB9|921600");
}
