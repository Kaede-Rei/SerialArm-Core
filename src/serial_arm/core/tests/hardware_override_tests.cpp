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

std::string load_name(const serial_arm::HardwareConfigOverrides& overrides = {}) {
    const auto config = write_hardware_yaml("hardware_override_test.yaml");
    serial_arm::HardwareLoader loader;
    auto bus = loader.load(SERIAL_ARM_TEST_FAKE_HARDWARE_PLUGIN, config.string(), overrides);
    EXPECT_TRUE(bus) << static_cast<int>(bus.error());
    if(!bus) return {};
    EXPECT_EQ(bus.value()->capabilities().size(), 1u);
    return bus.value()->capabilities().front().actuator_name;
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
