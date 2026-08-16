#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path);
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

void expect_classified_terminal_menu(const std::string& source) {
    EXPECT_NE(source.find("1. 状态查看"), std::string::npos);
    EXPECT_NE(source.find("2. 使能 / 失能 / 故障"), std::string::npos);
    EXPECT_NE(source.find("3. 模式与补偿"), std::string::npos);
    EXPECT_NE(source.find("4. 运动与命令"), std::string::npos);
    EXPECT_NE(source.find("5. 动力学与配置"), std::string::npos);
}

} // namespace

TEST(TerminalMenuTests, CppTerminalUsesClassifiedMenus) {
    const auto source = read_file(std::filesystem::path(SERIAL_ARM_TEST_REPO_ROOT) / "src/serial_arm/core/app/serial_arm_terminal.cpp");
    expect_classified_terminal_menu(source);
    EXPECT_EQ(source.find("6. 实验采集"), std::string::npos);
    EXPECT_EQ(source.find("handle_experiment_menu"), std::string::npos);
    EXPECT_EQ(source.find("Torque Residual Observer Test"), std::string::npos);
}

TEST(TerminalMenuTests, PythonTerminalUsesClassifiedMenus) {
    const auto source = read_file(std::filesystem::path(SERIAL_ARM_TEST_REPO_ROOT) / "src/serial_arm/core/app/serial_arm_terminal.py");
    expect_classified_terminal_menu(source);
    EXPECT_NE(source.find("6. 工具与监视"), std::string::npos);
}
