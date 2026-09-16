#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


class StandaloneDependencyContractTest(unittest.TestCase):
    def test_conan_recipe_declares_supported_dependencies(self) -> None:
        recipe = (ROOT / "conanfile.py").read_text(encoding="utf-8")
        for requirement in (
            "pinocchio/3.8.0",
            "yaml-cpp/0.9.0",
            "eigen/3.4.0",
            "gtest/1.17.0",
        ):
            self.assertIn(requirement, recipe)
        self.assertIn('required_conan_version = ">=2.1"', recipe)
        self.assertIn("CMakeDeps", recipe)
        self.assertIn("CMakeToolchain", recipe)
        self.assertIn("VirtualRunEnv", recipe)
        self.assertIn("pinocchio/*:with_collision_support", recipe)

    def test_bootstrap_uses_conan_toolchain_and_runs_core_tests(self) -> None:
        bootstrap = (ROOT / "tools/bootstrap_standalone.sh").read_text(encoding="utf-8")
        for fragment in (
            "conan profile detect",
            "conan install",
            "conan_toolchain.cmake",
            "SERIAL_ARM_BUILD_PYTHON=OFF",
            "BUILD_TESTING=ON",
            "ctest --test-dir",
            "cmake --install",
            "Conan did not generate runtime environment",
        ):
            self.assertIn(fragment, bootstrap)


    def test_standalone_python_extension_is_installed_into_wheel(self) -> None:
        cmake = (ROOT / "src/serial_arm/core/CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('LIBRARY DESTINATION "serial_arm"', cmake)
        self.assertGreaterEqual(cmake.count("install(TARGETS _serial_arm"), 2)
        self.assertIn("install(TARGETS serial_arm_core", cmake)
        self.assertIn('INSTALL_RPATH "$ORIGIN"', cmake)
        self.assertTrue((ROOT / "src/serial_arm/core/python/README.md").is_file())

    def test_bootstrap_can_install_complete_dm_arm_stack(self) -> None:
        bootstrap = (ROOT / "tools/bootstrap_standalone.sh").read_text(encoding="utf-8")
        for fragment in (
            "--robot",
            "dm_arm",
            "src/robot_supports/protocol/damiao_usb2can",
            "src/robot_supports/hardware/damiao",
            "src/robot_supports/profiles",
            "src/robot_supports/robots/dm_arm/description",
            "setup.bash",
        ):
            self.assertIn(fragment, bootstrap)


if __name__ == "__main__":
    unittest.main()
