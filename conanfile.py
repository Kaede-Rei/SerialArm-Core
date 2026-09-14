from conan import ConanFile


required_conan_version = ">=2.1"


class SerialArmStandaloneDeps(ConanFile):
    """Standalone dependency set for SerialArm-Core.

    The Core CMake project intentionally remains dependency-manager agnostic and
    continues to consume dependencies through find_package(). Conan only owns
    dependency acquisition and generates CMake package/toolchain metadata.
    """

    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "VirtualRunEnv"

    default_options = {
        "pinocchio/*:with_collision_support": False,
        "yaml-cpp/*:shared": False,
        "gtest/*:shared": False,
    }

    def requirements(self):
        self.requires("pinocchio/3.8.0")
        self.requires("yaml-cpp/0.9.0")
        self.requires("eigen/3.4.0")

    def build_requirements(self):
        self.test_requires("gtest/1.17.0")
