#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
CONAN_DIR="${CONAN_DIR:-${ROOT_DIR}/build/conan}"
CORE_BUILD_DIR="${CORE_BUILD_DIR:-${ROOT_DIR}/build/serial_arm_core}"
INSTALL_PREFIX="${INSTALL_PREFIX:-${ROOT_DIR}/install/standalone}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')}"
ROBOT=""
WITH_TESTS=0
DEPENDENCY_MODE="system"
RUNENV_FILE=""
CMAKE_DEP_ARGS=()

usage() {
    cat <<'USAGE'
Usage: ./tools/bootstrap_standalone.sh [--robot dm_arm] [--with-tests]

Dependency strategy:
  1. Prefer already installed system CMake packages.
  2. If system dependencies are incomplete, try Conan binary packages only.
  3. Build missing Conan packages from source only after explicit opt-in.

By default, build and install the standalone SerialArm Core without developer tests.
Use --with-tests to build and run the Core test suite with CTest.
With --robot dm_arm, also install the Damiao USB2CAN protocol, Damiao hardware
backend, robot profiles and DM-Arm resources required by dm_arm_gray/white.

Environment overrides:
  BUILD_TYPE                    CMake build type (default: Release)
  CONAN_DIR                     Conan generator directory
  CORE_BUILD_DIR                Core build directory
  INSTALL_PREFIX                Standalone install prefix
  JOBS                          Parallel build jobs
  SERIAL_ARM_ALLOW_SOURCE_BUILD Set to 1 to allow Conan --build=missing
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot)
            if [[ $# -lt 2 ]]; then
                printf 'error: --robot requires a value\n' >&2
                exit 2
            fi
            ROBOT="$2"
            shift 2
            ;;
        --with-tests)
            WITH_TESTS=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'error: unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -n "${ROBOT}" && "${ROBOT}" != "dm_arm" ]]; then
    printf 'error: unsupported robot: %s (supported: dm_arm)\n' "${ROBOT}" >&2
    exit 2
fi

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        printf 'error: required command not found: %s\n' "$1" >&2
        return 1
    fi
}

# Build a standalone-only CMake search path. Any prefix exported by a ROS 2
# environment (AMENT/COLCON) is removed, while ordinary system prefixes such
# as /opt/openrobots remain available for a pure C++ build.
standalone_cmake_prefix_path() {
    local raw="${CMAKE_PREFIX_PATH:-}"
    local deny="${AMENT_PREFIX_PATH:-}:${COLCON_PREFIX_PATH:-}"
    local normalized_raw="${raw//;/:}"
    local normalized_deny="${deny//;/:}"
    local out=()
    local entry deny_entry skip

    IFS=':' read -r -a raw_entries <<< "${normalized_raw}"
    IFS=':' read -r -a deny_entries <<< "${normalized_deny}"

    for entry in "${raw_entries[@]}"; do
        [[ -z "${entry}" ]] && continue
        skip=0
        if [[ "${entry}" == /opt/ros/* ]]; then
            skip=1
        else
            for deny_entry in "${deny_entries[@]}"; do
                [[ -z "${deny_entry}" ]] && continue
                if [[ "${entry}" == "${deny_entry}" ]]; then
                    skip=1
                    break
                fi
            done
        fi
        [[ ${skip} -eq 0 ]] && out+=("${entry}")
    done

    local joined=""
    for entry in "${out[@]}"; do
        if [[ -n "${joined}" ]]; then
            joined+=":"
        fi
        joined+="${entry}"
    done
    printf '%s' "${joined}"
}

run_standalone_command() {
    local filtered_prefix
    filtered_prefix="$(standalone_cmake_prefix_path)"
    env \
        -u AMENT_PREFIX_PATH \
        -u COLCON_PREFIX_PATH \
        -u ROS_DISTRO \
        -u ROS_VERSION \
        -u ROS_PYTHON_VERSION \
        CMAKE_PREFIX_PATH="${filtered_prefix}" \
        "$@"
}

configure_build_install() {
    local source_dir="$1"
    local build_dir="$2"
    local filtered_prefix cmake_prefix
    shift 2

    filtered_prefix="$(standalone_cmake_prefix_path)"
    cmake_prefix="${INSTALL_PREFIX}"
    if [[ -n "${filtered_prefix}" ]]; then
        cmake_prefix+=";${filtered_prefix//:/;}"
    fi

    run_standalone_command cmake \
        -S "${source_dir}" \
        -B "${build_dir}" \
        "${CMAKE_DEP_ARGS[@]}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_PREFIX_PATH="${cmake_prefix}" \
        "$@"
    run_standalone_command cmake --build "${build_dir}" --parallel "${JOBS}"
    run_standalone_command cmake --install "${build_dir}" --prefix "${INSTALL_PREFIX}"
}

probe_system_dependencies() {
    local probe_src="${ROOT_DIR}/build/serial_arm_dependency_probe_src"
    local probe_build="${ROOT_DIR}/build/serial_arm_dependency_probe"
    local probe_log="${ROOT_DIR}/build/serial_arm_dependency_probe.log"

    mkdir -p "${probe_src}"
    cat > "${probe_src}/CMakeLists.txt" <<'EOF_CMAKE'
cmake_minimum_required(VERSION 3.20)
project(serial_arm_dependency_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SERIAL_ARM_PROBE_TEST_DEPS "Probe standalone test dependencies" OFF)
set(SERIAL_ARM_MISSING_DEPS "")

find_package(yaml-cpp QUIET)
if(NOT TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
    list(APPEND SERIAL_ARM_MISSING_DEPS "yaml-cpp")
endif()

find_package(Eigen3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    list(APPEND SERIAL_ARM_MISSING_DEPS "Eigen3")
endif()

find_package(pinocchio CONFIG QUIET)
if(NOT TARGET pinocchio::pinocchio AND NOT pinocchio_FOUND)
    list(APPEND SERIAL_ARM_MISSING_DEPS "pinocchio")
endif()

if(SERIAL_ARM_PROBE_TEST_DEPS)
    find_package(GTest QUIET)
    if(NOT TARGET GTest::gtest_main AND NOT GTest_FOUND)
        list(APPEND SERIAL_ARM_MISSING_DEPS "GTest")
    endif()
endif()

if(SERIAL_ARM_MISSING_DEPS)
    list(JOIN SERIAL_ARM_MISSING_DEPS ", " SERIAL_ARM_MISSING_DEPS_TEXT)
    message(FATAL_ERROR "Missing system CMake packages: ${SERIAL_ARM_MISSING_DEPS_TEXT}")
endif()
EOF_CMAKE

    rm -rf "${probe_build}"
    if run_standalone_command cmake \
        -S "${probe_src}" \
        -B "${probe_build}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DSERIAL_ARM_PROBE_TEST_DEPS=$([[ ${WITH_TESTS} -eq 1 ]] && printf ON || printf OFF) \
        >"${probe_log}" 2>&1; then
        return 0
    fi

    printf 'System dependency probe did not find the complete standalone dependency set.\n'
    if grep -F "Missing system CMake packages:" "${probe_log}" >/dev/null 2>&1; then
        grep -F "Missing system CMake packages:" "${probe_log}" | tail -n 1 | sed 's/^[[:space:]]*//'
    fi
    return 1
}

source_build_allowed() {
    local value="${SERIAL_ARM_ALLOW_SOURCE_BUILD:-}"
    case "${value,,}" in
        1|true|yes|on)
            return 0
            ;;
        0|false|no|off)
            return 1
            ;;
        "")
            if [[ -t 0 && -t 1 ]]; then
                printf '\nConanCenter has no compatible binary for at least one dependency.\n'
                printf 'Building dependencies such as Boost/Pinocchio from source can take a long time.\n'
                read -r -p 'Build missing dependencies from source now? [y/N] ' answer
                case "${answer,,}" in
                    y|yes) return 0 ;;
                    *) return 1 ;;
                esac
            fi
            return 1
            ;;
        *)
            printf 'error: invalid SERIAL_ARM_ALLOW_SOURCE_BUILD=%q (use 0 or 1)\n' "${value}" >&2
            return 1
            ;;
    esac
}

setup_conan_dependencies() {
    require_command conan

    case "$(conan --version)" in
        "Conan version 2."*) ;;
        *)
            printf 'error: Conan 2 is required, got: %s\n' "$(conan --version)" >&2
            exit 2
            ;;
    esac

    if ! conan profile path default >/dev/null 2>&1; then
        conan profile detect
    fi

    local install_args=(
        install "${ROOT_DIR}"
        --output-folder="${CONAN_DIR}"
        -s build_type="${BUILD_TYPE}"
        -s compiler.cppstd=17
        -o "&:with_tests=$([[ ${WITH_TESTS} -eq 1 ]] && printf True || printf False)"
    )

    printf '\nTrying Conan binary packages first (source builds disabled)...\n'
    if ! conan "${install_args[@]}"; then
        if ! source_build_allowed; then
            cat >&2 <<'EOF_ERROR'
error: compatible Conan binaries are unavailable and source builds were not enabled.
No long source build was started.

Options:
  - Install/provide Pinocchio, Eigen3, yaml-cpp and GTest as system CMake packages, then rerun.
  - Or explicitly allow the slow fallback:
      SERIAL_ARM_ALLOW_SOURCE_BUILD=1 ./tools/bootstrap_standalone.sh
    Add --robot dm_arm to the command when needed.
EOF_ERROR
            exit 5
        fi

        printf '\nSource-build fallback explicitly enabled; Conan may take a long time.\n'
        conan "${install_args[@]}" --build=missing
    fi

    local toolchain_file="${CONAN_DIR}/conan_toolchain.cmake"
    if [[ ! -f "${toolchain_file}" ]]; then
        printf 'error: Conan did not generate %s\n' "${toolchain_file}" >&2
        exit 3
    fi

    RUNENV_FILE="${CONAN_DIR}/conanrun.sh"
    if [[ ! -f "${RUNENV_FILE}" ]]; then
        printf 'error: Conan did not generate runtime environment: %s\n' "${RUNENV_FILE}" >&2
        exit 4
    fi

    DEPENDENCY_MODE="conan"
    CMAKE_DEP_ARGS=(-DCMAKE_TOOLCHAIN_FILE="${toolchain_file}")

    # shellcheck disable=SC1090
    source "${RUNENV_FILE}"
}

require_command cmake
if [[ ${WITH_TESTS} -eq 1 ]]; then
    require_command ctest
fi

printf 'Checking installed system dependencies...\n'
if probe_system_dependencies; then
    printf 'Using system dependencies; Conan is not required.\n'
else
    printf 'Falling back to Conan dependency resolution.\n'
    setup_conan_dependencies
fi

run_standalone_command cmake \
    -S "${ROOT_DIR}/src/serial_arm/core" \
    -B "${CORE_BUILD_DIR}" \
    "${CMAKE_DEP_ARGS[@]}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DSERIAL_ARM_ENABLE_ROS2=OFF \
    -DSERIAL_ARM_BUILD_PYTHON=OFF \
    -DSERIAL_ARM_BUILD_TERMINAL=ON \
    -DBUILD_TESTING=$([[ ${WITH_TESTS} -eq 1 ]] && printf ON || printf OFF)

run_standalone_command cmake --build "${CORE_BUILD_DIR}" --parallel "${JOBS}"
if [[ ${WITH_TESTS} -eq 1 ]]; then
    run_standalone_command ctest --test-dir "${CORE_BUILD_DIR}" --output-on-failure
fi
run_standalone_command cmake --install "${CORE_BUILD_DIR}" --prefix "${INSTALL_PREFIX}"

if [[ "${ROBOT}" == "dm_arm" ]]; then
    configure_build_install \
        "${ROOT_DIR}/src/robot_supports/protocol/damiao_usb2can" \
        "${ROOT_DIR}/build/serial_arm_protocol_damiao_usb2can" \
        -DBUILD_TESTING=OFF

    configure_build_install \
        "${ROOT_DIR}/src/robot_supports/hardware/damiao" \
        "${ROOT_DIR}/build/serial_arm_hardware_damiao" \
        -DBUILD_TESTING=OFF

    configure_build_install \
        "${ROOT_DIR}/src/robot_supports/profiles" \
        "${ROOT_DIR}/build/serial_arm_robot_profiles"

    configure_build_install \
        "${ROOT_DIR}/src/robot_supports/robots/dm_arm/description" \
        "${ROOT_DIR}/build/dm_arm_description"
fi

mkdir -p "${INSTALL_PREFIX}"
SETUP_FILE="${INSTALL_PREFIX}/setup.bash"
{
    printf '%s\n' '#!/usr/bin/env bash'
    printf '%s\n' '# Generated by tools/bootstrap_standalone.sh'
    if [[ "${DEPENDENCY_MODE}" == "conan" ]]; then
        printf '%s\n' '# shellcheck disable=SC1091'
        printf 'source %q\n' "${RUNENV_FILE}"
    fi
    printf 'export SERIAL_ARM_RESOURCE_PATH=%q\n' "${INSTALL_PREFIX}"
    printf 'export PATH=%q:"${PATH:-}"\n' "${INSTALL_PREFIX}/bin"
    printf 'export LD_LIBRARY_PATH=%q:"${LD_LIBRARY_PATH:-}"\n' "${INSTALL_PREFIX}/lib"
} > "${SETUP_FILE}"
chmod +x "${SETUP_FILE}"

printf '\nStandalone Core is ready\n'
printf '  dependencies: %s\n' "${DEPENDENCY_MODE}"
printf '  build:        %s\n' "${CORE_BUILD_DIR}"
printf '  install:      %s\n' "${INSTALL_PREFIX}"
if [[ ${WITH_TESTS} -eq 1 ]]; then
    printf '  tests:        enabled\n'
else
    printf '  tests:        disabled (use --with-tests)\n'
fi
if [[ "${ROBOT}" == "dm_arm" ]]; then
    printf '  robot:        dm_arm (protocol + hardware + profiles + description)\n'
fi
printf '\nBefore running installed binaries in a new shell:\n'
printf '  source %q\n' "${SETUP_FILE}"
if [[ "${ROBOT}" == "dm_arm" ]]; then
    printf '  serial_arm_terminal --robot-profile dm_arm_gray\n'
fi
