#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
CONAN_DIR="${CONAN_DIR:-${ROOT_DIR}/build/conan}"
CORE_BUILD_DIR="${CORE_BUILD_DIR:-${ROOT_DIR}/build/serial_arm_core}"
INSTALL_PREFIX="${INSTALL_PREFIX:-${ROOT_DIR}/install/standalone}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')}"

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        printf 'error: required command not found: %s\n' "$1" >&2
        return 1
    fi
}

require_command conan
require_command cmake
require_command ctest

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

conan install "${ROOT_DIR}" \
    --output-folder="${CONAN_DIR}" \
    --build=missing \
    -s build_type="${BUILD_TYPE}"

TOOLCHAIN_FILE="${CONAN_DIR}/conan_toolchain.cmake"
if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
    printf 'error: Conan did not generate %s\n' "${TOOLCHAIN_FILE}" >&2
    exit 3
fi

cmake \
    -S "${ROOT_DIR}/src/serial_arm/core" \
    -B "${CORE_BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DSERIAL_ARM_BUILD_PYTHON=OFF \
    -DSERIAL_ARM_BUILD_TERMINAL=ON \
    -DBUILD_TESTING=ON

cmake --build "${CORE_BUILD_DIR}" --parallel "${JOBS}"

RUNENV_FILE="${CONAN_DIR}/conanrun.sh"
if [[ ! -f "${RUNENV_FILE}" ]]; then
    printf 'error: Conan did not generate runtime environment: %s\n' "${RUNENV_FILE}" >&2
    exit 4
fi

# shellcheck disable=SC1091
source "${RUNENV_FILE}"

ctest --test-dir "${CORE_BUILD_DIR}" --output-on-failure
cmake --install "${CORE_BUILD_DIR}" --prefix "${INSTALL_PREFIX}"

printf '\nStandalone Core is ready\n'
printf '  build:   %s\n' "${CORE_BUILD_DIR}"
printf '  install: %s\n' "${INSTALL_PREFIX}"
printf '\nBefore running installed binaries in a new shell, load Conan runtime paths:\n'
printf '  source %q\n' "${CONAN_DIR}/conanrun.sh"
