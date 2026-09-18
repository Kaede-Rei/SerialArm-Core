#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import tempfile
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "tools/bootstrap_standalone.sh"


def write_executable(path: Path, content: str) -> None:
    path.write_text(textwrap.dedent(content), encoding="utf-8")
    path.chmod(0o755)


def make_fake_tools(tmp: Path, *, system_deps: bool, binary_available: bool) -> tuple[Path, Path]:
    fake_bin = tmp / "bin"
    fake_bin.mkdir()
    log = tmp / "calls.log"

    write_executable(
        fake_bin / "cmake",
        f'''#!/usr/bin/env bash
set -euo pipefail
printf 'cmake %s\n' "$*" >> "$SERIALARM_TEST_LOG"
printf 'env CMAKE_PREFIX_PATH=%s AMENT_PREFIX_PATH=%s COLCON_PREFIX_PATH=%s\n' "${{CMAKE_PREFIX_PATH-}}" "${{AMENT_PREFIX_PATH-}}" "${{COLCON_PREFIX_PATH-}}" >> "$SERIALARM_TEST_LOG"
if [[ "$*" == *"serial_arm_dependency_probe"* ]]; then
  exit {0 if system_deps else 1}
fi
exit 0
''',
    )
    write_executable(
        fake_bin / "ctest",
        r'''#!/usr/bin/env bash
set -euo pipefail
printf 'ctest %s\n' "$*" >> "$SERIALARM_TEST_LOG"
exit 0
''',
    )
    write_executable(
        fake_bin / "conan",
        f'''#!/usr/bin/env bash
set -euo pipefail
printf 'conan %s\n' "$*" >> "$SERIALARM_TEST_LOG"
if [[ "${{1:-}}" == "--version" ]]; then
  printf 'Conan version 2.9.0\n'
  exit 0
fi
if [[ "${{1:-}}" == "profile" && "${{2:-}}" == "path" ]]; then
  exit 1
fi
if [[ "${{1:-}}" == "profile" && "${{2:-}}" == "detect" ]]; then
  exit 0
fi
if [[ "${{1:-}}" == "install" ]]; then
  out=''
  has_build_missing=0
  for arg in "$@"; do
    case "$arg" in
      --output-folder=*) out="${{arg#--output-folder=}}" ;;
      --build=missing) has_build_missing=1 ;;
    esac
  done
  if [[ {0 if binary_available else 1} -eq 1 && $has_build_missing -eq 0 ]]; then
    exit 1
  fi
  mkdir -p "$out"
  : > "$out/conan_toolchain.cmake"
  cat > "$out/conanrun.sh" <<'EOF'
#!/usr/bin/env bash
true
EOF
  exit 0
fi
exit 0
''',
    )
    return fake_bin, log


def run_bootstrap(
    tmp: Path,
    *,
    system_deps: bool,
    binary_available: bool,
    allow_source: str | None,
    robot: bool = False,
    with_tests: bool = False,
) -> tuple[subprocess.CompletedProcess[str], str, Path]:
    fake_bin, log = make_fake_tools(
        tmp, system_deps=system_deps, binary_available=binary_available
    )
    conan_out = tmp / "conan"
    build_root = tmp / "build"
    install_prefix = tmp / "install"

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "SERIALARM_TEST_LOG": str(log),
            "CONAN_DIR": str(conan_out),
            "CORE_BUILD_DIR": str(build_root / "core"),
            "INSTALL_PREFIX": str(install_prefix),
            "JOBS": "2",
            "CMAKE_PREFIX_PATH": "/opt/ros/humble:/opt/openrobots:/usr/local",
            "AMENT_PREFIX_PATH": "/opt/ros/humble",
            "COLCON_PREFIX_PATH": "/opt/ros/humble",
        }
    )
    if allow_source is not None:
        env["SERIAL_ARM_ALLOW_SOURCE_BUILD"] = allow_source
    else:
        env.pop("SERIAL_ARM_ALLOW_SOURCE_BUILD", None)

    args = [str(SCRIPT)]
    if robot:
        args += ["--robot", "dm_arm"]
    if with_tests:
        args += ["--with-tests"]

    proc = subprocess.run(
        args,
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    calls = log.read_text(encoding="utf-8") if log.exists() else ""
    return proc, calls, install_prefix


def test_system_dependencies_bypass_conan() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-system-deps-") as tmp_raw:
        proc, calls, install_prefix = run_bootstrap(
            Path(tmp_raw),
            system_deps=True,
            binary_available=False,
            allow_source=None,
            robot=True,
        )
        if proc.returncode != 0:
            raise AssertionError(f"system-dependency bootstrap failed:\n{proc.stdout}")
        if "conan " in calls:
            raise AssertionError(f"system dependency path unexpectedly called Conan:\n{calls}")
        if "CMAKE_TOOLCHAIN_FILE=" in calls:
            raise AssertionError(f"system dependency path unexpectedly used Conan toolchain:\n{calls}")
        if "-DSERIAL_ARM_ENABLE_ROS2=OFF" not in calls:
            raise AssertionError(f"standalone core build did not disable ROS2 integration:\n{calls}")
        env_lines = [line for line in calls.splitlines() if line.startswith("env ")]
        if not env_lines:
            raise AssertionError(f"fake cmake did not record environment:\n{calls}")
        for line in env_lines:
            if "/opt/ros/" in line:
                raise AssertionError(f"standalone cmake inherited ROS prefix:\n{line}")
            if "AMENT_PREFIX_PATH=" not in line or "COLCON_PREFIX_PATH=" not in line:
                raise AssertionError(f"missing standalone environment record:\n{line}")
        if not any("/opt/openrobots" in line for line in env_lines):
            raise AssertionError(f"non-ROS system prefix was removed unexpectedly:\n{calls}")
        for fragment in (
            "src/serial_arm/core",
            "src/robot_supports/protocol/damiao_usb2can",
            "src/robot_supports/hardware/damiao",
            "src/robot_supports/profiles",
            "src/robot_supports/robots/dm_arm/description",
        ):
            if fragment not in calls:
                raise AssertionError(f"system DM-Arm bootstrap skipped {fragment}\n{calls}")
        core_configures = [line for line in calls.splitlines() if line.startswith("cmake -S ") and "src/serial_arm/core" in line]
        if not core_configures or "-DBUILD_TESTING=OFF" not in core_configures[0]:
            raise AssertionError(f"default bootstrap must configure BUILD_TESTING=OFF:\n{calls}")
        if any(line.startswith("ctest ") for line in calls.splitlines()):
            raise AssertionError(f"default bootstrap unexpectedly ran ctest:\n{calls}")
        probe_lines = [line for line in calls.splitlines() if "serial_arm_dependency_probe_src" in line]
        if not probe_lines or "-DSERIAL_ARM_PROBE_TEST_DEPS=OFF" not in probe_lines[0]:
            raise AssertionError(f"default dependency probe should not require GTest:\n{calls}")
        setup = (install_prefix / "setup.bash").read_text(encoding="utf-8")
        if "conanrun.sh" in setup:
            raise AssertionError(f"system setup references Conan runtime unexpectedly:\n{setup}")
        for fragment in ("SERIAL_ARM_RESOURCE_PATH", "LD_LIBRARY_PATH", "PATH="):
            if fragment not in setup:
                raise AssertionError(f"setup.bash missing {fragment}:\n{setup}")


def test_conan_prefers_binary_and_forces_cppstd17() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-conan-binary-") as tmp_raw:
        proc, calls, _ = run_bootstrap(
            Path(tmp_raw),
            system_deps=False,
            binary_available=True,
            allow_source="0",
        )
        if proc.returncode != 0:
            raise AssertionError(f"binary-only Conan bootstrap failed:\n{proc.stdout}")
        installs = [line for line in calls.splitlines() if line.startswith("conan install ")]
        if len(installs) != 1:
            raise AssertionError(f"expected one Conan install, got {installs}")
        install = installs[0]
        if "compiler.cppstd=17" not in install:
            raise AssertionError(f"Conan install did not force cppstd=17:\n{install}")
        if "&:with_tests=False" not in install:
            raise AssertionError(f"default Conan install unexpectedly enabled test dependencies:\n{install}")
        if "--build=missing" in install:
            raise AssertionError(f"binary-first install unexpectedly enabled source builds:\n{install}")


def test_conan_source_build_requires_explicit_opt_in() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-conan-no-source-") as tmp_raw:
        proc, calls, _ = run_bootstrap(
            Path(tmp_raw),
            system_deps=False,
            binary_available=False,
            allow_source="0",
        )
        if proc.returncode == 0:
            raise AssertionError("bootstrap unexpectedly succeeded without binary or source opt-in")
        if "SERIAL_ARM_ALLOW_SOURCE_BUILD=1" not in proc.stdout:
            raise AssertionError(f"missing source-build opt-in guidance:\n{proc.stdout}")
        if "--build=missing" in calls:
            raise AssertionError(f"bootstrap started source builds without opt-in:\n{calls}")


def test_conan_source_build_runs_after_explicit_opt_in() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-conan-source-") as tmp_raw:
        proc, calls, _ = run_bootstrap(
            Path(tmp_raw),
            system_deps=False,
            binary_available=False,
            allow_source="1",
        )
        if proc.returncode != 0:
            raise AssertionError(f"source-build fallback failed:\n{proc.stdout}")
        installs = [line for line in calls.splitlines() if line.startswith("conan install ")]
        if len(installs) != 2:
            raise AssertionError(f"expected binary attempt + source fallback, got:\n{calls}")
        if "--build=missing" in installs[0] or "--build=missing" not in installs[1]:
            raise AssertionError(f"wrong Conan fallback order:\n{installs}")
        if any("compiler.cppstd=17" not in line for line in installs):
            raise AssertionError(f"cppstd=17 missing from Conan install attempts:\n{installs}")


def test_with_tests_explicitly_builds_and_runs_ctest() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-with-tests-") as tmp_raw:
        proc, calls, _ = run_bootstrap(
            Path(tmp_raw),
            system_deps=True,
            binary_available=False,
            allow_source=None,
            with_tests=True,
        )
        if proc.returncode != 0:
            raise AssertionError(f"--with-tests bootstrap failed:\n{proc.stdout}")
        core_configures = [line for line in calls.splitlines() if line.startswith("cmake -S ") and "src/serial_arm/core" in line]
        if not core_configures or "-DBUILD_TESTING=ON" not in core_configures[0]:
            raise AssertionError(f"--with-tests must configure BUILD_TESTING=ON:\n{calls}")
        if not any(line.startswith("ctest --test-dir ") for line in calls.splitlines()):
            raise AssertionError(f"--with-tests did not run ctest:\n{calls}")
        probe_lines = [line for line in calls.splitlines() if "serial_arm_dependency_probe_src" in line]
        if not probe_lines or "-DSERIAL_ARM_PROBE_TEST_DEPS=ON" not in probe_lines[0]:
            raise AssertionError(f"--with-tests dependency probe should include GTest:\n{calls}")


def test_with_tests_conan_requests_gtest_dependencies() -> None:
    with tempfile.TemporaryDirectory(prefix="serialarm-with-tests-conan-") as tmp_raw:
        proc, calls, _ = run_bootstrap(
            Path(tmp_raw),
            system_deps=False,
            binary_available=True,
            allow_source="0",
            with_tests=True,
        )
        if proc.returncode != 0:
            raise AssertionError(f"--with-tests Conan bootstrap failed:\n{proc.stdout}")
        installs = [line for line in calls.splitlines() if line.startswith("conan install ")]
        if len(installs) != 1 or "&:with_tests=True" not in installs[0]:
            raise AssertionError(f"--with-tests did not enable Conan test dependencies:\n{calls}")


def main() -> int:
    test_system_dependencies_bypass_conan()
    test_conan_prefers_binary_and_forces_cppstd17()
    test_conan_source_build_requires_explicit_opt_in()
    test_conan_source_build_runs_after_explicit_opt_in()
    test_with_tests_explicitly_builds_and_runs_ctest()
    test_with_tests_conan_requests_gtest_dependencies()
    print("BOOTSTRAP_STANDALONE_CLI_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
