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


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="serialarm-bootstrap-test-") as tmp_raw:
        tmp = Path(tmp_raw)
        fake_bin = tmp / "bin"
        fake_bin.mkdir()
        log = tmp / "calls.log"
        conan_out = tmp / "conan"
        build_root = tmp / "build"
        install_prefix = tmp / "install"

        write_executable(
            fake_bin / "conan",
            r'''#!/usr/bin/env bash
set -euo pipefail
printf 'conan %s\n' "$*" >> "$SERIALARM_TEST_LOG"
if [[ "${1:-}" == "--version" ]]; then
  printf 'Conan version 2.9.0\n'
  exit 0
fi
if [[ "${1:-}" == "profile" && "${2:-}" == "path" ]]; then
  exit 1
fi
if [[ "${1:-}" == "profile" && "${2:-}" == "detect" ]]; then
  exit 0
fi
if [[ "${1:-}" == "install" ]]; then
  out=''
  for arg in "$@"; do
    case "$arg" in
      --output-folder=*) out="${arg#--output-folder=}" ;;
    esac
  done
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
        write_executable(
            fake_bin / "cmake",
            r'''#!/usr/bin/env bash
set -euo pipefail
printf 'cmake %s\n' "$*" >> "$SERIALARM_TEST_LOG"
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

        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{fake_bin}:{env['PATH']}",
                "SERIALARM_TEST_LOG": str(log),
                "CONAN_DIR": str(conan_out),
                "CORE_BUILD_DIR": str(build_root / "core"),
                "INSTALL_PREFIX": str(install_prefix),
                "JOBS": "2",
            }
        )
        proc = subprocess.run(
            [str(SCRIPT), "--robot", "dm_arm"],
            cwd=ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if proc.returncode != 0:
            raise AssertionError(f"bootstrap failed with {proc.returncode}:\n{proc.stdout}")

        calls = log.read_text(encoding="utf-8")
        required = [
            "src/serial_arm/core",
            "src/robot_supports/protocol/damiao_usb2can",
            "src/robot_supports/hardware/damiao",
            "src/robot_supports/profiles",
            "src/robot_supports/robots/dm_arm/description",
        ]
        missing = [fragment for fragment in required if fragment not in calls]
        if missing:
            raise AssertionError(f"DM-Arm bootstrap skipped components: {missing}\nCalls:\n{calls}")

        setup = install_prefix / "setup.bash"
        if not setup.is_file():
            raise AssertionError(f"bootstrap did not create {setup}")
        setup_text = setup.read_text(encoding="utf-8")
        for fragment in ("conanrun.sh", "SERIAL_ARM_RESOURCE_PATH", "LD_LIBRARY_PATH"):
            if fragment not in setup_text:
                raise AssertionError(f"setup.bash missing {fragment}:\n{setup_text}")

        print("BOOTSTRAP_STANDALONE_DM_ARM_CLI_PASS")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
