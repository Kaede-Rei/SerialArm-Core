#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo 'Usage: ./launch.sh [--doctor [--profile NAME] [--profile-file FILE] [--resource-paths ROOTS]]'
    exit 0
fi
if [[ "${1:-}" == "--doctor" ]]; then
    shift
    if [[ -f .install/setup.bash ]] && python3 "$ROOT_DIR/tools/install/main.py" --check-environment; then
        : # Desktop and diagnostics construct a native environment without ROS
    fi
    DOCTOR_PYTHON="${SERIAL_ARM_LAUNCHER_PYTHON:-$ROOT_DIR/.install/gui-venv/bin/python}"
    [[ -x "$DOCTOR_PYTHON" ]] || DOCTOR_PYTHON="$(command -v python3)"
    exec node - "$ROOT_DIR" "$DOCTOR_PYTHON" "$@" <<'JS'
const {spawnSync} = require('node:child_process');
const path = require('node:path');
const [root, python, ...args] = process.argv.slice(2);
const {developmentBackend} = require(path.join(root, 'apps/launcher/desktop/native-runtime.cjs'));
const result = spawnSync(python, ['-u', path.join(root, 'apps/launcher/backend/doctor.py'), ...args],
  {env: developmentBackend(root).env, stdio: 'inherit'});
if (result.error) process.stderr.write(result.error.message + '\n');
process.exit(result.status ?? 1);
JS
fi
if [[ $# -ne 0 ]]; then
    echo 'Usage: ./launch.sh [--doctor [--profile NAME] [--profile-file FILE] [--resource-paths ROOTS]]' >&2
    exit 2
fi
ELECTRON="$ROOT_DIR/apps/launcher/node_modules/electron/dist/electron"
THREE_VENDOR="$ROOT_DIR/apps/launcher/renderer/vendor/three/build/three.module.js"
if [[ ! -x "$ELECTRON" || ! -x "$ROOT_DIR/.install/gui-venv/bin/python" ]]; then
    echo 'GUI 依赖未就绪，请执行 ./install.sh --gui-only --yes，或重新选择带 GUI 的安装预设' >&2
    exit 1
fi
if [[ ! -f "$THREE_VENDOR" ]]; then
    if [[ -f "$ROOT_DIR/apps/launcher/node_modules/three/build/three.module.js" ]]; then
        node "$ROOT_DIR/apps/launcher/scripts/vendor-three.cjs"
    else
        echo 'Model 依赖未就绪，请执行 ./install.sh --gui-only --yes' >&2
        exit 1
    fi
fi
if [[ -f .install/setup.bash ]]; then
    python3 "$ROOT_DIR/tools/install/main.py" --check-environment || exit 1
fi
export SERIAL_ARM_LAUNCHER_PYTHON="$ROOT_DIR/.install/gui-venv/bin/python"
BACKEND="$ROOT_DIR/apps/launcher/backend/bridge.py"
if ! BACKEND_ERROR="$("$SERIAL_ARM_LAUNCHER_PYTHON" -u "$BACKEND" --check 2>&1 >/dev/null)"; then
    echo 'Launcher 后端启动自检失败：' >&2
    echo "$BACKEND_ERROR" >&2
    echo '可先执行 ./install.sh --gui-only --yes 修复 GUI Python 依赖' >&2
    exit 1
fi
exec "$ELECTRON" "$ROOT_DIR/apps/launcher"
