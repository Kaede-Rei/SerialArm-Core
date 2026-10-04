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
    [[ -f .install/setup.bash ]] && source .install/setup.bash
    DOCTOR_PYTHON="${SERIAL_ARM_LAUNCHER_PYTHON:-$ROOT_DIR/.install/gui-venv/bin/python}"
    [[ -x "$DOCTOR_PYTHON" ]] || DOCTOR_PYTHON="$(command -v python3)"
    exec "$DOCTOR_PYTHON" -u "$ROOT_DIR/apps/launcher/backend/doctor.py" "$@"
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
[[ -f .install/setup.bash ]] && source .install/setup.bash
export SERIAL_ARM_LAUNCHER_PYTHON="$ROOT_DIR/.install/gui-venv/bin/python"
exec "$ELECTRON" "$ROOT_DIR/apps/launcher"
