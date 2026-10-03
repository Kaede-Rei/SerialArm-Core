#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
command -v python3 >/dev/null || { echo '需要 Python 3.10+：请先安装 python3' >&2; exit 2; }
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$ROOT_DIR/tools/install/main.py" "$@"
