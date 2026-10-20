#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$ROOT/packaging/linux"
APP="$ROOT/apps/launcher"
OUTPUT="$ROOT/dist"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SKIP_INSTALL=0
FORCE_REBUILD=0

fail() { printf '[SerialArm] Error: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'HELP'
Usage: ./build_linux.sh [--reuse-install | --rebuild]

Build the complete ROS2 / MoveIt / DM-Arm / GUI Linux release
This is the same component set as install.sh preset ros2 (interactive choice 4)

  Default            Reuse a verified full ROS2 installation, or install it if missing
  --reuse-install    Require an existing, verified full ROS2 installation
  --rebuild          Reinstall the full ROS2 preset before packaging

Build host: Ubuntu 22.04 x86_64, ROS2 Humble, Node.js 20+
Outputs: dist/SerialArm-Core-linux-x86_64.AppImage
         dist/SerialArm-Core-linux-amd64.deb
HELP
}
while (( $# > 0 )); do
    case "$1" in
      --reuse-install) SKIP_INSTALL=1 ;;
      --rebuild) FORCE_REBUILD=1 ;;
      -h|--help) usage; exit 0 ;;
      *) usage >&2; fail "Unknown argument: $1" ;;
    esac
    shift
done

[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || fail 'Linux x86_64 is required'
[[ -f /etc/os-release ]] || fail 'Ubuntu 22.04 is required'
# Use the same ABI and ROS2 distribution that the shipped DM-Arm stack was built for
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || fail 'Build on Ubuntu 22.04 with ROS2 Humble'
[[ "$ROS_DISTRO" == humble ]] || fail 'This Linux release targets ROS2 Humble'
[[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS2 Humble not found in /opt/ros/humble'
for c in node npm python3; do command -v "$c" >/dev/null 2>&1 || fail "Missing command: $c"; done
[[ "$(node -p 'Number(process.versions.node.split(".")[0])')" -ge 20 ]] || fail 'Node.js 20+ is required'
[[ -x /usr/bin/python3 ]] || fail 'System Python /usr/bin/python3 is required'

if (( FORCE_REBUILD && SKIP_INSTALL )); then
    fail '--rebuild and --reuse-install cannot be combined'
fi
if (( FORCE_REBUILD )) || ! /usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only >/dev/null 2>&1; then
    if (( SKIP_INSTALL )); then
        /usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only
        fail 'Complete ROS2 installation is unavailable'
    fi
    echo '[SerialArm] Building complete ROS2 + MoveIt installation (install.sh choice 4)'
    "$ROOT/install.sh" --preset ros2 --robot dm_arm --with-gui --with-python --with-terminal --with-moveit --without-tests --ros-distro humble --yes
else
    echo '[SerialArm] Reusing verified ROS2 + MoveIt installation'
fi

# Check a verified full-preset installation before creating or replacing any release assets
PREFIX="$(/usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only)" || fail 'The complete ROS2 installation did not pass packaging checks'
[[ -f "$PREFIX/local_setup.bash" ]] || fail "ROS2 overlay missing local_setup.bash: $PREFIX"
[[ -x "$APP/node_modules/electron/dist/electron" ]] || fail 'GUI Electron runtime missing; repeat full installation'
[[ -f "$APP/renderer/vendor/three/build/three.module.js" ]] || fail 'GUI model viewer assets missing; repeat full installation'

BUILDER="$PKG/node_modules/.bin/electron-builder"
if [[ ! -x "$BUILDER" ]]; then
    echo '[SerialArm] Installing electron-builder'
    npm --prefix "$PKG" install --no-audit --no-fund --ignore-scripts --package-lock=false
fi
[[ -x "$BUILDER" ]] || fail 'electron-builder not available'

mkdir -p "$OUTPUT"
STAGING="$(mktemp -d "$OUTPUT/.build-linux.XXXXXXXX")"
cleanup() { [[ ! -d "${STAGING:-}" ]] || rm -rf -- "$STAGING"; }
trap cleanup EXIT

# The GUI installer already installed and validated this exact Electron runtime
# Reuse it as electron-builder's source instead of downloading the same archive again
# electronDist accepts a directory containing electron-v<version>-linux-x64.zip
ELECTRON_LOCAL="$STAGING/electron-dist"
/usr/bin/python3 "$PKG/local_electron.py" --launcher "$APP" --output "$ELECTRON_LOCAL" \
    || fail 'Cannot prepare the locally installed Electron runtime'

# local_setup.bash extends the already sourced distribution environment
# Do not enable nounset when sourcing ROS2 setup scripts
(
    set +u
    source /opt/ros/humble/setup.bash
    source "$PREFIX/local_setup.bash"
    set -u
    /usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --stage "$STAGING"
)

printf '[SerialArm] Building AppImage and deb\n'
(cd "$STAGING/project" && "$BUILDER" --linux AppImage deb --x64 --publish never \
    "-c.electronDist=$ELECTRON_LOCAL" \
    "-c.directories.output=$STAGING/output")

APPIMAGE='SerialArm-Core-linux-x86_64.AppImage'
DEB='SerialArm-Core-linux-amd64.deb'
for name in "$APPIMAGE" "$DEB"; do
    [[ -s "$STAGING/output/$name" ]] || fail "Missing or empty release asset: $name"
done
if command -v dpkg-deb >/dev/null 2>&1; then
    dpkg-deb --field "$STAGING/output/$DEB" >/dev/null || fail 'Invalid deb metadata'
fi
# Publish only after both targets succeeded
mv -f -- "$STAGING/output/$APPIMAGE" "$OUTPUT/$APPIMAGE"
mv -f -- "$STAGING/output/$DEB" "$OUTPUT/$DEB"
echo '[SerialArm] Linux release packages ready'
ls -lh -- "$OUTPUT/$APPIMAGE" "$OUTPUT/$DEB"
sha256sum -- "$OUTPUT/$APPIMAGE" "$OUTPUT/$DEB" | tee "$OUTPUT/SHA256SUMS"
echo '[SerialArm] Verify on a separate Ubuntu 22.04 + ROS2 Humble system before publishing'
