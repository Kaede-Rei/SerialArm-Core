#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$ROOT/packaging/linux"
APP="$ROOT/apps/launcher"
OUTPUT="$ROOT/dist"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SKIP_INSTALL=0
FORCE_REBUILD=0
WITH_ROS2=0
STAGE_ARGS=()

fail() { printf '[SerialArm] Error: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'HELP'
Usage: ./build_linux.sh [--reuse-install | --rebuild] [--with-ros2]

Build a standalone Core / Python / DM-Arm / GUI Linux release
ROS2 and MoveIt are optional adapters and are excluded by default

  Default            Reuse a matching standalone installation, or rebuild it
  --reuse-install    Require an installation matching the current sources
  --rebuild          Reinstall the selected components before packaging
  --with-ros2        Include ROS2 / MoveIt adapters for hosts using ROS2 Humble

Build host: Ubuntu 22.04 x86_64, Node.js 20+
Run only when preparing a Release; normal changes do not require packaging
Outputs: dist/SerialArm-Core-linux-x86_64.AppImage
         dist/SerialArm-Core-linux-amd64.deb
HELP
}
while (( $# > 0 )); do
    case "$1" in
      --reuse-install) SKIP_INSTALL=1 ;;
      --rebuild) FORCE_REBUILD=1 ;;
      --with-ros2) WITH_ROS2=1; STAGE_ARGS=(--with-ros2) ;;
      -h|--help) usage; exit 0 ;;
      *) usage >&2; fail "Unknown argument: $1" ;;
    esac
    shift
done

[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || fail 'Linux x86_64 is required'
[[ -f /etc/os-release ]] || fail 'Ubuntu 22.04 is required'
# Use the same ABI and ROS2 distribution that the shipped DM-Arm stack was built for
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || fail 'Build on Ubuntu 22.04'
if (( WITH_ROS2 )); then
    [[ "$ROS_DISTRO" == humble ]] || fail 'The optional adapters target ROS2 Humble'
    [[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS2 adapters need /opt/ros/humble; omit --with-ros2 for a standalone release'
fi
for c in node npm python3 flock; do command -v "$c" >/dev/null 2>&1 || fail "Missing command: $c"; done
command -v dpkg-deb >/dev/null 2>&1 || fail 'Missing command: dpkg-deb; install the Release build dependency with: sudo apt install dpkg'
command -v unsquashfs >/dev/null 2>&1 || fail 'Missing command: unsquashfs; install the Release build dependency with: sudo apt install squashfs-tools'
[[ "$(node -p 'Number(process.versions.node.split(".")[0])')" -ge 20 ]] || fail 'Node.js 20+ is required'
[[ -x /usr/bin/python3 ]] || fail 'System Python /usr/bin/python3 is required'

if (( FORCE_REBUILD && SKIP_INSTALL )); then
    fail '--rebuild and --reuse-install cannot be combined'
fi
if (( FORCE_REBUILD )) || ! /usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only "${STAGE_ARGS[@]}" >/dev/null 2>&1; then
    if (( SKIP_INSTALL )); then
        /usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only "${STAGE_ARGS[@]}"
        fail 'The selected native installation is unavailable'
    fi
    if (( WITH_ROS2 )); then
        "$ROOT/install.sh" --preset ros2 --robot dm_arm --with-gui --with-python --with-terminal --with-moveit --ros-distro humble --jobs 1 --yes
    else
        "$ROOT/install.sh" --preset standalone --robot dm_arm --with-python --jobs 1 --yes
    fi
else
    echo '[SerialArm] Reusing the verified selected installation'
fi

# Keep the installer and another packager from replacing files during staging
exec {INSTALL_LOCK}>"$ROOT/.install/install.lock"
flock -n "$INSTALL_LOCK" || fail 'Another installation or packaging operation is running'

# Check a verified full-preset installation before creating or replacing any release assets
PREFIX="$(/usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --check-only "${STAGE_ARGS[@]}")" || fail 'The selected installation did not pass packaging checks'
[[ -x "$APP/node_modules/electron/dist/electron" ]] || fail 'GUI Electron runtime missing; repeat full installation'
[[ -f "$APP/renderer/vendor/three/build/three.module.js" ]] || fail 'GUI model viewer assets missing; repeat full installation'

BUILDER="$PKG/node_modules/.bin/electron-builder"
if [[ ! -x "$BUILDER" ]]; then
    echo '[SerialArm] Installing electron-builder'
    npm --prefix "$PKG" ci --no-audit --no-fund --ignore-scripts
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

# Core staging and validation do not source host ROS, even for adapter releases
/usr/bin/python3 "$PKG/stage.py" --root "$ROOT" --stage "$STAGING" "${STAGE_ARGS[@]}"

printf '[SerialArm] Building AppImage and deb\n'
(cd "$STAGING/project" && "$BUILDER" --linux AppImage deb --x64 --publish never \
    "-c.electronDist=$ELECTRON_LOCAL" \
    "-c.directories.output=$STAGING/output")

APPIMAGE='SerialArm-Core-linux-x86_64.AppImage'
DEB='SerialArm-Core-linux-amd64.deb'
for name in "$APPIMAGE" "$DEB"; do
    [[ -s "$STAGING/output/$name" ]] || fail "Missing or empty release asset: $name"
done
dpkg-deb --field "$STAGING/output/$DEB" >/dev/null || fail 'Invalid deb metadata'
/usr/bin/python3 "$PKG/verify.py" --root "$ROOT" \
    --deb "$STAGING/output/$DEB" --appimage "$STAGING/output/$APPIMAGE" "${STAGE_ARGS[@]}" \
    || fail 'Release payload verification failed'
# Publish only after both targets succeeded
mv -f -- "$STAGING/output/$APPIMAGE" "$OUTPUT/$APPIMAGE"
mv -f -- "$STAGING/output/$DEB" "$OUTPUT/$DEB"
echo '[SerialArm] Linux release packages ready'
ls -lh -- "$OUTPUT/$APPIMAGE" "$OUTPUT/$DEB"
(cd "$OUTPUT" && sha256sum -- "$APPIMAGE" "$DEB") | tee "$OUTPUT/SHA256SUMS"
echo '[SerialArm] Verify Core and GUI on a ROS-free Ubuntu 22.04 system before publishing; check adapters separately when included'
