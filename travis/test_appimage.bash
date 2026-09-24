#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly OUTPUT_DIR="$REPO_ROOT/build_output"
readonly APPDIR="$OUTPUT_DIR/xbot2_gui_client_x86_64"

PROJECT_VERSION="$(sed -nE 's/^project\(robot_monitoring VERSION ([0-9]+\.[0-9]+\.[0-9]+)\)$/\1/p' "$REPO_ROOT/CMakeLists.txt")"
readonly APPIMAGE_NAME="xbot2-gui-${PROJECT_VERSION}-x86_64.AppImage"
readonly APPIMAGE_PATH="$OUTPUT_DIR/$APPIMAGE_NAME"

test -x "$APPDIR/AppRun"
test -x "$APPDIR/bin/xbot2_gui"
test -x "$APPIMAGE_PATH"
test -f "$APPDIR/xbot2-gui.desktop"
test -f "$APPDIR/xbot2-gui.png"
test -L "$APPDIR/.DirIcon"
for soname in \
    libxcb-cursor.so.0 \
    libxcb-icccm.so.4 \
    libxcb-image.so.0 \
    libxcb-keysyms.so.1 \
    libxcb-render-util.so.0 \
    libxcb-shape.so.0 \
    libxcb-xkb.so.1 \
    libxkbcommon-x11.so.0 \
    libxcb-util.so.1; do
    test -f "$APPDIR/lib/$soname"
done

desktop-file-validate "$APPDIR/xbot2-gui.desktop"
(
    cd "$OUTPUT_DIR"
    sha256sum --check --strict "${APPIMAGE_NAME}.sha256"
)

readonly EXTRACT_DIR="$(mktemp -d)"
readonly LOG_FILE="$(mktemp)"
trap 'rm -rf "$EXTRACT_DIR" "$LOG_FILE"' EXIT
(
    cd "$EXTRACT_DIR"
    "$APPIMAGE_PATH" --appimage-extract >/dev/null
)
grep -Fxq "X-AppImage-Version=$PROJECT_VERSION" \
    "$EXTRACT_DIR/squashfs-root/xbot2-gui.desktop"
test "$(readlink "$EXTRACT_DIR/squashfs-root/.DirIcon")" = 'xbot2-gui.png'

export QTWEBENGINE_CHROMIUM_FLAGS="--disable-gpu"
export QT_QUICK_BACKEND="software"
xvfb-run -a "$APPDIR/AppRun" --version
xvfb-run -a "$APPIMAGE_PATH" --version
xvfb-run -a "$APPIMAGE_PATH" --appimage-extract-and-run --version

set +e
timeout 15s env QT_DEBUG_PLUGINS=1 xvfb-run -a "$APPIMAGE_PATH" >"$LOG_FILE" 2>&1
STARTUP_STATUS=$?
set -e

if [[ "$STARTUP_STATUS" -ne 0 && "$STARTUP_STATUS" -ne 124 ]]; then
    cat "$LOG_FILE" >&2
    echo "AppImage startup smoke test failed with status $STARTUP_STATUS" >&2
    exit "$STARTUP_STATUS"
fi

if grep -Eiq 'could not load the qt platform plugin|module .* is not installed|error while loading shared libraries' "$LOG_FILE"; then
    cat "$LOG_FILE" >&2
    echo "AppImage startup reported a missing runtime component" >&2
    exit 1
fi
