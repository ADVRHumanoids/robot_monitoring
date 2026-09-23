#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly OUTPUT_DIR="$REPO_ROOT/build_output"
readonly APPDIR="$OUTPUT_DIR/xbot2_gui_client_x86_64"
readonly APPIMAGETOOL_VERSION="1.9.1"
readonly APPIMAGETOOL_SHA256="ed4ce84f0d9caff66f50bcca6ff6f35aae54ce8135408b3fa33abfc3cb384eb0"
readonly APPIMAGE_RUNTIME_VERSION="20251108"
readonly APPIMAGE_RUNTIME_SHA256="2fca8b443c92510f1483a883f60061ad09b46b978b2631c807cd873a47ec260d"

PROJECT_VERSION="$(sed -nE 's/^project\(robot_monitoring VERSION ([0-9]+\.[0-9]+\.[0-9]+)\)$/\1/p' "$REPO_ROOT/CMakeLists.txt")"
if [[ -z "$PROJECT_VERSION" ]]; then
    echo "Unable to read robot_monitoring version from CMakeLists.txt" >&2
    exit 1
fi

if [[ -n "${TRAVIS_TAG:-}" ]]; then
    TAG_VERSION="${TRAVIS_TAG#v}"
    if [[ "$TRAVIS_TAG" != v* || "$TAG_VERSION" != "$PROJECT_VERSION" ]]; then
        echo "Release tag '$TRAVIS_TAG' does not match CMake project version '$PROJECT_VERSION'" >&2
        exit 1
    fi
fi

for required_path in \
    "$APPDIR/AppRun" \
    "$APPDIR/xbot2-gui.desktop" \
    "$APPDIR/xbot2-gui.png" \
    "$APPDIR/bin/xbot2_gui" \
    "$APPDIR/bin/QtWebEngineProcess"; do
    if [[ ! -e "$required_path" ]]; then
        echo "Missing required AppDir path: $required_path" >&2
        exit 1
    fi
done

chmod +x "$APPDIR/AppRun" "$APPDIR/bin/xbot2_gui"
ln -sfn xbot2-gui.png "$APPDIR/.DirIcon"

grep -Fxq 'Exec=xbot2_gui' "$APPDIR/xbot2-gui.desktop"
grep -Fxq 'Icon=xbot2-gui' "$APPDIR/xbot2-gui.desktop"

if command -v desktop-file-validate >/dev/null 2>&1; then
    desktop-file-validate "$APPDIR/xbot2-gui.desktop"
fi

readonly TOOL_DIR="$(mktemp -d)"
trap 'rm -rf "$TOOL_DIR"' EXIT

curl --fail --location --silent --show-error \
    "https://github.com/AppImage/appimagetool/releases/download/${APPIMAGETOOL_VERSION}/appimagetool-x86_64.AppImage" \
    --output "$TOOL_DIR/appimagetool.AppImage"
echo "$APPIMAGETOOL_SHA256  $TOOL_DIR/appimagetool.AppImage" | sha256sum --check --strict

curl --fail --location --silent --show-error \
    "https://github.com/AppImage/type2-runtime/releases/download/${APPIMAGE_RUNTIME_VERSION}/runtime-x86_64" \
    --output "$TOOL_DIR/runtime-x86_64"
echo "$APPIMAGE_RUNTIME_SHA256  $TOOL_DIR/runtime-x86_64" | sha256sum --check --strict

chmod +x "$TOOL_DIR/appimagetool.AppImage"
(
    cd "$TOOL_DIR"
    ./appimagetool.AppImage --appimage-extract >/dev/null
)

readonly APPIMAGE_NAME="xbot2-gui-${PROJECT_VERSION}-x86_64.AppImage"
readonly APPIMAGE_PATH="$OUTPUT_DIR/$APPIMAGE_NAME"

ARCH=x86_64 VERSION="$PROJECT_VERSION" \
    "$TOOL_DIR/squashfs-root/AppRun" \
    --runtime-file "$TOOL_DIR/runtime-x86_64" \
    --comp zstd \
    "$APPDIR" \
    "$APPIMAGE_PATH"

chmod +x "$APPIMAGE_PATH"
(
    cd "$OUTPUT_DIR"
    sha256sum "$APPIMAGE_NAME" > "${APPIMAGE_NAME}.sha256"
)

printf '%s\n' "$APPIMAGE_PATH"
