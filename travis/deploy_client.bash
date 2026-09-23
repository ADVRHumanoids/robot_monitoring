
if [ -z "${TRAVIS_TAG:-}" ]
    then echo "Not a tag build, will not upload to gh releases"
    exit 0
fi

set -e

PROJECT_VERSION="$(sed -nE 's/^project\(robot_monitoring VERSION ([0-9]+\.[0-9]+\.[0-9]+)\)$/\1/p' CMakeLists.txt)"
APPIMAGE="build_output/xbot2-gui-${PROJECT_VERSION}-x86_64.AppImage"
APK="build_output/xbot2_gui_client_android_arm64_v8a_signed.apk"
WINDOWS_INSTALLER="build_output/xbot2-gui-${PROJECT_VERSION}-windows-x86_64-setup.exe"

if [ "$TRAVIS_TAG" != "v${PROJECT_VERSION}" ]; then
    echo "Release tag '$TRAVIS_TAG' does not match CMake project version '$PROJECT_VERSION'" >&2
    exit 1
fi

for artifact in \
    "$APK" \
    "$APPIMAGE" \
    "${APPIMAGE}.sha256" \
    "$WINDOWS_INSTALLER" \
    "${WINDOWS_INSTALLER}.sha256"; do
    if [ ! -f "$artifact" ]; then
        echo "Missing release artifact: $artifact" >&2
        exit 1
    fi
done

# create release
gh release create "$TRAVIS_TAG" \
 "${APK}#Android APK" \
 "${APPIMAGE}#Linux AppImage" \
 "${APPIMAGE}.sha256#Linux AppImage SHA-256" \
 "${WINDOWS_INSTALLER}#Windows x86-64 Installer" \
 "${WINDOWS_INSTALLER}.sha256#Windows Installer SHA-256" \
 -t 'XBot2 GUI Client' \
 -n '' \
 --verify-tag
