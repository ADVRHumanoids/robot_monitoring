#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PROJECT_VERSION="$(sed -nE 's/^project\(robot_monitoring VERSION ([0-9]+\.[0-9]+\.[0-9]+)\)$/\1/p' CMakeLists.txt)"
if [[ -n "${TRAVIS_TAG:-}" && "$TRAVIS_TAG" != "v${PROJECT_VERSION}" ]]; then
    echo "Release tag '$TRAVIS_TAG' does not match CMake project version '$PROJECT_VERSION'" >&2
    exit 1
fi

# delete old artifacts
rm -rf build_output && mkdir build_output
rm -rf travis/docker/context && mkdir -p travis/docker/context/robot_monitoring

# copy whole repo into docker context
cp -r * travis/docker/context/robot_monitoring || true

# build docker image
docker build travis/docker -t travis_build_image

# build client via docker
docker run -i --rm \
 -v $PWD/build_output:/home/user/build_output \
 -e BUILD_SKIP_ANDROID -e BUILD_SKIP_LINUX \
 travis_build_image bash -i /home/user/build.bash

# restore correct ownership
sudo chown -R $USER build_output

# package and validate linux app
bash travis/package_appimage.bash
bash travis/test_appimage.bash

# sign apk
cd build_output
zipalign -p 4 xbot2_gui_client_android_arm64_v8a.apk xbot2_gui_client_android_arm64_v8a_signed.apk
printf '%s\n' "$KEYSTORE_PWD" | apksigner sign --ks-key-alias app --ks ../my.keystore xbot2_gui_client_android_arm64_v8a_signed.apk
