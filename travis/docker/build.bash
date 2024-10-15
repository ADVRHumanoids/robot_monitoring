echo user | sudo -S chown user /home/user/build_output

# test 
set -e
touch /home/user/build_output/BUILD_IN_PROGRESS

# tbd: from bashrc
export ANDROID_SDK_ROOT=$HOME/Android/Sdk
export ANDROID_NDK_ROOT=$ANDROID_SDK_ROOT/ndk/25.1.8937393

# android
mkdir build && cd build
~/Qt/$QT_VER/android_arm64_v8a/bin/qt-cmake -DCMAKE_BUILD_TYPE=Release ../robot_monitoring
make -j $(nproc)
cp xbot2_gui/android-build/build/outputs/apk/release/android-build-release-unsigned.apk /home/user/build_output/xbot2_gui_client_android_arm64_v8a.apk

cd ..
rm -rf build

# linux
export PATH="$HOME/Qt/Tools/CMake/bin:$PATH"

mkdir build && cd build
~/Qt/$QT_VER/gcc_64/bin/qt-cmake -DCMAKE_INSTALL_PREFIX=/home/user/build_output/xbot2_gui_client_x86_64 -DCMAKE_BUILD_TYPE=Release ../robot_monitoring
make install -j $(nproc)

touch /home/user/build_output/BUILD_OK
