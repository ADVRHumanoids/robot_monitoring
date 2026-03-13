echo user | sudo -S chown user /home/user/build_output

# test 
set -e
touch /home/user/build_output/BUILD_IN_PROGRESS

# unless BUILD_SKIP_ANDROID is set, build android apk
if [ -z "$BUILD_SKIP_ANDROID" ]; then
    echo "Building Android APK..."
    mkdir build && cd build
    /home/user/Qt/$QT_VER/android_arm64_v8a/bin/qt-cmake -DCMAKE_BUILD_TYPE=Release ../robot_monitoring
    make -j $(nproc)
    cp xbot2_gui/android-build/build/outputs/apk/release/android-build-release-unsigned.apk /home/user/build_output/xbot2_gui_client_android_arm64_v8a.apk
    cd ..
    rm -rf build
else
    echo "Skipping Android App build..."
fi


# unless BUILD_SKIP_LINUX is set, build linux app
if [ -z "$BUILD_SKIP_LINUX" ]; then
    echo "Building Linux App..."
    export PATH="$HOME/Qt/Tools/CMake/bin:$PATH"
    mkdir build && cd build
    /home/user/Qt/$QT_VER/gcc_64/bin/qt-cmake -DCMAKE_INSTALL_PREFIX=/home/user/build_output/xbot2_gui_client_x86_64 -DCMAKE_BUILD_TYPE=Release ../robot_monitoring
    make install -j $(nproc)
else
    echo "Skipping Linux App build..."
fi

touch /home/user/build_output/BUILD_OK
