set -e 

# delete old artifacts
rm -rf build_output && mkdir build_output
rm -rf travis/docker/context && mkdir -p travis/docker/context/robot_monitoring

# copy whole repo into docker context
cp -r * travis/docker/context/robot_monitoring || true

# build docker image
docker build travis/docker -t travis_build_image --pull

# build client via docker
docker run --rm \
 -v $PWD/build_output:/home/user/build_output \
 travis_build_image bash /home/user/build.bash

# restore correct ownership
sudo chown -R $USER build_output

# compress linux app
cd build_output
zip -r xbot2_gui_client_x86_64.zip xbot2_gui_client_x86_64

# sign apk
zipalign -p 4 xbot2_gui_client_android_arm64_v8a.apk xbot2_gui_client_android_arm64_v8a_signed.apk
echo $KEYSTORE_PWD | apksigner sign --ks-key-alias app --ks ../my.keystore xbot2_gui_client_android_arm64_v8a_signed.apk
