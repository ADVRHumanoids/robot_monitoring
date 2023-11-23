mkdir build_output

rm -rf travis/docker/context && mkdir -p travis/docker/context/robot_monitoring

cp -r * travis/docker/context/robot_monitoring

docker build travis/docker -t travis_build_image

docker run --rm \
 -v $PWD/build_output:/home/user/build_output \
 travis_build_image bash /home/user/build.bash

chmod -R user build_output

zip -r build_output/xbot2_gui_client.zip build_output/xbot2_gui_client
