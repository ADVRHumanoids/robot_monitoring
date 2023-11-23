mkdir build_output

docker run --rm \
 -v $PWD:/home/user/robot_monitoring \
 -v $PWD/build_output:/home/user/build_output \
 -v $PWD/travis/docker/build.bash:/home/user/build.bash \
 arturolaurenzi/qt6dev:latest bash /home/user/build.bash

zip -r build_output/xbot2_gui_client.zip build_output/xbot2_gui_client