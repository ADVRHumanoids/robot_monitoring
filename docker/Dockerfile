FROM arturolaurenzi/xbot2_focal_base 

USER root

RUN apt-get update && \
    apt-get install -y ros-noetic-theora-image-transport


USER user
WORKDIR /home/user

RUN mkdir -p forest_ws && cd forest_ws && forest init && mkdir -p ros_src
RUN echo "source /home/user/forest_ws/setup.bash" >> /home/user/.bashrc

RUN pip install aiohttp scipy numpy

RUN --mount=type=secret,id=mynetrc,dst=/home/user/.netrc,uid=1000 \
    git clone https://github.com/advrhumanoids/robot_monitoring --branch cards \
    && cd robot_monitoring

CMD cd robot_monitoring && bash -ic "python3 -m server.main /home/user/config.yaml"
