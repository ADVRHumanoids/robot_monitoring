set -e

ENV_ARG=""

if [ -n "$ROS_MASTER_URI" ]; then
    ENV_ARG="$ENV_ARG --env ROS_MASTER_URI=$ROS_MASTER_URI";
fi

if [ -n "$ROS_IP" ]; then
    ENV_ARG="$ENV_ARG --env ROS_IP=$ROS_IP";
fi

CONFIG=$(realpath $1)
EXEC=$2

docker run -it --rm --name next_ui \
--network host \
$ENV_ARG \
--volume $CONFIG:/home/user/config.yaml:ro \
--volume $HOME/.ssh:/home/user/.ssh:ro \
--volume $(rospack find centauro_urdf):/home/user/forest_ws/ros_src/centauro_urdf:ro \
next_ui $EXEC
