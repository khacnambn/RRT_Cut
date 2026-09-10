#!/bin/bash

cd "$(dirname "$0")/.."

xhost +local:docker

docker run -it --rm \
    --name ros_motion_planning_container \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/root/ros_motion_planning_ws \
    ros_motion_planning:noetic
