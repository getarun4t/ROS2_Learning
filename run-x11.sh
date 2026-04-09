#!/bin/bash

CONTAINER_NAME="ros2_jazzy"
WS_PATH="$(pwd)/ros2_ws"

xhost +local:docker

docker run -it --rm \
    --name $CONTAINER_NAME \
    --gpus all \
    --network host \
    --ipc host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$WS_PATH:/ros2_ws" \
    robotarm/ros2:jazzy