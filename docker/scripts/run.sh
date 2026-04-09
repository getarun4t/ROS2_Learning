#!/bin/bash

CONTAINER_NAME="ros2_jazzy"

WS_PATH="$(pwd)/ros2_ws"

docker run -it --rm \
    --name $CONTAINER_NAME \
    --gpus all \
    --network host \
    --ipc host \
    --device /dev/dxg \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e QT_QPA_PLATFORM=xcb \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
    -v /mnt/wslg:/mnt/wslg \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$WS_PATH:/ros2_ws" \
    robotarm/ros2:jazzy