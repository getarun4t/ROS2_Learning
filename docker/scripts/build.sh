#!/bin/bash
set -e

echo "Building base image..."
docker build -f docker/dockerfiles/base.24.04.Dockerfile -t robotarm/base:24.04 .

echo "Building ROS2 Jazzy image..."
docker build -f docker/dockerfiles/ros2.Dockerfile -t robotarm/ros2:jazzy .