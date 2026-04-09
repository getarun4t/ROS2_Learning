FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Basic tools
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    python3-pip \
    git \
    && locale-gen en_US en_US.UTF-8

# ROS2 Jazzy repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Jazzy + Gazebo
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    gazebo \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils

# Setup ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws

SHELL ["/bin/bash", "-c"]