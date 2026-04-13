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

# Add ROS2 GPG key (modern method)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

# Add ROS2 repo
RUN echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
> /etc/apt/sources.list.d/ros2.list

# Install ROS2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils

# Setup ROS
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

SHELL ["/bin/bash", "-c"]

CMD ["/bin/bash"]