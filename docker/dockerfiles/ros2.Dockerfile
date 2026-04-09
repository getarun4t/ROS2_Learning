FROM robotarm/base:24.04

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Install colcon tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions

# Source ROS in bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]