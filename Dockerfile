# Start with the official ROS 2 Humble image (Ubuntu 22.04)
FROM ros:humble-ros-core
LABEL authors="bdschnap"

# Install development tools and colcon
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    nano \
    build-essential \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-cmake \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install extra Python packages
RUN pip install --no-cache-dir numpy

# Setup workspace
ENV ROS2_WS=/ros2_ws
WORKDIR $ROS2_WS

# Copy local workspace into the container
COPY ws/src /ws/src
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN chmod +x /ws/build.sh
RUN chmod +x /ws/clean.sh

# Build the workspace
WORKDIR /ws
RUN ./clean.sh
RUN . /opt/ros/humble/setup.sh && ./build.sh

# Source the workspace by default
RUN echo 'source /opt/ros/humble/setup.sh' >> ~/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

# Source ROS and workspace, then run the node
WORKDIR /ws

ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
ENV ROS_DISTRO="humble"
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY="0"
ENV ROS_PYTHON_VERSION="3"
ENV ROS_VERSION="2"
ENV FASTDDS_BUILTIN_TRANSPORTS="UDPv4"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "simulation", "trajectory_planner_node"]