# Start with the official ROS 2 Humble image (Ubuntu 22.04)
FROM ros:humble-ros-base
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
    ros-humble-ament-cmake-auto \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-cmake \
    ros-humble-geographic-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-tools \
    python3-transforms3d \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install extra Python packages
RUN pip install --no-cache-dir numpy
RUN pip install --no-cache-dir git+https://github.com/DLu/tf_transformations.git

# Copy local workspace into the container, mark required scripts as executable
COPY ws/src /ws/src
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN ls -l /ros_entrypoint.sh

# Build the workspace
WORKDIR /ws
RUN rm -rf build install logs && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --merge-install"

# Source the workspace by default (added to .bashrc)
RUN echo 'source /ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.sh' >> ~/.bashrc

# Set environment variables
ENV RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
ENV ROS_DISTRO="humble"
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY="0"
ENV ROS_PYTHON_VERSION="3"
ENV ROS_VERSION="2"
ENV FASTDDS_BUILTIN_TRANSPORTS="UDPv4"

# Run the node
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "simulation", "matlab_sim_env.launch.py"]