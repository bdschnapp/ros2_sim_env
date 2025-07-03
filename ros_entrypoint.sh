#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status.

# Source global ROS 2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Source overlay workspace if present
if [ -f "/ws/install/setup.bash" ]; then
    source /ws/install/setup.bash
fi

exec "$@"
