#!/bin/bash
set -e
export ROS_DISTRO=jazzy
export WS=/home/ros2-workspace

# Source the setup.bash file
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS}/install/setup.bash

exec "$@"