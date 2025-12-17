#!/bin/bash
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros_ws/install/setup.bash

exec "$@"
