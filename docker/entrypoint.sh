#!/bin/bash
set -e

# Source ROS2 workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros_ws/install/setup.bash

# Check for calibration overrides
if [ -f "/ros_ws/config_overrides/robot_calibration.yaml" ]; then
    echo "✓ Using calibration override from volume mount"
fi

# Print environment info
echo "======================================"
echo "Yahboomcar ROS2 Container"
echo "======================================"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROBOT_TYPE: ${ROBOT_TYPE}"
echo "RPLIDAR_TYPE: ${RPLIDAR_TYPE}"
echo "======================================"

# Check device availability
echo "Checking hardware devices..."
[ -e /dev/ttyUSB0 ] && echo "✓ Motor controller (/dev/ttyUSB0) found" || echo "✗ Motor controller not found"
[ -e /dev/rplidar ] && echo "✓ Lidar (/dev/rplidar) found" || echo "✗ Lidar not found (check udev rules)"
[ -e /dev/video0 ] && echo "✓ Camera (/dev/video0) found" || echo "✗ Camera not found"
echo "======================================"

# Execute the command
exec "$@"
