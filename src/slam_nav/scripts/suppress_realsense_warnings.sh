#!/bin/bash

# Script to suppress RealSense warnings
# Usage: source suppress_realsense_warnings.sh

# LibRealSense logging levels: 
# 0 = DEBUG, 1 = INFO, 2 = WARN, 3 = ERROR, 4 = FATAL, 5 = NONE
export RS2_LOG_LEVEL=4  # ERROR level only

# ROS2 logging levels:
# DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=40  # ERROR level only

# Disable colorized output for cleaner logs
export RCUTILS_COLORIZED_OUTPUT=0

# LibUSB debug level (reduce USB warnings)
export LIBUSB_DEBUG=0

# Suppress kernel USB debugging if possible (requires sudo)
if [ "$EUID" -eq 0 ]; then
    # Running as root
    echo 0 > /sys/module/usbcore/parameters/usbfs_debug 2>/dev/null || true
    echo 1 > /sys/module/uvcvideo/parameters/uvc_no_drop_param 2>/dev/null || true
else
    # Not root - try with sudo
    sudo sh -c 'echo 0 > /sys/module/usbcore/parameters/usbfs_debug' 2>/dev/null || true
    sudo sh -c 'echo 1 > /sys/module/uvcvideo/parameters/uvc_no_drop_param' 2>/dev/null || true
fi

echo "RealSense warning suppression enabled:"
echo "  RS2_LOG_LEVEL=$RS2_LOG_LEVEL"
echo "  RCUTILS_LOGGING_SEVERITY_THRESHOLD=$RCUTILS_LOGGING_SEVERITY_THRESHOLD"
echo ""
echo "To use: source suppress_realsense_warnings.sh && ros2 launch ..."