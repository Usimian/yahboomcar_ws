#!/bin/bash
# Clean build script that eliminates environment variable warnings
# Use this in VSCode terminal: ./build_clean.sh

echo "🧹 Cleaning old environment variables..."

# Clear all ROS-related environment variables
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH

echo "📦 Loading fresh ROS 2 Humble environment..."

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Source library workspace
if [ -f "$HOME/yahboomcar_ros2/library_ws/install/setup.bash" ]; then
    source "$HOME/yahboomcar_ros2/library_ws/install/setup.bash"
fi

# Source current workspace if it exists (for incremental builds)
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
fi

echo "🔨 Building workspace..."
colcon build

echo ""
echo "✅ Build complete! To use the workspace, run:"
echo "   source install/setup.bash"
