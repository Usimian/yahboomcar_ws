#!/bin/bash
# Convenience script to run autonomous drive with debug monitoring
# Usage: ./run_auto_with_debug.sh [options]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
USE_RVIZ="true"
ENABLE_AUTONOMOUS="true"
MAX_SPEED="0.3"
SAFETY_DISTANCE="0.8"
DEBUG_UPDATE_RATE="2.0"
ENABLE_DEBUG="true"

# Function to display help
show_help() {
    echo -e "${BLUE}Autonomous Drive with Debug Monitor${NC}"
    echo -e "${BLUE}====================================${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  --no-rviz              Disable RViz visualization"
    echo "  --no-autonomous        Start in manual mode"
    echo "  --no-debug             Disable debug monitor"
    echo "  --speed SPEED          Set max speed (default: 0.3 m/s)"
    echo "  --safety DISTANCE      Set safety distance (default: 0.8 m)"
    echo "  --debug-rate RATE      Set debug update rate (default: 2.0 s)"
    echo "  --fast-debug           Set debug update rate to 0.5s"
    echo "  --slow-debug           Set debug update rate to 5.0s"
    echo ""
    echo "Examples:"
    echo "  $0                     # Start with default settings"
    echo "  $0 --no-rviz          # Start without RViz"
    echo "  $0 --speed 0.2         # Start with reduced speed"
    echo "  $0 --fast-debug        # Start with fast debug updates"
    echo "  $0 --no-autonomous     # Start in manual mode"
}

# Function to check if ROS2 is sourced
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}Error: ROS2 not found. Please source your ROS2 setup.bash${NC}"
        echo "Try: source /opt/ros/humble/setup.bash"
        echo "And: source ~/yahboomcar_ros2/yahboomcar_ws/install/setup.bash"
        exit 1
    fi
}

# Function to check if workspace is built
check_workspace() {
    if [ ! -f "install/setup.bash" ]; then
        echo -e "${RED}Error: Workspace not built. Please build the workspace first.${NC}"
        echo "Try: colcon build --packages-select auto_drive"
        exit 1
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --no-autonomous)
            ENABLE_AUTONOMOUS="false"
            shift
            ;;
        --no-debug)
            ENABLE_DEBUG="false"
            shift
            ;;
        --speed)
            MAX_SPEED="$2"
            shift 2
            ;;
        --safety)
            SAFETY_DISTANCE="$2"
            shift 2
            ;;
        --debug-rate)
            DEBUG_UPDATE_RATE="$2"
            shift 2
            ;;
        --fast-debug)
            DEBUG_UPDATE_RATE="0.5"
            shift
            ;;
        --slow-debug)
            DEBUG_UPDATE_RATE="5.0"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Main execution
echo -e "${BLUE}Starting Autonomous Drive with Debug Monitor${NC}"
echo -e "${BLUE}=============================================${NC}"

# Check prerequisites
check_ros2
check_workspace

# Display configuration
echo -e "${YELLOW}Configuration:${NC}"
echo "  RViz: $USE_RVIZ"
echo "  Autonomous Mode: $ENABLE_AUTONOMOUS"
echo "  Debug Monitor: $ENABLE_DEBUG"
echo "  Max Speed: $MAX_SPEED m/s"
echo "  Safety Distance: $SAFETY_DISTANCE m"
echo "  Debug Update Rate: $DEBUG_UPDATE_RATE s"
echo ""

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

# Launch the system
echo -e "${GREEN}Launching autonomous drive with debug monitor...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo ""

ros2 launch auto_drive auto_drive_with_debug.launch.py \
    use_rviz:=$USE_RVIZ \
    enable_autonomous:=$ENABLE_AUTONOMOUS \
    max_speed:=$MAX_SPEED \
    safety_distance:=$SAFETY_DISTANCE \
    debug_update_rate:=$DEBUG_UPDATE_RATE \
    enable_debug:=$ENABLE_DEBUG

echo -e "${GREEN}Autonomous drive with debug monitor stopped${NC}" 