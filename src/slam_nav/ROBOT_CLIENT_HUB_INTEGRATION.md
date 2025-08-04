# Robot Client Hub Integration

This document explains how to integrate the Yahboomcar X3 robot with a PC client hub that runs VILA for AI-powered navigation.

## Architecture Overview

```
┌─────────────────────┐    HTTP/JSON    ┌──────────────────────┐
│   Jetson (Robot)    │ ──────────────► │   PC (Client Hub)    │
│                     │                │                      │
│ • Camera Data       │                │ • VILA AI Model      │
│ • Lidar Data        │                │ • Image Analysis     │
│ • IMU Data          │                │ • Command Generation │
│ • Motor Control     │ ◄────────────── │                      │
│                     │   Commands     │                      │
└─────────────────────┘                └──────────────────────┘
```

**Important**: 
- **Robot side (Jetson)**: Collects data and executes commands - NO VILA/AI
- **Client side (PC)**: Runs VILA model for AI processing

## What's Included

### Robot Side (Jetson) Components

1. **Robot Data Server** (`robot_jetson_server`)
   - Collects camera images, lidar scans, IMU data
   - Sends data to PC client hub via HTTP
   - Receives navigation commands from client hub
   - Executes movement commands on robot

2. **Launch Files**
   - `slam_nav_cam_client_hub.launch.py` - Full robot system with client hub integration
   - `robot_data_server.launch.py` - Just the data server (for existing systems)

3. **Configuration**
   - `robot_jetson_server.yaml` - Server configuration parameters

## Usage Instructions

### 1. On the PC (Client Hub Side)

First, make sure the VILA client hub is running on your PC:
```bash
cd /home/mw/Robot_LLM
python3 robot_vila_server.py
```

This starts the server on port 5000 that processes images with VILA.

### 2. On the Robot (Jetson Side)

#### Option A: Full Robot System with Client Hub
Launch the complete robot system including client hub integration:
```bash
ros2 launch slam_nav slam_nav_cam_client_hub.launch.py \
  client_hub_url:=http://192.168.1.153:5000 \
  robot_id:=yahboomcar_x3_01
```

#### Option B: Add Client Hub to Existing System
If you already have robot systems running, just add the data server:
```bash
ros2 launch slam_nav robot_data_server.launch.py \
  client_hub_url:=http://192.168.1.153:5000 \
  robot_id:=yahboomcar_x3_01
```

### Configuration Parameters

Edit `src/slam_nav/config/robot_jetson_server.yaml`:

```yaml
robot_jetson_server:
  ros__parameters:
    client_hub_url: "http://192.168.1.153:5000"  # Your PC IP address
    robot_id: "yahboomcar_x3_01"                 # Unique robot identifier
    send_frequency: 2.0                          # Data send rate (Hz)
    image_quality: 85                            # JPEG quality (1-100)
    linear_velocity: 0.2                         # Forward speed (m/s)
    angular_velocity: 0.3                        # Turn speed (rad/s)
```

## ROS2 Topics

### Subscribed (Input to Robot Server)
- `/camera/color/image_raw` - RGB camera images
- `/scan` - Lidar scan data
- `/imu/data_raw` - IMU orientation/motion data

### Published (Output from Robot Server)
- `/cmd_vel` - Velocity commands for robot movement
- `/robot/vila_status` - Status updates from client hub
- `/robot/vila_analysis` - VILA analysis results

## How It Works

1. **Data Collection**: Robot server collects sensor data from ROS topics
2. **Data Transmission**: Sends image + sensor data to PC client hub via HTTP POST
3. **AI Processing**: PC client hub processes image with VILA model
4. **Command Generation**: VILA analysis generates navigation commands
5. **Command Execution**: Robot server receives commands and publishes to `/cmd_vel`

## Network Setup

### Find Your PC IP Address
On your PC, find the IP address:
```bash
ip addr show | grep "inet 192"
```

Update the configuration with your PC's IP address.

### Test Connection
Test if robot can reach the client hub:
```bash
# On robot (Jetson)
curl http://192.168.1.153:5000/health
```

## Safety Features

- **Emergency Stop**: Automatic stop if connection to PC is lost
- **Hazard Detection**: Stops robot if VILA detects hazards
- **Timeout Protection**: Safety stop after 10 seconds without response

## Troubleshooting

### Connection Issues
1. Check PC IP address in configuration
2. Ensure client hub is running on PC (`python3 robot_vila_server.py`)
3. Check firewall settings on both machines
4. Test network connectivity between robot and PC

### Robot Not Moving
1. Check if `/cmd_vel` topic has subscribers: `ros2 topic info /cmd_vel`
2. Verify robot base node is running
3. Check for emergency stop conditions in logs

### No Camera Data
1. Verify camera is connected and working
2. Check camera topics: `ros2 topic list | grep camera`
3. Test camera: `ros2 topic echo /camera/color/image_raw --max-count 1`

## Performance Tuning

### Reduce Network Load
- Lower `send_frequency` (e.g., 1.0 Hz)
- Reduce `image_quality` (e.g., 70)
- Use smaller image resolution in camera config

### Improve Responsiveness
- Increase `send_frequency` (up to 3-4 Hz)
- Ensure good WiFi connection
- Use wired connection if possible

## Building and Installing

To build the updated slam_nav package:
```bash
cd ~/yahboomcar_ros2/yahboomcar_ws
colcon build --packages-select slam_nav
source install/setup.bash
```

## Integration Notes

This integration is designed to work alongside existing robot functionality:
- Does not interfere with manual control or existing navigation
- Can be started/stopped independently
- Uses standard ROS2 topics for compatibility
- Publishes to `/cmd_vel` like other navigation systems