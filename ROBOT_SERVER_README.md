# Robot Jetson Server

This is the robot-side server that runs on the **Jetson Orin Nano** (robot side) and communicates with the **Client Hub** (PC side with VILA).

## Architecture Overview

```
┌─────────────────────┐    HTTP/JSON    ┌──────────────────────┐
│   Jetson (Robot)    │ ──────────────► │   PC (Client Hub)    │
│                     │                │                      │
│ • Camera Data       │                │ • VILA Processing    │
│ • Lidar Data        │                │ • Command Generation │
│ • IMU Data          │                │ • Analysis Results   │
│ • Motor Control     │ ◄────────────── │                      │
│                     │   Commands     │                      │
└─────────────────────┘                └──────────────────────┘
```

## What This Server Does

### Data Collection (Jetson → PC)
- **Camera Images**: RGB images from Intel Realsense D435i
- **Lidar Data**: 360° scan data from SLAMTEC S2
- **IMU Data**: Orientation and motion data
- **Robot Status**: Position, battery, health

### Command Execution (PC → Jetson)
- **Movement Commands**: Forward, turn left/right, stop
- **Safety Commands**: Emergency stop, hazard response
- **Status Updates**: Robot state changes

## Setup Instructions

### 1. On the Jetson (Robot Side)

#### Prerequisites
- ROS2 Humble installed and sourced
- Robot sensors running (camera, lidar, IMU)
- Network connection to PC

#### Install Dependencies
```bash
# Install Python packages
pip3 install requests pillow opencv-python

# Make sure robot server is executable
chmod +x robot_jetson_server.py
```

#### Configure Network
Edit `robot_server_config.yaml`:
```yaml
client_hub_url: "http://192.168.1.XXX:5000"  # Replace with PC IP
robot_id: "yahboomcar_x3_01"  # Unique robot name
```

#### Launch Robot Server
```bash
# Method 1: Direct execution
cd /path/to/robot/server
python3 robot_jetson_server.py

# Method 2: Using ROS2 launch (if integrated into workspace)
ros2 launch robot_integration launch_robot_server.py \
  client_hub_url:=http://192.168.1.100:5000 \
  robot_id:=yahboomcar_x3_01
```

### 2. On the PC (Client Hub Side)

The client hub should already be running with VILA. Make sure:
```bash
cd /home/mw/Robot_LLM
python3 robot_vila_server.py
```

## Communication Protocol

### Robot Registration
The robot automatically registers with the hub on startup:
```json
{
  "robot_id": "yahboomcar_x3_01",
  "name": "Yahboomcar X3 - yahboomcar_x3_01",
  "capabilities": ["navigation", "camera", "lidar", "mecanum_drive"],
  "position": {"x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0},
  "battery_level": 100.0,
  "connection_type": "http"
}
```

### Data Transmission
Every 0.5 seconds (2 Hz by default), robot sends:
```json
{
  "image": "base64_encoded_jpeg_image",
  "prompt": "Analyze this robot camera view for autonomous navigation...",
  "sensor_data": {
    "lidar": {...},
    "imu": {...},
    "position": {...},
    "battery": 100.0
  },
  "generate_commands": true
}
```

### Command Reception
Robot receives and executes:
```json
{
  "success": true,
  "analysis": "I can see a clear path ahead...",
  "commands": {
    "move_forward": true,
    "stop": false,
    "turn_left": false,
    "turn_right": false,
    "hazard_detected": false
  }
}
```

## Configuration Options

### robot_server_config.yaml
```yaml
robot_jetson_server:
  ros__parameters:
    client_hub_url: "http://192.168.1.100:5000"  # PC IP address
    robot_id: "yahboomcar_x3_01"                 # Unique robot ID
    send_frequency: 2.0                          # Data transmission rate (Hz)
    image_quality: 85                            # JPEG compression quality
    linear_velocity: 0.2                         # Forward speed (m/s)
    angular_velocity: 0.3                        # Turn speed (rad/s)
```

## ROS2 Topics

### Subscribed Topics (Input)
- `/camera/color/image_raw` - Camera images
- `/scan` - Lidar data  
- `/imu/data_raw` - IMU data

### Published Topics (Output)
- `/cmd_vel` - Velocity commands for robot movement
- `/robot/status` - Robot status and analysis results

## Safety Features

### Emergency Stop
- Automatic stop if connection to hub is lost
- Safety override for hazard detection
- Manual stop command support

### Error Handling
- Network timeout handling  
- Sensor data validation
- Graceful degradation on errors

## Troubleshooting

### Connection Issues
```bash
# Test connection to hub
curl http://192.168.1.100:5000/health

# Check robot topics are publishing
ros2 topic list
ros2 topic echo /camera/color/image_raw --max-count 1
```

### Common Problems

1. **No camera data**: Check if camera node is running
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

2. **No lidar data**: Check lidar connection and permissions
   ```bash
   ls -la /dev/ttyUSB*
   ros2 launch sllidar_ros2 sllidar_s2_launch.py
   ```

3. **Network errors**: Verify PC IP address and firewall settings

4. **Robot not moving**: Check if `/cmd_vel` topic has subscribers
   ```bash
   ros2 topic info /cmd_vel
   ```

## Performance Tuning

### Reduce Bandwidth
- Lower `send_frequency` (default: 2.0 Hz)
- Reduce `image_quality` (default: 85)
- Resize images before sending

### Improve Responsiveness  
- Increase `send_frequency` (but watch network load)
- Use compressed image topics
- Implement local obstacle avoidance as backup

## Integration with Existing Robot

This server is designed to work alongside existing robot functionality:
- Does not interfere with manual control
- Can be started/stopped independently  
- Publishes to standard `/cmd_vel` topic
- Compatible with existing navigation stack

## Monitoring

### Log Output
The server provides detailed logging:
```
🤖 Robot Server starting - ID: yahboomcar_x3_01
📡 Client Hub URL: http://192.168.1.100:5000
📡 ROS2 subscribers initialized
📢 ROS2 publishers initialized  
✅ Successfully registered with client hub
📤 Data sender started at 2.0 Hz
🧠 VILA Analysis: I can see a clear hallway ahead...
⬆️ Moving forward
```

### Status Monitoring
Monitor robot status via ROS2 topics:
```bash
ros2 topic echo /robot/status
```

## Next Steps

1. **Test the communication** between robot and client hub
2. **Tune parameters** for your specific environment  
3. **Add custom behaviors** based on VILA analysis
4. **Integrate with existing navigation** if needed
5. **Scale to multiple robots** using different robot IDs