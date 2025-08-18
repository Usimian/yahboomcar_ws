# Robot Client Integration Guide

## Overview

This guide provides complete integration information for controlling the Yahboom X3 robot and reading its sensors via ROS2. The robot uses a **simplified command interface** - you send movement commands via ROS2 services and receive sensor data via ROS2 topics.

---

## 🚀 Quick Start

### Prerequisites
- **ROS2 Humble** installed on your client machine (AMD64 Ubuntu 22.04)
- **Network connection** to the robot (Jetson Orin Nano)
- **robot_msgs** package built on your client system

### Robot System Startup
The robot operator starts the system with:
```bash
ros2 launch slam_nav robot_slam_nav.launch.py
```

This launches:
- Robot hardware drivers
- SLAM system  
- Robot command interface
- All sensor publishers

---

## 📡 Communication Architecture

```
Client (Your Code) ←→ Network ←→ Robot (Jetson Orin Nano)
     ↓                                    ↓
  ROS2 Service Calls              Robot Interface Node
  ROS2 Topic Subscriptions       Hardware Drivers
```

### Key Principles
- **Robot is a hardware interface** - no AI processing on robot side
- **Client does all processing** - VILA, navigation planning, decision making
- **Simple command interface** - basic move/turn/stop commands
- **Pull-based sensor data** - client subscribes to sensor topics

---

## 🎯 Robot Control Interface

### Primary Command Service
**Service**: `/robot/execute_command`  
**Type**: `robot_msgs/srv/ExecuteCommand`  
**Purpose**: Send movement commands to the robot

### Service Definition
```yaml
# Request
robot_msgs/RobotCommand command
---
# Response  
bool success                       # Command started successfully?
string result_message              # Human-readable status/error message
```

### RobotCommand Message
```yaml
# robot_msgs/msg/RobotCommand
string robot_id              # Always "yahboomcar_x3_01"
string command_type           # "move", "turn", or "stop"
float64 linear_x              # Forward/backward direction (-1.0 to 1.0)
float64 linear_y              # Left/right direction (-1.0 to 1.0)  
float64 angular_z             # Not used (legacy field)
float64 duration              # Optional timeout (seconds, 0 = no limit)
float64 distance              # Target distance for move (meters)
float64 angular               # Target angle for turn (degrees)
float64 linear_speed          # Movement speed (m/s)
float64 angular_speed         # Rotation speed (rad/s)
int64 timestamp_ns            # Command timestamp (nanoseconds)
string source_node            # Your node name (for logging)
```

---

## 🎮 Command Types

### 1. Move Command
**Purpose**: Linear movement with precision encoder feedback

```python
command.command_type = "move"
command.linear_x = 1.0        # 1.0=forward, -1.0=backward, 0.0=no movement
command.linear_y = 0.0        # 1.0=right, -1.0=left, 0.0=no strafe  
command.distance = 0.5        # Distance in meters (e.g., 0.1 = 10cm)
command.linear_speed = 0.2    # Speed in m/s (recommended: 0.1-0.5)
command.duration = 10.0       # Optional timeout in seconds
```

**Examples**:
- Forward 20cm: `linear_x=1.0, distance=0.2, linear_speed=0.1`
- Backward 10cm: `linear_x=-1.0, distance=0.1, linear_speed=0.1`
- Diagonal: `linear_x=0.707, linear_y=0.707, distance=0.141` (10cm diagonal)

### 2. Turn Command  
**Purpose**: Rotational movement with precision encoder feedback

```python
command.command_type = "turn"
command.angular = 90.0        # Degrees: +90=left, -90=right
command.angular_speed = 0.5   # Speed in rad/s (recommended: 0.3-0.8)
command.duration = 8.0        # Optional timeout in seconds
```

**Examples**:
- Turn left 90°: `angular=90.0, angular_speed=0.5`
- Turn right 45°: `angular=-45.0, angular_speed=0.3`
- Full rotation: `angular=360.0, angular_speed=0.5`

### 3. Stop Command
**Purpose**: Immediate emergency stop

```python
command.command_type = "stop"
# All other parameters ignored
```

---

## 📊 Sensor Data Interface

### Primary Sensor Topic
**Topic**: `/robot/sensors`  
**Type**: `robot_msgs/msg/SensorData`  
**Frequency**: ~2Hz  

### SensorData Message
```yaml
# robot_msgs/msg/SensorData
string robot_id              # "yahboomcar_x3_01"
float64 battery_voltage       # Battery voltage (V)
float64 cpu_temp             # CPU temperature (°C)
float64 distance_front       # Front lidar distance (m)
float64 distance_left        # Left lidar distance (m) 
float64 distance_right       # Right lidar distance (m)
float64 cpu_usage            # CPU usage (0-100%)
string camera_status         # "active" or "unknown"
int64 timestamp_ns           # Timestamp (nanoseconds)
```

### Additional Sensor Topics
```bash
# Camera (Intel RealSense D435i)
/realsense/camera/color/image_raw           # RGB images (sensor_msgs/Image)
/realsense/camera/depth/image_rect_raw      # Depth images (sensor_msgs/Image)  
/realsense/camera/aligned_depth_to_color/image_raw  # Aligned depth (sensor_msgs/Image)

# Lidar (S2 LiDAR)
/scan                                       # Laser scan (sensor_msgs/LaserScan)

# IMU and Motion
/imu/data_raw                              # Raw IMU data (sensor_msgs/Imu)
/imu/mag                                   # Magnetometer (sensor_msgs/MagneticField)
/odom                                      # Filtered odometry (nav_msgs/Odometry)
/odom_raw                                  # Raw wheel odometry (nav_msgs/Odometry)

# System Status  
/voltage                                   # Battery voltage (std_msgs/Float32)
```

---

## 🔧 Client Implementation Example

### Python Client Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotCommand, SensorData
from robot_msgs.srv import ExecuteCommand
from sensor_msgs.msg import Image, LaserScan
import time

class RobotClient(Node):
    def __init__(self):
        super().__init__('robot_client')
        
        # Service client for robot commands
        self.command_client = self.create_client(ExecuteCommand, '/robot/execute_command')
        
        # Sensor data subscribers
        self.sensor_sub = self.create_subscription(
            SensorData, '/robot/sensors', self.sensor_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/realsense/camera/color/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
            
        # Wait for robot service
        self.get_logger().info('Waiting for robot service...')
        self.command_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Robot service connected!')
        
    def send_command(self, command_type, **kwargs):
        """Send a command to the robot"""
        request = ExecuteCommand.Request()
        request.command = RobotCommand()
        request.command.robot_id = "yahboomcar_x3_01"
        request.command.command_type = command_type
        request.command.source_node = "your_client_name"
        request.command.timestamp_ns = int(time.time() * 1e9)
        
        # Set command parameters
        for key, value in kwargs.items():
            if hasattr(request.command, key):
                setattr(request.command, key, value)
        
        # Send command
        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result():
            return future.result().success, future.result().result_message
        return False, "Service call timeout"
    
    def move_forward(self, distance_m, speed_ms=0.2):
        """Move robot forward by specified distance"""
        return self.send_command('move', 
                               linear_x=1.0, linear_y=0.0,
                               distance=distance_m, linear_speed=speed_ms)
    
    def turn_left(self, angle_deg, speed_rads=0.5):
        """Turn robot left by specified angle"""
        return self.send_command('turn',
                               angular=angle_deg, angular_speed=speed_rads)
    
    def emergency_stop(self):
        """Stop robot immediately"""
        return self.send_command('stop')
    
    def sensor_callback(self, msg):
        """Handle sensor data updates"""
        self.get_logger().info(f'Battery: {msg.battery_voltage:.1f}V, '
                              f'Front distance: {msg.distance_front:.2f}m')
    
    def camera_callback(self, msg):
        """Handle camera images - process with VILA here"""
        # Convert ROS image to OpenCV/numpy format
        # Process with VILA model
        # Generate movement commands based on analysis
        pass
    
    def lidar_callback(self, msg):
        """Handle lidar data for obstacle detection"""
        # Process laser scan for navigation
        pass

def main():
    rclpy.init()
    client = RobotClient()
    
    try:
        # Example usage
        client.move_forward(0.1)  # Move forward 10cm
        time.sleep(3)
        client.turn_left(90)      # Turn left 90 degrees
        time.sleep(4)
        client.emergency_stop()   # Stop
        
        # Keep running to receive sensor data
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        client.emergency_stop()
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔍 VILA Integration Pattern

### Typical VILA Control Loop

```python
def vila_control_loop(self):
    """Main VILA processing loop"""
    while rclpy.ok():
        # 1. Get camera image
        if self.latest_camera_image is not None:
            
            # 2. Process with VILA model
            vila_response = self.process_with_vila(self.latest_camera_image)
            
            # 3. Parse VILA response for movement commands
            should_move_forward = self.parse_movement_intent(vila_response)
            should_turn_left = self.parse_turn_intent(vila_response)
            hazard_detected = self.parse_hazard_detection(vila_response)
            
            # 4. Execute movement commands
            if hazard_detected:
                self.emergency_stop()
            elif should_move_forward:
                self.move_forward(0.1, speed_ms=0.15)  # 10cm forward
            elif should_turn_left:
                self.turn_left(30, speed_rads=0.4)     # 30° left
                
            # 5. Wait before next analysis
            time.sleep(1.0)  # 1 second between decisions
```

### VILA Response Parsing Example

```python
def parse_movement_intent(self, vila_text_response):
    """Parse VILA response for movement commands"""
    forward_keywords = ['forward', 'ahead', 'straight', 'advance', 'proceed']
    stop_keywords = ['stop', 'wait', 'obstacle', 'blocked', 'danger']
    
    text_lower = vila_text_response.lower()
    
    # Check for hazards first
    if any(keyword in text_lower for keyword in stop_keywords):
        return 'stop'
    
    # Check for movement intent  
    if any(keyword in text_lower for keyword in forward_keywords):
        return 'forward'
        
    return 'none'
```

---

## ⚙️ Robot System Details

### Hardware Specifications
- **Platform**: Yahboom X3 robot on Jetson Orin Nano
- **Lidar**: S2 LiDAR (360° scanning)
- **Camera**: Intel RealSense D435i (RGB + Depth)
- **IMU**: 9-axis (accelerometer, gyroscope, magnetometer)
- **Wheels**: 4-wheel mecanum drive (omnidirectional)
- **Encoders**: Wheel encoders for precise movement

### Coordinate System
- **X-axis**: Forward (positive) / Backward (negative)
- **Y-axis**: Right (positive) / Left (negative)  
- **Z-axis**: Up (positive) / Down (negative)
- **Yaw**: Left turn (positive) / Right turn (negative)

### Movement Capabilities
- **Linear speed**: 0.1 - 0.5 m/s recommended
- **Angular speed**: 0.3 - 0.8 rad/s recommended  
- **Precision**: ~1cm linear, ~5° angular
- **Omnidirectional**: Can move in any direction
- **Encoder feedback**: Automatic distance/angle control

### Sensor Specifications
- **Lidar range**: 0.15m - 16m, 360° coverage
- **Camera**: 1280x720 RGB, 640x480 depth at 30fps
- **IMU**: Raw accelerometer/gyroscope data (no orientation fusion)
- **Battery**: 12V LiPo, voltage monitoring available
- **Update rates**: Sensors ~10Hz, Commands processed immediately

---

## 🚨 Safety and Best Practices

### Command Safety
- **Always set timeouts**: Use `duration` parameter for safety
- **Check battery level**: Monitor `/robot/sensors` for low battery
- **Emergency stop**: Always implement emergency stop capability
- **Obstacle detection**: Use lidar data to avoid collisions
- **Speed limits**: Keep speeds reasonable for indoor use

### Network Considerations
- **Reliable connection**: Ensure stable WiFi/Ethernet to robot
- **Timeout handling**: Handle service call timeouts gracefully
- **QoS settings**: Use default QoS for most applications
- **Bandwidth**: Camera streams use significant bandwidth

### Error Handling
```python
def safe_robot_command(self, command_type, **kwargs):
    """Send command with proper error handling"""
    try:
        # Check battery level first
        if self.last_battery_voltage < 10.5:  # Low battery
            self.get_logger().warn('Battery low, stopping robot')
            return self.emergency_stop()
        
        # Send command with timeout
        success, message = self.send_command(command_type, **kwargs)
        
        if not success:
            self.get_logger().error(f'Command failed: {message}')
            self.emergency_stop()  # Stop on failure
            
        return success
        
    except Exception as e:
        self.get_logger().error(f'Command error: {e}')
        self.emergency_stop()
        return False
```

---

## 📋 Testing and Debugging

### Test Commands
```bash
# Test robot service availability
ros2 service list | grep execute_command

# Test sensor data
ros2 topic echo /robot/sensors --once

# Test camera feed  
ros2 topic hz /realsense/camera/color/image_raw

# Manual robot command
ros2 service call /robot/execute_command robot_msgs/srv/ExecuteCommand \
  "{command: {robot_id: 'yahboomcar_x3_01', command_type: 'move', 
  linear_x: 1.0, distance: 0.1, linear_speed: 0.1, source_node: 'test'}}"
```

### Common Issues
- **Service not available**: Robot system not running
- **Command timeout**: Network issues or robot busy
- **No sensor data**: Check topic subscriptions and QoS
- **Movement inaccurate**: Check encoder calibration
- **Camera not working**: Check RealSense drivers

---

## 📞 Support Information

### Robot Operator Commands
```bash
# Start robot system
ros2 launch slam_nav robot_slam_nav.launch.py

# Check system status
ros2 node list
ros2 topic list

# Emergency stop all nodes
pkill -f ros2
```

### Troubleshooting
1. **Robot not responding**: Check network connection and service availability
2. **Jerky movement**: Reduce speed parameters  
3. **Sensor data missing**: Verify topic names and message types
4. **High latency**: Check network bandwidth and reduce camera resolution

---

## 📝 Message Dependencies

### Required ROS2 Packages
```bash
# On your client system
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-nav-msgs  
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-std-msgs

# Build robot_msgs package
git clone <robot_msgs_repo>
colcon build --packages-select robot_msgs
source install/setup.bash
```

---

This guide provides everything needed to integrate with the robot system. The robot acts as a simple hardware interface - you send movement commands and receive sensor data. All AI processing, path planning, and decision making happens on your client system.
