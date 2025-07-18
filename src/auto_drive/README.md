# Autonomous Drive Package

This package provides autonomous navigation capabilities for the Yahboomcar X3 robot with S2 lidar and Astra Pro Plus camera.

## Problem Fixed: Robot Only Driving Forward

### What was the issue?
The robot was only driving forward instead of performing intelligent autonomous navigation due to several issues:

1. **Autonomous mode not activated**: The robot started with `is_active = False` and required explicit activation
2. **Simple navigation logic**: The original code would just drive forward when no obstacles were detected
3. **No exploration behavior**: The robot had no mechanism to explore and map its environment

### What was fixed?

#### 1. Automatic Autonomous Mode Activation
- Added automatic initialization of autonomous mode based on launch parameter
- Robot now starts in autonomous mode by default when launched
- Added proper state management and logging

#### 2. Intelligent Navigation Logic
- **Obstacle Avoidance**: Robot now properly turns when obstacles are detected
- **Exploration Behavior**: Added random exploration when in open spaces
- **Multi-sector Analysis**: Robot analyzes front, left, right, front-left, and front-right sectors
- **Adaptive Speed Control**: Speed adjusts based on available space
- **Directional Preferences**: Smart turning decisions based on clearer paths

#### 3. Enhanced Logging and Debugging
- Added comprehensive logging for navigation decisions
- Created debug monitor tool for real-time status monitoring
- Clear status messages for different navigation states

## Usage

### Basic Launch (with RViz)
```bash
ros2 launch auto_drive auto_drive.launch.py
```

### Headless Launch (no graphics)
```bash
ros2 launch auto_drive auto_drive_headless.launch.py
```

### Manual Control
```bash
# Enable auto mode
ros2 run auto_drive auto_control enable

# Disable auto mode (manual control)
ros2 run auto_drive auto_control disable

# Check current status
ros2 topic echo /JoyState
```

### Debug Monitoring
```bash
# Run debug monitor to see real-time status
ros2 run auto_drive debug_monitor
```

This will show:
- Current mode (AUTONOMOUS/MANUAL)
- Battery voltage and status (HIGH/GOOD/LOW/CRITICAL)
- Detailed velocity commands:
  - Forward/Backward movement (linear.x)
  - Sideways movement (linear.y) - for mecanum wheels
  - Rotation (angular.z)
  - Total speed and movement direction
- Robot position (X, Y coordinates)
- Obstacle distances in all directions (front, left, right, back)

## Navigation Behavior

### Current Implementation
The robot now exhibits intelligent autonomous behavior:

1. **Obstacle Detection**: Monitors front, left, and right sectors
2. **Safe Navigation**: Maintains configurable safety distances
3. **Exploration**: Randomly explores when in open areas
4. **Adaptive Turning**: Chooses optimal turn direction based on available space
5. **Emergency Stop**: Immediate stop when obstacles are too close

### Navigation States

#### State 1: Obstacle Ahead
- **Behavior**: Stop and turn towards clearer side
- **Logic**: Compare left vs right distances, turn towards better option
- **Speed**: 0 m/s linear, up to 0.5 rad/s angular

#### State 2: Lots of Space Ahead
- **Behavior**: Move forward with occasional exploration
- **Logic**: 10% chance to add slight turning for exploration
- **Speed**: Up to 0.3 m/s linear, occasional 0.2 rad/s angular

#### State 3: Moderate Space
- **Behavior**: Cautious forward movement with steering
- **Logic**: Reduce speed, steer towards more open space
- **Speed**: Up to 0.18 m/s linear, slight angular adjustments

## Configuration

### Parameters (config/auto_drive.yaml)
```yaml
auto_navigator:
  ros__parameters:
    max_speed: 0.3                    # Maximum linear speed (m/s)
    max_angular_speed: 0.5           # Maximum angular speed (rad/s)
    safety_distance: 0.8             # Minimum distance to obstacles (m)
    emergency_distance: 0.4          # Emergency stop distance (m)
    enable_autonomous: true          # Enable autonomous driving
```

### Launch Parameters
```bash
# Custom speed and safety distance
ros2 launch auto_drive auto_drive.launch.py max_speed:=0.2 safety_distance:=1.0

# Disable auto mode on startup
ros2 launch auto_drive auto_drive.launch.py enable_auto:=false
```

## Troubleshooting

### Robot Still Not Moving?
1. **Check autonomous mode status**:
   ```bash
   ros2 topic echo /JoyState
   ```
   - `data: false` = Auto mode
   - `data: true` = Manual mode

2. **Enable auto mode**:
   ```bash
   ros2 run auto_drive auto_control enable
   ```

3. **Check sensor data**:
   ```bash
   ros2 topic echo /scan
   ros2 topic echo /odom
   ros2 topic echo /imu/data
   ```

4. **Monitor velocity commands**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### Robot Behaving Erratically?
1. **Adjust safety parameters** in `config/auto_drive.yaml`
2. **Check lidar data quality** - ensure lidar is working properly
3. **Verify IMU calibration** - robot needs proper orientation data

### No Sensor Data?
1. **Ensure robot hardware is launched**:
   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py
   ```

2. **Check topic availability**:
   ```bash
   ros2 topic list
   ```

### Battery Issues?
1. **Check battery voltage**:
   ```bash
   ros2 topic echo /voltage
   ```

2. **Battery voltage levels**:
   - **HIGH**: > 12.0V - Fully charged
   - **GOOD**: 11.0V - 12.0V - Normal operation
   - **LOW**: 10.0V - 11.0V - Should recharge soon
   - **CRITICAL**: < 10.0V - Recharge immediately

3. **Low battery behavior**:
   - Robot may move slower or behave erratically
   - Autonomous navigation may become unreliable
   - Sensors may provide inconsistent data

## Architecture

### Nodes
- **auto_navigator**: Main navigation logic
- **auto_control**: Enable/disable auto mode
- **debug_monitor**: Real-time status monitoring

### Key Topics
- `/cmd_vel`: Velocity commands to robot
- `/scan`: Lidar data
- `/odom`: Odometry data
- `/imu/data`: IMU orientation data
- `/JoyState`: Autonomous/manual mode control
- `/auto_drive/map`: Generated occupancy grid
- `/auto_drive/pose`: Robot pose estimation

### Features
- **Sensor Fusion**: Combines lidar, IMU, and odometry
- **Real-time Mapping**: Creates occupancy grid of environment
- **Reactive Navigation**: Immediate response to obstacles
- **Safety Systems**: Emergency stop and collision avoidance
- **Manual Override**: Joystick control always takes precedence

## Dependencies

- rclpy
- geometry_msgs
- sensor_msgs
- nav_msgs
- std_msgs
- tf2_ros
- yahboomcar_msgs (from yahboom_X3_lib)

## Future Enhancements

- Path planning algorithms (A*, RRT)
- SLAM integration
- Goal-directed navigation
- Dynamic obstacle avoidance
- Machine learning-based navigation 