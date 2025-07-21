# SLAM Navigation Package

This package provides comprehensive SLAM (Simultaneous Localization and Mapping) and navigation capabilities for the Yahboomcar X3 robot with S2 lidar sensor. It integrates slam_toolbox for persistent mapping with the ROS2 Nav2 navigation stack.

## Features

- **Complete Robot Bringup**: Hardware drivers, sensors, and robot description
- **SLAM Mapping**: slam_toolbox for persistent mapping with loop closure
- **Navigation**: Full Nav2 navigation stack with DWB controller and NavFn planner
- **Localization**: AMCL for localization with existing maps
- **Visualization**: RViz integration with navigation displays

## Package Structure

```
slam_nav/
├── launch/
│   ├── robot_bringup.launch.py     # Robot hardware and sensors only
│   ├── slam_nav.launch.py          # Full SLAM + Navigation with RViz
│   ├── slam_nav_headless.launch.py # Full SLAM + Navigation without RViz
│   └── localization_nav.launch.py  # Navigation with existing map
├── config/
│   ├── slam_toolbox_config.yaml    # SLAM Toolbox parameters
│   └── nav2_params.yaml           # Nav2 navigation parameters
├── maps/                          # Saved maps directory
├── rviz/                         # RViz configurations
└── README.md
```

## Quick Start

### 1. SLAM Mode (Creating a new map)

Launch the complete SLAM and navigation system with RViz:

```bash
ros2 launch slam_nav slam_nav.launch.py
```

Or launch without RViz (headless mode):

```bash
ros2 launch slam_nav slam_nav_headless.launch.py
```

This will start:
- Robot hardware drivers and sensors
- IMU filtering (Madgwick filter)
- Robot localization (EKF) for sensor fusion
- SLAM Toolbox for mapping
- Nav2 navigation stack
- Joystick control for manual robot operation
- RViz for visualization (only in non-headless mode)

### 2. Navigation Mode (Using existing map)

To navigate with a pre-existing map:

```bash
ros2 launch slam_nav localization_nav.launch.py map:=/path/to/your/map.yaml
```

### 3. Robot Only (No SLAM/Navigation)

To launch just the robot hardware and sensors:

```bash
ros2 launch slam_nav robot_bringup.launch.py
```

### 4. Headless Operation

For headless operation (no display), remote operation, or when you want to run RViz separately:

```bash
# SLAM mode without RViz (includes joystick control for manual driving)
ros2 launch slam_nav slam_nav_headless.launch.py

# Then optionally run RViz separately
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Note**: The headless mode includes joystick control so you can manually drive the robot around to create maps while SLAM is running.

## Detailed Usage

### SLAM Mode

The SLAM mode creates persistent maps that can be saved and reused. The slam_toolbox is configured for:
- Loop closure detection
- Persistent mapping (maps are saved automatically)
- Real-time mapping with the S2 lidar

**Key Features:**
- Maps are saved in `~/.ros/` directory by default
- Interactive mode enabled for map editing
- Optimized for indoor environments

**To save a map manually:**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

### Sensor Fusion

The slam_nav package includes robot localization using an Extended Kalman Filter (EKF) that fuses:
- **Wheel odometry** (`/odom_raw`): Provides position and velocity estimates
- **IMU data** (`/imu/data`): Provides orientation and angular velocity

The EKF outputs a filtered `/odom` topic that combines the best of both sensors:
- More accurate position tracking than wheel odometry alone
- Better orientation estimates from the IMU
- Robust performance even if one sensor temporarily fails

**SLAM Toolbox Data Flow:**
```
Wheel Encoders → /odom_raw ┐
                           ├→ EKF → /odom → SLAM Toolbox
IMU Sensor → /imu/data ────┘
Lidar → /scan ────────────────────────→ SLAM Toolbox
```

This ensures SLAM Toolbox receives the **best possible odometry estimate** from sensor fusion rather than raw encoder data.

### Navigation Mode

Navigation mode uses AMCL for localization and Nav2 for path planning:

**Setting Initial Pose:**
1. In RViz, click "2D Pose Estimate"
2. Click and drag on the map to set robot's initial position and orientation

**Sending Navigation Goals:**
1. In RViz, click "2D Goal Pose" 
2. Click and drag on the map to set the goal position and orientation

### Launch Arguments

All launch files support various arguments for customization:

#### slam_nav.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Start RViz visualization |
| `slam` | `True` | Enable SLAM (set to False for localization only) |
| `params_file` | `nav2_params.yaml` | Nav2 parameters file |
| `slam_params_file` | `slam_toolbox_config.yaml` | SLAM parameters file |
| `autostart` | `true` | Auto-start navigation nodes |
| `use_composition` | `True` | Use composed nodes for better performance |

**Example:**
```bash
ros2 launch slam_nav slam_nav.launch.py use_rviz:=false slam:=True
```

#### localization_nav.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `map` | **Required** | Full path to map YAML file |
| `use_rviz` | `true` | Start RViz visualization |
| `params_file` | `nav2_params.yaml` | Nav2 parameters file |

**Example:**
```bash
ros2 launch slam_nav localization_nav.launch.py map:=/home/user/my_map.yaml
```

#### robot_bringup.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `false` | Start RViz visualization |
| `gui` | `false` | Start joint state publisher GUI |
| `pub_odom_tf` | `false` | Publish odom->base_footprint transform |

## Configuration

### SLAM Toolbox Configuration

The SLAM configuration (`config/slam_toolbox_config.yaml`) is optimized for the S2 lidar:

- **Resolution**: 0.05m (5cm per pixel)
- **Max laser range**: 8.0m (S2 lidar specification)
- **Loop closure**: Enabled with optimized parameters
- **Scan matching**: Enabled for better accuracy

### Navigation Configuration

The Nav2 configuration (`config/nav2_params.yaml`) includes:

- **DWB Controller**: Dynamic Window Approach for local planning
- **NavFn Planner**: Global path planning with A* algorithm
- **Costmaps**: Local and global costmaps with obstacle inflation
- **Robot footprint**: Configured for X3 robot dimensions

### Key Parameters

**Robot Parameters:**
- Robot radius: 0.15m
- Max linear velocity: 0.5 m/s  
- Max angular velocity: 1.0 rad/s

**Sensor Fusion (EKF) Parameters:**
- Frequency: 30 Hz
- 2D mode: Enabled (planar navigation)
- Odometry input: `/odom_raw` (wheel encoders)
- IMU input: `/imu/data` (orientation and angular velocity)
- Output: Filtered `/odom` topic

**Costmap Parameters:**
- Local costmap: 3x3m rolling window
- Global costmap: Uses full map
- Resolution: 0.05m
- Inflation radius: 0.25m

## Topics and Services

### Important Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Lidar data |
| `/odom` | nav_msgs/Odometry | **EKF-fused odometry** (encoders + IMU) |
| `/odom_raw` | nav_msgs/Odometry | Raw wheel encoder odometry |
| `/imu/data` | sensor_msgs/Imu | Filtered IMU data |
| `/map` | nav_msgs/OccupancyGrid | SLAM-generated map |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/goal_pose` | geometry_msgs/PoseStamped | Navigation goals |

### Useful Services

| Service | Type | Description |
|---------|------|-------------|
| `/slam_toolbox/save_map` | slam_toolbox/srv/SaveMap | Save current map |
| `/slam_toolbox/serialize_map` | slam_toolbox/srv/SerializePoseGraph | Save pose graph |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Navigate to pose action |

## Troubleshooting

### Common Issues

1. **Robot not moving**
   - Check if navigation goals are being sent
   - Verify costmaps are being generated
   - Ensure laser scan data is being received

2. **Poor mapping quality**
   - Reduce robot speed during mapping
   - Ensure good lighting conditions
   - Check lidar is clean and unobstructed

3. **Navigation failures**
   - Verify initial pose is set correctly
   - Check for obstacles in costmaps
   - Ensure map and robot coordinate frames match

### Debugging Commands

```bash
# Check topics
ros2 topic list
ros2 topic echo /scan

# Check transforms
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo map base_footprint

# Check navigation status
ros2 topic echo /navigate_to_pose/_action/status
```

## Map Management

### Saving Maps

Maps are automatically saved by slam_toolbox, but you can manually save:

```bash
# Save map with custom name
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'office_map'}}"

# Serialize pose graph for later continuation
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'office_session'}"
```

### Loading Existing Maps

To continue mapping from a previous session:

```bash
ros2 launch slam_nav slam_nav.launch.py slam_params_file:=continue_session.yaml
```

Where `continue_session.yaml` contains:
```yaml
slam_toolbox:
  ros__parameters:
    # ... other parameters ...
    map_file_name: office_session
    map_start_at_dock: true
```

## Performance Tips

1. **Use composition**: Keep `use_composition:=True` for better performance
2. **Adjust frequencies**: Lower update frequencies if CPU usage is high
3. **Optimize costmaps**: Reduce costmap sizes for better performance
4. **Clean lidar**: Keep lidar sensor clean for best mapping results

## Dependencies

This package requires the following ROS2 packages:
- nav2_bringup
- slam_toolbox  
- yahboomcar_bringup
- yahboomcar_description
- sllidar_ros2
- robot_state_publisher
- joint_state_publisher

Install missing dependencies:
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox
```

## Contributing

When modifying configurations:
1. Test thoroughly in both SLAM and navigation modes
2. Update documentation for any new parameters
3. Ensure backward compatibility with existing maps 