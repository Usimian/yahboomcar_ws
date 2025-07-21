# Yahboomcar ROS2 Workspace Documentation

## Project Overview

This is a comprehensive ROS2 workspace for the **Yahboomcar X3** robot platform equipped with **S2 lidar** and **Astra Pro Plus camera**. The project provides a complete robotics development environment with autonomous navigation, SLAM, computer vision, and various control capabilities.

## Hardware Configuration

- **Robot Platform**: Yahboomcar X3 (Mecanum wheel robot)
- **Lidar**: SLAMTEC S2 lidar sensor
- **Camera**: Astra Pro Plus (RGB-D camera)
- **IMU**: Integrated inertial measurement unit
- **Microcontroller**: Communication via `/dev/ttyUSB0`

## Workspace Structure

```
yahboomcar_ws/
├── src/
│   ├── yahboomcar_bringup/         # Core system launch and drivers
│   ├── yahboomcar_base_node/       # Base node for odometry and transforms
│   ├── yahboomcar_description/     # Robot URDF models and meshes
│   ├── yahboomcar_msgs/            # Custom ROS2 message definitions
│   ├── yahboomcar_ctrl/            # Robot control interfaces
│   ├── yahboomcar_multi/           # Multi-robot support
│   ├── yahboomcar_nav/             # Navigation stack configuration
│   ├── yahboomcar_slam/            # SLAM algorithms and mapping
│   ├── yahboomcar_laser/           # Laser processing and obstacle detection
│   ├── yahboomcar_visual/          # Computer vision processing
│   ├── yahboomcar_astra/           # Astra camera integration
│   ├── yahboomcar_mediapipe/       # MediaPipe integration for AI
│   ├── yahboomcar_linefollow/      # Line following algorithms
│   ├── yahboomcar_voice_ctrl/      # Voice control functionality
│   ├── yahboomcar_point/           # Point cloud processing
│   ├── yahboomcar_kcf_tracker/      # KCF object tracking
│   ├── yahboomcar_rviz/            # RViz configurations
│   └── Additional packages...
├── build/                          # Build artifacts
├── install/                        # Installation files
└── log/                           # Log files
```

## Core Packages Description

### 1. yahboomcar_bringup
**Purpose**: Core system initialization and hardware drivers

**Key Features**:
- Main robot hardware driver (`Mcnamu_driver_X3`)
- Complete system launch files
- IMU filtering (Madgwick filter)
- Extended Kalman Filter (EKF) for sensor fusion
- S2 lidar integration
- Joystick control support

**Main Launch File**: `yahboomcar_x3_with_s2_lidar.launch.py`
- Brings up complete robot system
- Configurable with various options (RViz, GUI, custom URDF)
- Publishes essential topics: `/scan`, `/imu/data`, `/odom`, `/cmd_vel`

### 2. yahboomcar_description
**Purpose**: Robot model definitions and visualization

**Key Features**:
- URDF models for X3 and R2 robot variants
- 3D mesh files for visualization
- Support for multi-robot configurations
- Xacro-based parametric models

**Robot Models**:
- `yahboomcar_X3.urdf`: Main X3 robot model
- `yahboomcar_R2.urdf`: R2 robot variant
- Multi-robot support with namespacing

### 3. yahboomcar_nav
**Purpose**: Autonomous navigation capabilities

**Key Features**:
- **Navigation Stack**: Full Nav2 integration
- **Path Planning**: Multiple algorithms (DWA, TEB, NavFn)
- **Localization**: AMCL (Adaptive Monte Carlo Localization)
- **Costmaps**: Global and local costmap configurations
- **Behavior Trees**: Navigation behavior management

**Navigation Algorithms**:
- **DWA (Dynamic Window Approach)**: Local path planning
- **TEB (Timed Elastic Band)**: Advanced trajectory optimization
- **RTABMap**: Real-time appearance-based mapping

### 4. yahboomcar_slam
**Purpose**: Simultaneous Localization and Mapping

**Key Features**:
- **ORB-SLAM2/3**: Visual SLAM with RGB-D cameras
- **Point Cloud Mapping**: Real-time 3D mapping
- **Octomap Integration**: 3D occupancy mapping
- **Multiple SLAM Modes**: Mono, stereo, RGB-D

**SLAM Capabilities**:
- Visual odometry and mapping
- Loop closure detection
- Point cloud generation and processing
- Map saving and loading

### 5. yahboomcar_laser
**Purpose**: Laser-based perception and navigation

**Key Features**:
- **Obstacle Avoidance**: Real-time obstacle detection
- **Laser Tracking**: Follow closest objects
- **Warning System**: Collision prevention with buzzer alerts
- **PID Control**: Smooth motion control

**Laser Algorithms**:
- `laser_Avoidance_a1_X3`: Obstacle avoidance for X3
- `laser_Tracker_a1_X3`: Object tracking
- `laser_Warning_a1_X3`: Collision warning system

### 6. yahboomcar_visual
**Purpose**: Computer vision and image processing

**Key Features**:
- **RGB-D Processing**: Color and depth image handling
- **AR Visualization**: Augmented reality overlays
- **Image Publishing**: ROS2 image topic management
- **Camera Calibration**: Intrinsic parameter handling

### 7. yahboomcar_astra
**Purpose**: Astra camera integration

**Key Features**:
- **Color Tracking**: HSV-based object tracking
- **Depth Processing**: 3D object detection
- **PID Control**: Vision-based robot control
- **Real-time Processing**: Live camera feed processing

### 8. yahboomcar_mediapipe
**Purpose**: AI-powered computer vision

**Key Features**:
- **Hand Detection**: Real-time hand tracking
- **Pose Estimation**: Human pose detection
- **Face Recognition**: Face and eye detection
- **Gesture Recognition**: Hand gesture interpretation
- **Holistic Analysis**: Combined face, pose, and hand detection

### 9. yahboomcar_linefollow
**Purpose**: Line following capabilities

**Key Features**:
- **Color-based Detection**: HSV color space line detection
- **PID Control**: Smooth line following
- **Obstacle Integration**: Laser-based obstacle avoidance
- **Dynamic Tuning**: Real-time parameter adjustment

### 10. yahboomcar_voice_ctrl
**Purpose**: Voice command integration

**Key Features**:
- **Speech Recognition**: Voice command processing
- **Robot Control**: Voice-based movement commands
- **Integration**: Works with existing control systems

### 11. yahboomcar_msgs
**Purpose**: Custom message definitions

**Message Types**:
- `Position.msg`: 3D position with angles
- `Target.msg`: Object detection results
- `TargetArray.msg`: Multiple target tracking
- `ImageMsg.msg`: Custom image format
- `PointArray.msg`: Point cloud data

## Key Features and Capabilities

### 1. Autonomous Navigation
- **Full Nav2 Stack**: Complete autonomous navigation
- **Multiple Planners**: DWA, TEB, and NavFn support
- **Dynamic Obstacle Avoidance**: Real-time path replanning
- **Localization**: AMCL and visual odometry

### 2. SLAM and Mapping
- **Visual SLAM**: ORB-SLAM2/3 integration
- **3D Mapping**: Point cloud and octomap generation
- **Real-time Processing**: Live mapping capabilities
- **Map Persistence**: Save and load maps

### 3. Computer Vision
- **RGB-D Processing**: Color and depth integration
- **Object Tracking**: Multiple tracking algorithms
- **AI Integration**: MediaPipe for advanced vision
- **Augmented Reality**: AR visualization support

### 4. Sensor Fusion
- **IMU Integration**: Madgwick filter for orientation
- **EKF**: Extended Kalman Filter for state estimation
- **Multi-sensor**: Lidar, camera, and IMU fusion
- **Robust Localization**: Sensor redundancy

### 5. Control Systems
- **Joystick Control**: Manual robot operation
- **Voice Control**: Speech-based commands
- **Autonomous Modes**: Various autonomous behaviors
- **Safety Systems**: Collision avoidance and warnings

## Launch Files and Usage

### Basic System Startup
```bash
# Launch complete robot system
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py

# Launch with RViz visualization
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py use_rviz:=true

# Launch with GUI controls
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py gui:=true
```

### Navigation
```bash
# Launch navigation with DWA planner
ros2 launch yahboomcar_nav navigation_dwa_launch.py

# Launch navigation with TEB planner
ros2 launch yahboomcar_nav navigation_teb_launch.py

# Launch RTABMap navigation
ros2 launch yahboomcar_nav navigation_rtabmap_launch.py
```

### SLAM
```bash
# Launch ORB-SLAM with RGB-D
ros2 launch yahboomcar_slam orbslam_ros_launch.py orb_slam_type:=rgbd

# Launch point cloud mapping
ros2 launch yahboomcar_slam orbslam_pcl_map_launch.py

# Launch with octomap
ros2 launch yahboomcar_slam orbslam_pcl_octomap_launch.py
```

### Computer Vision
```bash
# Launch line following
ros2 launch yahboomcar_linefollow follow_line_a1_X3_launch.py

# Launch object tracking
ros2 run yahboomcar_astra colorTracker

# Launch MediaPipe hand detection
ros2 run yahboomcar_mediapipe 01_HandDetector
```

### Laser Processing
```bash
# Launch obstacle avoidance
ros2 run yahboomcar_laser laser_Avoidance_a1_X3

# Launch object tracking
ros2 run yahboomcar_laser laser_Tracker_a1_X3

# Launch warning system
ros2 run yahboomcar_laser laser_Warning_a1_X3
```

## Configuration and Parameters

### Robot Parameters
- **Linear Velocity Limits**: 0.0 - 1.0 m/s
- **Angular Velocity Limits**: 0.0 - 5.0 rad/s
- **Wheel Configuration**: Mecanum wheels
- **Base Dimensions**: Defined in URDF

### Sensor Parameters
- **Lidar Range**: 0.1 - 8.0 meters
- **Camera Resolution**: 640x480 pixels
- **IMU Frequency**: 30 Hz
- **Laser Scan Frequency**: 10 Hz

### Navigation Parameters
- **Costmap Resolution**: 0.05 meters
- **Planning Frequency**: 20 Hz
- **Controller Frequency**: 10 Hz
- **Robot Radius**: 0.1 meters

## Development Environment

### Dependencies
- **ROS2**: Humble or later
- **OpenCV**: 4.x
- **PCL**: Point Cloud Library
- **MediaPipe**: Google's ML framework
- **Nav2**: Navigation stack
- **RTABMap**: SLAM library

### Build Instructions
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build

# Source workspace
source install/setup.bash
```

## Troubleshooting

### Common Issues
1. **Lidar Connection**: Check USB permissions and device path
2. **Camera Issues**: Verify camera drivers and topics
3. **Transform Errors**: Ensure all TF publishers are running
4. **Navigation Failures**: Check costmap configurations

### Debugging Commands
```bash
# Check topics
ros2 topic list

# Monitor transforms
ros2 run tf2_tools view_frames

# Check node status
ros2 node list

# View logs
ros2 log info
```

## Future Enhancements

### Planned Features
- **Multi-robot Coordination**: Swarm robotics capabilities
- **Advanced AI**: Deep learning integration
- **Cloud Connectivity**: Remote monitoring and control
- **Enhanced SLAM**: Multi-session mapping

### Research Areas
- **Semantic SLAM**: Object-aware mapping
- **Human-Robot Interaction**: Social robotics
- **Autonomous Exploration**: Unknown environment mapping
- **Adaptive Control**: Learning-based control systems

## Contributing

### Development Guidelines
1. Follow ROS2 coding standards
2. Write comprehensive documentation
3. Include unit tests for new features
4. Maintain backward compatibility

### Package Structure
- Each package should have clear purpose
- Include proper package.xml metadata
- Provide launch files for major features
- Document parameters and topics

## License and Support

This project is developed for educational and research purposes. For technical support and contributions, please refer to the project maintainers.

---

**Last Updated**: 2024
**ROS2 Version**: Humble
**Platform**: Ubuntu 22.04 LTS
**Hardware**: Yahboomcar X3 with S2 Lidar and Astra Pro Plus Camera 