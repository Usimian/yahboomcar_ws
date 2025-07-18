# Yahboomcar X3 with S2 Lidar Launch File

This launch file (`yahboomcar_x3_with_s2_lidar.launch.py`) provides a complete startup solution for the yahboomcar X3 robot with S2 lidar sensor.

## Features

The launch file brings up the following components:

### Hardware Drivers
- **Mcnamu_driver_X3**: Main robot hardware driver for X3 platform
- **base_node_X3**: Base node for odometry and coordinate transforms
- **S2 Lidar**: SLAMTEC S2 lidar sensor via `sllidar_ros2` package

### Sensor Processing
- **IMU Filter**: Madgwick filter for IMU data processing
- **EKF**: Extended Kalman Filter for sensor fusion (odometry + IMU)
- **Static TF**: Transform from base_link to laser frame

### Robot Model & Visualization
- **Robot State Publisher**: Publishes robot model and transforms
- **Joint State Publisher**: Publishes joint states
- **RViz**: Optional 3D visualization (disabled by default)

### Control
- **Joystick Control**: Support for joystick/gamepad control
- **Joy Node**: ROS2 joystick driver

## Usage

### Basic Launch
```bash
# Launch the complete system
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py
```

### Launch with Options
```bash
# Launch with RViz for visualization
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py use_rviz:=true

# Launch with joint state publisher GUI
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py gui:=true

# Launch with custom URDF model
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py model:=/path/to/custom.urdf

# Launch with odometry transform publishing enabled
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py pub_odom_tf:=true
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `false` | Enable joint_state_publisher_gui |
| `model` | `yahboomcar_X3.urdf` | Path to robot URDF file |
| `rvizconfig` | `yahboomcar.rviz` | Path to RViz configuration file |
| `pub_odom_tf` | `false` | Publish transform from odom to base_footprint |
| `use_rviz` | `false` | Start RViz for visualization |

## Published Topics

### Sensor Data
- `/scan` - Laser scan data from S2 lidar
- `/imu/data` - Filtered IMU data
- `/odom` - Odometry data

### Robot State
- `/robot_description` - Robot model description
- `/joint_states` - Joint state information
- `/tf` - Transform tree
- `/tf_static` - Static transforms

### Control
- `/cmd_vel` - Velocity commands for robot control
- `/joy` - Joystick input data

## Hardware Requirements

- Yahboomcar X3 robot platform
- SLAMTEC S2 lidar sensor
- USB connection for lidar (/dev/ttyUSB* or /dev/rplidar)
- Optional: USB joystick/gamepad for manual control

## Dependencies

Make sure the following packages are installed:
- `sllidar_ros2` - S2 lidar driver
- `imu_filter_madgwick` - IMU filtering
- `robot_localization` - EKF for sensor fusion
- `joy` - Joystick support
- `robot_state_publisher` - Robot model publishing
- `joint_state_publisher` - Joint state publishing

## Troubleshooting

### Lidar Connection Issues
- Check that the lidar is connected to the correct USB port
- Verify permissions: `sudo chmod 666 /dev/ttyUSB*`
- Check if the `sllidar_ros2` package is properly installed

### IMU Issues
- Ensure the IMU filter parameter file exists: `param/imu_filter_param.yaml`
- Check IMU topic publishing: `ros2 topic echo /imu/data_raw`

### Transform Issues
- Verify all transforms are being published: `ros2 run tf2_tools view_frames`
- Check static transform from base_link to laser: `ros2 topic echo /tf_static`

### Control Issues
- Test joystick connection: `ros2 run joy joy_node`
- Check joystick permissions and device path
- Verify cmd_vel topic: `ros2 topic echo /cmd_vel`

## Example Usage Workflow

1. **Start the system**:
   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py
   ```

2. **Verify lidar data**:
   ```bash
   ros2 topic echo /scan
   ```

3. **Check transforms**:
   ```bash
   ros2 run tf2_ros tf2_echo base_link laser
   ```

4. **Test manual control** (with joystick connected):
   ```bash
   # The joystick should already be active from the launch file
   # Move the joystick to control the robot
   ```

5. **Visualize in RViz**:
   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py use_rviz:=true
   ```

## Integration with Other Systems

This launch file provides a solid foundation for:
- SLAM (Simultaneous Localization and Mapping)
- Navigation stack integration
- Autonomous navigation
- Sensor fusion applications
- Custom robot applications

The system publishes all necessary topics and transforms for integration with ROS2 navigation stack, SLAM algorithms, and other robotic applications. 