# Yahboomcar X3 Robot Description

This package contains the robot description files (URDF, meshes, and configurations) for the Yahboomcar X3 platform.

## Robot Specifications

### Physical Dimensions
- **Length**: 230mm
- **Width**: 150mm (chassis), 200mm (including wheels)
- **Height**: 80mm (main chassis)
- **Weight**: ~486g (base chassis)

### Wheel Configuration
- **Type**: Mecanum wheels
- **Diameter**: 64mm
- **Wheelbase**: 160mm (front to rear axle)
- **Track Width**: 174mm (center-to-center), 200mm (outer edge-to-edge)
- **Ground Clearance**: 17.5mm (bottom of chassis to ground)

### Sensor Configuration

#### Intel Realsense D435i Camera
- **Position**: 15mm behind front edge, centered
- **Height**: 122mm above ground
- **Mounting**: Front-facing, fixed
- **Purpose**: RGB-D vision, SLAM, navigation

#### SLAMTEC S2 Lidar
- **Position**: 71.5mm behind front edge, centered  
- **Height**: 191.5mm above ground
- **Range**: 8 meters, 360° scanning
- **Purpose**: 2D mapping, obstacle detection, navigation

#### IMU (Inertial Measurement Unit)
- **Position**: 58.5mm in front of rear edge, 15mm left of center
- **Height**: 191.5mm above ground (level with lidar)
- **Purpose**: Orientation, sensor fusion, navigation

## URDF Files

- **`yahboomcar_X3_simple.urdf`**: Simplified robot model with basic geometry
- **`yahboomcar_X3.urdf`**: Detailed model with STL meshes and physics
- **`yahboomcar_X3_robot1.urdf`**: Multi-robot configuration (robot1)
- **`yahboomcar_X3_robot2.urdf`**: Multi-robot configuration (robot2)
- **`yahboomcar_X3.urdf.xacro`**: Parametric Xacro model

## Usage

### Launch Robot Description
```bash
# Launch with simple URDF
ros2 launch yahboomcar_description display_simple.launch.py

# Launch with detailed URDF
ros2 launch yahboomcar_description display_detailed.launch.py
```

### View in RViz
```bash
# Launch RViz with robot model
ros2 launch yahboomcar_bringup yahboomcar_x3_with_s2_lidar.launch.py use_rviz:=true
```

## Coordinate Frames

- **base_footprint**: Ground-level reference frame
- **base_link**: Main robot chassis center (81.5mm above ground)
- **camera_link**: Camera sensor frame
- **laser_link**: Lidar sensor frame  
- **imu_link**: IMU sensor frame
- **wheel links**: Individual wheel frames (front_left, front_right, back_left, back_right)

## Visual Appearance

The robot model uses distinct colors for easy identification:
- **🟢 Chassis**: Green box (main robot body)
- **⚫ Wheels**: Black cylinders (4 mecanum wheels)
- **🔵 Camera**: Blue box (Intel Realsense D435i)
- **🔴 Lidar**: Red cylinder (SLAMTEC S2)
- **🟣 IMU**: Purple box (inertial measurement unit)

## Files Structure
```
yahboomcar_description/
├── urdf/                    # Robot URDF files
├── meshes/                  # 3D mesh files (STL)
├── launch/                  # Launch files
├── rviz/                    # RViz configuration files
└── README.md               # This file
```