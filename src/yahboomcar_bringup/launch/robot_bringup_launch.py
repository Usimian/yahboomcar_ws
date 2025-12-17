#!/usr/bin/env python3

"""
Robot Bringup Launch File for Yahboom X3
Brings up the core robot hardware system with calibration parameters.
This launch file uses robot_calibration.yaml and focuses only on robot hardware.

Includes:
- Robot hardware driver (Mcnamu_driver_X3) with calibration parameters
- Base node for odometry and transforms with calibration parameters  
- IMU filtering
- EKF sensor fusion
- Robot state publisher
- Joint state publisher
- Joystick control

Does NOT include:
- SLAM components (handled by higher-level launch files)

Optionally includes:
- Intel RealSense D435i camera (controlled by enable_camera argument)
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Print robot configuration
    if not os.environ.get("PRINTED_ROBOT_BRINGUP"):
        os.environ["PRINTED_ROBOT_BRINGUP"] = "1"
        print("---------------------Yahboom X3 Robot Bringup with Calibration---------------------")
    
    # Get package paths
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X3_simple.urdf'
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='false', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf',
        default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable Intel RealSense D435i camera'
    )
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # === ROBOT CALIBRATION PARAMETERS ===
    
    # Robot calibration parameter file - THIS IS THE KEY ADDITION
    calibration_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'robot_calibration.yaml'
    )
    
    odometry_config = os.path.join(
        get_package_share_directory('yahboomcar_base_node'),
        'config',
        'odometry_scaling.yaml'
    )
    
    # === ROBOT NODES ===
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Joint state publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # === HARDWARE DRIVERS WITH CALIBRATION ===
    
    # Main robot driver - USES CALIBRATION PARAMETERS
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        name='yahboomcar_driver',
        output='screen',
        parameters=[calibration_config]  # Load calibration parameters from YAML
    )
    
    # Base node for odometry and transforms - USES CALIBRATION PARAMETERS
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[
            odometry_config,     # Load odometry scaling parameters
            {'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}  # TF publishing control
        ]
    )
    
    # === IMU AND SENSOR FUSION ===
    
    # IMU filter configuration
    imu_filter_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    )
    
    # IMU filter node - DISABLED due to gyroscope bias issues
    # imu_filter_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter',
    #     output='screen',
    #     parameters=[imu_filter_config]
    # )
    
    # EKF for sensor fusion
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/ekf_x1_x3_launch.py'
        ])
    )
    
    # === LIDAR ===
    
    # S2 lidar node - Simple configuration to get full 360° data
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',     # Try Standard mode instead of DenseBoost
            'use_sim_time': False        # Explicitly set to false for system time usage
        }]
    )
    
    # === CONTROL ===
    
    # Joystick control node
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
        name='yahboom_joy',
        output='screen'
    )
    
    # Joy node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # === CAMERA (OPTIONAL) ===

    # RealSense camera configuration file
    realsense_config = os.path.join(
        get_package_share_directory('slam_nav'),
        'config',
        'realsense_params.yaml'
    )

    # RealSense D435i camera node
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        launch_arguments={
            'config_file': realsense_config,
            'camera_name': 'camera',           # Camera name
            'camera_namespace': '',            # Empty namespace to avoid /camera/camera/ double prefix
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # Return launch description
    return LaunchDescription([
        # Launch arguments
        gui_arg,
        model_arg,
        pub_odom_tf_arg,
        enable_camera_arg,

        # Robot description and visualization
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,

        # Hardware drivers WITH CALIBRATION
        driver_node,
        base_node,

        # IMU and sensor fusion
        # imu_filter_node,  # Disabled due to gyroscope bias
        ekf_node,

        # Lidar
        lidar_node,

        # Camera (optional)
        camera_node,

        # Control
        yahboom_joy_node,
        joy_node
    ])
