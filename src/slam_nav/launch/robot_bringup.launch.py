#!/usr/bin/env python3

"""
Robot bringup launch file for slam_nav package
Brings up the complete robot system including:
- Robot hardware drivers
- Base node for odometry and transforms  
- IMU filtering
- EKF for sensor fusion
- Intel RealSense D435i camera
- S2 lidar
- Robot state publisher
- Joint state publisher
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
    if not os.environ.get("PRINTED_SLAM_NAV"):
        os.environ["PRINTED_SLAM_NAV"] = "1"
        print("---------------------SLAM NAV: X3 with S2 lidar---------------------")
    
    # Launch configuration variables
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')
    
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
    
    
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
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
    

    
    # === HARDWARE DRIVERS ===
    
    # Main robot driver
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        name='yahboomcar_driver',
        output='screen',
        parameters=[{
            'car_type': 'X3',
            'imu_link': 'imu_link',
            'Prefix': '',
            'xlinear_limit': 1.0,
            'ylinear_limit': 1.0,
            'angular_limit': 5.0
        }]
    )
    
    # Base node for odometry and transforms
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[{
            'pub_odom_tf': pub_odom_tf,  # Use LaunchConfiguration parameter
            'base_footprint_frame': 'base_footprint',  # Use base_footprint for REP-105 compliance
        }]
    )
    
    # === IMU AND SENSOR FUSION ===
    
    # IMU filter configuration
    imu_filter_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    )
    
    # IMU filter node - TEMPORARILY DISABLED due to gyroscope bias issues
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
    
    # === CAMERA ===
    
    # RealSense D435i camera node
    realsense_params_file = os.path.join(
        get_package_share_directory('slam_nav'),
        'config',
        'realsense_params.yaml'
    )
    
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='realsense',  # Use realsense namespace
        name='camera',
        output='screen',
        parameters=[realsense_params_file],
        arguments=['--ros-args', '--log-level', 'warn']  # Reduce log verbosity
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
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        gui_arg,
        model_arg,
        pub_odom_tf_arg,
        
        # Robot description and visualization
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        
        # Hardware drivers
        driver_node,
        base_node,
        
        # IMU and sensor fusion
        # imu_filter_node,  # Disabled due to gyroscope bias
        ekf_node,
        
        # Camera
        realsense_node,
        
        # Lidar
        lidar_node,
        
        # Control
        yahboom_joy_node,
        joy_node
    ]) 