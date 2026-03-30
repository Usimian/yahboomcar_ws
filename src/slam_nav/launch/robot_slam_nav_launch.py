#!/usr/bin/env python3

"""
Robot SLAM Navigation Launch File - Single Comprehensive Launch
Brings up the complete robot system with SLAM mapping
This is the ONLY launch file needed to run the robot system.

Includes:
- Complete robot hardware system (drivers, sensors, lidar)
- Intel RealSense D435i camera (depth, color, point cloud)
- SLAM Toolbox for persistent mapping
- EKF sensor fusion and localization

Robot control via:
1. Manual joystick control
2. Robot Client System (AI-powered commands through /robot/execute_command)
3. Direct /cmd_vel publishing (for testing only)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                          IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    slam_nav_dir = get_package_share_directory('slam_nav')

    log_level = LaunchConfiguration('log_level')

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # === ROBOT BRINGUP ===
    # Include the yahboomcar_bringup launch file with calibration parameters
    robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch', 'robot_bringup_launch.py'))
    )

    # === SLAM TOOLBOX ===
    # SLAM Toolbox node for mapping - using async for proper odometry subscription
    slam_toolbox_node = Node(
        parameters=[
            os.path.join(slam_nav_dir, 'config', 'slam_toolbox_config.yaml'),
            {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # === POINT CLOUD HEIGHT FILTER ===
    # Filter RealSense point cloud by height to keep only relevant obstacles
    # Robot is 25cm tall, filter shows everything from 1cm to 260cm above floor
    pointcloud_height_filter_node = Node(
        package='slam_nav',
        executable='pointcloud_height_filter',
        name='pointcloud_height_filter',
        output='screen',
        parameters=[{
            'input_topic': '/realsense_camera/depth/color/points',
            'output_topic': '/camera/depth/points_filtered',
            'target_frame': 'base_link',
            'min_height': 0.01,  # 1cm above floor
            'max_height': 2.60,  # 260cm above floor
            'filter_nans': True,
            'voxel_leaf_size': 0.03  # 3cm voxel downsampling
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # === ROBOT INTERFACE ===
    # Direct robot interface node for external client control
    robot_interface_node = Node(
        package='slam_nav',
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Group point cloud filter with delay to ensure camera and TF tree are ready
    delayed_pointcloud_group = TimerAction(
        period=10.0,  # 10 second delay for camera startup and full TF tree initialization
        actions=[
            GroupAction([
                pointcloud_height_filter_node,
            ])
        ]
    )

    # Group SLAM components with delay to ensure robot is ready
    delayed_slam_group = TimerAction(
        period=5.0,  # 5 second delay
        actions=[
            GroupAction([
                slam_toolbox_node,
            ])
        ]
    )

    # Group Robot Interface with delay to ensure robot and SLAM are ready
    delayed_interface_group = TimerAction(
        period=8.0,  # 8 second delay
        actions=[
            GroupAction([
                robot_interface_node,
            ])
        ]
    )

    # Return launch description with proper sequencing
    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,

        declare_log_level_cmd,

        # Launch components in recommended sequence:
        # 1. Robot platform (immediate)
        robot_bringup_cmd,
        # 2. Point cloud filter (10s delay - camera ready)
        delayed_pointcloud_group,
        # 3. SLAM Toolbox (5s delay - robot ready)
        delayed_slam_group,
        # 4. Robot Interface (8s delay - SLAM ready)
        delayed_interface_group,
    ])
