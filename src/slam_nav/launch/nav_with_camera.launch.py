#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_nav'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # RGB Camera Node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_width': 640},
            {'image_height': 480},
            {'pixel_format': 'yuyv2rgb'},
            {'camera_frame_id': 'camera_link'},
            {'framerate': 30.0},
            {'io_method': 'mmap'},
            {'camera_name': 'camera'},
        ],
        output='screen'
    )

    # Navigation Launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_nav'),
                'launch',
                'slam_nav.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # RGB to Occupancy Converter - Using correct entry point
    rgb_to_occupancy = Node(
        package='slam_nav',
        executable='rgb_to_occupancy',
        name='rgb_to_occupancy',
        parameters=[
            {'camera_height': 0.1},
            {'max_detection_range': 2.0},
            {'obstacle_threshold': 60},
        ],
        output='screen'
    )

    # Camera Navigation Monitor (disabled - can be enabled later if needed)
    # camera_monitor = Node(
    #     package='slam_nav',
    #     executable='camera_nav_monitor.py',
    #     name='camera_nav_monitor',
    #     output='screen'
    # )

    # Static transform from base_link to camera_link
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '0.12', '0', '0.1',  # x, y, z (camera mounted 12cm forward, 10cm up from base_link)
            '0', '0', '0', '1',   # quaternion (no rotation)
            'base_link',          # parent frame
            'camera_link'         # child frame
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        camera_node,
        camera_tf,
        rgb_to_occupancy,  # Now enabled!
        # camera_monitor,    # Still disabled
        nav_launch,
    ]) 