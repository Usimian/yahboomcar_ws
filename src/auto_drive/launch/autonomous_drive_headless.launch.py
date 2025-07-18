#!/usr/bin/env python3
"""
Headless Launch file for autonomous driving system
Runs without any graphical display requirements
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for headless autonomous driving"""
    
    # Launch arguments
    enable_autonomous_arg = DeclareLaunchArgument(
        'enable_autonomous',
        default_value='true',
        description='Enable autonomous driving on startup'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.3',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.5',
        description='Maximum angular speed (rad/s)'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.8',
        description='Safety distance for obstacle avoidance (m)'
    )
    
    emergency_distance_arg = DeclareLaunchArgument(
        'emergency_distance',
        default_value='0.4',
        description='Emergency stop distance (m)'
    )
    
    # Include robot bringup (without RViz)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_x3_with_s2_lidar.launch.py'
        ]),
        launch_arguments={
            'use_rviz': 'false',
            'gui': 'false',
            'pub_odom_tf': 'true'
        }.items()
    )
    
    # Autonomous navigator node
    autonomous_navigator = Node(
        package='auto_drive',
        executable='autonomous_navigator',
        name='autonomous_navigator',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'safety_distance': LaunchConfiguration('safety_distance'),
            'emergency_distance': LaunchConfiguration('emergency_distance'),
            'goal_tolerance': 0.2,
            'enable_autonomous': LaunchConfiguration('enable_autonomous')
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/scan', '/scan'),
            ('/imu/data', '/imu/data'),
            ('/odom', '/odom'),
            ('/JoyState', '/JoyState')
        ]
    )
    
    # Optional: Log important topics to console
    log_status_node = Node(
        package='auto_drive',
        executable='autonomous_control',
        name='status_logger',
        output='screen',
        arguments=['status'],
        condition=lambda context: False  # Disabled by default
    )
    
    return LaunchDescription([
        # Launch arguments
        enable_autonomous_arg,
        max_speed_arg,
        max_angular_speed_arg,
        safety_distance_arg,
        emergency_distance_arg,
        
        # Nodes
        robot_bringup,
        autonomous_navigator,
        # log_status_node,  # Uncomment to enable status logging
    ]) 