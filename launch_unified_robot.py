#!/usr/bin/env python3
"""
Launch file for Unified Robot System
Launches the unified robot bridge node with proper parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value='yahboom_robot_001',
            description='Unique robot identifier'
        ),
        DeclareLaunchArgument(
            'controller_host',
            default_value='192.168.1.100',
            description='Unified controller host IP'
        ),
        DeclareLaunchArgument(
            'controller_port',
            default_value='5000',
            description='Unified controller port'
        ),
        
        # Launch unified robot bridge node
        Node(
            package='yahboomcar_ws',  # Replace with your actual package name
            executable='unified_robot_bridge.py',
            name='unified_robot_bridge',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'controller_host': LaunchConfiguration('controller_host'),
                'controller_port': LaunchConfiguration('controller_port')
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Launch updated robot server (using proper ROS2 entry point)
        Node(
            package='slam_nav',  # Correct package name
            executable='robot_jetson_server',  # ROS2 entry point (no .py)
            name='robot_jetson_server',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'controller_host': LaunchConfiguration('controller_host'),
                'controller_port': LaunchConfiguration('controller_port')
            }],
            output='screen',
            emulate_tty=True
        )
    ])

if __name__ == '__main__':
    generate_launch_description()