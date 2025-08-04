#!/usr/bin/env python3

"""
Robot Data Server Launch File
Launches just the robot data server for Jetson
Sends sensor data to PC client hub (which runs VILA) and receives commands back
Use this if you want to add client hub integration to existing robot systems

Usage:
ros2 launch slam_nav robot_data_server.launch.py client_hub_url:=http://192.168.1.153:5000
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    slam_nav_dir = get_package_share_directory('slam_nav')
    
    # Launch arguments
    client_hub_url_arg = DeclareLaunchArgument(
        'client_hub_url',
        default_value='http://192.168.1.153:5000',
        description='URL of the client hub (PC with VILA)'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='yahboomcar_x3_01',
        description='Unique robot identifier'
    )
    
    send_frequency_arg = DeclareLaunchArgument(
        'send_frequency',
        default_value='2.0',
        description='Frequency to send data to hub (Hz)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'robot_jetson_server.yaml'),
        description='Robot server configuration file'
    )
    
    # Robot server node
    robot_server_node = Node(
        package='slam_nav',
        executable='robot_jetson_server',
        name='robot_jetson_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'client_hub_url': LaunchConfiguration('client_hub_url'),
                'robot_id': LaunchConfiguration('robot_id'),
                'send_frequency': LaunchConfiguration('send_frequency')
            }
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg='🚀 Starting Robot Data Server for Client Hub Communication'),
        client_hub_url_arg,
        robot_id_arg,
        send_frequency_arg,
        config_file_arg,
        robot_server_node
    ])