#!/usr/bin/env python3
"""
Launch file for Robot Jetson Server
Launches the robot server along with necessary robot systems
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    client_hub_url_arg = DeclareLaunchArgument(
        'client_hub_url',
        default_value='http://192.168.1.100:5000',
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
    
    # Robot server node
    robot_server_node = Node(
        package='robot_integration',  # You may need to create this package
        executable='robot_jetson_server.py',
        name='robot_jetson_server',
        output='screen',
        parameters=[{
            'client_hub_url': LaunchConfiguration('client_hub_url'),
            'robot_id': LaunchConfiguration('robot_id'),
            'send_frequency': LaunchConfiguration('send_frequency')
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg='🚀 Starting Robot Jetson Server for VILA integration'),
        client_hub_url_arg,
        robot_id_arg,
        send_frequency_arg,
        robot_server_node
    ])