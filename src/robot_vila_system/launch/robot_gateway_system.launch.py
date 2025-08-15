#!/usr/bin/env python3

"""
Robot Gateway System Launch File
Starts the single gateway architecture with proper robot communication system
- robot_client_node: Implements ExecuteCommand service gateway
- gateway_validator_node: Monitors for cmd_vel violations
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='yahboomcar_x3_01',
        description='Unique robot identifier'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes'
    )
    
    # Robot Client Node - THE single gateway
    robot_client_node = Node(
        package='robot_vila_system',
        executable='robot_client_node',
        name='robot_client_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Gateway Validator Node - monitors compliance
    gateway_validator_node = Node(
        package='robot_vila_system',
        executable='gateway_validator_node',
        name='gateway_validator_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        LogInfo(msg='🚀 Starting Robot Gateway System - Single Command Architecture'),
        LogInfo(msg='🚪 ExecuteCommand service will be available at: /robot/execute_command'),
        LogInfo(msg='🛡️  Gateway validator will monitor for cmd_vel violations'),
        
        robot_id_arg,
        log_level_arg,
        
        robot_client_node,
        gateway_validator_node,
        
        LogInfo(msg='✅ Robot Gateway System launched successfully'),
    ])
