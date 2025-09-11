#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for Robot Exercise Program with calibration parameters"""
    
    # Robot calibration parameter file
    calibration_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'robot_calibration.yaml'
    )

    robot_exercise_node = Node(
        package='yahboomcar_bringup',
        executable='robot_exercise',
        name='robot_exercise',
        output='screen',
        parameters=[calibration_config],
        # Allow remapping odometry topic if needed
        remappings=[
            ('/odom', '/odom_raw')  # Use raw odometry for more accurate measurements
        ]
    )

    return LaunchDescription([
        robot_exercise_node
    ])
