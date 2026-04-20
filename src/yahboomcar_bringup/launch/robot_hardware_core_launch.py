#!/usr/bin/env python3
"""Always-on hardware core.

Owns /dev/ttyUSB0 via Rosmaster_Lib for the entire uptime of the Jetson.
Started at boot by robot_hardware.service — must never be included by other
launch files (would cause serial port contention).

Minimal scope: only nodes that require the MCU or the gamepad.
TF / URDF publishing belongs with whatever stack consumes it (slam_nav).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'), 'config', 'robot_calibration.yaml'
    )
    odometry_config = os.path.join(
        get_package_share_directory('yahboomcar_base_node'), 'config', 'odometry_scaling.yaml'
    )

    return LaunchDescription([
        Node(
            package='yahboomcar_bringup',
            executable='Mcnamu_driver_X3',
            name='yahboomcar_driver',
            output='screen',
            parameters=[calibration_config],
        ),
        Node(
            package='yahboomcar_base_node',
            executable='base_node_X3',
            name='base_node',
            output='screen',
            parameters=[odometry_config, {'pub_odom_tf': False}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='yahboomcar_ctrl',
            executable='yahboom_joy_X3',
            name='yahboom_joy',
            output='screen',
        ),
    ])
