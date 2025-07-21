#!/usr/bin/env python3

"""
Example launch file for slam_nav package
Demonstrates how to launch SLAM and navigation with custom parameters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    slam_nav_dir = get_package_share_directory('slam_nav')
    
    # Example: Launch SLAM and navigation with RViz
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_nav_dir, 'launch', 'slam_nav.launch.py')),
        launch_arguments={
            'use_rviz': 'true',
            'slam': 'True',
            'autostart': 'true',
            'use_composition': 'True'
        }.items())

    return LaunchDescription([
        slam_nav_launch
    ]) 