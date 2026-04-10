#!/usr/bin/env python3

"""
Robot Hardware Launch File
Brings up the complete robot hardware system.

Includes:
- Complete robot hardware (drivers, sensors, lidar)
- Intel RealSense D435i camera
- EKF sensor fusion (odom_raw -> odom + TF)
- Point cloud height filter
- Robot interface node for client control

SLAM and Nav2 run on the workstation, not here.
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
    slam_nav_dir = get_package_share_directory("slam_nav")

    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level")

    robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("yahboomcar_bringup"), "launch", "robot_bringup_launch.py"))
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("yahboomcar_bringup"), "launch", "ekf_x1_x3_launch.py"))
    )

    pointcloud_height_filter_node = Node(
        package="slam_nav",
        executable="pointcloud_height_filter",
        name="pointcloud_height_filter",
        output="screen",
        parameters=[{
            "input_topic": "/realsense_camera/depth/color/points",
            "output_topic": "/camera/depth/points_filtered",
            "target_frame": "base_footprint",
            "min_height": 0.02,
            "max_height": 0.25,
            "filter_nans": True,
            "voxel_leaf_size": 0.03
        }],
        arguments=["--ros-args", "--log-level", log_level]
    )

    robot_interface_node = Node(
        package="slam_nav",
        executable="robot_interface_node",
        name="robot_interface_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )

    delayed_pointcloud_group = TimerAction(
        period=10.0,
        actions=[GroupAction([pointcloud_height_filter_node])]
    )

    delayed_interface_group = TimerAction(
        period=8.0,
        actions=[GroupAction([robot_interface_node])]
    )


    display_status_node = Node(
        package="slam_nav",
        executable="display_status_node",
        name="display_status_node",
        output="screen",
    )
    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_log_level_cmd,
        robot_bringup_cmd,
        ekf_cmd,
        delayed_pointcloud_group,
        delayed_interface_group,
        display_status_node,
    ])
