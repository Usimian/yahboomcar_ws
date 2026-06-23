#!/usr/bin/env python3

"""
Robot Bringup Launch File for Yahboom X3
Brings up the core robot hardware system with calibration parameters.
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Print robot configuration
    if not os.environ.get("PRINTED_ROBOT_BRINGUP"):
        os.environ["PRINTED_ROBOT_BRINGUP"] = "1"
        print("---------------------Yahboom X3 Robot Bringup with Calibration---------------------")

    # Get package paths
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X3.urdf'

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', str(default_model_path)]),
        value_type=str
    )

    # === ROBOT CALIBRATION PARAMETERS ===

    calibration_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'robot_calibration.yaml'
    )

    odometry_config = os.path.join(
        get_package_share_directory('yahboomcar_base_node'),
        'config',
        'odometry_scaling.yaml'
    )

    # === ROBOT NODES ===

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # === HARDWARE DRIVERS WITH CALIBRATION ===

    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        name='yahboomcar_driver',
        output='screen',
        parameters=[calibration_config]
    )

    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[
            odometry_config,
            {'pub_odom_tf': False}
        ]
    )

    # === LIDAR ===

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False
        }]
    )

    # === CONTROL ===

    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
        name='yahboom_joy',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # === CAMERA ===

    realsense_config_file = os.path.join(
        get_package_share_directory('slam_nav'),
        'config',
        'realsense_params.yaml'
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        launch_arguments={
            'config_file': realsense_config_file,
            'camera_name': 'realsense_camera',
            'camera_namespace': '',
            'publish_tf': 'true',
            'tf_publish_rate': '0.0',
        }.items()
    )

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'realsense_camera_realsense_camera_link'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        driver_node,
        base_node,
        lidar_node,
        camera_node,
        camera_tf_node,
        yahboom_joy_node,
        joy_node
    ])
