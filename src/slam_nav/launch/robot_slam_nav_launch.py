#!/usr/bin/env python3

"""SLAM/Nav bringup.

Assumes robot_hardware.service is already running at boot (MCU driver + joystick
+ base_node). This launch brings up everything else needed for SLAM/Nav:
URDF/TF publishing, camera, lidar, EKF, pointcloud filter, and interface nodes.

SLAM and Nav2 themselves run on the workstation.
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, DeclareLaunchArgument, GroupAction,
                          IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    # Drop the redundant [node_name] tag from log lines — the launch prefix
    # already shows which process the line came from.
    log_format_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity} {time}] {message}")

    # Prepend the RSUSB-backend librealsense (built from source) so the
    # realsense node loads it instead of the apt V4L2 build. The JetPack
    # kernel has no hid-sensor support (CONFIG_HID_SENSOR_HUB unset), so the
    # Prefer the hand-built kernel-native librealsense in /usr/local: the
    # RSUSB build stalls (uvc endpoint watchdogs, mute starts — 2026-07-05).
    # Sourcing ROS puts the apt copy on LD_LIBRARY_PATH, so this must win.
    realsense_ld_envvar = SetEnvironmentVariable(
        "LD_LIBRARY_PATH",
        "/usr/local/lib:" + os.environ.get("LD_LIBRARY_PATH", ""))

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level")

    # URDF / TF
    urdf_path = get_package_share_path('yahboomcar_description') / 'urdf/yahboomcar_X3.urdf'
    robot_description = ParameterValue(Command(['xacro ', str(urdf_path)]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['camera_tilt_joint_state']}],
    )

    # EKF (odom_raw -> odom + TF)
    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("yahboomcar_bringup"),
                         "launch", "ekf_x1_x3_launch.py"))
    )

    # Lidar
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/rplidar",
            "serial_baudrate": 1000000,
            "frame_id": "laser_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
            "use_sim_time": False,
        }],
    )



    # RealSense camera
    realsense_config_file = os.path.join(
        get_package_share_directory("slam_nav"), "config", "realsense_params.yaml"
    )
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"), "launch"),
            "/rs_launch.py"
        ]),
        launch_arguments={
            "config_file": realsense_config_file,
            "camera_name": "realsense_camera",
            "camera_namespace": "",
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
        }.items()
    )
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_base_tf",
        arguments=["0", "0", "0", "0", "0", "0",
                   "camera_link", "realsense_camera_realsense_camera_link"],
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
            "voxel_leaf_size": 0.03,
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

    display_status_node = Node(
        package="slam_nav",
        executable="display_status_node",
        name="display_status_node",
        output="screen",
    )

    # RealSense depth-stream watchdog: USB-resets the camera if depth stops
    # publishing for 5 seconds. Works around the well-known librealsense bug
    # where the depth pipeline silently stalls on D435/D435i.
    realsense_watchdog_node = Node(
        package="slam_nav",
        executable="realsense_watchdog",
        name="realsense_watchdog",
        output="screen",
    )
    delayed_watchdog_group = TimerAction(
        period=15.0,
        actions=[GroupAction([realsense_watchdog_node])],
    )

    delayed_pointcloud_group = TimerAction(
        period=10.0,
        actions=[GroupAction([pointcloud_height_filter_node])]
    )
    color_compressor_node = ExecuteProcess(
        name='color_compressor',
        cmd=[
            os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'jazzy'), 'lib/image_transport/republish'),
            'raw', 'compressed',
            '--ros-args',
            '-r', '__node:=color_compressor',
            '-r', 'in:=/realsense_camera/color/image_raw',
            '-r', 'out/compressed:=/realsense_camera/color/image_raw/compressed',
        ],
        output='screen',
    )
    delayed_compressor_group = TimerAction(
        period=12.0,
        actions=[GroupAction([color_compressor_node])],
    )
    depth_compressor_node = ExecuteProcess(
        name='depth_compressor',
        cmd=[
            os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'jazzy'), 'lib/image_transport/republish'),
            'raw', 'compressedDepth',
            '--ros-args',
            '-r', '__node:=depth_compressor',
            '-r', 'in:=/realsense_camera/aligned_depth_to_color/image_raw',
            '-r', 'out/compressedDepth:=/realsense_camera/aligned_depth_to_color/image_raw/compressedDepth',
        ],
        output='screen',
    )
    delayed_depth_compressor_group = TimerAction(
        period=12.0,
        actions=[GroupAction([depth_compressor_node])],
    )
    delayed_interface_group = TimerAction(
        period=8.0,
        actions=[GroupAction([robot_interface_node])]
    )

    return LaunchDescription([
        realsense_ld_envvar,
        stdout_linebuf_envvar,
        log_format_envvar,
        declare_log_level_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        lidar_node,
        camera_node,
        camera_tf_node,
        ekf_cmd,
        delayed_pointcloud_group,
        delayed_watchdog_group,
        delayed_interface_group,
        delayed_compressor_group,
        delayed_depth_compressor_group,
        display_status_node,
    ])
