#!/usr/bin/env python3

"""cuVSLAM recording bring-up (ADDITIVE — does not touch robot_slam_nav_launch).

Brings up everything needed to record a visual-SLAM comparison bag, with the
D435i configured for cuVSLAM VIO:
  - stereo IR (infra1+infra2) + camera IMU (/realsense_camera/imu, unite_imu_method 2)
  - IR emitter OFF (the projected dot pattern corrupts visual features; depth is
    still recorded, just slightly noisier without the dots)
  - depth+IR at 640x360@30 (stereo module shares one profile across depth/IR;
    the nav-tuned 480x270x15 in realsense_params.yaml is overridden ONLY here)
  - RSUSB librealsense via LD_LIBRARY_PATH (kernel lacks hid_sensor; see
    llm-robot-ros docs/host_recovery.md §7b)
Plus the rest of the sensor set for the comparison: URDF/TF, EKF (/odom), lidar.

Run INSTEAD of slam_nav (both open the camera — they cannot run together).
Assumes robot_hardware.service is up (driver, base_node, joystick).

    ros2 launch slam_nav vslam_sensors_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command


def generate_launch_description():
    # URDF / TF (same as robot_slam_nav_launch)
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

    # EKF (odom_raw -> odom + odom->base_footprint TF), same include as slam_nav
    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yahboomcar_bringup'),
                         'launch', 'ekf_x1_x3_launch.py'))
    )

    # Lidar (same params as robot_slam_nav_launch)
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
            'use_sim_time': False,
        }],
    )

    # Camera in cuVSLAM mode. NOTE: this rs_launch version loads config_file
    # params AFTER launch arguments (yaml wins), so all VIO settings live in a
    # dedicated yaml rather than launch-arg overrides. The nav stack's
    # realsense_params.yaml is untouched.
    realsense_config_file = os.path.join(
        get_package_share_directory('slam_nav'), 'config', 'vslam_realsense_params.yaml'
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
        }.items()
    )

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_link', 'realsense_camera_realsense_camera_link'],
    )

    return LaunchDescription([
        # RSUSB librealsense (no-op if the prefix doesn't exist)
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ('/opt/librealsense_rsusb/lib:' if os.path.isdir('/opt/librealsense_rsusb/lib') else '')
            + os.environ.get('LD_LIBRARY_PATH', '')),
        robot_state_publisher_node,
        joint_state_publisher_node,
        ekf_cmd,
        lidar_node,
        camera_node,
        camera_tf_node,
    ])
