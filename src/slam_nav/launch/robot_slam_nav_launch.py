#!/usr/bin/env python3

"""
Robot SLAM Navigation Launch File - Single Comprehensive Launch
Brings up the complete robot system with SLAM mapping
This is the ONLY launch file needed to run the robot system.

Includes:
- Complete robot hardware system (drivers, sensors, lidar, camera)
- SLAM Toolbox for persistent mapping
- EKF sensor fusion and localization

Robot control via:
1. Manual joystick control
2. Robot Client System (AI-powered commands through /robot/execute_command)
3. Direct /cmd_vel publishing (for testing only)
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
    # Get the launch directory
    slam_nav_dir = get_package_share_directory('slam_nav')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    log_level = LaunchConfiguration('log_level')
    
    # RViz configuration
    use_rviz = LaunchConfiguration('use_rviz')

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'slam_toolbox_config.yaml'),
        description='Full path to the ROS2 parameters file to use for SLAM Toolbox')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')

    # === ROBOT BRINGUP ===
    # Include the yahboomcar_bringup launch file with calibration parameters
    robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch', 'robot_bringup_launch.py')),
        launch_arguments={
            'pub_odom_tf': 'true',   # Enable base_node to publish odom->base_footprint TF
        }.items()
    )

    # === SLAM TOOLBOX ===
    # SLAM Toolbox node for mapping
    slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # === ROBOT INTERFACE ===
    # Direct robot interface node for external client control
    robot_interface_node = Node(
        package='slam_nav',
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Group SLAM components with delay to ensure robot is ready
    delayed_slam_group = TimerAction(
        period=5.0,  # 5 second delay
        actions=[
            GroupAction([
                slam_toolbox_node,
            ])
        ]
    )
    
    # Group Robot Interface with delay to ensure robot and SLAM are ready
    delayed_interface_group = TimerAction(
        period=8.0,  # 8 second delay  
        actions=[
            GroupAction([
                robot_interface_node,
            ])
        ]
    )

    # Return launch description with proper sequencing
    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,
        
        # Declare the launch options
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_log_level_cmd,
        declare_use_rviz_cmd,

        # Launch components in recommended sequence:
        # 1. Robot platform (immediate)
        robot_bringup_cmd,
        # 2. SLAM Toolbox (5s delay - robot ready)
        delayed_slam_group,
        # 3. Robot Interface (8s delay - SLAM ready)
        delayed_interface_group,
    ])
