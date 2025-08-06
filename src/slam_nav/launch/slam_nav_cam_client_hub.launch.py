#!/usr/bin/env python3

"""
SLAM Navigation with Camera and Client Hub Integration Launch File
Extends slam_nav_cam.launch.py with robot data server for client hub communication
Brings up:
- Complete robot system (robot_bringup.launch.py with built-in IMU)
- Intel Realsense D435i camera (color + depth, IMU disabled)
- SLAM Toolbox for persistent mapping
- Nav2 navigation stack
- Robot Data Server (sends data to PC client hub for VILA processing)

Usage:
ros2 launch slam_nav slam_nav_cam_client_hub.launch.py client_hub_url:=http://192.168.1.153:5000
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                          IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    slam_nav_dir = get_package_share_directory('slam_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Initial pose parameters
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # Single robot system parameters
    client_hub_url = LaunchConfiguration('client_hub_url')
    controller_host = LaunchConfiguration('controller_host')
    controller_port = LaunchConfiguration('controller_port')
    robot_id = LaunchConfiguration('robot_id')
    send_frequency = LaunchConfiguration('send_frequency')
    command_poll_frequency = LaunchConfiguration('command_poll_frequency')

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'slam_toolbox_config.yaml'),
        description='Full path to the SLAM parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose x coordinate')

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose y coordinate')

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial pose yaw angle')

    # Single Robot System arguments (simplified)
    declare_client_hub_url_cmd = DeclareLaunchArgument(
        'client_hub_url',
        default_value='http://192.168.1.153:5000',
        description='Legacy: URL of the server (PC with VILA) - will be parsed for single robot system')
    
    declare_controller_host_cmd = DeclareLaunchArgument(
        'controller_host',
        default_value='192.168.1.153',
        description='Single robot server host IP address')
    
    declare_controller_port_cmd = DeclareLaunchArgument(
        'controller_port',
        default_value='5000',
        description='Single robot server port number')

    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_id',
        default_value='yahboomcar_x3_01',
        description='Robot identifier (hardcoded as yahboomcar_x3_01)')

    declare_send_frequency_cmd = DeclareLaunchArgument(
        'send_frequency',
        default_value='2.0',
        description='Frequency to send image/sensor data to server (Hz)')
    
    declare_command_poll_frequency_cmd = DeclareLaunchArgument(
        'command_poll_frequency',
        default_value='2.0',
        description='Frequency to poll server for commands (Hz)')

    # === ROBOT BRINGUP ===
    # Include the main slam_nav_cam launch file
    slam_nav_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_nav_dir, 'launch', 'slam_nav_cam.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items()
    )

    # === SINGLE ROBOT SYSTEM INTEGRATION ===
    robot_server_config = os.path.join(slam_nav_dir, 'config', 'robot_jetson_server.yaml')
    
    robot_server_node = Node(
        package='slam_nav',
        executable='robot_jetson_server',  
        name='robot_jetson_server',
        output='screen',
        parameters=[
            robot_server_config,
            {
                # Single robot system parameters
                'controller_host': controller_host,
                'controller_port': controller_port,
                'robot_id': robot_id,  # Hardcoded as yahboomcar_x3_01
                'send_frequency': send_frequency,
                'command_poll_frequency': command_poll_frequency,
                # Legacy parameter for backward compatibility
                'client_hub_url': client_hub_url,
            }
        ]
    )

    # Delayed single robot server launch - start after robot systems are ready
    delayed_robot_server = TimerAction(
        period=15.0,  # Start after SLAM and Nav2 are initialized
        actions=[robot_server_node]
    )

    # Return launch description
    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,
        
        # Declare the launch options
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_initial_pose_x_cmd,
        declare_initial_pose_y_cmd,
        declare_initial_pose_yaw_cmd,
        declare_client_hub_url_cmd,
        declare_controller_host_cmd,
        declare_controller_port_cmd,
        declare_robot_id_cmd,
        declare_send_frequency_cmd,
        declare_command_poll_frequency_cmd,

        # Launch components in sequence:
        # 1. Main SLAM navigation system
        slam_nav_cam_launch,
        # 2. Single robot system integration (delayed)
        delayed_robot_server,
    ])