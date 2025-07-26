#!/usr/bin/env python3

"""
SLAM Navigation Launch File for slam_nav package
Integrates slam_toolbox with Nav2 navigation stack for persistent mapping
Brings up:
- Complete robot system (robot_bringup.launch.py)
- SLAM Toolbox for persistent mapping
- Nav2 navigation stack

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                          IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
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

    declare_pub_odom_tf_cmd = DeclareLaunchArgument(
        'pub_odom_tf',
        default_value='true',  # FIXED: Default to true since base_node has correct rotation data
        description='Whether to publish odom->base_footprint transform')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_nav_dir, 'config', 'slam_toolbox_config.yaml'),
        description='Full path to the ROS2 parameters file to use for slam_toolbox')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')


    # Variables for robot bringup

    pub_odom_tf = LaunchConfiguration('pub_odom_tf')

    # === STARTUP SEQUENCING ===
    # Start all components together - no artificial delays needed
    # Robot transforms from URDF are static and available immediately

    # 1. Robot bringup (base_node, robot_state_publisher, static transforms)
    robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_nav'), 'launch', 'robot_bringup.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                         'pub_odom_tf': pub_odom_tf}.items()
    )

    # 2. SLAM Toolbox - start immediately with robot bringup
    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params_file,
                         'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam)
    )

    # Navigation launch
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'params_file': params_file,
                         'use_composition': use_composition,
                         'use_respawn': use_respawn,
                         'container_name': 'nav2_container'}.items())



    # Return launch description directly
    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,
        
        # Declare the launch options
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_pub_odom_tf_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,

        # Add the actions to launch all components
        robot_bringup_cmd,
        slam_launch_cmd,
        navigation_launch_cmd
    ]) 