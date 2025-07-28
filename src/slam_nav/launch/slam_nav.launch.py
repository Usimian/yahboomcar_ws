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
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
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
    
    # Initial pose parameters
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')


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

    # Initial pose arguments
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose X coordinate')

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', 
        default_value='0.0',
        description='Initial pose Y coordinate')

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0', 
        description='Initial pose yaw angle (radians)')


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

    # Create Nav2 container first for proper composition
    nav2_container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen'
    )

    # Navigation launch - use composition with fallback option
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'params_file': params_file,
                         'use_composition': 'False',  # Temporarily disable for stability
                         'use_respawn': 'True',  # Enable respawn for crash recovery
                         'container_name': 'nav2_container'}.items())



    # Implement proper startup sequencing as recommended for SLAM + Nav2:
    # 1. Robot platform first (immediate)
    # 2. Nav2 stack second (with delay)  
    # 3. SLAM Toolbox third (with longer delay)
    
    # Group Nav2 launch with delayed start (shorter delay since container is pre-created)
    delayed_navigation_group = GroupAction([
        TimerAction(
            period=3.0,  # Reduced to 3 seconds since container exists
            actions=[navigation_launch_cmd]
        )
    ])
    
    # Group SLAM launch with delayed start  
    delayed_slam_group = GroupAction([
        TimerAction(
            period=8.0,  # 8 second delay (reduced since Nav2 starts at 3s)
            actions=[slam_launch_cmd]
        )
    ])

    # Initial pose publisher node - starts after SLAM is ready
    initial_pose_publisher = Node(
        package='slam_nav',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y, 
            'initial_pose_yaw': initial_pose_yaw,
        }],
        output='screen'
    )

    # Delayed initial pose publication
    delayed_initial_pose = TimerAction(
        period=12.0,  # After SLAM is fully started
        actions=[initial_pose_publisher]
    )

    # Return launch description with proper sequencing
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
        declare_initial_pose_x_cmd,
        declare_initial_pose_y_cmd,
        declare_initial_pose_yaw_cmd,

        # Launch components in recommended sequence:
        # 1. Robot platform + Nav2 container (immediate)
        robot_bringup_cmd,
        nav2_container,
        # 2. Nav2 stack (3s delay - container ready)
        delayed_navigation_group,
        # 3. SLAM Toolbox (8s delay) 
        delayed_slam_group,
        # 4. Initial pose publisher (temporarily disabled for debugging)
        # delayed_initial_pose
    ]) 