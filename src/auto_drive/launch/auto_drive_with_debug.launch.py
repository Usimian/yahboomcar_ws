#!/usr/bin/env python3
"""
Launch file for autonomous driving system with debug monitoring
Starts the robot hardware, autonomous navigation, and debug monitor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for autonomous driving with debug monitoring"""
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz for visualization'
    )
    
    enable_autonomous_arg = DeclareLaunchArgument(
        'enable_autonomous',
        default_value='true',
        description='Enable autonomous driving on startup'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.3',
        description='Maximum linear speed (m/s)'
    )
    
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.8',
        description='Safety distance for obstacle avoidance (m)'
    )
    
    debug_update_rate_arg = DeclareLaunchArgument(
        'debug_update_rate',
        default_value='2.0',
        description='Debug monitor update rate in seconds'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='Enable debug monitor'
    )
    
    # Include robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_x3_with_s2_lidar.launch.py'
        ]),
        launch_arguments={
            'use_rviz': 'false',  # We'll start our own RViz
            'pub_odom_tf': 'true'
        }.items()
    )
    
    # Auto navigator node
    auto_navigator = Node(
        package='auto_drive',
        executable='auto_navigator',
        name='auto_navigator',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'max_angular_speed': 0.5,
            'safety_distance': LaunchConfiguration('safety_distance'),
            'emergency_distance': 0.4,
            'enable_autonomous': LaunchConfiguration('enable_autonomous')
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/scan', '/scan'),
            ('/imu/data', '/imu/data'),
            ('/odom', '/odom'),
            ('/JoyState', '/JoyState')
        ]
    )
    
    # Debug monitor node (start after a delay to ensure other nodes are ready)
    debug_monitor = TimerAction(
        period=3.0,  # Wait 3 seconds before starting debug monitor
        actions=[
            Node(
                package='auto_drive',
                executable='debug_monitor',
                name='debug_monitor',
                output='screen',
                parameters=[{
                    'update_rate': LaunchConfiguration('debug_update_rate')
                }],
                condition=IfCondition(LaunchConfiguration('enable_debug'))
            )
        ]
    )
    
    # RViz with custom config
    rviz_config = os.path.join(
        get_package_share_directory('auto_drive'),
        'config',
                        'auto_drive.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Map server (optional - for saving/loading maps)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(
                get_package_share_directory('auto_drive'),
                'maps',
                'saved_map.yaml'
            ),
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        enable_autonomous_arg,
        max_speed_arg,
        safety_distance_arg,
        debug_update_rate_arg,
        enable_debug_arg,
        
        # Nodes
        robot_bringup,
        auto_navigator,
        debug_monitor,  # Started with delay
        rviz_node,
        # map_server,  # Uncomment if you want to use saved maps
    ]) 