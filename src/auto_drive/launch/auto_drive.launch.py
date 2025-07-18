#!/usr/bin/env python3
"""
Launch file for autonomous driving system
Starts the robot hardware and autonomous navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for autonomous driving"""
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz for visualization'
    )
    
    enable_auto_arg = DeclareLaunchArgument(
        'enable_auto',
        default_value='true',
        description='Enable auto driving on startup'
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
        enable_auto_arg,
        max_speed_arg,
        safety_distance_arg,
        
        # Nodes
        robot_bringup,
        auto_navigator,
        rviz_node,
        # map_server,  # Uncomment if you want to use saved maps
    ]) 