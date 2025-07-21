#!/usr/bin/env python3

"""
Robot bringup launch file for slam_nav package
Brings up the complete robot system including:
- Robot hardware drivers
- Base node for odometry and transforms  
- IMU filtering
- EKF for sensor fusion
- S2 lidar
- Robot state publisher
- Joint state publisher
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Print robot configuration
    if not os.environ.get("PRINTED_SLAM_NAV"):
        os.environ["PRINTED_SLAM_NAV"] = "1"
        print("---------------------SLAM NAV: X3 with S2 lidar---------------------")
    
    # Get package paths
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X3_simple.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/yahboomcar.rviz'
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='false', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )
    
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', 
        default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', 
        default_value='false',
        description='Whether to start RViz'
    )
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # === ROBOT NODES ===
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Joint state publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # === HARDWARE DRIVERS ===
    
    # Main robot driver
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        name='yahboomcar_driver',
        output='screen',
        parameters=[{
            'car_type': 'X3',
            'imu_link': 'imu_link',
            'Prefix': '',
            'xlinear_limit': 1.0,
            'ylinear_limit': 1.0,
            'angular_limit': 5.0
        }]
    )
    
    # Base node for odometry and transforms
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[{
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }]
    )
    
    # === IMU AND SENSOR FUSION ===
    
    # IMU filter configuration
    imu_filter_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    )
    
    # IMU filter node - TEMPORARILY DISABLED for debugging
    # imu_filter_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter',
    #     output='screen',
    #     parameters=[imu_filter_config]
    # )
    
    # EKF for sensor fusion
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/ekf_x1_x3_launch.py'
        ])
    )
    
    # === LIDAR ===
    
    # S2 lidar node
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_s2_launch.py'
        ])
    )
    
    # Static transform from base_link to laser frame
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'base_link', 'laser']
    )
    
    # === CONTROL ===
    
    # Joystick control node
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
        name='yahboom_joy',
        output='screen'
    )
    
    # Joy node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        use_rviz_arg,
        
        # Robot description and visualization
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        
        # Hardware drivers
        driver_node,
        base_node,
        
        # IMU and sensor fusion - IMU filter temporarily disabled
        # imu_filter_node,
        ekf_node,
        
        # Lidar
        lidar_node,
        laser_tf_node,
        
        # Control
        yahboom_joy_node,
        joy_node
    ]) 