from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch argument for robot type
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='X3',
        description='Robot type: X1 or X3'
    )
    
    # EKF node configuration
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',  # Use base_footprint for proper REP-105 compliance
            'world_frame': 'odom',
            'frequency': 10.0,  # REDUCED to 10 Hz to match lidar frequency and reduce timing conflicts
            'sensor_timeout': 0.5,  # INCREASED timeout for more robustness
            'two_d_mode': True,
            'transform_time_offset': 0.0,
            'transform_timeout': 0.0,
            'print_diagnostics': True,
            'debug': False,
            'debug_out_file': '/path/to/debug/file.txt',
            'permit_corrected_publication': False,
            'publish_acceleration': False,
            'publish_tf': False,  # Disable EKF TF publishing, use base_node instead
            'map_frame': 'map',
            
            # Odometry configuration - wheel encoders for position and velocities
            'odom0': 'odom_raw',
            'odom0_config': [True,  True,  False,  # Use x, y position from odometry
                           False, False, False,  # Don't use yaw pose from odometry (use IMU)
                           True,  True,  False,  # Use linear velocities from odometry
                           False, False, True,   # Use yaw angular velocity from odometry
                           False, False, False],
            'odom0_queue_size': 20,  # Increase queue size for better buffering
            'odom0_nodelay': False,
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_pose_rejection_threshold': 5.0,
            'odom0_twist_rejection_threshold': 1.0,
            
            # IMU configuration - FIXED to work properly with odometry sensor fusion
            # Use IMU for orientation stability, odometry for angular velocity
            'imu0': 'imu/data_raw',
            'imu0_config': [False, False, False,
                           False, False, True,   # Use yaw orientation from IMU for stability
                           False, False, False,
                           False, False, False,  # Don't use IMU angular velocity (use odometry instead)
                           False, False, False], # Don't use accelerations to prevent drift
            'imu0_nodelay': False,
            'imu0_differential': False,
            'imu0_relative': False,  # Use absolute measurements for IMU
            'imu0_queue_size': 20,
            'imu0_pose_rejection_threshold': 2.0,  # More permissive threshold
            'imu0_twist_rejection_threshold': 1.0,
            'imu0_linear_acceleration_rejection_threshold': 1.0,
            'imu0_remove_gravitational_acceleration': True,
            
            # Process noise covariance matrix - tuned for X3 robot
            'process_noise_covariance': [0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                                       0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015],
            
            # Initial estimate covariance - robot starts at known position
            'initial_estimate_covariance': [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    1e-9, 0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    1e-9, 0,    0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    1e-9, 0,    0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    1e-9, 0,     0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-9,  0,     0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1e-9,  0,     0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1e-9,  0,    0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1e-9, 0,    0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1e-9, 0,
                                          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1e-9]
        }],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        ekf_node,
    ]) 