#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Launch robot driver for motor control
    robot_driver = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        name='yahboomcar_driver',
        output='screen'
    )
    
    # Launch joystick to cmd_vel converter
    joy_ctrl = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
        name='yahboom_joy',
        output='screen'
    )
    
    return LaunchDescription([
        joy_node,
        robot_driver,
        joy_ctrl,
    ]) 