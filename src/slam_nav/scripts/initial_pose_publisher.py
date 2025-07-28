#!/usr/bin/env python3

"""
Initial Pose Publisher Node
Automatically publishes initial pose for robot localization at startup
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Declare parameters
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0) 
        self.declare_parameter('initial_pose_yaw', 0.0)
        
        # Get parameters
        self.initial_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_pose_yaw').get_parameter_value().double_value
        
        # Create publisher
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait a moment then publish initial pose
        self.timer = self.create_timer(2.0, self.publisher_initial_pose)
        self.published = False
        
        self.get_logger().info(f'Initial Pose Publisher ready: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}')

    def publisher_initial_pose(self):
        if self.published:
            return
            
        # Create initial pose message
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        
        # Set position
        initial_pose_msg.pose.pose.position.x = self.initial_x
        initial_pose_msg.pose.pose.position.y = self.initial_y
        initial_pose_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, self.initial_yaw)
        initial_pose_msg.pose.pose.orientation.x = quat[0]
        initial_pose_msg.pose.pose.orientation.y = quat[1]
        initial_pose_msg.pose.pose.orientation.z = quat[2]
        initial_pose_msg.pose.pose.orientation.w = quat[3]
        
        # Set covariance (6x6 matrix, row-major order)
        # Diagonal elements represent uncertainty in x, y, z, roll, pitch, yaw
        covariance = [0.0] * 36
        covariance[0] = 0.25   # x variance
        covariance[7] = 0.25   # y variance  
        covariance[35] = 0.25  # yaw variance
        initial_pose_msg.pose.covariance = covariance
        
        # Publish initial pose
        self.pose_publisher.publish(initial_pose_msg)
        self.get_logger().info(f'Published initial pose: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw:.2f} rad')
        
        self.published = True
        
        # Shutdown after publishing
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    initial_pose_publisher = InitialPosePublisher()
    
    try:
        rclpy.spin(initial_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        initial_pose_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 