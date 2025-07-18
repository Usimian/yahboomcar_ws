#!/usr/bin/env python3
"""
Test script for debug monitor
Publishes test data to verify the debug monitor is working correctly
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import time


class DebugMonitorTest(Node):
    """Test node for debug monitor"""

    def __init__(self):
        super().__init__('debug_monitor_test')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_pub = self.create_publisher(Bool, '/JoyState', 10)
        self.battery_pub = self.create_publisher(Float32, '/voltage', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/auto_drive/pose', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer for publishing test data
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.test_counter = 0
        
        self.get_logger().info('Debug monitor test started')
        self.get_logger().info('Run "ros2 run auto_drive debug_monitor" in another terminal')
    
    def publish_test_data(self):
        """Publish test data for debug monitor"""
        self.test_counter += 1
        
        # Test different scenarios
        if self.test_counter <= 5:
            self.test_forward_movement()
        elif self.test_counter <= 10:
            self.test_sideways_movement()
        elif self.test_counter <= 15:
            self.test_rotation()
        elif self.test_counter <= 20:
            self.test_battery_levels()
        else:
            self.test_counter = 0  # Reset and repeat
    
    def test_forward_movement(self):
        """Test forward movement"""
        self.get_logger().info('Testing forward movement...')
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = 0.5  # Forward
        cmd.linear.y = 0.0  # No sideways
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)
        
        # Publish autonomous mode
        joy_msg = Bool()
        joy_msg.data = False  # Autonomous mode
        self.joy_pub.publish(joy_msg)
        
        # Publish good battery
        battery_msg = Float32()
        battery_msg.data = 11.5  # Good battery
        self.battery_pub.publish(battery_msg)
        
        self.publish_common_data()
    
    def test_sideways_movement(self):
        """Test sideways movement"""
        self.get_logger().info('Testing sideways movement...')
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = 0.2  # Slight forward
        cmd.linear.y = 0.3  # Sideways (mecanum wheels)
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)
        
        # Publish autonomous mode
        joy_msg = Bool()
        joy_msg.data = False  # Autonomous mode
        self.joy_pub.publish(joy_msg)
        
        # Publish good battery
        battery_msg = Float32()
        battery_msg.data = 11.8  # Good battery
        self.battery_pub.publish(battery_msg)
        
        self.publish_common_data()
    
    def test_rotation(self):
        """Test rotation"""
        self.get_logger().info('Testing rotation...')
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = 0.0  # No forward
        cmd.linear.y = 0.0  # No sideways
        cmd.angular.z = 0.5  # Rotate
        self.cmd_vel_pub.publish(cmd)
        
        # Publish manual mode
        joy_msg = Bool()
        joy_msg.data = True  # Manual mode
        self.joy_pub.publish(joy_msg)
        
        # Publish good battery
        battery_msg = Float32()
        battery_msg.data = 12.2  # High battery
        self.battery_pub.publish(battery_msg)
        
        self.publish_common_data()
    
    def test_battery_levels(self):
        """Test different battery levels"""
        battery_levels = [13.0, 11.5, 10.5, 9.5]  # HIGH, GOOD, LOW, CRITICAL
        level_names = ["HIGH", "GOOD", "LOW", "CRITICAL"]
        
        battery_idx = (self.test_counter - 16) % len(battery_levels)
        battery_voltage = battery_levels[battery_idx]
        
        self.get_logger().info(f'Testing {level_names[battery_idx]} battery level: {battery_voltage}V')
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = 0.1  # Slow forward
        cmd.linear.y = 0.0  # No sideways
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)
        
        # Publish autonomous mode
        joy_msg = Bool()
        joy_msg.data = False  # Autonomous mode
        self.joy_pub.publish(joy_msg)
        
        # Publish test battery level
        battery_msg = Float32()
        battery_msg.data = battery_voltage
        self.battery_pub.publish(battery_msg)
        
        self.publish_common_data()
    
    def publish_common_data(self):
        """Publish common test data"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = 1.5
        pose_msg.pose.position.y = 2.3
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)
        
        # Publish laser scan
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser'
        # S2 lidar actual coverage: -120° to +120° (240° total)
        laser_msg.angle_min = -math.pi * 2/3  # -120°
        laser_msg.angle_max = math.pi * 2/3   # +120°
        laser_msg.angle_increment = math.pi * 4/3 / 240.0  # 240 points for 240° coverage
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        
        # Create test ranges (240 points for 240° coverage)
        ranges = []
        for i in range(240):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            angle_deg = math.degrees(angle)
            
            # Simulate obstacles at different angles within the 240° coverage
            if -30 <= angle_deg <= 30:  # Front
                ranges.append(2.5)
            elif 60 <= angle_deg <= 120:  # Left
                ranges.append(1.8)
            elif -120 <= angle_deg <= -60:  # Right
                ranges.append(3.2)
            else:  # Other areas within coverage
                ranges.append(5.0)
        
        laser_msg.ranges = ranges
        self.laser_pub.publish(laser_msg)


def main():
    """Main function"""
    rclpy.init()
    
    try:
        test_node = DebugMonitorTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 