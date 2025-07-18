#!/usr/bin/env python3
"""
Test script for autonomous navigation
Verifies that the robot is working correctly after the fixes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class AutonomousTest(Node):
    """Test node for autonomous navigation"""

    def __init__(self):
        super().__init__('autonomous_test')
        
        # Test state
        self.cmd_vel_received = False
        self.laser_received = False
        self.joy_state_received = False
        self.autonomous_mode = False
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.joy_sub = self.create_subscription(
            Bool, '/JoyState', self.joy_callback, 10)
        
        # Publisher for enabling autonomous mode
        self.joy_pub = self.create_publisher(Bool, '/JoyState', 10)
        
        self.get_logger().info('Autonomous test node started')
        
        # Run test sequence
        self.run_test_sequence()
    
    def cmd_vel_callback(self, msg):
        """Track velocity commands"""
        self.cmd_vel_received = True
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.get_logger().info(f'✓ Velocity command received: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}')
    
    def laser_callback(self, msg):
        """Track laser data"""
        self.laser_received = True
        if not hasattr(self, 'laser_logged'):
            self.get_logger().info('✓ Laser data received')
            self.laser_logged = True
    
    def joy_callback(self, msg):
        """Track joystick state"""
        self.joy_state_received = True
        self.autonomous_mode = not msg.data
        mode = "AUTONOMOUS" if self.autonomous_mode else "MANUAL"
        self.get_logger().info(f'✓ Mode: {mode}')
    
    def run_test_sequence(self):
        """Run the test sequence"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('AUTONOMOUS NAVIGATION TEST')
        self.get_logger().info('='*50)
        
        # Test 1: Enable autonomous mode
        self.get_logger().info('\nTest 1: Enabling autonomous mode...')
        time.sleep(1)
        msg = Bool()
        msg.data = False  # False = autonomous mode
        self.joy_pub.publish(msg)
        
        # Wait and check results
        time.sleep(3)
        
        # Test 2: Check if sensors are working
        self.get_logger().info('\nTest 2: Checking sensor data...')
        time.sleep(2)
        
        # Test 3: Check if robot is generating commands
        self.get_logger().info('\nTest 3: Checking velocity commands...')
        time.sleep(3)
        
        # Results
        self.print_results()
    
    def print_results(self):
        """Print test results"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('TEST RESULTS')
        self.get_logger().info('='*50)
        
        # Check autonomous mode
        if self.joy_state_received and self.autonomous_mode:
            self.get_logger().info('✓ Autonomous mode: ENABLED')
        else:
            self.get_logger().warn('✗ Autonomous mode: DISABLED or not detected')
        
        # Check laser data
        if self.laser_received:
            self.get_logger().info('✓ Laser data: RECEIVED')
        else:
            self.get_logger().warn('✗ Laser data: NOT RECEIVED')
        
        # Check velocity commands
        if self.cmd_vel_received:
            self.get_logger().info('✓ Velocity commands: GENERATED')
        else:
            self.get_logger().warn('✗ Velocity commands: NOT GENERATED')
        
        # Overall result
        if self.joy_state_received and self.autonomous_mode and self.laser_received and self.cmd_vel_received:
            self.get_logger().info('\n🎉 ALL TESTS PASSED! Robot should be navigating autonomously.')
        else:
            self.get_logger().warn('\n⚠️  SOME TESTS FAILED. Check the issues above.')
        
        self.get_logger().info('\nTo monitor ongoing behavior, run:')
        self.get_logger().info('ros2 run auto_drive debug_monitor')
        self.get_logger().info('='*50)


def main():
    """Main function"""
    rclpy.init()
    
    try:
        test_node = AutonomousTest()
        # Run for 10 seconds
        rclpy.spin_once(test_node, timeout_sec=10.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 