#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a timer for publishing commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Control variables
        self.current_twist = Twist()
        self.movement_start_time = None
        self.movement_duration = 0.0
        self.is_moving = False
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        self.get_logger().info("Starting calibration test sequence...")
        
    def timer_callback(self):
        """Timer callback to publish velocity commands"""
        if self.is_moving:
            # Check if movement duration has elapsed
            if time.time() - self.movement_start_time >= self.movement_duration:
                self.stop_robot()
            else:
                self.cmd_vel_pub.publish(self.current_twist)
        
    def send_velocity_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=1.0):
        """Send velocity command for specified duration"""
        self.current_twist.linear.x = linear_x
        self.current_twist.linear.y = linear_y
        self.current_twist.angular.z = angular_z
        
        self.movement_start_time = time.time()
        self.movement_duration = duration
        self.is_moving = True
        
        # Wait for the movement to complete
        while self.is_moving:
            rclpy.spin_once(self, timeout_sec=0.1)
        
    def stop_robot(self):
        """Send zero velocity to stop the robot"""
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.current_twist)
        self.is_moving = False
        
    def run_test_sequence(self):
        """Run the complete calibration test sequence"""
        
        # Test 1: Forward 0.5m at 0.2 m/s (takes 2.5 seconds)
        self.get_logger().info("Test 1: Moving forward 0.5m...")
        self.send_velocity_command(linear_x=0.2, duration=2.5)
        time.sleep(1.0)  # 1 second pause
        
        # Test 2: Backward 0.5m at 0.2 m/s (takes 2.5 seconds)
        self.get_logger().info("Test 2: Moving backward 0.5m...")
        self.send_velocity_command(linear_x=-0.2, duration=2.5)
        time.sleep(1.0)  # 1 second pause
        
        # Test 3: Turn CW 90° at 0.5 rad/s (π/2 rad takes π seconds ≈ 3.14 seconds)
        self.get_logger().info("Test 3: Turning clockwise 90°...")
        self.send_velocity_command(angular_z=-0.5, duration=math.pi)  # CW is negative
        time.sleep(1.0)  # 1 second pause
        
        # Test 4: Turn CCW 90° at 0.5 rad/s (π/2 rad takes π seconds ≈ 3.14 seconds)
        self.get_logger().info("Test 4: Turning counter-clockwise 90°...")
        self.send_velocity_command(angular_z=0.5, duration=math.pi)  # CCW is positive
        time.sleep(1.0)  # 1 second pause
        
        self.get_logger().info("Calibration test sequence completed!")
        self.get_logger().info("Robot should be back at approximately the starting position and orientation.")

def main():
    rclpy.init()
    
    test_node = CalibrationTest()
    
    try:
        # Run the test sequence
        test_node.run_test_sequence()
        
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
        
    finally:
        # Make sure robot is stopped
        test_node.stop_robot()
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
