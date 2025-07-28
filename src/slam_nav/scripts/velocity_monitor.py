#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os

class VelocityMonitor(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)
        print("=== VELOCITY MONITOR ===")
        print("Move the robot with joystick and note the minimum values that make it move:")
        print("Linear (forward/back) | Angular (rotation)")
        print("-" * 40)

    def velocity_callback(self, msg):
        # Clear screen and show current values
        os.system('clear')
        print("=== VELOCITY MONITOR ===")
        print("Move the robot with joystick and note the minimum values that make it move:")
        print("Linear (forward/back) | Angular (rotation)")
        print("-" * 40)
        print(f"Linear:  {msg.linear.x:+7.3f} m/s")
        print(f"Angular: {msg.angular.z:+7.3f} rad/s")
        print("-" * 40)
        print("Press Ctrl+C to stop monitoring")

def main(args=None):
    rclpy.init(args=args)
    monitor = VelocityMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 