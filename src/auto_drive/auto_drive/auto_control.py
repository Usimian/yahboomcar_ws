#!/usr/bin/env python3
"""
Autonomous Control Script
Simple command-line interface to control autonomous driving
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import argparse


class AutonomousControl(Node):
    """Simple control interface for autonomous driving"""

    def __init__(self):
        super().__init__('auto_control')

        # Publishers
        self.joy_state_pub = self.create_publisher(Bool, '/JoyState', 10)

        self.get_logger().info('Autonomous Control initialized')

    def enable_autonomous(self):
        """Enable autonomous driving mode"""
        msg = Bool()
        msg.data = False  # JoyState False = autonomous mode
        self.joy_state_pub.publish(msg)
        self.get_logger().info('Autonomous mode ENABLED')

    def disable_autonomous(self):
        """Disable autonomous driving mode"""
        msg = Bool()
        msg.data = True  # JoyState True = manual mode
        self.joy_state_pub.publish(msg)
        self.get_logger().info('Autonomous mode DISABLED')


def main():
    """Main function with command line interface"""
    parser = argparse.ArgumentParser(description='Control autonomous driving')
    parser.add_argument('command', choices=['enable', 'disable', 'status'],
                        help='Command to execute')

    args = parser.parse_args()

    rclpy.init()

    try:
        control = AutonomousControl()

        if args.command == 'enable':
            control.enable_autonomous()
        elif args.command == 'disable':
            control.disable_autonomous()
        elif args.command == 'status':
            control.get_logger().info('Use "ros2 topic echo /JoyState" to check current status')

        # Keep node alive briefly to ensure message is sent
        rclpy.spin_once(control, timeout_sec=1.0)

    except KeyboardInterrupt:
        pass
    finally:
        if 'control' in locals():
            control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
