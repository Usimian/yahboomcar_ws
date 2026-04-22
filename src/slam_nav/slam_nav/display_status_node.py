#!/usr/bin/env python3
"""
Subscribes to /display/status (std_msgs/String) and writes the message
to /tmp/robot_status for the PiOLED boot_display.py to pick up.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

STATUS_FILE = '/tmp/robot_status'


class DisplayStatusNode(Node):
    def __init__(self):
        super().__init__('display_status_node')
        self.create_subscription(String, '/display/status', self.status_callback, 10)
        self.get_logger().info('Display status node started, listening on /display/status')

    def status_callback(self, msg: String):
        text = msg.data.strip() or 'System Ready'
        try:
            with open(STATUS_FILE, 'w') as f:
                f.write(text)
            self.get_logger().info(f'Display status: {text}')
        except Exception as e:
            self.get_logger().error(f'Failed to write status file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DisplayStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
