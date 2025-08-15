#!/usr/bin/env python3
"""
Gateway Validator Node - Ensures Single Gateway Compliance
Monitors all command topics and /cmd_vel to detect violations
Publishes violation reports and statistics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robot_msgs.msg import RobotCommand
import json
import time
from datetime import datetime


class GatewayValidatorNode(Node):
    def __init__(self):
        super().__init__('gateway_validator_node')
        
        # Robot identification
        self.robot_id = "yahboomcar_x3_01"
        
        # Violation tracking
        self.cmd_vel_publishers = set()
        self.violation_count = 0
        self.command_count = 0
        self.last_violation_time = 0
        
        # Publishers for violation reports
        self.setup_publishers()
        
        # Subscribers to monitor command flow
        self.setup_subscribers()
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.publish_statistics)  # Every 10 seconds
        
        self.get_logger().info('🛡️  Gateway Validator Node started')
        self.get_logger().info('👀 Monitoring for single gateway compliance')
        self.get_logger().info('⚠️  Will detect direct /cmd_vel publishing violations')
        
    def setup_publishers(self):
        """Setup publishers for violation reports and statistics"""
        
        # Gateway violation reports (documented topic)
        self.violation_pub = self.create_publisher(
            String, '/system/gateway_violations', 10)
            
        # Command statistics (documented topic)
        self.stats_pub = self.create_publisher(
            String, '/system/command_statistics', 10)
            
        self.get_logger().info('📡 Violation monitoring publishers initialized')
        
    def setup_subscribers(self):
        """Setup subscribers to monitor command flow"""
        
        # Monitor direct /cmd_vel publishing (VIOLATION DETECTION)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_violation_callback, 10)
            
        # Monitor legitimate command flow through robot topics
        self.command_sub = self.create_subscription(
            RobotCommand, '/robot/commands', 
            self.legitimate_command_callback, 10)
            
        self.get_logger().info('👀 Command monitoring subscribers initialized')
        
    def cmd_vel_violation_callback(self, msg):
        """
        Monitor /cmd_vel for violations
        NOTE: This should ONLY receive messages from the gateway service
        Any other source is a violation of the single gateway architecture
        """
        # Get publisher information (this is challenging in ROS2, so we'll use timing analysis)
        current_time = time.time()
        
        # Check if this is likely from our gateway (recent legitimate command)
        time_since_last_command = current_time - self.last_legitimate_command_time if hasattr(self, 'last_legitimate_command_time') else float('inf')
        
        # If /cmd_vel is published without a recent legitimate command, it's likely a violation
        if time_since_last_command > 2.0:  # 2 second tolerance
            self.violation_count += 1
            self.last_violation_time = current_time
            
            violation_data = {
                'timestamp': datetime.now().isoformat(),
                'violation_type': 'direct_cmd_vel_publishing',
                'message': 'Direct /cmd_vel publishing detected - violates single gateway architecture',
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z,
                'violation_count': self.violation_count
            }
            
            # Publish violation report
            violation_msg = String()
            violation_msg.data = json.dumps(violation_data)
            self.violation_pub.publish(violation_msg)
            
            # Log violation
            self.get_logger().warn(
                f'🚨 VIOLATION DETECTED: Direct /cmd_vel publishing '
                f'(linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f})'
            )
            self.get_logger().warn(
                f'   Time since last legitimate command: {time_since_last_command:.1f}s'
            )
            self.get_logger().warn(
                f'   Total violations: {self.violation_count}'
            )
            
        # Always log /cmd_vel activity for monitoring
        self.get_logger().debug(f'📊 /cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
        
    def legitimate_command_callback(self, msg):
        """Track legitimate commands through the documented topic"""
        self.command_count += 1
        self.last_legitimate_command_time = time.time()
        
        self.get_logger().info(
            f'✅ Legitimate command: {msg.command_type} from {msg.source}'
        )
        
    def publish_statistics(self):
        """Publish command flow statistics"""
        current_time = time.time()
        
        stats_data = {
            'timestamp': datetime.now().isoformat(),
            'robot_id': self.robot_id,
            'total_commands': self.command_count,
            'total_violations': self.violation_count,
            'last_violation_time': datetime.fromtimestamp(self.last_violation_time).isoformat() if self.last_violation_time > 0 else None,
            'compliance_rate': ((self.command_count - self.violation_count) / max(self.command_count, 1)) * 100,
            'monitoring_status': 'active'
        }
        
        # Publish statistics
        stats_msg = String()
        stats_msg.data = json.dumps(stats_data)
        self.stats_pub.publish(stats_msg)
        
        # Log statistics
        if self.violation_count > 0:
            self.get_logger().info(
                f'📊 Gateway Statistics: {self.command_count} commands, '
                f'{self.violation_count} violations ({stats_data["compliance_rate"]:.1f}% compliance)'
            )
        else:
            self.get_logger().info(
                f'📊 Gateway Statistics: {self.command_count} commands, '
                f'✅ 100% compliance (no violations)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GatewayValidatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Gateway Validator Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
