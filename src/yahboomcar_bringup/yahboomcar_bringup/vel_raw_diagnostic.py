#!/usr/bin/env python3

"""
Velocity Raw Diagnostic Tool

This tool helps calibrate the vel_raw feedback by monitoring actual movement.

Usage:
1. Run this tool
2. Manually push/roll the robot a known distance (e.g. 0.5m)
3. The tool will integrate the vel_raw values to calculate reported distance
4. Compare reported vs actual distance to determine scaling factor

The tool will also test commanded movements to verify calibration.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class VelRawDiagnostic(Node):
    def __init__(self):
        super().__init__('vel_raw_diagnostic')
        
        # Subscribers
        self.vel_raw_sub = self.create_subscription(Twist, '/vel_raw', self.vel_raw_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        
        # Publisher for commanded movements
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State tracking
        self.reset_tracking()
        
        self.get_logger().info("Velocity Raw Diagnostic Tool Started")
        self.get_logger().info("Commands:")
        self.get_logger().info("  'reset' - Reset distance tracking")
        self.get_logger().info("  'status' - Show current readings")
        self.get_logger().info("  'move X' - Command robot to move X meters")
        self.get_logger().info("  'manual' - Start manual movement tracking")
        self.get_logger().info("  'stop' - Stop any movement")
        self.get_logger().info("  'quit' - Exit program")
        
    def reset_tracking(self):
        """Reset all tracking variables"""
        self.start_time = time.time()
        self.integrated_distance = 0.0
        self.last_vel_time = time.time()
        self.current_odom = None
        self.start_odom = None
        self.is_tracking = False
        
    def vel_raw_callback(self, msg):
        """Integrate vel_raw to calculate distance"""
        if not self.is_tracking:
            return
            
        current_time = time.time()
        dt = current_time - self.last_vel_time
        
        if dt > 0 and dt < 1.0:  # Sanity check
            # Integrate linear velocity (assuming forward movement)
            velocity = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
            distance_increment = velocity * dt
            self.integrated_distance += distance_increment
            
            # Show live updates every 0.5 seconds
            if current_time - self.start_time > 0.5:
                self.get_logger().info(f"Integrated distance: {self.integrated_distance:.3f}m (vel: {velocity:.3f}m/s)")
                self.start_time = current_time
        
        self.last_vel_time = current_time
        
    def odom_callback(self, msg):
        """Track odometry-based distance"""
        self.current_odom = msg.pose.pose.position
        
        if self.is_tracking and self.start_odom is None:
            self.start_odom = msg.pose.pose.position
            
    def get_odom_distance(self):
        """Calculate distance from odometry"""
        if self.start_odom is None or self.current_odom is None:
            return 0.0
            
        dx = self.current_odom.x - self.start_odom.x
        dy = self.current_odom.y - self.start_odom.y
        return math.sqrt(dx*dx + dy*dy)
        
    def start_manual_tracking(self):
        """Start tracking for manual movement"""
        self.reset_tracking()
        self.is_tracking = True
        self.get_logger().info("Manual tracking started - push the robot now!")
        self.get_logger().info("Type 'status' to see current readings, 'stop' when done")
        
    def stop_tracking(self):
        """Stop tracking and show results"""
        if not self.is_tracking:
            self.get_logger().info("No tracking in progress")
            return
            
        self.is_tracking = False
        odom_distance = self.get_odom_distance()
        
        self.get_logger().info("=== TRACKING RESULTS ===")
        self.get_logger().info(f"Integrated vel_raw distance: {self.integrated_distance:.3f}m")
        self.get_logger().info(f"Odometry distance:           {odom_distance:.3f}m")
        self.get_logger().info("Now measure the ACTUAL distance the robot moved and compare!")
        
    def command_move(self, distance):
        """Command robot to move a specific distance"""
        self.get_logger().info(f"Commanding robot to move {distance:.3f}m...")
        
        # Start tracking
        self.reset_tracking()
        self.is_tracking = True
        
        # Send movement command
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # 0.2 m/s
        
        start_time = time.time()
        while self.is_tracking and (time.time() - start_time < distance/0.2 + 2.0):  # Add 2s buffer
            self.cmd_vel_pub.publish(move_cmd)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
            
            # Check if we've reached target distance (using odometry)
            odom_dist = self.get_odom_distance()
            if odom_dist >= abs(distance):
                break
                
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.stop_tracking()
        
    def show_status(self):
        """Show current status"""
        if not self.is_tracking:
            self.get_logger().info("Not currently tracking movement")
            return
            
        odom_distance = self.get_odom_distance()
        elapsed = time.time() - self.start_time
        
        self.get_logger().info("=== CURRENT STATUS ===")
        self.get_logger().info(f"Elapsed time:                {elapsed:.1f}s")
        self.get_logger().info(f"Integrated vel_raw distance: {self.integrated_distance:.3f}m")
        self.get_logger().info(f"Odometry distance:           {odom_distance:.3f}m")


def main():
    rclpy.init()
    
    diagnostic = VelRawDiagnostic()
    
    # Run interactive loop
    try:
        while rclpy.ok():
            try:
                command = input("\nDiagnostic> ").strip().lower()
                
                if command == 'quit' or command == 'q':
                    break
                elif command == 'reset':
                    diagnostic.reset_tracking()
                    print("✅ Tracking reset")
                elif command == 'status':
                    diagnostic.show_status()
                elif command == 'manual':
                    diagnostic.start_manual_tracking()
                elif command == 'stop':
                    diagnostic.stop_tracking()
                elif command.startswith('move '):
                    try:
                        distance = float(command.split()[1])
                        diagnostic.command_move(distance)
                    except (ValueError, IndexError):
                        print("❌ Usage: move <distance>")
                else:
                    print("❌ Unknown command")
                    
                # Process ROS callbacks
                rclpy.spin_once(diagnostic, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
                
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
