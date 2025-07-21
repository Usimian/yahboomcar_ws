#!/usr/bin/env python3
"""
Real-time SLAM mapping status monitor
Monitors map updates and provides feedback on mapping progress
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


class MappingStatusMonitor(Node):
    """Monitor SLAM mapping progress"""

    def __init__(self):
        super().__init__('mapping_status_monitor')
        
        # Data storage
        self.last_map_update = 0
        self.map_cells_occupied = 0
        self.map_cells_free = 0
        self.map_cells_unknown = 0
        self.last_robot_pos = None
        self.total_distance_traveled = 0.0
        self.laser_scan_count = 0
        self.map_update_count = 0
        
        # QoS profiles
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, self.map_qos)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, self.sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.sensor_qos)
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('SLAM Mapping Status Monitor Started')
        self.get_logger().info('Monitoring /map, /scan, and /odom topics...')
        self.start_time = time.time()

    def map_callback(self, msg):
        """Process map updates"""
        self.last_map_update = time.time()
        self.map_update_count += 1
        
        # Count different cell types
        occupied = sum(1 for cell in msg.data if cell > 50)
        free = sum(1 for cell in msg.data if 0 <= cell <= 25)
        unknown = sum(1 for cell in msg.data if cell == -1)
        
        # Only log if there's a significant change
        if (occupied != self.map_cells_occupied or 
            free != self.map_cells_free):
            
            self.map_cells_occupied = occupied
            self.map_cells_free = free
            self.map_cells_unknown = unknown
            
            self.get_logger().info(f'Map Update #{self.map_update_count}:')
            self.get_logger().info(f'  Occupied: {occupied}, Free: {free}, Unknown: {unknown}')
            self.get_logger().info(f'  Map size: {msg.info.width}x{msg.info.height} ({msg.info.resolution}m/cell)')

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_scan_count += 1
        
        # Check for valid laser data
        valid_ranges = [r for r in msg.ranges 
                       if msg.range_min <= r <= msg.range_max]
        
        if self.laser_scan_count % 50 == 0:  # Log every 50 scans
            self.get_logger().info(f'Laser: {len(valid_ranges)}/{len(msg.ranges)} valid points')

    def odom_callback(self, msg):
        """Process odometry data"""
        current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        if self.last_robot_pos is not None:
            # Calculate distance traveled
            dx = current_pos[0] - self.last_robot_pos[0]
            dy = current_pos[1] - self.last_robot_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance_traveled += distance
        
        self.last_robot_pos = current_pos

    def print_status(self):
        """Print current status"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'SLAM STATUS REPORT (Running for {elapsed_time:.1f}s)')
        self.get_logger().info('=' * 60)
        
        # Map status
        if self.last_map_update > 0:
            time_since_update = current_time - self.last_map_update
            self.get_logger().info(f'Map: Last update {time_since_update:.1f}s ago')
            self.get_logger().info(f'     Occupied cells: {self.map_cells_occupied}')
            self.get_logger().info(f'     Free cells: {self.map_cells_free}')
            self.get_logger().info(f'     Total updates: {self.map_update_count}')
        else:
            self.get_logger().warn('Map: No updates received yet!')
        
        # Robot movement
        if self.last_robot_pos is not None:
            self.get_logger().info(f'Robot: Position ({self.last_robot_pos[0]:.2f}, {self.last_robot_pos[1]:.2f})')
            self.get_logger().info(f'       Total distance: {self.total_distance_traveled:.2f}m')
        else:
            self.get_logger().warn('Robot: No position data received!')
        
        # Laser data
        self.get_logger().info(f'Laser: {self.laser_scan_count} scans received')
        
        # Recommendations
        self.get_logger().info('')
        if self.map_cells_occupied + self.map_cells_free < 100:
            self.get_logger().warn('⚠  Very few map cells updated - robot may not be moving enough')
        if self.total_distance_traveled < 0.1:
            self.get_logger().warn('⚠  Robot has barely moved - drive around to build map')
        if self.map_update_count < 2 and elapsed_time > 10:
            self.get_logger().warn('⚠  Map not updating frequently - check SLAM parameters')
        
        if (self.map_cells_occupied > 50 and 
            self.map_cells_free > 200 and 
            self.total_distance_traveled > 1.0):
            self.get_logger().info('✓ Mapping appears to be working well!')


def main(args=None):
    rclpy.init(args=args)
    
    monitor = MappingStatusMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Shutting down mapping monitor...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 