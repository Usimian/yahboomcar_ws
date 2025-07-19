#!/usr/bin/env python3
"""
Mapping Debug Tool for auto_drive package
Monitors the mapping system and provides detailed diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math


class MappingDebug(Node):
    """Debug tool for mapping system"""

    def __init__(self):
        super().__init__('mapping_debug')
        
        # QoS profiles
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Data storage
        self.laser_data = None
        self.pose_data = None
        self.imu_data = None
        self.odom_data = None
        self.map_data = None
        
        # Counters
        self.laser_count = 0
        self.pose_count = 0
        self.map_count = 0
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, self.sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/auto_drive/pose', self.pose_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, self.sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.sensor_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/auto_drive/map', self.map_callback, 10)
        
        # Timer for periodic reports
        self.timer = self.create_timer(3.0, self.print_status)
        
        self.get_logger().info('Mapping Debug Tool Started')

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg
        self.laser_count += 1

    def pose_callback(self, msg):
        """Process pose data"""
        self.pose_data = msg
        self.pose_count += 1

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg

    def map_callback(self, msg):
        """Process map data"""
        self.map_data = msg
        self.map_count += 1

    def print_status(self):
        """Print diagnostic status"""
        self.get_logger().info('=== MAPPING DEBUG STATUS ===')
        
        # Data availability
        self.get_logger().info(f'Laser data: {"✓" if self.laser_data else "✗"} (count: {self.laser_count})')
        self.get_logger().info(f'Pose data: {"✓" if self.pose_data else "✗"} (count: {self.pose_count})')
        self.get_logger().info(f'IMU data: {"✓" if self.imu_data else "✗"}')
        self.get_logger().info(f'Odom data: {"✓" if self.odom_data else "✗"}')
        self.get_logger().info(f'Map data: {"✓" if self.map_data else "✗"} (count: {self.map_count})')
        
        if self.laser_data and self.pose_data:
            # Analyze laser data
            valid_ranges = [r for r in self.laser_data.ranges 
                          if self.laser_data.range_min <= r <= self.laser_data.range_max]
            self.get_logger().info(f'Valid laser ranges: {len(valid_ranges)}/{len(self.laser_data.ranges)}')
            
            if valid_ranges:
                min_range = min(valid_ranges)
                max_range = max(valid_ranges)
                avg_range = sum(valid_ranges) / len(valid_ranges)
                self.get_logger().info(f'Range stats: min={min_range:.2f}m, max={max_range:.2f}m, avg={avg_range:.2f}m')
            
            # Current pose
            pos = self.pose_data.pose.position
            self.get_logger().info(f'Current pose: x={pos.x:.3f}m, y={pos.y:.3f}m')
            
            # Test mapping calculation
            self.test_mapping_calculation()
        
        if self.map_data:
            # Analyze map data
            occupied = sum(1 for cell in self.map_data.data if cell > 50)
            free = sum(1 for cell in self.map_data.data if 0 <= cell <= 25)
            unknown = sum(1 for cell in self.map_data.data if cell == -1)
            
            self.get_logger().info(f'Map cells: occupied={occupied}, free={free}, unknown={unknown}')
            self.get_logger().info(f'Map info: {self.map_data.info.width}x{self.map_data.info.height}, res={self.map_data.info.resolution:.3f}m')
        
        self.get_logger().info('============================')

    def test_mapping_calculation(self):
        """Test the mapping calculation logic"""
        if not self.laser_data or not self.pose_data:
            return
            
        # Map parameters (should match auto_navigator)
        map_resolution = 0.05
        map_width = 400
        map_height = 400
        map_origin_x = -10.0
        map_origin_y = -10.0
        
        # Current pose
        pose_x = self.pose_data.pose.position.x
        pose_y = self.pose_data.pose.position.y
        
        # Convert quaternion to yaw
        quat = self.pose_data.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        pose_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Test a few laser points
        test_points = 0
        valid_obstacles = 0
        
        for i in range(0, len(self.laser_data.ranges), 50):  # Sample every 50th point
            distance = self.laser_data.ranges[i]
            if distance < self.laser_data.range_min or distance > self.laser_data.range_max:
                continue
                
            test_points += 1
            
            # Calculate angle and obstacle position
            angle = self.laser_data.angle_min + i * self.laser_data.angle_increment
            global_angle = angle + pose_theta
            
            obstacle_x = pose_x + distance * math.cos(global_angle)
            obstacle_y = pose_y + distance * math.sin(global_angle)
            
            # Convert to grid coordinates
            grid_x = int((obstacle_x - map_origin_x) / map_resolution)
            grid_y = int((obstacle_y - map_origin_y) / map_resolution)
            
            if 0 <= grid_x < map_width and 0 <= grid_y < map_height:
                valid_obstacles += 1
        
        self.get_logger().info(f'Mapping test: {valid_obstacles}/{test_points} obstacles would be valid')


def main():
    """Main function"""
    rclpy.init()
    
    debug_node = MappingDebug()
    
    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        pass
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 