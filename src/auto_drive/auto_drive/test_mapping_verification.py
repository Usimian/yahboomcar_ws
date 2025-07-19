#!/usr/bin/env python3
"""
Test script to verify mapping functionality in auto_drive package
This script checks that the map is being created and updated correctly
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MappingVerificationTest(Node):
    """Test node to verify mapping functionality"""

    def __init__(self):
        super().__init__('mapping_test')
        
        # Test results
        self.map_received = False
        self.map_updates = 0
        self.last_map_size = 0
        self.laser_received = False
        self.pose_received = False
        
        # QoS profiles for different message types
        self.sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/auto_drive/map',
            self.map_callback,
            self.map_qos
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            self.sensor_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/auto_drive/pose',
            self.pose_callback,
            10
        )
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_tests)
        self.test_count = 0
        
        self.get_logger().info('Mapping verification test started')
        self.get_logger().info('Waiting for auto_drive node to publish map data...')

    def map_callback(self, msg):
        """Callback for map updates"""
        self.map_received = True
        self.map_updates += 1
        
        # Count occupied cells
        occupied_cells = sum(1 for cell in msg.data if cell > 50)  # > 50% occupied
        free_cells = sum(1 for cell in msg.data if 0 <= cell <= 25)  # < 25% occupied
        unknown_cells = sum(1 for cell in msg.data if cell == -1)  # Unknown
        
        current_map_size = occupied_cells + free_cells
        
        if current_map_size != self.last_map_size:
            self.get_logger().info(f'Map update #{self.map_updates}:')
            self.get_logger().info(f'  - Occupied cells: {occupied_cells}')
            self.get_logger().info(f'  - Free cells: {free_cells}')
            self.get_logger().info(f'  - Unknown cells: {unknown_cells}')
            self.get_logger().info(f'  - Resolution: {msg.info.resolution}m/cell')
            self.get_logger().info(f'  - Map size: {msg.info.width}x{msg.info.height}')
            self.get_logger().info(f'  - Frame ID: {msg.header.frame_id}')
            
            self.last_map_size = current_map_size

    def laser_callback(self, msg):
        """Callback for laser scan data"""
        if not self.laser_received:
            self.laser_received = True
            self.get_logger().info('✓ Laser scan data received')
            self.get_logger().info(f'  - Range: {msg.range_min:.2f}m to {msg.range_max:.2f}m')
            self.get_logger().info(f'  - Angle range: {msg.angle_min:.2f} to {msg.angle_max:.2f} rad')
            self.get_logger().info(f'  - Number of points: {len(msg.ranges)}')

    def pose_callback(self, msg):
        """Callback for robot pose"""
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info('✓ Robot pose data received')
            self.get_logger().info(f'  - Position: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
            self.get_logger().info(f'  - Frame ID: {msg.header.frame_id}')

    def run_tests(self):
        """Run periodic tests"""
        self.test_count += 1
        
        if self.test_count == 1:
            self.get_logger().info('=== MAPPING VERIFICATION TEST RESULTS ===')
            
        if self.test_count <= 10:  # Run for 20 seconds
            self.get_logger().info(f'Test cycle {self.test_count}/10:')
            self.get_logger().info(f'  - Map received: {"✓" if self.map_received else "✗"}')
            self.get_logger().info(f'  - Map updates: {self.map_updates}')
            self.get_logger().info(f'  - Laser data: {"✓" if self.laser_received else "✗"}')
            self.get_logger().info(f'  - Pose data: {"✓" if self.pose_received else "✗"}')
            
            if self.test_count == 10:
                self.final_report()
        else:
            self.destroy_timer(self.test_timer)

    def final_report(self):
        """Generate final test report"""
        self.get_logger().info('')
        self.get_logger().info('=== FINAL TEST REPORT ===')
        
        # Overall status
        mapping_working = (self.map_received and self.map_updates >= 2 and 
                          self.laser_received and self.pose_received)
        
        self.get_logger().info(f'Mapping Status: {"✓ WORKING" if mapping_working else "✗ ISSUES DETECTED"}')
        self.get_logger().info(f'Total map updates: {self.map_updates}')
        
        # Recommendations
        self.get_logger().info('')
        self.get_logger().info('=== RECOMMENDATIONS ===')
        
        if not self.map_received:
            self.get_logger().warn('⚠ Map not received - check if auto_navigator node is running')
        
        if not self.laser_received:
            self.get_logger().warn('⚠ Laser data not received - check lidar connection')
            
        if not self.pose_received:
            self.get_logger().warn('⚠ Pose data not received - check odometry/IMU')
            
        if self.map_updates < 2:
            self.get_logger().warn('⚠ Map not updating frequently - robot may not be moving')
        
        if mapping_working:
            self.get_logger().info('✓ Mapping system is functioning correctly!')
            self.get_logger().info('✓ Robot should be building a map as it moves')


def main():
    """Main function"""
    rclpy.init()
    
    test_node = MappingVerificationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 