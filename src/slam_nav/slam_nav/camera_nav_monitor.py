#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2

class CameraNavMonitor(Node):
    def __init__(self):
        super().__init__('camera_nav_monitor')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Status tracking
        self.image_count = 0
        self.scan_count = 0
        self.map_received = False
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Camera + Navigation Monitor Started')

    def image_callback(self, msg):
        self.image_count += 1
        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Add status overlay
            height, width = cv_image.shape[:2]
            cv2.putText(cv_image, f'Camera: {self.image_count} frames', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f'Laser: {self.scan_count} scans', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f'Map: {"Yes" if self.map_received else "No"}', 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display the image (comment out if running headless)
            # cv2.imshow('Navigation Camera View', cv_image)
            # cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def scan_callback(self, msg):
        self.scan_count += 1

    def map_callback(self, msg):
        self.map_received = True

    def print_status(self):
        self.get_logger().info(
            f'Status - Camera: {self.image_count} frames, '
            f'Laser: {self.scan_count} scans, '
            f'Map: {"Available" if self.map_received else "Not yet"}'
        )

def main(args=None):
    rclpy.init(args=args)
    monitor = CameraNavMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        # cv2.destroyAllWindows()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 