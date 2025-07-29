#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class RgbToOccupancy(Node):
    def __init__(self):
        super().__init__('rgb_to_occupancy')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('camera_height', 0.1)  # Height of camera above ground (meters)
        self.declare_parameter('camera_tilt', 0.0)    # Camera tilt angle (radians)
        self.declare_parameter('obstacle_threshold', 50)  # HSV value threshold for obstacles
        self.declare_parameter('max_detection_range', 2.0)  # Max range for obstacle detection
        
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_tilt = self.get_parameter('camera_tilt').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.max_range = self.get_parameter('max_detection_range').value
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/camera/points', 10)
        
        self.processed_image_pub = self.create_publisher(
            Image, '/camera/processed_image', 10)
        
        self.get_logger().info(f'RGB to Occupancy converter started')
        self.get_logger().info(f'Camera height: {self.camera_height}m, Max range: {self.max_range}m')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image for obstacle detection
            obstacles, processed_img = self.detect_obstacles(cv_image)
            
            # Convert obstacles to point cloud for navigation
            if obstacles:
                pointcloud = self.create_pointcloud(obstacles, msg.header)
                self.pointcloud_pub.publish(pointcloud)
            
            # Publish processed image for visualization
            processed_msg = self.bridge.cv2_to_imgmsg(processed_img, "bgr8")
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

    def detect_obstacles(self, image):
        """
        Simple obstacle detection using color and edge detection
        Returns list of obstacle points and processed image
        """
        height, width = image.shape[:2]
        processed_img = image.copy()
        
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for potential obstacles (darker regions, edges)
        # Adjust these values based on your environment
        lower_bound = np.array([0, 0, 0])
        upper_bound = np.array([180, 255, self.obstacle_threshold])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Apply edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Combine masks
        combined_mask = cv2.bitwise_or(mask, edges)
        
        # Find contours (potential obstacles)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        
        # Process contours to extract obstacle positions
        for contour in contours:
            # Filter small contours
            if cv2.contourArea(contour) < 100:
                continue
                
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Skip obstacles too high (likely not ground-level)
            if y < height * 0.3:  # Skip upper 30% of image
                continue
            
            # Estimate distance based on vertical position in image
            # This is a simple approximation - more sophisticated methods exist
            distance = self.estimate_distance(y, height)
            
            if distance > self.max_range:
                continue
            
            # Estimate horizontal angle based on x position
            angle = (x + w/2 - width/2) * (60.0 / width) * np.pi / 180  # Assume 60° FOV
            
            # Convert to Cartesian coordinates (camera frame)
            obstacle_x = distance * np.cos(angle)
            obstacle_y = distance * np.sin(angle)
            obstacle_z = 0.0  # Assume ground level
            
            obstacles.append([obstacle_x, obstacle_y, obstacle_z])
            
            # Draw on processed image
            cv2.rectangle(processed_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(processed_img, f'{distance:.1f}m', 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return obstacles, processed_img

    def estimate_distance(self, pixel_y, image_height):
        """
        Simple distance estimation based on pixel row
        This assumes camera is looking forward and obstacles are on the ground
        """
        # Normalize pixel position (0 = top, 1 = bottom)
        normalized_y = pixel_y / image_height
        
        # Simple inverse relationship - closer objects appear lower in image
        # This is a rough approximation and should be calibrated for your setup
        distance = self.max_range * (1.0 - normalized_y) + 0.2
        
        return min(distance, self.max_range)

    def create_pointcloud(self, obstacles, header):
        """
        Create a PointCloud2 message from obstacle points
        """
        points = []
        for obs in obstacles:
            points.append([obs[0], obs[1], obs[2]])  # x, y, z
        
        # Create PointCloud2 message
        pointcloud = pc2.create_cloud_xyz32(header, points)
        pointcloud.header.frame_id = "camera_link"
        
        return pointcloud

def main(args=None):
    rclpy.init(args=args)
    node = RgbToOccupancy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 