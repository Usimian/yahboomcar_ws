#!/usr/bin/env python3
"""
Robot Jetson Data Server - ROS2 Node for slam_nav package
Simple data collection and command execution server for Yahboomcar X3 robot
Runs on Jetson robot side:
1. Collects sensor data (camera, lidar, IMU)
2. Sends data to PC client hub (which runs VILA)
3. Receives navigation commands from client hub
4. Executes movement commands
"""

import rclpy
from rclpy.node import Node
import sys
import os
import json
import base64
import requests
import threading
import time
import cv2
import numpy as np
from io import BytesIO
from PIL import Image

# ROS2 message imports
from sensor_msgs.msg import Image as ImageMsg, CompressedImage, LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge

class RobotJetsonServer(Node):
    """
    Robot data server that runs on Jetson and communicates with PC client hub:
    1. Collects camera images, lidar data, IMU data from robot sensors
    2. Sends data to PC client hub (which runs VILA for AI processing)
    3. Receives navigation commands from client hub
    4. Executes movement commands on robot
    
    This node does NOT run VILA - that happens on the PC client hub side.
    """
    
    def __init__(self):
        super().__init__('robot_jetson_server')
        
        # Initialize parameters
        self.declare_parameter('client_hub_url', 'http://192.168.1.153:5000')  # Client PC IP
        self.declare_parameter('robot_id', 'yahboomcar_x3_01')
        self.declare_parameter('send_frequency', 2.0)  # Hz - how often to send data
        self.declare_parameter('image_quality', 85)  # JPEG quality
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        self.declare_parameter('angular_velocity', 0.3)  # rad/s
        self.declare_parameter('emergency_stop_enabled', True)
        
        self.client_hub_url = self.get_parameter('client_hub_url').value
        self.robot_id = self.get_parameter('robot_id').value
        self.send_frequency = self.get_parameter('send_frequency').value
        self.image_quality = self.get_parameter('image_quality').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.emergency_stop = self.get_parameter('emergency_stop_enabled').value
        
        self.get_logger().info(f'🤖 Robot Server starting - ID: {self.robot_id}')
        self.get_logger().info(f'📡 Client Hub URL: {self.client_hub_url}')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.latest_image = None
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_voltage = None  # Battery voltage from /voltage topic
        self.latest_battery = 100.0  # Calculated from voltage
        
        # Robot state
        self.robot_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0}
        self.robot_status = 'active'
        self.last_response_time = time.time()
        
        # Setup ROS2 subscribers
        self.setup_subscribers()
        
        # Setup ROS2 publishers
        self.setup_publishers()
        
        # Register with client hub
        self.register_with_hub()
        
        # Start data sending thread
        self.start_data_sender()
        
        # Start safety monitor
        if self.emergency_stop:
            self.start_safety_monitor()
        
        self.get_logger().info('✅ Robot Jetson Server ready!')
    
    def setup_subscribers(self):
        """Setup ROS2 topic subscribers"""
        # Camera subscriber (RGB image)
        self.image_sub = self.create_subscription(
            ImageMsg,
            '/camera/realsense_camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Lidar subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            100
        )
        
        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        # Battery voltage subscriber
        self.voltage_sub = self.create_subscription(
            Float32,
            '/voltage',
            self.voltage_callback,
            10
        )
        
        self.get_logger().info('📡 ROS2 subscribers initialized')
    
    def setup_publishers(self):
        """Setup ROS2 topic publishers"""
        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/robot/vila_status', 10)
        
        # Analysis result publisher
        self.analysis_pub = self.create_publisher(String, '/robot/vila_analysis', 10)
        
        self.get_logger().info('📢 ROS2 publishers initialized')
    
    def image_callback(self, msg):
        """Store latest camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert BGR to RGB for PIL
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Convert to PIL Image
            self.latest_image = Image.fromarray(rgb_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def lidar_callback(self, msg):
        """Store latest lidar data"""
        # Store basic lidar info - could be expanded for specific analysis
        self.latest_lidar = {
            'ranges': list(msg.ranges),
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }
    
    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
    
    def voltage_callback(self, msg):
        """Store latest voltage data and calculate battery percentage"""
        self.latest_voltage = msg.data
        
        # Convert voltage to battery percentage
        # Typical Li-ion battery: ~12.6V = 100%, ~10.0V = 0%
        # Adjust these values based on your robot's battery specifications
        voltage_max = 12.6  # Fully charged voltage
        voltage_min = 10.0  # Empty voltage
        
        if self.latest_voltage is not None:
            # Calculate percentage with bounds checking
            percentage = ((self.latest_voltage - voltage_min) / (voltage_max - voltage_min)) * 100
            self.latest_battery = max(0.0, min(100.0, percentage))
            
            # Log battery info occasionally
            if hasattr(self, '_last_battery_log'):
                if time.time() - self._last_battery_log > 30:  # Log every 30 seconds
                    self.get_logger().info(f'🔋 Battery: {self.latest_voltage:.1f}V ({self.latest_battery:.1f}%)')
                    self._last_battery_log = time.time()
            else:
                self._last_battery_log = time.time()
                self.get_logger().info(f'🔋 Battery monitoring started: {self.latest_voltage:.1f}V ({self.latest_battery:.1f}%)')
    
    def register_with_hub(self):
        """Register this robot with the client hub"""
        robot_info = {
            'robot_id': self.robot_id,
            'name': f'Yahboomcar X3 - {self.robot_id}',
            'capabilities': ['navigation', 'camera', 'lidar', 'mecanum_drive'],
            'position': self.robot_position,
            'battery_level': self.latest_battery,
            'connection_type': 'http'
        }
        
        try:
            response = requests.post(
                f'{self.client_hub_url}/robots/register',
                json=robot_info,
                timeout=5
            )
            
            if response.status_code == 200:
                self.get_logger().info('✅ Successfully registered with client hub')
                return True
            else:
                self.get_logger().warn(f'⚠️ Registration failed: {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to register with hub: {e}')
            return False
    
    def start_data_sender(self):
        """Start thread to send data to client hub"""
        def data_sender():
            rate = 1.0 / self.send_frequency  # Convert Hz to seconds
            
            while rclpy.ok():
                try:
                    self.send_data_to_hub()
                    time.sleep(rate)
                except Exception as e:
                    self.get_logger().error(f'Data sender error: {e}')
                    time.sleep(1.0)  # Wait before retrying
        
        self.data_thread = threading.Thread(target=data_sender, daemon=True)
        self.data_thread.start()
        self.get_logger().info(f'📤 Data sender started at {self.send_frequency} Hz')
    
    def start_safety_monitor(self):
        """Start safety monitoring thread"""
        def safety_monitor():
            while rclpy.ok():
                try:
                    # Check if we haven't received a response in too long
                    if time.time() - self.last_response_time > 10.0:  # 10 seconds
                        self.get_logger().warn('⚠️ No response from hub - emergency stop')
                        self.emergency_stop_robot()
                    
                    time.sleep(1.0)  # Check every second
                except Exception as e:
                    self.get_logger().error(f'Safety monitor error: {e}')
        
        self.safety_thread = threading.Thread(target=safety_monitor, daemon=True)
        self.safety_thread.start()
        self.get_logger().info('🛡️ Safety monitor started')
    
    def send_data_to_hub(self):
        """Send current sensor data to client hub for processing"""
        if self.latest_image is None:
            return  # No image data yet
        
        try:
            # Convert image to base64
            buffered = BytesIO()
            self.latest_image.save(buffered, format="JPEG", quality=self.image_quality)
            image_b64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
            
            # Prepare payload
            payload = {
                'image': image_b64,
                'prompt': 'Analyze this robot camera view for autonomous navigation. What should the robot do next?',
                'sensor_data': {
                    'lidar': self.latest_lidar,
                    'imu': self.latest_imu,
                    'position': self.robot_position,
                    'battery': self.latest_battery
                },
                'generate_commands': True
            }
            
            # Send to client hub for analysis
            response = requests.post(
                f'{self.client_hub_url}/robots/{self.robot_id}/analyze',
                json=payload,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                self.last_response_time = time.time()  # Update response time
                self.process_hub_response(result)
            else:
                self.get_logger().warn(f'Hub analysis failed: {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'Error sending data to hub: {e}')
    
    def process_hub_response(self, result):
        """Process analysis result from client hub and execute commands"""
        if not result.get('success', False):
            return
        
        analysis = result.get('analysis', '')
        commands = result.get('commands', {})
        
        self.get_logger().info(f'🧠 VILA Analysis: {analysis[:100]}...')
        
        # Execute movement commands
        self.execute_navigation_commands(commands)
        
        # Publish analysis result
        analysis_msg = String()
        analysis_msg.data = analysis
        self.analysis_pub.publish(analysis_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'analysis': analysis,
            'commands': commands,
            'timestamp': time.time()
        })
        self.status_pub.publish(status_msg)
    
    def execute_navigation_commands(self, commands):
        """Convert hub commands to robot movement"""
        twist = Twist()
        
        # Safety first - stop if hazard detected
        if commands.get('hazard_detected', False) or commands.get('stop', False):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('🛑 STOP - Hazard detected or stop command')
        
        # Execute movement commands
        elif commands.get('move_forward', False):
            twist.linear.x = self.linear_vel
            twist.angular.z = 0.0
            self.get_logger().info('⬆️ Moving forward')
        
        elif commands.get('turn_left', False):
            twist.linear.x = 0.0
            twist.angular.z = self.angular_vel
            self.get_logger().info('⬅️ Turning left')
        
        elif commands.get('turn_right', False):
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_vel
            self.get_logger().info('➡️ Turning right')
        
        else:
            # Default: stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
    
    def emergency_stop_robot(self):
        """Emergency stop the robot"""
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED')
    
    def check_for_pending_commands(self):
        """Check client hub for any pending commands"""
        try:
            response = requests.get(
                f'{self.client_hub_url}/robots/{self.robot_id}/commands',
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                commands = result.get('commands', [])
                
                for cmd in commands:
                    self.execute_robot_command(cmd)
                    
        except Exception as e:
            self.get_logger().error(f'Error checking commands: {e}')
    
    def execute_robot_command(self, command):
        """Execute a specific robot command"""
        cmd_type = command.get('command_type', '')
        params = command.get('parameters', {})
        
        if cmd_type == 'move':
            twist = Twist()
            direction = params.get('direction', 'stop')
            speed = params.get('speed', self.linear_vel)
            
            if direction == 'forward':
                twist.linear.x = speed
            elif direction == 'backward':
                twist.linear.x = -speed
            elif direction == 'left':
                twist.angular.z = speed
            elif direction == 'right':
                twist.angular.z = -speed
            
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Executing command: {cmd_type} - {direction}')
        
        elif cmd_type == 'stop':
            self.emergency_stop_robot()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_server = RobotJetsonServer()
        
        # Create timer for periodic command checking
        def check_commands():
            robot_server.check_for_pending_commands()
        
        robot_server.create_timer(5.0, check_commands)  # Check every 5 seconds
        
        rclpy.spin(robot_server)
        
    except KeyboardInterrupt:
        print('👋 Robot server shutting down...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()