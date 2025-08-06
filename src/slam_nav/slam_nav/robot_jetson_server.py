#!/usr/bin/env python3
"""
Robot Jetson Server - Single Robot System (PULL Architecture)
Updated for CLIENT-INITIATED (pull-based) communication
Based on ROBOT_UNIFIED_INTEGRATION_GUIDE.md specifications
Robot ID: yahboomcar_x3_01 (hardcoded)
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
import math
import psutil
import subprocess
from io import BytesIO
from PIL import Image
from datetime import datetime
from flask import Flask, jsonify

# ROS2 message imports
from sensor_msgs.msg import Image as ImageMsg, CompressedImage, LaserScan, Imu, BatteryState, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge

class RobotJetsonServer(Node):
    """
    Single Robot System Integration (PULL Architecture):
    1. Runs HTTP server on port 8080 for client to pull data
    2. Polls server for movement commands (ONLY robot-initiated communication)
    3. Executes movement commands with safety checks
    4. Client pulls images and sensor data when needed
    5. Hardcoded robot ID: yahboomcar_x3_01
    """
    
    def __init__(self):
        super().__init__('robot_jetson_server')
        
        # Single Robot System - Hardcoded Configuration
        self.robot_id = "yahboomcar_x3_01"  # Hardcoded as per guide
        
        # Initialize parameters with backward compatibility
        self.declare_parameter('controller_host', '192.168.1.153')  # Server IP
        self.declare_parameter('controller_port', 5000)  # Server port
        self.declare_parameter('robot_port', 8080)  # Robot HTTP server port
        self.declare_parameter('client_hub_url', '')  # Legacy parameter
        self.declare_parameter('command_poll_frequency', 2.0)  # Hz for command polling
        
        # Get parameters
        controller_host = self.get_parameter('controller_host').value
        controller_port = self.get_parameter('controller_port').value
        self.robot_port = self.get_parameter('robot_port').value
        self.command_poll_frequency = self.get_parameter('command_poll_frequency').value
        
        # Handle legacy client_hub_url parameter
        legacy_url = self.get_parameter('client_hub_url').value
        if legacy_url:
            import urllib.parse
            parsed = urllib.parse.urlparse(legacy_url)
            controller_host = parsed.hostname or controller_host
            controller_port = parsed.port or controller_port
        
        # Single Robot System URLs (simplified)
        self.server_url = f"http://{controller_host}:{controller_port}"
        self.commands_url = f"{self.server_url}/robots/{self.robot_id}/commands"
        
        self.get_logger().info(f'🤖 Single Robot System starting - ID: {self.robot_id}')
        self.get_logger().info(f'📡 Server URL: {self.server_url}')
        self.get_logger().info(f'🌐 Robot HTTP Server Port: {self.robot_port}')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage for PULL system
        self.latest_image = None
        self.sensor_data = {}
        self.last_command_time = time.time()
        
        # Robot state
        self.robot_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0}
        self.is_moving = False
        self.movement_end_time = 0.0
        self.current_twist = Twist()
        
        # Setup ROS2 subscribers and publishers
        self.setup_subscribers()
        self.setup_publishers()
        
        # Start HTTP server for PULL system
        self.start_http_server()
        
        # Start command polling loop (ONLY robot-initiated communication)
        self.start_robot_loops()
        
        self.get_logger().info('✅ Single Robot System ready (PULL architecture)!')
    
    def setup_subscribers(self):
        """Setup ROS2 topic subscribers"""
        # Camera subscriber (RGB image) - For client to pull
        # Subscribe to the actual RealSense camera topic
        self.image_sub = self.create_subscription(
            ImageMsg, '/camera/realsense_camera/color/image_raw', self.image_callback, 10)
        
        # Sensor subscribers for client to pull
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.battery_sub = self.create_subscription(
            Float32, '/voltage', self.voltage_callback, 10)
        self.temp_sub = self.create_subscription(
            Float32, '/temperature', self.temperature_callback, 10)
        
        self.get_logger().info('📡 ROS2 subscribers initialized')
    
    def setup_publishers(self):
        """Setup ROS2 topic publishers"""
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        self.get_logger().info('📢 ROS2 publishers initialized')
    
    def image_callback(self, msg):
        """Store latest camera image for client to pull"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert BGR to RGB for PIL
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Convert to PIL Image and store for client to pull
            self.latest_image = Image.fromarray(rgb_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def lidar_callback(self, msg):
        """Store lidar data for client to pull"""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_dist = min(valid_ranges)
            self.sensor_data["lidar_distance"] = min_dist
            self.get_logger().debug(f"📡 Lidar update: min_distance = {min_dist:.2f}m")
        else:
            self.sensor_data["lidar_distance"] = float('inf')
            self.get_logger().warning(f"📡 Lidar: No valid readings!")
    
    def imu_callback(self, msg):
        """Store IMU data for client to pull"""
        # Store acceleration values (x, y, z)
        acceleration = msg.linear_acceleration
        self.sensor_data["imu_values"] = {
            "x": acceleration.x,
            "y": acceleration.y,
            "z": acceleration.z
        }
        
        # Still calculate heading for robot position tracking
        orientation = msg.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        heading_degrees = math.degrees(yaw)
        if heading_degrees < 0:
            heading_degrees += 360.0
        
        self.robot_position['heading'] = heading_degrees
    
    def voltage_callback(self, msg: Float32):
        """Store voltage data for client to pull"""
        voltage = msg.data
        # Estimate battery percentage based on voltage (rough estimate for 12V system)
        # Typical LiPo 3S: 12.6V (100%) -> 10.5V (0%)
        battery_percentage = max(0, min(100, ((voltage - 10.5) / (12.6 - 10.5)) * 100))
        
        self.sensor_data.update({
            "battery_voltage": voltage,
            "battery_percentage": battery_percentage,
        })
    
    def temperature_callback(self, msg):
        """Store temperature data for client to pull"""
        self.sensor_data["temperature"] = msg.data
    
    def get_battery_voltage(self):
        """Read battery voltage from stored sensor data or system"""
        # Return stored sensor data if available
        if "battery_voltage" in self.sensor_data:
            return self.sensor_data["battery_voltage"]
        
        # Fallback: Try to read from system files or return default
        try:
            # Example for systems with power_supply info
            # with open('/sys/class/power_supply/BAT0/voltage_now', 'r') as f:
            #     voltage_uv = int(f.read().strip())
            #     return voltage_uv / 1000000.0  # Convert microvolts to volts
            pass
        except:
            pass
        
        return 12.4  # Default voltage
    
    def get_imu_values(self):
        """Read acceleration values from IMU data"""
        return self.sensor_data.get("imu_values", {"x": 0.0, "y": 0.0, "z": 0.0})
    
    def get_camera_status(self):
        """Check camera operational status"""
        return "Active" if self.latest_image is not None else "Inactive"
    
    def get_temperature(self):
        """Read system temperature"""
        if "temperature" in self.sensor_data:
            return self.sensor_data["temperature"]
        
        # Fallback: Read from system temperature sensor
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0  # Convert from millicelsius
                return temp
        except:
            return 45.2  # Default if can't read
    
    def start_http_server(self):
        """Start HTTP server for PULL system"""
        app = Flask(__name__)
        
        @app.route('/sensors', methods=['GET'])
        def get_sensors():
            """Return current sensor readings (PULL endpoint)"""
            sensor_data = {
                "battery_voltage": self.get_battery_voltage(),
                "imu_values": self.get_imu_values(),
                "camera_status": self.get_camera_status(),
                "temperature": self.get_temperature(),
                "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.000Z")
            }
            
            self.get_logger().debug("📊 Client pulled sensor data")
            
            return jsonify(sensor_data)
        
        @app.route('/image', methods=['GET'])
        def get_image():
            """Return current camera image as base64 (PULL endpoint)"""
            if self.latest_image is None:
                return jsonify({"error": "No image available"}), 404
            
            try:
                # Convert PIL image to base64
                buffered = BytesIO()
                self.latest_image.save(buffered, format="JPEG", quality=85)
                image_b64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
                
                self.get_logger().debug("📷 Client pulled camera image")
                return jsonify({
                    "image": image_b64,
                    "format": "JPEG",
                    "width": self.latest_image.width,
                    "height": self.latest_image.height,
                    "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.000Z")
                })
            except Exception as e:
                self.get_logger().error(f"Error serving image: {e}")
                return jsonify({"error": str(e)}), 500
        
        @app.route('/status', methods=['GET'])
        def get_status():
            """Return robot status (PULL endpoint)"""
            status_data = {
                "robot_id": self.robot_id,
                "is_moving": self.is_moving,
                "position": self.robot_position,
                "battery_percentage": self.sensor_data.get("battery_percentage", 100.0),
                "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.000Z")
            }
            return jsonify(status_data)
        
        # Start HTTP server in background thread
        def run_server():
            self.get_logger().info(f"🌐 Starting HTTP server on port {self.robot_port}")
            app.run(host='0.0.0.0', port=self.robot_port, debug=False, use_reloader=False)
        
        self.http_thread = threading.Thread(target=run_server, daemon=True)
        self.http_thread.start()
    
    def start_robot_loops(self):
        """Start the main robot control loops"""
        # Command polling loop (ONLY robot-initiated communication)
        self.command_timer = self.create_timer(
            1.0 / self.command_poll_frequency, self.command_poll_loop)
        
        # Safety monitoring loop
        self.safety_timer = self.create_timer(1.0, self.safety_check_loop)
        
        # Movement control loop (non-blocking movement)
        self.movement_timer = self.create_timer(0.1, self.movement_control_loop)
    
    def command_poll_loop(self):
        """Poll server for movement commands (ONLY robot-initiated communication)"""
        try:
            response = requests.get(self.commands_url, timeout=5)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    commands = result.get('commands', [])
                    command_count = result.get('count', 0)
                    
                    if commands and command_count > 0:
                        self.get_logger().info(f"📥 Received {command_count} commands from server")
                        for command in commands:
                            # Verify command has required fields as per guide
                            if all(key in command for key in ['robot_id', 'command_type', 'parameters']):
                                if command['robot_id'] == self.robot_id:
                                    self.execute_command_safely(command)
                                else:
                                    self.get_logger().warning(f"❌ Command for wrong robot: {command['robot_id']}")
                            else:
                                self.get_logger().warning(f"❌ Invalid command format: {command}")
                        self.last_command_time = time.time()
                    
            else:
                self.get_logger().debug(f"Command poll status: {response.status_code}")
                
        except Exception as e:
            self.get_logger().debug(f"Command poll error: {e}")
    
    def execute_command_safely(self, command):
        """Execute command with safety checks as per guide"""
        cmd_type = command.get('command_type')
        params = command.get('parameters', {})
        
        self.get_logger().info(f"🤖 Received command: {cmd_type} with params: {params}")
        
        # 1. Always check stop first
        if cmd_type == 'stop':
            self.emergency_stop()
            return
        
        # 2. Check battery level
        battery_pct = self.sensor_data.get("battery_percentage", 100.0)
        if battery_pct < 20:
            self.get_logger().warning(f"🔋 LOW BATTERY ({battery_pct:.1f}%) - TRIGGERING EMERGENCY STOP")
            self.emergency_stop()
            return
        
        # 3. Check for obstacles (only for linear movement)
        if cmd_type == 'move' and params.get('direction') in ['forward', 'backward']:
            if self.detect_obstacle():
                self.get_logger().warning("🚧 OBSTACLE DETECTED DURING COMMAND - TRIGGERING EMERGENCY STOP")
                self.emergency_stop()
                return
        
        # 4. Execute command with timeout protection
        self.execute_command_with_timeout(command, timeout=5.0)
    
    def execute_command_with_timeout(self, command, timeout=5.0):
        """Execute command with timeout protection"""
        cmd_type = command.get('command_type')
        params = command.get('parameters', {})
        
        self.get_logger().info(f"🤖 Executing: {cmd_type} with params: {params}")
        
        if cmd_type == 'move':
            direction = params.get('direction')
            speed = params.get('speed', 0.3)
            duration = min(params.get('duration', 2.0), timeout)  # Limit duration to timeout
            
            if direction == 'forward':
                self.move_forward(speed, duration)
            elif direction == 'backward':
                self.move_backward(speed, duration)
            elif direction == 'left':
                self.turn_left(speed, duration)
            elif direction == 'right':
                self.turn_right(speed, duration)
                
        elif cmd_type == 'turn':
            direction = params.get('direction')
            angle = params.get('angle', 45)
            speed = params.get('speed', 0.2)
            
            # Convert angle to duration (rough estimate) and limit to timeout
            duration = min(angle / 45.0, timeout)
            
            if direction == 'left':
                self.turn_left(speed, duration)
            elif direction == 'right':
                self.turn_right(speed, duration)
        
        self.get_logger().info(f"✅ Command {cmd_type} initiated")
    
    def move_forward(self, speed, duration):
        """Move robot forward (non-blocking)"""
        self.get_logger().info(f"🚀 Starting forward movement: speed={speed}, duration={duration}")
        self.is_moving = True
        self.movement_end_time = time.time() + duration
        self.current_twist = Twist()
        self.current_twist.linear.x = speed
        self.current_twist.angular.z = 0.0
    
    def move_backward(self, speed, duration):
        """Move robot backward (non-blocking)"""
        self.get_logger().info(f"🚀 Starting backward movement: speed={speed}, duration={duration}")
        self.is_moving = True
        self.movement_end_time = time.time() + duration
        self.current_twist = Twist()
        self.current_twist.linear.x = -speed
        self.current_twist.angular.z = 0.0
    
    def turn_left(self, speed, duration):
        """Turn robot left (non-blocking)"""
        self.get_logger().info(f"🚀 Starting left turn: speed={speed}, duration={duration}")
        self.is_moving = True
        self.movement_end_time = time.time() + duration
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = speed
    
    def turn_right(self, speed, duration):
        """Turn robot right (non-blocking)"""
        self.get_logger().info(f"🚀 Starting right turn: speed={speed}, duration={duration}")
        self.is_moving = True
        self.movement_end_time = time.time() + duration
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -speed
    
    def stop_robot(self):
        """Stop robot movement"""
        self.get_logger().info("🛑 Stopping robot")
        self.is_moving = False
        self.movement_end_time = 0.0
        self.current_twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(self.current_twist)
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all movement"""
        self.get_logger().error("🛑 EMERGENCY STOP EXECUTED - ALL MOVEMENT HALTED")
        self.is_moving = False
        self.movement_end_time = 0.0
        self.current_twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(self.current_twist)
    
    def detect_obstacle(self):
        """Check for obstacles using lidar"""
        lidar_distance = self.sensor_data.get("lidar_distance", float('inf'))
        obstacle_detected = lidar_distance < 0.5  # Stop if obstacle within 50cm
        if obstacle_detected:
            self.get_logger().warning(f"🚧 OBSTACLE DETECTED at {lidar_distance:.2f}m")
        return obstacle_detected
    
    def movement_control_loop(self):
        """Non-blocking movement control loop"""
        if self.is_moving:
            current_time = time.time()
            
            # Check if movement should end
            if current_time >= self.movement_end_time:
                self.get_logger().info("⏰ Movement duration completed")
                self.stop_robot()
                return
            
            # Check for safety issues during linear movement
            if self.current_twist.linear.x != 0.0:  # Linear movement
                if self.detect_obstacle():
                    self.emergency_stop()
                    return
            
            # Continue publishing current movement command
            self.cmd_vel_pub.publish(self.current_twist)
    
    def safety_check_loop(self):
        """Safety monitoring loop"""
        current_time = time.time()
        
        # Stop if no commands received for 30+ seconds
        time_since_command = current_time - self.last_command_time
        if time_since_command > 30.0:
            if self.is_moving:
                self.get_logger().warning(f"⏱️ COMMAND TIMEOUT ({time_since_command:.1f}s) - TRIGGERING EMERGENCY STOP")
                self.emergency_stop()
        
        # Check battery level
        battery_pct = self.sensor_data.get("battery_percentage", 100.0)
        if battery_pct < 20 and self.is_moving:
            self.get_logger().warning(f"🔋 LOW BATTERY DURING MOVEMENT ({battery_pct:.1f}%) - TRIGGERING EMERGENCY STOP")
            self.emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_server = RobotJetsonServer()
        rclpy.spin(robot_server)
        
    except KeyboardInterrupt:
        print('👋 Single Robot System shutting down...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()