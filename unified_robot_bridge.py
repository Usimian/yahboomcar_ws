#!/usr/bin/env python3
"""
Unified Robot Bridge Node for YahBoom Car X3
Bridges ROS2 sensors/actuators with HTTP-based unified controller
Based on ROBOT_UNIFIED_INTEGRATION_GUIDE.md specifications
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Range, Image as ImageMsg, LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
import asyncio
import aiohttp
import threading
import time
import psutil
import subprocess
import re
import cv2
import numpy as np
import math
from datetime import datetime
from cv_bridge import CvBridge

class UnifiedRobotBridge(Node):
    def __init__(self):
        super().__init__('unified_robot_bridge')
        
        # Robot identification
        self.declare_parameter('robot_id', 'yahboom_robot_001')
        self.declare_parameter('controller_host', '192.168.1.100')
        self.declare_parameter('controller_port', 5000)
        
        self.robot_id = self.get_parameter('robot_id').value
        controller_host = self.get_parameter('controller_host').value
        controller_port = self.get_parameter('controller_port').value
        
        self.controller_url = f"http://{controller_host}:{controller_port}"
        
        # Initialize CV bridge for image processing
        self.bridge = CvBridge()
        
        # ROS2 Subscribers (sensors)
        self.setup_subscribers()
        
        # ROS2 Publishers (actuators)
        self.setup_publishers()
        
        # Sensor data storage
        self.sensor_data = {}
        self.latest_image = None
        
        # Robot state
        self.robot_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0}
        
        # HTTP client setup
        self.setup_async_client()
        
        # Periodic sensor sending
        self.sensor_timer = self.create_timer(2.0, self.send_sensor_data_sync)
        
        # Periodic command checking
        self.command_timer = self.create_timer(1.0, self.check_safety_status)
        
        self.get_logger().info(f"🤖 Unified Robot Bridge started - ID: {self.robot_id}")
        self.get_logger().info(f"📡 Controller URL: {self.controller_url}")
    
    def setup_subscribers(self):
        """Setup ROS2 topic subscribers"""
        # Camera subscriber (RGB image)
        self.image_sub = self.create_subscription(
            ImageMsg, '/camera/color/image_raw', self.image_callback, 10)
        
        # Battery subscriber
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        
        # Range sensors (ultrasonic)
        self.range_front_sub = self.create_subscription(
            Range, '/ultrasonic_front', self.range_front_callback, 10)
        self.range_left_sub = self.create_subscription(
            Range, '/ultrasonic_left', self.range_left_callback, 10)
        self.range_right_sub = self.create_subscription(
            Range, '/ultrasonic_right', self.range_right_callback, 10)
        
        # Lidar subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # Temperature subscriber (if available)
        self.temp_sub = self.create_subscription(
            Float32, '/temperature', self.temperature_callback, 10)
        
        self.get_logger().info('📡 ROS2 subscribers initialized')
    
    def setup_publishers(self):
        """Setup ROS2 topic publishers"""
        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        self.get_logger().info('📢 ROS2 publishers initialized')
    
    def setup_async_client(self):
        """Setup async HTTP client in separate thread"""
        self.loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self.run_async_loop)
        self.async_thread.daemon = True
        self.async_thread.start()
        
        # Register robot
        asyncio.run_coroutine_threadsafe(self.register_robot(), self.loop)
    
    def run_async_loop(self):
        """Run async event loop in separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    async def register_robot(self):
        """Register with unified controller"""
        registration_data = {
            "robot_id": self.robot_id,
            "name": "YahBoom Robot X3",
            "position": {"x": 0, "y": 0, "z": 0, "heading": 0, "ip": "192.168.1.101"},
            "battery_level": self.sensor_data.get('battery_percentage', 0.0),
            "capabilities": ["navigation", "vision", "sensors", "lidar", "slam"],
            "connection_type": "http",
            "sensor_data": self.get_current_sensors()
        }
        
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.controller_url}/api/robots/register",
                    json=registration_data,
                    timeout=10
                ) as response:
                    if response.status == 200:
                        self.get_logger().info("✅ Registered with unified controller")
                    else:
                        self.get_logger().error(f"❌ Registration failed: {response.status}")
        except Exception as e:
            self.get_logger().error(f"❌ Registration error: {e}")
    
    def image_callback(self, msg):
        """Handle camera image data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.sensor_data["camera_status"] = "active"
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            self.sensor_data["camera_status"] = "error"
    
    def battery_callback(self, msg: BatteryState):
        """Handle battery sensor data"""
        self.sensor_data.update({
            "battery_voltage": msg.voltage,
            "battery_percentage": msg.percentage * 100 if msg.percentage >= 0 else 0.0,
            "battery_current": msg.current if hasattr(msg, 'current') else 0.0,
            "battery_temperature": msg.temperature if hasattr(msg, 'temperature') else 0.0
        })
    
    def range_front_callback(self, msg: Range):
        """Handle front ultrasonic sensor data"""
        if msg.range_min < msg.range < msg.range_max:
            self.sensor_data["distance_front"] = msg.range
        else:
            self.sensor_data["distance_front"] = float('inf')
    
    def range_left_callback(self, msg: Range):
        """Handle left ultrasonic sensor data"""
        if msg.range_min < msg.range < msg.range_max:
            self.sensor_data["distance_left"] = msg.range
        else:
            self.sensor_data["distance_left"] = float('inf')
    
    def range_right_callback(self, msg: Range):
        """Handle right ultrasonic sensor data"""
        if msg.range_min < msg.range < msg.range_max:
            self.sensor_data["distance_right"] = msg.range
        else:
            self.sensor_data["distance_right"] = float('inf')
    
    def lidar_callback(self, msg: LaserScan):
        """Handle lidar sensor data"""
        # Extract minimum distance for obstacle detection
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.sensor_data["lidar_min_distance"] = min(valid_ranges)
        else:
            self.sensor_data["lidar_min_distance"] = float('inf')
    
    def imu_callback(self, msg: Imu):
        """Handle IMU sensor data"""
        # Convert quaternion to euler angles for heading
        orientation = msg.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        heading_degrees = math.degrees(yaw)
        
        # Normalize to 0-360 degrees
        if heading_degrees < 0:
            heading_degrees += 360.0
        
        self.sensor_data.update({
            "imu_pitch": math.degrees(math.atan2(
                2 * (orientation.w * orientation.x + orientation.y * orientation.z),
                1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
            )),
            "imu_roll": math.degrees(math.asin(
                2 * (orientation.w * orientation.y - orientation.z * orientation.x)
            )),
            "imu_yaw": heading_degrees
        })
        
        # Update robot position heading
        self.robot_position['heading'] = heading_degrees
    
    def temperature_callback(self, msg: Float32):
        """Handle temperature sensor data"""
        self.sensor_data["temperature"] = msg.data
    
    def get_current_sensors(self):
        """Get real sensor readings in unified format"""
        sensors = {}
        
        # Required sensors
        sensors.update({
            "battery_voltage": self.sensor_data.get("battery_voltage", 12.0),
            "battery_percentage": self.sensor_data.get("battery_percentage", 0.0),
        })
        
        # Distance sensors (recommended)
        if "distance_front" in self.sensor_data and self.sensor_data["distance_front"] != float('inf'):
            sensors["distance_front"] = self.sensor_data["distance_front"]
        if "distance_left" in self.sensor_data and self.sensor_data["distance_left"] != float('inf'):
            sensors["distance_left"] = self.sensor_data["distance_left"]
        if "distance_right" in self.sensor_data and self.sensor_data["distance_right"] != float('inf'):
            sensors["distance_right"] = self.sensor_data["distance_right"]
        
        # Environmental (optional)
        if "temperature" in self.sensor_data:
            sensors["temperature"] = self.sensor_data["temperature"]
        
        # System monitoring (recommended)
        sensors.update({
            "cpu_usage": self.get_cpu_usage(),
            "memory_usage": self.get_memory_usage(),
            "wifi_signal": self.get_wifi_signal()
        })
        
        # Navigation (optional)
        if "imu_pitch" in self.sensor_data:
            sensors["imu_pitch"] = self.sensor_data["imu_pitch"]
        if "imu_roll" in self.sensor_data:
            sensors["imu_roll"] = self.sensor_data["imu_roll"]
        if "imu_yaw" in self.sensor_data:
            sensors["imu_yaw"] = self.sensor_data["imu_yaw"]
        
        # Position
        sensors.update({
            "position_x": self.robot_position.get("x", 0.0),
            "position_y": self.robot_position.get("y", 0.0),
            "heading": self.robot_position.get("heading", 0.0)
        })
        
        return sensors
    
    def send_sensor_data_sync(self):
        """Send sensor data (sync wrapper for async)"""
        if self.sensor_data:
            asyncio.run_coroutine_threadsafe(
                self.send_sensor_data_async(), self.loop
            )
    
    async def send_sensor_data_async(self):
        """Send sensor data to unified controller"""
        if not self.sensor_data:
            return
        
        # Get current sensor readings in unified format
        sensor_payload = self.get_current_sensors()
        sensor_payload["timestamp"] = datetime.now().isoformat()
        
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.controller_url}/api/robots/{self.robot_id}/sensors",
                    json=sensor_payload,
                    timeout=5
                ) as response:
                    if response.status == 200:
                        self.get_logger().debug(f"📊 Sent sensor data: {len(sensor_payload)} fields")
                    else:
                        self.get_logger().warning(f"⚠️ Sensor data failed: {response.status}")
        except Exception as e:
            self.get_logger().error(f"❌ Sensor send error: {e}")
    
    def check_safety_status(self):
        """Check safety status and handle emergency stops"""
        asyncio.run_coroutine_threadsafe(
            self.check_safety_status_async(), self.loop
        )
    
    async def check_safety_status_async(self):
        """Check safety status from unified controller"""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"{self.controller_url}/api/safety/status",
                    timeout=3
                ) as response:
                    if response.status == 200:
                        safety_status = await response.json()
                        
                        # Handle emergency stop
                        if safety_status.get('emergency_stop', False):
                            self.get_logger().error("🚨 EMERGENCY STOP ACTIVE - Halting immediately")
                            self.publish_stop_command()
                        
                        # Check movement permissions
                        if not safety_status.get('movement_enabled', True):
                            self.get_logger().warning("🚫 Movement disabled by safety system")
                            
        except Exception as e:
            self.get_logger().debug(f"Safety check error: {e}")
    
    async def execute_movement_command(self, command):
        """Execute movement with safety check"""
        # Check with unified controller safety system
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(f"{self.controller_url}/api/safety/status") as response:
                    if response.status == 200:
                        safety_status = await response.json()
                        
                        # Check if movement is allowed
                        if not safety_status.get('movement_enabled', False):
                            self.get_logger().warning("🚫 Movement blocked by safety system")
                            return False
                        
                        if safety_status.get('emergency_stop', False):
                            self.get_logger().error("🚨 EMERGENCY STOP ACTIVE - Halting immediately")
                            self.publish_stop_command()
                            return False
                            
        except Exception as e:
            self.get_logger().error(f"❌ Safety check failed: {e}")
            return False  # Fail safe - don't move if can't verify safety
        
        # If safety allows, execute the command
        self.get_logger().info(f"✅ Executing movement: {command}")
        # Movement execution would go here
        return True
    
    def publish_stop_command(self):
        """Immediately stop all robot movement"""
        stop_twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().warning("🛑 STOP command published")
    
    def get_cpu_usage(self) -> float:
        """Get CPU usage percentage"""
        try:
            return psutil.cpu_percent(interval=0.1)
        except:
            return 0.0
    
    def get_memory_usage(self) -> float:
        """Get memory usage percentage"""
        try:
            return psutil.virtual_memory().percent
        except:
            return 0.0
    
    def get_wifi_signal(self) -> int:
        """Get WiFi signal strength"""
        try:
            result = subprocess.run(['iwconfig'], capture_output=True, text=True)
            # Parse signal strength from iwconfig output
            for line in result.stdout.split('\n'):
                if 'Signal level' in line:
                    # Extract signal strength (e.g., "-45 dBm")
                    match = re.search(r'Signal level=(-?\d+)', line)
                    if match:
                        return int(match.group(1))
            return -99  # No signal
        except:
            return -99

def main(args=None):
    rclpy.init(args=args)
    bridge = UnifiedRobotBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()