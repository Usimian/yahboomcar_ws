#!/usr/bin/env python3
"""
Robot Client Node - Hardware Interface with Single Gateway
Implements the ExecuteCommand service as the single gateway for all robot commands
Interfaces with physical robot hardware and publishes sensor data
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Image, LaserScan, Imu, BatteryState
from std_msgs.msg import String, Bool, Float32
from robot_msgs.msg import RobotCommand, RobotStatus, SensorData
from robot_msgs.srv import ExecuteCommand
import json
import time
import psutil
from datetime import datetime


class RobotClientNode(Node):
    def __init__(self):
        super().__init__('robot_client_node')
        
        # Robot identification
        self.robot_id = "yahboomcar_x3_01"
        
        # Single Gateway Service - THE primary command entry point
        self.execute_command_service = self.create_service(
            ExecuteCommand, 
            '/robot/execute_command', 
            self.execute_command_callback
        )
        
        # ROS2 Publishers for robot status and sensor data
        self.setup_publishers()
        
        # ROS2 Subscribers for sensor input
        self.setup_subscribers()
        
        # Robot state
        self.is_moving = False
        self.last_command_time = time.time()
        self.current_command = None
        self.robot_status = "online"
        self.battery_level = 100.0
        
        # Sensor data storage
        self.sensor_data = SensorData()
        self.sensor_data.robot_id = self.robot_id
        self.sensor_data.camera_status = "unknown"  # Initialize camera status
        
        # Publishing timers
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)  # 1Hz
        self.sensor_timer = self.create_timer(0.5, self.publish_sensor_data)   # 2Hz
        self.voltage_timer = self.create_timer(0.1, self.publish_voltage)      # 10Hz (like robot driver)
        
        self.get_logger().info(f'🤖 Robot Client Node started - ID: {self.robot_id}')
        self.get_logger().info('🚪 Single Gateway Service: /robot/execute_command')
        self.get_logger().info('📡 Publishing status and sensor data via ROS2')
        
    def setup_publishers(self):
        """Setup ROS2 publishers for robot status and sensor data"""
        
        # Robot status and sensor data (documented topics)
        self.status_pub = self.create_publisher(
            RobotStatus, '/robot/status', 10)
        self.sensor_pub = self.create_publisher(
            SensorData, '/robot/sensors', 10)
            
        # Camera feed (documented topics)
        self.image_pub = self.create_publisher(
            Image, '/robot/camera/image_raw', 10)
            
        # Command output to robot actuators
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Legacy compatibility topics
        self.voltage_pub = self.create_publisher(Float32, '/voltage', 10)
        
        self.get_logger().info('📡 Robot status and sensor publishers initialized')
        
    def setup_subscribers(self):
        """Setup ROS2 subscribers for sensor input"""
        
        # Camera input
        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.camera_callback, 10)
            
        # Lidar input
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        # Battery voltage input
        self.battery_sub = self.create_subscription(
            Float32, '/voltage', self.battery_callback, 10)
            
        # IMU input (available on /imu/data_raw, no need to republish)
        # self.imu_sub = self.create_subscription(
        #     Imu, '/imu/data', self.imu_callback, 10)
            
        self.get_logger().info('📡 Sensor subscribers initialized')
        
    def execute_command_callback(self, request, response):
        """
        Single Gateway Service Callback - THE command entry point
        All robot commands must go through this service
        """
        command = request.command
        
        self.get_logger().info(f'🚪 Gateway: Received command from {command.source_node}')
        self.get_logger().info(f'   Type: {command.command_type}')
        self.get_logger().info(f'   Robot: {command.robot_id}')
        
        # Validate robot ID
        if command.robot_id != self.robot_id:
            response.success = False
            response.result_message = f"Invalid robot ID: {command.robot_id}, expected: {self.robot_id}"
            response.execution_id = f"exec_{int(time.time() * 1e9)}"
            response.estimated_duration_ns = 0
            return response
        
        try:
            # Parse command parameters
            if command.parameters_json:
                parameters = json.loads(command.parameters_json)
            else:
                parameters = {}
            
            # Store current command
            self.current_command = command
            self.last_command_time = time.time()
            
            # Execute command based on type
            success = self.execute_gateway_command(command)
            
            # Prepare response
            response.success = success
            response.result_message = f"Command {command.command_type} {'executed' if success else 'failed'}"
            response.execution_id = f"exec_{int(time.time() * 1e9)}"
            response.estimated_duration_ns = int(command.duration * 1e9) if command.duration > 0 else 1000000000  # 1 second default
            
            self.get_logger().info(f'✅ Gateway: Command {command.command_type} {"executed" if success else "failed"}')
            
        except json.JSONDecodeError as e:
            response.success = False
            response.result_message = f"Invalid parameters JSON: {str(e)}"
            response.execution_id = f"exec_error_{int(time.time() * 1e9)}"
            response.estimated_duration_ns = 0
            self.get_logger().error(f'❌ JSON decode error: {str(e)}')
            
        except Exception as e:
            response.success = False
            response.result_message = f"Command execution error: {str(e)}"
            response.execution_id = f"exec_error_{int(time.time() * 1e9)}"
            response.estimated_duration_ns = 0
            self.get_logger().error(f'❌ Command execution error: {str(e)}')
        
        return response
    
    def execute_gateway_command(self, command):
        """
        Execute commands through the single gateway
        Returns True if successful, False otherwise
        """
        try:
            # Parse additional parameters if provided
            parameters = {}
            if command.parameters_json:
                parameters = json.loads(command.parameters_json)
            
            if command.command_type == 'move':
                # Create Twist message from command fields and parameters
                twist = Twist()
                twist.linear.x = float(command.linear_x)
                twist.linear.y = float(command.linear_y)
                twist.angular.z = float(command.angular_z)
                
                self.execute_movement(twist)
                return True
                
            elif command.command_type == 'stop':
                # Stop the robot
                twist = Twist()  # All zeros
                self.execute_movement(twist)
                return True
                
            elif command.command_type == 'rotate':
                # Rotate robot
                twist = Twist()
                twist.angular.z = float(command.angular_z) if command.angular_z != 0.0 else 0.5
                
                self.execute_movement(twist)
                return True
                
            else:
                self.get_logger().warning(f'❓ Unknown command type: {command.command_type}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Gateway command execution error: {str(e)}')
            return False
    
    def execute_movement(self, twist_msg):
        """Execute movement command through cmd_vel publisher"""
        # This is the ONLY place where /cmd_vel should be published from
        self.cmd_vel_pub.publish(twist_msg)
        self.is_moving = (twist_msg.linear.x != 0.0 or 
                         twist_msg.linear.y != 0.0 or 
                         twist_msg.angular.z != 0.0)
        
        self.get_logger().info(f'🚗 Movement: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}')
    
    def publish_robot_status(self):
        """Publish robot status to documented topic"""
        status_msg = RobotStatus()
        status_msg.robot_id = self.robot_id
        status_msg.name = "YahBoomcar X3"
        status_msg.last_seen_ns = int(time.time() * 1e9)
        
        # Position (placeholder - would come from odometry/SLAM)
        status_msg.position = Pose()
        
        status_msg.battery_level = self.battery_level
        status_msg.status = self.robot_status
        status_msg.capabilities = ["move", "rotate", "stop", "camera", "lidar"]
        status_msg.connection_type = "ROS2_unicast"
        status_msg.sensor_data = self.sensor_data
        
        if self.current_command:
            status_msg.last_command = f"{self.current_command.command_type}"
        else:
            status_msg.last_command = "none"
            
        self.status_pub.publish(status_msg)
    
    def publish_sensor_data(self):
        """Publish sensor data to documented topic"""
        # Update timestamp
        self.sensor_data.timestamp_ns = int(time.time() * 1e9)
        
        # Update CPU temperature (read from system)
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                self.sensor_data.cpu_temp = temp_millicelsius / 1000.0  # Convert to Celsius
        except Exception as e:
            self.get_logger().debug(f'Could not read CPU temperature: {e}')
            self.sensor_data.cpu_temp = 0.0
        
        # Update CPU usage (using psutil)
        try:
            # Get CPU usage as percentage (0-100%)
            cpu_percent = psutil.cpu_percent(interval=None)  # Non-blocking call
            self.sensor_data.cpu_usage = cpu_percent
        except Exception as e:
            self.get_logger().debug(f'Could not read CPU usage: {e}')
            self.sensor_data.cpu_usage = 0.0
        
        # Publish sensor data
        self.sensor_pub.publish(self.sensor_data)
    
    def publish_voltage(self):
        """Publish battery voltage to legacy /voltage topic for compatibility"""
        voltage_msg = Float32()
        # Use battery_level as voltage (can be updated from actual sensor data)
        # Typical LiPo battery: 100% = ~12.6V, 0% = ~10.5V
        voltage_msg.data = 10.5 + (self.battery_level / 100.0) * 2.1
        self.voltage_pub.publish(voltage_msg)
    
    def camera_callback(self, msg):
        """Handle camera input and republish to documented topic"""
        # Republish camera image to documented topic
        self.image_pub.publish(msg)
        
        # Update camera status in sensor data
        self.sensor_data.camera_status = "active"
    
    def scan_callback(self, msg):
        """Handle lidar scan input"""
        # Update sensor data with lidar info
        if len(msg.ranges) > 0:
            # Simple front/left/right distance calculation
            front_idx = len(msg.ranges) // 2
            left_idx = len(msg.ranges) * 3 // 4
            right_idx = len(msg.ranges) // 4
            
            self.sensor_data.distance_front = msg.ranges[front_idx] if front_idx < len(msg.ranges) else 0.0
            self.sensor_data.distance_left = msg.ranges[left_idx] if left_idx < len(msg.ranges) else 0.0
            self.sensor_data.distance_right = msg.ranges[right_idx] if right_idx < len(msg.ranges) else 0.0
    
    def battery_callback(self, msg):
        """Handle battery voltage input"""
        # Update sensor data with battery voltage
        self.sensor_data.battery_voltage = msg.data
        
        # Also update battery_level for compatibility
        # Typical LiPo: 12.6V = 100%, 10.5V = 0%
        voltage_range = 2.1  # 12.6 - 10.5
        self.battery_level = max(0.0, min(100.0, ((msg.data - 10.5) / voltage_range) * 100.0))
        
        # Battery percentage removed from SensorData - available in battery_level if needed
    
    def imu_callback(self, msg):
        """Handle IMU input"""
        # IMU data is now published directly on /imu/data_raw topic
        # No need to include in SensorData as it's available separately
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Robot Client Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
