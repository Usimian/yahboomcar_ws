#!/usr/bin/env python3
"""
Robot Interface Node - Simple Command Gateway for SLAM/Nav
Provides a clean interface for external clients to control the robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from robot_msgs.msg import SensorData
from robot_msgs.srv import ExecuteCommand, GetBatteryVoltage
import time
import psutil
import math
import threading


class RobotInterfaceNode(Node):
    def __init__(self):
        super().__init__('robot_interface_node')
        
        # Robot identification
        self.robot_id = "yahboomcar_x3_01"

        # Movement state tracking
        self.movement_active = False
        self.movement_thread = None
        self.stop_movement = threading.Event()
        
        # Odometry tracking for precision movement
        self.current_pose = None
        
        # Command Gateway Service - THE primary command entry point
        self.execute_command_service = self.create_service(
            ExecuteCommand, 
            '/robot/execute_command', 
            self.execute_command_callback
        )

        # Battery voltage service client
        self.battery_client = self.create_client(GetBatteryVoltage, 'get_battery_voltage')
        
        # Setup publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()
        
        # Robot state
        self.is_moving = False
        self.last_command_time = time.time()
        self.robot_status = "online"
        
        # Sensor data storage
        self.sensor_data = SensorData()
        self.sensor_data.robot_id = self.robot_id
        self.sensor_data.camera_status = "unknown"
        self.sensor_data.battery_voltage = 0.0  # Initialize battery voltage
        
        # Publishing timer
        self.sensor_timer = self.create_timer(0.5, self.publish_sensor_data)   # 2Hz
        
        self.get_logger().info(f'🤖 Robot Interface Node started - ID: {self.robot_id}')
        self.get_logger().info('🚪 Command Gateway Service: /robot/execute_command')
        self.get_logger().info('📡 Publishing sensor data for SLAM/Nav and external clients')
        
    def setup_publishers(self):
        """Setup ROS2 publishers for robot status and control"""
        
        # Sensor data for external clients
        self.sensor_pub = self.create_publisher(SensorData, '/robot/sensors', 10)
            
        # Direct robot control (used by this node only)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('📡 Robot interface publishers initialized')
        
    def setup_subscribers(self):
        """Setup ROS2 subscribers for sensor input"""
        
        # Camera status monitoring
        self.camera_sub = self.create_subscription(
            Image, '/realsense/camera/color/image_raw', self.camera_callback, 10)
            
        # Lidar input
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Odometry input for precision movement
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
            
        self.get_logger().info('📡 Robot interface subscribers initialized')
        
    def execute_command_callback(self, request, response):
        """
        Command Gateway Service Callback
        Accepts commands from external clients
        """
        command = request.command
        
        self.get_logger().info(f'🚪 Gateway: Received {command.command_type} from {command.source_node}')
        
        # Validate robot ID
        if command.robot_id != self.robot_id:
            response.success = False
            response.result_message = f"Invalid robot ID: {command.robot_id}"
            return response
        
        try:
            # Execute command
            success = self.execute_command(command)
            
            # Prepare response
            response.success = success
            response.result_message = f"Command {command.command_type} {'executed' if success else 'failed'}"
            
            self.get_logger().info(f'✅ Command {command.command_type} {"executed" if success else "failed"}')
            
        except Exception as e:
            response.success = False
            response.result_message = f"Command execution error: {str(e)}"
            self.get_logger().error(f'❌ Command execution error: {str(e)}')
        
        return response
    
    def execute_command(self, command):
        """Execute robot commands - simplified interface"""
        try:
            if command.command_type == 'move':
                return self.execute_movement(command)
            elif command.command_type == 'turn':
                return self.execute_turn(command)
            elif command.command_type == 'stop':
                return self.execute_stop()
            else:
                self.get_logger().warning(f'❓ Unknown command type: {command.command_type}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Command execution error: {str(e)}')
            return False
    
    def execute_movement(self, command):
        """Execute linear movement with encoder feedback"""
        try:
            # Stop any existing movement
            if self.movement_active:
                self.execute_stop()
                time.sleep(0.1)
            
            # Wait for odometry
            if self.current_pose is None:
                self.get_logger().warning('⚠️ Waiting for odometry data...')
                timeout = 5.0
                start_wait = time.time()
                while self.current_pose is None and (time.time() - start_wait) < timeout:
                    time.sleep(0.1)
                
                if self.current_pose is None:
                    self.get_logger().error('❌ No odometry data available')
                    return False
            
            # Extract movement parameters
            distance = getattr(command, 'distance', 0.5)  # meters
            if distance <= 0:
                distance = 0.5
            else:
                distance = distance / 1000.0 if distance > 10 else distance  # Convert mm to m if needed
                
            linear_speed = getattr(command, 'linear_speed', 0.2)
            if linear_speed <= 0:
                linear_speed = 0.2
                
            duration = getattr(command, 'duration', 0.0)
            
            # Calculate velocities
            vx = command.linear_x * linear_speed
            vy = command.linear_y * linear_speed
            
            self.get_logger().info(f'🎯 Movement: distance={distance:.3f}m, speed={linear_speed}m/s')
            
            # Start movement in separate thread
            self.stop_movement.clear()
            self.movement_thread = threading.Thread(
                target=self._movement_worker,
                args=(vx, vy, distance, duration)
            )
            self.movement_active = True
            self.movement_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Movement error: {str(e)}')
            return False
    
    def execute_turn(self, command):
        """Execute rotational movement with encoder feedback"""
        try:
            # Stop any existing movement
            if self.movement_active:
                self.execute_stop()
                time.sleep(0.1)
            
            # Wait for odometry
            if self.current_pose is None:
                self.get_logger().warning('⚠️ Waiting for odometry data...')
                timeout = 5.0
                start_wait = time.time()
                while self.current_pose is None and (time.time() - start_wait) < timeout:
                    time.sleep(0.1)
                
                if self.current_pose is None:
                    self.get_logger().error('❌ No odometry data available')
                    return False
            
            # Extract turning parameters
            angular_deg = getattr(command, 'angular', 90.0)
            if angular_deg == 0:
                angular_deg = 90.0
                
            angular_speed = getattr(command, 'angular_speed', 0.5)
            if angular_speed <= 0:
                angular_speed = 0.5
                
            duration = getattr(command, 'duration', 0.0)
            
            # Convert to radians and determine direction
            angular_rad = math.radians(angular_deg)
            angular_velocity = angular_speed * math.copysign(1, angular_deg)
            
            self.get_logger().info(f'🔄 Turn: angle={angular_deg}°, speed={angular_speed}rad/s')
            
            # Start turning in separate thread
            self.stop_movement.clear()
            self.movement_thread = threading.Thread(
                target=self._turn_worker,
                args=(angular_velocity, angular_rad, duration)
            )
            self.movement_active = True
            self.movement_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Turn error: {str(e)}')
            return False
    
    def execute_stop(self):
        """Execute immediate stop command"""
        try:
            # Signal any running movement thread to stop
            self.stop_movement.set()
            
            # Wait for movement thread to finish
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join(timeout=1.0)
            
            # Stop the robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            
            # Clear movement state
            self.movement_active = False
            self.is_moving = False
            
            self.get_logger().info('⏹️ Robot stopped')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Stop command error: {str(e)}')
            return False
    
    def _movement_worker(self, vx, vy, target_distance_m, max_duration):
        """Worker thread for odometry-based linear movement"""
        try:
            start_pose = self.current_pose
            start_time = time.time()
            
            # Create velocity command
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = 0.0
            
            # Start movement
            self.cmd_vel_pub.publish(twist)
            
            while not self.stop_movement.is_set():
                # Check duration limit
                if max_duration > 0 and (time.time() - start_time) > max_duration:
                    self.get_logger().info('⏰ Movement duration limit reached')
                    break
                
                # Calculate distance traveled
                if self.current_pose and start_pose:
                    dx = self.current_pose.position.x - start_pose.position.x
                    dy = self.current_pose.position.y - start_pose.position.y
                    distance_traveled = math.sqrt(dx*dx + dy*dy)
                    
                    # Check if target reached
                    if distance_traveled >= target_distance_m:
                        self.get_logger().info(f'🎯 Target distance reached: {distance_traveled:.3f}m')
                        break
                
                # Continue publishing velocity
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)  # 20Hz update rate
            
            # Stop movement
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.movement_active = False
            
        except Exception as e:
            self.get_logger().error(f'❌ Movement worker error: {str(e)}')
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.movement_active = False
    
    def _turn_worker(self, angular_velocity, target_angle_rad, max_duration):
        """Worker thread for odometry-based rotational movement"""
        try:
            start_pose = self.current_pose
            start_time = time.time()
            
            # Extract initial yaw angle
            def get_yaw_from_quaternion(q):
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                return math.atan2(siny_cosp, cosy_cosp)
            
            start_yaw = get_yaw_from_quaternion(start_pose.orientation)
            
            # Create angular velocity command
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = angular_velocity
            
            # Start rotation
            self.cmd_vel_pub.publish(twist)
            
            while not self.stop_movement.is_set():
                # Check duration limit
                if max_duration > 0 and (time.time() - start_time) > max_duration:
                    self.get_logger().info('⏰ Turn duration limit reached')
                    break
                
                # Calculate angle turned
                if self.current_pose:
                    current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
                    
                    # Calculate signed angle difference
                    angle_diff_raw = current_yaw - start_yaw
                    
                    # Normalize angle to [-pi, pi] range
                    while angle_diff_raw > math.pi:
                        angle_diff_raw -= 2 * math.pi
                    while angle_diff_raw < -math.pi:
                        angle_diff_raw += 2 * math.pi
                    
                    # Check if target reached
                    if abs(angle_diff_raw) >= abs(target_angle_rad):
                        self.get_logger().info(f'🎯 Turn completed: {math.degrees(angle_diff_raw):.1f}°')
                        break
                
                # Continue publishing angular velocity
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)  # 20Hz update rate
            
            # Stop rotation
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.movement_active = False
            
        except Exception as e:
            self.get_logger().error(f'❌ Turn worker error: {str(e)}')
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.movement_active = False
    
    def publish_sensor_data(self):
        """Publish sensor data for external clients"""
        # Update timestamp
        self.sensor_data.timestamp_ns = int(time.time() * 1e9)
        
        # Update CPU temperature
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                self.sensor_data.cpu_temp = temp_millicelsius / 1000.0
        except Exception:
            self.sensor_data.cpu_temp = 0.0
        
        # Update CPU usage
        try:
            self.sensor_data.cpu_usage = psutil.cpu_percent(interval=None)
        except Exception:
            self.sensor_data.cpu_usage = 0.0

        # Get battery voltage from main driver via service
        # Use a cached value approach to avoid blocking the timer callback
        if not hasattr(self, '_last_battery_update'):
            self._last_battery_update = 0
            self._cached_battery_voltage = 0.0
            self._battery_request_pending = False
        
        current_time = time.time()
        
        # Update battery voltage every 2 seconds (less frequent than sensor publishing)
        if current_time - self._last_battery_update > 2.0 and not self._battery_request_pending:
            if self.battery_client.service_is_ready():
                try:
                    self._battery_request_pending = True
                    request = GetBatteryVoltage.Request()
                    future = self.battery_client.call_async(request)
                    
                    # Add callback to handle response asynchronously
                    def battery_response_callback(future):
                        try:
                            response = future.result()
                            if response.success:
                                self._cached_battery_voltage = response.voltage
                                if not hasattr(self, '_battery_connected_logged'):
                                    self.get_logger().info(f"Battery service connected - voltage: {response.voltage:.2f}V")
                                    self._battery_connected_logged = True
                            else:
                                self.get_logger().warning(f"Battery service failed: {response.message}")
                        except Exception as e:
                            self.get_logger().warning(f"Battery service error: {str(e)}")
                        finally:
                            self._battery_request_pending = False
                            self._last_battery_update = time.time()
                    
                    future.add_done_callback(battery_response_callback)
                    
                except Exception as e:
                    self.get_logger().warning(f"Failed to call battery service: {str(e)}")
                    self._battery_request_pending = False
            else:
                if not hasattr(self, '_service_warning_shown'):
                    self.get_logger().warning("Battery service not ready")
                    self._service_warning_shown = True
                self._last_battery_update = current_time  # Don't spam warnings
        
        # Use cached battery voltage
        self.sensor_data.battery_voltage = self._cached_battery_voltage

        # Publish sensor data
        self.sensor_pub.publish(self.sensor_data)
    
    def camera_callback(self, msg):
        """Handle camera input for status monitoring"""
        self.sensor_data.camera_status = "active"
    
    def scan_callback(self, msg):
        """Handle lidar scan input"""
        if len(msg.ranges) > 0:
            # COORDINATE FRAME CORRECTION for sensor message only:
            # Lidar 0° points backward from robot perspective, so we need to map:
            # - Robot front = Lidar 180° (index 0, since lidar goes from -180° to +180°)
            # - Robot left = Lidar 90° (index 3/4 of array) 
            # - Robot right = Lidar -90° (index 1/4 of array)
            
            front_idx = 0  # Lidar -180°/180° = Robot front
            left_idx = len(msg.ranges) * 3 // 4   # Lidar 90° = Robot left
            right_idx = len(msg.ranges) // 4      # Lidar -90° = Robot right


    def odom_callback(self, msg):
        """Handle odometry updates for precision movement"""
        self.current_pose = msg.pose.pose

    
    def __del__(self):
        """Cleanup robot interface"""
        try:
            if hasattr(self, 'movement_active') and self.movement_active:
                self.execute_stop()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Robot Interface Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
