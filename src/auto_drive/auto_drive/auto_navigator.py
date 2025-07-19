#!/usr/bin/env python3
"""
Autonomous Navigator for Yahboomcar
Integrates lidar, IMU, and wheel encoders for localization and mapping
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import math
import signal
import sys
from collections import deque
from dataclasses import dataclass

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


@dataclass
class Pose2D:
    """2D pose representation"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class OccupancyCell:
    """Occupancy grid cell"""
    probability: float = 0.5  # 0.0 = free, 1.0 = occupied
    last_update: float = 0.0


class AutonomousNavigator(Node):
    """Main autonomous driving node"""

    def __init__(self):
        super().__init__('auto_navigator')

        # State variables
        self.current_pose = Pose2D()
        self.velocity = Twist()
        self.is_active = False
        self.emergency_stop = False
        self.initialized = False
        self.shutting_down = False

        # Sensor data
        self.laser_data = None
        self.imu_data = None
        self.odom_data = None

        # Localization
        self.pose_history = deque(maxlen=100)
        self.last_odom_time = 0.0
        self.last_imu_time = 0.0

        # Navigation parameters - declare first
        self.declare_navigation_parameters()

        # Mapping
        self.map_resolution = 0.05  # 5cm per cell
        self.map_width = 400  # 20m x 20m map
        self.map_height = 400
        self.occupancy_grid = {}  # Sparse representation
        self.map_origin_x = -10.0  # Map center at robot start
        self.map_origin_y = -10.0
        # Get mapping parameters from config
        self.map_decay_time = float(self.get_parameter('map_decay_time').value or 10.0)
        self.map_decay_rate = float(self.get_parameter('map_decay_rate').value or 0.01)

        # QoS profiles
        self.sensor_qos = QoSProfile(depth=1,
                                     reliability=ReliabilityPolicy.BEST_EFFORT)
        self.control_qos = QoSProfile(depth=10,
                                      reliability=ReliabilityPolicy.RELIABLE)
        self.map_qos = QoSProfile(depth=1,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/auto_drive/map', self.map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/auto_drive/pose', 10)

        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, self.sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, self.sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.sensor_qos)
        self.joy_sub = self.create_subscription(
            Bool, '/JoyState', self.joy_callback, 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.mapping_timer = self.create_timer(0.2, self.update_map)    # 5 Hz
        self.localization_timer = self.create_timer(0.05,
                                                    self.update_localization)  # 20 Hz
        self.manual_monitor_timer = self.create_timer(2.0, self.manual_mode_monitor)  # 0.5 Hz (every 2 seconds)

        # Initialize autonomous mode based on parameter
        self.init_timer = self.create_timer(2.0, self.initialize_auto_mode)  # Wait 2 seconds for startup

        self.get_logger().info('Autonomous Navigator initialized')

    def declare_navigation_parameters(self):
        """Declare ROS navigation parameters"""
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('safety_distance', 0.8)
        self.declare_parameter('emergency_distance', 0.4)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('enable_autonomous', True)
        self.declare_parameter('map_decay_time', 10.0)
        self.declare_parameter('map_decay_rate', 0.01)

    def get_parameters(self):
        """Get current parameters"""
        return {
            'max_speed': float(self.get_parameter('max_speed').value or 0.3),
            'max_angular_speed': float(self.get_parameter('max_angular_speed').value or 0.5),
            'safety_distance': float(self.get_parameter('safety_distance').value or 0.8),
            'emergency_distance': float(self.get_parameter('emergency_distance').value or 0.4),
            'goal_tolerance': float(self.get_parameter('goal_tolerance').value or 0.2),
            'enable_autonomous': bool(self.get_parameter('enable_autonomous').value or True),
        }

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg
        self.last_imu_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg
        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    def joy_callback(self, msg):
        """Handle joystick override"""
        if msg.data:
            self.is_active = False
            self.get_logger().info('Manual control activated')
        else:
            self.is_active = True
            self.get_logger().info('Autonomous control activated')

    def update_localization(self):
        """Update robot pose using sensor fusion"""
        if self.shutting_down or not self.odom_data or not self.imu_data:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Extract odometry pose
        odom_pose = self.odom_data.pose.pose
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y

        # Extract orientation from IMU (more accurate than wheel encoders)
        imu_orientation = self.imu_data.orientation
        _, _, yaw = self.quaternion_to_euler(
            imu_orientation.x, imu_orientation.y,
            imu_orientation.z, imu_orientation.w)

        # Simple sensor fusion: use odometry for position, IMU for orientation
        self.current_pose.x = odom_x
        self.current_pose.y = odom_y
        self.current_pose.theta = yaw

        # Store pose history
        self.pose_history.append((current_time, Pose2D(odom_x, odom_y, yaw)))

        # Publish pose
        self.publish_pose()

        # Publish TF
        self.publish_transform()

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_pose(self):
        """Publish current pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.position.x = self.current_pose.x
        pose_msg.pose.position.y = self.current_pose.y
        pose_msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.current_pose.theta)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = (math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) -
              math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        qy = (math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) +
              math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2))
        qz = (math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) -
              math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2))
        qw = (math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) +
              math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        return qx, qy, qz, qw

    def publish_transform(self):
        """Publish TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.current_pose.x
        t.transform.translation.y = self.current_pose.y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.current_pose.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def update_map(self):
        """Update occupancy grid map"""
        if self.shutting_down or not self.laser_data or not hasattr(self, 'current_pose'):
            # Always publish map even if no data to show empty/unknown map
            self.publish_map()
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Debug: Track how many points we process
        processed_points = 0
        valid_obstacles = 0

        # Process laser scan
        for i, distance in enumerate(self.laser_data.ranges):
            if (distance < self.laser_data.range_min or
                    distance > self.laser_data.range_max):
                continue

            processed_points += 1

            # Calculate angle
            angle = self.laser_data.angle_min + i * self.laser_data.angle_increment
            global_angle = angle + self.current_pose.theta

            # Calculate obstacle position
            obstacle_x = self.current_pose.x + distance * math.cos(global_angle)
            obstacle_y = self.current_pose.y + distance * math.sin(global_angle)

            # Convert to grid coordinates
            grid_x = int((obstacle_x - self.map_origin_x) / self.map_resolution)
            grid_y = int((obstacle_y - self.map_origin_y) / self.map_resolution)

            # Update occupancy grid
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                key = (grid_x, grid_y)
                if key not in self.occupancy_grid:
                    self.occupancy_grid[key] = OccupancyCell()

                # Mark as occupied
                current_prob = self.occupancy_grid[key].probability
                self.occupancy_grid[key].probability = min(1.0, current_prob + 0.1)
                self.occupancy_grid[key].last_update = current_time
                valid_obstacles += 1

            # Mark free space along the ray
            steps = int(distance / self.map_resolution)
            for step in range(1, steps):
                ray_x = self.current_pose.x + (step * self.map_resolution) * math.cos(global_angle)
                ray_y = self.current_pose.y + (step * self.map_resolution) * math.sin(global_angle)
                
                ray_grid_x = int((ray_x - self.map_origin_x) / self.map_resolution)
                ray_grid_y = int((ray_y - self.map_origin_y) / self.map_resolution)
                
                if 0 <= ray_grid_x < self.map_width and 0 <= ray_grid_y < self.map_height:
                    key = (ray_grid_x, ray_grid_y)
                    if key not in self.occupancy_grid:
                        self.occupancy_grid[key] = OccupancyCell()
                    
                    # Mark as free
                    current_prob = self.occupancy_grid[key].probability
                    self.occupancy_grid[key].probability = max(0.0, current_prob - 0.02)
                    self.occupancy_grid[key].last_update = current_time

        # Debug logging (every 50 updates to avoid spam)
        if hasattr(self, '_map_update_count'):
            self._map_update_count += 1
        else:
            self._map_update_count = 1
            
        if self._map_update_count % 50 == 0:
            self.get_logger().info(f'Map update: processed {processed_points} points, '
                                 f'{valid_obstacles} valid obstacles, '
                                 f'{len(self.occupancy_grid)} total cells')

        # Apply map decay to old data (but don't remove cells completely)
        self.apply_map_decay(current_time)

        # Always publish map
        self.publish_map()

    def apply_map_decay(self, current_time):
        """Apply time-based decay to map data"""
        # Note: We no longer remove cells completely to maintain map history
        
        for key, cell in self.occupancy_grid.items():
            time_since_update = current_time - cell.last_update
            
            # Only apply decay after decay_time has passed
            if time_since_update > self.map_decay_time:
                # Decay the certainty - move probability toward 0.5 (unknown)
                decay_amount = self.map_decay_rate * (time_since_update - self.map_decay_time)
                
                if cell.probability > 0.5:
                    # Occupied cell - decay toward unknown
                    cell.probability = max(0.5, cell.probability - decay_amount)
                elif cell.probability < 0.5:
                    # Free cell - decay toward unknown  
                    cell.probability = min(0.5, cell.probability + decay_amount)
                
                # Don't remove cells - keep them as unknown for visualization

    def publish_map(self):
        """Publish occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'odom'

        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Convert sparse grid to dense array
        data = [-1] * (self.map_width * self.map_height)  # Unknown = -1

        for (x, y), cell in self.occupancy_grid.items():
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                index = y * self.map_width + x
                # Convert probability to occupancy value (0-100)
                occupancy_value = int(cell.probability * 100)
                # Ensure values are in valid range
                data[index] = max(0, min(100, occupancy_value))

        map_msg.data = data
        self.map_pub.publish(map_msg)

    def control_loop(self):
        """Main control loop"""
        if self.shutting_down or not self.laser_data:
            return

        # Check if autonomous mode is active - JoyState overrides launch parameter
        if not self.is_active:
            # In manual mode, just return without sending any commands
            return

        params = self.get_parameters()

        if not params['enable_autonomous']:
            return

        # Emergency stop check
        if self.check_emergency_stop():
            self.emergency_stop = True
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        else:
            self.emergency_stop = False

        # Simple obstacle avoidance navigation
        cmd = self.calculate_safe_velocity()
        self.cmd_vel_pub.publish(cmd)

    def manual_mode_monitor(self):
        """Monitor and report obstacle distances in manual mode"""
        if self.shutting_down or not self.laser_data:
            return
            
        # Only show obstacle info in manual mode
        if self.is_active:  # is_active = True means autonomous mode
            return
            
        # Calculate distances in all sectors
        left_distance = self.get_sector_distance(-120, -60)      # Right sector = actual left
        right_distance = self.get_sector_distance(60, 120)       # Left sector = actual right
        front_distance = self.get_sector_distance(150, -150)     # Back sector (wrap-around) = actual front
        front_left_distance = self.get_sector_distance(-150, -120) # Back-right sector = actual front-left
        front_right_distance = self.get_sector_distance(120, 150)  # Back-left sector = actual front-right
        
        # Get minimum front distance for safety awareness
        min_front_distance = min(front_distance, front_left_distance, front_right_distance)
        
        # Create status message
        status = f"MANUAL MODE - Obstacles: Front: {min_front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m"
        
        # Add warning if obstacles are close
        params = self.get_parameters()
        if min_front_distance < params['safety_distance']:
            status += f" ⚠️  CLOSE OBSTACLE AHEAD!"
        elif left_distance < params['safety_distance'] or right_distance < params['safety_distance']:
            status += f" ⚠️  Side obstacles detected"
            
        self.get_logger().info(status)

    def check_emergency_stop(self):
        """Check if emergency stop is needed"""
        if not self.laser_data:
            return False

        params = self.get_parameters()
        min_distance = float('inf')

        # Check front sector (which is actually the back sector in lidar coordinates)
        for i, distance in enumerate(self.laser_data.ranges):
            if (distance < self.laser_data.range_min or
                    distance > self.laser_data.range_max or
                    math.isnan(distance) or math.isinf(distance)):
                continue

            angle_rad = self.laser_data.angle_min + i * self.laser_data.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normalize angle to -180 to +180 range
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            # Check actual front sector (±180° in lidar coordinates, 60° total)
            if angle_deg >= 150 or angle_deg <= -150:
                min_distance = min(min_distance, distance)

        return min_distance < params['emergency_distance']

    def calculate_safe_velocity(self):
        """Calculate safe velocity command with exploration behavior"""
        params = self.get_parameters()
        cmd = Twist()

        if not self.laser_data:
            return cmd

        # Analyze laser data in different sectors
        # IMPORTANT: Lidar coordinate system appears to be rotated 180°
        # What we call "back" is actually the front of the robot
        # Adjusting sectors accordingly:
        left_distance = self.get_sector_distance(-120, -60)      # Right sector = actual left
        right_distance = self.get_sector_distance(60, 120)       # Left sector = actual right
        front_distance = self.get_sector_distance(150, -150)     # Back sector (wrap-around) = actual front
        front_left_distance = self.get_sector_distance(-150, -120) # Back-right sector = actual front-left
        front_right_distance = self.get_sector_distance(120, 150)  # Back-left sector = actual front-right

        # Get minimum distance in front for safety
        min_front_distance = min(front_distance, front_left_distance, front_right_distance)

        # Navigation logic with exploration behavior
        if min_front_distance < params['safety_distance']:
            # Obstacle ahead - need to turn
            cmd.linear.x = 0.0
            
            # Choose turn direction based on clearer side
            if left_distance > right_distance + 0.2:  # Bias towards left
                cmd.angular.z = params['max_angular_speed'] * 0.7
                self.get_logger().info(f'Turning left - Front: {min_front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m')
            elif right_distance > left_distance + 0.2:  # Bias towards right
                cmd.angular.z = -params['max_angular_speed'] * 0.7
                self.get_logger().info(f'Turning right - Front: {min_front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m')
            else:
                # If both sides are similar, turn towards the slightly better side
                if left_distance >= right_distance:
                    cmd.angular.z = params['max_angular_speed'] * 0.5
                else:
                    cmd.angular.z = -params['max_angular_speed'] * 0.5
                self.get_logger().info(f'Obstacle ahead - turning (Front: {min_front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m)')
        
        elif front_distance > params['safety_distance'] * 2.0:
            # Lots of space ahead - move forward but with some exploration
            base_speed = min(params['max_speed'], front_distance * 0.2)
            cmd.linear.x = base_speed
            
            # Add slight random exploration turning
            import random
            if random.random() < 0.1:  # 10% chance to explore
                if left_distance > right_distance:
                    cmd.angular.z = params['max_angular_speed'] * 0.2
                else:
                    cmd.angular.z = -params['max_angular_speed'] * 0.2
                self.get_logger().info(f'Exploring while moving forward - speed: {cmd.linear.x:.2f}m/s')
            else:
                self.get_logger().info(f'Moving forward - speed: {cmd.linear.x:.2f}m/s, front: {front_distance:.2f}m')
        
        else:
            # Moderate space ahead - move forward cautiously
            cmd.linear.x = min(params['max_speed'] * 0.6, front_distance * 0.3)
            
            # Slight steering towards more open space
            if abs(left_distance - right_distance) > 0.3:
                if left_distance > right_distance:
                    cmd.angular.z = params['max_angular_speed'] * 0.1
                else:
                    cmd.angular.z = -params['max_angular_speed'] * 0.1
            
            self.get_logger().info(f'Cautious forward - speed: {cmd.linear.x:.2f}m/s, front: {front_distance:.2f}m')

        # Limit velocities to safe ranges
        cmd.linear.x = max(-params['max_speed'],
                           min(params['max_speed'], cmd.linear.x))
        cmd.angular.z = max(-params['max_angular_speed'],
                            min(params['max_angular_speed'], cmd.angular.z))

        return cmd

    def get_sector_distance(self, start_angle_deg, end_angle_deg):
        """Get minimum distance in angular sector (angles in degrees)"""
        if not self.laser_data:
            return float('inf')

        min_distance = float('inf')

        for i, distance in enumerate(self.laser_data.ranges):
            # Skip invalid readings
            if (distance < self.laser_data.range_min or
                    distance > self.laser_data.range_max or
                    math.isnan(distance) or math.isinf(distance)):
                continue

            # Calculate angle in radians, then convert to degrees
            angle_rad = self.laser_data.angle_min + i * self.laser_data.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normalize angle to -180 to +180 range
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            # Check if angle is in the desired sector
            if start_angle_deg <= end_angle_deg:
                # Normal case (e.g., -30 to 30)
                if start_angle_deg <= angle_deg <= end_angle_deg:
                    min_distance = min(min_distance, distance)
            else:
                # Wrap-around case (e.g., 150 to -150 for back sector)
                if angle_deg >= start_angle_deg or angle_deg <= end_angle_deg:
                    min_distance = min(min_distance, distance)

        return min_distance if min_distance != float('inf') else 10.0  # Return large value if no valid readings

    def initialize_auto_mode(self):
        """Initialize autonomous mode based on parameter - START IN MANUAL MODE BY DEFAULT"""
        if self.initialized:
            return
            
        # Always start in manual mode for safety - user must explicitly enable autonomous mode
        self.is_active = False
        self.get_logger().info('System started in MANUAL MODE (safe default)')
        self.get_logger().info('Use "ros2 run auto_drive auto_control enable" to activate autonomous mode')
        
        self.initialized = True

    def shutdown(self):
        """Properly shutdown the navigator"""
        if self.shutting_down:
            return
        
        self.shutting_down = True
        self.get_logger().info('Shutting down Autonomous Navigator...')
        
        # Stop the robot immediately
        stop_cmd = Twist()
        try:
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info('Robot stopped')
        except Exception as e:
            self.get_logger().warn(f'Failed to stop robot: {e}')
        
        # Cancel all timers
        try:
            if hasattr(self, 'control_timer'):
                self.control_timer.cancel()
            if hasattr(self, 'mapping_timer'):
                self.mapping_timer.cancel()
            if hasattr(self, 'localization_timer'):
                self.localization_timer.cancel()
            if hasattr(self, 'manual_monitor_timer'):
                self.manual_monitor_timer.cancel()
            if hasattr(self, 'init_timer'):
                self.init_timer.cancel()
            self.get_logger().info('All timers cancelled')
        except Exception as e:
            self.get_logger().warn(f'Error cancelling timers: {e}')
        
        # Destroy subscriptions and publishers
        try:
            if hasattr(self, 'laser_sub'):
                self.destroy_subscription(self.laser_sub)
            if hasattr(self, 'imu_sub'):
                self.destroy_subscription(self.imu_sub)
            if hasattr(self, 'odom_sub'):
                self.destroy_subscription(self.odom_sub)
            if hasattr(self, 'joy_sub'):
                self.destroy_subscription(self.joy_sub)
            self.get_logger().info('Subscriptions destroyed')
        except Exception as e:
            self.get_logger().warn(f'Error destroying subscriptions: {e}')
        
        # Clear data structures
        self.occupancy_grid.clear()
        self.pose_history.clear()
        
        self.get_logger().info('Autonomous Navigator shutdown complete')

# Global variable to store the navigator instance for signal handling
navigator_instance = None

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    global navigator_instance
    print(f"\nReceived signal {signum}. Shutting down gracefully...")
    
    if navigator_instance:
        navigator_instance.shutdown()
    
    # Give some time for cleanup
    import time
    time.sleep(1)
    
    # Force shutdown ROS
    try:
        rclpy.shutdown()
    except:
        pass
    
    sys.exit(0)

def main(args=None):
    """Main function"""
    global navigator_instance
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)

    try:
        navigator_instance = AutonomousNavigator()
        rclpy.spin(navigator_instance)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Cleaning up...")
        if navigator_instance:
            navigator_instance.shutdown()
        
        try:
            if navigator_instance:
                navigator_instance.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("Shutdown complete")

if __name__ == '__main__':
    main()
