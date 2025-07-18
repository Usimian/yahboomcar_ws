#!/usr/bin/env python3
"""
Debug Monitor for Autonomous Navigation
Displays real-time information about robot state and sensor data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math


class DebugMonitor(Node):
    """Debug monitor for autonomous navigation"""

    def __init__(self):
        super().__init__('debug_monitor')

        # QoS profiles
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, self.sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/auto_drive/pose', self.pose_callback, 10)
        self.joy_sub = self.create_subscription(
            Bool, '/JoyState', self.joy_callback, 10)
        self.battery_sub = self.create_subscription(
            Float32, '/voltage', self.battery_callback, 10)

        # State variables
        self.current_cmd = None
        self.current_pose = None
        self.laser_data = None
        self.joy_state = None
        self.battery_voltage = None
        
        # Position filtering for stability
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.pose_change_threshold = 0.01  # 1cm threshold for position changes

        # Create timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info('Debug Monitor started')

    def cmd_vel_callback(self, msg):
        """Track velocity commands"""
        self.current_cmd = msg

    def laser_callback(self, msg):
        """Track laser data"""
        self.laser_data = msg

    def pose_callback(self, msg):
        """Track robot pose with filtering for stability"""
        self.current_pose = msg

    def joy_callback(self, msg):
        """Track joystick state"""
        self.joy_state = msg

    def battery_callback(self, msg):
        """Track battery voltage"""
        self.battery_voltage = msg.data

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

        return min_distance if min_distance != float('inf') else 0.0

    def get_battery_status(self):
        """Get battery status string"""
        if self.battery_voltage is None:
            return "UNKNOWN"
        
        voltage = self.battery_voltage
        if voltage > 12.0:
            return f"HIGH ({voltage:.2f}V)"
        elif voltage > 11.0:
            return f"GOOD ({voltage:.2f}V)"
        elif voltage > 10.0:
            return f"LOW ({voltage:.2f}V)"
        else:
            return f"CRITICAL ({voltage:.2f}V)"

    def get_stable_position(self):
        """Get stable position display (filters out small changes)"""
        if not self.current_pose:
            return None, None
            
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Initialize if this is the first reading
        if self.last_pose_x == 0.0 and self.last_pose_y == 0.0:
            self.last_pose_x = current_x
            self.last_pose_y = current_y
            return self.last_pose_x, self.last_pose_y
        
        # Check if position changed significantly
        x_change = abs(current_x - self.last_pose_x)
        y_change = abs(current_y - self.last_pose_y)
        
        # Use larger threshold for large position values to account for odometry drift
        threshold = max(self.pose_change_threshold, abs(current_x) * 0.001, abs(current_y) * 0.001)
        
        if x_change > threshold or y_change > threshold:
            self.last_pose_x = current_x
            self.last_pose_y = current_y
            
        return self.last_pose_x, self.last_pose_y

    def print_status(self):
        """Print current status"""
        print("\n" + "="*70)
        print("AUTONOMOUS NAVIGATION DEBUG STATUS")
        print("="*70)

        # Joystick state
        if self.joy_state is not None:
            mode = "MANUAL" if self.joy_state.data else "AUTONOMOUS"
            print(f"Mode: {mode}")
        else:
            print("Mode: UNKNOWN (no JoyState data)")

        # Battery level
        battery_status = self.get_battery_status()
        print(f"Battery: {battery_status}")

        # Current command with sideways movement
        if self.current_cmd:
            linear_x = self.current_cmd.linear.x
            linear_y = self.current_cmd.linear.y  # Sideways movement
            angular_z = self.current_cmd.angular.z
            
            print(f"Velocity Commands:")
            print(f"  Forward/Backward: {linear_x:.3f} m/s")
            print(f"  Sideways (Left/Right): {linear_y:.3f} m/s")
            print(f"  Rotation: {angular_z:.3f} rad/s")
            
            # Calculate total speed
            total_linear_speed = math.sqrt(linear_x**2 + linear_y**2)
            print(f"  Total Speed: {total_linear_speed:.3f} m/s")
            
            # Movement direction
            if abs(linear_x) > 0.01 or abs(linear_y) > 0.01:
                direction_angle = math.degrees(math.atan2(linear_y, linear_x))
                print(f"  Direction: {direction_angle:.1f}° (0°=forward, 90°=left)")
        else:
            print("Velocity Commands: No commands received")

        # Stable pose
        stable_x, stable_y = self.get_stable_position()
        if stable_x is not None and stable_y is not None:
            print(f"Position: X={stable_x:.3f}m, Y={stable_y:.3f}m")
        else:
            print("Position: No pose data")

        # Laser distances with corrected angle calculations
        if self.laser_data:
            # IMPORTANT: Lidar coordinate system appears to be rotated 180°
            # What we call "back" is actually the front of the robot
            # Adjusting sectors accordingly:
            front = self.get_sector_distance(150, -150)      # Back sector (wrap-around) = actual front
            left = self.get_sector_distance(-120, -60)       # Right sector = actual left  
            right = self.get_sector_distance(60, 120)        # Left sector = actual right
            back = self.get_sector_distance(-30, 30)         # Front sector = actual back
            
            print(f"Obstacle Distances:")
            print(f"  Front: {front:.3f}m")
            print(f"  Left:  {left:.3f}m")
            print(f"  Right: {right:.3f}m")
            print(f"  Back:  {back:.3f}m")
            
            # Debug: Show raw angle range and some sample readings
            if hasattr(self.laser_data, 'angle_min') and hasattr(self.laser_data, 'angle_max'):
                angle_min_deg = math.degrees(self.laser_data.angle_min)
                angle_max_deg = math.degrees(self.laser_data.angle_max)
                num_readings = len(self.laser_data.ranges)
                print(f"  Lidar: {angle_min_deg:.1f}° to {angle_max_deg:.1f}° ({num_readings} readings)")
                
                # Show some sample readings for debugging
                if num_readings > 0:
                    front_idx = num_readings // 2  # Middle should be 0° 
                    back_idx = 0  # First reading should be -180°
                    quarter_idx = num_readings // 4  # Should be -90°
                    
                    if front_idx < len(self.laser_data.ranges):
                        front_reading = self.laser_data.ranges[front_idx]
                        print(f"  Debug: 0° reading[{front_idx}] = {front_reading:.3f}m")
                    
                    if back_idx < len(self.laser_data.ranges):
                        back_reading = self.laser_data.ranges[back_idx]
                        print(f"  Debug: -180° reading[{back_idx}] = {back_reading:.3f}m")
        else:
            print("Obstacle Distances: No laser data")

        print("="*70)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        monitor = DebugMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 