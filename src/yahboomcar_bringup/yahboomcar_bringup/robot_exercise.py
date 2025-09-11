#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys

class RobotExercise(Node):
    def __init__(self):
        super().__init__('robot_exercise')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        
        # Current robot state
        self.current_pose = None
        self.current_heading = 0.0
        
        # Movement parameters (defaults)
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 30.0  # deg/s (converted to rad/s when used)
        self.target_distance = 0.5  # meters
        self.target_rotation = 90.0  # degrees
        
        # Movement tracking
        self.start_pose = None
        self.start_heading = 0.0
        self.is_moving = False
        
        # Movement correction factors (system defaults)
        self.hardware_linear_x_cal = 1.0     # Hardware driver calibration
        self.hardware_linear_y_cal = 1.0     # Hardware driver calibration
        self.hardware_angular_cal = 1.0      # Hardware driver calibration
        self.odom_linear_scale_x = 1.0       # Odometry scale factor
        self.odom_linear_scale_y = 1.0       # Odometry scale factor
        self.odom_angular_scale = 1.0        # Odometry scale factor
        self.hardware_velocity_correction = 1.52  # Hardware velocity under-reporting correction
        
        # Try to get actual parameters from the system (if available)
        self.get_correction_factors()
        
        self.get_logger().info("Robot Exercise Program Started")
        self.get_logger().info("Waiting for odometry data on /odom_raw...")
        
        # Wait for initial odometry - this should work if base_node is functioning
        timeout_counter = 0
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_counter += 1
            
            # Print helpful messages every 5 seconds
            if timeout_counter % 50 == 0:
                self.get_logger().warning("Still waiting for odometry data...")
            
            # Timeout after 10 seconds - fail fast if there's a real problem
            if timeout_counter > 100:
                self.get_logger().error("CRITICAL: No odometry data received from /odom_raw!")
                raise RuntimeError("No odometry data received - base_node not functioning properly")
            
        self.get_logger().info("Odometry received. Ready for commands.")
        
    def get_correction_factors(self):
        """Load correction factors from parameter file"""
        try:
            self.get_logger().debug("Loading movement correction factors from parameter file...")
            
            # Declare parameters from robot_calibration.yaml
            self.declare_parameter('hardware_calibration.linear_x_cal_factor', 1.0)
            self.declare_parameter('hardware_calibration.linear_y_cal_factor', 1.0)
            self.declare_parameter('hardware_calibration.angular_cal_factor', 1.0)
            self.declare_parameter('odometry_scaling.linear_scale_x', 1.0)
            self.declare_parameter('odometry_scaling.linear_scale_y', 1.0)
            self.declare_parameter('odometry_scaling.angular_scale', 1.0)
            self.declare_parameter('velocity_corrections.linear_velocity_correction', 1.52)
            self.declare_parameter('velocity_corrections.angular_velocity_correction', 1.0)
            
            # Load parameters
            self.hardware_linear_x_cal = self.get_parameter('hardware_calibration.linear_x_cal_factor').get_parameter_value().double_value
            self.hardware_linear_y_cal = self.get_parameter('hardware_calibration.linear_y_cal_factor').get_parameter_value().double_value
            self.hardware_angular_cal = self.get_parameter('hardware_calibration.angular_cal_factor').get_parameter_value().double_value
            self.odom_linear_scale_x = self.get_parameter('odometry_scaling.linear_scale_x').get_parameter_value().double_value
            self.odom_linear_scale_y = self.get_parameter('odometry_scaling.linear_scale_y').get_parameter_value().double_value
            self.odom_angular_scale = self.get_parameter('odometry_scaling.angular_scale').get_parameter_value().double_value
            self.hardware_velocity_correction = self.get_parameter('velocity_corrections.linear_velocity_correction').get_parameter_value().double_value
            
            self.get_logger().debug("Movement correction factors loaded successfully from parameter file")
            self.get_logger().debug(f"Hardware calibration: X={self.hardware_linear_x_cal:.3f}, Y={self.hardware_linear_y_cal:.3f}, Angular={self.hardware_angular_cal:.3f}")
            self.get_logger().debug(f"Odometry scaling: X={self.odom_linear_scale_x:.3f}, Y={self.odom_linear_scale_y:.3f}, Angular={self.odom_angular_scale:.3f}")
            self.get_logger().debug(f"Velocity correction: {self.hardware_velocity_correction:.3f}x")
            
        except Exception as e:
            self.get_logger().warning(f"Could not load correction factors from parameters: {str(e)}")
        
    def odom_callback(self, msg):
        """Update current robot pose and heading from odometry"""
        self.current_pose = msg.pose.pose.position
        
        # Convert quaternion to yaw angle
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)
    
    def stop_robot(self):
        """Send stop command to robot"""
        try:
            stop_msg = Twist()
            if rclpy.ok() and self.cmd_vel_pub is not None:
                self.cmd_vel_pub.publish(stop_msg)
            self.is_moving = False
        except Exception as e:
            # Ignore shutdown errors - robot will stop naturally
            pass
        
    def move_forward_backward(self, distance):
        """Move robot forward (positive) or backward (negative) by specified distance relative to current heading"""
        if self.current_pose is None:
            self.get_logger().error("No odometry data available!")
            return False
            
        self.get_logger().info(f"Moving {'forward' if distance > 0 else 'backward'} {abs(distance):.3f}m at {self.linear_velocity}m/s")
        
        # Record starting position and heading
        self.start_pose = self.current_pose
        start_heading = self.current_heading
        target_distance = abs(distance)
        direction = 1.0 if distance > 0 else -1.0
        
        # Create movement command
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_velocity * direction
        
        self.is_moving = True
        start_time = time.time()
        
        while self.is_moving:
            # Calculate distance traveled using simple Euclidean distance
            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            
            # Use Euclidean distance instead of projection
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            
            # DEBUG OUTPUT
            self.get_logger().info(f"DEBUG: dx={dx:.3f}, dy={dy:.3f}, distance_traveled={distance_traveled:.3f}, target={target_distance:.3f}")
            
            if distance_traveled >= target_distance:
                self.stop_robot()
                self.get_logger().info(f"Movement complete. Traveled {distance_traveled:.3f}m")
                break
                
            # Safety timeout
            if time.time() - start_time > 10.0:
                self.stop_robot()
                self.get_logger().warning("Movement timeout!")
                break
                
            # Continue movement
            self.cmd_vel_pub.publish(move_cmd)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
            
        return True
    
    def strafe_left_right(self, distance):
        """Strafe robot right (positive) or left (negative) by specified distance relative to current heading"""
        if self.current_pose is None:
            self.get_logger().error("No odometry data available!")
            return False
            
        self.get_logger().info(f"Strafing {'right' if distance > 0 else 'left'} {abs(distance):.3f}m at {self.linear_velocity}m/s")
        
        # Record starting position and heading
        self.start_pose = self.current_pose
        start_heading = self.current_heading
        target_distance = abs(distance)
        direction = 1.0 if distance > 0 else -1.0
        
        # Create movement command
        move_cmd = Twist()
        move_cmd.linear.y = -self.linear_velocity * direction  # Negative because positive y is left
        
        self.is_moving = True
        start_time = time.time()
        
        while self.is_moving:
            # Calculate distance traveled perpendicular to initial heading
            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            
            # Project displacement onto direction perpendicular to initial heading (strafe direction)
            distance_traveled = -dx * math.sin(start_heading) + dy * math.cos(start_heading)
            distance_traveled = abs(distance_traveled)  # Take absolute value for comparison
            
            if distance_traveled >= target_distance:
                self.stop_robot()
                self.get_logger().info(f"Strafe complete. Traveled {distance_traveled:.3f}m")
                break
                
            # Safety timeout
            if time.time() - start_time > 10.0:
                self.stop_robot()
                self.get_logger().warning("Strafe timeout!")
                break
                
            # Continue movement
            self.cmd_vel_pub.publish(move_cmd)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
            
        return True
    
    def turn_cw_ccw(self, angle_degrees):
        """Turn robot clockwise (positive) or counter-clockwise (negative) by specified angle relative to current heading"""
        if self.current_pose is None:
            self.get_logger().error("No odometry data available!")
            return False
            
        self.get_logger().info(f"Turning {'CW' if angle_degrees > 0 else 'CCW'} {abs(angle_degrees):.1f}° at {self.angular_velocity}°/s")
        
        # Record starting heading
        self.start_heading = self.current_heading
        target_angle = math.radians(abs(angle_degrees))
        direction = 1.0 if angle_degrees > 0 else -1.0
        angular_vel_rad = math.radians(self.angular_velocity)
        
        # Create movement command
        move_cmd = Twist()
        move_cmd.angular.z = angular_vel_rad * direction
        
        self.is_moving = True
        start_time = time.time()
        
        while self.is_moving:
            # Update odometry data
            rclpy.spin_once(self, timeout_sec=0.0)
            
            # Calculate angle turned relative to starting heading
            angle_diff = self.current_heading - self.start_heading
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Check if we've turned the correct amount in the correct direction
            angle_turned = abs(angle_diff)
            
            if angle_turned >= target_angle:
                self.stop_robot()
                self.get_logger().info(f"Turn complete. Turned {math.degrees(angle_turned):.1f}°")
                break
                
            # Safety timeout
            if time.time() - start_time > 15.0:
                self.stop_robot()
                self.get_logger().warning("Turn timeout!")
                break
                
            # Continue movement
            self.cmd_vel_pub.publish(move_cmd)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
            
        return True
    
    def print_status(self):
        """Print current settings and robot status"""
        # Spin once to get latest odometry data
        rclpy.spin_once(self, timeout_sec=0.1)
        
        print("\n" + "="*70)
        print("ROBOT EXERCISE PROGRAM - CURRENT SETTINGS & CORRECTION FACTORS")
        print("="*70)
        print(f"Linear Velocity:    {self.linear_velocity:.3f} m/s")
        print(f"Angular Velocity:   {self.angular_velocity:.1f} deg/s")
        print(f"Target Distance:    {self.target_distance:.3f} m")
        print(f"Target Rotation:    {self.target_rotation:.1f} deg")
        print("-"*70)
        if self.current_pose:
            print(f"Current Position:   x={self.current_pose.x:.3f}m, y={self.current_pose.y:.3f}m")
            print(f"Current Heading:    {math.degrees(self.current_heading):.1f}°")
        else:
            print("Current Position:   [No odometry data]")
        print("-"*70)
        print("MOVEMENT CORRECTION FACTORS:")
        print(f"Hardware Driver Calibration:")
        print(f"  • Linear X Factor:     {self.hardware_linear_x_cal:.3f}")
        print(f"  • Linear Y Factor:     {self.hardware_linear_y_cal:.3f}")
        print(f"  • Angular Factor:      {self.hardware_angular_cal:.3f}")
        print(f"Odometry Scale Factors:")
        print(f"  • Linear X Scale:      {self.odom_linear_scale_x:.3f}")
        print(f"  • Linear Y Scale:      {self.odom_linear_scale_y:.3f}")
        print(f"  • Angular Scale:       {self.odom_angular_scale:.3f}")
        print(f"Hardware Velocity Correction:")
        print(f"  • Velocity Correction: {self.hardware_velocity_correction:.3f}x (compensates under-reporting)")
        print(f"Combined Effective Multipliers:")
        print(f"  • Linear X Effective:  {self.hardware_linear_x_cal * self.odom_linear_scale_x:.3f}")
        print(f"  • Linear Y Effective:  {self.hardware_linear_y_cal * self.odom_linear_scale_y:.3f}")
        print(f"  • Angular Effective:   {self.hardware_angular_cal * self.odom_angular_scale:.3f}")
        print("="*70)
    
    def print_help(self):
        """Print available commands"""
        print("\n🤖 ROBOT EXERCISE PROGRAM - COMMANDS")
        print("="*50)
        print("PARAMETER SETTING:")
        print("  lv <speed>     - Set linear velocity (m/s)")
        print("  av <speed>     - Set angular velocity (deg/s)")
        print("  d <distance>   - Set target distance (m)")
        print("  r <rotation>   - Set target rotation (deg)")
        print()
        print("MOVEMENT COMMANDS:")
        print("  move           - Move forward/backward (+ = forward, - = backward)")
        print("  strafe         - Strafe left/right (+ = right, - = left)")
        print("  turn           - Turn CW/CCW (+ = CW, - = CCW)")
        print()
        print("CONTROL COMMANDS:")
        print("  stop           - Emergency stop")
        print("  status         - Show current settings")
        print("  help           - Show this help")
        print("  quit           - Exit program")
        print("="*50)
    
    def run_interactive(self):
        """Run interactive command loop"""
        print("\n🤖 ROBOT EXERCISE PROGRAM STARTED")
        self.print_help()
        
        while rclpy.ok():
            try:
                self.print_status()
                command = input("\nEnter command: ").strip().lower().split()
                
                if not command:
                    continue
                    
                cmd = command[0]
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'help' or cmd == 'h':
                    self.print_help()
                elif cmd == 'status' or cmd == 's':
                    continue  # Status printed at top of loop
                elif cmd == 'stop':
                    self.stop_robot()
                    print("🛑 Robot stopped")
                elif cmd == 'lv' and len(command) > 1:
                    try:
                        self.linear_velocity = float(command[1])
                        print(f"✅ Linear velocity set to {self.linear_velocity:.3f} m/s")
                    except ValueError:
                        print("❌ Invalid velocity value")
                elif cmd == 'av' and len(command) > 1:
                    try:
                        self.angular_velocity = float(command[1])
                        print(f"✅ Angular velocity set to {self.angular_velocity:.1f} deg/s")
                    except ValueError:
                        print("❌ Invalid velocity value")
                elif cmd == 'd' and len(command) > 1:
                    try:
                        self.target_distance = float(command[1])
                        print(f"✅ Target distance set to {self.target_distance:.3f} m")
                    except ValueError:
                        print("❌ Invalid distance value")
                elif cmd == 'r' and len(command) > 1:
                    try:
                        self.target_rotation = float(command[1])
                        print(f"✅ Target rotation set to {self.target_rotation:.1f} deg")
                    except ValueError:
                        print("❌ Invalid rotation value")
                elif cmd == 'move':
                    print(f"🚀 Executing move: {self.target_distance:.3f}m")
                    self.move_forward_backward(self.target_distance)
                elif cmd == 'strafe':
                    print(f"🚀 Executing strafe: {self.target_distance:.3f}m")
                    self.strafe_left_right(self.target_distance)
                elif cmd == 'turn':
                    print(f"🚀 Executing turn: {self.target_rotation:.1f}°")
                    self.turn_cw_ccw(self.target_rotation)
                else:
                    print("❌ Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")
                
        self.stop_robot()
        print("\n👋 Robot Exercise Program Terminated")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_exercise = RobotExercise()
        robot_exercise.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        if 'robot_exercise' in locals():
            robot_exercise.stop_robot()
            robot_exercise.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
