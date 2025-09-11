#!/usr/bin/env python3
"""
Robot Calibration Test Program
Determines scaling factors for linear x, y and angular z movement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import sys


class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        # Current odometry data
        self.current_odom = None
        self.start_odom = None

        # Test results
        self.test_results = {}

        self.get_logger().info("Calibration Test Program Started")
        self.get_logger().info("Commands:")
        self.get_logger().info("  'f' - Test forward movement (0.2m)")
        self.get_logger().info("  'b' - Test backward movement (0.2m)")
        self.get_logger().info("  'l' - Test left strafe movement (0.2m)")
        self.get_logger().info("  'r' - Test right strafe movement (0.2m)")
        self.get_logger().info("  't' - Test 90° turn")
        self.get_logger().info("  's' - Stop movement")
        self.get_logger().info("  'q' - Quit and show results")

    def odom_callback(self, msg):
        self.current_odom = msg

    def wait_for_odom(self, timeout=5.0):
        """Wait for odometry data to be available"""
        start_time = time.time()
        while self.current_odom is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_odom is not None

    def get_current_position(self):
        """Get current position and orientation"""
        if self.current_odom is None:
            return None

        pos = self.current_odom.pose.pose.position
        orient = self.current_odom.pose.pose.orientation

        # Convert quaternion to yaw
        yaw = math.atan2(2*(orient.w*orient.z + orient.x*orient.y),
                        1 - 2*(orient.y*orient.y + orient.z*orient.z))

        return {
            'x': pos.x,
            'y': pos.y,
            'yaw': yaw,
            'yaw_deg': math.degrees(yaw)
        }

    def send_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=None):
        """Send velocity command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z

        if duration:
            end_time = time.time() + duration
            while time.time() < end_time:
                self.cmd_vel_publisher.publish(twist)
                time.sleep(0.1)
            # Send zero velocity to stop
            zero_twist = Twist()
            self.cmd_vel_publisher.publish(zero_twist)
        else:
            self.cmd_vel_publisher.publish(twist)

    def test_linear_movement(self, direction, distance=0.2):
        """Test linear movement in specified direction"""
        print(f"\n=== Testing {direction.upper()} movement ({distance}m) ===")
        print("Press ENTER when ready to start the test...")
        input()

        # Record starting position
        print("Recording starting position...")
        self.start_odom = self.get_current_position()
        if not self.start_odom:
            print("ERROR: Could not get odometry data")
            return

        print(f"Start: x={self.start_odom['x']:.3f}, y={self.start_odom['y']:.3f}")

        # Send movement command
        if direction == 'forward':
            self.send_velocity(linear_x=0.2, duration=1.0)  # 0.2 m/s for 1 second = 0.2m
        elif direction == 'backward':
            self.send_velocity(linear_x=-0.2, duration=1.0)
        elif direction == 'left':
            self.send_velocity(linear_y=0.2, duration=1.0)
        elif direction == 'right':
            self.send_velocity(linear_y=-0.2, duration=1.0)

        print("Movement command sent. Press ENTER when robot has stopped...")
        input()

        # Record ending position
        end_odom = self.get_current_position()
        if not end_odom:
            print("ERROR: Could not get final odometry data")
            return

        print(f"End: x={end_odom['x']:.3f}, y={end_odom['y']:.3f}")

        # Calculate actual movement
        delta_x = end_odom['x'] - self.start_odom['x']
        delta_y = end_odom['y'] - self.start_odom['y']
        actual_distance = math.sqrt(delta_x**2 + delta_y**2)

        print(f"Expected distance: {distance}m")
        print(f"Actual distance: {actual_distance:.3f}m")
        print(f"Scale factor needed: {distance/actual_distance:.3f}")

        self.test_results[f'{direction}_scale'] = distance / actual_distance

    def test_angular_movement(self, angle_deg=90.0):
        """Test angular movement"""
        print(f"\n=== Testing {angle_deg}° turn ===")
        print("Press ENTER when ready to start the test...")
        input()

        # Record starting orientation
        print("Recording starting orientation...")
        self.start_odom = self.get_current_position()
        if not self.start_odom:
            print("ERROR: Could not get odometry data")
            return

        print(f"Start yaw: {self.start_odom['yaw_deg']:.1f}°")

        # Send turn command (90 degrees = π/2 radians, at 1 rad/s for 1.57 seconds)
        self.send_velocity(angular_z=1.0, duration=1.57)

        print("Turn command sent. Press ENTER when robot has stopped...")
        input()

        # Record ending orientation
        end_odom = self.get_current_position()
        if not end_odom:
            print("ERROR: Could not get final odometry data")
            return

        print(f"End yaw: {end_odom['yaw_deg']:.1f}°")

        # Calculate actual turn
        delta_yaw = end_odom['yaw'] - self.start_odom['yaw']

        # Handle angle wrapping
        while delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        while delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi

        actual_angle_deg = abs(math.degrees(delta_yaw))

        print(f"Expected angle: {angle_deg}°")
        print(f"Actual angle: {actual_angle_deg:.1f}°")
        print(f"Scale factor needed: {angle_deg/actual_angle_deg:.3f}")

        self.test_results['angular_scale'] = angle_deg / actual_angle_deg

    def print_results(self):
        """Print calibration results and recommendations"""
        print("\n" + "="*50)
        print("CALIBRATION RESULTS")
        print("="*50)

        recommendations = []

        for test, factor in self.test_results.items():
            print(".3f")

            if 'scale' in test:
                if factor > 1.0:
                    recommendations.append(f"  {test.replace('_scale', '')}: Increase scale by {factor:.2f}x")
                else:
                    recommendations.append(f"  {test.replace('_scale', '')}: Decrease scale by {1/factor:.2f}x")

        print("\nRECOMMENDED CHANGES:")
        for rec in recommendations:
            print(rec)

        print("\nAdd these to your base_node parameters in robot_bringup.launch.py")
        print("Example:")
        print("  'linear_scale_x': 1.0 * forward_scale,")
        print("  'linear_scale_y': 1.0 * strafe_scale,")
        print("  'angular_scale': 1.0 * angular_scale,")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTest()

    if not node.wait_for_odom():
        print("ERROR: Could not get odometry data. Make sure the robot is running.")
        return

    try:
        while rclpy.ok():
            print("\nCalibration Test Menu:")
            print("f - Test forward movement (0.2m)")
            print("b - Test backward movement (0.2m)")
            print("l - Test left strafe movement (0.2m)")
            print("r - Test right strafe movement (0.2m)")
            print("t - Test 90° turn")
            print("s - Stop movement")
            print("q - Quit and show results")
            print("Choice: ", end="")

            choice = input().strip().lower()

            if choice == 'f':
                node.test_linear_movement('forward')
            elif choice == 'b':
                node.test_linear_movement('backward')
            elif choice == 'l':
                node.test_linear_movement('left')
            elif choice == 'r':
                node.test_linear_movement('right')
            elif choice == 't':
                node.test_angular_movement(90.0)
            elif choice == 's':
                node.send_velocity(0, 0, 0)
                print("Movement stopped")
            elif choice == 'q':
                node.print_results()
                break
            else:
                print("Invalid choice. Please try again.")

            # Process any pending callbacks
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.print_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
