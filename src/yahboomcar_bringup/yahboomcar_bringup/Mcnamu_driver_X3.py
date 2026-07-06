#!/usr/bin/env python
# encoding: utf-8

#public lib
# import sys
# import math
# import random
# import threading
from math import pi
import os
import time
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool,UInt8MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock
from robot_msgs.srv import GetBatteryVoltage

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		
		# Initialize hardware with proper error handling
		try:
			self.car = Rosmaster(com="/dev/myserial", debug=False)
			self.get_logger().info("Rosmaster initialized successfully")
		except Exception as e:
			self.get_logger().error(f"Failed to initialize Rosmaster: {str(e)}")
			raise
			
		# Set car type and wait for initialization
		self.car.set_car_type(1)
		self.get_logger().info("Car type set to X3")
		
		#get parameter
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 5.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)
		self.declare_parameter('debug', False)
		self.debug_enabled = self.get_parameter('debug').get_parameter_value().bool_value
		print (f"Debug mode: {self.debug_enabled}")

		# Load calibration factors from parameter file
		self.declare_parameter('calibration.linear_x_factor', 1.0)
		self.declare_parameter('calibration.linear_y_factor', 1.0)
		self.declare_parameter('calibration.angular_z_factor', 1.0)
		
		self.linear_x_factor = self.get_parameter('calibration.linear_x_factor').get_parameter_value().double_value
		self.linear_y_factor = self.get_parameter('calibration.linear_y_factor').get_parameter_value().double_value
		self.angular_z_factor = self.get_parameter('calibration.angular_z_factor').get_parameter_value().double_value
		
		self.get_logger().info(f"Calibration factors: X={self.linear_x_factor:.3f}, Y={self.linear_y_factor:.3f}, Angular={self.angular_z_factor:.3f}")

		
		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)
		self.sub_LedCommand = self.create_subscription(UInt8MultiArray,"led_command",self.LedCommandCallback,10)
		# --- Servo control (additive): PWM S1-S4 + serial bus servos ---
		self.sub_pwm_servo = self.create_subscription(UInt8MultiArray,"pwm_servo",self.pwm_servo_callback,10)
		self.sub_bus_servo = self.create_subscription(Int32MultiArray,"bus_servo",self.bus_servo_callback,10)
		self._bus_servo_enabled = False

		#create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"imu/mag",100)

		#create service
		self.battery_service = self.create_service(GetBatteryVoltage, 'get_battery_voltage', self.get_battery_voltage_callback)

		# Initialize hardware communication properly
		self.car.create_receive_threading()
		self.get_logger().info("Hardware receive threading started")
		
		# Wait for hardware to initialize
		sleep(1.0)
		
		# Enable auto reporting for sensor data
		self.car.set_auto_report_state(True, forever=False)
		self.get_logger().info("Auto reporting enabled")
		
		# Wait for auto reporting to start and data to stabilize
		sleep(2.0)
		
		# Test initial sensor readings
		try:
			test_version = self.car.get_version()
			self.get_logger().info(f"Initial sensor test - Version: {test_version:.1f}")
			
			# Read and print first IMU message
			try:
				ax, ay, az = self.car.get_accelerometer_data()
				gx, gy, gz = self.car.get_gyroscope_data()
				mx, my, mz = self.car.get_magnetometer_data()
				self.get_logger().info(f"First IMU reading - Accel: ({ax:.3f}, {ay:.3f}, {az:.3f}), Gyro: ({gx:.3f}, {gy:.3f}, {gz:.3f}), Mag: ({mx:.1f}, {my:.1f}, {mz:.1f})")
			except Exception as e:
				self.get_logger().error(f"Failed to read initial IMU data: {str(e)}")
				
		except Exception as e:
			self.get_logger().error(f"Initial sensor test failed: {str(e)}")

		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		
		# Add debug counter
		self.debug_counter = 0
		
		# FIXED: Store commanded velocities for proper feedback
		self.last_cmd_vx = 0.0
		self.last_cmd_vy = 0.0  
		self.last_cmd_angular = 0.0
		
		# Calibrate the z-gyro bias by averaging readings at rest. Movement
		# during a window corrupts the estimate, so a window is accepted only
		# if it is QUIET (at-rest sample spread is ~0.0005 rad/s; motion is
		# orders of magnitude larger) and the mean is plausible as a bias.
		# Up to 3 windows are tried. A good measurement is stored on disk;
		# if no quiet window is found (robot handled during boot), the stored
		# value from the previous boot is used — a slightly stale bias is far
		# closer to the truth than no correction. Zero only if neither exists.
		self.imu_bias_file = os.path.expanduser('~/.yahboomcar_imu_z_bias')
		self.get_logger().info("Calibrating IMU z-gyro bias...")
		self.imu_angular_bias = 0.0
		for attempt in range(3):
			bias_samples = []
			for i in range(50):  # 50 samples over 0.5 seconds
				gx, gy, gz = self.car.get_gyroscope_data()
				bias_samples.append(gz)
				time.sleep(0.01)
			mean = sum(bias_samples) / len(bias_samples)
			spread = (sum((s - mean) ** 2 for s in bias_samples) / len(bias_samples)) ** 0.5
			if spread < 0.005 and abs(mean) < 0.05:
				self.imu_angular_bias = mean
				self.get_logger().info(
					f"IMU z-gyro bias calibrated: {mean:+.5f} rad/s (spread {spread:.5f})")
				try:
					with open(self.imu_bias_file, 'w') as f:
						f.write(f"{mean:.6f}\n")
				except OSError as e:
					self.get_logger().warning(f"Could not store IMU bias: {e}")
				break
			self.get_logger().warning(
				f"IMU bias window {attempt + 1}/3 rejected (mean {mean:+.4f}, "
				f"spread {spread:.4f} rad/s) — robot moving? retrying")
			time.sleep(0.5)
		else:
			try:
				with open(self.imu_bias_file) as f:
					self.imu_angular_bias = float(f.read().strip())
				age_h = (time.time() - os.path.getmtime(self.imu_bias_file)) / 3600.0
				self.get_logger().warning(
					f"IMU bias calibration failed (robot moving during boot?) — "
					f"using stored bias {self.imu_angular_bias:+.5f} rad/s "
					f"({age_h:.1f} hours old)")
			except (OSError, ValueError):
				self.get_logger().error(
					"IMU bias calibration failed and no stored value exists — "
					"publishing uncorrected gyro (bias 0.0).")

		# Continuous recalibration: the MEMS bias wanders far beyond the
		# boot snapshot (measured +0.0004 -> +0.0045 rad/s over one warm-up
		# hour, 2026-07-05 — 16 deg/min of heading drift). Re-measure at
		# rest, passively, from the samples pub_data already reads. Any
		# drive command or measured wheel motion discards the window in
		# progress; the last good bias stays in effect until a full quiet
		# window passes the same gates as the boot calibration.
		self.recal_interval_s = 120.0   # earliest next update after a good one
		self.recal_window_size = 30     # 3 s of samples at the 10 Hz pub rate
		self.recal_window = []
		self.last_recal_time = time.time()
		self.commanded_moving = False

 		# Hardware speed feedback system (better than raw encoders)
		# The robot firmware provides processed velocity feedback that accounts for
		# wheel dynamics, slip compensation, and motor characteristics
		
		self.get_logger().info("Yahboom X3 driver initialization complete")

	def get_hardware_velocities(self, gz):
		"""Get actual velocities from hardware speed feedback"""
		vx, vy, vz_wheel = self.car.get_motion_data()
		return vx, vy, gz - self.imu_angular_bias

	def cmd_vel_callback(self,msg):
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # Apply calibration factors to ensure accurate movement
		vx = msg.linear.x * self.linear_x_factor
		vy = msg.linear.y * self.linear_y_factor
		angular = msg.angular.z * self.angular_z_factor
		self.car.set_car_motion(vx, vy, angular)

		# Any nonzero drive command aborts an in-progress bias window.
		# (Zero commands don't: idle teleop streams zeros continuously,
		# and blocking on those would prevent recalibration forever.)
		self.commanded_moving = (abs(vx) > 1e-3 or abs(vy) > 1e-3
		                         or abs(angular) > 1e-3)
		if self.commanded_moving:
			self.recal_window.clear()
		
		# FIXED: Store commanded velocities for velocity feedback
		self.last_cmd_vx = vx
		self.last_cmd_vy = vy
		self.last_cmd_angular = angular
		
	def RGBLightcallback(self,msg):
        # RGB Light control, server callback function
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	def LedCommandCallback(self, msg):
		# /led_command: flat UInt8 array of [index, R, G, B] groups.
		# index 0xFF = all LEDs. Stops any active effect first.
		if not isinstance(msg, UInt8MultiArray): return
		data = list(msg.data)
		if len(data) % 4 != 0:
			self.get_logger().warn(f"led_command length {len(data)} not multiple of 4, ignoring")
			return
		# Effect mode masks per-LED writes — clear it before any direct write.
		self.car.set_colorful_effect(0, 6, parm=1)
		for i in range(0, len(data), 4):
			idx, r, g, b = data[i], data[i+1], data[i+2], data[i+3]
			self.car.set_colorful_lamps(idx, r, g, b)

	def pwm_servo_callback(self, msg):
		# [servo_id (1-4), angle (0-180)] -- camera tilt is S1
		if not isinstance(msg, UInt8MultiArray): return
		d = list(msg.data)
		if len(d) < 2:
			self.get_logger().warn('pwm_servo expects [servo_id, angle]'); return
		servo_id = max(1, min(4, int(d[0])))
		angle = max(0, min(180, int(d[1])))
		self.car.set_pwm_servo(servo_id, angle)

	def bus_servo_callback(self, msg):
		# [servo_id, pulse (96-4000), run_time_ms] -- serial bus servo (Feetech-class)
		if not isinstance(msg, Int32MultiArray): return
		d = list(msg.data)
		if len(d) < 2:
			self.get_logger().warn('bus_servo expects [servo_id, pulse, run_time]'); return
		if not self._bus_servo_enabled:
			self.car.set_uart_servo_ctrl_enable(True)
			self.car.set_uart_servo_torque(1)
			self._bus_servo_enabled = True
		servo_id = int(d[0])
		pulse = max(96, min(4000, int(d[1])))
		run_time = int(d[2]) if len(d) >= 3 else 500
		self.car.set_uart_servo(servo_id, pulse, run_time)

	def get_battery_voltage_callback(self, request, response):
		"""Service callback to provide battery voltage to other nodes"""
		try:
			voltage = self.car.get_battery_voltage()
			response.voltage = float(voltage)
			response.success = True
			response.message = "Battery voltage read successfully"
		except Exception as e:
			response.voltage = 0.0
			response.success = False
			response.message = f"Failed to read battery voltage: {str(e)}"
		
		return response

	def pub_data(self):
		try:
			time_stamp = Clock().now()
			imu = Imu()
			twist = Twist()
			edition = Float32()
			mag = MagneticField()
			state = JointState()
			state.header.stamp = time_stamp.to_msg()
			state.header.frame_id = "joint_states"
			if len(self.Prefix)==0:
				state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
								"front_right_steer_joint", "front_right_wheel_joint"]
			else:
				state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
								self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
			
			# Get sensor data with error handling — read gyroscope once, share with vel computation
			try:
				ax, ay, az = self.car.get_accelerometer_data()
				gx, gy, gz = self.car.get_gyroscope_data()
				mx, my, mz = self.car.get_magnetometer_data()
				hardware_vx, hardware_vy, hardware_vz = self.get_hardware_velocities(gz)
			except Exception as e:
				self.get_logger().error(f"Error reading sensor data: {str(e)}")
				ax = ay = az = 0.0
				gx = gy = gz = 0.0
				mx = my = mz = 0.0
				hardware_vx = hardware_vy = hardware_vz = 0.0

			imu.header.stamp = time_stamp.to_msg()
			imu.header.frame_id = self.imu_link
			imu.linear_acceleration.x = ax
			imu.linear_acceleration.y = ay
			imu.linear_acceleration.z = az
			imu.angular_velocity.x = gx
			imu.angular_velocity.y = gy

			# Passive bias recalibration (see __init__ note)
			if (self.commanded_moving
					or abs(hardware_vx) > 0.01 or abs(hardware_vy) > 0.01):
				self.recal_window.clear()
			elif time.time() - self.last_recal_time >= self.recal_interval_s:
				self.recal_window.append(gz)
				if len(self.recal_window) >= self.recal_window_size:
					mean = sum(self.recal_window) / len(self.recal_window)
					spread = (sum((s - mean) ** 2 for s in self.recal_window)
					          / len(self.recal_window)) ** 0.5
					if spread < 0.005 and abs(mean) < 0.05:
						if abs(mean - self.imu_angular_bias) > 0.002:
							self.get_logger().info(
								f"IMU z-gyro bias updated "
								f"{self.imu_angular_bias:+.5f} -> {mean:+.5f} rad/s")
						self.imu_angular_bias = mean
						self.last_recal_time = time.time()
						try:
							with open(self.imu_bias_file, 'w') as f:
								f.write(f"{mean:.6f}\n")
						except OSError:
							pass
					self.recal_window.clear()

			# Apply the bias calibration (same correction the wheel
			# velocity path already uses) so consumers get a drift-free rate.
			imu.angular_velocity.z = gz - self.imu_angular_bias

			mag.header.stamp = time_stamp.to_msg()
			mag.header.frame_id = self.imu_link
			mag.magnetic_field.x = float(mx)
			mag.magnetic_field.y = float(my)
			mag.magnetic_field.z = float(mz)

			twist.linear.x = hardware_vx
			twist.linear.y = hardware_vy
			twist.angular.z = hardware_vz
			
			self.velPublisher.publish(twist)
			self.imuPublisher.publish(imu)
			self.magPublisher.publish(mag)
			
		except Exception as e:
			self.get_logger().error(f"Error in pub_data: {str(e)}")
		
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

if __name__ == '__main__':
	main()
