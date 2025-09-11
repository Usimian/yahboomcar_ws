#!/usr/bin/env python
# encoding: utf-8

#public lib
# import sys
# import math
# import random
# import threading
from math import pi
import time
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
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
			self.car = Rosmaster(com="/dev/ttyUSB0", debug=False)
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
		self.declare_parameter('hardware_calibration.linear_x_cal_factor', 1.0)
		self.declare_parameter('hardware_calibration.linear_y_cal_factor', 1.0)
		self.declare_parameter('hardware_calibration.angular_cal_factor', 1.0)
		self.declare_parameter('velocity_corrections.linear_velocity_correction', 1.52)
		self.declare_parameter('velocity_corrections.angular_velocity_correction', 1.0)
		
		self.linear_x_cal_factor = self.get_parameter('hardware_calibration.linear_x_cal_factor').get_parameter_value().double_value
		self.linear_y_cal_factor = self.get_parameter('hardware_calibration.linear_y_cal_factor').get_parameter_value().double_value
		self.angular_cal_factor = self.get_parameter('hardware_calibration.angular_cal_factor').get_parameter_value().double_value
		self.linear_velocity_correction = self.get_parameter('velocity_corrections.linear_velocity_correction').get_parameter_value().double_value
		self.angular_velocity_correction = self.get_parameter('velocity_corrections.angular_velocity_correction').get_parameter_value().double_value
		
		self.get_logger().info(f"Calibration factors - Linear X: {self.linear_x_cal_factor:.3f}, Linear Y: {self.linear_y_cal_factor:.3f}, Angular: {self.angular_cal_factor:.3f}")
		self.get_logger().info(f"Velocity corrections - Linear: {self.linear_velocity_correction:.3f}, Angular: {self.angular_velocity_correction:.3f}")
		
		# Note: IMU gyroscope bias correction removed - using wheel odometry for angular velocity instead

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

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
		
		# Calibrate IMU bias by averaging readings when stationary
		self.get_logger().info("Calibrating IMU bias...")
		bias_samples = []
		for i in range(50):  # Take 50 samples over 0.5 seconds
			gx, gy, gz = self.car.get_gyroscope_data()
			bias_samples.append(gz)
			time.sleep(0.01)
		
		self.imu_angular_bias = sum(bias_samples) / len(bias_samples)
		self.get_logger().info(f"IMU angular bias calibrated: {self.imu_angular_bias:.3f} rad/s")
		
 		# Hardware speed feedback system (better than raw encoders)
		# The robot firmware provides processed velocity feedback that accounts for
		# wheel dynamics, slip compensation, and motor characteristics
		
		self.get_logger().info("Yahboom X3 driver initialization complete")

	def get_hardware_velocities(self):
		"""Get actual velocities from hardware speed feedback"""
		# Get wheel-based linear velocities (reliable)
		vx, vy, vz_wheel = self.car.get_motion_data()
		
		# CRITICAL: Hardware velocity feedback is severely under-reporting
		# Based on measurements: actual/reported ratios
		# Linear: 0.62m actual / 0.408m reported = 1.52 (now loaded from parameter file)
		vx_corrected = vx * self.linear_velocity_correction
		vy_corrected = vy * self.linear_velocity_correction  
		
		# Use IMU angular velocity instead of unreliable wheel-based calculation
		gx, gy, gz = self.car.get_gyroscope_data()
		vz_imu = gz - self.imu_angular_bias  # IMU Z-axis angular velocity (yaw rate) with bias correction
		
		# Debug output to see what hardware is actually reporting
		if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz_wheel) > 0.01 or abs(vz_imu) > 0.01:
			self.get_logger().info(f"Wheel velocities: vx={vx:.3f}, vy={vy:.3f}, vz_wheel={vz_wheel:.3f}")
			self.get_logger().info(f"IMU angular: gz={gz:.3f} rad/s")
			self.get_logger().info(f"Final: vx={vx_corrected:.3f}, vy={vy_corrected:.3f}, vz_imu={vz_imu:.3f}")
		
		return vx_corrected, vy_corrected, vz_imu

	def cmd_vel_callback(self,msg):
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # Apply calibration factors - SINGLE POINT OF CALIBRATION
		vx = msg.linear.x * self.linear_x_cal_factor
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y * self.linear_y_cal_factor
		angular = msg.angular.z * self.angular_cal_factor
		self.car.set_car_motion(vx, vy, angular)
		
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
			
			# Get sensor data with error handling
			try:
				#print ("mag: ",self.car.get_magnetometer_data())
				edition.data = self.car.get_version()*1.0
				ax, ay, az = self.car.get_accelerometer_data()
				gx, gy, gz = self.car.get_gyroscope_data()
				mx, my, mz = self.car.get_magnetometer_data()
				mx = mx * 1.0
				my = my * 1.0
				mz = mz * 1.0
				vx, vy, angular = self.car.get_motion_data()
				
				# Debug output every 50 cycles (5 seconds)
				self.debug_counter += 1
				if self.debug_enabled and self.debug_counter % 50 == 0:
					self.get_logger().info(f"Hardware data - Version: {edition.data:.1f}")
					self.get_logger().info(f"IMU - Accel: ({ax:.3f}, {ay:.3f}, {az:.3f}), Gyro: ({gx:.3f}, {gy:.3f}, {gz:.3f})")
					self.get_logger().info(f"Mag: ({mx:.1f}, {my:.1f}, {mz:.1f}), Motion: ({vx:.3f}, {vy:.3f}, {angular:.3f})")
					
			except Exception as e:
				self.get_logger().error(f"Error reading sensor data: {str(e)}")
				# Set default values on error
				edition.data = -1.0
				ax = ay = az = 0.0
				gx = gy = gz = 0.0
				mx = my = mz = 0.0
				vx = vy = angular = 0.0
			
			# Publish raw gyroscope data (bias correction not needed since using wheel odometry for angular velocity)
			imu.header.stamp = time_stamp.to_msg()
			imu.header.frame_id = self.imu_link
			imu.linear_acceleration.x = ax*1.0
			imu.linear_acceleration.y = ay*1.0
			imu.linear_acceleration.z = az*1.0
			imu.angular_velocity.x = gx*1.0
			imu.angular_velocity.y = gy*1.0
			imu.angular_velocity.z = gz*1.0

			mag.header.stamp = time_stamp.to_msg()
			mag.header.frame_id = self.imu_link
			mag.magnetic_field.x = mx*1.0
			mag.magnetic_field.y = my*1.0
			mag.magnetic_field.z = mz*1.0
			
			# FIXED: Publish commanded velocities instead of unreliable hardware feedback
			# This ensures proper angular velocity feedback for odometry and SLAM
			# Use hardware speed feedback for accurate odometry
			# This provides actual measured velocities from the robot's firmware
			hardware_vx, hardware_vy, hardware_vz = self.get_hardware_velocities()
			twist.linear.x = hardware_vx
			twist.linear.y = hardware_vy
			twist.angular.z = hardware_vz
			
			self.velPublisher.publish(twist)
			self.imuPublisher.publish(imu)
			self.magPublisher.publish(mag)
			self.EdiPublisher.publish(edition)
			
		except Exception as e:
			self.get_logger().error(f"Error in pub_data: {str(e)}")
		
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

if __name__ == '__main__':
	main()
