#!/usr/bin/env python
# encoding: utf-8

#public lib
import time
import getpass

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool
from robot_msgs.srv import ExecuteCommand
from robot_msgs.msg import RobotCommand

class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = False
		self.Buzzer_active = False
		self.RGBLight_index = 0
		self.cancel_time = time.time()
		self.user_name = getpass.getuser()
		print(self.user_name)
		self.linear_speed = 1.0
		self.angular_speed = 1.0
		
		# Button state tracking for edge detection
		self.prev_rgb_button_state = False
		self.prev_linear_button_state = False
		self.prev_angular_button_state = False
		self.prev_start_button_state = False
		self.prev_b_button_state = False
		
		# Define joystick control mappings
		self.setup_control_mappings()
		
		#create pub
		self.pub_goal = self.create_publisher(GoalID,"move_base/cancel",10)
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_Buzzer = self.create_publisher(Bool,"Buzzer",  1)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		self.pub_RGBLight = self.create_publisher(Int32,"RGBLight" , 10)
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)

		#create service client for robot commands
		self.robot_command_client = self.create_client(ExecuteCommand, '/robot/execute_command')
		
		#declare parameter and get the value
		self.declare_parameter('xspeed_limit',0.5)
		self.declare_parameter('yspeed_limit',0.5)
		self.declare_parameter('angular_speed_limit',0.8)
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		
	def setup_control_mappings(self):
		"""Setup joystick control mappings for different controller types"""
		# Jetson/Default controller mapping
		self.jetson_controls = {
			'buttons': {
				'start': 11,	# start/stop drive
				'right_button': 7,	# RGB light
				'select': 10,		# Buzzer
				'left_joystick_button': 13,	# Linear gear
				'right_joystick_button': 14,	# Angular gear
				'b_button': 1		# Stop command
			},
			'axes': {
				'joy_left_y': 1,    # Forward/backward
				'joy_left_x': 0,    # Left/right
				'joy_right_x': 2    # Rotation
			}
		}
		# Set active control mapping
		self.active_controls = self.jetson_controls
		
	def get_button_state(self, joy_data, button_name):
		"""Get button state by name"""
		if button_name in self.active_controls['buttons']:
			button_index = self.active_controls['buttons'][button_name]
			if button_index < len(joy_data.buttons):
				return joy_data.buttons[button_index] == 1
		return False
		
	def get_axis_value(self, joy_data, axis_name):
		"""Get axis value by name"""
		if axis_name in self.active_controls['axes']:
			axis_index = self.active_controls['axes'][axis_name]
			if axis_index < len(joy_data.axes):
				return joy_data.axes[axis_index]
		return 0.0
		
	def buttonCallback(self,joy_data):
		if not isinstance(joy_data, Joy): return
		self.user_jetson(joy_data)

	def user_jetson(self, joy_data):
		# Get current button states
		current_start_button = self.get_button_state(joy_data, 'start')
		current_rgb_button = self.get_button_state(joy_data, 'right_button')
		current_linear_button = self.get_button_state(joy_data, 'left_joystick_button')
		current_angular_button = self.get_button_state(joy_data, 'right_joystick_button')
		current_b_button = self.get_button_state(joy_data, 'b_button')
		
		# Drive on/off - detect button press transition
		if current_start_button and not self.prev_start_button_state:
			self.toggle_drive_state()
			
		# RGB Light control - detect button press transition
		if current_rgb_button and not self.prev_rgb_button_state:
			RGBLight_ctrl = Int32()
			RGBLight_ctrl.data = self.RGBLight_index
			for i in range(3): self.pub_RGBLight.publish(RGBLight_ctrl)
			
			# Cycle through 0 to 6 inclusive
			self.RGBLight_index = (self.RGBLight_index + 1) % 7
			
		# Linear gear control - detect button press transition
		if current_linear_button and not self.prev_linear_button_state:
			if self.linear_speed == 1.0: self.linear_speed = 1.0 / 3
			elif self.linear_speed == 1.0 / 3: self.linear_speed = 2.0 / 3
			elif self.linear_speed == 2.0 / 3: self.linear_speed = 1
			
		# Angular gear control - detect button press transition
		if current_angular_button and not self.prev_angular_button_state:
			if self.angular_speed == 1.0: self.angular_speed = 1.0 / 4
			elif self.angular_speed == 1.0 / 4: self.angular_speed = 1.0 / 2
			elif self.angular_speed == 1.0 / 2: self.angular_speed = 3.0 / 4
			elif self.angular_speed == 3.0 / 4: self.angular_speed = 1.0

		# B button - Stop command - detect button press transition
		if current_b_button and not self.prev_b_button_state:
			self.execute_stop_command()
			
		# Update previous button states for next iteration
		self.prev_start_button_state = current_start_button
		self.prev_rgb_button_state = current_rgb_button
		self.prev_linear_button_state = current_linear_button
		self.prev_angular_button_state = current_angular_button
		self.prev_b_button_state = current_b_button
			
		# Get movement values using named axes
		xlinear_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_left_y')) * self.xspeed_limit * self.linear_speed
		ylinear_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_left_x')) * self.yspeed_limit * self.linear_speed
		angular_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_right_x')) * self.angular_speed_limit * self.angular_speed
		
		# Apply speed limits
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		
		# Create and publish twist message
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		if self.Joy_active == True:
			# print("joy control now")
			for i in range(3): self.pub_cmdVel.publish(twist)


	def filter_data(self, value):
		return (value ** 2.5) if value >= 0 else -((-value) ** 2.5)
		
	def toggle_drive_state(self):
		now_time = time.time()
		if now_time - self.cancel_time > 1:
			Joy_ctrl = Bool()
			self.Joy_active = not self.Joy_active
			Joy_ctrl.data = self.Joy_active

			# Beep pattern: 1 beep for enable, 2 beeps for disable
			beep_msg = Bool()
			if self.Joy_active:
				# Single beep for enable
				beep_msg.data = True
				self.pub_Buzzer.publish(beep_msg)
				time.sleep(0.2)
				beep_msg.data = False
				self.pub_Buzzer.publish(beep_msg)
			else:
				# Double beep for disable
				beep_msg.data = True
				self.pub_Buzzer.publish(beep_msg)
				time.sleep(0.2)
				beep_msg.data = False
				self.pub_Buzzer.publish(beep_msg)
				time.sleep(0.1)
				beep_msg.data = True
				self.pub_Buzzer.publish(beep_msg)
				time.sleep(0.2)
				beep_msg.data = False
				self.pub_Buzzer.publish(beep_msg)

			self.pub_JoyState.publish(Joy_ctrl)
			self.pub_cmdVel.publish(Twist())
			self.cancel_time = now_time

	def execute_stop_command(self):
		"""Execute stop command via robot interface service"""
		try:
			# Create stop command
			command = RobotCommand()
			command.command_type = "stop"

			# Create service request
			request = ExecuteCommand.Request()
			request.command = command

			# Send async request
			if self.robot_command_client.service_is_ready():
				future = self.robot_command_client.call_async(request)
				future.add_done_callback(self.stop_command_callback)

				# Triple beep for stop command
				beep_msg = Bool()
				for i in range(3):
					beep_msg.data = True
					self.pub_Buzzer.publish(beep_msg)
					time.sleep(0.1)
					beep_msg.data = False
					self.pub_Buzzer.publish(beep_msg)
					time.sleep(0.1)

				self.get_logger().info("🛑 Stop command sent via B button")
			else:
				self.get_logger().warning("⚠️ Robot command service not ready")

		except Exception as e:
			self.get_logger().error(f"❌ Error executing stop command: {str(e)}")

	def stop_command_callback(self, future):
		"""Callback for stop command response"""
		try:
			response = future.result()
			if response.success:
				self.get_logger().info("✅ Stop command executed successfully")
			else:
				self.get_logger().warning(f"⚠️ Stop command failed: {response.result_message}")
		except Exception as e:
			self.get_logger().error(f"❌ Stop command callback error: {str(e)}")
			
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	rclpy.spin(joy_ctrl)		
