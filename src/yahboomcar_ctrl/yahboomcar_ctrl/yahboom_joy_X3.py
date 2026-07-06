#!/usr/bin/env python
# encoding: utf-8

#public lib
import time
import getpass
import subprocess

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool, UInt8MultiArray

class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = False
		self.Buzzer_active = False
		self.RGBLight_index = 0
		self.cancel_time = time.time()
		self.user_name = getpass.getuser()
		print(self.user_name)
		# Gear lists and speed limits are declared further down as ROS params.
		
		# Button state tracking for edge detection
		self.prev_rgb_button_state = False
		self.prev_linear_button_state = False
		self.prev_angular_button_state = False
		self.prev_start_button_state = False
		self.prev_b_button_state = False
		self.headlight_on = False
		# Camera tilt: Y = up, A = down. HELD = continuous slow motion (joy_node
		# autorepeats at 20 Hz). Step per frame is small; clamped to hard limits.
		# 2017 = level (IMU-calibrated 2026-06-10); range [1580 (-39 deg down), 2796 (up)].
		self.TILT_MIN = 1580
		self.TILT_MAX = 2796
		self.TILT_LEVEL = 2017			# IMU-calibrated level; X snaps back to it via /camera_tilt_precise
		self.TILT_STEP = 3			# counts/frame; ~5 deg/s at 20 Hz (11.38 counts/deg)
		self.tilt_target = self.TILT_LEVEL	# start at level (camera homes here on boot)
		self.prev_x_button_state = False

		# Start-button long-hold -> shutdown
		self.start_hold_begin = None		# monotonic time when start was first pressed
		self.shutdown_triggered = False		# latches once poweroff has fired
		self.SHUTDOWN_HOLD_SECONDS = 5.0
		
		# Define joystick control mappings
		self.setup_control_mappings()
		
		#create pub
		self.pub_goal = self.create_publisher(GoalID,"move_base/cancel",10)
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_Buzzer = self.create_publisher(Bool,"Buzzer",  1)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		self.pub_RGBLight = self.create_publisher(Int32,"RGBLight" , 10)
		self.pub_LedCommand = self.create_publisher(UInt8MultiArray,"led_command", 10)
		self.pub_CameraTilt = self.create_publisher(Int32,"camera_tilt", 10)
		self.pub_CameraTiltPrecise = self.create_publisher(Int32,"camera_tilt_precise", 10)	# X-snap-to-level: bridge settles from above, taking up ~2 deg gear lash
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)

		# Declare parameters (overridable from yaml). Ceilings are hard clamps;
		# gear lists are the values cycled by the stick-click buttons.
		self.declare_parameter('xspeed_limit', 1.0)
		self.declare_parameter('yspeed_limit', 1.0)
		self.declare_parameter('angular_speed_limit', 1.0)
		self.declare_parameter('linear_gears', [0.5, 1.0])
		self.declare_parameter('angular_gears', [0.5, 1.0])
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		self._linear_gears = list(self.get_parameter('linear_gears').get_parameter_value().double_array_value)
		self._angular_gears = list(self.get_parameter('angular_gears').get_parameter_value().double_array_value)
		self.linear_speed = self._linear_gears[-1]
		self.angular_speed = self._angular_gears[-1]
		self.get_logger().info(
			f"Joy config: linear_gears={self._linear_gears} angular_gears={self._angular_gears} "
			f"ceilings x={self.xspeed_limit} y={self.yspeed_limit} ang={self.angular_speed_limit}"
		)
		
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
				'b_button': 1,		# Headlight toggle (all LEDs white)
				'a_button': 0,		# Camera tilt DOWN (hold for continuous)
				'x_button': 3,		# Camera tilt -> LEVEL (single press)
				'y_button': 4		# Camera tilt UP (hold for continuous)
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
		current_a_button = self.get_button_state(joy_data, 'a_button')
		current_x_button = self.get_button_state(joy_data, 'x_button')
		current_y_button = self.get_button_state(joy_data, 'y_button')

		# Start button:
		#   - Short press (<5s): toggle drive state on release (normal behavior)
		#   - Long hold (>=5s): beep twice and poweroff
		if current_start_button and not self.prev_start_button_state:
			# Press edge: begin hold timer, don't toggle yet
			self.start_hold_begin = time.monotonic()
			self.shutdown_triggered = False
		elif current_start_button and self.prev_start_button_state:
			# Still held: check for long-hold shutdown
			if (not self.shutdown_triggered
					and self.start_hold_begin is not None
					and time.monotonic() - self.start_hold_begin >= self.SHUTDOWN_HOLD_SECONDS):
				self.shutdown_triggered = True
				self.trigger_shutdown()
		elif not current_start_button and self.prev_start_button_state:
			# Release edge: toggle drive only if we didn't long-hold
			if not self.shutdown_triggered:
				self.toggle_drive_state()
			self.start_hold_begin = None
			self.shutdown_triggered = False
			
		# RGB Light control - detect button press transition
		if current_rgb_button and not self.prev_rgb_button_state:
			RGBLight_ctrl = Int32()
			RGBLight_ctrl.data = self.RGBLight_index
			for i in range(3): self.pub_RGBLight.publish(RGBLight_ctrl)
			
			# Cycle through 0 to 6 inclusive
			self.RGBLight_index = (self.RGBLight_index + 1) % 7
			
		# Linear gear control - detect button press transition
		if current_linear_button and not self.prev_linear_button_state:
			try:
				i = self._linear_gears.index(self.linear_speed)
			except ValueError:
				i = -1
			self.linear_speed = self._linear_gears[(i + 1) % len(self._linear_gears)]
			self.get_logger().warning(f"Linear gear -> {self.linear_speed:.3f}")

		# Angular gear control - detect button press transition
		if current_angular_button and not self.prev_angular_button_state:
			try:
				i = self._angular_gears.index(self.angular_speed)
			except ValueError:
				i = -1
			self.angular_speed = self._angular_gears[(i + 1) % len(self._angular_gears)]
			self.get_logger().warning(f"Angular gear -> {self.angular_speed:.3f}")

		# B button - Headlight toggle (all LEDs white on/off) - press transition
		if current_b_button and not self.prev_b_button_state:
			self.headlight_on = not self.headlight_on
			level = 255 if self.headlight_on else 0
			msg = UInt8MultiArray()
			msg.data = [0xFF, level, level, level]
			self.pub_LedCommand.publish(msg)
			self.get_logger().warning(f"Headlight {'ON' if self.headlight_on else 'OFF'}")

		# X button - snap camera tilt back to level - press transition.
		# Uses the PRECISE path so the bridge settles level from above and takes up
		# the ~2 deg gear lash; a direct up-to-level move leaves the depth floor tilted.
		if current_x_button and not self.prev_x_button_state:
			self.tilt_target = self.TILT_LEVEL
			tilt_msg = Int32()
			tilt_msg.data = self.tilt_target
			self.pub_CameraTiltPrecise.publish(tilt_msg)
			self.get_logger().warning("Camera tilt -> level (precise)")

		# Camera tilt - Y = up, A = down. HELD = continuous (joy_node autorepeats
		# at 20 Hz). Nudge the target a small step per frame, clamp to hard limits,
		# publish only when it actually changed.
		elif current_y_button or current_a_button:
			step = self.TILT_STEP if current_y_button else -self.TILT_STEP
			new_target = max(self.TILT_MIN, min(self.TILT_MAX, self.tilt_target + step))
			if new_target != self.tilt_target:
				self.tilt_target = new_target
				tilt_msg = Int32()
				tilt_msg.data = self.tilt_target
				self.pub_CameraTilt.publish(tilt_msg)

		# Update previous button states for next iteration (edge-detected buttons
		# only; Y/A tilt is level-triggered/held, so they need no prev state).
		self.prev_start_button_state = current_start_button
		self.prev_rgb_button_state = current_rgb_button
		self.prev_x_button_state = current_x_button
		self.prev_linear_button_state = current_linear_button
		self.prev_angular_button_state = current_angular_button
		self.prev_b_button_state = current_b_button
			
		# Get movement values using named axes.
		# linear_speed / angular_speed are the max m/s and rad/s at full stick.
		# xspeed_limit / yspeed_limit / angular_speed_limit are safety ceilings
		# clamped below and can't be exceeded by the gear list.
		xlinear_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_left_y')) * self.linear_speed
		ylinear_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_left_x')) * self.linear_speed
		angular_speed = self.filter_data(self.get_axis_value(joy_data, 'joy_right_x')) * self.angular_speed
		
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

		if xlinear_speed or ylinear_speed or angular_speed:
			self.get_logger().info(f"Speed: {xlinear_speed:.2f} m/s, {ylinear_speed:.2f} m/s, {angular_speed:.2f} rad/s")

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

	def trigger_shutdown(self):
		"""Beep twice and power off the robot (start button held >=5s)."""
		self.get_logger().warning("⏻ Start button held — powering off")
		# Stop the robot first
		self.pub_cmdVel.publish(Twist())
		# Two beeps
		beep_msg = Bool()
		for _ in range(2):
			beep_msg.data = True
			self.pub_Buzzer.publish(beep_msg)
			time.sleep(0.25)
			beep_msg.data = False
			self.pub_Buzzer.publish(beep_msg)
			time.sleep(0.15)
		try:
			subprocess.Popen(['sudo', '-n', '/usr/sbin/poweroff'])
		except Exception as e:
			self.get_logger().error(f"poweroff failed: {e}")

def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	rclpy.spin(joy_ctrl)		
