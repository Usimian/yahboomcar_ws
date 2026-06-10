#!/usr/bin/env python3
"""Camera-tilt bridge node.

The STS3215 tilt servo is driven natively off the Yahboom expansion board's bus
servo port (patched STM32 firmware -> little-endian STS write), commanded via the
Mcnamu driver's /bus_servo topic. This node bridges the existing /camera_tilt
interface so joystick button Y (yahboom_joy_X3.py) and herbie tilt-tracking keep
working unchanged:

  sub  /camera_tilt              std_msgs/Int32           -> target servo position
  pub  /bus_servo                std_msgs/Int32MultiArray -> [id, pos, time_ms] to Mcnamu driver
  pub  /camera_tilt_state        std_msgs/Int32           -> commanded position (open-loop echo)
  pub  /camera_tilt_joint_state  sensor_msgs/JointState   -> camera_tilt_joint angle (rad) for robot_state_publisher

Open-loop: the firmware servo-READ path isn't STS-patched, so there's no position
feedback through the board -- the joint angle is from the COMMANDED position.
Host clamps to the mechanical range [1755,2796] (firmware only guards 96..4000).
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import JointState

SERVO_ID = 3
TILT_MIN, TILT_MAX = 1755, 2796          # hard mechanical limits (host-side clamp)
TILT_HOME = 2015                         # level
MOVE_TIME_MS = 500                       # per-command move time sent to /bus_servo
TILT_JOINT_NAME = "camera_tilt_joint"
TILT_LEVEL_POS  = 2015                   # counts at level == 0 rad
COUNTS_PER_DEG  = 11.38                  # 4096 / 360


class CameraTiltBridge(Node):
    def __init__(self):
        super().__init__("camera_tilt_bridge")
        self._pos = TILT_HOME
        self._homed = False
        self.bus_pub   = self.create_publisher(Int32MultiArray, "bus_servo", 10)
        self.state_pub = self.create_publisher(Int32, "camera_tilt_state", 10)
        self.joint_pub = self.create_publisher(JointState, "camera_tilt_joint_state", 10)
        self.sub = self.create_subscription(Int32, "camera_tilt", self.on_target, 10)
        self.timer = self.create_timer(0.2, self.publish_state)     # 5 Hz joint-state for TF
        self.home_timer = self.create_timer(1.0, self._home_once)   # home to level shortly after start
        self.get_logger().info(f"camera_tilt bridge ready: /camera_tilt -> /bus_servo id={SERVO_ID}, "
                               f"limits [{TILT_MIN},{TILT_MAX}]")

    def _home_once(self):
        if not self._homed:
            self._homed = True
            self.command(TILT_HOME)

    def on_target(self, msg):
        self.command(int(msg.data))

    def command(self, pos):
        pos = max(TILT_MIN, min(TILT_MAX, int(pos)))
        self._pos = pos
        m = Int32MultiArray()
        m.data = [SERVO_ID, pos, MOVE_TIME_MS]   # patched firmware: LE pos + time to STS reg 0x2A/0x2C
        self.bus_pub.publish(m)
        self.state_pub.publish(Int32(data=pos))
        self.publish_state()

    def publish_state(self):
        angle = math.radians((self._pos - TILT_LEVEL_POS) / COUNTS_PER_DEG)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [TILT_JOINT_NAME]
        js.position = [angle]
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = CameraTiltBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
