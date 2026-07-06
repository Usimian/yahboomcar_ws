#!/usr/bin/env python3
"""Camera-tilt bridge node.

The STS3215 tilt servo is driven natively off the Yahboom expansion board's bus
servo port (patched STM32 firmware -> little-endian STS write), commanded via the
Mcnamu driver's /bus_servo topic. This node bridges the existing /camera_tilt
interface so joystick button Y (yahboom_joy_X3.py) and herbie tilt-tracking keep
working unchanged:

  sub  /camera_tilt              std_msgs/Int32           -> DIRECT target (jog; no settle)
  sub  /camera_tilt_precise      std_msgs/Int32           -> PRECISE target (settle from above)
  pub  /bus_servo                std_msgs/Int32MultiArray -> [id, pos, time_ms] to Mcnamu driver
  pub  /camera_tilt_state        std_msgs/Int32           -> commanded position (open-loop echo)
  pub  /camera_tilt_joint_state  sensor_msgs/JointState   -> camera_tilt_joint angle (rad) for robot_state_publisher

Open-loop: the firmware servo-READ path isn't STS-patched, so there's no position
feedback through the board -- the joint angle is from the COMMANDED position.
Host clamps to the mechanical range [1755,2796] (firmware only guards 96..4000).

Backlash takeup (2026-07-06): the gear train has ~2 deg of lash. Measured against
the depth floor plane, the physical lens pitch at a given commanded position depends
on the approach direction -- settling to level from below left the floor tilted
+2.15 deg, from above +0.10 deg (open-loop TF read 0.0 rad both times, blind to the
lash). So there are two kinds of tilt move, by intent:

  * DIRECT (/camera_tilt)  -- coarse aiming: joystick jog. Move straight there; the
    operator is watching and will nudge. No settle, stays responsive.
  * PRECISE (/camera_tilt_precise) -- the pointing must be TRUE when you act on it:
    homing to level, and any "move there to take a measurement" (depth/TSDF scan,
    tilt-track-and-capture). Overshoots ~1.5x the lash ABOVE the target, waits for the
    move to finish, then settles DOWN onto it -- always loading the same tooth face,
    like tuning a guitar string up to pitch, or slewing a telescope onto target from
    one side. This makes the open-loop commanded->rad mapping physically true at the
    settled angle, so no URDF pitch offset is needed.

Boot-home routes through the precise path. The reported joint angle holds the final
target throughout a precise overshoot (the transient servo position is unobservable
open-loop anyway). NOTE: the joystick X-tap "snap to level" still publishes on
/camera_tilt (direct); for it to land precisely it should publish on
/camera_tilt_precise instead -- companion change in yahboom_joy_X3.py.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import JointState

SERVO_ID = 3
TILT_MIN, TILT_MAX = 1580, 2796          # down-limit 1580 (~-39deg) verified free via webcam+IMU 2026-06-21 after rewiring (was 1755/-23deg)
TILT_HOME = 2017                         # level (IMU-calibrated 2026-06-10, backlash-band midpoint)
MOVE_TIME_MS = 500                       # per-command move time sent to /bus_servo
TILT_JOINT_NAME = "camera_tilt_joint"
TILT_LEVEL_POS  = 2017                   # counts at level == 0 rad (IMU-calibrated)
COUNTS_PER_DEG  = 11.38                  # 4096 / 360

LASH_COUNTS      = 24                     # ~2.0 deg measured lash (2026-07-06) => 2.0 * 11.38
OVERSHOOT_COUNTS = int(LASH_COUNTS * 1.5) # a precise move drives this far above target, then settles down onto it
SETTLE_S         = 0.6                    # >= MOVE_TIME_MS; let the overshoot move finish before settling down


class CameraTiltBridge(Node):
    def __init__(self):
        super().__init__("camera_tilt_bridge")
        self._pos = TILT_HOME
        self._homed = False
        self._settle_target = None       # final target to settle onto after a precise overshoot
        self._settle_timer = None
        self.bus_pub   = self.create_publisher(Int32MultiArray, "bus_servo", 10)
        self.state_pub = self.create_publisher(Int32, "camera_tilt_state", 10)
        self.joint_pub = self.create_publisher(JointState, "camera_tilt_joint_state", 10)
        self.sub         = self.create_subscription(Int32, "camera_tilt", self.on_target, 10)
        self.sub_precise = self.create_subscription(Int32, "camera_tilt_precise", self.on_target_precise, 10)
        self.timer = self.create_timer(0.2, self.publish_state)     # 5 Hz joint-state for TF
        self.home_timer = self.create_timer(1.0, self._home_once)   # home to level shortly after start
        self.get_logger().info(f"camera_tilt bridge ready: /camera_tilt (direct) + "
                               f"/camera_tilt_precise (settle-from-above) -> /bus_servo id={SERVO_ID}, "
                               f"limits [{TILT_MIN},{TILT_MAX}], overshoot {OVERSHOOT_COUNTS} counts")

    def _home_once(self):
        if not self._homed:
            self._homed = True
            self.command_precise(TILT_HOME)    # boot-home must land level exactly

    def on_target(self, msg):
        self.command_direct(int(msg.data))

    def on_target_precise(self, msg):
        self.command_precise(int(msg.data))

    def command_direct(self, pos):
        # Coarse aiming (joystick jog): move straight there, no backlash takeup.
        # Cancels any in-flight precise settle so a jog always wins.
        self._cancel_settle()
        self._send(max(TILT_MIN, min(TILT_MAX, pos)))

    def command_precise(self, pos):
        # Precise setpoint (home / measurement): reach it from above so gear lash is
        # always taken up on the same tooth face. Overshoot up, then settle down onto
        # the target after the move completes.
        pos = max(TILT_MIN, min(TILT_MAX, pos))
        over = min(TILT_MAX, pos + OVERSHOOT_COUNTS)
        if over > pos:
            self._settle_target = pos
            self._send(over, report_pos=pos)   # move up past target; TF already reads final target
            self._cancel_settle(keep_target=True)
            self._settle_timer = self.create_timer(SETTLE_S, self._settle)
        else:
            # target is already at/above TILT_MAX, no room to overshoot -- go direct
            self._send(pos)

    def _settle(self):
        self._cancel_settle(keep_target=True)
        if self._settle_target is not None:
            target = self._settle_target
            self._settle_target = None
            self._send(target)

    def _cancel_settle(self, keep_target=False):
        if self._settle_timer is not None:
            self._settle_timer.cancel()
            self._settle_timer = None
        if not keep_target:
            self._settle_target = None

    def _send(self, pos, report_pos=None):
        # report_pos: the position to reflect in TF / state echo. During a precise
        # overshoot the servo is transiently above the target, but open-loop we
        # report the final target so the joint angle doesn't wiggle.
        self._pos = pos if report_pos is None else report_pos
        m = Int32MultiArray()
        m.data = [SERVO_ID, pos, MOVE_TIME_MS]   # patched firmware: LE pos + time to STS reg 0x2A/0x2C
        self.bus_pub.publish(m)
        self.state_pub.publish(Int32(data=self._pos))
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
