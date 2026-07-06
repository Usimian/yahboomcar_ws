#!/usr/bin/env python3
"""
RealSense depth watchdog.

Subscribes to a depth-stream topic. If no message arrives for `stale_threshold`
seconds, performs a USB-level reset of the camera by writing to its sysfs
`authorized` file (0 then 1). librealsense usually recovers and resumes
streaming on its own once the kernel re-enumerates the device.

Why: D435/D435i on Linux has a well-known issue where the depth pipeline
silently stops delivering frames. Color may keep working. No reliable
recovery exists in librealsense itself. The standard mitigation is a
watchdog that USB-resets the device when streams stall.

Parameters:
  topic                 (string) Topic to monitor. Default
                                 /realsense_camera/depth/color/points.
  usb_vendor_id         (string) USB idVendor of the camera. Default 8086.
  usb_product_id        (string) USB idProduct of the camera. Default 0b3a.
                                 The sysfs path is resolved by identity at
                                 every reset: the path itself moves across
                                 re-enumerations (2-1.3 -> 2-1.1 observed
                                 2026-07-05, which left a hardcoded-path
                                 watchdog resetting a nonexistent device).
  stale_threshold       (float)  Seconds without a message before triggering
                                 recovery. Default 5.0.
  warmup_sec            (float)  Grace period after node start before
                                 watchdog can fire. Default 15.0.
  cooldown_sec          (float)  After a reset, wait this long before
                                 considering the topic stale again. Default
                                 10.0.
"""
import glob
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class RealsenseWatchdog(Node):
    def __init__(self):
        super().__init__('realsense_watchdog')

        self.declare_parameter('topic', '/realsense_camera/depth/color/points')
        self.declare_parameter('usb_vendor_id', '8086')
        self.declare_parameter('usb_product_id', '0b3a')
        self.declare_parameter('stale_threshold', 5.0)
        self.declare_parameter('warmup_sec', 15.0)
        self.declare_parameter('cooldown_sec', 10.0)

        self.topic = self.get_parameter('topic').value
        self.usb_vendor = self.get_parameter('usb_vendor_id').value
        self.usb_product = self.get_parameter('usb_product_id').value
        self.stale_threshold = float(self.get_parameter('stale_threshold').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        # Use BEST_EFFORT/depth=1 to match how high-rate sensor topics are
        # typically published; mismatched RELIABLE here would silently fail to
        # match if the publisher is BEST_EFFORT.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            PointCloud2, self.topic, self._on_msg, qos)

        now = time.monotonic()
        self._last_msg_time = now
        self._start_time = now
        self._next_eligible_check = now + self.warmup_sec
        self._reset_count = 0

        # Check once a second.
        self._timer = self.create_timer(1.0, self._check)

        self.get_logger().info(
            f"realsense_watchdog started. Watching {self.topic}, "
            f"resetting USB {self.usb_vendor}:{self.usb_product} after "
            f"{self.stale_threshold}s of silence.")

    def _on_msg(self, _msg):
        self._last_msg_time = time.monotonic()

    def _check(self):
        now = time.monotonic()
        if now < self._next_eligible_check:
            return
        gap = now - self._last_msg_time
        if gap < self.stale_threshold:
            return
        self.get_logger().warn(
            f"No depth pointcloud for {gap:.1f}s; resetting USB device "
            f"{self.usb_vendor}:{self.usb_product} "
            f"(reset #{self._reset_count + 1}).")
        ok = self._usb_reset()
        if ok:
            self._reset_count += 1
            # Pretend we just received a message so we don't immediately
            # re-trigger; real messages should resume within cooldown_sec.
            self._last_msg_time = now
            self._next_eligible_check = now + self.cooldown_sec
        else:
            # If reset failed, back off a little so we don't spin.
            self._next_eligible_check = now + 5.0

    def _find_device(self):
        """Resolve the camera's sysfs path by USB identity, fresh each time
        (the path changes across re-enumerations)."""
        for vid_file in glob.glob('/sys/bus/usb/devices/*/idVendor'):
            dev = os.path.dirname(vid_file)
            try:
                with open(vid_file) as f:
                    vid = f.read().strip()
                with open(os.path.join(dev, 'idProduct')) as f:
                    pid = f.read().strip()
            except OSError:
                continue
            if vid == self.usb_vendor and pid == self.usb_product:
                return dev
        return None

    def _usb_reset(self) -> bool:
        """Write 0 then 1 to <device>/authorized via sudo."""
        dev = self._find_device()
        if dev is None:
            self.get_logger().error(
                f"no USB device {self.usb_vendor}:{self.usb_product} found "
                "to reset — camera absent from the bus?")
            return False
        auth = os.path.join(dev, 'authorized')
        try:
            for value in ('0', '1'):
                # Sudoers must allow mw to run `tee <auth>` without password.
                proc = subprocess.run(
                    ['sudo', '-n', 'tee', auth],
                    input=value.encode(),
                    capture_output=True,
                    timeout=5.0,
                )
                if proc.returncode != 0:
                    self.get_logger().error(
                        f"USB reset failed at write '{value}': "
                        f"rc={proc.returncode} err={proc.stderr.decode().strip()}")
                    return False
                time.sleep(0.5)
            self.get_logger().info(f"USB reset of {dev} complete.")
            return True
        except subprocess.TimeoutExpired:
            self.get_logger().error("USB reset command timed out.")
            return False
        except Exception as e:
            self.get_logger().error(f"USB reset exception: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
