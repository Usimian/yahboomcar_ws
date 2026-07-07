#!/usr/bin/env python3

"""
Point Cloud Traversability Filter Node

Classifies a depth point cloud into drivable floor vs obstacle by height above the
KNOWN floor plane (base_footprint z ~= 0), not by an absolute height band, and
republishes only the obstacle points for the Nav2 costmap + collision_monitor.

This replaces the earlier flat height-band filter, which marked the floor as an
obstacle wherever it rose into the band (at range, on carpet, or under residual
camera-pitch error), flooding the costmap.

Why "anchored floor" rather than a from-scratch plane fit: the robot's floor is at
base_footprint z ~= 0 by calibration (verified to ~0.2 deg after the camera-tilt
lash fix, 2026-07-06). So the floor is a KNOWN quantity, not something to rediscover
per frame. We fit only a gentle per-frame tilt correction seeded by points already
near z=0 -- this tracks residual body pitch and real ramps, but cannot lock onto a
mattress / low platform / wall as "floor" (those are above the z-seed and excluded).

Method (designed against captured floor/carpet/4cm-object/wall scenes, 2026-07-07;
final thresholds to be confirmed live):
  1. Transform the cloud into base_footprint using live TF (camera tilts on a servo).
  2. Range-gate to the depth camera's honest working range.
  3. Fit a gentle floor plane z = a*x + b*y + c using ONLY points near z=0 (the
     floor seed). If too few floor-seed points exist, the floor reference falls back
     to the plane z=0 -- so a wall filling the frame is still measured against the
     ground and correctly marked (no silent no-op).
  4. residual = height above that floor plane -- the obstacle signal.
  5. Range-aware threshold: thresh(range) = base_step + noise_alpha * range^2. This
     matches RealSense depth noise, which grows as range^2, so far-range floor noise
     stays below threshold while near-field low objects (down to base_step) are kept.
     One smooth curve, not a near/far cliff.
  6. Grid the obstacle points; a cell with >= min_cell_points obstacle points is an
     obstacle cell. Emit every obstacle point in obstacle cells.

FULL FIELD OF VIEW -- no forward corridor. The output feeds both the local_costmap
depth layer AND the collision_monitor, whose stop/slowdown zones are circular
(this mecanum base strafes and rotates in any direction). Obstacles beside the robot
must be visible, so scoping to a forward corridor here would blind the all-around
emergency stop. Any "is it in my path" logic belongs in the planner/collision layer,
not in this sensor feed.

Ramp-invariant: because the floor plane tilts with the ground (within the seed
band), a gentle ramp reads as floor, not a wall.

Subscribes to: /realsense_camera/depth/color/points (sensor_msgs/msg/PointCloud2)
Publishes to:  /camera/depth/points_filtered (sensor_msgs/msg/PointCloud2)
"""

# Cap BLAS/OpenMP thread pools BEFORE numpy is imported -- the per-frame linear
# algebra is small; OpenBLAS's one-thread-per-core default costs more than the work
# and steals cores from the rest of the ROS stack.
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration


class PointCloudTraversabilityFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_height_filter')

        self.declare_parameter('input_topic', '/realsense_camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/points_filtered')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('range_min', 0.35)
        self.declare_parameter('range_max', 2.5)
        # floor-seed half-thickness: points within +/- this of z=0 define the floor.
        # Must exceed the worst residual floor tilt over the range (a 1 deg tilt at
        # 2.5 m is ~4 cm), but stay below the shortest obstacle we must detect.
        self.declare_parameter('floor_seed_z', 0.05)
        # obstacle threshold at range 0, and its range^2 growth (depth noise law).
        self.declare_parameter('base_step_threshold', 0.03)
        self.declare_parameter('noise_alpha', 0.010)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('min_cell_points', 3)
        self.declare_parameter('voxel_leaf_size', 0.03)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.floor_seed_z = self.get_parameter('floor_seed_z').value
        self.base_step = self.get_parameter('base_step_threshold').value
        self.noise_alpha = self.get_parameter('noise_alpha').value
        self.grid = self.get_parameter('grid_resolution').value
        self.min_cell_points = self.get_parameter('min_cell_points').value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').value

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._rot_T = None
        self._translation = None
        self._cached_field_layout = None
        self._cached_xyz_contiguous = None

        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.pointcloud_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.frame_count = 0
        self.error_count = 0
        self.no_floor_count = 0
        self.get_logger().info(
            f'Traversability filter started: {self.input_topic} -> {self.output_topic} '
            f'range=[{self.range_min},{self.range_max}] floor_seed={self.floor_seed_z} '
            f'thresh={self.base_step}+{self.noise_alpha}*r^2 grid={self.grid} '
            f'full-FOV threads=1'
        )

    def _ensure_transform(self, source_frame):
        # The camera tilts on a servo, so this transform changes -- look it up live
        # each frame. The TF buffer is local so per-frame lookup is cheap.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.error_count += 1
            if self.error_count % 30 == 0:
                self.get_logger().warn(f'TF lookup failed: {e}')
            return False

        q = tf.transform.rotation
        t = tf.transform.translation
        rot = np.array([
            [1 - 2*(q.y**2 + q.z**2),   2*(q.x*q.y - q.z*q.w),   2*(q.x*q.z + q.y*q.w)],
            [    2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x**2 + q.z**2),   2*(q.y*q.z - q.x*q.w)],
            [    2*(q.x*q.z - q.y*q.w),   2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x**2 + q.y**2)],
        ], dtype=np.float32)
        self._rot_T = np.ascontiguousarray(rot.T)
        self._translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return True

    def _pointcloud2_to_xyz(self, msg):
        """Zero-copy xyz extraction from PointCloud2 into a float32 (N,3) array."""
        layout = self._cached_field_layout
        # Cache key includes the field offsets, not just point_step, so a layout
        # change that keeps the same step can't reuse stale offsets.
        field_map = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
        key = (msg.point_step, ox, oy, oz)
        if layout != key:
            self._cached_field_layout = key
            self._cached_xyz_contiguous = (ox == 0 and oy == 4 and oz == 8)
        point_step = msg.point_step

        n_points = msg.width * msg.height
        expected = n_points * point_step

        if self._cached_xyz_contiguous and point_step == 12 and len(msg.data) == expected:
            xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, 3)
        elif self._cached_xyz_contiguous and point_step % 4 == 0 and len(msg.data) == expected:
            quads = point_step // 4
            all_floats = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, quads)
            xyz = all_floats[:, :3]
        else:
            xyz = self._extract_xyz_fallback(msg.data, n_points, point_step, ox, oy, oz)

        xyz = xyz[np.isfinite(xyz).all(axis=1)]   # drop NaN/inf depth returns
        return xyz

    @staticmethod
    def _extract_xyz_fallback(data_bytes, n_points, point_step, ox, oy, oz):
        raw = np.frombuffer(data_bytes, dtype=np.uint8)
        usable = (len(raw) // point_step) * point_step
        raw = raw[:usable].reshape(-1, point_step)
        x = raw[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        y = raw[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        z = raw[:, oz:oz+4].copy().view(np.float32).reshape(-1)
        return np.stack([x, y, z], axis=1)

    def _fit_anchored_floor(self, x, y, z):
        """Gentle floor plane z=a*x+b*y+c seeded by points near the known floor
        height (|z| < floor_seed_z). Returns (a,b,c), or (0,0,0) when no floor seed
        is visible (reference the ground plane z=0 directly, so tall structure is
        still measured and marked)."""
        seed = np.abs(z) < self.floor_seed_z
        if seed.sum() < 50:
            self.no_floor_count += 1
            if self.no_floor_count % 30 == 0:
                self.get_logger().warn(
                    'No floor visible (no points near z=0); measuring obstacles '
                    'against ground plane z=0. Camera may be pitched or boxed in.')
            return 0.0, 0.0, 0.0
        xs, ys, zs = x[seed], y[seed], z[seed]
        A = np.column_stack((xs, ys, np.ones(len(xs))))
        a, b, c = np.linalg.lstsq(A, zs, rcond=None)[0]
        # one robust refit: drop seed points that don't fit the first plane
        keep = np.abs(zs - (a*xs + b*ys + c)) < 0.02
        if keep.sum() > 50:
            A = np.column_stack((xs[keep], ys[keep], np.ones(int(keep.sum()))))
            a, b, c = np.linalg.lstsq(A, zs[keep], rcond=None)[0]
        # a real floor is near-horizontal; if the fit came out steep, distrust it
        if math.hypot(a, b) > 0.30:   # ~17 deg
            return 0.0, 0.0, 0.0
        return a, b, c

    def _classify_obstacles(self, points):
        """Return the subset of `points` (N,3, base_footprint) that are obstacles.
        Full FOV; obstacle = height above the anchored floor exceeds a range-aware
        threshold."""
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        rng = np.sqrt(x*x + y*y)
        gate = (rng > self.range_min) & (rng < self.range_max) & (z > -0.15) & (z < 1.0)
        xg, yg, zg, rg = x[gate], y[gate], z[gate], rng[gate]
        if len(zg) < 100:
            return np.empty((0, 3), dtype=np.float32)

        a, b, c = self._fit_anchored_floor(xg, yg, zg)
        resid = zg - (a*xg + b*yg + c)

        # range-aware per-point threshold: base + alpha*range^2 (depth noise law)
        thr = self.base_step + self.noise_alpha * rg * rg
        is_obst_pt = resid > thr
        if not is_obst_pt.any():
            return np.empty((0, 3), dtype=np.float32)

        # grid; require >= min_cell_points obstacle points in a cell to mark it
        # (rejects isolated noise speckles). Then emit every obstacle point in
        # surviving cells.
        ix = np.floor(xg / self.grid).astype(np.int64)
        iy = np.floor(yg / self.grid).astype(np.int64)
        ix -= ix.min(); iy -= iy.min()
        width = ix.max() + 1
        key = iy * width + ix

        obst_keys = key[is_obst_pt]
        uniq, counts = np.unique(obst_keys, return_counts=True)
        good_cells = uniq[counts >= self.min_cell_points]
        if len(good_cells) == 0:
            return np.empty((0, 3), dtype=np.float32)

        emit = is_obst_pt & np.isin(key, good_cells)
        return np.column_stack((xg[emit], yg[emit], zg[emit])).astype(np.float32)

    def _voxel_grid(self, points, leaf_size):
        if len(points) == 0:
            return points
        voxel_idx = np.floor(points / leaf_size).astype(np.int64)
        keys = (voxel_idx[:, 0] * 1_000_003 +
                voxel_idx[:, 1] * 1_009 +
                voxel_idx[:, 2])
        _, first_idx = np.unique(keys, return_index=True)
        return points[first_idx]

    def _xyz_to_pointcloud2(self, points, frame_id, stamp):
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id, stamp=stamp)
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg

    def pointcloud_callback(self, msg):
        try:
            self.frame_count += 1

            if not self._ensure_transform(msg.header.frame_id):
                return

            points = self._pointcloud2_to_xyz(msg)
            if len(points) == 0:
                return

            transformed = points @ self._rot_T + self._translation
            obstacles = self._classify_obstacles(transformed)

            if self.voxel_leaf_size > 0 and len(obstacles) > 0:
                obstacles = self._voxel_grid(obstacles, self.voxel_leaf_size)

            # Publish every frame (even 0 obstacles): a clear frame is a valid
            # observation. Note the depth costmap layer reclaims free space by its
            # own time-decay, not by raytracing this cloud -- so this publish marks
            # obstacles; it does not itself clear.
            self.publisher.publish(
                self._xyz_to_pointcloud2(obstacles, self.target_frame, msg.header.stamp))

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTraversabilityFilter()
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
