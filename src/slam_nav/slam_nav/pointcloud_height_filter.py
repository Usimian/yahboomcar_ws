#!/usr/bin/env python3

"""
Point Cloud Height Filter Node

Filters a point cloud by height relative to base_link frame, removing floor and ceiling points.
This keeps only obstacles relevant for navigation at robot height.

Subscribes to: /camera/depth/color/points (sensor_msgs/msg/PointCloud2)
Publishes to: /camera/depth/points_filtered (sensor_msgs/msg/PointCloud2)

Parameters:
- input_topic: Input point cloud topic (default: /camera/depth/color/points)
- output_topic: Output filtered point cloud topic (default: /camera/depth/points_filtered)
- target_frame: Frame to filter relative to (default: base_link)
- min_height: Minimum height in meters (default: -0.062)
- max_height: Maximum height in meters (default: 0.21)
- filter_nans: Remove NaN/invalid points (default: true)
- voxel_leaf_size: Voxel grid downsampling size in meters, 0 to disable (default: 0.03)
"""

# Cap BLAS/OpenMP thread pools BEFORE numpy is imported. The per-frame
# matmul is a trivial (3,3)@(3,N) operation; OpenBLAS's default policy of
# spawning one thread per core costs more than the work itself (measured:
# 6 threads at ~80% each, ~455% total CPU). Single-threaded is strictly
# faster for this workload and frees cores for the rest of the ROS stack.
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration


class PointCloudHeightFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_height_filter')

        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/points_filtered')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_height', -0.062)
        self.declare_parameter('max_height', 0.21)
        self.declare_parameter('filter_nans', True)
        self.declare_parameter('voxel_leaf_size', 0.03)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.filter_nans = self.get_parameter('filter_nans').value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').value

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cache: camera-optical-frame → target_frame transform. The camera is
        # rigidly mounted so this is effectively static.
        self._rot_T = None          # pre-transposed for points @ rot.T
        self._translation = None
        self._cached_field_layout = None   # (point_step, ox, oy, oz)
        self._cached_xyz_contiguous = None # whether xyz is contiguous in the point buffer

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.frame_count = 0
        self.error_count = 0
        self.get_logger().info(
            f'Height filter started: {self.input_topic} -> {self.output_topic} '
            f'height=[{self.min_height}, {self.max_height}] voxel={self.voxel_leaf_size} '
            f'threads=1'
        )

    def _ensure_transform(self, source_frame):
        # The camera is on a tilt servo, so this transform changes every time the
        # camera tilts -- it MUST be looked up live each frame, not cached. (A
        # stale cache rotated the floor by the uncorrected tilt angle, lifting it
        # into the obstacle band.) The TF buffer is local so per-frame lookup is cheap.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
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
        if layout is None or layout[0] != msg.point_step:
            field_map = {f.name: f.offset for f in msg.fields}
            ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
            layout = (msg.point_step, ox, oy, oz)
            self._cached_field_layout = layout
            # Standard layout: x/y/z contiguous float32 at the start of each point.
            self._cached_xyz_contiguous = (ox == 0 and oy == 4 and oz == 8)
        point_step, ox, oy, oz = layout

        n_points = msg.width * msg.height

        if self._cached_xyz_contiguous and point_step == 12:
            # Tightly-packed xyz points, no padding: reinterpret the buffer
            # directly, no copy.
            xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, 3)
        elif self._cached_xyz_contiguous:
            # xyz contiguous but with per-point padding (e.g. RGB, normals):
            # reinterpret the whole buffer as a structured view of shape
            # (N, point_step/4) and take the first 3 float columns.
            quads = point_step // 4
            if point_step % 4 == 0:
                all_floats = np.frombuffer(msg.data, dtype=np.float32).reshape(n_points, quads)
                # Slicing a view here does not copy; arithmetic below triggers
                # at most one materialisation.
                xyz = all_floats[:, :3]
            else:
                xyz = self._extract_xyz_fallback(msg.data, n_points, point_step, ox, oy, oz)
        else:
            xyz = self._extract_xyz_fallback(msg.data, n_points, point_step, ox, oy, oz)

        if self.filter_nans:
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
        return xyz

    @staticmethod
    def _extract_xyz_fallback(data_bytes, n_points, point_step, ox, oy, oz):
        """Per-axis extraction when xyz are not contiguous."""
        raw = np.frombuffer(data_bytes, dtype=np.uint8).reshape(n_points, point_step)
        x = raw[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        y = raw[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        z = raw[:, oz:oz+4].copy().view(np.float32).reshape(-1)
        return np.stack([x, y, z], axis=1)

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

    def _voxel_grid(self, points, leaf_size):
        if len(points) == 0:
            return points
        voxel_idx = np.floor(points / leaf_size).astype(np.int32)
        keys = (voxel_idx[:, 0].astype(np.int64) * 1_000_003 +
                voxel_idx[:, 1].astype(np.int64) * 1_009 +
                voxel_idx[:, 2].astype(np.int64))
        _, first_idx = np.unique(keys, return_index=True)
        return points[first_idx]

    def pointcloud_callback(self, msg):
        try:
            self.frame_count += 1

            if not self._ensure_transform(msg.header.frame_id):
                return

            points = self._pointcloud2_to_xyz(msg)
            if len(points) == 0:
                return

            transformed = points @ self._rot_T + self._translation

            z = transformed[:, 2]
            mask = (z >= self.min_height) & (z <= self.max_height)
            filtered = transformed[mask]

            if self.voxel_leaf_size > 0:
                filtered = self._voxel_grid(filtered, self.voxel_leaf_size)

            if len(filtered) == 0:
                return

            self.publisher.publish(
                self._xyz_to_pointcloud2(filtered, self.target_frame, msg.header.stamp)
            )

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudHeightFilter()
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
