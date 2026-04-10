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
            f'height=[{self.min_height}, {self.max_height}] voxel={self.voxel_leaf_size}'
        )

    def _pointcloud2_to_xyz(self, msg):
        """Extract xyz as (N,3) float32 numpy array from PointCloud2, skipping NaNs if requested."""
        field_map = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
        point_step = msg.point_step
        n_points = msg.width * msg.height

        # Read as uint8, reshape to (N, point_step), then extract float32 columns by byte offset
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)
        x = data[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        y = data[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        z = data[:, oz:oz+4].copy().view(np.float32).reshape(-1)
        points = np.stack([x, y, z], axis=1)  # (N, 3)

        if self.filter_nans:
            valid = np.isfinite(points).all(axis=1)
            points = points[valid]

        return points

    def _xyz_to_pointcloud2(self, points, frame_id, stamp):
        """Pack (N,3) float32 numpy array into a PointCloud2 message."""
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id, stamp=stamp)
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 × float32
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def _voxel_grid(self, points, leaf_size):
        """Vectorised voxel grid: one representative point per voxel (first-point, not centroid)."""
        if len(points) == 0:
            return points
        voxel_idx = np.floor(points / leaf_size).astype(np.int32)
        # Pack three int32 columns into one int64 key for unique()
        keys = (voxel_idx[:, 0].astype(np.int64) * 1_000_003 +
                voxel_idx[:, 1].astype(np.int64) * 1_009 +
                voxel_idx[:, 2].astype(np.int64))
        _, first_idx = np.unique(keys, return_index=True)
        return points[first_idx]

    def pointcloud_callback(self, msg):
        try:
            self.frame_count += 1

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.error_count += 1
                if self.error_count % 30 == 0:
                    self.get_logger().warn(f'TF lookup failed: {e}')
                return

            points = self._pointcloud2_to_xyz(msg)
            if len(points) == 0:
                return

            # Build rotation matrix from quaternion
            q = transform.transform.rotation
            t = transform.transform.translation
            rot = np.array([
                [1 - 2*(q.y**2 + q.z**2),   2*(q.x*q.y - q.z*q.w),   2*(q.x*q.z + q.y*q.w)],
                [    2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x**2 + q.z**2),   2*(q.y*q.z - q.x*q.w)],
                [    2*(q.x*q.z - q.y*q.w),   2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x**2 + q.y**2)],
            ], dtype=np.float64)

            transformed = (rot @ points.T).T + np.array([t.x, t.y, t.z])

            # Height filter (z in target frame)
            mask = (transformed[:, 2] >= self.min_height) & (transformed[:, 2] <= self.max_height)
            filtered = transformed[mask].astype(np.float32)

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
