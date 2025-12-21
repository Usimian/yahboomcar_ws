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
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import struct


class PointCloudHeightFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_height_filter')

        # Declare parameters
        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/points_filtered')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_height', -0.062)
        self.declare_parameter('max_height', 0.21)
        self.declare_parameter('filter_nans', True)
        self.declare_parameter('voxel_leaf_size', 0.03)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.filter_nans = self.get_parameter('filter_nans').value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').value

        # TF2 setup
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Give TF buffer time to fill
        self.get_logger().info('Waiting 2 seconds for TF buffer to fill...')
        import time
        time.sleep(2.0)

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )

        self.get_logger().info(f'Point cloud height filter node started')
        self.get_logger().info(f'  Input: {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Target frame: {self.target_frame}')
        self.get_logger().info(f'  Height range: [{self.min_height}, {self.max_height}] m')
        self.get_logger().info(f'  Filter NaNs: {self.filter_nans}')
        self.get_logger().info(f'  Voxel leaf size: {self.voxel_leaf_size} m')

        self.frame_count = 0
        self.error_count = 0

    def pointcloud_callback(self, msg):
        """Process incoming point cloud message"""
        try:
            self.frame_count += 1

            if self.frame_count == 1:
                self.get_logger().info(f'Received first point cloud message from frame: {msg.header.frame_id}')

            # Get transform from point cloud frame to target frame
            # Use time(0) for latest available transform (handles static transforms)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),  # Use current time for static transforms
                    timeout=Duration(seconds=1.0)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                if self.error_count % 30 == 0:  # Log every 30th error to avoid spam
                    self.get_logger().warn(f'TF lookup failed: {e}')
                self.error_count += 1
                return

            # Extract points from message
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=self.filter_nans))

            if len(points_list) == 0:
                return

            # Convert structured array to regular numpy array
            # points_list is a list of tuples like [(x1, y1, z1), (x2, y2, z2), ...]
            points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)

            # Transform points to target frame
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to rotation matrix
            q = rotation
            rot_matrix = np.array([
                [1 - 2*(q.y**2 + q.z**2),     2*(q.x*q.y - q.z*q.w),     2*(q.x*q.z + q.y*q.w)],
                [    2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x**2 + q.z**2),     2*(q.y*q.z - q.x*q.w)],
                [    2*(q.x*q.z - q.y*q.w),     2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x**2 + q.y**2)]
            ])

            # Apply rotation and translation
            transformed_points = (rot_matrix @ points.T).T
            transformed_points[:, 0] += translation.x
            transformed_points[:, 1] += translation.y
            transformed_points[:, 2] += translation.z

            # Filter by height (z-coordinate in target frame)
            height_mask = (transformed_points[:, 2] >= self.min_height) & (transformed_points[:, 2] <= self.max_height)
            filtered_points = transformed_points[height_mask]

            if self.frame_count == 1:
                self.get_logger().info(f'After height filter: {len(filtered_points)} points (from {len(points)} original)')

            # Apply voxel grid downsampling if enabled
            if self.voxel_leaf_size > 0:
                filtered_points = self.voxel_grid_filter(filtered_points, self.voxel_leaf_size)

            # Create output message
            if len(filtered_points) > 0:
                output_msg = self.create_point_cloud_message(
                    filtered_points,
                    self.target_frame,
                    msg.header.stamp
                )
                self.publisher.publish(output_msg)

                if self.frame_count % 100 == 0:
                    reduction_pct = 100.0 * (1.0 - len(filtered_points) / len(points))
                    self.get_logger().info(
                        f'Processed {self.frame_count} frames. '
                        f'Input: {len(points)} pts, Output: {len(filtered_points)} pts '
                        f'({reduction_pct:.1f}% reduction)'
                    )

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def voxel_grid_filter(self, points, leaf_size):
        """Simple voxel grid downsampling"""
        if len(points) == 0:
            return points

        # Compute voxel indices
        voxel_indices = np.floor(points / leaf_size).astype(np.int32)

        # Find unique voxels
        unique_voxels, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)

        # Compute centroid of points in each voxel
        downsampled_points = np.zeros((len(unique_voxels), 3), dtype=np.float32)
        for i in range(len(unique_voxels)):
            mask = inverse_indices == i
            downsampled_points[i] = np.mean(points[mask], axis=0)

        return downsampled_points

    def create_point_cloud_message(self, points, frame_id, stamp):
        """Create a PointCloud2 message from numpy array"""
        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp

        # Create point cloud message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = not self.filter_nans
        msg.is_bigendian = False

        # Define fields
        msg.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]

        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width

        # Pack point data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))

        msg.data = b''.join(buffer)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudHeightFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
