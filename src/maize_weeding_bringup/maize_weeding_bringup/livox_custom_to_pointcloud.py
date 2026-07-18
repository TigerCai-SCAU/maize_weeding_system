#!/usr/bin/env python3
"""Convert Livox CustomMsg scans to a body-frame XYZ PointCloud2."""

import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


def filtered_xyz(points, min_range_m: float, max_range_m: float) -> np.ndarray:
    xyz = np.asarray(
        [(point.x, point.y, point.z) for point in points],
        dtype=np.float32,
    ).reshape(-1, 3)
    if xyz.size == 0:
        return xyz
    range_sq = np.einsum("ij,ij->i", xyz, xyz)
    mask = np.isfinite(xyz).all(axis=1)
    mask &= range_sq >= min_range_m * min_range_m
    mask &= range_sq <= max_range_m * max_range_m
    return np.ascontiguousarray(xyz[mask], dtype=np.float32)


def make_xyz_cloud(header, xyz: np.ndarray, frame_id: str) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = header
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = int(xyz.shape[0])
    cloud.fields = XYZ_FIELDS
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = xyz.astype("<f4", copy=False).tobytes()
    cloud.is_dense = True
    return cloud


class LivoxCustomToPointCloud(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_pointcloud")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/bench/livox_points")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("min_range_m", 0.8)
        self.declare_parameter("max_range_m", 50.0)
        self.declare_parameter("publish_every_n_scans", 1)
        self.declare_parameter("log_period_sec", 5.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.publish_every_n_scans = int(
            self.get_parameter("publish_every_n_scans").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        if not 0.0 <= self.min_range_m < self.max_range_m:
            raise ValueError("expected 0 <= min_range_m < max_range_m")
        if self.publish_every_n_scans < 1:
            raise ValueError("publish_every_n_scans must be positive")
        if self.log_period_sec <= 0.0:
            raise ValueError("log_period_sec must be positive")

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            CustomMsg,
            self.input_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.scan_count = 0
        self.publish_count = 0
        self.last_log_monotonic = time.monotonic()
        self.get_logger().info(
            "Livox bridge started: %s -> %s frame=%s range=[%.2f, %.2f]m"
            % (
                self.input_topic,
                self.output_topic,
                self.frame_id,
                self.min_range_m,
                self.max_range_m,
            )
        )

    def scan_cb(self, msg: CustomMsg) -> None:
        self.scan_count += 1
        if (self.scan_count - 1) % self.publish_every_n_scans:
            return

        xyz = filtered_xyz(
            msg.points,
            self.min_range_m,
            self.max_range_m,
        )
        if xyz.shape[0] == 0:
            self.get_logger().warn(
                "Livox scan contains no valid XYZ points",
                throttle_duration_sec=2.0,
            )
            return

        self.publisher.publish(make_xyz_cloud(msg.header, xyz, self.frame_id))
        self.publish_count += 1

        now = time.monotonic()
        if now - self.last_log_monotonic >= self.log_period_sec:
            self.get_logger().info(
                "Livox bridge healthy: scans=%d published=%d latest_points=%d"
                % (self.scan_count, self.publish_count, xyz.shape[0])
            )
            self.last_log_monotonic = now


def main() -> None:
    rclpy.init()
    node = LivoxCustomToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
