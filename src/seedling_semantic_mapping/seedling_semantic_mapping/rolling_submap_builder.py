#!/usr/bin/env python3
"""Build a dense rolling work-zone submap from registered FAST-LIVO scans."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


@dataclass
class Scan:
    stamp_sec: float
    points_world: np.ndarray


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def odom_pose(odom: Odometry) -> Tuple[np.ndarray, np.ndarray]:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    translation = np.array([p.x, p.y, p.z], dtype=np.float64)
    rotation = quat_to_rot(q.x, q.y, q.z, q.w)
    return rotation, translation


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    if points.shape[0] == 0 or leaf <= 0.0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


class RollingSubmapBuilder(Node):
    def __init__(self) -> None:
        super().__init__("rolling_submap_builder")

        self.declare_parameter("input_cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("output_cloud_topic", "/seedling/local_submap_body")
        self.declare_parameter("world_frame", "camera_init")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("window_sec", 1.5)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("voxel_size_m", 0.02)
        self.declare_parameter("forward_m", 5.0)
        self.declare_parameter("backward_m", 2.0)
        self.declare_parameter("lateral_half_width_m", 1.5)
        self.declare_parameter("below_m", 1.0)
        self.declare_parameter("above_m", 1.5)
        self.declare_parameter("crop_margin_m", 1.0)
        self.declare_parameter("cloud_stride", 1)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("min_points", 100)

        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_cloud_topic = str(self.get_parameter("output_cloud_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.voxel_size_m = float(self.get_parameter("voxel_size_m").value)
        self.forward_m = float(self.get_parameter("forward_m").value)
        self.backward_m = float(self.get_parameter("backward_m").value)
        self.lateral_half_width_m = float(
            self.get_parameter("lateral_half_width_m").value
        )
        self.below_m = float(self.get_parameter("below_m").value)
        self.above_m = float(self.get_parameter("above_m").value)
        self.crop_margin_m = float(self.get_parameter("crop_margin_m").value)
        self.cloud_stride = max(1, int(self.get_parameter("cloud_stride").value))
        self.max_points = max(1000, int(self.get_parameter("max_points").value))
        self.min_points = max(1, int(self.get_parameter("min_points").value))

        self.scans: Deque[Scan] = deque()
        self.latest_odom: Optional[Odometry] = None
        self.latest_cloud_stamp_sec: Optional[float] = None

        qos_cloud = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_odom = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_output = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self.cloud_cb,
            qos_cloud,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            qos_odom,
        )
        self.submap_pub = self.create_publisher(
            PointCloud2,
            self.output_cloud_topic,
            qos_output,
        )

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.publish_timer = self.create_timer(period, self.publish_submap)

        self.get_logger().info(
            f"RollingSubmapBuilder: input={self.input_cloud_topic}, "
            f"output={self.output_cloud_topic}, window={self.window_sec:.2f}s"
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def cloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        if not msg.fields or not msg.data or msg.width * msg.height == 0:
            return np.empty((0, 3), dtype=np.float64)
                
        field_lookup = {
            field.name.lower(): field.name
            for field in msg.fields
        }

        missing = [
            name for name in ("x", "y", "z")
            if name not in field_lookup
        ]
        if missing:
            available = ", ".join(
                field.name for field in msg.fields
            ) or "<none>"

            self.get_logger().error(
                f"Input cloud has no XYZ fields; "
                f"missing={missing}, available=[{available}]",
                throttle_duration_sec=2.0,
            )
            return np.empty((0, 3), dtype=np.float64)

        xyz_names = tuple(
            field_lookup[name] for name in ("x", "y", "z")
        )

        try:
            # 不指定 field_names，兼容 Humble 和 FAST-LIVO 的混合字段点云。
            structured = point_cloud2.read_points(
                msg,
                field_names=None,
                skip_nans=False,
            )

            dtype_names = getattr(
                getattr(structured, "dtype", None),
                "names",
                None,
            )
            if not dtype_names:
                raise ValueError(
                    "read_points returned no structured fields"
                )

            points = np.stack(
                [
                    np.asarray(
                        structured[name],
                        dtype=np.float64,
                    ).reshape(-1)
                    for name in xyz_names
                ],
                axis=1,
            )

        except Exception as exc:
            self.get_logger().error(
                f"Failed to decode PointCloud2 "
                f"fields {xyz_names}: {exc}",
                throttle_duration_sec=2.0,
            )
            return np.empty((0, 3), dtype=np.float64)

        if self.cloud_stride > 1:
            points = points[::self.cloud_stride]

        return points[np.isfinite(points).all(axis=1)]

    def crop_body(self, points_body: np.ndarray, margin: float) -> np.ndarray:
        mask = (
            (points_body[:, 0] >= -(self.backward_m + margin))
            & (points_body[:, 0] <= self.forward_m + margin)
            & (np.abs(points_body[:, 1]) <= self.lateral_half_width_m + margin)
            & (points_body[:, 2] >= -(self.below_m + margin))
            & (points_body[:, 2] <= self.above_m + margin)
        )
        return mask

    def prune_scans(self, reference_sec: float) -> None:
        oldest = reference_sec - self.window_sec
        while self.scans and self.scans[0].stamp_sec < oldest:
            self.scans.popleft()

    def cloud_cb(self, msg: PointCloud2) -> None:
        if self.latest_odom is None:
            return
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().error(
                f"Input cloud frame is '{msg.header.frame_id}', expected "
                f"'{self.world_frame}'.",
                throttle_duration_sec=2.0,
            )
            return

        points_world = self.cloud_to_xyz(msg)
        if points_world.shape[0] == 0:
            return

        rotation, translation = odom_pose(self.latest_odom)
        points_body = (
            rotation.T @ (points_world - translation.reshape(1, 3)).T
        ).T
        keep = self.crop_body(points_body, self.crop_margin_m)
        points_world = points_world[keep]
        points_world = voxel_downsample(points_world, self.voxel_size_m)
        if points_world.shape[0] == 0:
            return

        stamp_sec = stamp_to_sec(msg.header.stamp)
        self.latest_cloud_stamp_sec = stamp_sec
        self.scans.append(Scan(stamp_sec, points_world))
        self.prune_scans(stamp_sec)

    def publish_submap(self) -> None:
        if (
            self.latest_odom is None
            or self.latest_cloud_stamp_sec is None
            or not self.scans
        ):
            return

        self.prune_scans(self.latest_cloud_stamp_sec)
        points_world = np.concatenate(
            [scan.points_world for scan in self.scans],
            axis=0,
        )

        rotation, translation = odom_pose(self.latest_odom)
        points_body = (
            rotation.T @ (points_world - translation.reshape(1, 3)).T
        ).T
        points_body = points_body[self.crop_body(points_body, 0.0)]
        points_body = voxel_downsample(points_body, self.voxel_size_m)

        if points_body.shape[0] < self.min_points:
            return
        if points_body.shape[0] > self.max_points:
            step = int(math.ceil(points_body.shape[0] / self.max_points))
            points_body = points_body[::step]

        header = Header()
        header.stamp = self.latest_odom.header.stamp
        header.frame_id = self.body_frame
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            points_body.astype(np.float32),
        )
        self.submap_pub.publish(cloud_msg)

        self.get_logger().info(
            f"local_submap scans={len(self.scans)} points={points_body.shape[0]}",
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RollingSubmapBuilder()
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
