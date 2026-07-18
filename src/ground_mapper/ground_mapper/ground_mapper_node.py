#!/usr/bin/env python3
"""Adaptive rolling ground map for the maize weeding robot."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import (
    Float64,
    Float64MultiArray,
    Header,
    String,
)
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class TimedPose:
    stamp_sec: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass
class StoredScan:
    stamp_sec: float
    points_world: np.ndarray


@dataclass
class PlaneFit:
    normal_body: np.ndarray
    d_body: float
    rmse: float
    inlier_ratio: float
    inlier_count: int


@dataclass
class GroundMapCell:
    point_world: np.ndarray
    observations: int
    last_seen_sec: float


@dataclass
class TerrainGrid:
    heights: np.ndarray
    supported: np.ndarray
    surface_points_body: np.ndarray
    lateral_count: int
    forward_count: int


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return np.asarray(q, dtype=np.float64) / norm


def quaternion_to_rotation(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q_xyzw)
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_slerp(
    q0_xyzw: np.ndarray,
    q1_xyzw: np.ndarray,
    ratio: float,
) -> np.ndarray:
    q0 = normalize_quaternion(q0_xyzw)
    q1 = normalize_quaternion(q1_xyzw)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(q0 + ratio * (q1 - q0))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    w0 = math.sin((1.0 - ratio) * theta) / sin_theta
    w1 = math.sin(ratio * theta) / sin_theta
    return normalize_quaternion(w0 * q0 + w1 * q1)


def read_xyz_points(msg: PointCloud2) -> np.ndarray:
    """Decode x/y/z from common FAST-LIVO PointCloud2 layouts."""
    if not msg.fields or not msg.data or msg.width * msg.height <= 0:
        return np.empty((0, 3), dtype=np.float64)

    names = {field.name.lower(): field.name for field in msg.fields}
    if any(name not in names for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    xyz_names = tuple(names[name] for name in ("x", "y", "z"))

    try:
        values = point_cloud2.read_points_numpy(
            msg,
            field_names=xyz_names,
            skip_nans=False,
        )
        points = np.asarray(values, dtype=np.float64).reshape(-1, 3)
    except (AssertionError, AttributeError, TypeError, ValueError):
        try:
            values = point_cloud2.read_points(
                msg,
                field_names=xyz_names,
                skip_nans=False,
            )
            dtype_names = getattr(
                getattr(values, "dtype", None),
                "names",
                None,
            )
            if dtype_names:
                points = np.stack(
                    [
                        np.asarray(values[name], dtype=np.float64).reshape(-1)
                        for name in xyz_names
                    ],
                    axis=1,
                )
            else:
                points = np.asarray(
                    [
                        (float(point[0]), float(point[1]), float(point[2]))
                        for point in values
                    ],
                    dtype=np.float64,
                ).reshape(-1, 3)
        except (AssertionError, AttributeError, TypeError, ValueError):
            return np.empty((0, 3), dtype=np.float64)

    return points[np.isfinite(points).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    return (rotation @ points.T).T + translation.reshape(1, 3)


def inverse_transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    return (rotation.T @ (points - translation.reshape(1, 3)).T).T


def make_xyz_cloud(
    points: np.ndarray,
    stamp,
    frame_id: str,
) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    if points.shape[0] == 0:
        return point_cloud2.create_cloud_xyz32(header, [])
    return point_cloud2.create_cloud_xyz32(
        header,
        np.asarray(points[:, :3], dtype=np.float32).tolist(),
    )


class GroundMapper(Node):
    """Build a locally dense ground map from registered FAST-LIVO scans."""

    def __init__(self) -> None:
        super().__init__("ground_mapper_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Keep this ROS queue deliberately short. The Python plane fitter
            # can occupy the single-threaded executor for a while; a deep,
            # reliable queue would replay stale odometry afterwards and make
            # the registered cloud appear to be at the wrong body pose.
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(depth=10)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_cb,
            sensor_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            odom_qos,
        )

        self.ground_pub = self.create_publisher(
            PointCloud2,
            self.ground_points_topic,
            output_qos,
        )
        self.non_ground_pub = self.create_publisher(
            PointCloud2,
            self.non_ground_points_topic,
            output_qos,
        )
        self.elevation_pub = self.create_publisher(
            PointCloud2,
            self.elevation_points_topic,
            output_qos,
        )
        self.global_elevation_pub = self.create_publisher(
            PointCloud2,
            self.global_elevation_points_topic,
            output_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.elevation_markers_topic,
            status_qos,
        )
        self.plane_pub = self.create_publisher(
            Float64MultiArray,
            self.plane_topic,
            status_qos,
        )
        self.height_pub = self.create_publisher(
            Float64,
            self.sensor_height_topic,
            status_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos,
        )
        self.save_map_service = self.create_service(
            Trigger,
            "/ground/save_map",
            self.save_map_cb,
        )
        self.clear_map_service = self.create_service(
            Trigger,
            "/ground/clear_map",
            self.clear_map_cb,
        )

        self.odom_buffer: Deque[TimedPose] = deque()
        self.scans: Deque[StoredScan] = deque()
        self.latest_pose: Optional[TimedPose] = None
        self.latest_cloud_stamp = None
        self.latest_cloud_stamp_sec: Optional[float] = None

        self.plane_normal_world: Optional[np.ndarray] = None
        self.plane_d_world: Optional[float] = None
        self.last_plane_update_sec: Optional[float] = None
        self.last_fit_rmse = math.nan
        self.last_fit_ratio = 0.0

        self.ground_map_cells: Dict[Tuple[int, int], GroundMapCell] = {}
        self.last_global_map_publish_sec: Optional[float] = None
        self.map_capacity_warning_issued = False

        self.random = np.random.default_rng(20260713)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_ground_map,
        )

        self.get_logger().info(
            "Adaptive GroundMapper 0.4.0: "
            f"cloud={self.cloud_topic} ({self.cloud_frame_mode} frame), "
            f"odom={self.odom_topic}, "
            f"body_axes=(vertical={self.vertical_axis}, "
            f"lateral={self.lateral_axis}, forward={self.forward_axis}, "
            f"forward_sign={self.forward_sign:+.0f}, "
            f"ground_normal_sign={self.ground_normal_sign:+.0f}), "
            f"y=[{self.lateral_min_m:.2f},{self.lateral_max_m:.2f}] m, "
            f"map_mode={self.map_mode}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "cloud_topic": "/cloud_registered",
            "cloud_frame_mode": "map",
            "odom_topic": "/aft_mapped_to_init",
            "map_frame": "camera_init",
            "body_frame": "body",
            "ground_points_topic": "/ground/points",
            "non_ground_points_topic": "/ground/non_ground_points",
            "elevation_points_topic": "/ground/elevation_points",
            "global_elevation_points_topic": (
                "/ground/global_elevation_points"
            ),
            "elevation_markers_topic": "/ground/elevation_markers",
            "plane_topic": "/ground/plane",
            "sensor_height_topic": "/ground/sensor_height",
            "status_topic": "/ground/status",
            "vertical_axis": 0,
            "lateral_axis": 1,
            "forward_axis": 2,
            "forward_sign": -1.0,
            # 0 accepts either plane orientation. +/-1 constrains the fitted
            # normal on vertical_axis after it is oriented to keep d positive.
            "ground_normal_sign": 0.0,
            "lateral_min_m": -1.5,
            "lateral_max_m": 1.5,
            "forward_min_m": -1.0,
            "forward_max_m": 5.0,
            "window_sec": 1.5,
            "publish_rate_hz": 2.0,
            "input_voxel_size_m": 0.03,
            "max_points_per_scan": 30000,
            "max_rolling_points": 120000,
            "odom_cache_size": 300,
            "max_cloud_odom_dt_sec": 0.30,
            "ransac_iterations": 60,
            "ransac_sample_points": 5000,
            "ransac_distance_m": 0.08,
            "max_normal_angle_deg": 35.0,
            "min_plane_inliers": 250,
            "min_plane_inlier_ratio": 0.05,
            "max_plane_rmse_m": 0.12,
            "min_sensor_height_m": 0.25,
            "max_sensor_height_m": 3.0,
            "max_height_jump_m": 0.20,
            "plane_smoothing_alpha": 0.20,
            "plane_timeout_sec": 3.0,
            "ground_below_margin_m": 0.10,
            "ground_above_margin_m": 0.06,
            "non_ground_min_height_m": 0.06,
            "max_object_height_m": 0.60,
            "terrain_coarse_below_m": 0.35,
            "terrain_coarse_above_m": 0.30,
            "terrain_height_quantile": 0.20,
            "terrain_min_points_per_cell": 2,
            "terrain_max_neighbor_jump_m": 0.12,
            "terrain_min_neighbor_cells": 2,
            "terrain_min_fill_neighbors": 2,
            "grid_resolution_m": 0.05,
            "max_fill_distance_m": 0.15,
            "publish_full_plane_roi": False,
            "max_dense_cells": 12000,
            "map_mode": "distance",
            "map_resolution_m": 0.05,
            "map_keep_behind_m": 5.0,
            "map_keep_ahead_m": 5.0,
            "map_max_cells": 300000,
            "map_height_smoothing_alpha": 0.20,
            "map_publish_rate_hz": 0.5,
            "save_map_path": "/tmp/ground_global_map.csv",
            "save_map_on_shutdown": False,
            "publish_non_ground": True,
            "publish_markers": True,
            "marker_max_points": 5000,
            "log_debug": True,
        }
        self.parameter_names = tuple(defaults.keys())
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        string_names = (
            "cloud_topic",
            "cloud_frame_mode",
            "odom_topic",
            "map_frame",
            "body_frame",
            "ground_points_topic",
            "non_ground_points_topic",
            "elevation_points_topic",
            "global_elevation_points_topic",
            "elevation_markers_topic",
            "plane_topic",
            "sensor_height_topic",
            "status_topic",
            "map_mode",
            "save_map_path",
        )
        int_names = (
            "vertical_axis",
            "lateral_axis",
            "forward_axis",
            "max_points_per_scan",
            "max_rolling_points",
            "odom_cache_size",
            "ransac_iterations",
            "ransac_sample_points",
            "min_plane_inliers",
            "terrain_min_points_per_cell",
            "terrain_min_neighbor_cells",
            "terrain_min_fill_neighbors",
            "max_dense_cells",
            "map_max_cells",
            "marker_max_points",
        )
        bool_names = (
            "publish_full_plane_roi",
            "save_map_on_shutdown",
            "publish_non_ground",
            "publish_markers",
            "log_debug",
        )
        for name in self.parameter_names:
            value = self.get_parameter(name).value
            if name in string_names:
                value = str(value)
            elif name in int_names:
                value = int(value)
            elif name in bool_names:
                value = bool(value)
            else:
                value = float(value)
            setattr(self, name, value)

        self.max_points_per_scan = max(1000, self.max_points_per_scan)
        self.max_rolling_points = max(1000, self.max_rolling_points)
        self.odom_cache_size = max(10, self.odom_cache_size)
        self.ransac_iterations = max(10, self.ransac_iterations)
        self.ransac_sample_points = max(100, self.ransac_sample_points)
        self.min_plane_inliers = max(20, self.min_plane_inliers)
        self.terrain_min_points_per_cell = max(
            1,
            self.terrain_min_points_per_cell,
        )
        self.terrain_min_neighbor_cells = max(
            1,
            self.terrain_min_neighbor_cells,
        )
        self.terrain_min_fill_neighbors = max(
            1,
            self.terrain_min_fill_neighbors,
        )
        self.max_dense_cells = max(100, self.max_dense_cells)
        self.map_max_cells = max(1000, self.map_max_cells)
        self.marker_max_points = max(100, self.marker_max_points)

    def _validate_parameters(self) -> None:
        axes = {
            self.vertical_axis,
            self.lateral_axis,
            self.forward_axis,
        }
        if axes != {0, 1, 2}:
            raise ValueError(
                "vertical_axis, lateral_axis, and forward_axis must be "
                "a permutation of 0, 1, 2"
            )
        if abs(abs(self.forward_sign) - 1.0) > 1e-6:
            raise ValueError("forward_sign must be +1.0 or -1.0")
        if self.ground_normal_sign not in (-1.0, 0.0, 1.0):
            raise ValueError(
                "ground_normal_sign must be -1.0, 0.0, or +1.0"
            )
        if self.lateral_min_m >= self.lateral_max_m:
            raise ValueError("lateral_min_m must be smaller than lateral_max_m")
        if self.forward_min_m >= self.forward_max_m:
            raise ValueError("forward_min_m must be smaller than forward_max_m")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.grid_resolution_m <= 0.0:
            raise ValueError("grid_resolution_m must be positive")
        if not 0.0 <= self.terrain_height_quantile <= 1.0:
            raise ValueError("terrain_height_quantile must be in [0, 1]")
        if self.terrain_coarse_below_m < 0.0:
            raise ValueError("terrain_coarse_below_m must not be negative")
        if self.terrain_coarse_above_m < 0.0:
            raise ValueError("terrain_coarse_above_m must not be negative")
        if self.terrain_max_neighbor_jump_m <= 0.0:
            raise ValueError(
                "terrain_max_neighbor_jump_m must be positive"
            )
        if self.cloud_frame_mode not in ("map", "body"):
            raise ValueError("cloud_frame_mode must be 'map' or 'body'")
        if self.map_mode not in ("rolling", "distance", "global"):
            raise ValueError(
                "map_mode must be 'rolling', 'distance', or 'global'"
            )
        if self.map_resolution_m <= 0.0:
            raise ValueError("map_resolution_m must be positive")
        if self.map_keep_behind_m < 0.0 or self.map_keep_ahead_m < 0.0:
            raise ValueError("map keep distances must not be negative")
        if self.map_publish_rate_hz <= 0.0:
            raise ValueError("map_publish_rate_hz must be positive")

    def odom_cb(self, msg: Odometry) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = TimedPose(
            stamp_sec=stamp_sec,
            translation=np.array(
                [position.x, position.y, position.z],
                dtype=np.float64,
            ),
            quaternion_xyzw=np.array(
                [
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ],
                dtype=np.float64,
            ),
        )
        if self.odom_buffer and stamp_sec < self.odom_buffer[-1].stamp_sec:
            self.get_logger().warn("odom timestamp moved backwards; clearing cache")
            self.odom_buffer.clear()
        self.odom_buffer.append(pose)
        while len(self.odom_buffer) > self.odom_cache_size:
            self.odom_buffer.popleft()

    def pose_at(self, stamp_sec: float) -> Optional[TimedPose]:
        if not self.odom_buffer:
            return None
        first = self.odom_buffer[0]
        last = self.odom_buffer[-1]
        if stamp_sec <= first.stamp_sec:
            if first.stamp_sec - stamp_sec <= self.max_cloud_odom_dt_sec:
                return first
            return None
        if stamp_sec >= last.stamp_sec:
            if stamp_sec - last.stamp_sec <= self.max_cloud_odom_dt_sec:
                return last
            return None

        previous = first
        for following in list(self.odom_buffer)[1:]:
            if following.stamp_sec >= stamp_sec:
                interval = following.stamp_sec - previous.stamp_sec
                if interval <= 1e-9:
                    return following
                ratio = (stamp_sec - previous.stamp_sec) / interval
                translation = (
                    (1.0 - ratio) * previous.translation
                    + ratio * following.translation
                )
                quaternion = quaternion_slerp(
                    previous.quaternion_xyzw,
                    following.quaternion_xyzw,
                    ratio,
                )
                return TimedPose(stamp_sec, translation, quaternion)
            previous = following
        return None

    def work_zone_mask(self, points_body: np.ndarray) -> np.ndarray:
        lateral = points_body[:, self.lateral_axis]
        forward = (
            self.forward_sign * points_body[:, self.forward_axis]
        )
        return (
            (lateral >= self.lateral_min_m)
            & (lateral <= self.lateral_max_m)
            & (forward >= self.forward_min_m)
            & (forward <= self.forward_max_m)
        )

    def cloud_cb(self, msg: PointCloud2) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        pose = self.pose_at(stamp_sec)
        if pose is None:
            self.get_logger().warn(
                "waiting for odom close to registered cloud timestamp",
                throttle_duration_sec=2.0,
            )
            return

        expected_frame = (
            self.map_frame
            if self.cloud_frame_mode == "map"
            else self.body_frame
        )
        received_frame = str(msg.header.frame_id).lstrip("/")
        if (
            received_frame
            and received_frame != str(expected_frame).lstrip("/")
        ):
            self.get_logger().warn(
                f"input cloud frame is '{msg.header.frame_id}', expected "
                f"'{expected_frame}' for cloud_frame_mode="
                f"'{self.cloud_frame_mode}'",
                throttle_duration_sec=2.0,
            )
            return

        points_input = read_xyz_points(msg)
        if points_input.shape[0] == 0:
            self.get_logger().warn(
                "registered input cloud is empty or has no XYZ fields",
                throttle_duration_sec=2.0,
            )
            return

        if points_input.shape[0] > self.max_points_per_scan:
            step = int(
                math.ceil(points_input.shape[0] / self.max_points_per_scan)
            )
            points_input = points_input[::step]

        rotation = quaternion_to_rotation(pose.quaternion_xyzw)
        if self.cloud_frame_mode == "map":
            # /cloud_registered is already deskewed and expressed in
            # camera_init. Odom is only needed to describe the work strip in
            # the current vehicle/body frame; do not transform the cloud into
            # the map a second time.
            points_world = points_input
            points_body = inverse_transform_points(
                points_world,
                rotation,
                pose.translation,
            )
        else:
            # Optional compatibility mode for a deskewed body-frame topic.
            points_body = points_input
            points_world = transform_points(
                points_body,
                rotation,
                pose.translation,
            )

        points_world = points_world[self.work_zone_mask(points_body)]
        points_world = voxel_downsample(
            points_world,
            self.input_voxel_size_m,
        )
        if points_world.shape[0] < 3:
            return

        self.scans.append(StoredScan(stamp_sec, points_world))
        self.latest_pose = pose
        self.latest_cloud_stamp = msg.header.stamp
        self.latest_cloud_stamp_sec = stamp_sec
        self.prune_scans(stamp_sec)

    def prune_scans(self, reference_sec: float) -> None:
        oldest = reference_sec - self.window_sec
        while self.scans and self.scans[0].stamp_sec < oldest:
            self.scans.popleft()

    def fit_ground_plane(self, points_body: np.ndarray) -> Optional[PlaneFit]:
        if points_body.shape[0] < self.min_plane_inliers:
            return None

        if points_body.shape[0] > self.ransac_sample_points:
            selection = self.random.choice(
                points_body.shape[0],
                self.ransac_sample_points,
                replace=False,
            )
            candidates = points_body[selection]
        else:
            candidates = points_body

        min_vertical_component = math.cos(
            math.radians(self.max_normal_angle_deg)
        )
        best_count = 0
        best_normal = None
        best_d = None
        best_mask = None

        for _ in range(self.ransac_iterations):
            indices = self.random.choice(candidates.shape[0], 3, replace=False)
            p0, p1, p2 = candidates[indices]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = float(np.linalg.norm(normal))
            if norm < 1e-8:
                continue
            normal /= norm
            if abs(normal[self.vertical_axis]) < min_vertical_component:
                continue
            d = -float(np.dot(normal, p0))
            if d < 0.0:
                normal = -normal
                d = -d
            if (
                self.ground_normal_sign != 0.0
                and normal[self.vertical_axis] * self.ground_normal_sign
                < min_vertical_component
            ):
                continue
            if not (
                self.min_sensor_height_m
                <= d
                <= self.max_sensor_height_m
            ):
                continue
            distances = np.abs(candidates @ normal + d)
            inliers = distances <= self.ransac_distance_m
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_normal = normal.copy()
                best_d = d
                best_mask = inliers

        if best_normal is None or best_mask is None:
            return None
        ratio = float(best_count / candidates.shape[0])
        if (
            best_count < self.min_plane_inliers
            or ratio < self.min_plane_inlier_ratio
        ):
            return None

        inlier_points = candidates[best_mask]
        centroid = np.mean(inlier_points, axis=0)
        centered = inlier_points - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        normal = vh[-1]
        normal /= max(float(np.linalg.norm(normal)), 1e-12)
        if abs(normal[self.vertical_axis]) < min_vertical_component:
            return None
        d = -float(np.dot(normal, centroid))
        if d < 0.0:
            normal = -normal
            d = -d
        if (
            self.ground_normal_sign != 0.0
            and normal[self.vertical_axis] * self.ground_normal_sign
            < min_vertical_component
        ):
            return None
        if not (
            self.min_sensor_height_m
            <= d
            <= self.max_sensor_height_m
        ):
            return None

        residuals = inlier_points @ normal + d
        rmse = float(math.sqrt(np.mean(residuals * residuals)))
        if rmse > self.max_plane_rmse_m:
            return None

        return PlaneFit(normal, d, rmse, ratio, best_count)

    def update_world_plane(
        self,
        fit: PlaneFit,
        pose: TimedPose,
    ) -> bool:
        rotation = quaternion_to_rotation(pose.quaternion_xyzw)
        new_normal = rotation @ fit.normal_body
        new_normal /= max(float(np.linalg.norm(new_normal)), 1e-12)
        new_d = fit.d_body - float(np.dot(new_normal, pose.translation))
        new_height = new_d + float(np.dot(new_normal, pose.translation))
        if new_height < 0.0:
            new_normal = -new_normal
            new_d = -new_d
            new_height = -new_height

        if self.plane_normal_world is not None:
            old_normal = self.plane_normal_world
            old_d = float(self.plane_d_world)
            if float(np.dot(old_normal, new_normal)) < 0.0:
                new_normal = -new_normal
                new_d = -new_d
            new_height = abs(
                new_d + float(np.dot(new_normal, pose.translation))
            )
            old_height = abs(
                old_d + float(np.dot(old_normal, pose.translation))
            )
            height_jump = abs(new_height - old_height)
            plane_age = max(
                0.0,
                pose.stamp_sec - float(self.last_plane_update_sec),
            )
            if height_jump > self.max_height_jump_m:
                if plane_age <= self.plane_timeout_sec:
                    self.get_logger().warn(
                        "rejected ground plane height jump: "
                        f"old={old_height:.3f}m new={new_height:.3f}m "
                        f"delta={height_jump:.3f}m age={plane_age:.2f}s",
                        throttle_duration_sec=2.0,
                    )
                    return False

                # FAST-LIVO can change its initialized pose after startup. A
                # permanent jump guard would then lock the mapper to the first
                # provisional plane forever. Once the old plane has timed out,
                # accept a new high-quality fit as a clean re-acquisition.
                self.get_logger().warn(
                    "re-acquired ground plane after timeout: "
                    f"old={old_height:.3f}m new={new_height:.3f}m "
                    f"delta={height_jump:.3f}m",
                    throttle_duration_sec=2.0,
                )
                self.plane_normal_world = new_normal
                self.plane_d_world = new_d
            else:
                alpha = float(np.clip(self.plane_smoothing_alpha, 0.0, 1.0))
                mixed_normal = (
                    (1.0 - alpha) * old_normal + alpha * new_normal
                )
                mixed_d = (1.0 - alpha) * old_d + alpha * new_d
                norm = max(float(np.linalg.norm(mixed_normal)), 1e-12)
                self.plane_normal_world = mixed_normal / norm
                self.plane_d_world = mixed_d / norm
        else:
            self.plane_normal_world = new_normal
            self.plane_d_world = new_d

        self.last_plane_update_sec = pose.stamp_sec
        self.last_fit_rmse = fit.rmse
        self.last_fit_ratio = fit.inlier_ratio
        return True

    def plane_in_body(
        self,
        pose: TimedPose,
    ) -> Optional[Tuple[np.ndarray, float]]:
        if self.plane_normal_world is None or self.plane_d_world is None:
            return None
        rotation = quaternion_to_rotation(pose.quaternion_xyzw)
        normal_body = rotation.T @ self.plane_normal_world
        d_body = float(
            self.plane_d_world
            + np.dot(self.plane_normal_world, pose.translation)
        )
        if d_body < 0.0:
            normal_body = -normal_body
            d_body = -d_body
        return normal_body, d_body

    def terrain_cell_indices(
        self,
        points_body: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
        resolution = self.grid_resolution_m
        lateral_count = int(
            math.ceil(
                (self.lateral_max_m - self.lateral_min_m) / resolution
            )
        )
        forward_count = int(
            math.ceil(
                (self.forward_max_m - self.forward_min_m) / resolution
            )
        )
        lateral = points_body[:, self.lateral_axis]
        forward = self.forward_sign * points_body[:, self.forward_axis]
        lateral_index = np.floor(
            (lateral - self.lateral_min_m) / resolution
        ).astype(np.int64)
        forward_index = np.floor(
            (forward - self.forward_min_m) / resolution
        ).astype(np.int64)
        valid = (
            (lateral_index >= 0)
            & (lateral_index < lateral_count)
            & (forward_index >= 0)
            & (forward_index < forward_count)
        )
        return (
            lateral_index,
            forward_index,
            valid,
            lateral_count,
            forward_count,
        )

    @staticmethod
    def shifted_slices(length: int, offset: int) -> Tuple[slice, slice]:
        if offset >= 0:
            return slice(0, length - offset), slice(offset, length)
        return slice(-offset, length), slice(0, length + offset)

    def build_terrain_grid(
        self,
        points_body: np.ndarray,
        normal_body: np.ndarray,
        d_body: float,
    ) -> TerrainGrid:
        (
            lateral_index,
            forward_index,
            inside_roi,
            lateral_count,
            forward_count,
        ) = self.terrain_cell_indices(points_body)
        empty_heights = np.full(
            (lateral_count, forward_count),
            np.nan,
            dtype=np.float64,
        )
        empty_surface = np.empty((0, 3), dtype=np.float64)
        if lateral_count <= 0 or forward_count <= 0:
            return TerrainGrid(
                empty_heights,
                np.zeros_like(empty_heights, dtype=bool),
                empty_surface,
                lateral_count,
                forward_count,
            )

        coarse_height = points_body @ normal_body + d_body
        candidates = (
            inside_roi
            & (coarse_height >= -self.terrain_coarse_below_m)
            & (coarse_height <= self.terrain_coarse_above_m)
        )
        if not np.any(candidates):
            return TerrainGrid(
                empty_heights,
                np.zeros_like(empty_heights, dtype=bool),
                empty_surface,
                lateral_count,
                forward_count,
            )

        candidate_linear = (
            lateral_index[candidates] * forward_count
            + forward_index[candidates]
        )
        candidate_heights = coarse_height[candidates]
        # Sort by cell first and height second. This lets us evaluate the low
        # quantile for every cell in one vectorized operation instead of
        # calling np.quantile thousands of times on Jetson.
        order = np.lexsort((candidate_heights, candidate_linear))
        sorted_linear = candidate_linear[order]
        sorted_heights = candidate_heights[order]
        unique_cells, starts, counts = np.unique(
            sorted_linear,
            return_index=True,
            return_counts=True,
        )

        observed_heights = empty_heights.copy()
        enough_points = counts >= self.terrain_min_points_per_cell
        selected_cells = unique_cells[enough_points]
        selected_starts = starts[enough_points]
        selected_counts = counts[enough_points]
        quantile_position = (
            self.terrain_height_quantile * (selected_counts - 1)
        )
        lower_offset = np.floor(quantile_position).astype(np.int64)
        upper_offset = np.ceil(quantile_position).astype(np.int64)
        fraction = quantile_position - lower_offset
        lower_height = sorted_heights[selected_starts + lower_offset]
        upper_height = sorted_heights[selected_starts + upper_offset]
        selected_height = (
            (1.0 - fraction) * lower_height + fraction * upper_height
        )
        rows = selected_cells // forward_count
        columns = selected_cells % forward_count
        observed_heights[rows, columns] = selected_height

        observed = np.isfinite(observed_heights)
        if not np.any(observed):
            return TerrainGrid(
                empty_heights,
                observed,
                empty_surface,
                lateral_count,
                forward_count,
            )

        # Reject isolated vegetation-only cells by comparing their low
        # quantile height with directly adjacent terrain cells.
        neighbor_sum = np.zeros_like(observed_heights)
        neighbor_count = np.zeros_like(observed_heights, dtype=np.int32)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                source_i, target_i = self.shifted_slices(lateral_count, di)
                source_j, target_j = self.shifted_slices(forward_count, dj)
                source_valid = observed[source_i, source_j]
                source_height = observed_heights[source_i, source_j]
                neighbor_sum[target_i, target_j] += np.where(
                    source_valid,
                    source_height,
                    0.0,
                )
                neighbor_count[target_i, target_j] += source_valid.astype(
                    np.int32
                )
        neighbor_mean = np.divide(
            neighbor_sum,
            neighbor_count,
            out=np.zeros_like(neighbor_sum),
            where=neighbor_count > 0,
        )
        isolated_jump = (
            observed
            & (neighbor_count >= self.terrain_min_neighbor_cells)
            & (
                np.abs(observed_heights - neighbor_mean)
                > self.terrain_max_neighbor_jump_m
            )
        )
        observed_heights[isolated_jump] = np.nan
        observed = np.isfinite(observed_heights)

        # Interpolate only a short distance from real terrain observations.
        # Empty regions remain unknown instead of being flattened onto the
        # coarse RANSAC plane.
        fill_sum = np.zeros_like(observed_heights)
        fill_weight = np.zeros_like(observed_heights)
        fill_neighbors = np.zeros_like(observed_heights, dtype=np.int32)
        radius_cells = int(
            math.ceil(self.max_fill_distance_m / self.grid_resolution_m)
        )
        for di in range(-radius_cells, radius_cells + 1):
            for dj in range(-radius_cells, radius_cells + 1):
                distance_cells = math.sqrt(di * di + dj * dj)
                if distance_cells > radius_cells:
                    continue
                source_i, target_i = self.shifted_slices(lateral_count, di)
                source_j, target_j = self.shifted_slices(forward_count, dj)
                source_valid = observed[source_i, source_j]
                source_height = observed_heights[source_i, source_j]
                weight = 1.0 / max(distance_cells, 1.0)
                fill_sum[target_i, target_j] += np.where(
                    source_valid,
                    source_height * weight,
                    0.0,
                )
                fill_weight[target_i, target_j] += (
                    source_valid.astype(np.float64) * weight
                )
                fill_neighbors[target_i, target_j] += source_valid.astype(
                    np.int32
                )

        filled_heights = observed_heights.copy()
        fillable = (
            ~observed
            & (fill_neighbors >= self.terrain_min_fill_neighbors)
            & (fill_weight > 0.0)
        )
        filled_heights[fillable] = (
            fill_sum[fillable] / fill_weight[fillable]
        )
        supported = np.isfinite(filled_heights)

        cells = np.argwhere(supported)
        if cells.shape[0] > self.max_dense_cells:
            step = int(math.ceil(cells.shape[0] / self.max_dense_cells))
            cells = cells[::step]
        if cells.shape[0] == 0:
            return TerrainGrid(
                filled_heights,
                supported,
                empty_surface,
                lateral_count,
                forward_count,
            )

        resolution = self.grid_resolution_m
        lateral_values = (
            self.lateral_min_m + (cells[:, 0] + 0.5) * resolution
        )
        forward_values = (
            self.forward_min_m + (cells[:, 1] + 0.5) * resolution
        )
        terrain_height = filled_heights[cells[:, 0], cells[:, 1]]
        surface = np.zeros((cells.shape[0], 3), dtype=np.float64)
        surface[:, self.lateral_axis] = lateral_values
        surface[:, self.forward_axis] = forward_values / self.forward_sign
        vertical_component = float(normal_body[self.vertical_axis])
        if abs(vertical_component) < 1e-6:
            surface = empty_surface
        else:
            known_term = (
                normal_body[self.lateral_axis] * lateral_values
                + normal_body[self.forward_axis]
                * surface[:, self.forward_axis]
                + d_body
            )
            surface[:, self.vertical_axis] = (
                terrain_height - known_term
            ) / vertical_component

        return TerrainGrid(
            filled_heights,
            supported,
            surface,
            lateral_count,
            forward_count,
        )

    def classify_with_terrain(
        self,
        points_body: np.ndarray,
        terrain: TerrainGrid,
        normal_body: np.ndarray,
        d_body: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        lateral_index, forward_index, inside, _, _ = (
            self.terrain_cell_indices(points_body)
        )
        supported_points = np.zeros(points_body.shape[0], dtype=bool)
        terrain_height = np.zeros(points_body.shape[0], dtype=np.float64)
        inside_indices = np.flatnonzero(inside)
        if inside_indices.size > 0:
            rows = lateral_index[inside_indices]
            columns = forward_index[inside_indices]
            cell_supported = terrain.supported[rows, columns]
            selected = inside_indices[cell_supported]
            supported_points[selected] = True
            terrain_height[selected] = terrain.heights[
                lateral_index[selected],
                forward_index[selected],
            ]

        coarse_height = points_body @ normal_body + d_body
        height_above_terrain = coarse_height - terrain_height
        ground_mask = (
            supported_points
            & (height_above_terrain >= -self.ground_below_margin_m)
            & (height_above_terrain <= self.ground_above_margin_m)
        )
        non_ground_mask = (
            supported_points
            & (height_above_terrain > self.non_ground_min_height_m)
            & (height_above_terrain <= self.max_object_height_m)
        )
        return ground_mask, non_ground_mask

    def update_persistent_ground_map(
        self,
        dense_world: np.ndarray,
        stamp_sec: float,
        pose: TimedPose,
    ) -> None:
        if self.map_mode == "rolling" or dense_world.shape[0] == 0:
            return

        resolution = self.map_resolution_m
        alpha = float(
            np.clip(self.map_height_smoothing_alpha, 0.0, 1.0)
        )
        horizontal = dense_world[
            :,
            [self.lateral_axis, self.forward_axis],
        ]
        keys = np.floor(horizontal / resolution).astype(np.int64)

        for point, key_values in zip(dense_world, keys):
            key = (int(key_values[0]), int(key_values[1]))
            cell = self.ground_map_cells.get(key)
            if cell is None:
                if len(self.ground_map_cells) >= self.map_max_cells:
                    if not self.map_capacity_warning_issued:
                        self.get_logger().error(
                            "persistent ground map reached map_max_cells="
                            f"{self.map_max_cells}; new cells are ignored"
                        )
                        self.map_capacity_warning_issued = True
                    continue
                cell_point = np.asarray(
                    point,
                    dtype=np.float64,
                ).copy()
                cell_point[self.lateral_axis] = (
                    key[0] + 0.5
                ) * resolution
                cell_point[self.forward_axis] = (
                    key[1] + 0.5
                ) * resolution
                self.ground_map_cells[key] = GroundMapCell(
                    point_world=cell_point,
                    observations=1,
                    last_seen_sec=stamp_sec,
                )
                continue

            # The horizontal position remains tied to the map cell centre;
            # smooth only the fitted vertical height to avoid map jitter.
            cell.point_world[self.vertical_axis] = (
                (1.0 - alpha) * cell.point_world[self.vertical_axis]
                + alpha * float(point[self.vertical_axis])
            )
            cell.observations += 1
            cell.last_seen_sec = stamp_sec

        if self.map_mode == "distance":
            self.prune_distance_ground_map(pose)

    def prune_distance_ground_map(self, pose: TimedPose) -> None:
        if not self.ground_map_cells:
            return
        cell_keys = list(self.ground_map_cells.keys())
        points_world = np.stack(
            [self.ground_map_cells[key].point_world for key in cell_keys],
            axis=0,
        )
        rotation = quaternion_to_rotation(pose.quaternion_xyzw)
        points_body = inverse_transform_points(
            points_world,
            rotation,
            pose.translation,
        )
        lateral = points_body[:, self.lateral_axis]
        forward = self.forward_sign * points_body[:, self.forward_axis]
        keep = (
            (lateral >= self.lateral_min_m)
            & (lateral <= self.lateral_max_m)
            & (forward >= -self.map_keep_behind_m)
            & (forward <= self.map_keep_ahead_m)
        )
        for key, keep_cell in zip(cell_keys, keep):
            if not bool(keep_cell):
                del self.ground_map_cells[key]

    def persistent_ground_points(self) -> np.ndarray:
        if not self.ground_map_cells:
            return np.empty((0, 3), dtype=np.float64)
        return np.stack(
            [
                self.ground_map_cells[key].point_world
                for key in sorted(self.ground_map_cells)
            ],
            axis=0,
        )

    def publish_persistent_ground_map(self, stamp, stamp_sec: float) -> None:
        if self.map_mode == "rolling":
            return
        interval = 1.0 / self.map_publish_rate_hz
        if (
            self.last_global_map_publish_sec is not None
            and stamp_sec - self.last_global_map_publish_sec < interval
        ):
            return
        points_world = self.persistent_ground_points()
        self.global_elevation_pub.publish(
            make_xyz_cloud(points_world, stamp, self.map_frame)
        )
        self.last_global_map_publish_sec = stamp_sec

    def save_ground_map(self) -> Tuple[bool, str]:
        if not self.ground_map_cells:
            return False, "persistent ground map is empty"
        path = os.path.abspath(os.path.expanduser(self.save_map_path))
        directory = os.path.dirname(path) or "."
        try:
            os.makedirs(directory, exist_ok=True)
            rows = []
            for key in sorted(self.ground_map_cells):
                cell = self.ground_map_cells[key]
                rows.append(
                    [
                        float(cell.point_world[0]),
                        float(cell.point_world[1]),
                        float(cell.point_world[2]),
                        float(cell.observations),
                        float(cell.last_seen_sec),
                    ]
                )
            temporary_path = path + ".tmp"
            np.savetxt(
                temporary_path,
                np.asarray(rows, dtype=np.float64),
                delimiter=",",
                header="x,y,z,observations,last_seen_sec",
                comments="",
                fmt=["%.6f", "%.6f", "%.6f", "%.0f", "%.9f"],
            )
            os.replace(temporary_path, path)
        except (OSError, ValueError) as exc:
            return False, f"failed to save map: {exc}"
        return True, f"saved {len(rows)} cells to {path}"

    def save_map_cb(self, request, response):
        del request
        response.success, response.message = self.save_ground_map()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response

    def clear_map_cb(self, request, response):
        del request
        count = len(self.ground_map_cells)
        self.ground_map_cells.clear()
        self.last_global_map_publish_sec = None
        self.map_capacity_warning_issued = False
        response.success = True
        response.message = f"cleared {count} persistent ground cells"
        self.get_logger().info(response.message)
        return response

    def publish_ground_map(self) -> None:
        if (
            self.latest_pose is None
            or self.latest_cloud_stamp_sec is None
            or self.latest_cloud_stamp is None
            or not self.scans
        ):
            self.publish_status(False, "waiting_for_cloud_and_odom", 0, 0, 0)
            return

        self.prune_scans(self.latest_cloud_stamp_sec)
        points_world = np.concatenate(
            [scan.points_world for scan in self.scans],
            axis=0,
        )
        if points_world.shape[0] > self.max_rolling_points:
            step = int(
                math.ceil(points_world.shape[0] / self.max_rolling_points)
            )
            points_world = points_world[::step]

        pose = self.latest_pose
        rotation = quaternion_to_rotation(pose.quaternion_xyzw)
        points_body = inverse_transform_points(
            points_world,
            rotation,
            pose.translation,
        )
        points_body = points_body[self.work_zone_mask(points_body)]
        if points_body.shape[0] < self.min_plane_inliers:
            self.publish_status(
                False,
                "rolling_cloud_too_small",
                points_body.shape[0],
                0,
                0,
            )
            return

        fit = self.fit_ground_plane(points_body)
        if fit is not None:
            self.update_world_plane(fit, pose)

        if self.last_plane_update_sec is None:
            self.publish_status(
                False,
                "no_valid_ground_plane",
                points_body.shape[0],
                0,
                0,
            )
            return
        plane_age = self.latest_cloud_stamp_sec - self.last_plane_update_sec
        if plane_age > self.plane_timeout_sec:
            self.publish_status(
                False,
                "ground_plane_timeout",
                points_body.shape[0],
                0,
                0,
            )
            return

        plane_body = self.plane_in_body(pose)
        if plane_body is None:
            return
        normal_body, d_body = plane_body
        terrain = self.build_terrain_grid(
            points_body,
            normal_body,
            d_body,
        )
        if terrain.surface_points_body.shape[0] == 0:
            self.publish_status(
                False,
                "terrain_grid_empty",
                points_body.shape[0],
                0,
                0,
            )
            return
        ground_mask, non_ground_mask = self.classify_with_terrain(
            points_body,
            terrain,
            normal_body,
            d_body,
        )
        ground_body = points_body[ground_mask]
        non_ground_body = points_body[non_ground_mask]
        dense_body = terrain.surface_points_body

        ground_world = transform_points(
            ground_body,
            rotation,
            pose.translation,
        )
        non_ground_world = transform_points(
            non_ground_body,
            rotation,
            pose.translation,
        )
        dense_world = transform_points(
            dense_body,
            rotation,
            pose.translation,
        )

        self.update_persistent_ground_map(
            dense_world,
            self.latest_cloud_stamp_sec,
            pose,
        )

        stamp = self.latest_cloud_stamp
        self.ground_pub.publish(
            make_xyz_cloud(ground_world, stamp, self.map_frame)
        )
        if self.publish_non_ground:
            self.non_ground_pub.publish(
                make_xyz_cloud(non_ground_world, stamp, self.map_frame)
            )
        self.elevation_pub.publish(
            make_xyz_cloud(dense_world, stamp, self.map_frame)
        )
        self.publish_persistent_ground_map(
            stamp,
            self.latest_cloud_stamp_sec,
        )
        if self.publish_markers:
            self.publish_elevation_markers(dense_world, stamp)

        plane_message = Float64MultiArray()
        plane_message.data = [
            float(self.plane_normal_world[0]),
            float(self.plane_normal_world[1]),
            float(self.plane_normal_world[2]),
            float(self.plane_d_world),
            float(self.last_fit_rmse),
            float(self.last_fit_ratio),
        ]
        self.plane_pub.publish(plane_message)
        height_message = Float64()
        height_message.data = float(d_body)
        self.height_pub.publish(height_message)
        self.publish_status(
            True,
            "ok",
            points_body.shape[0],
            ground_world.shape[0],
            dense_world.shape[0],
            sensor_height=d_body,
            plane_age=plane_age,
        )

        if self.log_debug:
            self.get_logger().info(
                "[GROUND_MAP] "
                f"scans={len(self.scans)} roi={points_body.shape[0]} "
                f"ground={ground_world.shape[0]} "
                f"objects={non_ground_world.shape[0]} "
                f"dense={dense_world.shape[0]} "
                f"map_cells={len(self.ground_map_cells)} "
                f"height={d_body:.3f}m rmse={self.last_fit_rmse:.3f}m "
                f"ratio={self.last_fit_ratio:.2f}",
                throttle_duration_sec=1.0,
            )

    def publish_elevation_markers(self, points_world, stamp) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = "adaptive_ground"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.grid_resolution_m * 0.7
        marker.scale.y = self.grid_resolution_m * 0.7
        marker.scale.z = self.grid_resolution_m * 0.35
        marker.color.r = 0.10
        marker.color.g = 0.90
        marker.color.b = 0.15
        marker.color.a = 0.85
        marker.lifetime.sec = 1

        if points_world.shape[0] > self.marker_max_points:
            step = int(
                math.ceil(points_world.shape[0] / self.marker_max_points)
            )
            points_world = points_world[::step]
        for x, y, z in points_world:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        message = MarkerArray()
        message.markers.append(marker)
        self.marker_pub.publish(message)

    def publish_status(
        self,
        valid: bool,
        reason: str,
        roi_points: int,
        ground_points: int,
        dense_points: int,
        sensor_height: float = math.nan,
        plane_age: float = math.nan,
    ) -> None:
        message = String()
        message.data = (
            f"valid={'true' if valid else 'false'} "
            f"reason={reason} scans={len(self.scans)} "
            f"roi_points={roi_points} ground_points={ground_points} "
            f"dense_points={dense_points} "
            f"map_mode={self.map_mode} "
            f"map_cells={len(self.ground_map_cells)} "
            f"sensor_height_m={sensor_height:.3f} "
            f"plane_age_sec={plane_age:.3f}"
        )
        self.status_pub.publish(message)

    def destroy_node(self):
        if self.save_map_on_shutdown and self.ground_map_cells:
            success, message = self.save_ground_map()
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundMapper()
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
