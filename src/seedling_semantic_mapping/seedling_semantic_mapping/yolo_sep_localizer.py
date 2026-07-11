#!/usr/bin/env python3
"""YOLO SEP pixel -> 3D map observation node.

Inputs:
  - color image from camera node, preferably a 10 Hz rgb8 image
  - deskewed cloud from FAST-LIVO2, preferably in body/IMU frame
  - FAST-LIVO2 odometry T_map_body

Output:
  - geometry_msgs/PointStamped observations in map frame
  - RViz markers for current observations

The node intentionally does not create persistent seedling IDs.  Persistent IDs
are handled by seedling_mapper.py.
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class CloudCacheItem:
    stamp_sec: float
    msg: PointCloud2


@dataclass
class OdomCacheItem:
    stamp_sec: float
    msg: Odometry


@dataclass
class SepDetection:
    u: float
    v: float
    conf: float
    bbox: Optional[Tuple[float, float, float, float]] = None


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rot_xyzw(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion xyzw -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """Convert common ROS Image encodings to numpy.

    Supports mono8, rgb8, bgr8, bgra8, rgba8.  The returned array references a
    copy of msg.data, so it is safe after callback returns.
    """
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8).copy()

    if enc in ("mono8", "8uc1"):
        return data.reshape((h, msg.step))[:, :w]

    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }.get(enc)

    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    row_pixels = int(msg.step) // channels
    img = data.reshape((h, row_pixels, channels))[:, :w, :]

    if enc == "rgb8":
        return img
    if enc == "bgr8":
        return img[:, :, ::-1]
    if enc == "rgba8":
        return img[:, :, :3]
    if enc == "bgra8":
        return img[:, :, [2, 1, 0]]
    return img


class YoloSepLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("yolo_sep_localizer")

        # Topics
        self.declare_parameter("image_topic", "/left_camera/image_10hz")
        self.declare_parameter("cloud_topic", "/fastlivo/deskew/cloud_body")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("observation_topic", "/seedling/observation_point_map")
        self.declare_parameter("marker_topic", "/seedling/current_observation_markers")

        # YOLO / SEP detection
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "")  # e.g. "cpu", "0"
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("det_conf", 0.35)
        self.declare_parameter("kp_conf", 0.30)
        self.declare_parameter("keypoint_index", 0)
        self.declare_parameter("class_id", -1)  # -1 means all classes
        self.declare_parameter("max_detections", 50)
        self.declare_parameter("nms_sep_px", 12.0)

        # Synchronization. The image is the trigger; the node finds nearest cloud and odom by header.stamp.
        self.declare_parameter("max_image_cloud_dt", 0.08)
        self.declare_parameter("max_image_odom_dt", 0.08)
        # Backward compatibility with the older config key.
        self.declare_parameter("max_sync_dt", 0.08)
        self.declare_parameter("cache_size", 50)

        # Camera intrinsics. Use values at original calibration resolution, then
        # multiply by intrinsic_scale if FAST-LIVO/image was resized.
        self.declare_parameter("cam_fx", 1293.56944)
        self.declare_parameter("cam_fy", 1293.3155)
        self.declare_parameter("cam_cx", 626.91359)
        self.declare_parameter("cam_cy", 522.799224)

        # ROS CameraInfo plumb_bob distortion:
        # D = [k1, k2, p1, p2, k3]
        self.declare_parameter("use_distortion", True)
        self.declare_parameter("dist_k1", -0.554065)
        self.declare_parameter("dist_k2", 0.220066)
        self.declare_parameter("dist_p1", 0.001350)
        self.declare_parameter("dist_p2", 0.001332)
        self.declare_parameter("dist_k3", 0.0)
        self.declare_parameter("intrinsic_scale", 1.0)

        # FAST-Calib extrinsic: P_cam = Rcl * P_lidar + Pcl
        self.declare_parameter("Rcl", [-0.011051, -0.999830, 0.014773,
                                       0.522598, -0.018371, -0.852382,
                                       0.852508, -0.001699, 0.522712])
        self.declare_parameter("Pcl", [-0.012874, -0.114651, 0.012899])

        # Input cloud frame. Use "body" for /fastlivo/deskew/cloud_body, or "lidar" for a lidar-frame cloud.
        self.declare_parameter("cloud_frame", "body")

        # LiDAR -> body/IMU extrinsic: P_body = extR * P_lidar + extT
        self.declare_parameter("extR", [1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0])
        self.declare_parameter("extT", [-0.011, -0.02329, 0.04412])

        # 2D->3D localization
        self.declare_parameter("search_radius_px", 12.0)
        self.declare_parameter("min_candidate_points", 12)
        self.declare_parameter("cloud_stride", 1)
        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("use_plane_intersection", True)
        self.declare_parameter("max_plane_rmse", 0.05)
        # 只使用离检测像素最近的一部分投影点，避免混入其他表面。
        self.declare_parameter("max_candidate_points", 40)

        self.image_topic = self.get_parameter("image_topic").value
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.observation_topic = self.get_parameter("observation_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value

        legacy_max_sync_dt = float(self.get_parameter("max_sync_dt").value)
        self.max_image_cloud_dt = float(self.get_parameter("max_image_cloud_dt").value)
        self.max_image_odom_dt = float(self.get_parameter("max_image_odom_dt").value)
        # If the new keys are accidentally set to non-positive values, fall back to max_sync_dt.
        if self.max_image_cloud_dt <= 0.0:
            self.max_image_cloud_dt = legacy_max_sync_dt
        if self.max_image_odom_dt <= 0.0:
            self.max_image_odom_dt = legacy_max_sync_dt
        self.cache_size = int(self.get_parameter("cache_size").value)

        scale = float(self.get_parameter("intrinsic_scale").value)
        self.fx = float(self.get_parameter("cam_fx").value) * scale
        self.fy = float(self.get_parameter("cam_fy").value) * scale
        self.cx = float(self.get_parameter("cam_cx").value) * scale
        self.cy = float(self.get_parameter("cam_cy").value) * scale

        self.use_distortion = bool(
            self.get_parameter("use_distortion").value
        )
        self.dist_k1 = float(self.get_parameter("dist_k1").value)
        self.dist_k2 = float(self.get_parameter("dist_k2").value)
        self.dist_p1 = float(self.get_parameter("dist_p1").value)
        self.dist_p2 = float(self.get_parameter("dist_p2").value)
        self.dist_k3 = float(self.get_parameter("dist_k3").value)

        self.Rcl = np.array(self.get_parameter("Rcl").value, dtype=np.float64).reshape(3, 3)
        self.Pcl = np.array(self.get_parameter("Pcl").value, dtype=np.float64).reshape(3)
        self.cloud_frame = str(self.get_parameter("cloud_frame").value).lower().strip()
        if self.cloud_frame not in ("body", "lidar"):
            self.get_logger().warn(f"Unsupported cloud_frame={self.cloud_frame}; fallback to body")
            self.cloud_frame = "body"
        self.extR = np.array(self.get_parameter("extR").value, dtype=np.float64).reshape(3, 3)
        self.extT = np.array(self.get_parameter("extT").value, dtype=np.float64).reshape(3)

        self.search_radius_px = float(self.get_parameter("search_radius_px").value)
        self.min_candidate_points = int(self.get_parameter("min_candidate_points").value)
        self.cloud_stride = max(1, int(self.get_parameter("cloud_stride").value))
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.use_plane_intersection = bool(self.get_parameter("use_plane_intersection").value)
        self.max_plane_rmse = float(self.get_parameter("max_plane_rmse").value)
        self.max_candidate_points = max(
            self.min_candidate_points,
            int(self.get_parameter("max_candidate_points").value),
        )

        self.det_conf = float(self.get_parameter("det_conf").value)
        self.kp_conf = float(self.get_parameter("kp_conf").value)
        self.keypoint_index = int(self.get_parameter("keypoint_index").value)
        self.class_id = int(self.get_parameter("class_id").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.nms_sep_px = float(self.get_parameter("nms_sep_px").value)

        self.cloud_cache: Deque[CloudCacheItem] = deque(maxlen=self.cache_size)
        self.odom_cache: Deque[OdomCacheItem] = deque(maxlen=self.cache_size)

        # 图像处理较重，必须允许点云和里程计回调并行更新缓存。
        self.cache_lock = threading.Lock()
        self.image_cb_group = MutuallyExclusiveCallbackGroup()
        self.cloud_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        self.model = None
        model_path = str(self.get_parameter("model_path").value)
        if model_path:
            try:
                from ultralytics import YOLO  # type: ignore

                self.model = YOLO(model_path)
                self.get_logger().info(f"Loaded YOLO model: {model_path}")
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to load YOLO model '{model_path}': {exc}. "
                    "Node will run but publish no detections."
                )
        else:
            self.get_logger().warn("model_path is empty; yolo_sep_localizer will publish no detections.")

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_cb,
            qos_sensor,
            callback_group=self.cloud_cb_group,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            qos_pub,
            callback_group=self.odom_cb_group,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_sensor,
            callback_group=self.image_cb_group,
        )

        self.obs_pub = self.create_publisher(PointStamped, self.observation_topic, qos_pub)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos_pub)

        self.frame_count = 0
        self.obs_count = 0
        self.get_logger().info(
            "YoloSepLocalizer started. image=%s cloud=%s cloud_frame=%s odom=%s obs=%s"
            % (self.image_topic, self.cloud_topic, self.cloud_frame, self.odom_topic, self.observation_topic)
        )

    def cloud_cb(self, msg: PointCloud2) -> None:
        item = CloudCacheItem(stamp_to_sec(msg.header.stamp), msg)
        with self.cache_lock:
            self.cloud_cache.append(item)

    def odom_cb(self, msg: Odometry) -> None:
        item = OdomCacheItem(stamp_to_sec(msg.header.stamp), msg)
        with self.cache_lock:
            self.odom_cache.append(item)

    def nearest_cloud(self, t: float) -> Tuple[Optional[PointCloud2], Optional[float]]:
        with self.cache_lock:
            cache = list(self.cloud_cache)

        if not cache:
            return None, None

        item = min(cache, key=lambda c: abs(c.stamp_sec - t))
        dt = abs(item.stamp_sec - t)
        return (item.msg, dt) if dt <= self.max_image_cloud_dt else (None, dt)

    def nearest_odom(self, t: float) -> Tuple[Optional[Odometry], Optional[float]]:
        with self.cache_lock:
            cache = list(self.odom_cache)

        if not cache:
            return None, None

        item = min(cache, key=lambda o: abs(o.stamp_sec - t))
        dt = abs(item.stamp_sec - t)
        return (item.msg, dt) if dt <= self.max_image_odom_dt else (None, dt)

    def run_yolo(self, img_rgb: np.ndarray) -> List[SepDetection]:
        if self.model is None:
            return []

        kwargs = {
            "conf": self.det_conf,
            "imgsz": int(self.get_parameter("imgsz").value),
            "verbose": False,
        }
        device = str(self.get_parameter("device").value)
        if device:
            kwargs["device"] = device

        results = self.model.predict(img_rgb, **kwargs)
        if not results:
            return []

        res = results[0]
        detections: List[SepDetection] = []

        if getattr(res, "keypoints", None) is None or res.keypoints is None:
            self.get_logger().warn("YOLO result has no keypoints; expected pose model.", throttle_duration_sec=2.0)
            return []

        kps_xy = res.keypoints.xy.cpu().numpy() if res.keypoints.xy is not None else None
        kps_conf = res.keypoints.conf.cpu().numpy() if res.keypoints.conf is not None else None
        boxes_xyxy = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else None
        boxes_cls = res.boxes.cls.cpu().numpy().astype(int) if res.boxes is not None and res.boxes.cls is not None else None
        boxes_conf = res.boxes.conf.cpu().numpy() if res.boxes is not None and res.boxes.conf is not None else None

        if kps_xy is None:
            return []

        for i in range(min(len(kps_xy), self.max_detections)):
            if self.class_id >= 0 and boxes_cls is not None and int(boxes_cls[i]) != self.class_id:
                continue
            if boxes_conf is not None and float(boxes_conf[i]) < self.det_conf:
                continue
            if self.keypoint_index >= kps_xy.shape[1]:
                continue

            u, v = kps_xy[i, self.keypoint_index]
            conf = 1.0
            if kps_conf is not None:
                conf = float(kps_conf[i, self.keypoint_index])
            if conf < self.kp_conf:
                continue
            if not np.isfinite(u) or not np.isfinite(v) or (u <= 0 and v <= 0):
                continue

            bbox = None
            if boxes_xyxy is not None:
                bbox = tuple(float(x) for x in boxes_xyxy[i])
            detections.append(SepDetection(float(u), float(v), conf, bbox))

        return self.sep_nms(detections)

    def sep_nms(self, detections: List[SepDetection]) -> List[SepDetection]:
        if not detections:
            return []
        detections = sorted(detections, key=lambda d: d.conf, reverse=True)
        kept: List[SepDetection] = []
        r2 = self.nms_sep_px * self.nms_sep_px
        for det in detections:
            duplicate = False
            for old in kept:
                if (det.u - old.u) ** 2 + (det.v - old.v) ** 2 < r2:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(det)
        return kept

    def pointcloud_to_xyz(self, cloud_msg: PointCloud2) -> np.ndarray:
        pts = []
        idx = 0
        for p in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            if idx % self.cloud_stride == 0:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
            idx += 1
        if not pts:
            return np.empty((0, 3), dtype=np.float64)
        arr = np.asarray(pts, dtype=np.float64)
        finite = np.isfinite(arr).all(axis=1)
        return arr[finite]

    def cloud_points_to_lidar(self, pts_cloud: np.ndarray) -> np.ndarray:
        """Convert input cloud points to LiDAR frame for camera projection.

        /fastlivo/deskew/cloud_body is in body/IMU frame. Camera projection uses
        the calibrated LiDAR-camera extrinsic P_cam = Rcl * P_lidar + Pcl, so
        body points must be transformed back to LiDAR frame first.
        """
        if pts_cloud.shape[0] == 0:
            return pts_cloud
        if self.cloud_frame == "lidar":
            return pts_cloud
        # P_body = extR * P_lidar + extT
        # P_lidar = extR^T * (P_body - extT)
        return (self.extR.T @ (pts_cloud - self.extT.reshape(1, 3)).T).T

    def project_lidar_points(
        self,
        pts_lidar: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Project LiDAR points onto the raw distorted camera image."""
        # P_cam = Rcl * P_lidar + Pcl
        pts_cam = (
            self.Rcl @ pts_lidar.T
        ).T + self.Pcl.reshape(1, 3)

        z = pts_cam[:, 2]
        valid = (
            np.isfinite(pts_cam).all(axis=1)
            & (z > self.min_depth)
            & (z < self.max_depth)
        )

        safe_z = np.where(np.abs(z) > 1e-9, z, 1.0)
        x = pts_cam[:, 0] / safe_z
        y = pts_cam[:, 1] / safe_z

        if self.use_distortion:
            x2 = x * x
            y2 = y * y
            xy = x * y
            r2 = x2 + y2
            r4 = r2 * r2
            r6 = r4 * r2

            radial = (
                1.0
                + self.dist_k1 * r2
                + self.dist_k2 * r4
                + self.dist_k3 * r6
            )

            x_distorted = (
                x * radial
                + 2.0 * self.dist_p1 * xy
                + self.dist_p2 * (r2 + 2.0 * x2)
            )

            y_distorted = (
                y * radial
                + self.dist_p1 * (r2 + 2.0 * y2)
                + 2.0 * self.dist_p2 * xy
            )
        else:
            x_distorted = x
            y_distorted = y

        u = self.fx * x_distorted + self.cx
        v = self.fy * y_distorted + self.cy

        uv = np.column_stack((u, v))
        valid &= np.isfinite(uv).all(axis=1)
        return uv, valid

    def pixel_to_camera_ray(
        self,
        u: float,
        v: float,
    ) -> np.ndarray:
        """Convert a distorted image pixel to an undistorted camera ray."""
        xd = (float(u) - self.cx) / self.fx
        yd = (float(v) - self.cy) / self.fy

        if not self.use_distortion:
            ray = np.array([xd, yd, 1.0], dtype=np.float64)
            return ray / np.linalg.norm(ray)

        # Fixed-point inverse of the plumb_bob distortion model.
        x = xd
        y = yd

        for _ in range(10):
            x2 = x * x
            y2 = y * y
            xy = x * y
            r2 = x2 + y2
            r4 = r2 * r2
            r6 = r4 * r2

            radial = (
                1.0
                + self.dist_k1 * r2
                + self.dist_k2 * r4
                + self.dist_k3 * r6
            )

            if abs(radial) < 1e-9:
                break

            delta_x = (
                2.0 * self.dist_p1 * xy
                + self.dist_p2 * (r2 + 2.0 * x2)
            )

            delta_y = (
                self.dist_p1 * (r2 + 2.0 * y2)
                + 2.0 * self.dist_p2 * xy
            )

            x = (xd - delta_x) / radial
            y = (yd - delta_y) / radial

        ray = np.array([x, y, 1.0], dtype=np.float64)
        return ray / np.linalg.norm(ray)

    def localize_sep_in_lidar(self, det: SepDetection, pts_lidar: np.ndarray, uv: np.ndarray, valid: np.ndarray) -> Optional[np.ndarray]:
        du = uv[:, 0] - det.u
        dv = uv[:, 1] - det.v
        dist2 = du * du + dv * dv

        candidate_indices = np.flatnonzero(
            valid & (dist2 <= self.search_radius_px * self.search_radius_px)
        )
        if candidate_indices.size < self.min_candidate_points:
            return None

        # 搜索圆内可能同时包含球、地面和背景。
        # 只保留像素距离检测点最近的若干投影点，防止三维结果在不同表面间跳变。
        if candidate_indices.size > self.max_candidate_points:
            order = np.argsort(dist2[candidate_indices])
            candidate_indices = candidate_indices[order[:self.max_candidate_points]]

        candidates = pts_lidar[candidate_indices]

        if not self.use_plane_intersection:
            return np.median(candidates, axis=0)

        centroid = candidates.mean(axis=0)
        demean = candidates - centroid
        try:
            _, svals, vh = np.linalg.svd(demean, full_matrices=False)
        except np.linalg.LinAlgError:
            return np.median(candidates, axis=0)
        normal = vh[-1]
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-9:
            return np.median(candidates, axis=0)
        normal = normal / normal_norm

        # Plane RMSE filter.
        dists = demean @ normal
        rmse = float(np.sqrt(np.mean(dists * dists)))
        if rmse > self.max_plane_rmse:
            return np.median(candidates, axis=0)

        # Camera ray in lidar frame.
        ray_cam = self.pixel_to_camera_ray(det.u, det.v)
        cam_center_lidar = -self.Rcl.T @ self.Pcl
        ray_lidar = self.Rcl.T @ ray_cam
        ray_lidar = ray_lidar / np.linalg.norm(ray_lidar)

        denom = float(normal @ ray_lidar)
        if abs(denom) < 1e-6:
            return np.median(candidates, axis=0)
        lam = float(normal @ (centroid - cam_center_lidar) / denom)
        if lam <= 0 or not math.isfinite(lam):
            return np.median(candidates, axis=0)

        p = cam_center_lidar + lam * ray_lidar
        if not np.isfinite(p).all():
            return np.median(candidates, axis=0)
        return p

    def lidar_to_map(self, p_lidar: np.ndarray, odom_msg: Odometry) -> np.ndarray:
        p_body = self.extR @ p_lidar + self.extT
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        R_map_body = quat_to_rot_xyzw(ori.x, ori.y, ori.z, ori.w)
        t_map_body = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        return R_map_body @ p_body + t_map_body

    def publish_observation_marker(self, points_map: Sequence[np.ndarray], stamp: Time) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, p in enumerate(points_map):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "current_seedling_observations"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = float(p[2])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.04
            m.scale.y = 0.04
            m.scale.z = 0.04
            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.9
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def image_cb(self, msg: Image) -> None:
        self.frame_count += 1
        t = stamp_to_sec(msg.header.stamp)
        cloud_msg, cloud_dt = self.nearest_cloud(t)
        odom_msg, odom_dt = self.nearest_odom(t)
        if cloud_msg is None or odom_msg is None:
            cloud_dt_s = "none" if cloud_dt is None else f"{cloud_dt*1000.0:.1f}ms"
            odom_dt_s = "none" if odom_dt is None else f"{odom_dt*1000.0:.1f}ms"
            self.get_logger().warn(
                f"Waiting sync data: cloud={'ok' if cloud_msg else 'missing'} dt={cloud_dt_s}, "
                f"odom={'ok' if odom_msg else 'missing'} dt={odom_dt_s}",
                throttle_duration_sec=2.0,
            )
            return

        try:
            img = image_msg_to_numpy(msg)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        detections = self.run_yolo(img)
        if not detections:
            return

        pts_cloud = self.pointcloud_to_xyz(cloud_msg)
        if pts_cloud.shape[0] == 0:
            return
        pts_lidar = self.cloud_points_to_lidar(pts_cloud)
        uv, valid = self.project_lidar_points(pts_lidar)

        points_map: List[np.ndarray] = []
        for det in detections:
            p_lidar = self.localize_sep_in_lidar(det, pts_lidar, uv, valid)
            if p_lidar is None:
                continue
            p_map = self.lidar_to_map(p_lidar, odom_msg)
            if not np.isfinite(p_map).all():
                continue

            obs = PointStamped()
            obs.header.stamp = msg.header.stamp
            obs.header.frame_id = "map"
            obs.point.x = float(p_map[0])
            obs.point.y = float(p_map[1])
            obs.point.z = float(p_map[2])
            self.obs_pub.publish(obs)
            self.obs_count += 1
            points_map.append(p_map)

        if points_map:
            self.publish_observation_marker(points_map, msg.header.stamp)
            self.get_logger().info(
                f"frame={self.frame_count}, detections={len(detections)}, observations={len(points_map)}, "
                f"dt_cloud={cloud_dt*1000.0:.1f}ms, dt_odom={odom_dt*1000.0:.1f}ms, total_obs={self.obs_count}",
                throttle_duration_sec=1.0,
            )


def main() -> None:
    rclpy.init()
    node = YoloSepLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
