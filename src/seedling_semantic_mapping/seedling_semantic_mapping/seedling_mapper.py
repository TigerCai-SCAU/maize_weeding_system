#!/usr/bin/env python3
"""Fuse repeated 3D SEP observations into a persistent seedling landmark map."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty, SetBool


@dataclass
class Landmark:
    id: int
    x: float
    y: float
    z: float
    hit_count: int = 1
    status: str = "tentative"
    created_sec: float = 0.0
    last_seen_sec: float = 0.0
    last_hit_increment_sec: float = 0.0
    obs_count: int = 1
    covariance_trace_xy: float = 0.0
    # Welford-like running spread in xy for debugging
    sum_sq_xy: float = 0.0



def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_key(stamp: Time) -> Tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


class SeedlingMapper(Node):
    def __init__(self) -> None:
        super().__init__("seedling_mapper")

        self.declare_parameter("observation_topic", "/seedling/observation_point_map")
        self.declare_parameter("marker_topic", "/seedling/map_markers")
        self.declare_parameter("map_points_topic", "/seedling/map_points")
        self.declare_parameter("csv_path", "/tmp/seedling_map_confirmed.csv")
        self.declare_parameter("world_frame", "camera_init")

        self.declare_parameter("gate_xy", 0.10)
        self.declare_parameter("gate_z", 0.20)
        self.declare_parameter("confirm_hits", 3)
        self.declare_parameter("min_update_dt", 0.20)
        self.declare_parameter("position_alpha", 0.25)  # EMA update, 0 means count average
        self.declare_parameter("publish_all_landmarks", True)
        self.declare_parameter("map_points_confirmed_only", True)
        self.declare_parameter("save_every_update", True)
        self.declare_parameter("same_stamp_duplicate_policy", "skip")  # skip or allow
        self.declare_parameter("max_same_stamp_history", 20)
        # 未确认候选点超过该时间未再次观测，就自动删除。
        self.declare_parameter("tentative_timeout_sec", 3.0)
        self.declare_parameter("reset_service", "/seedling/reset_map")
        self.declare_parameter("freeze_service", "/seedling/freeze_map")

        self.observation_topic = self.get_parameter("observation_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.map_points_topic = self.get_parameter("map_points_topic").value
        self.csv_path = Path(str(self.get_parameter("csv_path").value))
        self.world_frame = str(self.get_parameter("world_frame").value).strip()

        self.gate_xy = float(self.get_parameter("gate_xy").value)
        self.gate_z = float(self.get_parameter("gate_z").value)
        self.confirm_hits = int(self.get_parameter("confirm_hits").value)
        self.min_update_dt = float(self.get_parameter("min_update_dt").value)
        self.position_alpha = float(self.get_parameter("position_alpha").value)
        self.publish_all_landmarks = bool(self.get_parameter("publish_all_landmarks").value)
        self.map_points_confirmed_only = bool(self.get_parameter("map_points_confirmed_only").value)
        self.save_every_update = bool(self.get_parameter("save_every_update").value)
        self.same_stamp_duplicate_policy = str(self.get_parameter("same_stamp_duplicate_policy").value)
        self.max_same_stamp_history = int(self.get_parameter("max_same_stamp_history").value)
        self.tentative_timeout_sec = float(
            self.get_parameter("tentative_timeout_sec").value
        )
        self.reset_service = str(self.get_parameter("reset_service").value)
        self.freeze_service = str(self.get_parameter("freeze_service").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(PointStamped, self.observation_topic, self.obs_cb, qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.map_points_pub = self.create_publisher(PoseArray, self.map_points_topic, qos)
        self.reset_server = self.create_service(
            Empty, self.reset_service, self.reset_map_callback
        )
        self.freeze_server = self.create_service(
            SetBool, self.freeze_service, self.freeze_map_callback
        )

        self.landmarks: List[Landmark] = []
        self.next_id = 1
        self.map_frozen = False
        self.used_landmarks_by_stamp: Dict[Tuple[int, int], Set[int]] = {}
        self.stamp_history: List[Tuple[int, int]] = []

        self.get_logger().info(
            f"SeedlingMapper started. obs={self.observation_topic}, map_points={self.map_points_topic}, gate_xy={self.gate_xy:.3f}, csv={self.csv_path}"
        )

        # 周期发布苗点地图，避免 RViz 后启动时收不到 VOLATILE 消息
        self.map_publish_timer = self.create_timer(
            1.0,
            self.publish_map_timer_cb,
        )

    def prune_stamp_history(self) -> None:
        while len(self.stamp_history) > self.max_same_stamp_history:
            old = self.stamp_history.pop(0)
            self.used_landmarks_by_stamp.pop(old, None)

    def prune_stale_tentative(self, now_sec: float) -> None:
        if self.tentative_timeout_sec <= 0.0:
            return

        before = len(self.landmarks)
        self.landmarks = [
            lm
            for lm in self.landmarks
            if not (
                lm.status == "tentative"
                and (now_sec - lm.last_seen_sec) > self.tentative_timeout_sec
            )
        ]

        removed = before - len(self.landmarks)
        if removed > 0:
            self.get_logger().info(
                f"removed {removed} stale tentative landmarks"
            )

    def nearest_landmark(
        self,
        x: float,
        y: float,
        z: float,
        used_ids: Optional[Set[int]],
    ) -> Tuple[Optional[Landmark], float, float]:
        best = None
        best_xy = float("inf")
        best_z = float("inf")
        best_score = float("inf")

        xy_scale = max(self.gate_xy, 1e-6)
        z_scale = max(self.gate_z, 1e-6)

        for lm in self.landmarks:
            if used_ids and lm.id in used_ids:
                continue

            dxy = math.hypot(x - lm.x, y - lm.y)
            dz = abs(z - lm.z)

            # 先执行完整三维门限，不能只按 XY 选最近点后再检查 Z。
            if dxy > self.gate_xy or dz > self.gate_z:
                continue

            score = (dxy / xy_scale) ** 2 + (dz / z_scale) ** 2
            if score < best_score:
                best = lm
                best_xy = dxy
                best_z = dz
                best_score = score

        return best, best_xy, best_z

    def create_landmark(self, x: float, y: float, z: float, t: float) -> Landmark:
        lm = Landmark(
            id=self.next_id,
            x=x,
            y=y,
            z=z,
            hit_count=1,
            status="tentative",
            created_sec=t,
            last_seen_sec=t,
            last_hit_increment_sec=t,
            obs_count=1,
        )
        if lm.hit_count >= self.confirm_hits:
            lm.status = "confirmed"
        self.landmarks.append(lm)
        self.next_id += 1
        self.get_logger().info(f"create landmark id={lm.id} pos=[{x:.3f}, {y:.3f}, {z:.3f}]")
        return lm

    def update_landmark(self, lm: Landmark, x: float, y: float, z: float, t: float) -> None:
        old_x, old_y = lm.x, lm.y
        lm.obs_count += 1

        # Position update.  EMA is more robust to occasional poor projections;
        # count-average can be selected by setting alpha <= 0.
        if self.position_alpha > 0.0:
            a = min(max(self.position_alpha, 0.0), 1.0)
            lm.x = (1.0 - a) * lm.x + a * x
            lm.y = (1.0 - a) * lm.y + a * y
            lm.z = (1.0 - a) * lm.z + a * z
        else:
            n = float(lm.obs_count)
            lm.x = (lm.x * (n - 1.0) + x) / n
            lm.y = (lm.y * (n - 1.0) + y) / n
            lm.z = (lm.z * (n - 1.0) + z) / n

        dx, dy = x - old_x, y - old_y
        lm.sum_sq_xy += dx * dx + dy * dy
        lm.covariance_trace_xy = lm.sum_sq_xy / max(1, lm.obs_count - 1)
        lm.last_seen_sec = t

        if (t - lm.last_hit_increment_sec) >= self.min_update_dt:
            lm.hit_count += 1
            lm.last_hit_increment_sec = t

        if lm.hit_count >= self.confirm_hits:
            lm.status = "confirmed"

    def obs_cb(self, msg: PointStamped) -> None:
        if self.map_frozen:
            return
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().error(
                f"Observation frame_id is '{msg.header.frame_id}', expected "
                f"'{self.world_frame}'. Observation ignored.",
                throttle_duration_sec=2.0,
            )
            return

        x, y, z = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        if not all(math.isfinite(v) for v in (x, y, z)):
            return
        t = stamp_to_sec(msg.header.stamp)
        self.prune_stale_tentative(t)
        key = stamp_key(msg.header.stamp)

        if key not in self.used_landmarks_by_stamp:
            self.used_landmarks_by_stamp[key] = set()
            self.stamp_history.append(key)
            self.prune_stamp_history()
        used_ids = self.used_landmarks_by_stamp[key]

        lm, dxy, dz = self.nearest_landmark(x, y, z, used_ids if self.same_stamp_duplicate_policy == "skip" else None)
        if lm is not None and dxy <= self.gate_xy and dz <= self.gate_z:
            self.update_landmark(lm, x, y, z, t)
            used_ids.add(lm.id)
            self.get_logger().info(
                f"update id={lm.id} hit={lm.hit_count} status={lm.status} dxy={dxy:.3f} dz={dz:.3f}",
                throttle_duration_sec=1.0,
            )
        else:
            new_lm = self.create_landmark(x, y, z, t)
            used_ids.add(new_lm.id)

        self.publish_markers(msg.header.stamp)
        self.publish_map_points(msg.header.stamp)
        if self.save_every_update:
            self.save_csv()

    def reset_map_callback(self, _request, response):
        self.landmarks.clear()
        self.used_landmarks_by_stamp.clear()
        self.stamp_history.clear()
        self.next_id = 1
        stamp = self.get_clock().now().to_msg()
        self.publish_markers(stamp)
        self.publish_map_points(stamp)
        self.save_csv()
        self.get_logger().info("seedling map reset")
        return response

    def freeze_map_callback(self, request, response):
        self.map_frozen = bool(request.data)
        response.success = True
        response.message = (
            "seedling map frozen" if self.map_frozen else "seedling map resumed"
        )
        self.get_logger().info(response.message)
        return response

    def publish_map_timer_cb(self) -> None:
        """周期发布当前地图。候选点只按传感器时间清理。"""
        now = self.get_clock().now()

        if not self.landmarks:
            return

        stamp = now.to_msg()
        self.publish_markers(stamp)
        self.publish_map_points(stamp)

    def publish_markers(self, stamp: Time) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        marker_id = 0
        for lm in self.landmarks:
            if not self.publish_all_landmarks and lm.status != "confirmed":
                continue

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = "seedling_landmarks"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = lm.x
            m.pose.position.y = lm.y
            m.pose.position.z = lm.z
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06 if lm.status == "confirmed" else 0.04
            m.scale.y = 0.06 if lm.status == "confirmed" else 0.04
            m.scale.z = 0.06 if lm.status == "confirmed" else 0.04
            if lm.status == "confirmed":
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 0.9
            else:
                m.color.r = 1.0
                m.color.g = 0.8
                m.color.b = 0.0
                m.color.a = 0.75
            ma.markers.append(m)

            text = Marker()
            text.header.frame_id = self.world_frame
            text.header.stamp = stamp
            text.ns = "seedling_landmark_ids"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = lm.x
            text.pose.position.y = lm.y
            text.pose.position.z = lm.z + 0.10
            text.pose.orientation.w = 1.0
            text.scale.z = 0.08
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.9
            text.text = f"{lm.id}:{lm.hit_count}"
            ma.markers.append(text)

        self.marker_pub.publish(ma)


    def publish_map_points(self, stamp: Time) -> None:
        """Publish fused seedling landmarks as a PoseArray for online path planning.

        PoseArray intentionally contains only positions.  The landmark id is not
        represented by PoseArray, so planners that need ids can use the order in
        this array together with the CSV/logs, or subscribe to the MarkerArray for
        visualization.  For online motion planning, confirmed positions are the
        important part.
        """
        pa = PoseArray()
        pa.header.frame_id = self.world_frame
        pa.header.stamp = stamp

        # Keep output order stable by landmark id.
        for lm in sorted(self.landmarks, key=lambda item: item.id):
            if self.map_points_confirmed_only and lm.status != "confirmed":
                continue

            pose = Pose()
            pose.position.x = lm.x
            pose.position.y = lm.y
            pose.position.z = lm.z
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.map_points_pub.publish(pa)

    def save_csv(self) -> None:
        try:
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            with self.csv_path.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "id", "x", "y", "z", "hit_count", "status", "created_sec", "last_seen_sec", "obs_count", "cov_trace_xy"
                ])
                for lm in self.landmarks:
                    if lm.status != "confirmed":
                        continue
                    writer.writerow([
                        lm.id,
                        f"{lm.x:.6f}",
                        f"{lm.y:.6f}",
                        f"{lm.z:.6f}",
                        lm.hit_count,
                        lm.status,
                        f"{lm.created_sec:.6f}",
                        f"{lm.last_seen_sec:.6f}",
                        lm.obs_count,
                        f"{lm.covariance_trace_xy:.8f}",
                    ])
        except Exception as exc:
            self.get_logger().error(f"Failed to save CSV '{self.csv_path}': {exc}")


def main() -> None:
    rclpy.init()
    node = SeedlingMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_csv()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
