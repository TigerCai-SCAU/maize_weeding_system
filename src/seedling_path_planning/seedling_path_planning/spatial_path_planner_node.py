"""ROS 2 node that turns a fused seedling map into dual-arm 3D S paths."""

from __future__ import annotations

import json
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .planner_core import (
    PlannerConfig,
    estimate_terrain_surface,
    lateral_travel_height_to_world,
    minimum_seedling_clearance,
    offset_height_from_terrain,
    plan_dual_arm_s,
    world_to_lateral_travel,
)


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _axis_index(value: str) -> int:
    key = str(value).strip().lower()
    if key not in AXIS_INDEX:
        raise ValueError(f"axis must be x, y or z, got {value!r}")
    return AXIS_INDEX[key]


def _orientation_along(direction: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion that rotates the local +X axis toward a world direction."""
    vector = np.asarray(direction, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    target = vector / norm
    source = np.asarray([1.0, 0.0, 0.0])
    dot = float(np.dot(source, target))
    if dot < -0.999999:
        return 0.0, 0.0, 1.0, 0.0
    cross = np.cross(source, target)
    quaternion = np.asarray([cross[0], cross[1], cross[2], 1.0 + dot])
    quaternion /= max(float(np.linalg.norm(quaternion)), 1e-9)
    return tuple(float(value) for value in quaternion)


class SpatialPathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("spatial_path_planner")
        self.declare_parameter("seedling_map_topic", "/seedling/map_points")
        self.declare_parameter(
            "terrain_topic", "/ground/global_elevation_points"
        )
        self.declare_parameter("path_topic", "/weeding/tool_path")
        self.declare_parameter("work_points_topic", "/weeding/work_points")
        self.declare_parameter("arm_1_path_topic", "/weeding/arm_1/tool_path")
        self.declare_parameter("arm_1_work_points_topic", "/weeding/arm_1/work_points")
        self.declare_parameter("arm_2_path_topic", "/weeding/arm_2/tool_path")
        self.declare_parameter("arm_2_work_points_topic", "/weeding/arm_2/work_points")
        self.declare_parameter("marker_topic", "/weeding/path_markers")
        self.declare_parameter("status_topic", "/weeding/plan_status")
        self.declare_parameter("world_frame", "camera_init")

        self.declare_parameter("travel_axis", "x")
        self.declare_parameter("lateral_axis", "y")
        self.declare_parameter("height_axis", "z")
        self.declare_parameter("expected_row_spacing", 0.55)
        self.declare_parameter("expected_plant_spacing", 0.20)
        self.declare_parameter("row_cluster_threshold", 0.22)
        self.declare_parameter("plant_spacing_tolerance", 0.35)
        self.declare_parameter("min_plants_per_row", 2)
        self.declare_parameter("protection_radius", 0.05)
        self.declare_parameter("safety_margin", 0.0)
        self.declare_parameter("coverage_spacing", 0.06)
        self.declare_parameter("path_resolution", 0.005)
        self.declare_parameter("lateral_work_margin", 0.10)
        self.declare_parameter("travel_work_margin", 0.05)
        self.declare_parameter("max_grid_cells", 250000)
        self.declare_parameter("s_sweep_offset", 0.05)
        self.declare_parameter("s_close_cluster_gap", 0.0)
        self.declare_parameter("s_first_side", 1)

        self.declare_parameter("tool_surface_offset", 0.02)
        self.declare_parameter("height_axis_up_sign", 1)
        self.declare_parameter("terrain_search_radius", 0.18)
        self.declare_parameter("terrain_nearest_count", 6)
        self.declare_parameter("terrain_min_neighbors", 3)
        self.declare_parameter("terrain_max_fit_rms", 0.025)
        self.declare_parameter("terrain_max_slope_deg", 35.0)
        self.declare_parameter("max_terrain_points", 30000)
        self.declare_parameter("replan_period_sec", 1.0)
        self.declare_parameter("minimum_map_points", 4)

        self.seedling_map_topic = str(
            self.get_parameter("seedling_map_topic").value
        )
        self.terrain_topic = str(self.get_parameter("terrain_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.work_points_topic = str(
            self.get_parameter("work_points_topic").value
        )
        self.arm_path_topics = [
            str(self.get_parameter("arm_1_path_topic").value),
            str(self.get_parameter("arm_2_path_topic").value),
        ]
        self.arm_work_points_topics = [
            str(self.get_parameter("arm_1_work_points_topic").value),
            str(self.get_parameter("arm_2_work_points_topic").value),
        ]
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value).strip()

        self.travel_axis = _axis_index(
            str(self.get_parameter("travel_axis").value)
        )
        self.lateral_axis = _axis_index(
            str(self.get_parameter("lateral_axis").value)
        )
        self.height_axis = _axis_index(
            str(self.get_parameter("height_axis").value)
        )
        if sorted((self.travel_axis, self.lateral_axis, self.height_axis)) != [
            0,
            1,
            2,
        ]:
            raise ValueError("travel_axis, lateral_axis and height_axis must differ")

        self.config = PlannerConfig(
            expected_row_spacing=float(
                self.get_parameter("expected_row_spacing").value
            ),
            expected_plant_spacing=float(
                self.get_parameter("expected_plant_spacing").value
            ),
            row_cluster_threshold=float(
                self.get_parameter("row_cluster_threshold").value
            ),
            plant_spacing_tolerance=float(
                self.get_parameter("plant_spacing_tolerance").value
            ),
            min_plants_per_row=int(
                self.get_parameter("min_plants_per_row").value
            ),
            protection_radius=float(
                self.get_parameter("protection_radius").value
            ),
            safety_margin=float(self.get_parameter("safety_margin").value),
            coverage_spacing=float(
                self.get_parameter("coverage_spacing").value
            ),
            path_resolution=float(
                self.get_parameter("path_resolution").value
            ),
            lateral_work_margin=float(
                self.get_parameter("lateral_work_margin").value
            ),
            travel_work_margin=float(
                self.get_parameter("travel_work_margin").value
            ),
            max_grid_cells=int(self.get_parameter("max_grid_cells").value),
            s_sweep_offset=float(self.get_parameter("s_sweep_offset").value),
            s_close_cluster_gap=float(
                self.get_parameter("s_close_cluster_gap").value
            ),
            s_first_side=int(self.get_parameter("s_first_side").value),
        )
        self.tool_surface_offset = float(
            self.get_parameter("tool_surface_offset").value
        )
        self.height_axis_up_sign = int(
            self.get_parameter("height_axis_up_sign").value
        )
        if self.height_axis_up_sign not in (-1, 1):
            raise ValueError("height_axis_up_sign must be +1 or -1")
        self.terrain_search_radius = float(
            self.get_parameter("terrain_search_radius").value
        )
        self.terrain_nearest_count = int(
            self.get_parameter("terrain_nearest_count").value
        )
        self.terrain_min_neighbors = int(
            self.get_parameter("terrain_min_neighbors").value
        )
        self.terrain_max_fit_rms = float(
            self.get_parameter("terrain_max_fit_rms").value
        )
        self.terrain_max_slope_deg = float(
            self.get_parameter("terrain_max_slope_deg").value
        )
        self.max_terrain_points = int(
            self.get_parameter("max_terrain_points").value
        )
        self.minimum_map_points = int(
            self.get_parameter("minimum_map_points").value
        )

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PoseArray,
            self.seedling_map_topic,
            self.seedling_map_callback,
            qos_reliable,
        )
        self.create_subscription(
            PointCloud2,
            self.terrain_topic,
            self.terrain_callback,
            qos_sensor,
        )
        self.path_publisher = self.create_publisher(
            Path, self.path_topic, qos_reliable
        )
        self.work_points_publisher = self.create_publisher(
            PoseArray, self.work_points_topic, qos_reliable
        )
        self.arm_path_publishers = [
            self.create_publisher(Path, topic, qos_reliable)
            for topic in self.arm_path_topics
        ]
        self.arm_work_points_publishers = [
            self.create_publisher(PoseArray, topic, qos_reliable)
            for topic in self.arm_work_points_topics
        ]
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, qos_reliable
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, qos_reliable
        )

        self.lock = threading.Lock()
        self.seedlings_xyz = np.empty((0, 3), dtype=np.float64)
        self.seedling_stamp = None
        self.terrain_lth = np.empty((0, 3), dtype=np.float64)
        self.terrain_version = 0
        self.last_map_signature: Optional[tuple] = None
        self.last_planned_signature: Optional[tuple] = None
        replan_period = max(
            0.1, float(self.get_parameter("replan_period_sec").value)
        )
        self.create_timer(replan_period, self.plan_timer_callback)
        self.get_logger().info(
            "SpatialPathPlanner started. map=%s terrain=%s axes=(travel=%d,"
            " lateral=%d, height=%d) protection=%.3f+%.3f m"
            % (
                self.seedling_map_topic,
                self.terrain_topic,
                self.travel_axis,
                self.lateral_axis,
                self.height_axis,
                self.config.protection_radius,
                self.config.safety_margin,
            )
        )

    def seedling_map_callback(self, message: PoseArray) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self._publish_status(
                {
                    "valid": False,
                    "reason": "seedling_frame_mismatch",
                    "received_frame": message.header.frame_id,
                    "expected_frame": self.world_frame,
                }
            )
            return
        points = np.asarray(
            [
                [pose.position.x, pose.position.y, pose.position.z]
                for pose in message.poses
            ],
            dtype=np.float64,
        ).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        signature = tuple(np.round(points.reshape(-1), 4))
        with self.lock:
            self.seedlings_xyz = points
            self.seedling_stamp = message.header.stamp
            self.last_map_signature = signature

    def terrain_callback(self, message: PointCloud2) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warn(
                "terrain frame '%s' does not match '%s'; ignored"
                % (message.header.frame_id, self.world_frame),
                throttle_duration_sec=3.0,
            )
            return
        try:
            xyz = np.asarray(
                [
                    (float(point[0]), float(point[1]), float(point[2]))
                    for point in point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ],
                dtype=np.float64,
            ).reshape(-1, 3)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(
                f"failed to read terrain cloud: {exc}",
                throttle_duration_sec=3.0,
            )
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if self.max_terrain_points > 0 and len(xyz) > self.max_terrain_points:
            stride = int(math.ceil(len(xyz) / self.max_terrain_points))
            xyz = xyz[::stride]
        if not xyz.size:
            return
        terrain_lth = xyz[
            :, [self.lateral_axis, self.travel_axis, self.height_axis]
        ]
        with self.lock:
            self.terrain_lth = terrain_lth
            self.terrain_version += 1

    def plan_timer_callback(self) -> None:
        with self.lock:
            seedlings_xyz = self.seedlings_xyz.copy()
            terrain_lth = self.terrain_lth.copy()
            stamp = self.seedling_stamp
            map_signature = self.last_map_signature
            terrain_version = self.terrain_version
        if map_signature is None:
            return
        signature = (map_signature, terrain_version)
        if signature == self.last_planned_signature:
            return
        if len(seedlings_xyz) < self.minimum_map_points:
            self._publish_empty_plan()
            self._publish_status(
                {
                    "valid": False,
                    "reason": "insufficient_seedlings",
                    "seedling_count": int(len(seedlings_xyz)),
                    "minimum_map_points": self.minimum_map_points,
                }
            )
            self.last_planned_signature = signature
            return

        seedling_lt = world_to_lateral_travel(
            seedlings_xyz, self.lateral_axis, self.travel_axis
        )
        try:
            result = plan_dual_arm_s(seedling_lt, self.config)
        except ValueError as exc:
            self._publish_empty_plan()
            self._publish_status(
                {
                    "valid": False,
                    "reason": "planning_failed",
                    "detail": str(exc),
                    "seedling_count": int(len(seedlings_xyz)),
                }
            )
            self.last_planned_signature = signature
            return

        arm_paths_xyz = []
        terrain_fit_rms_values = []
        terrain_slope_values = []
        terrain_support_values = []
        terrain_failure = None
        for arm_path_lt in result.arm_paths_lt:
            path_xyz = []
            for lateral, travel in arm_path_lt:
                surface = estimate_terrain_surface(
                    float(lateral),
                    float(travel),
                    terrain_lth,
                    self.terrain_search_radius,
                    self.terrain_nearest_count,
                    self.terrain_min_neighbors,
                )
                if surface is None:
                    terrain_failure = {
                        "reason": "terrain_support_missing",
                        "lateral_m": round(float(lateral), 4),
                        "travel_m": round(float(travel), 4),
                    }
                    break
                slope_deg = math.degrees(
                    math.atan(
                        math.hypot(
                            surface.slope_lateral,
                            surface.slope_travel,
                        )
                    )
                )
                if (
                    surface.rms_error > self.terrain_max_fit_rms
                    or slope_deg > self.terrain_max_slope_deg
                ):
                    terrain_failure = {
                        "reason": "terrain_fit_rejected",
                        "lateral_m": round(float(lateral), 4),
                        "travel_m": round(float(travel), 4),
                        "fit_rms_m": round(surface.rms_error, 4),
                        "slope_deg": round(slope_deg, 2),
                    }
                    break
                height = offset_height_from_terrain(
                    surface,
                    self.tool_surface_offset,
                    self.height_axis_up_sign,
                )
                terrain_fit_rms_values.append(surface.rms_error)
                terrain_slope_values.append(slope_deg)
                terrain_support_values.append(surface.support_count)
                path_xyz.append(
                    lateral_travel_height_to_world(
                        float(lateral),
                        float(travel),
                        height,
                        self.lateral_axis,
                        self.travel_axis,
                        self.height_axis,
                    )
                )
            if terrain_failure is not None:
                break
            arm_paths_xyz.append(np.asarray(path_xyz, dtype=np.float64))
        if terrain_failure is not None:
            self._publish_empty_plan()
            self._publish_status(
                {
                    "valid": False,
                    **terrain_failure,
                    "terrain_point_count": int(len(terrain_lth)),
                    "terrain_search_radius_m": self.terrain_search_radius,
                }
            )
            self.last_planned_signature = signature
            return
        clearances = [
            minimum_seedling_clearance(path, result.seedling_lt)
            for path in result.arm_paths_lt
        ]
        now = self.get_clock().now().to_msg()
        # The legacy topics remain aliases for arm 1 so existing visualization
        # and logging consumers do not break. New control code must use the two
        # explicit arm topics.
        self._publish_path(arm_paths_xyz[0], now)
        for arm_index, path_xyz in enumerate(arm_paths_xyz):
            self._publish_path(
                path_xyz,
                now,
                self.arm_path_publishers[arm_index],
                self.arm_work_points_publishers[arm_index],
            )
        self._publish_markers(result, arm_paths_xyz, seedlings_xyz, now)
        missing_slots = int(sum(row.missing_slots for row in result.rows))
        close_pairs = int(sum(row.close_pairs for row in result.rows))
        path_lengths = [
            float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            for path in arm_paths_xyz
        ]
        predicted_missing = [
            [list(point) for point in row.predicted_missing_lt]
            for row in result.rows
        ]
        self._publish_status(
            {
                "valid": True,
                "frame": self.world_frame,
                "seedling_count": int(len(seedlings_xyz)),
                "row_count": len(result.rows),
                "row_point_counts": [
                    len(row.point_indices) for row in result.rows
                ],
                "expected_row_spacing_m": self.config.expected_row_spacing,
                "measured_row_spacing_m": [
                    round(value, 4) for value in result.row_spacing_measured
                ],
                "expected_plant_spacing_m": self.config.expected_plant_spacing,
                "possible_missing_slots": missing_slots,
                "close_or_resown_pairs": close_pairs,
                "protection_radius_m": self.config.protection_radius,
                "effective_obstacle_radius_m": result.obstacle_radius,
                "minimum_path_clearance_m": [
                    round(value, 4) for value in clearances
                ],
                "arm_path_pose_counts": [
                    int(len(path)) for path in arm_paths_xyz
                ],
                "arm_path_lengths_m": [
                    round(value, 3) for value in path_lengths
                ],
                "arm_topics": self.arm_path_topics,
                "legacy_path_alias": "arm_1",
                "predicted_missing_lt_m": predicted_missing,
                "terrain_point_count": int(len(terrain_lth)),
                "tool_surface_offset_m": self.tool_surface_offset,
                "tool_mode": (
                    "above_surface"
                    if self.tool_surface_offset >= 0.0
                    else "below_surface"
                ),
                "height_axis_up_sign": self.height_axis_up_sign,
                "terrain_fit_max_rms_m": round(
                    max(terrain_fit_rms_values, default=0.0), 4
                ),
                "terrain_max_slope_deg": round(
                    max(terrain_slope_values, default=0.0), 2
                ),
                "terrain_min_support_count": min(
                    terrain_support_values, default=0
                ),
                "source_stamp": (
                    {
                        "sec": int(stamp.sec),
                        "nanosec": int(stamp.nanosec),
                    }
                    if stamp is not None
                    else None
                ),
            }
        )
        self.last_planned_signature = signature

    def _publish_empty_plan(self) -> None:
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = stamp
        self.path_publisher.publish(path)
        poses = PoseArray()
        poses.header = path.header
        self.work_points_publisher.publish(poses)
        for path_publisher, work_points_publisher in zip(
            self.arm_path_publishers, self.arm_work_points_publishers
        ):
            path_publisher.publish(path)
            work_points_publisher.publish(poses)
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.marker_publisher.publish(markers)

    def _publish_path(
        self,
        path_xyz: np.ndarray,
        stamp,
        path_publisher=None,
        work_points_publisher=None,
    ) -> None:
        path_message = Path()
        path_message.header.frame_id = self.world_frame
        path_message.header.stamp = stamp
        pose_array = PoseArray()
        pose_array.header = path_message.header
        for index, xyz in enumerate(path_xyz):
            if index + 1 < len(path_xyz):
                direction = path_xyz[index + 1] - xyz
            elif index > 0:
                direction = xyz - path_xyz[index - 1]
            else:
                direction = np.asarray([1.0, 0.0, 0.0])
            qx, qy, qz, qw = _orientation_along(direction)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                float(xyz[0]),
                float(xyz[1]),
                float(xyz[2]),
            )
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            stamped = PoseStamped()
            stamped.header = path_message.header
            stamped.pose = pose
            path_message.poses.append(stamped)
            pose_array.poses.append(pose)
        (path_publisher or self.path_publisher).publish(path_message)
        (work_points_publisher or self.work_points_publisher).publish(pose_array)

    def _publish_markers(
        self,
        result,
        arm_paths_xyz: list[np.ndarray],
        seedlings_xyz: np.ndarray,
        stamp,
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 0

        for index, xyz in enumerate(seedlings_xyz):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = "seedling_protection"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(xyz[0])
            marker.pose.position.y = float(xyz[1])
            marker.pose.position.z = float(xyz[2] + 0.01)
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * result.obstacle_radius
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter
            if self.height_axis == 0:
                marker.scale.x = 0.035
            elif self.height_axis == 1:
                marker.scale.y = 0.035
            else:
                marker.scale.z = 0.035
            marker.color.r = 0.92
            marker.color.g = 0.22
            marker.color.b = 0.12
            marker.color.a = 0.42
            markers.markers.append(marker)

        path_colors = ((0.10, 0.55, 0.95), (0.58, 0.25, 0.86))
        for arm_index, path_xyz in enumerate(arm_paths_xyz):
            path_marker = Marker()
            path_marker.header.frame_id = self.world_frame
            path_marker.header.stamp = stamp
            path_marker.ns = f"arm_{arm_index + 1}_s_path"
            path_marker.id = marker_id
            marker_id += 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.012
            (
                path_marker.color.r,
                path_marker.color.g,
                path_marker.color.b,
            ) = path_colors[arm_index]
            path_marker.color.a = 0.95
            for xyz in path_xyz:
                point = Point()
                point.x, point.y, point.z = map(float, xyz)
                path_marker.points.append(point)
            markers.markers.append(path_marker)

        seedling_lt = result.seedling_lt
        fallback_height = float(np.median(seedlings_xyz[:, self.height_axis]))
        for row in result.rows:
            row_marker = Marker()
            row_marker.header.frame_id = self.world_frame
            row_marker.header.stamp = stamp
            row_marker.ns = "estimated_seedling_rows"
            row_marker.id = marker_id
            marker_id += 1
            row_marker.type = Marker.LINE_STRIP
            row_marker.action = Marker.ADD
            row_marker.scale.x = 0.018
            row_marker.color.r = 0.95
            row_marker.color.g = 0.72
            row_marker.color.b = 0.10
            row_marker.color.a = 0.90
            for travel in (result.bounds[2], result.bounds[3]):
                xyz = lateral_travel_height_to_world(
                    row.lateral_at(travel),
                    travel,
                    fallback_height + 0.035,
                    self.lateral_axis,
                    self.travel_axis,
                    self.height_axis,
                )
                point = Point()
                point.x, point.y, point.z = map(float, xyz)
                row_marker.points.append(point)
            markers.markers.append(row_marker)

        self.marker_publisher.publish(markers)

    def _publish_status(self, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = SpatialPathPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
