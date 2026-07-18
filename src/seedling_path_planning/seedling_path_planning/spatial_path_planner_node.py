"""ROS 2 node that turns a fused seedling map into a safe 3D coverage path."""

from __future__ import annotations

import json
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
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
    interpolate_terrain_height,
    lateral_travel_height_to_world,
    minimum_seedling_clearance,
    plan_coverage,
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
        self.declare_parameter("protection_radius", 0.08)
        self.declare_parameter("safety_margin", 0.015)
        self.declare_parameter("coverage_spacing", 0.06)
        self.declare_parameter("path_resolution", 0.02)
        self.declare_parameter("lateral_work_margin", 0.10)
        self.declare_parameter("travel_work_margin", 0.05)
        self.declare_parameter("max_grid_cells", 250000)

        self.declare_parameter("tool_ground_clearance", 0.02)
        self.declare_parameter("terrain_search_radius", 0.18)
        self.declare_parameter("terrain_nearest_count", 6)
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
        )
        self.tool_ground_clearance = float(
            self.get_parameter("tool_ground_clearance").value
        )
        self.terrain_search_radius = float(
            self.get_parameter("terrain_search_radius").value
        )
        self.terrain_nearest_count = int(
            self.get_parameter("terrain_nearest_count").value
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
            result = plan_coverage(seedling_lt, self.config)
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

        fallback_height = float(np.median(seedlings_xyz[:, self.height_axis]))
        path_xyz = []
        for lateral, travel in result.path_lt:
            height = interpolate_terrain_height(
                float(lateral),
                float(travel),
                terrain_lth,
                self.terrain_search_radius,
                fallback_height,
                self.terrain_nearest_count,
            )
            path_xyz.append(
                lateral_travel_height_to_world(
                    float(lateral),
                    float(travel),
                    height + self.tool_ground_clearance,
                    self.lateral_axis,
                    self.travel_axis,
                    self.height_axis,
                )
            )
        path_xyz_array = np.asarray(path_xyz, dtype=np.float64)
        clearance = minimum_seedling_clearance(
            result.path_lt, result.seedling_lt
        )
        now = self.get_clock().now().to_msg()
        self._publish_path(path_xyz_array, now)
        self._publish_markers(result, path_xyz_array, seedlings_xyz, now)
        missing_slots = int(sum(row.missing_slots for row in result.rows))
        close_pairs = int(sum(row.close_pairs for row in result.rows))
        path_length = float(
            np.linalg.norm(np.diff(path_xyz_array, axis=0), axis=1).sum()
        )
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
                "minimum_path_clearance_m": round(clearance, 4),
                "path_pose_count": int(len(path_xyz_array)),
                "path_length_m": round(path_length, 3),
                "terrain_point_count": int(len(terrain_lth)),
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
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.marker_publisher.publish(markers)

    def _publish_path(self, path_xyz: np.ndarray, stamp) -> None:
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
        self.path_publisher.publish(path_message)
        self.work_points_publisher.publish(pose_array)

    def _publish_markers(
        self,
        result,
        path_xyz: np.ndarray,
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

        path_marker = Marker()
        path_marker.header.frame_id = self.world_frame
        path_marker.header.stamp = stamp
        path_marker.ns = "weeding_coverage_path"
        path_marker.id = marker_id
        marker_id += 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.012
        path_marker.color.r = 0.10
        path_marker.color.g = 0.55
        path_marker.color.b = 0.95
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
