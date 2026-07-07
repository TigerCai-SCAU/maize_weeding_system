#!/usr/bin/env python3
"""Ground mapper for maize weeding robot.

Input is usually FAST-LIVO's /cloud_registered, which is already registered in
map/world frame. The node extracts ground points and builds a local elevation map
by taking a low percentile height per XY grid cell.
"""

from collections import deque
import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(q) -> float:
    # ROS quaternion: x, y, z, w. Only yaw is needed for local ROI.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_xyz_points(msg: PointCloud2) -> np.ndarray:
    """Read x/y/z from PointCloud2 as Nx3 float32 array."""
    # Humble sensor_msgs_py may have read_points_numpy. Keep a fallback for safety.
    try:
        arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape((-1, 3))
        return arr[:, :3]
    except Exception:
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((float(p[0]), float(p[1]), float(p[2])))
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)


def make_xyz_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    if points is None or len(points) == 0:
        return pc2.create_cloud_xyz32(header, [])
    pts = np.asarray(points[:, :3], dtype=np.float32)
    return pc2.create_cloud_xyz32(header, pts.tolist())


class GroundMapper(Node):
    def __init__(self):
        super().__init__('ground_mapper_node')

        # Topics
        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('ground_points_topic', '/ground/points')
        self.declare_parameter('non_ground_points_topic', '/ground/non_ground_points')
        self.declare_parameter('elevation_points_topic', '/ground/elevation_points')
        self.declare_parameter('elevation_markers_topic', '/ground/elevation_markers')

        # ROI. If use_odom_roi is true and odom is available, x/y ROI is in robot/body local frame.
        # Otherwise x/y ROI is treated as input cloud frame/map coordinates.
        self.declare_parameter('use_odom_roi', True)
        self.declare_parameter('roi_x_min', -1.0)
        self.declare_parameter('roi_x_max', 4.0)
        self.declare_parameter('roi_y_min', -1.5)
        self.declare_parameter('roi_y_max', 1.5)
        self.declare_parameter('roi_z_min', -3.0)
        self.declare_parameter('roi_z_max', 1.0)

        # Elevation grid parameters
        self.declare_parameter('grid_resolution', 0.03)
        self.declare_parameter('height_percentile', 20.0)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('ground_keep_above', 0.04)
        self.declare_parameter('ground_keep_below', 0.03)
        self.declare_parameter('non_ground_above', 0.06)

        # Runtime/performance
        self.declare_parameter('process_every_n_clouds', 1)
        self.declare_parameter('max_points_per_cloud', 0)  # 0 means no downsample cap
        self.declare_parameter('publish_non_ground', True)
        self.declare_parameter('publish_elevation_markers', True)
        self.declare_parameter('marker_max_cells', 4000)
        self.declare_parameter('log_debug', True)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.ground_points_topic = self.get_parameter('ground_points_topic').value
        self.non_ground_points_topic = self.get_parameter('non_ground_points_topic').value
        self.elevation_points_topic = self.get_parameter('elevation_points_topic').value
        self.elevation_markers_topic = self.get_parameter('elevation_markers_topic').value

        self.use_odom_roi = bool(self.get_parameter('use_odom_roi').value)
        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)
        self.roi_z_min = float(self.get_parameter('roi_z_min').value)
        self.roi_z_max = float(self.get_parameter('roi_z_max').value)

        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.height_percentile = float(self.get_parameter('height_percentile').value)
        self.min_points_per_cell = int(self.get_parameter('min_points_per_cell').value)
        self.ground_keep_above = float(self.get_parameter('ground_keep_above').value)
        self.ground_keep_below = float(self.get_parameter('ground_keep_below').value)
        self.non_ground_above = float(self.get_parameter('non_ground_above').value)

        self.process_every_n_clouds = max(1, int(self.get_parameter('process_every_n_clouds').value))
        self.max_points_per_cloud = int(self.get_parameter('max_points_per_cloud').value)
        self.publish_non_ground = bool(self.get_parameter('publish_non_ground').value)
        self.publish_elevation_markers = bool(self.get_parameter('publish_elevation_markers').value)
        self.marker_max_cells = int(self.get_parameter('marker_max_cells').value)
        self.log_debug = bool(self.get_parameter('log_debug').value)

        if self.grid_resolution <= 0.0:
            raise ValueError('grid_resolution must be > 0')

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        normal_qos = QoSProfile(depth=10)

        self.sub_cloud = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, normal_qos)

        self.pub_ground = self.create_publisher(PointCloud2, self.ground_points_topic, sensor_qos)
        self.pub_non_ground = self.create_publisher(PointCloud2, self.non_ground_points_topic, sensor_qos)
        self.pub_elev = self.create_publisher(PointCloud2, self.elevation_points_topic, sensor_qos)
        self.pub_markers = self.create_publisher(MarkerArray, self.elevation_markers_topic, normal_qos)

        self.last_odom: Optional[Tuple[float, float, float, float]] = None  # x, y, z, yaw
        self.cloud_count = 0

        self.get_logger().info(
            f'ground_mapper started. cloud={self.cloud_topic}, odom={self.odom_topic}, '
            f'res={self.grid_resolution:.3f} m, percentile={self.height_percentile:.1f}'
        )

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.last_odom = (float(p.x), float(p.y), float(p.z), yaw)

    def cloud_cb(self, msg: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n_clouds != 0:
            return

        points = read_xyz_points(msg)
        if points.shape[0] == 0:
            self.get_logger().warn('input cloud empty')
            return

        if self.max_points_per_cloud > 0 and points.shape[0] > self.max_points_per_cloud:
            # Deterministic uniform stride downsample to limit CPU load.
            stride = int(math.ceil(points.shape[0] / self.max_points_per_cloud))
            points = points[::stride]

        points_roi = self.apply_roi(points)
        if points_roi.shape[0] < self.min_points_per_cell:
            self.get_logger().warn('ROI cloud too small after filtering')
            return

        ground_pts, non_ground_pts, elev_pts = self.extract_ground(points_roi)

        frame_id = msg.header.frame_id
        stamp = msg.header.stamp

        if ground_pts.shape[0] > 0:
            self.pub_ground.publish(make_xyz_cloud(ground_pts, stamp, frame_id))
        else:
            self.pub_ground.publish(make_xyz_cloud(np.empty((0, 3), dtype=np.float32), stamp, frame_id))

        if self.publish_non_ground:
            self.pub_non_ground.publish(make_xyz_cloud(non_ground_pts, stamp, frame_id))

        self.pub_elev.publish(make_xyz_cloud(elev_pts, stamp, frame_id))

        if self.publish_elevation_markers:
            self.publish_markers(elev_pts, stamp, frame_id)

        if self.log_debug:
            self.get_logger().info(
                f'[GROUND_MAP] input={points.shape[0]}, roi={points_roi.shape[0]}, '
                f'ground={ground_pts.shape[0]}, non_ground={non_ground_pts.shape[0]}, '
                f'cells={elev_pts.shape[0]}',
                throttle_duration_sec=1.0,
            )

    def apply_roi(self, points: np.ndarray) -> np.ndarray:
        z = points[:, 2]
        mask = (z >= self.roi_z_min) & (z <= self.roi_z_max)

        if self.use_odom_roi and self.last_odom is not None:
            ox, oy, _oz, yaw = self.last_odom
            dx = points[:, 0] - ox
            dy = points[:, 1] - oy
            c = math.cos(yaw)
            s = math.sin(yaw)
            # Map/world -> body-local XY using inverse yaw rotation.
            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy
            mask &= (local_x >= self.roi_x_min) & (local_x <= self.roi_x_max)
            mask &= (local_y >= self.roi_y_min) & (local_y <= self.roi_y_max)
        else:
            # Fallback: treat x/y ROI as fixed coordinates in the cloud frame.
            if self.use_odom_roi and self.last_odom is None:
                self.get_logger().warn(
                    'use_odom_roi=true but no odom received yet; using fixed-frame x/y ROI',
                    throttle_duration_sec=2.0,
                )
            mask &= (points[:, 0] >= self.roi_x_min) & (points[:, 0] <= self.roi_x_max)
            mask &= (points[:, 1] >= self.roi_y_min) & (points[:, 1] <= self.roi_y_max)

        return points[mask]

    def extract_ground(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        res = self.grid_resolution
        ix = np.floor(points[:, 0] / res).astype(np.int64)
        iy = np.floor(points[:, 1] / res).astype(np.int64)
        keys = np.stack((ix, iy), axis=1)

        unique_keys, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        ground_z_for_point = np.full(points.shape[0], np.nan, dtype=np.float32)
        elev_list = []

        z_values = points[:, 2]
        for cell_idx, key in enumerate(unique_keys):
            if counts[cell_idx] < self.min_points_per_cell:
                continue
            cell_mask = (inv == cell_idx)
            z_cell = z_values[cell_mask]
            ground_z = float(np.percentile(z_cell, self.height_percentile))
            ground_z_for_point[cell_mask] = ground_z

            cx = (float(key[0]) + 0.5) * res
            cy = (float(key[1]) + 0.5) * res
            elev_list.append((cx, cy, ground_z))

        valid = np.isfinite(ground_z_for_point)
        if not np.any(valid):
            empty = np.empty((0, 3), dtype=np.float32)
            return empty, empty, empty

        dz = points[:, 2] - ground_z_for_point
        ground_mask = valid & (dz >= -self.ground_keep_below) & (dz <= self.ground_keep_above)
        non_ground_mask = valid & (dz > self.non_ground_above)

        ground_pts = points[ground_mask].astype(np.float32)
        non_ground_pts = points[non_ground_mask].astype(np.float32)
        elev_pts = np.asarray(elev_list, dtype=np.float32) if elev_list else np.empty((0, 3), dtype=np.float32)
        return ground_pts, non_ground_pts, elev_pts

    def publish_markers(self, elev_pts: np.ndarray, stamp, frame_id: str):
        arr = MarkerArray()

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'ground_elevation'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.grid_resolution * 0.7
        marker.scale.y = self.grid_resolution * 0.7
        marker.scale.z = self.grid_resolution * 0.7
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 0.85
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0

        if elev_pts.shape[0] > self.marker_max_cells > 0:
            stride = int(math.ceil(elev_pts.shape[0] / self.marker_max_cells))
            pts_for_marker = elev_pts[::stride]
        else:
            pts_for_marker = elev_pts

        for x, y, z in pts_for_marker:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        arr.markers.append(marker)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GroundMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
