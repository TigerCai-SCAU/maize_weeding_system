# -*- coding: utf-8 -*-
"""
Row seedling avoidance planner test node.

Updated:
- /weedarm/test_seedlings_arm remains PoseArray, but arrow orientation is changed to vertical +Z.
- Added /weedarm/test_seedling_markers as visualization_msgs/MarkerArray, using green cylinders/spheres to show plants more naturally.

Coordinate:
  arm_base frame
  X: forward travel direction
  Y: lateral direction
  Z: upward positive, downward negative
"""

import math
import random
import bisect
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration


def smoothstep(a: float) -> float:
    a = max(0.0, min(1.0, a))
    return a * a * (3.0 - 2.0 * a)


def cluster_close_points(xs: List[float], ys: List[float], dist_threshold: float):
    if not xs:
        return [], []

    items = sorted(zip(xs, ys), key=lambda p: p[0])
    x_new, y_new = [], []

    cluster_x = [items[0][0]]
    cluster_y = [items[0][1]]

    for x, y in items[1:]:
        if x - cluster_x[-1] < dist_threshold:
            cluster_x.append(x)
            cluster_y.append(y)
        else:
            x_new.append(sum(cluster_x) / len(cluster_x))
            y_new.append(sum(cluster_y) / len(cluster_y))
            cluster_x = [x]
            cluster_y = [y]

    x_new.append(sum(cluster_x) / len(cluster_x))
    y_new.append(sum(cluster_y) / len(cluster_y))
    return x_new, y_new


class CubicHermite1D:
    """No scipy dependency. Smooth y=f(x) interpolation for strictly increasing x."""

    def __init__(self, xs: List[float], ys: List[float]):
        if len(xs) < 2:
            raise ValueError("Need at least 2 points")
        items = sorted(zip(xs, ys), key=lambda p: p[0])
        self.x = [p[0] for p in items]
        self.y = [p[1] for p in items]

        for i in range(1, len(self.x)):
            if self.x[i] <= self.x[i - 1]:
                self.x[i] = self.x[i - 1] + 1e-6

        n = len(self.x)
        self.m = [0.0] * n
        for i in range(n):
            if i == 0:
                dx = self.x[1] - self.x[0]
                self.m[i] = (self.y[1] - self.y[0]) / dx
            elif i == n - 1:
                dx = self.x[-1] - self.x[-2]
                self.m[i] = (self.y[-1] - self.y[-2]) / dx
            else:
                dx = self.x[i + 1] - self.x[i - 1]
                self.m[i] = (self.y[i + 1] - self.y[i - 1]) / dx

    def sample(self, xq: float) -> float:
        if xq <= self.x[0]:
            return self.y[0]
        if xq >= self.x[-1]:
            return self.y[-1]

        i = bisect.bisect_right(self.x, xq) - 1
        i = max(0, min(i, len(self.x) - 2))

        x0, x1 = self.x[i], self.x[i + 1]
        y0, y1 = self.y[i], self.y[i + 1]
        m0, m1 = self.m[i], self.m[i + 1]
        h = x1 - x0
        t = (xq - x0) / h

        h00 = 2 * t**3 - 3 * t**2 + 1
        h10 = t**3 - 2 * t**2 + t
        h01 = -2 * t**3 + 3 * t**2
        h11 = t**3 - t**2
        return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1


class RowSeedlingPlannerTest(Node):
    def __init__(self):
        super().__init__("row_seedling_planner_test")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("point_count", 64)
        self.declare_parameter("point_dt", 0.02)

        self.declare_parameter("num_plants", 20)
        self.declare_parameter("spacing", 0.25)
        self.declare_parameter("jitter_y", 0.03)
        self.declare_parameter("leak_prob", 0.10)
        self.declare_parameter("replay_prob", 0.15)
        self.declare_parameter("replay_dist", 0.10)
        self.declare_parameter("cluster_dist", 0.10)

        self.declare_parameter("safe_dist", 0.04)
        self.declare_parameter("first_up", True)
        self.declare_parameter("y_limit", 0.20)

        self.declare_parameter("vehicle_speed", 0.05)
        self.declare_parameter("loop_row", False)

        self.declare_parameter("ground_z0", -0.13)
        self.declare_parameter("work_depth", 0.02)
        self.declare_parameter("terrain_slope_x", 0.0)
        self.declare_parameter("terrain_slope_y", 0.0)
        self.declare_parameter("terrain_wave_amp", 0.0)
        self.declare_parameter("terrain_wave_len", 1.0)
        self.declare_parameter("z_min", -0.40)
        self.declare_parameter("z_max", -0.02)

        # 一行作业完成后：回安全位 -> 保持 -> 自动停止作业
        self.declare_parameter("run_once", True)
        self.declare_parameter("safe_y", 0.0)
        self.declare_parameter("safe_z", -0.08)
        self.declare_parameter("retract_time", 2.0)
        self.declare_parameter("hold_safe_time", 1.0)
        self.declare_parameter("auto_stop_cmd_start", True)

        self.declare_parameter("random_seed", 42)
        self.declare_parameter("preview_points", 300)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.point_count = int(self.get_parameter("point_count").value)
        self.point_dt = float(self.get_parameter("point_dt").value)

        self.num_plants = int(self.get_parameter("num_plants").value)
        self.spacing = float(self.get_parameter("spacing").value)
        self.jitter_y = float(self.get_parameter("jitter_y").value)
        self.leak_prob = float(self.get_parameter("leak_prob").value)
        self.replay_prob = float(self.get_parameter("replay_prob").value)
        self.replay_dist = float(self.get_parameter("replay_dist").value)
        self.cluster_dist = float(self.get_parameter("cluster_dist").value)

        self.safe_dist = float(self.get_parameter("safe_dist").value)
        self.first_up = bool(self.get_parameter("first_up").value)
        self.y_limit = abs(float(self.get_parameter("y_limit").value))

        self.vehicle_speed = float(self.get_parameter("vehicle_speed").value)
        self.loop_row = bool(self.get_parameter("loop_row").value)

        self.ground_z0 = float(self.get_parameter("ground_z0").value)
        self.work_depth = float(self.get_parameter("work_depth").value)
        self.terrain_slope_x = float(self.get_parameter("terrain_slope_x").value)
        self.terrain_slope_y = float(self.get_parameter("terrain_slope_y").value)
        self.terrain_wave_amp = float(self.get_parameter("terrain_wave_amp").value)
        self.terrain_wave_len = float(self.get_parameter("terrain_wave_len").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        self.run_once = bool(self.get_parameter("run_once").value)
        self.safe_y = float(self.get_parameter("safe_y").value)
        self.safe_z = float(self.get_parameter("safe_z").value)
        self.retract_time = float(self.get_parameter("retract_time").value)
        self.hold_safe_time = float(self.get_parameter("hold_safe_time").value)
        self.auto_stop_cmd_start = bool(self.get_parameter("auto_stop_cmd_start").value)

        self.random_seed = int(self.get_parameter("random_seed").value)
        self.preview_points = int(self.get_parameter("preview_points").value)

        self.pub_traj = self.create_publisher(JointTrajectory, "/weedarm/trajectory_yz", 10)
        self.pub_seedlings = self.create_publisher(PoseArray, "/weedarm/test_seedlings_arm", 10)
        self.pub_seedling_markers = self.create_publisher(MarkerArray, "/weedarm/test_seedling_markers", 10)
        self.pub_path = self.create_publisher(Path, "/weedarm/row_path_preview", 10)
        self.pub_cmd_start = self.create_publisher(Bool, "/weedarm/cmd_start", 10)
        self.pub_phase = self.create_publisher(String, "/weedarm/planner_phase", 10)

        self.t0 = time.monotonic()

        self.raw_x, self.raw_y, self.plant_x, self.plant_y = self.generate_seedlings()
        self.wp_x, self.wp_y = self.generate_avoidance_waypoints(self.plant_x, self.plant_y)
        self.path_interp = CubicHermite1D(self.wp_x, self.wp_y)

        self.path_start = self.wp_x[0]
        self.path_end = self.wp_x[-1]
        self.path_len = max(1e-6, self.path_end - self.path_start)
        self.row_duration = self.path_len / max(1e-6, abs(self.vehicle_speed))
        self.stop_sent = False

        self.publish_seedlings()
        self.publish_path_preview()

        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.on_timer)

        self.get_logger().info(
            "row_seedling_planner_test started: "
            f"plants={len(self.plant_x)}, safe_dist={self.safe_dist:.3f}m, "
            f"vehicle_speed={self.vehicle_speed:.3f}m/s, row_duration={self.row_duration:.2f}s, "
            f"ground_z0={self.ground_z0:.3f}m, depth={self.work_depth:.3f}m, "
            f"run_once={self.run_once}, safe=({self.safe_y:.3f},{self.safe_z:.3f})"
        )

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def generate_seedlings(self):
        rng = random.Random(self.random_seed)
        x_std = [i * self.spacing for i in range(self.num_plants)]
        y_std = [rng.uniform(0.0, self.jitter_y) for _ in range(self.num_plants)]

        x_leaked, y_leaked = [], []
        for x, y in zip(x_std, y_std):
            if rng.random() > self.leak_prob:
                x_leaked.append(x)
                y_leaked.append(y)

        x_replay, y_replay = [], []
        for x, y in zip(x_leaked, y_leaked):
            if rng.random() < self.replay_prob:
                x_replay.append(x + rng.uniform(-self.replay_dist / 2.0, self.replay_dist / 2.0))
                y_replay.append(y + rng.uniform(-self.replay_dist / 2.0, self.replay_dist / 2.0))

        x_all = x_leaked + x_replay
        y_all = y_leaked + y_replay
        plant_x, plant_y = cluster_close_points(x_all, y_all, self.cluster_dist)
        return x_all, y_all, plant_x, plant_y

    def generate_avoidance_waypoints(self, x_plants: List[float], y_plants: List[float]):
        if len(x_plants) < 2:
            return [0.0, 1.0], [0.0, 0.0]

        x_traj = []
        y_traj = []

        x_traj.append(x_plants[0] - self.spacing / 2.0)
        y_traj.append(y_plants[0] + self.safe_dist if self.first_up else y_plants[0] - self.safe_dist)

        for i, (x_p, y_p) in enumerate(zip(x_plants, y_plants)):
            up = ((i % 2 == 0) == self.first_up)
            y_peak = y_p + self.safe_dist if up else y_p - self.safe_dist
            x_traj.append(x_p)
            y_traj.append(y_peak)

            if i < len(x_plants) - 1:
                x_valley = 0.5 * (x_plants[i] + x_plants[i + 1])
                y_valley = 0.5 * (y_plants[i] + y_plants[i + 1])
                x_traj.append(x_valley)
                y_traj.append(y_valley)

        last_up = (((len(x_plants) - 1) % 2 == 0) == self.first_up)
        x_traj.append(x_plants[-1] + self.spacing / 2.0)
        y_traj.append(y_plants[-1] + self.safe_dist if last_up else y_plants[-1] - self.safe_dist)

        for i in range(1, len(x_traj)):
            if x_traj[i] <= x_traj[i - 1]:
                x_traj[i] = x_traj[i - 1] + 1e-5

        y_traj = [self.clamp(y, -self.y_limit, self.y_limit) for y in y_traj]
        return x_traj, y_traj

    def wrap_x(self, x: float) -> float:
        if not self.loop_row:
            return self.clamp(x, self.path_start, self.path_end)
        return self.path_start + ((x - self.path_start) % self.path_len)

    def ground_z(self, x: float, y: float) -> float:
        z = self.ground_z0 + self.terrain_slope_x * x + self.terrain_slope_y * y
        if abs(self.terrain_wave_amp) > 1e-9 and self.terrain_wave_len > 1e-6:
            z += self.terrain_wave_amp * math.sin(2.0 * math.pi * x / self.terrain_wave_len)
        return z

    def row_target_at_elapsed(self, elapsed: float):
        x_abs = self.path_start + self.vehicle_speed * elapsed
        x = self.wrap_x(x_abs)
        y = self.path_interp.sample(x)
        y = self.clamp(y, -self.y_limit, self.y_limit)
        z = self.ground_z(x, y) - self.work_depth
        z = self.clamp(z, self.z_min, self.z_max)
        return x, y, z

    def target_at_future_time(self, future_t: float):
        elapsed_now = time.monotonic() - self.t0
        elapsed = elapsed_now + future_t

        if (not self.run_once) or self.loop_row:
            return self.row_target_at_elapsed(elapsed)

        row_end_elapsed = self.row_duration
        retract_end_elapsed = self.row_duration + self.retract_time

        if elapsed <= row_end_elapsed:
            return self.row_target_at_elapsed(elapsed)

        _, row_end_y, row_end_z = self.row_target_at_elapsed(row_end_elapsed)

        if elapsed <= retract_end_elapsed:
            a = smoothstep((elapsed - row_end_elapsed) / max(1e-6, self.retract_time))
            y = row_end_y + a * (self.safe_y - row_end_y)
            z = row_end_z + a * (self.safe_z - row_end_z)
            z = self.clamp(z, self.z_min, self.z_max)
            return self.path_end, y, z

        return self.path_end, self.safe_y, self.safe_z

    def current_phase(self, elapsed_now: float) -> str:
        if (not self.run_once) or self.loop_row:
            return "LOOP_ROW"
        if elapsed_now <= self.row_duration:
            return "RUN_ROW"
        if elapsed_now <= self.row_duration + self.retract_time:
            return "RETRACT_TO_SAFE"
        if elapsed_now <= self.row_duration + self.retract_time + self.hold_safe_time:
            return "HOLD_SAFE"
        return "STOPPED"

    def maybe_auto_stop(self, elapsed_now: float):
        stop_time = self.row_duration + self.retract_time + self.hold_safe_time
        if self.run_once and self.auto_stop_cmd_start and (not self.stop_sent) and elapsed_now >= stop_time:
            msg = Bool()
            msg.data = False
            self.pub_cmd_start.publish(msg)
            self.stop_sent = True
            self.get_logger().info("一行完成，已回安全位，并发布 /weedarm/cmd_start false")

    def on_timer(self):
        now_msg = self.get_clock().now().to_msg()
        elapsed_now = time.monotonic() - self.t0
        self.maybe_auto_stop(elapsed_now)

        msg = JointTrajectory()
        msg.header.stamp = now_msg
        msg.joint_names = ["tool_y", "tool_z"]

        for i in range(self.point_count):
            future_t = i * self.point_dt
            _, y, z = self.target_at_future_time(future_t)

            p = JointTrajectoryPoint()
            p.positions = [y, z]
            total = i * self.point_dt
            sec = int(total)
            nanosec = int((total - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(p)

        self.pub_traj.publish(msg)

        phase_msg = String()
        phase_msg.data = self.current_phase(elapsed_now)
        self.pub_phase.publish(phase_msg)

        self.publish_seedlings(now_msg)
        self.publish_path_preview(now_msg)

    def publish_seedlings(self, stamp=None):
        stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        # PoseArray: RViz Arrow 默认沿 +X。这里把箭头方向旋转到 +Z。
        # Quaternion: rotation about Y by -90 deg maps +X to +Z.
        pa = PoseArray()
        pa.header.frame_id = "arm_base"
        pa.header.stamp = stamp

        for x, y in zip(self.plant_x, self.plant_y):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.ground_z(x, y)
            pose.orientation.x = 0.0
            pose.orientation.y = -0.70710678
            pose.orientation.z = 0.0
            pose.orientation.w = 0.70710678
            pa.poses.append(pose)

        self.pub_seedlings.publish(pa)

        # MarkerArray: 更推荐 RViz 显示苗点，用圆柱/球体，不会受姿态箭头影响。
        ma = MarkerArray()

        for i, (x, y) in enumerate(zip(self.plant_x, self.plant_y)):
            gz = self.ground_z(x, y)

            m = Marker()
            m.header.frame_id = "arm_base"
            m.header.stamp = stamp
            m.ns = "seedlings"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            # Cylinder axis is along local Z by default.
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = gz + 0.025
            m.pose.orientation.w = 1.0

            m.scale.x = 0.035
            m.scale.y = 0.035
            m.scale.z = 0.05

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.9

            ma.markers.append(m)

        # delete possible old markers if plant count changes
        for j in range(len(self.plant_x), 100):
            m = Marker()
            m.header.frame_id = "arm_base"
            m.header.stamp = stamp
            m.ns = "seedlings"
            m.id = j
            m.action = Marker.DELETE
            ma.markers.append(m)

        self.pub_seedling_markers.publish(ma)

    def publish_path_preview(self, stamp=None):
        path = Path()
        path.header.frame_id = "arm_base"
        path.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        for i in range(max(2, self.preview_points)):
            a = i / max(1, self.preview_points - 1)
            x = self.path_start + a * self.path_len
            y = self.path_interp.sample(x)
            z = self.ground_z(x, y) - self.work_depth

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = RowSeedlingPlannerTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

