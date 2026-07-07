# -*- coding: utf-8 -*-
import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SyncTrajPublisher(Node):
    def __init__(self):
        super().__init__("sync_traj_publisher")
        self.declare_parameter("amp_y", 0.02)
        self.declare_parameter("amp_z", 0.01)
        self.declare_parameter("z_center", -0.12)
        self.declare_parameter("period", 4.0)
        self.declare_parameter("phase_z_deg", 90.0)
        self.declare_parameter("point_count", 64)
        self.declare_parameter("point_dt", 0.02)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("ramp_time", 2.0)

        self.amp_y = float(self.get_parameter("amp_y").value)
        self.amp_z = float(self.get_parameter("amp_z").value)
        self.z_center = float(self.get_parameter("z_center").value)
        self.period = float(self.get_parameter("period").value)
        self.phase_z = math.radians(float(self.get_parameter("phase_z_deg").value))
        self.point_count = int(self.get_parameter("point_count").value)
        self.point_dt = float(self.get_parameter("point_dt").value)
        self.ramp_time = float(self.get_parameter("ramp_time").value)

        self.pub = self.create_publisher(JointTrajectory, "trajectory_yz", 10)
        self.t0 = time.monotonic()
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.on_timer)

        self.get_logger().info(
            f"发布同步Y/Z测试轨迹: amp_y={self.amp_y}m, amp_z={self.amp_z}m, "
            f"z_center={self.z_center}m, period={self.period}s"
        )

    @staticmethod
    def smoothstep(a: float) -> float:
        a = max(0.0, min(1.0, a))
        return a * a * (3.0 - 2.0 * a)

    def target(self, t: float):
        w = 2.0 * math.pi / self.period
        y = self.amp_y * math.sin(w * t)
        z = self.z_center + self.amp_z * math.sin(w * t + self.phase_z)
        if t < self.ramp_time:
            a = self.smoothstep(t / self.ramp_time)
            y *= a
            z = self.z_center + a * (z - self.z_center)
        return y, z

    def on_timer(self):
        elapsed = time.monotonic() - self.t0
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["tool_y", "tool_z"]

        for i in range(self.point_count):
            t = elapsed + i * self.point_dt
            y, z = self.target(t)
            p = JointTrajectoryPoint()
            p.positions = [y, z]
            total = i * self.point_dt
            sec = int(total)
            nanosec = int((total - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(p)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncTrajPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
