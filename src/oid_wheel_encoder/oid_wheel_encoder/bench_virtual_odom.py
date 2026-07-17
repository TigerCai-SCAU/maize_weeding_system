#!/usr/bin/env python3
"""Map scalar measuring-wheel travel into a configurable bench odometry pose."""

import math
from typing import Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def vector3(values: Sequence[float], name: str) -> Tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def quaternion_xyzw(
    values: Sequence[float],
) -> Tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("orientation_xyzw must contain exactly 4 values")
    quaternion = tuple(float(value) for value in values)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("orientation_xyzw must be a finite non-zero quaternion")
    return tuple(value / norm for value in quaternion)


def mapped_translation(
    distance_m: float,
    zero_distance_m: float,
    origin_xyz: Sequence[float],
    translation_per_meter: Sequence[float],
) -> Tuple[float, float, float]:
    relative_distance = float(distance_m) - float(zero_distance_m)
    return tuple(
        float(origin) + relative_distance * float(scale)
        for origin, scale in zip(origin_xyz, translation_per_meter)
    )


class BenchVirtualOdom(Node):
    def __init__(self) -> None:
        super().__init__("bench_virtual_odom")

        self.declare_parameter("input_topic", "/wheel/odom")
        self.declare_parameter("output_topic", "/bench/aft_mapped_to_init")
        self.declare_parameter("world_frame", "camera_init")
        self.declare_parameter("child_frame", "body")
        self.declare_parameter("translation_per_meter", [0.0, 0.0, -1.0])
        self.declare_parameter("origin_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("zero_on_start", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.translation_per_meter = vector3(
            self.get_parameter("translation_per_meter").value,
            "translation_per_meter",
        )
        self.origin_xyz = vector3(
            self.get_parameter("origin_xyz").value,
            "origin_xyz",
        )
        self.orientation = quaternion_xyzw(
            self.get_parameter("orientation_xyzw").value
        )
        self.zero_on_start = bool(self.get_parameter("zero_on_start").value)
        self.zero_distance_m = None

        self.publisher = self.create_publisher(
            Odometry,
            self.output_topic,
            50,
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_cb,
            50,
        )

        self.get_logger().info(
            "Bench virtual odom started: %s -> %s, translation_per_meter=%s"
            % (
                self.input_topic,
                self.output_topic,
                self.translation_per_meter,
            )
        )

    def odom_cb(self, msg: Odometry) -> None:
        distance_m = float(msg.pose.pose.position.x)
        if not math.isfinite(distance_m):
            return

        if self.zero_distance_m is None:
            self.zero_distance_m = distance_m if self.zero_on_start else 0.0

        translation = mapped_translation(
            distance_m,
            self.zero_distance_m,
            self.origin_xyz,
            self.translation_per_meter,
        )
        input_speed = float(msg.twist.twist.linear.x)

        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.world_frame
        output.child_frame_id = self.child_frame
        output.pose.pose.position.x = translation[0]
        output.pose.pose.position.y = translation[1]
        output.pose.pose.position.z = translation[2]
        output.pose.pose.orientation.x = self.orientation[0]
        output.pose.pose.orientation.y = self.orientation[1]
        output.pose.pose.orientation.z = self.orientation[2]
        output.pose.pose.orientation.w = self.orientation[3]
        output.twist.twist.linear.x = input_speed * self.translation_per_meter[0]
        output.twist.twist.linear.y = input_speed * self.translation_per_meter[1]
        output.twist.twist.linear.z = input_speed * self.translation_per_meter[2]
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = BenchVirtualOdom()
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
