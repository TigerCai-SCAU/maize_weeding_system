#!/usr/bin/env python3
"""Broadcast the FAST-LIVO odometry pose as camera_init -> body TF."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("parent_frame", "camera_init")
        self.declare_parameter("child_frame", "body")

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value).strip()
        self.child_frame = str(self.get_parameter("child_frame").value).strip()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            qos,
        )

        self.get_logger().info(
            f"Broadcasting {self.parent_frame} -> {self.child_frame} "
            f"from {self.odom_topic}"
        )

    def odom_cb(self, msg: Odometry) -> None:
        parent_frame = self.parent_frame or msg.header.frame_id
        child_frame = self.child_frame or msg.child_frame_id
        if not parent_frame or not child_frame:
            self.get_logger().error(
                "Cannot broadcast odometry TF: parent or child frame is empty.",
                throttle_duration_sec=2.0,
            )
            return
        if parent_frame == child_frame:
            self.get_logger().error(
                f"Cannot broadcast TF with identical frames '{parent_frame}'.",
                throttle_duration_sec=2.0,
            )
            return

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
