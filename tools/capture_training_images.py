#!/usr/bin/env python3
"""Save a throttled ROS 2 Image stream as JPEG training frames."""

import argparse
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class TrainingImageCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("training_image_capture")
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.period_ns = int(1e9 / args.rate_hz)
        self.jpeg_quality = args.jpeg_quality
        self.max_images = args.max_images
        self.bridge = CvBridge()
        self.last_saved_stamp_ns = 0
        self.saved = 0
        self.received = 0
        self.create_subscription(
            Image, args.topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Saving {args.topic} at {args.rate_hz:.2f} Hz to {self.output_dir}"
        )

    def image_callback(self, msg: Image) -> None:
        self.received += 1
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = time.time_ns()
        if (
            self.last_saved_stamp_ns
            and stamp_ns - self.last_saved_stamp_ns < self.period_ns
        ):
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        filename = self.output_dir / f"frame_{self.saved:06d}_{stamp_ns}.jpg"
        if not cv2.imwrite(
            str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ):
            self.get_logger().error(f"Failed to save {filename}")
            return

        self.last_saved_stamp_ns = stamp_ns
        self.saved += 1
        if self.saved == 1 or self.saved % 20 == 0:
            self.get_logger().info(
                f"saved={self.saved}, received={self.received}, latest={filename.name}"
            )
        if self.max_images > 0 and self.saved >= self.max_images:
            self.get_logger().info(f"Reached max_images={self.max_images}")
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/miivii_gmsl/image3")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--max-images", type=int, default=0)
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")

    rclpy.init()
    node = TrainingImageCapture(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Capture stopped: saved={node.saved}, received={node.received}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
