#!/usr/bin/env python3
# Copyright 2026 Maize Weeding System Contributors
# SPDX-License-Identifier: MIT

"""Gate formal recording on recent Livox LiDAR/IMU health."""

import argparse
import json
import math
import re
import signal
import statistics
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class ImuHealthNode(Node):
    def __init__(self, subscription_depth):
        super().__init__("livox_recording_health_check")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=subscription_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.receipt_ns = []
        self.header_ns = []
        self.subscription = self.create_subscription(
            Imu, "/livox/imu", self.callback, qos
        )

    def callback(self, message):
        self.receipt_ns.append(time.time_ns())
        stamp = message.header.stamp
        self.header_ns.append(stamp.sec * 1_000_000_000 + stamp.nanosec)


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.communicate()


def main():
    default_config = (
        get_package_share_directory("livox_ros_driver2")
        + "/config/livox_recording.yaml"
    )
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config", default=default_config)
    config_args, _ = config_parser.parse_known_args()
    with open(config_args.config, encoding="utf-8") as config_file:
        health_config = yaml.safe_load(config_file)["health"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=default_config)
    parser.add_argument(
        "--duration", type=float, default=health_config["duration_s"]
    )
    parser.add_argument(
        "--imu-min-hz", type=float, default=health_config["imu_min_hz"]
    )
    parser.add_argument(
        "--lidar-min-hz", type=float, default=health_config["lidar_min_hz"]
    )
    parser.add_argument(
        "--lidar-max-hz", type=float, default=health_config["lidar_max_hz"]
    )
    parser.add_argument(
        "--imu-max-gap-ms",
        type=float,
        default=health_config["imu_max_header_gap_ms"],
    )
    parser.add_argument(
        "--imu-max-delay-p95-ms",
        type=float,
        default=health_config["imu_max_delay_p95_ms"],
    )
    parser.add_argument(
        "--imu-max-delay-ms",
        type=float,
        default=health_config["imu_max_delay_ms"],
    )
    parser.add_argument("--output")
    args = parser.parse_args()

    lidar_hz = subprocess.Popen(
        [
            "ros2",
            "topic",
            "hz",
            "--window",
            str(health_config["lidar_rate_window"]),
            "/livox/lidar",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    rclpy.init()
    node = ImuHealthNode(health_config["imu_subscription_depth"])
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    lidar_output, _ = stop_process(lidar_hz)
    lidar_rates = [
        float(value)
        for value in re.findall(r"average rate:\s*([0-9.]+)", lidar_output)
    ]
    lidar_rate = lidar_rates[-1] if lidar_rates else None

    receipt_gaps = [
        (right - left) / 1e9
        for left, right in zip(node.receipt_ns, node.receipt_ns[1:])
    ]
    header_gaps = [
        (right - left) / 1e9
        for left, right in zip(node.header_ns, node.header_ns[1:])
    ]
    delays = [
        (receipt - header) / 1e9
        for receipt, header in zip(node.receipt_ns, node.header_ns)
    ]
    receipt_span = (
        (node.receipt_ns[-1] - node.receipt_ns[0]) / 1e9
        if len(node.receipt_ns) > 1
        else 0.0
    )
    imu_rate = (
        (len(node.receipt_ns) - 1) / receipt_span
        if receipt_span > 0.0
        else 0.0
    )
    delay_p95 = percentile(delays, 0.95)
    failures = []
    if imu_rate < args.imu_min_hz:
        failures.append(
            f"IMU rate {imu_rate:.2f} Hz is below {args.imu_min_hz:.2f} Hz"
        )
    if lidar_rate is None:
        failures.append("LiDAR rate could not be measured")
    elif not args.lidar_min_hz <= lidar_rate <= args.lidar_max_hz:
        failures.append(
            f"LiDAR rate {lidar_rate:.2f} Hz is outside "
            f"[{args.lidar_min_hz:.2f}, {args.lidar_max_hz:.2f}] Hz"
        )
    nonmonotonic = sum(gap <= 0.0 for gap in header_gaps)
    if nonmonotonic:
        failures.append(f"IMU has {nonmonotonic} non-monotonic timestamps")
    max_header_gap = max(header_gaps, default=float("inf"))
    if max_header_gap * 1000.0 > args.imu_max_gap_ms:
        failures.append(
            f"IMU max header gap {max_header_gap * 1000.0:.2f} ms exceeds "
            f"{args.imu_max_gap_ms:.2f} ms"
        )
    if delay_p95 is None or (
        delay_p95 * 1000.0 > args.imu_max_delay_p95_ms
    ):
        failures.append(
            "IMU p95 receive delay is unavailable or exceeds "
            f"{args.imu_max_delay_p95_ms:.2f} ms"
        )
    max_delay = max(delays, default=float("inf"))
    if max_delay * 1000.0 > args.imu_max_delay_ms:
        failures.append(
            f"IMU max receive delay {max_delay * 1000.0:.2f} ms exceeds "
            f"{args.imu_max_delay_ms:.2f} ms"
        )

    report = {
        "healthy": not failures,
        "duration_s": args.duration,
        "imu": {
            "count": len(node.receipt_ns),
            "rate_hz": imu_rate,
            "receipt_gap_max_s": max(receipt_gaps, default=None),
            "header_gap_median_s": (
                statistics.median(header_gaps) if header_gaps else None
            ),
            "header_gap_max_s": (
                max(header_gaps) if header_gaps else None
            ),
            "header_nonmonotonic": nonmonotonic,
            "receive_delay_median_s": (
                statistics.median(delays) if delays else None
            ),
            "receive_delay_p95_s": delay_p95,
            "receive_delay_max_s": max(delays, default=None),
        },
        "lidar": {
            "rate_hz": lidar_rate,
            "topic_hz_output": lidar_output.strip().splitlines()[-8:],
        },
        "failures": failures,
    }
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(text + "\n")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
