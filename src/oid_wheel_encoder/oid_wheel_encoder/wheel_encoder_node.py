import math
import socket
import struct
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .protocol import counts_to_distance, make_read_position_request, parse_position_response, wrapped_delta


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_SFF_MASK = 0x7FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000


class WheelEncoderNode(Node):
    def __init__(self) -> None:
        super().__init__("oid_wheel_encoder")
        defaults = {
            "can_interface": "can0",
            "encoder_id": 1,
            "counts_per_rev": 32768,
            "wheel_diameter_m": 0.063,
            "encoder_revs_per_wheel_rev": 1.0,
            "direction": 1,
            "poll_hz": 50.0,
            "min_dt_sec": 0.005,
            "max_dt_sec": 0.20,
            "max_speed_mps": 3.0,
            "velocity_alpha": 0.35,
            "velocity_variance": 0.04,
            "odom_topic": "/wheel/odom",
            "frame_id": "wheel_odom",
            "child_frame_id": "aft_mapped",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.interface = str(self.get_parameter("can_interface").value)
        self.device_id = int(self.get_parameter("encoder_id").value)
        self.counts_per_rev = int(self.get_parameter("counts_per_rev").value)
        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.encoder_revs_per_wheel_rev = float(self.get_parameter("encoder_revs_per_wheel_rev").value)
        self.direction = int(self.get_parameter("direction").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.min_dt_sec = float(self.get_parameter("min_dt_sec").value)
        self.max_dt_sec = float(self.get_parameter("max_dt_sec").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.velocity_alpha = float(self.get_parameter("velocity_alpha").value)
        self.velocity_variance = float(self.get_parameter("velocity_variance").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)

        if self.poll_hz <= 0.0 or not 0.0 < self.velocity_alpha <= 1.0:
            raise ValueError("poll_hz must be positive and velocity_alpha must be in (0, 1]")
        make_read_position_request(self.device_id)
        counts_to_distance(0, self.counts_per_rev, self.wheel_diameter_m,
                           self.encoder_revs_per_wheel_rev, self.direction)

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setblocking(False)
        self.sock.bind((self.interface,))

        self.previous_count: int | None = None
        self.previous_time_ns: int | None = None
        self.filtered_speed = 0.0
        self.distance_m = 0.0
        self.request = make_read_position_request(self.device_id)
        self.timer = self.create_timer(1.0 / self.poll_hz, self._on_timer)
        self.get_logger().info(
            f"OID wheel encoder started: {self.interface} id={self.device_id} "
            f"{self.counts_per_rev} count/rev diameter={self.wheel_diameter_m:.3f}m"
        )

    def _on_timer(self) -> None:
        self._drain_responses()
        try:
            frame = CAN_FRAME.pack(self.device_id, len(self.request), self.request.ljust(8, b"\0"))
            self.sock.send(frame)
        except OSError as exc:
            self.get_logger().warn(f"CAN request failed on {self.interface}: {exc}", throttle_duration_sec=5.0)

    def _drain_responses(self) -> None:
        while True:
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f"CAN receive failed: {exc}", throttle_duration_sec=5.0)
                return

            if len(raw) != CAN_FRAME.size:
                continue
            can_id, dlc, payload = CAN_FRAME.unpack(raw)
            if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG) or (can_id & CAN_SFF_MASK) != self.device_id:
                continue
            count = parse_position_response(payload[:dlc], self.device_id)
            if count is None or not 0 <= count < self.counts_per_rev:
                continue
            self._handle_count(count, time.monotonic_ns())

    def _handle_count(self, count: int, now_ns: int) -> None:
        if self.previous_count is None or self.previous_time_ns is None:
            self.previous_count, self.previous_time_ns = count, now_ns
            return

        dt = (now_ns - self.previous_time_ns) * 1e-9
        delta = wrapped_delta(count, self.previous_count, self.counts_per_rev)
        self.previous_count, self.previous_time_ns = count, now_ns
        if not self.min_dt_sec <= dt <= self.max_dt_sec:
            return

        distance = counts_to_distance(
            delta,
            self.counts_per_rev,
            self.wheel_diameter_m,
            self.encoder_revs_per_wheel_rev,
            self.direction,
        )
        speed = distance / dt
        if not math.isfinite(speed) or abs(speed) > self.max_speed_mps:
            self.get_logger().warn(
                f"Rejected wheel speed {speed:.3f}m/s (delta={delta}, dt={dt:.4f}s)",
                throttle_duration_sec=2.0,
            )
            return

        self.distance_m += distance
        self.filtered_speed += self.velocity_alpha * (speed - self.filtered_speed)
        self._publish(now_ns)

    def _publish(self, _now_ns: int) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.distance_m
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = self.filtered_speed

        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 1e6
            msg.twist.covariance[index] = 1e6
        msg.twist.covariance[0] = self.velocity_variance
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        self.sock.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WheelEncoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
