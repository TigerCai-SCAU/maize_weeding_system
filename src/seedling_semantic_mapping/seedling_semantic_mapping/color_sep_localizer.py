#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
橙色乒乓球测试节点：
不用 YOLO，直接用 HSV 颜色分割找橙色乒乓球，把球的底部中心当作“假出苗点”。

输入：
  - 彩色图像
  - /fastlivo/deskew/cloud_body
  - /aft_mapped_to_init

输出：
  - /seedling/observation_point_map
  - /seedling/current_observation_markers
  - /seedling/orange_mask
"""

from __future__ import annotations

from typing import List

import numpy as np
import rclpy
from sensor_msgs.msg import Image

from .yolo_sep_localizer import YoloSepLocalizer, SepDetection


class ColorSepLocalizer(YoloSepLocalizer):
    def __init__(self) -> None:
        # 这里故意沿用 yolo_sep_localizer 作为节点名，
        # 这样可以直接复用 seedling_pipeline.yaml 里面 yolo_sep_localizer: 下的参数。
        super().__init__()

        self.declare_parameter("orange_h_low", 5)
        self.declare_parameter("orange_h_high", 28)
        self.declare_parameter("orange_s_low", 80)
        self.declare_parameter("orange_v_low", 80)

        self.declare_parameter("color_min_area", 80)
        self.declare_parameter("color_max_area", 200000)
        self.declare_parameter("color_min_fill_ratio", 0.20)
        self.declare_parameter("color_point_v_ratio", 0.88)
        self.declare_parameter("color_morph_kernel", 5)
        self.declare_parameter("color_debug_log_every", 30)

        self.declare_parameter("publish_debug_mask", True)
        self.declare_parameter("debug_mask_topic", "/seedling/orange_mask")

        self.publish_debug_mask = bool(self.get_parameter("publish_debug_mask").value)
        self.debug_mask_topic = str(self.get_parameter("debug_mask_topic").value)

        self.debug_mask_pub = None
        if self.publish_debug_mask:
            self.debug_mask_pub = self.create_publisher(Image, self.debug_mask_topic, 5)

        self.get_logger().info("ColorSepLocalizer started: using orange ping-pong ball as fake SEP.")

    def publish_mask(self, mask: np.ndarray) -> None:
        if self.debug_mask_pub is None:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = int(mask.shape[0])
        msg.width = int(mask.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = int(mask.shape[1])
        msg.data = mask.astype(np.uint8).tobytes()
        self.debug_mask_pub.publish(msg)

    def run_yolo(self, img_rgb: np.ndarray) -> List[SepDetection]:
        """
        覆盖原来的 YOLO 推理函数。
        父类 image_cb 仍然负责：
          图像/点云/里程计同步
          2D 像素点到 3D 点云定位
          map 坐标转换
          发布 observation 和 marker
        """
        try:
            import cv2
        except Exception as exc:
            self.get_logger().error(f"cv2 import failed: {exc}. Try: sudo apt install python3-opencv")
            return []

        if img_rgb.ndim != 3 or img_rgb.shape[2] < 3:
            self.get_logger().warn(
                "ColorSepLocalizer needs color image, but current image looks mono.",
                throttle_duration_sec=2.0,
            )
            return []

        h_low = int(self.get_parameter("orange_h_low").value)
        h_high = int(self.get_parameter("orange_h_high").value)
        s_low = int(self.get_parameter("orange_s_low").value)
        v_low = int(self.get_parameter("orange_v_low").value)

        min_area = int(self.get_parameter("color_min_area").value)
        max_area = int(self.get_parameter("color_max_area").value)
        min_fill = float(self.get_parameter("color_min_fill_ratio").value)
        v_ratio = float(self.get_parameter("color_point_v_ratio").value)
        kernel_size = int(self.get_parameter("color_morph_kernel").value)
        debug_every = max(1, int(self.get_parameter("color_debug_log_every").value))

        hsv = cv2.cvtColor(img_rgb[:, :, :3], cv2.COLOR_RGB2HSV)

        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        self.publish_mask(mask)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

        detections: List[SepDetection] = []

        for label in range(1, num_labels):
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])

            if area < min_area or area > max_area:
                continue

            bbox_area = max(1, w * h)
            fill_ratio = float(area) / float(bbox_area)
            if fill_ratio < min_fill:
                continue

            cx, cy = centroids[label]
            if not np.isfinite(cx) or not np.isfinite(cy):
                continue

            # 对放在地面的球，底部中心点比中心点更接近地面接触点。
            u = float(cx)
            v = float(y + v_ratio * h)

            detections.append(
                SepDetection(
                    u=u,
                    v=v,
                    conf=float(area),
                    bbox=(float(x), float(y), float(x + w), float(y + h)),
                )
            )

        detections = sorted(detections, key=lambda d: d.conf, reverse=True)
        detections = detections[: self.max_detections]
        detections = self.sep_nms(detections)

        if self.frame_count % debug_every == 0:
            if detections:
                d = detections[0]
                self.get_logger().info(
                    f"orange detections={len(detections)} best_uv=({d.u:.1f}, {d.v:.1f}) area={d.conf:.0f}"
                )
            else:
                self.get_logger().info("orange detections=0")

        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorSepLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
