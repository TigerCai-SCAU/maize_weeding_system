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
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from .yolo_sep_localizer import YoloSepLocalizer, SepDetection, image_msg_to_numpy, stamp_to_sec


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
        # 颜色检测在缩小图像上运行，检测坐标随后恢复到原始分辨率。
        self.declare_parameter("color_process_scale", 0.5)

        self.declare_parameter("publish_debug_mask", True)
        self.declare_parameter("debug_mask_topic", "/seedling/orange_mask")

        # 雷达投影调试图：
        # 绿色 = 投影雷达点
        # 红色 = 橙色检测点
        # 蓝色 = 距检测点最近的雷达投影点
        self.declare_parameter("publish_projection_debug", True)
        self.declare_parameter(
            "projection_debug_topic",
            "/seedling/projection_debug",
        )
        self.declare_parameter("projection_debug_every", 3)
        self.declare_parameter("projection_debug_max_points", 1500)

        self.publish_debug_mask = bool(self.get_parameter("publish_debug_mask").value)
        self.debug_mask_topic = str(self.get_parameter("debug_mask_topic").value)

        self.debug_mask_pub = None
        if self.publish_debug_mask:
            self.debug_mask_pub = self.create_publisher(
                Image,
                self.debug_mask_topic,
                5,
            )

        self.publish_projection_debug_enabled = bool(
            self.get_parameter("publish_projection_debug").value
        )
        self.projection_debug_topic = str(
            self.get_parameter("projection_debug_topic").value
        )
        self.projection_debug_every = max(
            1,
            int(self.get_parameter("projection_debug_every").value),
        )
        self.projection_debug_max_points = max(
            100,
            int(self.get_parameter("projection_debug_max_points").value),
        )

        self.projection_debug_pub = None
        if self.publish_projection_debug_enabled:
            self.projection_debug_pub = self.create_publisher(
                Image,
                self.projection_debug_topic,
                2,
            )

        self.get_logger().info(
            "ColorSepLocalizer started: using orange ping-pong ball as fake SEP."
        )

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

    def publish_projection_debug_image(
        self,
        img_rgb: np.ndarray,
        detections: List[SepDetection],
        uv: np.ndarray,
        valid: np.ndarray,
        source_msg: Image,
    ) -> None:
        """发布雷达投影点与橙色检测点的叠加图。"""
        if self.projection_debug_pub is None:
            return

        if self.frame_count % self.projection_debug_every != 0:
            return

        try:
            import cv2
        except Exception as exc:
            self.get_logger().error(
                f"projection debug cv2 failed: {exc}"
            )
            return

        overlay = np.ascontiguousarray(
            img_rgb[:, :, :3].copy()
        )
        height, width = overlay.shape[:2]

        finite = (
            valid
            & np.isfinite(uv).all(axis=1)
        )

        in_image = (
            finite
            & (uv[:, 0] >= 0.0)
            & (uv[:, 0] < float(width))
            & (uv[:, 1] >= 0.0)
            & (uv[:, 1] < float(height))
        )

        projected_indices = np.flatnonzero(in_image)

        if projected_indices.size > self.projection_debug_max_points:
            step = int(
                np.ceil(
                    projected_indices.size
                    / float(self.projection_debug_max_points)
                )
            )
            projected_indices = projected_indices[::step]

        # 绿色：雷达投影点
        for index in projected_indices:
            u, v = uv[index]
            cv2.circle(
                overlay,
                (int(round(u)), int(round(v))),
                2,
                (0, 255, 0),
                -1,
            )

        valid_indices = np.flatnonzero(finite)
        log_parts = []

        for det_index, det in enumerate(detections):
            det_u = int(round(det.u))
            det_v = int(round(det.v))

            # 红色：检测到的球底部点
            cv2.circle(
                overlay,
                (det_u, det_v),
                12,
                (255, 0, 0),
                3,
            )

            # 黄色：检测框
            if det.bbox is not None:
                x1, y1, x2, y2 = det.bbox
                cv2.rectangle(
                    overlay,
                    (int(round(x1)), int(round(y1))),
                    (int(round(x2)), int(round(y2))),
                    (255, 255, 0),
                    3,
                )

            nearest_distance = float("inf")
            candidate_count = 0

            if valid_indices.size > 0:
                valid_uv = uv[valid_indices]

                du = valid_uv[:, 0] - det.u
                dv = valid_uv[:, 1] - det.v
                dist2 = du * du + dv * dv

                nearest_local = int(np.argmin(dist2))
                nearest_global = int(valid_indices[nearest_local])

                nearest_distance = float(
                    np.sqrt(dist2[nearest_local])
                )

                candidate_count = int(
                    np.count_nonzero(
                        dist2
                        <= self.search_radius_px
                        * self.search_radius_px
                    )
                )

                nearest_u = int(
                    round(uv[nearest_global, 0])
                )
                nearest_v = int(
                    round(uv[nearest_global, 1])
                )

                # 蓝色：最近投影点
                cv2.circle(
                    overlay,
                    (nearest_u, nearest_v),
                    10,
                    (0, 0, 255),
                    3,
                )

                # 青色：检测点到最近雷达点的偏差
                cv2.line(
                    overlay,
                    (det_u, det_v),
                    (nearest_u, nearest_v),
                    (0, 255, 255),
                    2,
                )

            text = (
                f"{det_index}: d={nearest_distance:.1f}px "
                f"n={candidate_count}"
            )

            cv2.putText(
                overlay,
                text,
                (det_u + 15, max(30, det_v - 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

            log_parts.append(
                f"#{det_index} "
                f"uv=({det.u:.1f},{det.v:.1f}) "
                f"nearest={nearest_distance:.1f}px "
                f"candidates={candidate_count}"
            )

        output = Image()
        output.header = source_msg.header
        output.height = int(height)
        output.width = int(width)
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = int(width * 3)
        output.data = overlay.tobytes()

        self.projection_debug_pub.publish(output)

        self.get_logger().info(
            "projection: " + " | ".join(log_parts),
            throttle_duration_sec=1.0,
        )

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
        debug_every = max(
            1,
            int(self.get_parameter("color_debug_log_every").value),
        )

        process_scale = float(
            self.get_parameter("color_process_scale").value
        )
        process_scale = min(max(process_scale, 0.1), 1.0)

        source_rgb = img_rgb[:, :, :3]

        if process_scale < 0.999:
            work_rgb = cv2.resize(
                source_rgb,
                None,
                fx=process_scale,
                fy=process_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            work_rgb = source_rgb

        inverse_scale = 1.0 / process_scale
        area_scale = process_scale * process_scale

        min_area_work = max(
            1,
            int(round(min_area * area_scale)),
        )
        max_area_work = max(
            min_area_work + 1,
            int(round(max_area * area_scale)),
        )

        kernel_size_work = max(
            1,
            int(round(kernel_size * process_scale)),
        )
        if kernel_size_work > 1 and kernel_size_work % 2 == 0:
            kernel_size_work += 1

        hsv = cv2.cvtColor(work_rgb, cv2.COLOR_RGB2HSV)

        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        if kernel_size_work > 1:
            kernel = np.ones(
                (kernel_size_work, kernel_size_work),
                np.uint8,
            )
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

            if area < min_area_work or area > max_area_work:
                continue

            bbox_area = max(1, w * h)
            fill_ratio = float(area) / float(bbox_area)
            if fill_ratio < min_fill:
                continue

            cx, cy = centroids[label]
            if not np.isfinite(cx) or not np.isfinite(cy):
                continue

            # 在缩小图上检测，但输出必须恢复到原始图像坐标。
            # 对放在地面的球，底部中心点接近地面接触位置。
            u = float(cx * inverse_scale)
            v = float((y + v_ratio * h) * inverse_scale)

            detections.append(
                SepDetection(
                    u=u,
                    v=v,
                    conf=float(area / area_scale),
                    bbox=(
                        float(x * inverse_scale),
                        float(y * inverse_scale),
                        float((x + w) * inverse_scale),
                        float((y + h) * inverse_scale),
                    ),
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

    def image_cb(self, msg: Image) -> None:
        """
        调试版回调：
        1. 收到图像后先做颜色分割并发布 /seedling/orange_mask
        2. 再去找最近 cloud/odom
        3. 同步成功后再发布 3D observation
        """
        self.frame_count += 1

        try:
            img = image_msg_to_numpy(msg)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        # 先运行颜色检测。run_yolo() 内部会发布 /seedling/orange_mask
        detections = self.run_yolo(img)

        image_time = stamp_to_sec(msg.header.stamp)
        cloud_msg, image_cloud_dt = self.nearest_cloud(image_time)
        if cloud_msg is None:
            cloud_dt_s = (
                "none"
                if image_cloud_dt is None
                else f"{image_cloud_dt*1000.0:.1f}ms"
            )
            self.get_logger().warn(
                f"orange image ok, but waiting sync: "
                f"cloud=missing dt={cloud_dt_s}, "
                f"detections={len(detections)}",
                throttle_duration_sec=2.0,
            )
            return

        cloud_time = stamp_to_sec(cloud_msg.header.stamp)
        odom_msg, cloud_odom_dt = self.nearest_odom(cloud_time)
        if odom_msg is None:
            odom_dt_s = (
                "none"
                if cloud_odom_dt is None
                else f"{cloud_odom_dt*1000.0:.1f}ms"
            )
            self.get_logger().warn(
                f"orange image/cloud ok, but odom is missing: "
                f"dt={odom_dt_s}, detections={len(detections)}",
                throttle_duration_sec=2.0,
            )
            return

        odom_frame = odom_msg.header.frame_id or self.world_frame
        if odom_frame != self.world_frame:
            self.get_logger().error(
                f"odom frame is '{odom_frame}', configured world_frame is "
                f"'{self.world_frame}'. A frame transform is required.",
                throttle_duration_sec=2.0,
            )
            return
        if not self.accept_cloud_once(cloud_msg):
            return

        if not detections:
            return

        pts_cloud = self.pointcloud_to_xyz(cloud_msg)
        if pts_cloud.shape[0] == 0:
            self.get_logger().warn("cloud has zero valid points", throttle_duration_sec=2.0)
            return

        pts_lidar = self.cloud_points_to_lidar(pts_cloud)
        uv, valid = self.project_lidar_points(pts_lidar)

        self.publish_projection_debug_image(
            img,
            detections,
            uv,
            valid,
            msg,
        )

        points_world = []
        for det in detections:
            p_lidar = self.localize_sep_in_lidar(det, pts_lidar, uv, valid)
            if p_lidar is None:
                self.get_logger().warn(
                    f"orange detected but no nearby projected cloud points: "
                    f"u={det.u:.1f}, v={det.v:.1f}, search_radius_px={self.search_radius_px}",
                    throttle_duration_sec=2.0,
                )
                continue

            p_world = self.lidar_to_world(p_lidar, odom_msg)
            if not np.isfinite(p_world).all():
                continue

            obs = PointStamped()
            obs.header.stamp = cloud_msg.header.stamp
            obs.header.frame_id = self.world_frame
            obs.point.x = float(p_world[0])
            obs.point.y = float(p_world[1])
            obs.point.z = float(p_world[2])
            self.obs_pub.publish(obs)

            self.obs_count += 1
            points_world.append(p_world)

        if points_world:
            self.publish_observation_marker(points_world, cloud_msg.header.stamp)
            self.get_logger().info(
                f"orange frame={self.frame_count}, detections={len(detections)}, "
                f"observations={len(points_world)}, "
                f"dt_image_cloud={image_cloud_dt*1000.0:.1f}ms, "
                f"dt_cloud_odom={cloud_odom_dt*1000.0:.1f}ms, "
                f"total_obs={self.obs_count}",
                throttle_duration_sec=1.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorSepLocalizer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
