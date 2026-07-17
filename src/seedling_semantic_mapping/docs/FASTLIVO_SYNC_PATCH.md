# FAST-LIVO2 与 terrain-ray 的接口说明

当前 terrain-ray 方案不再需要 FAST-LIVO2 发布
`/fastlivo/semantic_sync/*`，也不需要在检测像素附近查询当前帧 LiDAR 点。
不要为苗点定位继续修改 FAST-LIVO 的图像或点云主线程。

## 所需接口

```text
/aft_mapped_to_init               nav_msgs/Odometry
/cloud_registered                sensor_msgs/PointCloud2
/ground/global_elevation_points  sensor_msgs/PointCloud2
```

- `/aft_mapped_to_init` 由 FAST-LIVO 发布，terrain-ray 用它插值图像曝光时刻的
  `camera_init -> body` 位姿。
- `/cloud_registered` 由 `ground_mapper` 使用，用于维护持久 2.5D 地形。
- `/ground/global_elevation_points` 由 `ground_mapper` 发布，苗点定位只与
  这个地形求交。

苗点输出仍是：

```text
/seedling/observation_point_map
```

`seedling_mapper` 的订阅和输出接口没有变化。

## 时间关系

```text
t_query = t_image + image_time_offset_sec
```

`yolo_sep_localizer` 在缓存中查找 `t_query` 前后的两帧
`/aft_mapped_to_init`，平移使用线性插值，旋转使用四元数 SLERP。没有包围
位姿时拒绝该图像，不使用最近邻位姿冒充同步结果。

当前配置：

```yaml
image_time_offset_sec: 0.10
max_odom_bracket_gap_sec: 0.25
```

这两个值都在 `seedling_pipeline.yaml` 中，后续用实车 rosbag 标定。

## 坐标链

检测像素先根据相机内参与畸变参数转换为相机射线，再使用已有外参：

```text
P_cam = Rcl * P_lidar + Pcl
P_body = extR * P_lidar + extT
```

反求相机原点和射线方向到 body，再用插值后的 FAST-LIVO 位姿变换到
`camera_init`。射线最终与同一 `camera_init` 下的 2.5D 地形求交。

## 启动前检查

```bash
chronyc waitsync 120 0.01 0 1

ros2 topic echo /aft_mapped_to_init --field header --once
ros2 topic echo /cloud_registered --field header --once
ros2 topic echo /ground/global_elevation_points --field header \
  --qos-reliability best_effort --once
```

要求：

- 米文时间已经锁定，不能在 FAST-LIVO 运行中发生系统时间大跳变；
- 三个消息属于同一时间域；
- odometry 和地形的 `frame_id` 均为 `camera_init`；
- 地形持续更新且包含相机射线落点附近的地面覆盖。

## 旧方案

以下 topic 属于旧的像素邻域点云方案，当前实现不依赖它们：

```text
/fastlivo/semantic_sync/image
/fastlivo/semantic_sync/cloud_lidar
/fastlivo/semantic_sync/cloud_body
/fastlivo/semantic_sync/cloud_world
/fastlivo/semantic_sync/odom
```

`mid360.yaml` 中若仍保留 `semantic_sync.*` 参数，它们不构成 terrain-ray
的运行条件。后续整理 FAST-LIVO 版本时可以删除，但无需为本功能补实现。
