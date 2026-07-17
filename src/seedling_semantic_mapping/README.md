# seedling_semantic_mapping

玉米苗检测与地图构建包。当前 3D 定位使用 **terrain-ray**：复用 YOLO
Pose（或颜色检测）、相机标定和 FAST-LIVO 里程计，将检测像素对应的相机射线
与 `ground_mapper` 发布的持久 2.5D 地形求交。

定位不再要求检测像素附近存在 LiDAR 回波，也不需要修改 FAST-LIVO 主线程来
发布语义同步点云。

## 数据接口

输入：

```text
/miivii_gmsl/image3               sensor_msgs/Image，原始 2880x1860 彩图
/aft_mapped_to_init               nav_msgs/Odometry，FAST-LIVO 位姿
/ground/global_elevation_points   sensor_msgs/PointCloud2，持久 2.5D 地形
```

输出接口保持不变：

```text
/seedling/observation_point_map       geometry_msgs/PointStamped，单帧苗点观测
/seedling/current_observation_markers visualization_msgs/MarkerArray
/seedling/map_markers                 visualization_msgs/MarkerArray
/seedling/map_points                  geometry_msgs/PoseArray，confirmed 苗点
/tmp/seedling_map_confirmed.csv       confirmed 苗点记录
```

以上地图数据默认都在 `camera_init` 坐标系。

## 定位流程

```text
图像时间 t_image
  -> t_pose = t_image + image_time_offset_sec
  -> 查找前后两帧 /aft_mapped_to_init
  -> 平移线性插值 + 姿态 SLERP
  -> YOLO/颜色检测得到像素
  -> 相机去畸变射线
  -> 相机/LiDAR/IMU 外参变换到 camera_init
  -> 与 /ground/global_elevation_points 求第一个有效交点
  -> 发布 /seedling/observation_point_map
  -> seedling_mapper 多帧融合
```

没有前后包围位姿、地形过期、射线经过地形空洞或没有零交叉时，该次检测会被
拒绝，不会回退到像素附近点云查询。

## 关键参数

全部运行参数位于 `config/seedling_pipeline.yaml`：

```yaml
image_topic: /miivii_gmsl/image3
odom_topic: /aft_mapped_to_init
terrain_topic: /ground/global_elevation_points
world_frame: camera_init

image_time_offset_sec: 0.10
max_odom_bracket_gap_sec: 0.25
max_terrain_age_sec: 3.0

terrain_resolution_m: 0.05
terrain_vertical_axis: 0
terrain_horizontal_axes: [1, 2]
terrain_lookup_radius_m: 0.08

ray_min_range_m: 0.10
ray_max_range_m: 8.0
ray_step_m: 0.025
ray_height_tolerance_m: 0.02
```

`image_time_offset_sec=0.10` 是当前待实车验证值，不是永久标定结果。米文硬件
授时后，如果 ROS 图像头使用曝光触发时间，该值可能应接近 0；必须通过 bag
回放比较后再调整。

相机内参对应原始 `2880x1860 /miivii_gmsl/image3`。只有输入图像按相同比例
缩放时才修改 `intrinsic_scale`。

## 构建与纯函数测试

```bash
cd ~/maize_weeding_system
source /opt/ros/humble/setup.bash

python3 -m py_compile \
  src/seedling_semantic_mapping/seedling_semantic_mapping/terrain_ray.py \
  src/seedling_semantic_mapping/seedling_semantic_mapping/yolo_sep_localizer.py \
  src/seedling_semantic_mapping/launch/seedling_pipeline.launch.py

python3 -m pytest -q src/seedling_semantic_mapping/test/test_terrain_ray.py

colcon build \
  --packages-select seedling_semantic_mapping \
  --symlink-install
```

## 启动

先确认米文已经锁定 RTK 时间：

```bash
chronyc waitsync 120 0.01 0 1
chronyc sources -v
```

`PPS` 应显示 `#*`。然后启动 FAST-LIVO、`ground_mapper`，最后启动苗点
管线：

```bash
source ~/maize_weeding_system/install/setup.bash

ros2 launch ground_mapper ground_mapper.launch.py
ros2 launch seedling_semantic_mapping seedling_pipeline.launch.py \
  config_file:=~/maize_weeding_system/src/seedling_semantic_mapping/config/seedling_pipeline.yaml
```

YOLO 模型配置为：

```yaml
model_path: /home/nvidia/maize_models/maize_pose_best.pt
det_conf: 0.35
kp_conf: 0.30
```

室内可以先用橙色目标检查完整 terrain-ray 通路：

```bash
ros2 launch seedling_semantic_mapping seedling_pipeline.launch.py \
  localizer_executable:=color_sep_localizer
```

颜色模式查看 `/seedling/orange_mask` 和最终 observation；旧的
`/seedling/projection_debug` 是点云邻域方案遗留调试输出，不作为
terrain-ray 的通过条件。

## 运行检查

```bash
ros2 topic hz /miivii_gmsl/image3
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /ground/global_elevation_points --qos-reliability best_effort

ros2 topic echo /miivii_gmsl/image3 --field header.stamp --once
ros2 topic echo /aft_mapped_to_init --field header.stamp --once
ros2 topic echo /ground/global_elevation_points --field header.stamp \
  --qos-reliability best_effort --once

ros2 topic echo /seedling/observation_point_map --once
ros2 topic echo /seedling/map_points --once
```

三个输入的 `sec` 必须属于同一时间域，图像的 `t+0.1s` 必须能被前后
FAST-LIVO 位姿包围。若 observation 为空，按顺序检查：

1. YOLO/颜色 mask 是否确实检测到目标；
2. 图像与里程计是否有包围位姿；
3. 地形消息 frame 是否为 `camera_init` 且没有过期；
4. 相机射线是否朝向已观测地形范围；
5. 再调整 `image_time_offset_sec`、地形查询半径和射线容差。

不要通过增大像素点云搜索半径来“修好” terrain-ray；当前算法不使用该路径。

## 雨停后的分阶段验证

1. 静止 30 秒：确认时间戳、frame 和地形持续更新，无苗时不得产生随机观测。
2. 橙色目标：在 1、2、3 米各放一次，检查射线交点随地面起伏变化。
3. 单株玉米苗：只检查单帧 observation，暂不启用 confirmed 过滤。
4. 直行 3～5 米重复观测同一苗：检查是否融合成同一 landmark。
5. 最后开启多株建图并记录 rosbag，对 `image_time_offset_sec` 做离线扫描。

建议记录：

```bash
ros2 bag record \
  /miivii_gmsl/image3 \
  /aft_mapped_to_init \
  /ground/global_elevation_points \
  /seedling/observation_point_map \
  /seedling/map_points
```
