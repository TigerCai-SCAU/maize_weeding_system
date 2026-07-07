# seedling_semantic_mapping

这个版本改成**外部同步方案**：FAST-LIVO2 只负责稳定定位建图，苗识别、2D→3D 定位和苗点地图都放在独立 ROS2 节点里。

推荐输入：

```text
/left_camera/image_10hz              彩色图，YOLO 用
/fastlivo/deskew/cloud_body          FAST-LIVO 轻量输出的当前帧去畸变 body 点云
/aft_mapped_to_init                  FAST-LIVO 里程计 T_map_body
```

输出：

```text
/seedling/observation_point_map       单帧 SEP 3D 观测，map 坐标
/seedling/current_observation_markers 当前帧观测 RViz marker
/seedling/map_markers                 融合后的苗点地图 RViz marker
/tmp/seedling_map_confirmed.csv       confirmed 苗点 CSV
```

---

## 1. 为什么改成外部同步

不建议继续在 FAST-LIVO 主线程里发布彩图、大点云、world 点云。那会增加 `cv_bridge`、图像拷贝、`pcl::toROSMsg` 和坐标转换负担，容易让 FAST-LIVO 卡顿。

现在只要求 FAST-LIVO 额外轻量发布一个去畸变点云：

```text
/fastlivo/deskew/cloud_body
```

这个点云是 FAST-LIVO 内部 IMU 去畸变后的当前帧点云，后端苗点节点再按时间戳匹配彩图和里程计。

---

## 2. 节点说明

### 2.1 `yolo_sep_localizer`

流程：

```text
彩色图像 t_img
  ↓
YOLO pose 检测 SEP 像素
  ↓
缓存里找最近 cloud_body 和 odom
  ↓
cloud_body -> cloud_lidar
  ↓
P_cam = Rcl * P_lidar + Pcl 投影到图像
  ↓
SEP 附近找点 / 平面拟合 / 射线求交
  ↓
P_lidar -> P_body -> P_map
  ↓
发布 /seedling/observation_point_map
```

关键参数：

```yaml
image_topic: /left_camera/image_10hz
cloud_topic: /fastlivo/deskew/cloud_body
cloud_frame: body
odom_topic: /aft_mapped_to_init
max_image_cloud_dt: 0.08
max_image_odom_dt: 0.08
intrinsic_scale: 1.0
```

如果你的彩图是 1280x1024，`intrinsic_scale: 1.0`。如果你发布的是 640x512 的彩图，设成 `0.5`。

### 2.2 `seedling_mapper`

接收 `/seedling/observation_point_map`，按距离门限融合成苗点地图：

```yaml
gate_xy: 0.12
gate_z: 0.25
confirm_hits: 2
```

跑通后可以收紧到：

```yaml
gate_xy: 0.08~0.10
gate_z: 0.15~0.20
confirm_hits: 3
```

---

## 3. 安装

```bash
cd ~/fast_ws/src
unzip ~/Downloads/seedling_semantic_mapping_external_sync.zip

cd ~/fast_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select seedling_semantic_mapping
source install/setup.bash
```

编辑配置：

```bash
gedit ~/fast_ws/src/seedling_semantic_mapping/config/seedling_pipeline.yaml
```

最少要改：

```yaml
model_path: "/home/nvidia/models/你的苗点pose模型.pt"
image_topic: /left_camera/image_10hz
cloud_topic: /fastlivo/deskew/cloud_body
cloud_frame: body
odom_topic: /aft_mapped_to_init
intrinsic_scale: 1.0
```

---

## 4. 运行前检查

```bash
ros2 topic hz /left_camera/image_10hz
ros2 topic hz /fastlivo/deskew/cloud_body --qos-reliability best_effort
ros2 topic hz /aft_mapped_to_init
```

检查时间戳是否在同一时间基准：

```bash
ros2 topic echo /left_camera/image_10hz --field header.stamp --once
ros2 topic echo /fastlivo/deskew/cloud_body --field header.stamp --qos-reliability best_effort --once
ros2 topic echo /aft_mapped_to_init --field header.stamp --once
```

三个 `sec` 应该接近。如果相差几百秒或时间基准不同，先不要入图。

---

## 5. 启动

```bash
ros2 launch seedling_semantic_mapping seedling_pipeline.launch.py
```

看单帧观测：

```bash
ros2 topic echo /seedling/observation_point_map --once
```

看苗点地图：

```bash
cat /tmp/seedling_map_confirmed.csv
```

RViz 添加：

```text
MarkerArray: /seedling/current_observation_markers
MarkerArray: /seedling/map_markers
```

---

## 6. 如果没有观测

先看日志里的同步差：

```text
Waiting sync data: cloud=missing dt=xxxms, odom=missing dt=xxxms
```

如果 dt 经常超过 80 ms，先把参数放宽验证：

```yaml
max_image_cloud_dt: 0.12
max_image_odom_dt: 0.12
```

如果 YOLO 有检测但没有 3D 点：

```yaml
search_radius_px: 18.0
min_candidate_points: 6
```

如果苗点重复成多个 ID：

```yaml
gate_xy: 0.12~0.15
gate_z: 0.25
```

---

## 在线苗点输出给路径规划

新版 `seedling_mapper` 会额外发布：

```text
/seedling/map_points    geometry_msgs/msg/PoseArray
```

这个 topic 只发布已经 `confirmed` 的苗点，坐标系是 `map`。每个 `Pose.position` 是一株苗的融合位置，`orientation.w=1.0` 只是占位。后面的机械臂路径规划节点应该订阅这个 topic，而不是实时读取 CSV。

查看：

```bash
ros2 topic echo /seedling/map_points --once
ros2 topic hz /seedling/map_points
```

CSV 仍然保留在 `/tmp/seedling_map_confirmed.csv`，主要用于实验记录、调试和论文数据分析。
