# FAST-LIVO2 同步数据出口修改说明

本功能包默认 FAST-LIVO2 已经输出以下同步 topic：

```text
/fastlivo/semantic_sync/image
/fastlivo/semantic_sync/cloud_lidar
/fastlivo/semantic_sync/cloud_body
/fastlivo/semantic_sync/cloud_world
/fastlivo/semantic_sync/odom
```

其中 `image / cloud_lidar / odom` 必须使用同一个 `header.stamp`。苗点定位节点会用这三个 topic 做 2D SEP 到 3D map 点的关联。

## 为什么不要直接查全局地图

不要这样做：

```text
YOLO 当前图像 -> 去全局点云地图里找最近点
```

原因：全局地图是多时刻累积结果，和当前图像不严格同步；直接查全局地图容易把苗点投到旧点、重复点或漂移后的点上。

推荐这样做：

```text
当前同步图像
当前同步局部点云 cloud_lidar
当前同步里程计 odom
        ↓
SEP 像素 -> 当前局部点云 -> P_lidar -> P_body -> P_map
        ↓
seedling_mapper 做多帧地图融合
```

## 坐标链

LiDAR 点投影到图像：

```text
P_cam = Rcl * P_lidar + Pcl
u = fx * X_cam / Z_cam + cx
v = fy * Y_cam / Z_cam + cy
```

LiDAR 点转 body：

```text
P_body = extR * P_lidar + extT
```

body 点转 map：

```text
P_map = R_map_body * P_body + t_map_body
```

## FAST-LIVO2 内部建议发布位置

优先在 `handleVIO()` 处理完一帧图像后发布同步数据，因为苗点来自图像，应该以图像帧为主。

建议在 `initializeSubscribersAndPublishers()` 里创建 publisher：

```cpp
auto sync_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

pubSemanticSyncImage =
    this->node->create_publisher<sensor_msgs::msg::Image>(
        "/fastlivo/semantic_sync/image", sync_qos);

pubSemanticSyncCloudLidar =
    this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/fastlivo/semantic_sync/cloud_lidar", sync_qos);

pubSemanticSyncCloudBody =
    this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/fastlivo/semantic_sync/cloud_body", sync_qos);

pubSemanticSyncCloudWorld =
    this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/fastlivo/semantic_sync/cloud_world", sync_qos);

pubSemanticSyncOdom =
    this->node->create_publisher<nav_msgs::msg::Odometry>(
        "/fastlivo/semantic_sync/odom", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
```

并在每次 VIO 图像帧处理完成后调用同步发布函数。

## 检查时间戳

启动 FAST-LIVO2 后检查：

```bash
ros2 topic echo /fastlivo/semantic_sync/image --field header.stamp --once
ros2 topic echo /fastlivo/semantic_sync/cloud_lidar --field header.stamp --once
ros2 topic echo /fastlivo/semantic_sync/odom --field header.stamp --once
```

三者应该完全相同或极其接近。若相差超过 `max_sync_dt`，`yolo_sep_localizer` 会丢弃该帧。
