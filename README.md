# 玉米除草机器人系统

这是玉米田间除草机器人项目的 ROS 2 工作空间源码快照，主要用于苗点检测与三维定位、苗点地图构建、地面高度提取，以及后续除草机械臂路径规划和执行控制实验。

## 1. 仓库结构

```text
src/
  FAST-LIVO2/                    # FAST-LIVO2，已轻量修改，用于发布 /fastlivo/deskew/cloud_body
  livox_ros_driver2/             # Livox MID360 的 ROS 2 驱动
  miivii_gmsl_camera/            # 米文 GMSL 相机驱动
  seedling_semantic_mapping/     # YOLO 出苗点检测 + 三维苗点定位 + 苗点地图
  ground_mapper/                 # 地面点 / 非地面点提取与地面高度点发布
  weedarm_h5u_bridge_ros2/       # 汇川 H5U 通信与轨迹下发桥接节点
  weedarm_row_planner_test/      # 行间 / 绕苗路径规划测试节点
```

## 2. 系统主流程

整体运行思路如下：

```text
Livox MID360 + GMSL 相机 + IMU
        ↓
FAST-LIVO2 定位建图
        ↓
/fastlivo/deskew/cloud_body + 彩色图像 + /aft_mapped_to_init
        ↓
seedling_semantic_mapping 生成三维苗点地图
        ↓
/seedling/map_points 给路径规划使用

/cloud_registered
        ↓
ground_mapper 提取地面高度
        ↓
/ground/elevation_points 给地面仿形和入土深度控制使用
```

## 3. 主要话题

### 3.1 FAST-LIVO2 输出

```text
/aft_mapped_to_init              # FAST-LIVO2 位姿 / 里程计
/cloud_registered                # 注册到地图系的点云
/fastlivo/deskew/cloud_body      # 当前帧去畸变 body/IMU 坐标点云
```

### 3.2 苗点检测与建图

输入：

```text
/left_camera/image_10hz
/fastlivo/deskew/cloud_body
/aft_mapped_to_init
```

输出：

```text
/seedling/observation_point_map          # 单次出苗点三维观测
/seedling/map_points                     # 融合后的苗点 PoseArray，路径规划主要用这个
/seedling/map_markers                    # RViz 苗点可视化
/tmp/seedling_map_confirmed.csv          # confirmed 苗点记录
```

### 3.3 地面高度提取

输入：

```text
/cloud_registered
/aft_mapped_to_init
```

输出：

```text
/ground/points                   # 地面点
/ground/non_ground_points        # 非地面点，例如作物、杂草、离群高点
/ground/elevation_points         # 栅格地面高度点
/ground/elevation_markers        # RViz 高度点可视化
```

## 4. 编译方法

进入仓库：

```bash
cd ~/maize_weeding_system
source /opt/ros/humble/setup.bash
```

### 4.1 单独编译 Livox ROS2 驱动

`livox_ros_driver2` 需要带 ROS 2 编译参数：

```bash
colcon build --symlink-install --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

### 4.2 编译其余包

```bash
source install/setup.bash

mkdir -p src/rpg_vikit/vikit_common/lib
mkdir -p src/rpg_vikit/vikit_common/bin
touch src/rpg_vikit/vikit_common/lib/.gitkeep
touch src/rpg_vikit/vikit_common/bin/.gitkeep

colcon build --symlink-install --packages-skip livox_ros_driver2
source install/setup.bash
```

### 4.3 只验证苗点建图和地面提取

```bash
colcon build --symlink-install --packages-select seedling_semantic_mapping ground_mapper
source install/setup.bash
```

## 5. 苗点检测与建图

运行前先修改配置文件：

```bash
gedit src/seedling_semantic_mapping/config/seedling_pipeline.yaml
```

重点检查下面几个参数：

```yaml
image_topic: /left_camera/image_10hz
cloud_topic: /fastlivo/deskew/cloud_body
cloud_frame: body
odom_topic: /aft_mapped_to_init
model_path: "/home/nvidia/models/your_maize_sep_pose_model.pt"
```

如果实际彩色图像话题不是 `/left_camera/image_10hz`，例如是 `/miivii_gmsl/image3`，需要把 `image_topic` 改成实际话题。

启动：

```bash
ros2 launch seedling_semantic_mapping seedling_pipeline.launch.py
```

检查输出：

```bash
ros2 topic echo /seedling/observation_point_map --once
ros2 topic echo /seedling/map_points --once
cat /tmp/seedling_map_confirmed.csv
```

## 6. 地面高度提取

启动：

```bash
ros2 launch ground_mapper ground_mapper.launch.py
```

检查话题频率：

```bash
ros2 topic hz /ground/points
ros2 topic hz /ground/non_ground_points
ros2 topic hz /ground/elevation_points
```

不要直接 `echo` 完整点云，可以只看点云宽度：

```bash
ros2 topic echo /ground/points --field width --qos-reliability best_effort --once
ros2 topic echo /ground/non_ground_points --field width --qos-reliability best_effort --once
ros2 topic echo /ground/elevation_points --field width --qos-reliability best_effort --once
```

## 7. RViz 建议显示

建议添加下面几个显示项：

```text
PointCloud2: /cloud_registered
PointCloud2: /fastlivo/deskew/cloud_body
PointCloud2: /ground/points
PointCloud2: /ground/non_ground_points
PointCloud2: /ground/elevation_points
MarkerArray: /ground/elevation_markers
MarkerArray: /seedling/map_markers
PoseArray: /seedling/map_points
```

## 8. Git 提交注意事项

不要提交下面这些文件到仓库：

```text
build/
install/
log/
*.bag
*.db3
*.mcap
*.pt
*.onnx
*.engine
*.pcd
*.ply
*.csv
*.so
```

`src/rpg_vikit/vikit_common/bin/` 和 `src/rpg_vikit/vikit_common/lib/` 目录需要存在，否则 `vikit_common` 编译会失败；但是里面生成的测试程序和 `.so` 文件不应该提交，只保留 `.gitkeep` 即可。

## 9. 当前开发状态

当前版本重点是先跑通：

1. FAST-LIVO2 稳定输出位姿、地图点云和当前帧去畸变 body 点云。
2. YOLO 检测出苗点，并通过点云投影得到三维苗点。
3. 多帧苗点观测融合成 `/seedling/map_points`。
4. 从 `/cloud_registered` 中提取地面高度点 `/ground/elevation_points`。
5. 后续再接绕苗路径规划、地面仿形和 H5U 伺服执行。

## 10. 后续计划

后续建议继续完善：

1. 将 `ground_mapper` 从单帧地面提取升级为 rolling elevation map。
2. 让路径规划节点同时订阅 `/seedling/map_points` 和 `/ground/elevation_points`。
3. 根据地面高度计算刀具目标深度，例如保持地面以下约 2 cm。
4. 将规划轨迹通过 H5U / Modbus 下发给执行机构。
5. 增加实验记录脚本，自动保存苗点地图、地面高度图和执行轨迹。
