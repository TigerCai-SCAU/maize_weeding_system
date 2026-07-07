# weedarm_row_planner_test

这个包用于测试“沿行绕苗 + Z 恒定耕深”。

它比之前的 fake_seedling_planner 更接近实际逻辑：

- X 只用于根据车辆前进预测未来位置；
- Y 根据苗点生成平滑绕苗轨迹；
- Z 根据地形高度 `ground_z(x,y)` 和耕深 `work_depth` 计算；
- H5U 仍然只接收未来 64 点 `tool_y/tool_z`。

## 编译

```bash
cd ~/weed_ws/src
unzip ~/Downloads/weedarm_row_planner_test.zip
cd ~/weed_ws
colcon build --packages-select weedarm_row_planner_test
source install/setup.bash
```

也可以和桥接包一起编译：

```bash
colcon build --packages-select weedarm_h5u_bridge weedarm_row_planner_test
source install/setup.bash
```

## 运行

终端 1：启动 H5U 桥接：

```bash
ros2 launch weedarm_h5u_bridge h5u_bridge.launch.py plc_ip:=192.168.1.88
```

终端 2：启动行绕苗规划测试：

```bash
ros2 launch weedarm_row_planner_test row_seedling_planner_test.launch.py \
  vehicle_speed:=0.05 safe_dist:=0.04 ground_z0:=-0.13 work_depth:=0.02
```

更接近你代码里的 8 cm 安全距离：

```bash
ros2 launch weedarm_row_planner_test row_seedling_planner_test.launch.py \
  vehicle_speed:=0.05 safe_dist:=0.08 ground_z0:=-0.13 work_depth:=0.02
```

测试地形起伏导致的 Z 恒深控制：

```bash
ros2 launch weedarm_row_planner_test row_seedling_planner_test.launch.py \
  vehicle_speed:=0.05 safe_dist:=0.04 ground_z0:=-0.13 work_depth:=0.02 terrain_wave_amp:=0.01
```

## 输出话题

- `/weedarm/trajectory_yz`：发给 H5U 桥接节点
- `/weedarm/test_seedlings_arm`：模拟聚合后苗点
- `/weedarm/row_path_preview`：完整行间绕苗路径预览

## 查看

```bash
ros2 topic echo /weedarm/trajectory_yz
ros2 topic echo /weedarm/tool_yz_feedback
ros2 topic echo /weedarm/diagnostics
```
