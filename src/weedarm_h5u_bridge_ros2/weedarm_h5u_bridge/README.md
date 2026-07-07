# weedarm_h5u_bridge

ROS2 bridge for H5U + YKB200 weed arm CSP trajectory buffering.

## Interface

Subscribe:

- `/weedarm/trajectory_yz` (`trajectory_msgs/msg/JointTrajectory`)
  - `joint_names = ["tool_y", "tool_z"]`
  - `positions[0] = tool_y` in meters
  - `positions[1] = tool_z` in meters, upward positive, downward negative

Publish:

- `/weedarm/joint_state_feedback` (`sensor_msgs/JointState`) pitch/yaw in rad
- `/weedarm/tool_yz_feedback` (`geometry_msgs/PointStamped`) arm-base Y/Z in meters
- `/weedarm/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

The upstream ROS node should already compensate vehicle pose, ground height, seedling map, and avoidance path. This bridge only sends local arm-base `Y/Z` to H5U.

## Build

```bash
cd ~/weed_ws/src
unzip ~/Downloads/weedarm_h5u_bridge_ros2.zip
cd ~/weed_ws
python3 -m pip install pymodbus
colcon build --packages-select weedarm_h5u_bridge
source install/setup.bash
```

## Run bridge

```bash
ros2 launch weedarm_h5u_bridge h5u_bridge.launch.py plc_ip:=192.168.1.88
```

PLC side must already be prepared:

- `MAIN_H5U_WeedArm_CSPTrack_v3_safe_start.st`
- low-public variable table
- MAIN fixed scan period = 4 ms
- `Cmd_ServoOn = ON`
- `Home_Done = ON`
- `Guide_Ready = ON`
- `PC_Enable = ON`
- `Cmd_SweepStart = ON`

## Run test trajectory

Small safe synchronized Y/Z test:

```bash
ros2 launch weedarm_h5u_bridge test_sync_traj.launch.py amp_y:=0.01 amp_z:=0.01 z_center:=-0.12 period:=4.0
```

Larger synchronized Y/Z motion:

```bash
ros2 launch weedarm_h5u_bridge test_sync_traj.launch.py amp_y:=0.03 amp_z:=0.02 z_center:=-0.15 period:=5.0
```
