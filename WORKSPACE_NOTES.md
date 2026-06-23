# ROS2 workspace notes

Packages under `src/`:

- `fast_livo` from `FAST-LIVO2-ros2`
- `fdilink_ahrs` from `FDILink_ROS2`
- `fdilink_gnss_bridge`
- `livox_ros_driver2`
- `miivii_gmsl_camera`
- `mvs_ros2_pkg`
- `fast_calib`
- `weed_bringup`
- `vikit_common`
- `vikit_ros`

AGX integration notes:

- Runtime-tested AGX configs were merged from `src.zip`.
- `FAST-LIVO2-ros2/config/mid360.yaml` and `FAST-LIVO2-ros2/launch/mapping_mid360.launch.py` were added.
- `FDILink_ROS2/launch/ahrs_driver.launch.py` uses `/dev/ttyUART_485_422_A`, matching the AGX debug setup.
- `FDILink_ROS2` publishes `NavSatFix` covariance from FDILink `hAcc/vAcc` when available.
- `fdilink_gnss_bridge` publishes raw `/gnss/odom`, `/ins/odom` with IMU orientation, and gated `/gnss/odom_gated`; use the gated topic for fusion and `/ins/odom` for AX=XB calibration.
- `fast_livo` has an optional soft GNSS position constraint. It is disabled by default and can be enabled with `enable_gnss_constraint:=true`.
- `fast_livo` also has an optional wheel velocity constraint from `/wheel/odom.twist.twist.linear.x`, enabled with `enable_wheel_constraint:=true`.
- `weed_calibration_tools` exports odometry topics to TUM format and estimates GNSS-to-Fast-LIVO2 trajectory alignment.
- `calib_axxb` is a ROS 2 wrapper around `LiangHongY/calib_AXXB` for Ceres-based `AX=XB` LiDAR/RTK trajectory extrinsic calibration.
- `livox_ros_driver2/config/MID360_config.json` uses the AGX network settings.
- The AGX Fast-LIVO2 algorithm source changes under `src/` and `include/` were not merged, because they included temporary debug changes.

Build on AGX Orin:

```bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

If you are not using ROS 2 Humble, replace `humble` with your ROS 2 distro name.

Livox driver 2 also requires the Livox-SDK2 shared library and headers installed under `/usr/local/lib` and a standard include path. If `colcon build` fails on `liblivox_lidar_sdk_shared.so` or `livox_lidar_api.h`, install Livox-SDK2 on the AGX first.

Typical bringup:

```bash
ros2 launch weed_bringup weed_system.launch.py
```

Manual bringup:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fdilink_ahrs ahrs_driver.launch.py
ros2 launch fdilink_gnss_bridge gnss_bridge.launch.py
ros2 launch fast_livo mapping_mid360.launch.py
```

Bringup options:

```bash
ros2 launch weed_bringup weed_system.launch.py use_rviz:=true
ros2 launch weed_bringup weed_system.launch.py enable_gnss_constraint:=true
ros2 launch weed_bringup weed_system.launch.py enable_wheel_constraint:=true
ros2 launch weed_bringup weed_system.launch.py start_fast_livo:=false
ros2 launch weed_bringup weed_system.launch.py start_camera:=false
ros2 launch weed_bringup weed_system.launch.py start_livox:=false start_camera:=false start_fast_livo:=false
ros2 launch weed_bringup weed_system.launch.py camera_output:=screen camera_log_level:=info camera_verbose:=true
```

Offline trajectory alignment:

```bash
ros2 run weed_calibration_tools export_odom_tum --bag replay_baseline --topic /aft_mapped_to_init --out /tmp/fastlivo.tum --stamp-source bag
ros2 run weed_calibration_tools export_odom_tum --bag replay_rtk --topic /gnss/odom_gated --out /tmp/gnss.tum --stamp-source bag
ros2 run weed_calibration_tools align_trajectories --source /tmp/gnss.tum --target /tmp/fastlivo.tum --max-dt 0.05 --out /tmp/T_gnss_to_fastlivo.txt
ros2 run weed_calibration_tools calib_axxb --traj-a /tmp/gnss.tum --traj-b /tmp/fastlivo.tum --max-dt 0.05 --skip 10 --out /tmp/T_axxb_gnss_fastlivo.txt
ros2 run calib_axxb calib_axxb_lidar_rtk /tmp/calib_fastlivo_rtk.yaml
```
