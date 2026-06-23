# calib_axxb ROS 2 package

This package wraps the original `calib_AXXB` C++/Ceres hand-eye calibration tool as an `ament_cmake` ROS 2 package.

## Dependencies

Install the system dependencies in your ROS 2 environment:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ament-cmake libeigen3-dev libyaml-cpp-dev libceres-dev
```

## Build

From the workspace root:

```bash
colcon build --packages-select calib_axxb
source install/setup.bash
```

You can also run the package helper:

```bash
src/calib_axxb/build.sh
```

## Run

The input trajectory format is:

```text
timestamp x y z qx qy qz qw
```

Run the bundled example config from the workspace root:

```bash
ros2 run calib_axxb calib_axxb_lidar_rtk src/calib_axxb/config/calib.yaml
```

For your own data, copy `config/calib.yaml`, set absolute paths for `A_poses_file`, `B_poses_file`, and `save_result_file`, then run:

```bash
ros2 run calib_axxb calib_axxb_lidar_rtk /path/to/calib.yaml
```

For RTK/INS plus Fast-LIVO2 trajectories, start with `calibration_mode: "planar"`. Keep `A_poses_file` as the Fast-LIVO2/LiDAR trajectory and `B_poses_file` as the RTK/INS trajectory. The reported `T_A_B` is then the 2D transform from RTK/INS coordinates into the Fast-LIVO2 map coordinates.
