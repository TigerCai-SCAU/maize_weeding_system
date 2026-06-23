# fdilink_gnss_bridge

This package keeps the FDILink vendor driver unchanged and completes its odometry messages for calibration/fusion.

Assumption:

- `/NED_odometry` from the FDILink driver is already lever-arm compensated by the INS and represents the INS IMU origin.
- The bridge does not apply any extra antenna-to-IMU or INS-to-target lever arm.

Inputs:

- `/NED_odometry` (`nav_msgs/msg/Odometry`): primary INS/GNSS local NED trajectory.
- `/imu` (`sensor_msgs/msg/Imu`): used to fill missing odometry orientation and angular velocity.
- `/gps/fix` (`sensor_msgs/msg/NavSatFix`): used to fill GNSS fix metadata and odometry covariance.

Outputs:

- `/gnss/fix` (`sensor_msgs/msg/NavSatFix`): cleaned fix topic with frame id/status/covariance.
- `/ins/odom_ned` (`nav_msgs/msg/Odometry`): completed NED odometry at the INS IMU origin.
- `/gnss/odom` (`nav_msgs/msg/Odometry`): ENU version of `/ins/odom_ned`.
- `/gnss/odom_gated` (`nav_msgs/msg/Odometry`): optional quality-gated ENU odometry.

NED to ENU conversion:

- `x_enu = East`
- `y_enu = North`
- `z_enu = -Down`

Build:

```bash
cd ~/your_ros2_ws
colcon build --packages-select fdilink_gnss_bridge
source install/setup.bash
```

Run after starting the FDILink driver:

```bash
ros2 launch fdilink_gnss_bridge gnss_bridge.launch.py
```

For trajectory calibration against Fast-LIVO2, record either the raw completed NED output or the ENU output:

```bash
ros2 bag record /aft_mapped_to_init /ins/odom_ned /gnss/odom /gnss/odom_gated /gps/fix /imu -o calib_traj_bag
```

Use `/ins/odom_ned` when you want to preserve the FDILink native NED trajectory. Use `/gnss/odom` or `/gnss/odom_gated` when downstream code expects ROS-style ENU coordinates.
