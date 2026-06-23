# weed_bringup

One-command bringup for the weeding robot localization stack.

Default startup:

```bash
ros2 launch weed_bringup weed_system.launch.py
```

Useful options:

```bash
ros2 launch weed_bringup weed_system.launch.py use_rviz:=true
ros2 launch weed_bringup weed_system.launch.py start_fast_livo:=false
ros2 launch weed_bringup weed_system.launch.py start_camera:=false
ros2 launch weed_bringup weed_system.launch.py start_livox:=false
ros2 launch weed_bringup weed_system.launch.py camera_output:=screen camera_log_level:=info camera_verbose:=true
```

Shortcut script:

```bash
bash src/weed_bringup/scripts/start_weed_system.sh
```

Calibration recording:

```bash
ros2 launch weed_bringup calib_record.launch.py bag_name:=calib_traj_bag
```

This starts the normal sensor/RTK/Fast-LIVO2 stack with GNSS fusion disabled, then records:

```text
/aft_mapped_to_init /ins/odom_ned /gnss/odom /gnss/odom_gated /NED_odometry /gps/fix /imu /rtk_path /rtk_gated_path /ins_ned_path
```

The launch also publishes live trajectory lines for RViz:

- Fast-LIVO2: `/path` with fixed frame `camera_init`
- RTK ENU: `/rtk_path` and `/rtk_gated_path` with fixed frame `map`
- INS NED: `/ins_ned_path` with fixed frame `ned`

Shortcut:

```bash
bash src/weed_bringup/scripts/start_calib_record.sh bag_name:=calib_traj_bag
```

The script assumes the workspace is `~/weed_ws`. Override it if needed:

```bash
WORKSPACE=~/your_ws bash src/weed_bringup/scripts/start_weed_system.sh
```
