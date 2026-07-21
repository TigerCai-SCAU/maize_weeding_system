# ground_mapper 0.4.0

Adaptive rolling ground-map package for the maize weeding robot.

The vehicle installation is configured as:

```text
body x   vertical
body y   lateral
body -z  forward
```

FAST-LIVO keeps the complete 360-degree scan for odometry. This package reads
the already deskewed and registered current scan, then crops only its downstream
copy to the active work strip.

## Inputs

```text
/cloud_registered            (camera_init frame)
/aft_mapped_to_init
```

## Outputs

```text
/ground/points             observed ground returns, camera_init frame
/ground/non_ground_points  weeds/objects close to the ground
/ground/elevation_points   dense fitted and locally filled ground surface
/ground/global_elevation_points  persistent distance/global 2.5D map
/ground/elevation_markers  RViz visualization
/ground/plane              [nx, ny, nz, d, rmse, inlier_ratio]
/ground/sensor_height      online LiDAR height above the plane
/ground/status             validity and diagnostic summary
```

## Algorithm

1. Read `/cloud_registered`, which is already in `camera_init`; do not apply the
   odometry transform to it again.
2. Match its timestamp to `/aft_mapped_to_init` and keep a short rolling window
   directly in the map frame.
3. Transform the window back to the current body frame only for ROI selection
   and ground fitting.
4. Crop lateral y to `[-1.5, 1.5]` m and crop along body `-z`.
5. Fit a coarse plane whose normal is close to body x. It estimates orientation
   and sensor height but is not published as the final terrain.
6. Divide raw returns into 5 cm cells and estimate each cell's real terrain
   height from a low height quantile.
7. Reject isolated height jumps and interpolate only short gaps near observed
   terrain cells. Unknown regions remain unknown.
8. Classify observed ground and near-ground objects relative to the local 2.5D
   terrain, preserving ridges, depressions, and gradual field undulation.

`cloud_frame_mode` defaults to `map`. A `body` compatibility mode remains
available for a future deskewed body-frame topic, but it is not used by the
default launch.

Version 0.2.2 keeps the ROS input queues short so stale odometry cannot build
up behind plane fitting on Jetson. It also re-acquires a valid plane after the
old plane times out, which handles FAST-LIVO startup pose corrections without
locking the mapper permanently to the first height estimate.

Version 0.3.0 adds a bounded 2.5D persistent ground map. Each 10 cm horizontal
cell stores one smoothed height, so repeated scans update existing cells instead
of appending duplicate point clouds.

Version 0.4.0 replaces single-plane surface generation with a real local 2.5D
terrain estimator. The global/distance map resolution also defaults to 5 cm.

## Build

```bash
cd ~/maize_weeding_system
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ground_mapper
source install/setup.bash
```

## Run

```bash
ros2 launch ground_mapper ground_mapper.launch.py
```

The default is the bounded `distance` mode with 5 m retained behind and ahead.

### Map retention modes

Local rolling window only (lowest memory):

```bash
ros2 launch ground_mapper ground_mapper.launch.py map_mode:=rolling
```

Keep 5 m behind and 5 m ahead of the vehicle:

```bash
ros2 launch ground_mapper ground_mapper.launch.py \
  map_mode:=distance \
  map_keep_behind_m:=5.0 \
  map_keep_ahead_m:=5.0
```

Keep the complete map for the current session:

```bash
ros2 launch ground_mapper ground_mapper.launch.py \
  map_mode:=global
```

For `distance` and `global`, display
`/ground/global_elevation_points` in RViz with Best Effort QoS. The topic
contains the complete retained map on every publication, so RViz Decay Time
should remain zero.

Save the retained map on demand:

```bash
ros2 service call /ground/save_map std_srvs/srv/Trigger "{}"
```

Clear it without restarting:

```bash
ros2 service call /ground/clear_map std_srvs/srv/Trigger "{}"
```

To save automatically on Ctrl-C:

```bash
ros2 launch ground_mapper ground_mapper.launch.py \
  map_mode:=global \
  save_map_on_shutdown:=true \
  save_map_path:=/tmp/ground_global_map.csv
```

## Check

```bash
ros2 topic echo /ground/status --once
ros2 topic echo /ground/sensor_height --once
ros2 topic echo /ground/points --field width \
  --qos-reliability best_effort --once
ros2 topic echo /ground/elevation_points --field width \
  --qos-reliability best_effort --once
```

The first version models the local work area as one robust plane. A later
version can add a piecewise 2.5D surface for strongly uneven soil while keeping
the same topics.
