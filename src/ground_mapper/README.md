# ground_mapper

`ground_mapper` is a small ROS 2 Humble Python package for extracting ground points and building a local elevation map from FAST-LIVO's registered point cloud.

The first version is designed for your current maize weeding robot workflow:

```text
FAST-LIVO
  /cloud_registered
  /aft_mapped_to_init
        ↓
ground_mapper
  /ground/points
  /ground/non_ground_points
  /ground/elevation_points
  /ground/elevation_markers
```

## Why use `/cloud_registered`?

`/cloud_registered` is already processed by FAST-LIVO. It is registered in the SLAM/map frame and is more suitable for ground elevation mapping than raw `/livox/lidar`.

Do **not** treat `/cloud_registered` directly as the clean ground surface. It still includes crop points, weeds, sparse outliers, and non-ground points. This package filters them and estimates a local ground height grid.

## Algorithm

For each incoming cloud:

1. Read `x, y, z` from `/cloud_registered`.
2. Crop ROI. If odom is available, x/y ROI is centered on the robot pose and aligned with odom yaw.
3. Split points into XY grid cells.
4. In each cell, take a low z percentile as the local ground height.
5. Keep points near the ground height as `/ground/points`.
6. Publish points significantly above ground as `/ground/non_ground_points`.
7. Publish one elevation point per valid cell as `/ground/elevation_points`.

This works well when most points are ground and crops are sparse high points.

## Install

```bash
cd ~/fast_ws/src
unzip ~/Downloads/ground_mapper.zip

cd ~/fast_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select ground_mapper
source install/setup.bash
```

## Run

```bash
ros2 launch ground_mapper ground_mapper.launch.py
```

## Check topics

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /ground/points
ros2 topic hz /ground/elevation_points
```

Check one message:

```bash
ros2 topic echo /ground/elevation_points --field width --qos-reliability best_effort --once
```

## RViz

Add these displays:

- `PointCloud2`: `/cloud_registered`, original FAST-LIVO cloud.
- `PointCloud2`: `/ground/points`, extracted ground points.
- `PointCloud2`: `/ground/non_ground_points`, crop/weed/outlier points.
- `PointCloud2`: `/ground/elevation_points`, one point per grid cell.
- `MarkerArray`: `/ground/elevation_markers`, elevation grid visualization.

## Important parameters

Edit:

```bash
gedit ~/fast_ws/src/ground_mapper/config/ground_mapper.yaml
```

Typical first values:

```yaml
grid_resolution: 0.03
height_percentile: 20.0
min_points_per_cell: 3
ground_keep_above: 0.04
ground_keep_below: 0.03
non_ground_above: 0.06
```

Tuning guide:

- Crop/crop leaf points remain in `/ground/points`: lower `ground_keep_above` to `0.03`, or lower `height_percentile` to `10.0`.
- Too many real ground points are removed: raise `ground_keep_above` to `0.05`.
- Elevation map has holes: increase `grid_resolution` to `0.05` or lower `min_points_per_cell` to `2`.
- CPU is high: set `process_every_n_clouds: 2` or `max_points_per_cloud: 30000`.

## Relationship with seedling map

`ground_mapper` is separate from `seedling_semantic_mapping`.

Later path planning should use:

```text
/seedling/map_points
/ground/elevation_points
```

For each planned tool XY point:

```text
tool_z = ground_height(x, y) - target_depth
```

For 2 cm tillage depth:

```text
tool_z = ground_height(x, y) - 0.02
```

Confirm your map z-axis direction before using this for real actuator control.
