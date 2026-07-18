# Seedling spatial path planning

This ROS 2 package consumes the existing fused seedling map and 2.5D terrain:

- `/seedling/map_points` (`geometry_msgs/PoseArray`)
- `/ground/global_elevation_points` (`sensor_msgs/PointCloud2`)

It publishes:

- `/weeding/arm_1/tool_path` and `/weeding/arm_2/tool_path`
  (`nav_msgs/Path`): synchronized, forward-only 3D S paths, one row per arm
- `/weeding/arm_1/work_points` and `/weeding/arm_2/work_points`
  (`geometry_msgs/PoseArray`): the same paths for controller adapters
- `/weeding/tool_path` and `/weeding/work_points`: compatibility aliases for
  arm 1; new controller code must use the explicit per-arm topics
- `/weeding/path_markers` (`visualization_msgs/MarkerArray`): protection
  zones, estimated rows and path
- `/weeding/plan_status` (`std_msgs/String`): JSON diagnostics

## Planning policy

Planting standards are used as approximate priors:

- expected row spacing, normally 0.55 or 0.65 m
- expected plant spacing, normally 0.20 m
- crop protection radius, currently 0.05 m with no added safety margin

The planner never creates synthetic crop obstacles. Predicted missing slots
are diagnostics only. Missing sowing therefore leaves workable soil, while
close/re-sown seedlings retain separate measured positions and protection
zones.

The vehicle or conveyor supplies forward progress from the wheel encoder.
Each arm is assigned one measured row and sweeps laterally around its own
seedlings. Plants whose protection zones nearly touch are treated as a cluster
and passed on the same side; side changes occur in the safe gap between
clusters. Both paths use the same strictly increasing travel grid. Path height
is fitted independently from nearby points in the 2.5D terrain map. The local
plane slope is used to convert the requested normal surface offset into the
configured height axis, so the depth does not change on rolling ground. A path
is inhibited instead of guessing a height when local terrain support is
missing, too steep, or has excessive plane-fit error.

The vehicle config uses `x/y/z = travel/lateral/height`. The bench config uses
`z/y/x = travel/lateral/height`. `tool_surface_offset` is signed relative to
the ground: the vehicle config uses `-0.02` m for 2 cm cultivation depth,
while the bench config uses `+0.02` m to keep the tool 2 cm above the conveyor.
`height_axis_up_sign` records whether the configured positive height axis
points upward (`+1`) or downward (`-1`).

```bash
ros2 launch seedling_path_planning spatial_path_planner.launch.py

ros2 launch seedling_path_planning spatial_path_planner.launch.py \
  config_file:=$(ros2 pkg prefix seedling_path_planning)/share/seedling_path_planning/config/spatial_path_planner_bench.yaml

ros2 topic echo /weeding/plan_status
```

Recommended staged workflow:

```bash
# Start a fresh mapping run while observations are paused.
ros2 service call /seedling/freeze_map std_srvs/srv/SetBool "{data: true}"
ros2 service call /seedling/reset_map std_srvs/srv/Empty "{}"

# Build the map.
ros2 service call /seedling/freeze_map std_srvs/srv/SetBool "{data: false}"

# Freeze the accepted map before controller dry-run.
ros2 service call /seedling/freeze_map std_srvs/srv/SetBool "{data: true}"
```

Resetting a frozen map keeps it frozen. Clearing the map also makes the
planner publish an empty path and `valid=false`, so an old path cannot remain
active after a new run starts.

The published path is perception output only. Do not command the tool until
the controller-side workspace limits, emergency stop, path age and actuator
interlocks have all passed.
