# Seedling spatial path planning

This ROS 2 package consumes the existing fused seedling map and 2.5D terrain:

- `/seedling/map_points` (`geometry_msgs/PoseArray`)
- `/ground/global_elevation_points` (`sensor_msgs/PointCloud2`)

It publishes:

- `/weeding/tool_path` (`nav_msgs/Path`): collision-checked 3D tool path
- `/weeding/work_points` (`geometry_msgs/PoseArray`): the same path for
  controller adapters
- `/weeding/path_markers` (`visualization_msgs/MarkerArray`): protection
  zones, estimated rows and path
- `/weeding/plan_status` (`std_msgs/String`): JSON diagnostics

## Planning policy

Planting standards are used as approximate priors:

- expected row spacing, normally 0.55 or 0.65 m
- expected plant spacing, normally 0.20 m
- crop protection radius, normally 0.08 m

The planner never creates synthetic crop obstacles. Missing sowing therefore
leaves workable soil; close/re-sown seedlings produce separate overlapping
protection zones. A lawnmower coverage path is generated over the mapped
working window and A* connectors route around the union of all protection
zones. Path height is interpolated from the terrain map.

The vehicle config uses `x/y/z = travel/lateral/height`. The bench config uses
`z/y/x = travel/lateral/height`.

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
