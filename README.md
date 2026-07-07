# Maize Weeding System

ROS 2 workspace source snapshot for maize seedling mapping, ground extraction, and weeding-arm planning experiments.

## Repository layout

```text
src/
  FAST-LIVO2/                    # FAST-LIVO2, lightly patched to publish /fastlivo/deskew/cloud_body
  livox_ros_driver2/             # Livox MID360 ROS2 driver
  miivii_gmsl_camera/            # GMSL camera driver
  seedling_semantic_mapping/     # YOLO SEP detection + 3D seedling landmark map
  ground_mapper/                 # Ground / non-ground extraction and elevation points
  weedarm_h5u_bridge_ros2/       # H5U bridge and trajectory publisher
  weedarm_row_planner_test/      # Row / seedling planner test node




```

## Main runtime topics

FAST-LIVO2:

```text
/aft_mapped_to_init
/cloud_registered
/fastlivo/deskew/cloud_body
```

Seedling mapping:

```text
/left_camera/image_10hz
/fastlivo/deskew/cloud_body
/aft_mapped_to_init
/seedling/observation_point_map
/seedling/map_points
/seedling/map_markers
```

Ground mapper:

```text
/cloud_registered
/aft_mapped_to_init
/ground/points
/ground/non_ground_points
/ground/elevation_points
/ground/elevation_markers
```

## Build

Livox ROS2 driver needs ROS2 CMake arguments:

```bash
cd ~/maize_weeding_system
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

Then build the rest:

```bash
source install/setup.bash
mkdir -p src/rpg_vikit/vikit_common/lib

colcon build --symlink-install --packages-skip livox_ros_driver2
source install/setup.bash
```

## Seedling mapping

Edit the model path before running:

```bash
gedit src/seedling_semantic_mapping/config/seedling_pipeline.yaml
```

Set:

```yaml
model_path: "/home/nvidia/models/your_maize_sep_pose_model.pt"
```

Run:

```bash
ros2 launch seedling_semantic_mapping seedling_pipeline.launch.py
```

## Ground mapper

Run:

```bash
ros2 launch ground_mapper ground_mapper.launch.py
```

Do not commit build outputs, bags, models, PCD maps, CSV logs, or binary libraries.
