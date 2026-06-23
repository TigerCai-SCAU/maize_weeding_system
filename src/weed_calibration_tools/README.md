# weed_calibration_tools

Offline utilities for checking and calibrating trajectory alignment between Fast-LIVO2 and RTK/GNSS odometry.

## Export trajectories

Export Fast-LIVO2 odometry:

```bash
ros2 run weed_calibration_tools export_odom_tum \
  --bag replay_baseline \
  --topic /aft_mapped_to_init \
  --out /tmp/fastlivo.tum \
  --stamp-source bag
```

Export RTK/GNSS odometry:

```bash
ros2 run weed_calibration_tools export_odom_tum \
  --bag replay_rtk \
  --topic /gnss/odom_gated \
  --out /tmp/gnss.tum \
  --stamp-source bag
```

## Align trajectories

Estimate planar yaw and translation from GNSS trajectory to Fast-LIVO2 trajectory:

```bash
ros2 run weed_calibration_tools align_trajectories \
  --source /tmp/gnss.tum \
  --target /tmp/fastlivo.tum \
  --max-dt 0.05 \
  --out /tmp/T_gnss_to_fastlivo.txt
```

The output reports matched pairs, yaw, translation, and residual errors. Large residuals usually mean time sync, topic mismatch, poor RTK quality, or dynamic inconsistency.

Use `--stamp-source bag` for replay-generated bags when some topics preserve original sensor header stamps while other topics are produced during replay.

## AX=XB calibration

Estimate the rigid transform `X` in `A X = X B` using relative motions from two trajectories:

```bash
ros2 run weed_calibration_tools calib_axxb \
  --traj-a /tmp/gnss.tum \
  --traj-b /tmp/fastlivo.tum \
  --max-dt 0.05 \
  --skip 10 \
  --out /tmp/T_axxb_gnss_fastlivo.txt
```

Use richer calibration motion for better observability: straight lines alone are weak; turns, S-curves, rectangles, and figure-eight paths are better.
