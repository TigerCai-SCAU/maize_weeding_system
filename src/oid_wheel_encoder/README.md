# OID wheel encoder

ROS 2 SocketCAN driver for the OID CAN single-turn absolute encoder used with a
63 mm measuring wheel. The default configuration is 15-bit (32768 count/rev),
CAN ID 1, and 500 kbit/s.

The driver sends the documented position query `[04 01 01 00]`, unwraps the
single-turn count, computes forward speed from the host monotonic clock, and
publishes `/wheel/odom`. It intentionally does not change persistent encoder
settings. Only `twist.linear.x` should be fused; a single measuring wheel cannot
observe yaw or lateral velocity.

Manual wiring: red is 5-24 V, black is ground, green is CANH, white is CANL,
and yellow is the zero-setting input (leave it insulated when unused). Fit a
120 ohm termination resistor at the bus end.

Bring up the selected SocketCAN interface before starting ROS (replace `can0`
if the encoder is wired to another controller):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

The default vehicle configuration keeps the tractor mounting direction:

```bash
ros2 launch oid_wheel_encoder oid_wheel_encoder.launch.py
ros2 topic echo /wheel/odom --field twist.twist.linear.x
```

For the indoor conveyor, select the bench YAML explicitly:

```bash
ros2 launch oid_wheel_encoder oid_wheel_encoder.launch.py \
  config_file:=$(ros2 pkg prefix --share oid_wheel_encoder)/config/oid_wheel_encoder_bench.yaml
```

Both configurations publish forward travel as positive. Calibrate
`wheel_diameter_m` from a measured travel distance on the actual contact
surface.
