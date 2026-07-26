# MS200 Dora Node

Pure Dora driver for the Oradar/Orbbec MS200p lidar.

This node does not use ROS or ROS2. It links directly against the Oradar MS200
SDK in `sdk/` and publishes a JSON `LaserScan` output in Dora.

## Build

```bash
cmake -S . -B build
cmake --build build -j2
```

## Run

```bash
~/dora-main/target/release/dora start ms200_smoke_dataflow.yml --name ms200-smoke --detach
```

Default serial settings:

```text
SERIAL_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA6084348-if00
SERIAL_BAUD=230400
```

## Output

The node publishes `LaserScan` as JSON:

```json
{
  "header": {"frame_id": "lidar", "seq": 1, "stamp": {"sec": 0, "nanosec": 0}},
  "angle_min": 0.0,
  "angle_max": 6.283185307179586,
  "angle_increment": 0.0015,
  "scan_time": 0.1,
  "time_increment": 0.0,
  "range_min": 0.05,
  "range_max": 12.0,
  "ranges": [],
  "intensities": [],
  "point_count": 0,
  "rotation_speed_hz": 10.0
}
```
