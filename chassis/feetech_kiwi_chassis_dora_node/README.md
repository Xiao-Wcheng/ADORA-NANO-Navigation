# Feetech Kiwi Chassis Dora SDK Node

This directory now keeps only the Feetech SDK based Dora chassis driver for the
Adora Nano three-wheel kiwi base.

## Runtime Node

```text
sdk_node/target/release/feetech-kiwi-chassis-sdk-node
```

The node uses `feetech-servo-sdk` to access the Feetech servo bus directly.

## Interface

Input:

- `CmdVelTwist`: JSON Twist command.

```json
{
  "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
}
```

Output:

- `Odometry`: JSON odometry with pose and twist fields.

## Hardware Defaults

```text
serial port: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6086267-if00
baud:        1000000
servo IDs:   13, 14, 15
wheel angle: 60, 180, 300 deg
```

## Control Model

The node maps body-frame velocity to three wheel speed commands:

```text
wheel_speed = -sin(wheel_angle) * linear.x
            +  cos(wheel_angle) * linear.y
            +  angular.z
```

The conversion is scaled by calibrated parameters:

```text
LINEAR_TICKS_PER_MPS
ANGULAR_TICKS_PER_RADPS
SPEED_LIMIT
```

When `ODOM_SOURCE=feedback`, the node reads Feetech present speed feedback,
converts wheel speeds back to body velocity, and integrates wheel-kinematics
odometry.

Optional wheel PID is available but disabled by default:

```text
PID_ENABLED=0
PID_KP
PID_KI
PID_KD
```

## Build

```bash
cd ~/adora_nano_dora_navigation/chassis/feetech_kiwi_chassis_dora_node/sdk_node
cargo build --release
```

From the full navigation project:

```bash
cd ~/adora_nano_dora_navigation/apps/adora_nano_navigation
./adora_nav.py build
```

## Smoke Test

This starts only the SDK chassis node and publishes `Odometry` on every tick. It
does not send motion commands unless a `CmdVelTwist` input is connected.

```bash
cd ~/adora_nano_dora_navigation/chassis/feetech_kiwi_chassis_dora_node
dora start feetech_kiwi_sdk_smoke_dataflow.yml
```

## Notes

Legacy C++ Dora C-API chassis control, keyboard control, and generated CMake
build files were removed from this directory. The current project navigation
uses the SDK node above.
