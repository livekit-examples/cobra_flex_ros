# Cobra Flex ROS

> [!IMPORTANT]
> This repository is currently in Developer Preview mode and not ready for production use.
> There may be bugs, and APIs and configuration options are subject to change during this period.

ROS2 stack for the [Waveshare Cobra Flex](https://www.waveshare.com/wiki/Cobra_Flex)
4WD chassis, driven from a Jetson Orin Nano over serial.

## Packages

| Directory | Package | Purpose |
| --- | --- | --- |
| `driver/` | `cobra_flex_driver` | Serial JSON bridge to the ESP32-S3 board: `cmd_vel` in; `wheel_states` (JointState) + `battery_state` (BatteryState) out. |
| `control/` | `cobra_flex_control` | `wheel_odometry` node: integrates `wheel_states` into `odom/wheel` (nav_msgs/Odometry) + optional `odom -> base_link` TF. |
| `localization/` | `cobra_flex_localization` | robot_localization EKF config. Wheel-odometry-only today; has a commented slot for a future IMU. |
| `pan_tilt/` | `cobra_flex_pan_tilt` | Driver for the Feetech STS3215 pan/tilt head (vendored [teleop pan_tilt_demo](https://github.com/livekit-examples/teleop/tree/main/pan_tilt_demo) controller + SCServo SDK): `pan_tilt_position_cmd` / `pan_tilt_velocity_cmd` (JointState) in; `joint_states` out; `~/home` Trigger service. |
| `bringup/` | `cobra_flex_bringup` | Launch + shared params tying the stack together. |

## Workspace setup

```bash
git clone git@github.com:livekit-examples/cobra_flex_ros.git
mkdir -p src/externals/
vcs import src/externals < external.repos
```

Finally, build the workspace:
```bash
colcon build --packages-up-to cobra_flex_bringup
```

### Docker
Build the docker image:
```bash
docker compose build
```
This will build the cobra flex ros image and the cobra flex source.

Run the docker compose file:
From the repo root, run a dev cobra flex container and the ros portal:
```bash
LIVEKIT_URL=http://localhost:7880 LIVEKIT_TOKEN=test1234 docker compose up
```
you can optionally set a custom livekit config file  with `LIVEKIT_CONFIG`.

`docker-compose.yml` mounts the compose-file directory at `/cobra_flex_ros` and binds
`src/bringup/config/livekit.yaml` into the portal as `/tmp/cobra_flex_livekit.yaml`.
Override the portal config with `LIVEKIT_CONFIG` if needed:
```bash
LIVEKIT_URL=http://localhost:7880 LIVEKIT_TOKEN=test1234 \
  LIVEKIT_CONFIG=./src/bringup/config/livekit.yaml docker compose up
```

## Hardware summary (wiki spec sheet)

- 4x bus hub motors with built-in FOC (closed-loop speed control), differential
  drive; wheels commanded per side.
- ESP32-S3 driver board, JSON-over-serial protocol (USB or UART header):
  - drive: `{"T":1,"L":<0.1rpm>,"R":<0.1rpm>}`, range +-1800 (+-180 rpm)
  - feedback: `{"T":130}` poll / `{"T":131,"cmd":1}` continuous stream ->
    `{"T":1001,"M1":..,"M2":..,"M3":..,"M4":..,"odl":..,"odr":..,"v":..}`
    (per-wheel 0.1 rpm speeds; per-side mileage in cm; battery voltage in 0.01 V)
- Geometry: 74.5 mm drive wheels, 228 mm track width, 154 mm wheelbase,
  max 0.53 m/s.
- Sensors: wheel feedback and battery voltage only. **No IMU** on the chassis
  (unlike the WAVE ROVER) and no additional sensors installed yet.

### Determine the serial port
```bash
python3 /cobra_flex_ros/src/bringup/scripts/identify_serial_ports.py
```

## Usage

```bash
colcon build --packages-up-to cobra_flex_bringup
source install/setup.bash

# Driver + wheel odometry (wheel_odometry owns odom -> base_link):
ros2 launch cobra_flex_bringup cobra_flex.launch.py rover_port:=/dev/ttyACM0

# Same, with the robot_localization EKF owning the transform:
ros2 launch cobra_flex_bringup cobra_flex.launch.py use_ekf:=true

# Teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# With the pan/tilt head (install the udev rule once first, see below).
# NOTE: the driver homes both servos to center on startup -- make sure the
# arm is calibrated and free to move before enabling.
ros2 launch cobra_flex_bringup cobra_flex.launch.py rover_port:=/dev/ttyACM0 pan_tilt_port:=/dev/ttyACM1
```

Offline tests (no hardware; pure kinematics/odometry math):

```bash
colcon test --packages-select cobra_flex_driver cobra_flex_control
colcon test-result --verbose
```

## First-bring-up checklist

1. Confirm the serial device (`ls /dev/ttyACM* /dev/ttyUSB*` with the board on
   USB, or the Jetson UART header device) and baud.
2. Echo raw feedback: `ros2 topic echo /wheel_states` and `/battery_state`;
   check the frame fields match `{"T":1001,...}` above and that voltage reads
   sanely (~9-12.6 V).
3. Wheels off the ground: publish a small `cmd_vel` and verify direction
   conventions (M1 LF / M2 RF / M3 RR / M4 LR; positive x forward, positive
   yaw CCW).
4. Drive a measured straight line / in-place turn and compare against
   `odom/wheel`; tune covariances. Expect yaw over-reporting on in-place turns
   (skid-steer scrub).


## Known gaps / next steps

- No URDF/description package yet (nothing publishes `base_link` -> wheel
  frames); add one when a sensor mast or camera needs a TF tree.
- Waveshare publishes its own ROS2 driver + model package (linked from the
  wiki's Resources section) -- worth mining for the URDF meshes and any
  protocol details once hardware is in hand.
- No IMU: EKF is a passthrough placeholder until one is added.
