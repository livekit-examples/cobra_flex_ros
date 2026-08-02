# Cobra Flex ROS

ROS2 stack for the [Waveshare Cobra Flex](https://www.waveshare.com/wiki/Cobra_Flex)
4WD chassis, driven from a Jetson Orin Nano over serial.

## Packages

| Directory | Package | Purpose |
| --- | --- | --- |
| `driver/` | `cobra_flex_driver` | Serial JSON bridge to the ESP32-S3 board: `cmd_vel` in; `wheel_states` (JointState) + `battery_state` (BatteryState) out. |
| `control/` | `cobra_flex_control` | `wheel_odometry` node: integrates `wheel_states` into `odom/wheel` (nav_msgs/Odometry) + optional `odom -> base_link` TF. |
| `localization/` | `cobra_flex_localization` | robot_localization EKF config. Wheel-odometry-only today; has a commented slot for a future IMU. |
| `bringup/` | `cobra_flex_bringup` | Launch + shared params tying the stack together. |

## Workspace setup

This repo is designed to be cloned into a colcon workspace alongside
[ros-portal](https://github.com/livekit/ros-portal), which is pulled in via
the [vcs tool](https://github.com/dirk-thomas/vcstool) using
[`cobra_flex.repos`](./cobra_flex.repos):

```bash
mkdir -p cobra_ws/src && cd cobra_ws
git clone git@github.com:livekit-examples/cobra_flex_ros.git src/cobra_flex_ros

vcs import src < src/cobra_flex_ros/cobra_flex.repos   # brings in ros-portal
vcs import src < src/ros-portal/external.repos         # ros_portal's own deps (client-sdk-cpp, cpp-tools, ...)

# TEMPORARY: ros-portal still ships its own copy of the cobra_flex packages;
# hide it so colcon doesn't see duplicate package names. Drop this once the
# in-tree copy is removed upstream.
touch src/ros-portal/src/test/cobra_flex/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to cobra_flex_bringup
```

The second `vcs import` (and building `ros_portal` itself) is only needed when
running the LiveKit bridge with [`bringup/config/livekit.yaml`](./bringup/config/livekit.yaml);
the bare robot stack builds without it. See the
[ros-portal README](https://github.com/livekit/ros-portal#readme) for details
on building and running `ros_portal`.

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

## Usage

```bash
colcon build --packages-up-to cobra_flex_bringup
source install/setup.bash

# Driver + wheel odometry (wheel_odometry owns odom -> base_link):
ros2 launch cobra_flex_bringup cobra_flex.launch.py serial_port:=/dev/ttyACM0

# Same, with the robot_localization EKF owning the transform:
ros2 launch cobra_flex_bringup cobra_flex.launch.py use_ekf:=true

# Teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
