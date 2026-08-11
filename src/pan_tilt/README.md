# cobra_flex_pan_tilt

ROS 2 driver for the two-axis STS3215 pan/tilt camera mount, plus a keyboard
teleop node.

## Nodes

### pan_tilt_driver

Owns the servo serial bus; homes on startup and runs current/deadman
watchdogs.

```
ros2 launch cobra_flex_pan_tilt pan_tilt.launch.py serial_port:=/dev/ttyACM0
```

Angles are radians from home. Home = pan centered, tilt camera-level
(the tilt servo's center tick points the camera straight down; home sits a
quarter turn up from it). Tilt range is [-90°, 0°]: 0 = level, -90° = down.

| Interface | Type | Notes |
|---|---|---|
| `pan_tilt_position_cmd` (sub) | `sensor_msgs/JointState` | `position` rad from home, clamped to limits, held |
| `pan_tilt_velocity_cmd` (sub) | `sensor_msgs/JointState` | `velocity` rad/s; halts 300 ms after commands stop |
| `joint_states` (pub) | `sensor_msgs/JointState` | position/velocity feedback |
| `~/home` (srv) | `std_srvs/Trigger` | blocking re-home |

Key parameters: `serial_port` (default `/dev/ttyACM1`; enumeration swaps
across boots — check `/dev/serial/by-id/`), `pan_motor_id`/`tilt_motor_id`
(1/2), `exercise_limits` (4-corner range-of-motion self-test, default false),
`run_calibration_ofs` (bench only).

### pan_tilt_keyboard_teleop

Publishes velocity commands to the driver from a terminal (run each in its
own terminal):

```
ros2 run cobra_flex_pan_tilt pan_tilt_keyboard_teleop
```

| Key | Action |
|---|---|
| `a` / `d` | pan left / right |
| `w` / `s` | tilt up / down |
| `q` / `e` | speed scale down / up |
| `space` | stop |
| `x` | quit |

Hold a key to move; motion stops `key_timeout` (0.4 s) after auto-repeat
stops. Parameters: `pan_speed` (1.0 rad/s), `tilt_speed` (0.5 rad/s),
`publish_rate` (20 Hz), `key_timeout`.

## Layout

- `src/pan_tilt_controller.*` — serial servo controller (homing, limits,
  watchdogs)
- `src/pan_tilt_node.cpp` — ROS driver node
- `src/pan_tilt_keyboard_node.cpp` — keyboard teleop
- `scservo/` — vendored Feetech SCServo SDK (verbatim upstream; keep diffable)
