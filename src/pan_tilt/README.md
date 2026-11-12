# cobra_flex_pan_tilt

ROS 2 driver for the STS3215 pan/tilt camera mount, plus a keyboard teleop
node. Axis geometry is per-robot configuration, so the same driver runs a
two-axis head or a pan-only one.

## Nodes

### pan_tilt_driver

Owns the servo serial bus; homes on startup and runs current/deadman
watchdogs.

```
ros2 launch cobra_flex_pan_tilt pan_tilt.launch.py serial_port:=/dev/ttyACM0
```

Angles are radians measured from each axis' home tick. By default both axes
home at the servo center tick (2048) — pan centered, tilt camera-level — with
pan travelling ±75° and tilt ±90° (+ = camera up).

| Interface | Type | Notes |
|---|---|---|
| `pan_tilt_position_cmd` (sub) | `sensor_msgs/JointState` | `position` rad from home, clamped to limits, held |
| `pan_tilt_velocity_cmd` (sub) | `sensor_msgs/JointState` | `velocity` rad/s; halts 300 ms after commands stop |
| `joint_states` (pub) | `sensor_msgs/JointState` | position/velocity feedback |
| `~/home` (srv) | `std_srvs/Trigger` | blocking re-home |

Key parameters: `serial_port` (default `/dev/ttyACM1`; enumeration swaps
across boots — check `/dev/serial/by-id/`), `pan_motor_id`/`tilt_motor_id`
(1/2), `exercise_limits` (range-of-motion self-test at startup, default true),
`run_calibration_ofs` (bench only).

#### Per-axis configuration

Each axis is described by four parameters, so different robot models are a
config change rather than a code change:

| Parameter | Default | Meaning |
|---|---|---|
| `pan_enabled` / `tilt_enabled` | `true` | Axis is physically present |
| `pan_home_ticks` / `tilt_home_ticks` | `2048` | Tick that reads as angle 0 |
| `pan_min_angle_deg` / `pan_max_angle_deg` | `-75` / `75` | Travel, as an offset from home |
| `tilt_min_angle_deg` / `tilt_max_angle_deg` | `-90` / `90` | Travel, as an offset from home |

A disabled axis is skipped in every bus transaction, omitted from
`joint_states`, and its commands are ignored — so a pan-only head runs on
`tilt_enabled: false` with no servo on ID 2. At least one axis must be
enabled. `config/pan_only.yaml` is a worked example: one pan servo with
near-full-turn travel.

The driver validates the geometry before opening the bus and refuses to start
if travel (`home_ticks` ± the limits) crosses the 0/4095 encoder wrap or
exceeds ±180°, since both make reported positions disagree with commanded
ones. The servo's own EEPROM angle limits should also contain the software
range, or moves will time out short of a target the servo will never reach.

**Two kinds of zero.** `*_home_ticks` is the *software* zero: per-robot, lives
in config, visible in review — the right knob for mounting variance.
`run_calibration_ofs` writes the current physical pose into the servo's EEPROM
as its center; it persists across host reflashes but is invisible from config
and travels with the servo, so a servo swap silently changes behaviour. Prefer
the software offset. (The stock tilt axis currently depends on the EEPROM path
for camera-level to read as tick 2048.)

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
- `config/` — sample per-robot parameter files
- `scservo/` — vendored Feetech SCServo SDK (verbatim upstream; keep diffable)
