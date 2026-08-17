/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pan_tilt_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unistd.h>

namespace {
constexpr useconds_t kCommandSettleUs = 500000;
constexpr useconds_t kStatePollUs = 10 * 1000;
constexpr int kWatchdogRateHz = 30;
constexpr int kPersistentOvercurrentSamplesBeforeTorqueCut = 4;
constexpr int kBlockingTimeoutMs = 5000;
constexpr int kPositionToleranceTicks = 8;
constexpr int kSettledSampleCount = 3;
constexpr int kPanIndex = 0;
constexpr int kTiltIndex = 1;
constexpr int kInitRetryCount = 3;
constexpr useconds_t kInitRetryDelayUs = 100 * 1000;

int circularDistanceTicks(const int a, const int b) {
  const int raw = std::abs(a - b);
  return std::min(raw, PanTiltController::kTicksPerRevolution - raw);
}
} // namespace

PanTiltController::PanTiltController(
    const std::string &serial_port,
    const std::array<AxisConfig, kMotorCount> &axes, const int baud)
    : serial_port_(serial_port), axes_(axes), baud_(baud),
      opened_(false), watchdog_stop_requested_(false),
      watchdog_running_(false) {
  for (auto &mode : motor_bus_mode_) {
    mode.store(static_cast<int>(BusMode::kUnknown));
  }
}

PanTiltController::~PanTiltController() {
  stopWatchdogThread();
  if (opened_) {
    const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    sms_sts_.end();
    opened_ = false;
  }
}

bool PanTiltController::initialize(const bool run_calibration_ofs) {
  // Validate before opening: a bad geometry config should fail without
  // energizing a motor that might then be driven into a hard stop.
  if (!validateAxes()) {
    return false;
  }
  if (!open()) {
    return false;
  }
  if (!initMotors()) {
    return false;
  }
  if (!pingMotors()) {
    return false;
  }
  if (!ensureFeedback()) {
    return false;
  }
  if (run_calibration_ofs && !runCalibrationOfs()) {
    return false;
  }
  if (!homeMotors()) {
    return false;
  }

  startWatchdogThread();
  return true;
}

bool PanTiltController::exerciseLimits() {
  // Waypoints hold one offset-from-home target per axis; entries for absent
  // axes are never read.
  using Waypoint = std::array<double, kMotorCount>;
  std::array<Waypoint, 4> waypoints{};
  int waypoint_count = 0;

  const auto extremes = [this](const int motor_index) {
    return std::array<double, 2>{axes_[motor_index].max_from_home_rad,
                                 axes_[motor_index].min_from_home_rad};
  };

  if (axes_[kPanIndex].present && axes_[kTiltIndex].present) {
    const auto pan = extremes(kPanIndex);
    const auto tilt = extremes(kTiltIndex);
    // Serpentine order: hold tilt at an extreme, sweep pan across, step tilt,
    // sweep pan back. Reproduces the historical corner ordering
    // (max/max -> min/max -> min/min -> max/min).
    for (int t = 0; t < 2; ++t) {
      for (int p = 0; p < 2; ++p) {
        Waypoint &waypoint = waypoints[waypoint_count++];
        waypoint[kPanIndex] = pan[(t == 0) ? p : 1 - p];
        waypoint[kTiltIndex] = tilt[t];
      }
    }
  } else {
    // Single-axis head: the limit box degenerates to a line segment.
    const int motor_index = axes_[kPanIndex].present ? kPanIndex : kTiltIndex;
    for (const double extreme : extremes(motor_index)) {
      waypoints[waypoint_count++][motor_index] = extreme;
    }
  }

  WriteLine(std::cout, "[pan_tilt] Exercising motion limits ({} waypoints)",
            waypoint_count);

  for (int w = 0; w < waypoint_count; ++w) {
    for (int i = 0; i < kMotorCount; ++i) {
      if (!axes_[i].present) {
        continue;
      }
      if (!setMotorAngleFromHome(i, waypoints[w][i])) {
        WriteLine(std::cerr,
                  "[pan_tilt] exerciseLimits failed to command waypoint");
        return false;
      }
    }

    for (int i = 0; i < kMotorCount; ++i) {
      if (!axes_[i].present) {
        continue;
      }
      const int target_ticks = angleFromHomeToTicks(i, waypoints[w][i]);
      if (!waitForMotorPositionMoveComplete(i, target_ticks,
                                            kBlockingTimeoutMs)) {
        WriteLine(std::cerr,
                  "[pan_tilt] exerciseLimits waypoint move did not settle");
        return false;
      }
    }
  }

  return homeMotors();
}

bool PanTiltController::homeMotors() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    WriteLine(std::cout, "[pan_tilt] Homing {} (ID {}) to {} ticks",
              axes_[i].name, id, homeTicks(i));
    // The motors may be in wheel (velocity) mode; WritePosEx sent in wheel
    // mode is misinterpreted as continuous rotation and runs to the stops.
    if (!ensureServoMode(i)) {
      WriteLine(std::cerr, "[pan_tilt] Home ensureServoMode failed for ID {}",
                id);
      return false;
    }
    if (!sms_sts_.WritePosEx(static_cast<u8>(id),
                             static_cast<s16>(homeTicks(i)),
                             kDefaultMoveSpeed, kDefaultMoveAcc)) {
      WriteLine(std::cerr, "[pan_tilt] Home WritePosEx failed for ID {}", id);
      return false;
    }
  }

  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    if (!waitForMotorPositionMoveComplete(i, homeTicks(i), 3000)) {
      WriteLine(std::cerr, "[pan_tilt] Home move failed for ID {}", motorId(i));
      return false;
    }
    // Settled at a known absolute pose in servo mode: trusted anchor.
    anchorTracking(i, homeTicks(i));
  }
  return true;
}

bool PanTiltController::setMotorAngle(const int motor_index,
                                      const double absolute_angle_rad,
                                      const u16 speed) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  if (!ensureServoMode(motor_index)) {
    return false;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  const int target_ticks = wrapTicks(angleRadToTicks(absolute_angle_rad));
  if (!sms_sts_.WritePosEx(static_cast<u8>(id), static_cast<s16>(target_ticks),
                           speed, kDefaultMoveAcc)) {
    WriteLine(std::cerr, "[pan_tilt] setMotorAngle failed for ID {}", id);
    return false;
  }
  WriteLine(std::cout, "[pan_tilt] setMotorAngle ID {} -> {} rad ({} ticks)",
            id, absolute_angle_rad, target_ticks);
  return true;
}

double PanTiltController::clampAngleFromHomeRad(
    const int motor_index, const double angle_from_home_rad) const {
  // The configured min/max may arrive in either numeric order, so derive the
  // actual bounds here. std::clamp has undefined behavior if the lower bound
  // exceeds the upper bound.
  const auto [lo, hi] = std::minmax(axes_[motor_index].min_from_home_rad,
                                    axes_[motor_index].max_from_home_rad);
  return std::clamp(angle_from_home_rad, lo, hi);
}

int PanTiltController::angleFromHomeToTicks(
    const int motor_index, const double angle_from_home_rad) const {
  const double clamped_rad =
      clampAngleFromHomeRad(motor_index, angle_from_home_rad);
  return wrapTicks(homeTicks(motor_index) + angleRadToTicks(clamped_rad));
}

double PanTiltController::ticksToAngleFromHomeRad(const int motor_index,
                                                  const int ticks) const {
  // The STS3215 position register accumulates multi-turn counts after
  // wheel-mode (velocity) driving, so reduce to a within-revolution tick
  // first, then take the shortest signed distance from home so the reported
  // angle always lands in (-pi, pi]. C++ % is negative for negative operands
  // (wheel-mode multi-turn positions go negative), so normalize into
  // [0, kTicksPerRevolution) before recentering.
  int delta = (ticks - homeTicks(motor_index)) % kTicksPerRevolution;
  if (delta < 0) {
    delta += kTicksPerRevolution;
  }
  if (delta > kTicksPerRevolution / 2) {
    delta -= kTicksPerRevolution;
  }
  return ticksToAngleRad(delta);
}

bool PanTiltController::validateAxes() const {
  bool any_present = false;
  for (int i = 0; i < kMotorCount; ++i) {
    const AxisConfig &axis = axes_[i];
    if (!axis.present) {
      continue;
    }
    any_present = true;

    if (axis.home_ticks < 0 || axis.home_ticks >= kTicksPerRevolution) {
      WriteLine(std::cerr,
                "[pan_tilt] {}: home_ticks {} outside [0, {})", axis.name,
                axis.home_ticks, kTicksPerRevolution);
      return false;
    }
    if (!std::isfinite(axis.min_from_home_rad) ||
        !std::isfinite(axis.max_from_home_rad) ||
        axis.min_from_home_rad >= axis.max_from_home_rad) {
      WriteLine(std::cerr,
                "[pan_tilt] {}: limits [{}, {}] rad are not a finite, "
                "increasing range",
                axis.name, axis.min_from_home_rad, axis.max_from_home_rad);
      return false;
    }
    // Beyond +/-pi the shortest-signed-distance reduction in
    // ticksToAngleFromHomeRad() aliases to the wrong sign, so reported
    // positions would fight commanded ones.
    if (axis.min_from_home_rad < -kPi || axis.max_from_home_rad > kPi) {
      WriteLine(std::cerr,
                "[pan_tilt] {}: limits [{}, {}] rad exceed +/-pi", axis.name,
                axis.min_from_home_rad, axis.max_from_home_rad);
      return false;
    }
    // Travel must not cross the 0/4095 encoder wrap: angleFromHomeToTicks()
    // wraps the target, so a range straddling the seam would silently command
    // the long way around.
    const int min_ticks =
        axis.home_ticks + angleRadToTicks(axis.min_from_home_rad);
    const int max_ticks =
        axis.home_ticks + angleRadToTicks(axis.max_from_home_rad);
    if (min_ticks < 0 || max_ticks >= kTicksPerRevolution) {
      WriteLine(std::cerr,
                "[pan_tilt] {}: travel spans ticks [{}, {}], which crosses the "
                "0/{} encoder wrap (home_ticks={})",
                axis.name, min_ticks, max_ticks, kTicksPerRevolution - 1,
                axis.home_ticks);
      return false;
    }
  }

  if (!any_present) {
    WriteLine(std::cerr, "[pan_tilt] No axis is configured as present");
    return false;
  }
  return true;
}

int PanTiltController::homeTicks(const int motor_index) const {
  return axes_[motor_index].home_ticks;
}

bool PanTiltController::setMotorAngleFromHome(const int motor_index,
                                             const double angle_from_home_rad,
                                             const u16 speed) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  if (!ensureServoMode(motor_index)) {
    return false;
  }

  const double clamped_rad =
      clampAngleFromHomeRad(motor_index, angle_from_home_rad);
  if (clamped_rad != angle_from_home_rad) {
    WriteLine(std::cerr,
              "[pan_tilt] Position request {} rad clamped to {} rad (limits) "
              "for motor {}",
              angle_from_home_rad, clamped_rad, axes_[motor_index].name);
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  const int target_ticks =
      wrapTicks(homeTicks(motor_index) + angleRadToTicks(clamped_rad));
  if (!sms_sts_.WritePosEx(static_cast<u8>(id), static_cast<s16>(target_ticks),
                           speed, kDefaultMoveAcc)) {
    WriteLine(std::cerr, "[pan_tilt] setMotorAngleFromHome failed for ID {}", id);
    return false;
  }

  // Latch position-hold so the velocity deadman watchdog stays suspended and
  // the commanded angle is held indefinitely until a new command arrives.
  position_hold_active_.store(true);
  return true;
}

bool PanTiltController::setMotorAngleBlocking(const int motor_index,
                                              const double absolute_angle_rad,
                                              const u16 speed) {
  if (!setMotorAngle(motor_index, absolute_angle_rad, speed)) {
    return false;
  }

  const int target_ticks = wrapTicks(angleRadToTicks(absolute_angle_rad));
  return waitForMotorPositionMoveComplete(motor_index, target_ticks,
                                          kBlockingTimeoutMs);
}

bool PanTiltController::setMotorAngleRelative(const int motor_index,
                                              const double relative_angle_rad,
                                              const u16 speed) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  const int current_ticks = sms_sts_.ReadPos(static_cast<u8>(id));
  if (current_ticks < 0) {
    WriteLine(std::cerr, "[pan_tilt] ReadPos failed for ID {}", id);
    return false;
  }

  const int delta_ticks = angleRadToTicks(relative_angle_rad);
  const int target_ticks = wrapTicks(current_ticks + delta_ticks);
  if (!sms_sts_.WritePosEx(static_cast<u8>(id), static_cast<s16>(target_ticks),
                           speed, kDefaultMoveAcc)) {
    WriteLine(std::cerr, "[pan_tilt] setMotorAngleRelative failed for ID {}",
              id);
    return false;
  }

  WriteLine(std::cout,
            "[pan_tilt] Relative move ID {}: {} rad ({} -> {} ticks)", id,
            relative_angle_rad, current_ticks, target_ticks);
  return true;
}

bool PanTiltController::setMotorAngleRelativeBlocking(
    const int motor_index, const double relative_angle_rad, const u16 speed) {

  if (!setMotorAngleRelative(motor_index, relative_angle_rad, speed)) {
    return false;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  const int current_ticks = sms_sts_.ReadPos(static_cast<u8>(id));
  if (current_ticks < 0) {
    WriteLine(
        std::cerr,
        "[pan_tilt] setMotorAngleRelativeBlocking ReadPos failed for ID {}",
        id);
    return false;
  }
  const int target_ticks =
      wrapTicks(current_ticks + angleRadToTicks(relative_angle_rad));

  return waitForMotorPositionMoveComplete(motor_index, target_ticks,
                                          kBlockingTimeoutMs);
}

bool PanTiltController::setVelocity(const int motor_index,
                                    const s16 velocity_steps_per_sec,
                                    const u8 acc) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }
  if (velocity_steps_per_sec < -3400 || velocity_steps_per_sec > 3400) {
    WriteLine(std::cerr, "[pan_tilt] Velocity {} out of range [-3400, 3400]",
              velocity_steps_per_sec);
    return false;
  }
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  // Only re-initialize on a mode transition: InitMotor writes the operating
  // mode to servo EEPROM, so issuing it on every velocity command (e.g. the
  // watchdog's periodic halt) wears the EEPROM and floods the bus.
  if (motor_bus_mode_[motor_index].load() !=
      static_cast<int>(BusMode::kWheel)) {
    if (!sms_sts_.InitMotor(static_cast<u8>(id), SMS_STS_MODE_WHEEL_CLOSED,
                            1)) {
      WriteLine(std::cerr, "[pan_tilt] Failed to set wheel mode for ID {}", id);
      return false;
    }
    motor_bus_mode_[motor_index].store(static_cast<int>(BusMode::kWheel));
    invalidateTrackingBaseline(motor_index);
  }
  // A velocity command means we are back under velocity control; release any
  // latched position hold so the deadman watchdog re-arms.
  position_hold_active_.store(false);
  if (!sms_sts_.WriteSpe(static_cast<u8>(id), velocity_steps_per_sec, acc)) {
    WriteLine(std::cerr, "[pan_tilt] setVelocity WriteSpe failed for ID {}",
              id);
    return false;
  }

  // Every velocity command feeds the deadman; it should only trip when
  // commands stop arriving, not while a nonzero stream is active.
  last_user_input_velocity_set_time_.store(std::chrono::steady_clock::now());

  if (velocity_steps_per_sec > 0) {
    WriteLine(std::cout, "[pan_tilt] setVelocity ID {} -> {} steps/s", id,
              velocity_steps_per_sec);
  }
  return true;
}

bool PanTiltController::haltMotors(const u8 acc) {
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    if (!setVelocity(i, 0, acc)) {
      WriteLine(std::cerr, "[pan_tilt] halt failed for motor index {}", i);
      return false;
    }
  }
  WriteLine(std::cout, "[pan_tilt] halt complete");
  return true;
}

void PanTiltController::anchorTracking(const int motor_index,
                                       const int absolute_ticks) {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  tracked_ticks_[motor_index] = wrapTicks(absolute_ticks);
  last_raw_ticks_[motor_index] = absolute_ticks;
  tracking_valid_[motor_index] = true;
  baseline_valid_[motor_index] = true;
}

void PanTiltController::invalidateTrackingBaseline(const int motor_index) {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  baseline_valid_[motor_index] = false;
}

int PanTiltController::updateTracking(const int motor_index,
                                      const int raw_ticks) {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  if (!tracking_valid_[motor_index]) {
    anchorTracking(motor_index, raw_ticks);
    return tracked_ticks_[motor_index];
  }
  if (!baseline_valid_[motor_index]) {
    // First sample after a mode switch: the register frame just jumped.
    // Re-seed the baseline without integrating the discontinuity.
    last_raw_ticks_[motor_index] = raw_ticks;
    baseline_valid_[motor_index] = true;
    return tracked_ticks_[motor_index];
  }
  // Shortest wrapped delta between consecutive samples. Poll periods are
  // short enough (~170 ticks max at 3400 steps/s @ 20 Hz) that a half-rev
  // ambiguity cannot occur between successive valid reads.
  int delta = (raw_ticks - last_raw_ticks_[motor_index]) % kTicksPerRevolution;
  if (delta < 0) {
    delta += kTicksPerRevolution;
  }
  if (delta > kTicksPerRevolution / 2) {
    delta -= kTicksPerRevolution;
  }
  last_raw_ticks_[motor_index] = raw_ticks;
  tracked_ticks_[motor_index] += delta;
  return tracked_ticks_[motor_index];
}

bool PanTiltController::resyncServoHold() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  bool all_ok = true;
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    if (!ensureServoMode(i)) {
      all_ok = false;
      continue;
    }
    const int current_ticks = sms_sts_.ReadPos(static_cast<u8>(id));
    if (current_ticks < 0) {
      WriteLine(std::cerr, "[pan_tilt] resyncServoHold ReadPos failed for ID {}",
                id);
      all_ok = false;
      continue;
    }
    if (!sms_sts_.WritePosEx(static_cast<u8>(id),
                             static_cast<s16>(wrapTicks(current_ticks)),
                             kDefaultMoveSpeed, kDefaultMoveAcc)) {
      WriteLine(std::cerr,
                "[pan_tilt] resyncServoHold WritePosEx failed for ID {}", id);
      all_ok = false;
      continue;
    }
    // Back in servo mode, the register is absolute again: trusted anchor.
    anchorTracking(i, current_ticks);
  }
  if (all_ok) {
    // Holding an absolute pose; keep the deadman suspended until the next
    // velocity command releases the latch.
    position_hold_active_.store(true);
    WriteLine(std::cout,
              "[pan_tilt] Re-synced to servo mode hold after velocity idle");
  }
  return all_ok;
}

std::array<PanTiltController::ServoState, PanTiltController::kMotorCount>
PanTiltController::pollState() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  std::array<ServoState, kMotorCount> states{};

  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    ServoState &state = states[i];
    state.motor_id = id;

    if (!sms_sts_.FeedBack(id)) {
      WriteLine(std::cerr, "[pan_tilt] FeedBack failed while polling ID {}",
                id);
      continue;
    }

    state.position_ticks = sms_sts_.ReadPos(-1);
    state.speed = sms_sts_.ReadSpeed(-1);
    state.load_pwm = sms_sts_.ReadLoad(-1);
    state.voltage_01v = sms_sts_.ReadVoltage(-1);
    state.temperature_celsius = sms_sts_.ReadTemper(-1);
    state.moving = sms_sts_.ReadMove(-1);
    state.current_milliamps = sms_sts_.ReadCurrent(-1);

    state.valid = (state.position_ticks >= 0 && state.voltage_01v >= 0 &&
                   state.temperature_celsius >= 0 && state.moving >= 0 &&
                   state.current_milliamps >= 0);
    if (!state.valid) {
      WriteLine(std::cerr, "[pan_tilt] State decode incomplete for ID {}", id);
      continue;
    }
    state.tracked_ticks = updateTracking(i, state.position_ticks);
    state.angle_from_home_rad =
        ticksToAngleFromHomeRad(i, state.tracked_ticks);
  }

  return states;
}

bool PanTiltController::printAngles() {
  const auto states = pollState();

  bool all_ok = true;
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const ServoState &state = states[i];
    if (!state.valid) {
      WriteLine(std::cerr, "[pan_tilt] printAngles: invalid state for {}",
                axes_[i].name);
      all_ok = false;
      continue;
    }
    WriteLine(std::cout,
              "[pan_tilt] Angles  {}: relative={:.2f} deg global={:.2f} deg",
              axes_[i].name,
              ticksToAngleDeg(state.position_ticks - homeTicks(i)),
              ticksToAngleDeg(state.position_ticks));
  }
  return all_ok;
}

bool PanTiltController::waitForMotorPositionMoveComplete(const int motor_index,
                                                         const int target_ticks,
                                                         const int timeout_ms) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  const int id = motorId(motor_index);
  int settled_samples = 0;
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    if (std::chrono::steady_clock::now() - start >
        std::chrono::milliseconds(timeout_ms)) {
      WriteLine(std::cerr,
                "[pan_tilt] waitForMotorPositionMoveComplete timeout for ID {}",
                id);
      return false;
    }

    const auto states = pollState();
    const ServoState &state = states[motor_index];
    if (!state.valid) {
      // Feedback reads can fail transiently while the motors are moving
      // (EMI/supply sag on the servo bus). Keep polling; the timeout above
      // is the arbiter of a genuinely dead bus.
      WriteLine(std::cerr,
                "[pan_tilt] transient poll failure for ID {}; retrying", id);
      settled_samples = 0;
      usleep(kStatePollUs);
      continue;
    }

    const int dist_ticks =
        circularDistanceTicks(state.position_ticks, target_ticks);
    if (state.moving == 0 && dist_ticks <= kPositionToleranceTicks) {
      ++settled_samples;
    } else {
      settled_samples = 0;
    }

    if (settled_samples >= kSettledSampleCount) {
      return true;
    }
    usleep(kStatePollUs);
  }
}

bool PanTiltController::open() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  WriteLine(std::cout, "[pan_tilt] Opening {} @ {} baud", serial_port_, baud_);
  if (!sms_sts_.begin(baud_, serial_port_.c_str())) {
    WriteLine(std::cerr, "[pan_tilt] Failed to init SMS/STS bus");
    return false;
  }
  sms_sts_.Level = 1;
  opened_ = true;
  return true;
}

void PanTiltController::startWatchdogThread() {
  if (watchdog_running_.load()) {
    return;
  }

  watchdog_stop_requested_.store(false);
  watchdog_thread_ = std::thread(&PanTiltController::watchdogThreadMain, this);
  watchdog_running_.store(true);
  WriteLine(std::cout, "[pan_tilt] Current watchdog started at {} Hz",
            kWatchdogRateHz);
}

void PanTiltController::stopWatchdogThread() {
  if (!watchdog_running_.load()) {
    return;
  }

  watchdog_stop_requested_.store(true);
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  watchdog_running_.store(false);
  WriteLine(std::cout, "[pan_tilt] Current watchdog stopped");
}

void PanTiltController::watchdogThreadMain() {
  const auto watchdog_period =
      std::chrono::milliseconds(1000 / kWatchdogRateHz);
  auto next_wakeup = std::chrono::steady_clock::now();
  std::array<int, kMotorCount> overcurrent_sample_counts{};
  bool torque_cut_applied = false;
  uint64_t count = 0;
  while (!watchdog_stop_requested_.load()) {
    ++count;
    bool any_motor_overcurrent = false;

    if (count % kWatchdogRateHz == 0) {
      printAngles();
    }

    // The velocity deadman is suspended while an absolute position is held:
    // position commands are latched and intentionally do not time out.
    // Overcurrent protection below still applies in all modes.
    // It only engages while a motor is under velocity (wheel) control --
    // motors in servo mode are holding a position and have no stale velocity
    // to halt.
    bool any_motor_in_wheel_mode = false;
    for (int i = 0; i < kMotorCount; ++i) {
      any_motor_in_wheel_mode |= axes_[i].present &&
                                 motor_bus_mode_[i].load() ==
                                     static_cast<int>(BusMode::kWheel);
    }
    const auto time_since_last_user_input_velocity_set =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() -
            last_user_input_velocity_set_time_.load());
    if (any_motor_in_wheel_mode && !position_hold_active_.load() &&
        time_since_last_user_input_velocity_set.count() > 300) {
      if (count % 10 == 0) {
        WriteLine(std::cerr,
                  "[pan_tilt] Time since last user input velocity set: {} ms",
                  time_since_last_user_input_velocity_set.count());
      }
      haltMotors();
      resyncServoHold();
    }
    for (int i = 0; i < kMotorCount; ++i) {
      if (!axes_[i].present) {
        continue;
      }
      int current_milliamps = -1;
      if (!readMotorCurrentMilliamps(i, current_milliamps)) {
        continue;
      }

      if (current_milliamps > kCurrentLimitMilliamps) {
        any_motor_overcurrent = true;
        ++overcurrent_sample_counts[i];
        const int id = motorId(i);

        if (overcurrent_sample_counts[i] == 1) {
          WriteLine(std::cerr,
                    "[pan_tilt] Overcurrent detected on ID {}: {} mA > {} "
                    "mA; halting motors",
                    id, current_milliamps, kCurrentLimitMilliamps);
          if (!haltMotors()) {
            WriteLine(std::cerr,
                      "[pan_tilt] Failed to halt motors after overcurrent");
          }
        }

        if (!torque_cut_applied &&
            overcurrent_sample_counts[i] >=
                kPersistentOvercurrentSamplesBeforeTorqueCut) {
          WriteLine(std::cerr,
                    "[pan_tilt] Persistent overcurrent on ID {} after halt; "
                    "disabling torque",
                    id);
          for (int m = 0; m < kMotorCount; ++m) {
            if (!axes_[m].present) {
              continue;
            }
            if (!disableMotorTorque(m)) {
              WriteLine(
                  std::cerr,
                  "[pan_tilt] Failed to disable torque for motor index {}", m);
            }
          }
          torque_cut_applied = true;
        }
      } else {
        overcurrent_sample_counts[i] = 0;
      }
    }

    // Rearm persistent overcurrent handling after currents normalize.
    if (!any_motor_overcurrent) {
      torque_cut_applied = false;
    }

    next_wakeup += watchdog_period;
    const auto now = std::chrono::steady_clock::now();
    if (next_wakeup > now) {
      std::this_thread::sleep_until(next_wakeup);
    } else {
      next_wakeup = now;
    }
  }
}

bool PanTiltController::readMotorCurrentMilliamps(const int motor_index,
                                                  int &current_milliamps) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const auto id = motorId(motor_index);
  const int read_current = sms_sts_.ReadCurrent(static_cast<u8>(id));
  if (read_current < 0) {
    return false;
  }

  current_milliamps = read_current;
  return true;
}

bool PanTiltController::disableMotorTorque(const int motor_index) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  if (!sms_sts_.EnableTorque(static_cast<u8>(id), 0)) {
    WriteLine(std::cerr, "[pan_tilt] EnableTorque(OFF) failed for ID {}", id);
    return false;
  }

  WriteLine(std::cerr, "[pan_tilt] Torque disabled for ID {}", id);
  return true;
}

bool PanTiltController::ensureServoMode(const int motor_index) {
  if (!isValidMotorIndex(motor_index)) {
    return false;
  }
  if (motor_bus_mode_[motor_index].load() ==
      static_cast<int>(BusMode::kServo)) {
    return true;
  }

  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  const int id = motorId(motor_index);
  if (!sms_sts_.InitMotor(static_cast<u8>(id), SMS_STS_MODE_SERVO, 1)) {
    WriteLine(std::cerr, "[pan_tilt] Failed to set servo mode for ID {}", id);
    return false;
  }
  motor_bus_mode_[motor_index].store(static_cast<int>(BusMode::kServo));
  invalidateTrackingBaseline(motor_index);
  return true;
}

bool PanTiltController::initMotors() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    // Individual bus transactions fail transiently (stale adapter buffers on
    // reopen, EMI from the motors), so retry before declaring the motor dead.
    bool ok = false;
    for (int attempt = 0; attempt < kInitRetryCount && !ok; ++attempt) {
      if (attempt > 0) {
        WriteLine(std::cerr, "[pan_tilt] InitMotor retry {} for ID {}",
                  attempt, id);
        usleep(kInitRetryDelayUs);
      }
      ok = sms_sts_.InitMotor(static_cast<u8>(id), SMS_STS_MODE_SERVO, 1) != 0;
    }
    if (!ok) {
      WriteLine(std::cerr, "[pan_tilt] InitMotor failed for ID {}", id);
      return false;
    }
    motor_bus_mode_[i].store(static_cast<int>(BusMode::kServo));
    WriteLine(std::cout, "[pan_tilt] InitMotor OK for ID {}", id);
  }
  return true;
}

bool PanTiltController::pingMotors() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    bool ok = false;
    for (int attempt = 0; attempt < kInitRetryCount && !ok; ++attempt) {
      if (attempt > 0) {
        WriteLine(std::cerr, "[pan_tilt] Ping retry {} for ID {}", attempt, id);
        usleep(kInitRetryDelayUs);
      }
      ok = sms_sts_.Ping(static_cast<u8>(id)) == id;
    }
    if (!ok) {
      WriteLine(std::cerr, "[pan_tilt] Ping failed for ID {}", id);
      return false;
    }
    WriteLine(std::cout, "[pan_tilt] Ping OK for ID {}", id);
  }
  return true;
}

bool PanTiltController::ensureFeedback() {
  const auto states = pollState();
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const ServoState &state = states[i];
    if (!state.valid) {
      WriteLine(std::cerr, "[pan_tilt] Feedback validation failed for ID {}",
                state.motor_id);
      return false;
    }
    WriteLine(std::cout,
              "[pan_tilt] Feedback OK for ID {} (pos={} voltage={} temp={})",
              state.motor_id, state.position_ticks, state.voltage_01v,
              state.temperature_celsius);
  }
  return true;
}

bool PanTiltController::runCalibrationOfs() {
  const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
  for (int i = 0; i < kMotorCount; ++i) {
    if (!axes_[i].present) {
      continue;
    }
    const int id = motorId(i);
    if (!sms_sts_.CalibrationOfs(static_cast<u8>(id))) {
      WriteLine(std::cerr, "[pan_tilt] CalibrationOfs failed for ID {}", id);
      return false;
    }
    WriteLine(std::cout, "[pan_tilt] CalibrationOfs OK for ID {}", id);
  }
  return true;
}

bool PanTiltController::isValidMotorIndex(const int motor_index) const {
  if (motor_index < 0 || motor_index >= kMotorCount) {
    WriteLine(std::cerr, "[pan_tilt] Invalid motor index {}", motor_index);
    return false;
  }
  // Single choke point for absent axes: every command path already guards on
  // this, so an unconfigured axis rejects commands instead of talking to a
  // motor ID that is not on the bus.
  if (!axes_[motor_index].present) {
    WriteLine(std::cerr, "[pan_tilt] Motor index {} is not configured present",
              motor_index);
    return false;
  }
  return true;
}

int PanTiltController::motorId(const int motor_index) const {
  return static_cast<int>(axes_[motor_index].motor_id);
}