// Copyright 2026 LiveKit
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief ROS 2 driver for the Feetech STS3215 pan/tilt head.
 *
 * Thin rclcpp shim around PanTiltController (originally derived from the
 * LiveKit teleop pan_tilt_demo, since diverged), which owns the serial bus,
 * software angle limits, overcurrent protection, and a velocity deadman
 * watchdog.
 *
 * Axis geometry (motor ID, home tick, travel limits) and presence are
 * per-robot parameters, so a head with only a pan motor is supported via
 * tilt_enabled:=false.
 *
 * Interface (all angles rad from home/center, velocities rad/s):
 *  - sub  pan_tilt_position_cmd  sensor_msgs/JointState (position field;
 *         joints matched by name, missing joints leave that axis alone)
 *  - sub  pan_tilt_velocity_cmd  sensor_msgs/JointState (velocity field)
 *  - pub  joint_states           sensor_msgs/JointState at state_publish_rate
 *  - srv  ~/home                 std_srvs/Trigger (blocking center move)
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pan_tilt_controller.h"

namespace {

constexpr int kPanIndex = 0;
constexpr int kTiltIndex = 1;
constexpr double kTwoPi = 2.0 * PanTiltController::kPi;
constexpr double kServoStepsPerRadian =
    PanTiltController::kTicksPerRevolution / kTwoPi;
// Hardware ceiling for STS3215 wheel-mode speed (steps/s).
constexpr int kServoVelocityLimitStepsPerSec = 3400;

// Same conversion as the demo's radPerSecToServoStepsPerSec
// (pan_tilt_livekit.cpp): rad/s -> clamped servo steps/s.
s16 radPerSecToServoStepsPerSec(const double rad_per_sec) {
  const double unclamped_steps = rad_per_sec * kServoStepsPerRadian;
  const double clamped_steps = std::clamp(
      unclamped_steps, -static_cast<double>(kServoVelocityLimitStepsPerSec),
      static_cast<double>(kServoVelocityLimitStepsPerSec));
  return static_cast<s16>(std::lround(clamped_steps));
}

double degToRad(const double degrees) {
  return degrees * PanTiltController::kPi / 180.0;
}

double stepsPerSecToRadPerSec(const int steps_per_sec) {
  return steps_per_sec * kTwoPi / PanTiltController::kTicksPerRevolution;
}

}  // namespace

class PanTiltDriverNode : public rclcpp::Node {
public:
  PanTiltDriverNode() : Node("pan_tilt_driver") {
    // Serial device of the Waveshare/Feetech servo bus adapter. The chassis
    // ESP32 usually owns /dev/ttyACM0; enumeration order can swap across
    // boots, so a udev rule with a stable symlink is the durable fix.
    declare_parameter("serial_port", "/dev/ttyACM1");
    declare_parameter("baud", PanTiltController::kDefaultBaud);
    declare_parameter("pan_motor_id", 1);
    declare_parameter("tilt_motor_id", 2);
    // Robot models without a tilt motor set tilt_enabled:=false; the axis is
    // then omitted from every bus transaction and from joint_states.
    declare_parameter("pan_enabled", true);
    declare_parameter("tilt_enabled", true);
    // Tick that reads as angle 0 for each axis. Prefer adjusting this over
    // run_calibration_ofs for per-robot mounting variance: it lives in config
    // rather than in the servo's EEPROM.
    declare_parameter("pan_home_ticks", PanTiltController::kPanHomeTicks);
    declare_parameter("tilt_home_ticks", PanTiltController::kTiltHomeTicks);
    // Software travel limits, as an offset from the home tick.
    declare_parameter("pan_min_angle_deg", -75.0);
    declare_parameter("pan_max_angle_deg", 75.0);
    declare_parameter("tilt_min_angle_deg", -90.0);
    declare_parameter("tilt_max_angle_deg", 90.0);
    declare_parameter("pan_joint_name", "pan_joint");
    declare_parameter("tilt_joint_name", "tilt_joint");
    declare_parameter("state_publish_rate", 20.0);  // Hz
    // Runs CalibrationOfs during init (writes the current pose as center to
    // the servo EEPROM) -- bench operation, leave false in normal use.
    declare_parameter("run_calibration_ofs", false);
    // Sweeps the software limit box after init as a startup self-test that the
    // range of motion is unobstructed (four corners with both axes present,
    // two endpoints with one).
    declare_parameter("exercise_limits", true);
    declare_parameter("position_move_speed",
                      static_cast<int>(PanTiltController::kPositionMoveSpeed));
    declare_parameter("velocity_acc",
                      static_cast<int>(PanTiltController::kDefaultMoveAcc));

    const std::string serial_port = get_parameter("serial_port").as_string();
    const int baud = static_cast<int>(get_parameter("baud").as_int());
    const int pan_id = static_cast<int>(get_parameter("pan_motor_id").as_int());
    const int tilt_id =
        static_cast<int>(get_parameter("tilt_motor_id").as_int());
    pan_enabled_ = get_parameter("pan_enabled").as_bool();
    tilt_enabled_ = get_parameter("tilt_enabled").as_bool();
    pan_joint_name_ = get_parameter("pan_joint_name").as_string();
    tilt_joint_name_ = get_parameter("tilt_joint_name").as_string();
    const double state_rate = get_parameter("state_publish_rate").as_double();
    const bool run_calibration_ofs =
        get_parameter("run_calibration_ofs").as_bool();
    const bool exercise_limits = get_parameter("exercise_limits").as_bool();
    const int position_move_speed =
        static_cast<int>(get_parameter("position_move_speed").as_int());
    const int velocity_acc =
        static_cast<int>(get_parameter("velocity_acc").as_int());

    if (serial_port.empty()) {
      throw std::invalid_argument("serial_port must not be empty");
    }
    if (baud <= 0) {
      throw std::invalid_argument("baud must be > 0");
    }
    if (!pan_enabled_ && !tilt_enabled_) {
      throw std::invalid_argument(
          "at least one of pan_enabled/tilt_enabled must be true");
    }
    if ((pan_enabled_ && (pan_id < 0 || pan_id > 253)) ||
        (tilt_enabled_ && (tilt_id < 0 || tilt_id > 253))) {
      throw std::invalid_argument(
          "pan_motor_id/tilt_motor_id must be in [0, 253]");
    }
    // Only a conflict when both axes actually address the bus.
    if (pan_enabled_ && tilt_enabled_ && pan_id == tilt_id) {
      throw std::invalid_argument(
          "pan_motor_id and tilt_motor_id must be distinct");
    }
    if (state_rate <= 0.0) {
      throw std::invalid_argument("state_publish_rate must be > 0");
    }
    if (position_move_speed < 1 ||
        position_move_speed > kServoVelocityLimitStepsPerSec) {
      throw std::invalid_argument("position_move_speed must be in [1, 3400]");
    }
    if (velocity_acc < 0 || velocity_acc > 255) {
      throw std::invalid_argument("velocity_acc must be in [0, 255]");
    }
    position_move_speed_ = static_cast<u16>(position_move_speed);
    velocity_acc_ = static_cast<u8>(velocity_acc);

    std::array<PanTiltController::AxisConfig, PanTiltController::kMotorCount>
        axes;
    axes[kPanIndex] = {
        pan_enabled_,
        static_cast<u8>(pan_id),
        static_cast<int>(get_parameter("pan_home_ticks").as_int()),
        degToRad(get_parameter("pan_min_angle_deg").as_double()),
        degToRad(get_parameter("pan_max_angle_deg").as_double()),
        "pan"};
    axes[kTiltIndex] = {
        tilt_enabled_,
        static_cast<u8>(tilt_id),
        static_cast<int>(get_parameter("tilt_home_ticks").as_int()),
        degToRad(get_parameter("tilt_min_angle_deg").as_double()),
        degToRad(get_parameter("tilt_max_angle_deg").as_double()),
        "tilt"};

    controller_ =
        std::make_unique<PanTiltController>(serial_port, axes, baud);
    if (!controller_->initialize(run_calibration_ofs)) {
      throw std::runtime_error(
          "PanTiltController init failed on " + serial_port +
          " (check port, baud, servo IDs, and the axis limit/home config)");
    }
    if (exercise_limits && !controller_->exerciseLimits()) {
      throw std::runtime_error(
          "exercise_limits self-test failed (range of motion obstructed?)");
    }
    std::string axis_summary;
    if (pan_enabled_) {
      axis_summary = "pan id " + std::to_string(pan_id);
    }
    if (tilt_enabled_) {
      if (!axis_summary.empty()) {
        axis_summary += ", ";
      }
      axis_summary += "tilt id " + std::to_string(tilt_id);
    }
    RCLCPP_INFO(get_logger(), "Pan/tilt initialized and homed on %s (%s)",
                serial_port.c_str(), axis_summary.c_str());

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS());
    position_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "pan_tilt_position_cmd", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          onPositionCmd(*msg);
        });
    // No node-side command watchdog: the controller runs its own deadman
    // thread that halts the motors when velocity commands stop arriving.
    velocity_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "pan_tilt_velocity_cmd", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          onVelocityCmd(*msg);
        });
    // homeMotors() blocks until the move settles, so state publishing pauses
    // during the service call -- acceptable for a recovery operation.
    home_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/home",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          resp->success = controller_->homeMotors();
          resp->message = resp->success ? "homed" : "homeMotors failed";
        });
    state_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / state_rate),
        [this]() { publishState(); });
  }

  ~PanTiltDriverNode() override {
    if (controller_) {
      controller_->haltMotors();
    }
  }

private:
  // Maps a joint name to a motor index, or -1 (with a throttled warning) if
  // the name is not one of the configured joints. A disabled axis is treated
  // as unknown, so commands for a motor this robot does not have are dropped
  // rather than sent to the bus.
  int motorIndexForJoint(const std::string &name) {
    if (pan_enabled_ && name == pan_joint_name_) {
      return kPanIndex;
    }
    if (tilt_enabled_ && name == tilt_joint_name_) {
      return kTiltIndex;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring unknown joint '%s'", name.c_str());
    return -1;
  }

  void onPositionCmd(const sensor_msgs::msg::JointState &msg) {
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      const int idx = motorIndexForJoint(msg.name[i]);
      if (idx < 0) {
        continue;
      }
      // Controller clamps to the per-axis software limits and holds.
      if (!controller_->setMotorAngleFromHome(idx, msg.position[i],
                                              position_move_speed_)) {
        RCLCPP_WARN(get_logger(), "Failed position command for '%s' (%f rad)",
                    msg.name[i].c_str(), msg.position[i]);
      }
    }
  }

  void onVelocityCmd(const sensor_msgs::msg::JointState &msg) {
    for (size_t i = 0; i < msg.name.size() && i < msg.velocity.size(); ++i) {
      const int idx = motorIndexForJoint(msg.name[i]);
      if (idx < 0) {
        continue;
      }
      const s16 steps = radPerSecToServoStepsPerSec(msg.velocity[i]);
      if (!controller_->setVelocity(idx, steps, velocity_acc_)) {
        RCLCPP_WARN(get_logger(),
                    "Failed velocity command for '%s' (%f rad/s, %d steps/s)",
                    msg.name[i].c_str(), msg.velocity[i], steps);
      }
    }
  }

  void publishState() {
    const auto states = controller_->pollState();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    const std::array<const std::string *, PanTiltController::kMotorCount>
        names{&pan_joint_name_, &tilt_joint_name_};
    for (int i = 0; i < PanTiltController::kMotorCount; ++i) {
      if (!states[i].valid) {
        continue;
      }
      msg.name.push_back(*names[i]);
      // angle_from_home_rad is derived from the host-side dead-reckoned
      // tracked_ticks, so it stays truthful through wheel-mode multi-turn
      // count slips that corrupt the raw position register.
      msg.position.push_back(states[i].angle_from_home_rad);
      msg.velocity.push_back(stepsPerSecToRadPerSec(states[i].speed));
    }
    if (msg.name.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No valid servo feedback; skipping joint_states");
      return;
    }
    joint_state_pub_->publish(msg);
  }

  std::string pan_joint_name_;
  std::string tilt_joint_name_;
  bool pan_enabled_{true};
  bool tilt_enabled_{true};
  u16 position_move_speed_{PanTiltController::kPositionMoveSpeed};
  u8 velocity_acc_{PanTiltController::kDefaultMoveAcc};
  std::unique_ptr<PanTiltController> controller_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      position_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      velocity_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<PanTiltDriverNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("pan_tilt_driver"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
