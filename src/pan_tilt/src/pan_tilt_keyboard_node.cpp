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

/**
 * Keyboard velocity teleop for the pan/tilt driver (run alongside
 * pan_tilt_driver, e.g. with `ros2 run cobra_flex_pan_tilt
 * pan_tilt_keyboard_teleop`).
 *
 * Publishes sensor_msgs/JointState velocity commands (rad/s) on
 * pan_tilt_velocity_cmd, the same interface the driver's deadman watchdog
 * expects: commands are re-published at `publish_rate` while a key is held,
 * and a single zero command is sent when input goes idle so the motors stop
 * crisply instead of waiting for the driver-side deadman.
 *
 * Keys:
 *   a / d : pan left / right
 *   w / s : tilt up / down
 *   q / e : decrease / increase speed scale
 *   space : immediate stop
 *   x     : quit (Ctrl-C also works)
 *
 * A terminal keypress cannot report key-up, so "held" means auto-repeat
 * events keep arriving; releasing a key stops motion after `key_timeout`.
 */

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

/** Puts the controlling terminal in unbuffered, no-echo mode; restores the
 * original settings on destruction. ISIG is left enabled so Ctrl-C still
 * delivers SIGINT for a clean rclcpp shutdown. */
class RawTerminal {
public:
  RawTerminal() {
    ok_ = tcgetattr(STDIN_FILENO, &saved_) == 0;
    if (!ok_) {
      return;
    }
    termios raw = saved_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    ok_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }
  ~RawTerminal() {
    if (ok_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
  }
  RawTerminal(const RawTerminal &) = delete;
  RawTerminal &operator=(const RawTerminal &) = delete;
  bool ok() const { return ok_; }

private:
  termios saved_{};
  bool ok_{false};
};

/** Reads one byte from stdin, waiting at most timeout_ms. Returns -1 if no
 * input arrived. */
int readKey(const int timeout_ms) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) {
    return -1;
  }
  unsigned char c = 0;
  if (read(STDIN_FILENO, &c, 1) != 1) {
    return -1;
  }
  return c;
}

constexpr char kUsage[] =
    "pan/tilt keyboard teleop\n"
    "  a/d: pan left/right   w/s: tilt up/down\n"
    "  q/e: speed down/up    space: stop    x: quit\n";

} // namespace

class PanTiltKeyboardTeleop : public rclcpp::Node {
public:
  PanTiltKeyboardTeleop() : Node("pan_tilt_keyboard_teleop") {
    declare_parameter("pan_joint_name", "pan_joint");
    declare_parameter("tilt_joint_name", "tilt_joint");
    declare_parameter("pan_speed", 1.0);   // rad/s at scale 1.0
    declare_parameter("tilt_speed", 0.5);  // rad/s at scale 1.0
    declare_parameter("publish_rate", 20.0);  // Hz; must beat the driver's
                                              // 300 ms velocity deadman
    declare_parameter("key_timeout", 0.4);  // s without repeats = released

    pan_joint_name_ = get_parameter("pan_joint_name").as_string();
    tilt_joint_name_ = get_parameter("tilt_joint_name").as_string();
    pan_speed_ = get_parameter("pan_speed").as_double();
    tilt_speed_ = get_parameter("tilt_speed").as_double();
    publish_period_ =
        std::chrono::duration<double>(1.0 / std::max(
            get_parameter("publish_rate").as_double(), 1.0));
    key_timeout_ =
        std::chrono::duration<double>(get_parameter("key_timeout").as_double());

    pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "pan_tilt_velocity_cmd", 10);
  }

  /** Blocking key loop; returns when the user quits or rclcpp shuts down. */
  void run() {
    std::fputs(kUsage, stdout);
    std::fflush(stdout);

    auto last_key_time = std::chrono::steady_clock::now();
    auto next_publish = last_key_time;
    bool moving = false;

    while (rclcpp::ok()) {
      const int timeout_ms = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              publish_period_).count());
      const int key = readKey(timeout_ms);
      const auto now = std::chrono::steady_clock::now();

      switch (key) {
      case 'a': pan_dir_ = 1; tilt_dir_ = 0; break;
      case 'd': pan_dir_ = -1; tilt_dir_ = 0; break;
      case 'w': tilt_dir_ = 1; pan_dir_ = 0; break;
      case 's': tilt_dir_ = -1; pan_dir_ = 0; break;
      case 'q':
        scale_ = std::max(scale_ * 0.8, 0.05);
        std::printf("speed scale: %.2f\n", scale_);
        break;
      case 'e':
        scale_ = std::min(scale_ * 1.25, 2.0);
        std::printf("speed scale: %.2f\n", scale_);
        break;
      case ' ': pan_dir_ = 0; tilt_dir_ = 0; break;
      case 'x': publishVelocity(0.0, 0.0); return;
      default: break;
      }
      if (key >= 0) {
        last_key_time = now;
      }

      // Terminals report key-down only via auto-repeat: once repeats stop
      // arriving for key_timeout, treat the key as released.
      if (now - last_key_time > key_timeout_) {
        pan_dir_ = 0;
        tilt_dir_ = 0;
      }

      const bool want_motion = pan_dir_ != 0 || tilt_dir_ != 0;
      if (want_motion && now >= next_publish) {
        publishVelocity(pan_dir_ * pan_speed_ * scale_,
                        tilt_dir_ * tilt_speed_ * scale_);
        next_publish = now + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(publish_period_);
      } else if (!want_motion && moving) {
        // One explicit stop on release; the driver's deadman is the backup.
        publishVelocity(0.0, 0.0);
      }
      moving = want_motion;
    }
  }

private:
  void publishVelocity(const double pan_rad_s, const double tilt_rad_s) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {pan_joint_name_, tilt_joint_name_};
    msg.velocity = {pan_rad_s, tilt_rad_s};
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::string pan_joint_name_;
  std::string tilt_joint_name_;
  double pan_speed_{1.0};
  double tilt_speed_{0.5};
  double scale_{1.0};
  int pan_dir_{0};
  int tilt_dir_{0};
  std::chrono::duration<double> publish_period_{0.05};
  std::chrono::duration<double> key_timeout_{0.4};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  RawTerminal raw_terminal;
  if (!raw_terminal.ok()) {
    std::fprintf(stderr,
                 "stdin is not an interactive terminal; run from a tty\n");
    rclcpp::shutdown();
    return 1;
  }
  auto node = std::make_shared<PanTiltKeyboardTeleop>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
