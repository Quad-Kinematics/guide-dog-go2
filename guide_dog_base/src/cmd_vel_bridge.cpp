#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// Axis order for the smoothing arrays
enum { VX = 0, VY = 1, WZ = 2 };

class CmdVelUnitreeBridge : public rclcpp::Node {
public:
  CmdVelUnitreeBridge()
  : Node("cmd_vel_bridge")
  {
    // Params
    max_vx_   = this->declare_parameter<double>("max_vx",   0.6);
    max_vy_   = this->declare_parameter<double>("max_vy",   0.3);
    max_wz_   = this->declare_parameter<double>("max_wz",   1.2);
    timeout_s_= this->declare_parameter<double>("timeout_s",0.25);

    // Smoothing (smooth:=false restores the old pass-through). Every /cmd_vel
    // source steps its output: Nav2 Foxy RPP turns in place at a fixed rate
    // (its accel limits are a no-op, see guide_dog_navigation's
    // nav2_params_rpp.yaml), the spin recovery and SCAN jump straight to
    // their yaw rate, and the controller stops dead at the goal. Instead of forwarding each message,
    // a timer ramps toward the latest command within these accel limits (open
    // loop, like Nav2's velocity_smoother) and low-pass filters the result.
    // The defaults match DWB's acc_lim_* in nav2_params_dwb.yaml.
    smooth_   = this->declare_parameter<bool>("smooth", true);
    rate_hz_  = this->declare_parameter<double>("rate_hz", 50.0);
    accel_[VX] = this->declare_parameter<double>("accel_vx", 0.8);   // m/s^2
    decel_[VX] = this->declare_parameter<double>("decel_vx", 1.0);
    accel_[VY] = this->declare_parameter<double>("accel_vy", 0.8);
    decel_[VY] = this->declare_parameter<double>("decel_vy", 1.0);
    accel_[WZ] = this->declare_parameter<double>("accel_wz", 1.2);   // rad/s^2
    decel_[WZ] = this->declare_parameter<double>("decel_wz", 1.5);
    // Low-pass time constant (s) on top of the ramp: rounds off the ramp
    // corners and the 20 Hz jitter in Nav2's yaw rate. 0 disables it.
    tau_s_    = this->declare_parameter<double>("smoothing_tau_s", 0.1);
    // Decel multiplier once /cmd_vel goes quiet. Normal stops send a zero
    // (goal reached, cancel, SCAN/ALIGN done) and use decel_*. Silence means
    // a crashed publisher or Foxy RPP's "collision ahead", which throws
    // without publishing a zero, so brake harder there.
    timeout_decel_factor_ = this->declare_parameter<double>("timeout_decel_factor", 2.0);

    pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&CmdVelUnitreeBridge::on_cmd_vel, this, std::placeholders::_1)
    );

    last_cmd_time_ = this->now();
    last_tick_ = std::chrono::steady_clock::now();

    if (smooth_) {
      rate_hz_ = clamp(rate_hz_, 10.0, 200.0);
      watchdog_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&CmdVelUnitreeBridge::smooth_tick, this)
      );
    } else {
      watchdog_ = this->create_wall_timer(
        50ms, std::bind(&CmdVelUnitreeBridge::watchdog, this)
      );
    }

    RCLCPP_INFO(this->get_logger(),
      "cmd_vel -> /api/sport/request bridge running (api_id=1008). Limits: vx<=%.2f vy<=%.2f wz<=%.2f timeout=%.2fs",
      max_vx_, max_vy_, max_wz_, timeout_s_);
    if (smooth_) {
      RCLCPP_INFO(this->get_logger(),
        "Smoothing at %.0f Hz: accel/decel vx %.2f/%.2f vy %.2f/%.2f wz %.2f/%.2f, low-pass %.2fs, "
        "timeout decel x%.1f",
        rate_hz_, accel_[VX], decel_[VX], accel_[VY], decel_[VY], accel_[WZ], decel_[WZ], tau_s_,
        timeout_decel_factor_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Smoothing off: forwarding /cmd_vel as is");
    }
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // clamp() turns NaN into the maximum (both comparisons are false), so a
    // NaN would command full speed. Drop it; if only NaNs arrive, the timeout
    // brings the robot to a stop.
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Ignoring non-finite /cmd_vel (%f, %f, %f)", msg->linear.x, msg->linear.y, msg->angular.z);
      return;
    }

    last_cmd_time_ = this->now();

    double vx = clamp(msg->linear.x,  -max_vx_,  max_vx_);
    double vy = clamp(msg->linear.y,  -max_vy_,  max_vy_);
    double wz = clamp(msg->angular.z, -max_wz_,  max_wz_);

    if (smooth_) {
      target_ = {vx, vy, wz};  // smooth_tick() sends it
      return;
    }
    publish_move(vx, vy, wz, /*noreply=*/true);
  }

  void watchdog() {
    const auto dt = (this->now() - last_cmd_time_).seconds();
    if (dt > timeout_s_) {
      // auto-stop once, then keep quiet
      if (!stopped_) {
        publish_move(0.0, 0.0, 0.0, /*noreply=*/true);
        stopped_ = true;
        RCLCPP_WARN(this->get_logger(), "cmd_vel timeout -> STOP published");
      }
    } else {
      stopped_ = false;
    }
  }

  void smooth_tick() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = clamp(
      std::chrono::duration<double>(now - last_tick_).count(), 0.0, 3.0 / rate_hz_);
    last_tick_ = now;

    const bool fresh = (this->now() - last_cmd_time_).seconds() <= timeout_s_;
    if (!fresh) {
      target_ = {0.0, 0.0, 0.0};  // publisher went quiet: ramp down to a stop
    }
    step(dt, fresh ? 1.0 : timeout_decel_factor_);

    if (fresh) {
      publish_move(out_[VX], out_[VY], out_[WZ], /*noreply=*/true);
      stopped_ = false;
    } else if (!stopped_) {
      // Keep sending the ramp-down; once at zero (this publish is the stop),
      // go quiet so the remote / Unitree app are not overridden
      publish_move(out_[VX], out_[VY], out_[WZ], /*noreply=*/true);
      if (out_[VX] == 0.0 && out_[VY] == 0.0 && out_[WZ] == 0.0) {
        stopped_ = true;
        RCLCPP_WARN(this->get_logger(), "cmd_vel timeout -> STOP published");
      }
    }
  }

  // Move ramp_ toward target_ within the accel limits, then low-pass it into
  // out_. All axes are scaled by the same factor so the velocity changes along
  // a straight line, which keeps the commanded curvature (wz / vx) while
  // speeding up from rest, as Nav2's velocity_smoother scale_velocities does.
  void step(double dt, double decel_factor) {
    std::array<double, 3> dv;
    double scale = 1.0;
    for (int i = 0; i < 3; ++i) {
      dv[i] = target_[i] - ramp_[i];
      const bool speeding_up =
        ramp_[i] * target_[i] >= 0.0 && std::fabs(target_[i]) > std::fabs(ramp_[i]);
      const double max_dv = (speeding_up ? accel_[i] : decel_[i] * decel_factor) * dt;
      if (std::fabs(dv[i]) > max_dv) {
        scale = std::min(scale, max_dv / std::fabs(dv[i]));
      }
    }

    const double alpha = tau_s_ > 0.0 ? 1.0 - std::exp(-dt / tau_s_) : 1.0;
    for (int i = 0; i < 3; ++i) {
      ramp_[i] = scale >= 1.0 ? target_[i] : ramp_[i] + scale * dv[i];
      out_[i] += alpha * (ramp_[i] - out_[i]);
      // Cut the low-pass tail: the Go2 only steps in place below ~0.05, so
      // a long run of tiny commands would just keep it marching
      if (ramp_[i] == 0.0 && std::fabs(out_[i]) < kSnap) {
        out_[i] = 0.0;
      }
    }
  }

  void publish_move(double vx, double vy, double wz, bool noreply) {
    unitree_api::msg::Request req;

    // --- header (matching your observed structure) ---
    // identity.id: use monotonically increasing id (timestamp ns is fine)
    req.header.identity.id = static_cast<uint64_t>(this->now().nanoseconds());
    req.header.identity.api_id = 1008;          // observed move command
    req.header.lease.id = 0;

    req.header.policy.priority = 0;
    req.header.policy.noreply  = noreply;

    // parameter JSON exactly like your echo: {"x":...,"y":...,"z":...}
    std::ostringstream ss;
    ss << std::setprecision(12);
    ss << "{\"x\":" << vx << ",\"y\":" << vy << ",\"z\":" << wz << "}";
    req.parameter = ss.str();

    req.binary.clear(); // observed empty

    pub_->publish(req);
  }

  static constexpr double kSnap = 0.02;  // m/s and rad/s

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  rclcpp::Time last_cmd_time_;
  bool stopped_{false};

  double max_vx_{0.6};
  double max_vy_{0.3};
  double max_wz_{1.2};
  double timeout_s_{0.25};

  bool smooth_{true};
  double rate_hz_{50.0};
  double tau_s_{0.1};
  double timeout_decel_factor_{2.0};
  std::array<double, 3> accel_{{0.8, 0.8, 1.2}};
  std::array<double, 3> decel_{{1.0, 1.0, 1.5}};
  std::array<double, 3> target_{{0.0, 0.0, 0.0}};  // latest /cmd_vel, clamped
  std::array<double, 3> ramp_{{0.0, 0.0, 0.0}};    // accel-limited
  std::array<double, 3> out_{{0.0, 0.0, 0.0}};     // low-passed, sent to the Go2
  std::chrono::steady_clock::time_point last_tick_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelUnitreeBridge>());
  rclcpp::shutdown();
  return 0;
}
