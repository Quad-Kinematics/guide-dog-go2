#include "guide_dog_mission/states/align_state.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
// Wrap an angle to [-pi, pi]
double wrap_angle(double a) { return std::remainder(a, 2.0 * M_PI); }
}

AlignState::AlignState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"ALIGNED", "LOST_FACE"}), node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, std::bind(&AlignState::face_callback, this, std::placeholders::_1));

    sport_state_sub_ = node_->create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate", 10, std::bind(&AlignState::sport_state_callback, this, std::placeholders::_1));

    announce_pub_ = create_announce_publisher(node_);
}

std::string AlignState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
    RCLCPP_INFO(node_->get_logger(), "Entering ALIGN state. Turning to face the target...");
    // Back from SCAN after losing the person: same encounter, so don't say it
    // again (ALIGN and SCAN can take turns several times before the greeting)
    if (blackboard->contains(kTargetLostKey) && blackboard->get<bool>(kTargetLostKey)) {
        clear_announcement(announce_pub_);
    } else {
        announce(announce_pub_, "I see someone. Turning to face you.");
    }

    const std::string outcome = turn_to_target();
    blackboard->set<bool>(kTargetLostKey, outcome == "LOST_FACE");
    return outcome;
}

std::string AlignState::turn_to_target()
{
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(50);

    const rclcpp::Time entry = node_->now();
    {
        // The callbacks run in every state: a target from an earlier mission is not this person
        std::lock_guard<std::mutex> lock(data_mutex_);
        if ((entry - last_seen_time_).seconds() > timeout_sec_) {
            have_target_ = false;
        }
    }

    // 0. Settle: if the robot is still walking (PATROL just cancelled Nav2),
    // hold zero velocity until it has stopped. The zeros also override a late
    // Nav2 command after the cancel.
    bool was_walking = false;
    rclcpp::Time stopped_since = entry;
    while (rclcpp::ok()) {
        const rclcpp::Time now = node_->now();
        double speed;
        rclcpp::Time state_time;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            speed = speed_;
            state_time = state_time_;
        }
        // Without /sportmodestate there is no telling: wait the full max_settle_sec_
        if ((now - state_time).seconds() > state_timeout_sec_ || speed > stopped_speed_) {
            was_walking = true;
            stopped_since = now;
        }
        if (!was_walking || (now - stopped_since).seconds() >= settle_hold_sec_ ||
            (now - entry).seconds() >= max_settle_sec_) {
            break;
        }
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }
    if (was_walking) {
        RCLCPP_INFO(node_->get_logger(), "Stood still for %.1f s before turning.",
                    (node_->now() - entry).seconds());
    }

    const rclcpp::Time track_start = node_->now();

    while (rclcpp::ok()) {
        const rclcpp::Time now = node_->now();
        double yaw, turn_rate, target_yaw, target_rate, offset;
        bool have_target;
        rclcpp::Time state_time, last_seen;

        // Safely copy the latest data from the callbacks
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            yaw = yaw_;
            turn_rate = wrap_angle(yaw_ - yaw_at(now - rclcpp::Duration(std::chrono::milliseconds(100)))) / 0.1;
            state_time = state_time_;
            have_target = have_target_;
            target_yaw = target_yaw_;
            target_rate = target_rate_;
            offset = target_offset_x_;
            last_seen = last_seen_time_;
        }

        const double since_seen = (now - last_seen).seconds();
        const double tracking = (now - track_start).seconds();
        const bool fresh = since_seen <= stale_sec_;

        // 1. Lost: nothing seen for timeout_sec_ (the turn gets that long at least)
        if (std::min(since_seen, tracking) > timeout_sec_) {
            RCLCPP_WARN(node_->get_logger(), "No detection for %.1f s, target lost.", since_seen);
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return "LOST_FACE";
        }

        // 2. Don't follow a moving person forever
        if (tracking > max_align_sec_) {
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            if (fresh) {
                RCLCPP_WARN(node_->get_logger(), "Not centered after %.0f s (offset %.2f), greeting anyway.",
                            max_align_sec_, offset);
                return "ALIGNED";
            }
            return "LOST_FACE";
        }

        if ((now - state_time).seconds() > state_timeout_sec_) {
            // 3. No IMU yaw, so no heading to steer by: hold still
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                  "No /sportmodestate for %.1f s, cannot turn.", (now - state_time).seconds());
            cmd.angular.z = 0.0;
        } else if (!have_target) {
            // 4. Wait for a detection
            cmd.angular.z = 0.0;
        } else {
            const double error = wrap_angle(target_yaw - yaw);
            if (std::abs(error) <= heading_tol_) {
                cmd.angular.z = 0.0;
                // 5. Aligned once a fresh frame shows the person near the center,
                // the robot has stopped turning and the person is not walking on
                if (fresh && std::abs(offset) <= offset_tol_ && std::abs(turn_rate) <= settled_rate_ &&
                    std::abs(target_rate) <= settled_rate_) {
                    RCLCPP_INFO(node_->get_logger(), "Target aligned (offset %.2f) after %.1f s.",
                                offset, (now - entry).seconds());
                    cmd_vel_pub_->publish(cmd);
                    return "ALIGNED";
                }
            } else {
                // 6. P-controller on the heading error. Positive error = target to the
                // left = positive (counterclockwise) yaw rate. At least min_yaw_rate_
                // so the robot does turn instead of stepping in place.
                const double wz = K_yaw_ * error;
                cmd.angular.z = std::copysign(
                    std::clamp(std::abs(wz), min_yaw_rate_, max_yaw_rate_), wz);
            }
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                 "ALIGN: heading error %+.0f deg, cmd %+.2f rad/s, turning %+.2f rad/s, "
                                 "offset %+.2f seen %.1f s ago, person moving %+.2f rad/s",
                                 error * 180.0 / M_PI, cmd.angular.z, turn_rate, offset, since_seen,
                                 target_rate);
        }

        // Publish velocity and sleep
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }

    return "LOST_FACE"; // Fallback if rclcpp shuts down
}

double AlignState::yaw_at(const rclcpp::Time & stamp) const
{
    // Latest sample at or before stamp
    for (auto it = yaw_history_.rbegin(); it != yaw_history_.rend(); ++it) {
        if (it->first <= stamp) {
            return it->second;
        }
    }
    return yaw_history_.empty() ? yaw_ : yaw_history_.front().second;
}

void AlignState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg)
{
    if (msg->name == "Unknown") {
        return;
    }
    const rclcpp::Time now = node_->now();
    rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0) {
        stamp = now;  // unstamped: assume no delay
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    target_offset_x_ = msg->center_offset_x;
    last_seen_time_ = now; // Reset the timeout clock
    if (!yaw_history_.empty()) {
        // Where the person stands: the heading when the frame was taken plus
        // their bearing in it (positive offset = right of center = clockwise)
        const double bearing = -std::atan(msg->center_offset_x * tan_half_fov_);
        const double target = wrap_angle(yaw_at(stamp) + bearing);

        // How fast the person moves around the robot: compare with a frame at
        // least rate_baseline_sec_ older. Unknown (infinite) until there is one.
        while (!target_history_.empty() && (stamp - target_history_.front().first).seconds() > 1.5) {
            target_history_.pop_front();
        }
        target_rate_ = std::numeric_limits<double>::infinity();
        for (auto it = target_history_.rbegin(); it != target_history_.rend(); ++it) {
            const double dt = (stamp - it->first).seconds();
            if (dt >= rate_baseline_sec_) {
                target_rate_ = wrap_angle(target - it->second) / dt;
                break;
            }
        }
        target_history_.emplace_back(stamp, target);

        target_yaw_ = target;
        have_target_ = true;
    }
}

void AlignState::sport_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
    // The robot's clock is minutes off the Jetson's (camera) clock, so key the
    // history by receive time
    const rclcpp::Time now = node_->now();

    std::lock_guard<std::mutex> lock(data_mutex_);
    yaw_ = msg->imu_state.rpy[2];
    speed_ = std::hypot(msg->velocity[0], msg->velocity[1]);
    state_time_ = now;
    yaw_history_.emplace_back(now, yaw_);
    // Detections arrive up to ~0.5 s after their frame
    while ((now - yaw_history_.front().first).seconds() > 2.0) {
        yaw_history_.pop_front();
    }
}
