#include "guide_dog_mission/states/align_state.hpp"
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

AlignState::AlignState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"ALIGNED", "LOST_FACE"}), node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, std::bind(&AlignState::face_callback, this, std::placeholders::_1));

    announce_pub_ = create_announce_publisher(node_);
}

std::string AlignState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
    RCLCPP_INFO(node_->get_logger(), "Entering ALIGN state. Stopping, then centering on target...");
    announce(announce_pub_, "I see someone. Turning to face you.");

    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(20); // 20 Hz gives smooth P-controller performance

    // 0. Settle: hold zero velocity until the robot has stopped walking or
    // spinning, so it never brakes and turns at the same time. The zeros also
    // override a late Nav2 command after the cancel.
    const rclcpp::Time settle_start = node_->now();
    while (rclcpp::ok() && (node_->now() - settle_start).seconds() < settle_sec_) {
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }

    const rclcpp::Time track_start = node_->now();

    while (rclcpp::ok()) {
        double current_offset;
        rclcpp::Time last_seen;

        // Safely copy the latest data from the camera callback
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_offset = target_offset_x_;
            last_seen = last_seen_time_;
        }

        const rclcpp::Time now = node_->now();
        const double since_seen = (now - last_seen).seconds();
        const double tracking = (now - track_start).seconds();

        // 1. Check for Timeout (Lost Face): nothing seen since we stopped
        if (std::min(since_seen, tracking) > timeout_sec_) {
            RCLCPP_WARN(node_->get_logger(), "Target lost for %f seconds! Aborting align.", timeout_sec_);
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return "LOST_FACE";
        }

        // 2. Don't keep turning back and forth: the person is in view, close enough
        if (tracking > max_align_sec_) {
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            if (since_seen <= stale_sec_) {
                RCLCPP_WARN(node_->get_logger(), "Not centered after %.0f s (offset %.2f), greeting anyway.",
                            max_align_sec_, current_offset);
                return "ALIGNED";
            }
            return "LOST_FACE";
        }

        if (since_seen > stale_sec_) {
            // 3. No fresh detection: hold still instead of turning on an old offset
            cmd.angular.z = 0.0;
        } else if (std::abs(current_offset) <= tolerance_) {
            // 4. Check if Aligned (Success)
            RCLCPP_INFO(node_->get_logger(), "Target perfectly aligned!");
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return "ALIGNED";
        } else {
            // 5. P-Controller Math
            // If offset is positive (right side of image), we must turn right (negative yaw in ROS).
            // Limited to [min_yaw_rate_, max_yaw_rate_] so the robot turns gently
            // but does turn (below the minimum it only steps in place).
            const double wz = -current_offset * K_p_;
            cmd.angular.z = std::copysign(
                std::clamp(std::abs(wz), min_yaw_rate_, max_yaw_rate_), wz);
        }

        // Publish velocity and sleep
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }

    return "LOST_FACE"; // Fallback if rclcpp shuts down
}

void AlignState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg) 
{
    if (msg->name != "Unknown") {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_offset_x_ = msg->center_offset_x;
        last_seen_time_ = node_->now(); // Reset the timeout clock
    }
}