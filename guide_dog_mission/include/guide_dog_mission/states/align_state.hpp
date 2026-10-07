#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/state.hpp>
#include <yasmin/blackboard.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include "guide_dog_mission/announce.hpp"
#include <mutex>

class AlignState : public yasmin::State {
public:
    AlignState(rclcpp::Node::SharedPtr node);
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;
    AnnouncePublisher::SharedPtr announce_pub_;

    // P-Controller Tuning Parameters
    const double K_p_ = 0.5;       // Proportional gain
    const double tolerance_ = 0.1; // Deadband: within 5% of the image width from center
    const double timeout_sec_ = 2.0;

    // Balance limits. PATROL hands over mid-stride (Nav2 just braked) and SCAN
    // mid-spin, so stand still for settle_sec_ before turning in place, then
    // turn gently and only on fresh detections.
    const double settle_sec_ = 1.0;
    const double min_yaw_rate_ = 0.1;   // rad/s; the Go2 steps in place below ~0.05
    const double max_yaw_rate_ = 0.5;   // rad/s
    const double stale_sec_ = 0.5;      // no detection for this long: hold still
    const double max_align_sec_ = 10.0; // stop turning back and forth after this

    // Shared data between threads
    std::mutex data_mutex_;
    double target_offset_x_ = 0.0;
    rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};  // last detection

    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
};