#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/state.hpp>
#include <yasmin/blackboard.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include <mutex>

class AlignState : public yasmin::State {
public:
    AlignState(rclcpp::Node::SharedPtr node);
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;

    // P-Controller Tuning Parameters
    const double K_p_ = 0.5;       // Proportional gain
    const double tolerance_ = 0.05; // Deadband (Stop when error is < 5%)
    const double timeout_sec_ = 5.0;

    // Shared data between threads
    std::mutex data_mutex_;
    double target_offset_x_;
    rclcpp::Time last_seen_time_;

    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
};