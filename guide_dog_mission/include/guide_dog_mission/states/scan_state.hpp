#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/state.hpp>
#include <yasmin/blackboard.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include <atomic>
#include <cmath>

class ScanState : public yasmin::State {
public:
    ScanState(rclcpp::Node::SharedPtr node);
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;

    std::atomic<bool> face_found_;
    
    // Odometry tracking variables
    double last_yaw_;
    double accumulated_yaw_;
    bool first_odom_received_;

    void sport_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg);
    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
};
