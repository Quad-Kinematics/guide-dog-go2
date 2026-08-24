#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/blackboard.hpp>
#include <yasmin_ros/action_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include <vector>

class PatrolState : public yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose> {
public:
    PatrolState(rclcpp::Node::SharedPtr node);

    nav2_msgs::action::NavigateToPose::Goal create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;
    std::vector<std::vector<double>> waypoints_;
    int current_index_;
    
    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
};