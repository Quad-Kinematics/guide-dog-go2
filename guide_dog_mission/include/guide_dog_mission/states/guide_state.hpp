#pragma once
#include <rclcpp/rclcpp.hpp>
#include <yasmin/blackboard.hpp>
#include <yasmin_ros/action_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class GuideState : public yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose> {
public:
    GuideState(rclcpp::Node::SharedPtr node);

    nav2_msgs::action::NavigateToPose::Goal create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

private:
    rclcpp::Node::SharedPtr node_;
};