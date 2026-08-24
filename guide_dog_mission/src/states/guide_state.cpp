#include "guide_dog_mission/states/guide_state.hpp"

GuideState::GuideState(rclcpp::Node::SharedPtr node)
    : yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose>(
          "/navigate_to_pose",
          std::bind(&GuideState::create_goal_handler, this, std::placeholders::_1)),
      node_(node)
{
}

nav2_msgs::action::NavigateToPose::Goal GuideState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard)
{
    RCLCPP_INFO(node_->get_logger(), "Starting GUIDE state. Escorting target to the Lab.");

    nav2_msgs::action::NavigateToPose::Goal goal;
    
    // Set the reference frame to the global map
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    
    // Hardcoded coordinates for the "Destination"
    goal.pose.pose.position.x = 8.5; 
    goal.pose.pose.position.y = -3.2; 
    
    // Basic orientation (facing straight forward: yaw = 0)
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    return goal;
}