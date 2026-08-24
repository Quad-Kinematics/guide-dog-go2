#include "guide_dog_mission/states/patrol_state.hpp"

PatrolState::PatrolState(rclcpp::Node::SharedPtr node)
   : yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose>(
          "/navigate_to_pose",
          std::bind(&PatrolState::create_goal_handler, this, std::placeholders::_1)),
      node_(node), current_index_(0)
{
    waypoints_ = {
        {-2.0, -1.0, 0.0},     
        {5.0, 2.0, 0},    
        {-2.0, 4.0, 0.0}      
    };

    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, 
        std::bind(&PatrolState::face_callback, this, std::placeholders::_1)
    );
}

nav2_msgs::action::NavigateToPose::Goal PatrolState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard)
{
    auto coords = waypoints_[current_index_];
    RCLCPP_INFO(node_->get_logger(), "Patrol heading to Waypoint %d [x:%.2f, y:%.2f]", current_index_, coords[0], coords[1]);

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = coords[0];
    goal.pose.pose.position.y = coords[1];
    // (Note: You need a helper function to convert 'yaw' to a quaternion here for orientation.z/w)

    // Increment index for the NEXT time this state is called. Wrap around using modulo.
    current_index_ = (current_index_ + 1) % waypoints_.size();

    return goal;
}

void PatrolState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg)
{
    // If we see Janith while walking...
    if (msg->name != "Unknown") {
        RCLCPP_INFO(node_->get_logger(), "Face detected mid-patrol! Canceling Nav2 goal.");
        
        // This is a built-in YASMIN function that instantly stops the Action Client
        // and forces the state to return "CANCELED", transitioning us directly to ALIGN.
        this->cancel_state(); 
    }
}