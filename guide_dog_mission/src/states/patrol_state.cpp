#include "guide_dog_mission/states/patrol_state.hpp"

PatrolState::PatrolState(rclcpp::Node::SharedPtr node)
   : yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose>(
          "/navigate_to_pose",
          std::bind(&PatrolState::create_goal_handler, this, std::placeholders::_1)),
      node_(node), current_index_(0), cancel_requested_(false), is_active_(false)
{
    waypoints_ = {
        {-8.0, -5.0, 0.0},     
        {5.0, 2.0, 0},    
        {-2.0, 4.0, 0.0}      
    };

    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, 
        std::bind(&PatrolState::face_callback, this, std::placeholders::_1)
    );
}

std::string PatrolState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
    // Lower the shield: We are walking, listen to the camera
    is_active_ = true;
    cancel_requested_ = false;

    // Run the standard YASMIN Action Client loop
    std::string outcome = yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose>::execute(blackboard);

    // Raise the shield: Walk is over, ignore the camera
    is_active_ = false; 

    return outcome;
}

nav2_msgs::action::NavigateToPose::Goal PatrolState::create_goal_handler(yasmin::Blackboard::SharedPtr blackboard)
{
    cancel_requested_ = false;

    auto coords = waypoints_[current_index_];
    RCLCPP_INFO(node_->get_logger(), "Patrol heading to Waypoint %d [x:%.2f, y:%.2f]", current_index_, coords[0], coords[1]);

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = coords[0];
    goal.pose.pose.position.y = coords[1];
    //Todo: add a helper function to convert 'yaw' to a quaternion here for orientation

    current_index_ = (current_index_ + 1) % waypoints_.size();
    //Todo: add a counter here. Eg: after about 2 iterations robot go to idle state rather than continously patroling and spinning

    return goal;
}

void PatrolState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg)
{
    if (!is_active_ || cancel_requested_) {
        return; 
    }
    // If we see Janith while walking...
    if (msg->name != "Unknown") {
        RCLCPP_INFO(node_->get_logger(), "Face detected mid-patrol! Canceling Nav2 goal.");

        cancel_requested_ = true;
        
        // This is a built-in YASMIN function that instantly stops the Action Client
        this->cancel_state(); 
    }
}