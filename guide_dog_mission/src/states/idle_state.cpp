#include "guide_dog_mission/states/idle_state.hpp"

IdleState::IdleState(rclcpp::Node::SharedPtr node):yasmin::State({"START"}), node_(node), start_requested_(false){
    service_ = node_->create_service<std_srvs::srv::Trigger>("/start_mission", std::bind(&IdleState::handle_start_request, this, std::placeholders::_1, std::placeholders::_2));
}

std::string IdleState::execute(yasmin::Blackboard::SharedPtr blackboard) 
{
    RCLCPP_INFO(node_->get_logger(), "Robot is IDLE. Waiting for /start_mission service call...");
    
    start_requested_ = false;

    // Loop until the flag is flipped by the service callback
    while (rclcpp::ok() && !start_requested_) { //Srinath asked to check whether we have a better approch than this or not.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node_->get_logger(), "Start Mission received! Transitioning to PATROL.");
    return "START";
}


void IdleState::handle_start_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
    start_requested_ = true; 
    
    response->success = true;
    response->message = "Mission authorized. Dog is standing up.";
}