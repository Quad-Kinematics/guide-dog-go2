#include "guide_dog_mission/states/arrive_state.hpp"
#include <thread>
#include <chrono>

ArriveState::ArriveState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"DONE"}), node_(node)
{
    tts_client_ = node_->create_client<guide_dog_interfaces::srv::Speak>("/speak");
}

std::string ArriveState::execute(yasmin::Blackboard::SharedPtr blackboard) 
{
    RCLCPP_INFO(node_->get_logger(), "Destination reached. Entering ARRIVE state.");

    if (!tts_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node_->get_logger(), "Audio node offline. Skipping spoken arrival.");
        return "DONE"; 
    }

    auto request = std::make_shared<guide_dog_interfaces::srv::Speak::Request>();
    request->text = "We have arrived at your destination. I am going back to IDLE state. Mission accomplished!";

    auto future = tts_client_->async_send_request(request);

    std::future_status status;
    do {
        if (!rclcpp::ok()) {
            return "DONE";
        }
        status = future.wait_for(std::chrono::milliseconds(100));
    } while (status != std::future_status::ready);

    RCLCPP_INFO(node_->get_logger(), "Arrival announcement completed.");
    
    return "DONE";
}