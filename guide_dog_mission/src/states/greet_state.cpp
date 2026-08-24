#include "guide_dog_mission/states/greet_state.hpp"
#include <thread>

GreetState::GreetState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"SPOKEN", "FAILED"}), node_(node)
{
    tts_client_ = node_->create_client<guide_dog_interfaces::srv::Speak>("/speak");
}

std::string GreetState::execute(yasmin::Blackboard::SharedPtr blackboard) 
{
    RCLCPP_INFO(node_->get_logger(), "Entering GREET state. Preparing to speak.");

    if (!tts_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(), "TTS Service not available. Skipping greet.");
        return "FAILED";
    }

    auto request = std::make_shared<guide_dog_interfaces::srv::Speak::Request>();
    request->text = "Hello! Please follow me to reach your destination. Make sure to keep a close distance between me and you.";

    auto future = tts_client_->async_send_request(request);

    std::future_status status;
    do {
        if (!rclcpp::ok()) {
            return "FAILED";
        }
        status = future.wait_for(std::chrono::milliseconds(100));
    } while (status != std::future_status::ready);

    if (future.get()->success) {
        RCLCPP_INFO(node_->get_logger(), "Greeting completed successfully.");
        return "SPOKEN";
    } else {
        RCLCPP_WARN(node_->get_logger(), "TTS node reported a failure.");
        return "FAILED";
    }
}