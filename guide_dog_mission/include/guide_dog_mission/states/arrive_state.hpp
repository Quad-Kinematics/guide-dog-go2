#pragma once
#include <rclcpp/rclcpp.hpp>
#include <yasmin/state.hpp>
#include <yasmin/blackboard.hpp>
#include "guide_dog_interfaces/srv/speak.hpp"

class ArriveState : public yasmin::State {
public:
    ArriveState(rclcpp::Node::SharedPtr node);
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<guide_dog_interfaces::srv::Speak>::SharedPtr tts_client_;
};