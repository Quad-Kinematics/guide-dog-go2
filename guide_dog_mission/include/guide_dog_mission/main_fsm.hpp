#pragma once
#include <rclcpp/rclcpp.hpp>
#include <yasmin/state_machine.hpp>

class GuideDogMission : public rclcpp::Node {
public:
    GuideDogMission();
    void run();
private:
    std::shared_ptr<yasmin::StateMachine> main_sm_;
    
    void setup_state_machine();
};