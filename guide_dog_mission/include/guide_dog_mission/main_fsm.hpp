#pragma once
#include <rclcpp/rclcpp.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

class GuideDogMission : public rclcpp::Node {
public:
    GuideDogMission();
    void run();
private:
    std::shared_ptr<yasmin::StateMachine> main_sm_;

    std::shared_ptr<yasmin_viewer::YasminViewerPub> viewer_pub_;
    
    void setup_state_machine();
};