#pragma once

#include "yasmin/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class IdleState: public yasmin::State{
    public:
        IdleState(rclcpp::Node::SharedPtr node);

        std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
        std::atomic<bool> start_requested_;

        void handle_start_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};