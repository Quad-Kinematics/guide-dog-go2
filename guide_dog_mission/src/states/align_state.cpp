#include "guide_dog_mission/states/align_state.hpp"
#include <thread>
#include <chrono>

AlignState::AlignState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"ALIGNED", "LOST_FACE"}), node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, std::bind(&AlignState::face_callback, this, std::placeholders::_1));
}

std::string AlignState::execute(yasmin::Blackboard::SharedPtr blackboard) 
{
    RCLCPP_INFO(node_->get_logger(), "Entering ALIGN state. Centering on target...");
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_seen_time_ = node_->now();
    }

    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(20); // 20 Hz gives smooth P-controller performance

    while (rclcpp::ok()) {
        double current_offset;
        rclcpp::Time last_seen;
        
        // Safely copy the latest data from the camera callback
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_offset = target_offset_x_;
            last_seen = last_seen_time_;
        }

        // 1. Check for Timeout (Lost Face)
        if ((node_->now() - last_seen).seconds() > timeout_sec_) {
            RCLCPP_WARN(node_->get_logger(), "Target lost for %f seconds! Aborting align.", timeout_sec_);
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return "LOST_FACE";
        }

        // 2. Check if Aligned (Success)
        if (std::abs(current_offset) <= tolerance_) {
            RCLCPP_INFO(node_->get_logger(), "Target perfectly aligned!");
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return "ALIGNED";
        }

        // 3. P-Controller Math
        // If offset is positive (right side of image), we must turn right (negative yaw in ROS).
        cmd.angular.z = -current_offset * K_p_;
        
        // Publish velocity and sleep
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }

    return "LOST_FACE"; // Fallback if rclcpp shuts down
}

void AlignState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg) 
{
    if (msg->name != "Unknown") {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_offset_x_ = msg->center_offset_x;
        last_seen_time_ = node_->now(); // Reset the timeout clock
    }
}