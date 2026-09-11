#include "guide_dog_mission/states/scan_state.hpp"
#include <thread>
#include <chrono>

ScanState::ScanState(rclcpp::Node::SharedPtr node)
    : yasmin::State({"FACE_DETECTED", "NO_FACE"}), node_(node) 
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ScanState::odom_callback, this, std::placeholders::_1));
        
    face_sub_ = node_->create_subscription<guide_dog_interfaces::msg::DetectedFace>(
        "/detected_face", 10, std::bind(&ScanState::face_callback, this, std::placeholders::_1));
}

std::string ScanState::execute(yasmin::Blackboard::SharedPtr blackboard) 
{
    RCLCPP_INFO(node_->get_logger(), "Starting 360-degree scan...");
    
    face_found_ = false;
    accumulated_yaw_ = 0.0;
    first_odom_received_ = false;

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.5; 

    rclcpp::Rate rate(10); 

    // Loop until we rotate a full 2*PI radians, or a face is found
    while (rclcpp::ok() && accumulated_yaw_ < 2 * M_PI && !face_found_) {
        cmd_vel_pub_->publish(cmd);
        rate.sleep();
    }

    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    if (face_found_) {
        RCLCPP_INFO(node_->get_logger(), "Scan interrupted: Face detected.");
        blackboard->set<std::string>("target_name", detected_person_name_);
        return "FACE_DETECTED";
    } else {
        RCLCPP_INFO(node_->get_logger(), "Scan complete. No face found.");
        return "NO_FACE";
    }
}

void ScanState::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

    if (!first_odom_received_) {
        last_yaw_ = current_yaw;
        first_odom_received_ = true;
        return;
    }

    // Calculate delta and handle the Pi to -Pi wrap-around
    double delta_yaw = current_yaw - last_yaw_;
    
    // Normalize delta_yaw to the range [-pi, pi]
    if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

    // Accumulate the absolute distance turned
    accumulated_yaw_ += std::abs(delta_yaw);
    last_yaw_ = current_yaw;
}

void ScanState::face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg) 
{
    if (msg->name != "Unknown" && msg->person_confidence > 0.75) {
        face_found_ = true;
        detected_person_name_ = msg->name;
    }
}

double ScanState::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& msg_q) 
{
    tf2::Quaternion q(msg_q.x, msg_q.y, msg_q.z, msg_q.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}