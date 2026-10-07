#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/blackboard.hpp>
#include <yasmin_ros/action_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include "guide_dog_mission/announce.hpp"
#include <vector>

class PatrolState : public yasmin_ros::ActionState<nav2_msgs::action::NavigateToPose> {
public:
    PatrolState(rclcpp::Node::SharedPtr node);

    nav2_msgs::action::NavigateToPose::Goal create_goal_handler(yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;
    AnnouncePublisher::SharedPtr announce_pub_;
    std::vector<std::vector<double>> waypoints_;
    int current_index_;
    std::atomic<bool> cancel_requested_;
    std::atomic<bool> is_active_;

    // Debounce: stop the walk only after min_detections_ detections in a row,
    // each within max_detection_gap_sec_ of the previous one. Cancelling brakes
    // the robot mid-stride, so a single false positive should not do it.
    const int min_detections_ = 3;
    const double max_detection_gap_sec_ = 1.0;
    std::atomic<int> detection_count_;
    rclcpp::Time last_detection_time_;  // executor thread only

    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
};