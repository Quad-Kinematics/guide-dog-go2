#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yasmin/state.hpp>
#include <yasmin/blackboard.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include "guide_dog_interfaces/msg/detected_face.hpp"
#include "guide_dog_mission/announce.hpp"
#include <deque>
#include <mutex>
#include <utility>

// Blackboard flag: ALIGN lost the person and SCAN is looking for them again.
// ALIGN sets it on LOST_FACE and clears it on ALIGNED; SCAN clears it on NO_FACE.
inline constexpr char kTargetLostKey[] = "target_lost";

// Turns the robot to face the detected person.
//
// Steering straight on the camera offset was slow: detections come at ~3.7 Hz
// and 0.2-0.5 s late, so the gain had to stay low, and the robot stood still
// between frames. Instead each detection becomes a target heading in the IMU
// yaw frame (the yaw when the frame was taken plus the person's bearing in
// the image), and a controller on the IMU yaw turns toward it at 50 Hz.
// This also brings back a person that SCAN's stop carried out of the frame.
//
// The Go2 does not follow small yaw rates (in-place test 2026-10-09, through
// the cmd_vel bridge): at 0.4-0.5 rad/s it nudges a few degrees and stops; 0.6
// gives ~0.25 rad/s, 0.7 ~0.4, 0.8 ~0.6 after about a second. It starts ~0.35 s
// after the command and coasts 0-6 deg (up to 10) after it. So large errors
// are turned continuously at 0.6-0.8 rad/s, and the last degrees in short
// pulses (0.4 s ~ 3 deg, 0.8 s ~ 9 deg, coast included), each followed by a
// pause until the heading has settled.
class AlignState : public yasmin::State {
public:
    AlignState(rclcpp::Node::SharedPtr node);
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<guide_dog_interfaces::msg::DetectedFace>::SharedPtr face_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    AnnouncePublisher::SharedPtr announce_pub_;

    // RealSense color 1280x720, fx 912: tan(half HFOV) = 640 / 912 (35 deg
    // each side). Turns center_offset_x into a bearing. With another resolution
    // this is off by a factor, which only scales each correction; the next
    // frames still pull the person to the center.
    const double tan_half_fov_ = 0.70;

    // Heading controller. Continuous turn above turn_start_error_ until
    // turn_stop_error_ (the coast takes it further), pulses below that.
    const double K_yaw_ = 1.5;          // rad/s per rad of heading error
    const double min_yaw_rate_ = 0.6;   // rad/s; below ~0.5 the Go2 stops turning
    const double max_yaw_rate_ = 0.8;   // rad/s; SCAN spins at 1.0
    const double turn_start_error_ = 0.26;  // rad (15 deg)
    const double turn_stop_error_ = 0.21;   // rad (12 deg)
    const double pulse_rate_ = 0.8;          // rad/s
    const double pulse_deg_per_sec_ = 14.6;  // a pulse turns about this * (length - pulse_dead_sec_)
    const double pulse_dead_sec_ = 0.21;
    const double min_pulse_sec_ = 0.35;
    const double max_pulse_sec_ = 1.0;       // ~11.5 deg, the continuous turn covers more
    const double settle_sec_ = 0.8;     // hold still this long after a pulse or a turn
    const double heading_tol_ = 0.09;   // rad (5 deg): close enough, a pulse lands +-2-3 deg off
    const double offset_tol_ = 0.15;    // (6 deg) a fresh frame this close to center confirms ALIGNED
    const double settled_rate_ = 0.2;   // rad/s: and neither the robot nor the person (over
                                        // rate_baseline_sec_) is still moving around
    const double rate_baseline_sec_ = 0.4;  // two frames in a row are too noisy

    const double timeout_sec_ = 2.5;    // no detection for this long: LOST_FACE
    const double stale_sec_ = 0.6;      // detections older than this do not confirm ALIGNED
    const double max_align_sec_ = 10.0; // stop following a moving person after this
    const double state_timeout_sec_ = 0.5;  // /sportmodestate older than this: hold still

    // Balance: PATROL hands over mid-stride (Nav2 just braked). Hold zero
    // velocity until the robot has stopped walking and settle_hold_sec_ more,
    // so it never brakes and turns at once. From SCAN it is only turning in
    // place, and the cmd_vel bridge ramps the yaw rate, so it turns at once.
    const double stopped_speed_ = 0.1;    // m/s
    const double settle_hold_sec_ = 0.5;
    const double max_settle_sec_ = 1.5;

    // Shared between the callbacks (executor thread) and execute()
    std::mutex data_mutex_;
    std::deque<std::pair<rclcpp::Time, double>> yaw_history_;  // (receive time, IMU yaw)
    double yaw_ = 0.0;
    double speed_ = 0.0;  // m/s, horizontal
    rclcpp::Time state_time_{0, 0, RCL_ROS_TIME};
    bool have_target_ = false;
    double target_yaw_ = 0.0;
    double target_rate_ = 0.0;  // rad/s the person moves around the robot
    std::deque<std::pair<rclcpp::Time, double>> target_history_;  // (frame stamp, target yaw)
    double target_offset_x_ = 0.0;
    double target_conf_ = 0.0;  // of the last detection, for the log
    rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};  // last detection

    std::string turn_to_target();  // the ALIGNED / LOST_FACE loop
    double yaw_at(const rclcpp::Time & stamp) const;  // needs data_mutex_
    void face_callback(const guide_dog_interfaces::msg::DetectedFace::SharedPtr msg);
    void sport_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg);
};
