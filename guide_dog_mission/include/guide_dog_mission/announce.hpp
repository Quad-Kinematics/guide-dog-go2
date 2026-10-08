#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// Spoken status lines: tts_node plays them on the Go2 speaker. Fire-and-forget,
// so a state never waits on speech; tts_node keeps only the newest line while it
// is busy. Use the /speak service instead when the state must wait for the words.
using AnnouncePublisher = rclcpp::Publisher<std_msgs::msg::String>;

inline AnnouncePublisher::SharedPtr create_announce_publisher(const rclcpp::Node::SharedPtr & node)
{
    return node->create_publisher<std_msgs::msg::String>("/announce", 1);
}

inline void announce(const AnnouncePublisher::SharedPtr & pub, const std::string & text)
{
    std_msgs::msg::String msg;
    msg.data = text;
    pub->publish(msg);
}

// Drops the line still waiting in tts_node, if any (an empty line plays
// nothing). A state with nothing to announce calls this on entry, so the last
// state's line is not spoken after that state has ended. tts_node handles
// waiting /announce lines before a waiting /speak request, so GREET and ARRIVE
// also need this to keep a stale line from playing ahead of their phrase.
inline void clear_announcement(const AnnouncePublisher::SharedPtr & pub)
{
    announce(pub, "");
}
