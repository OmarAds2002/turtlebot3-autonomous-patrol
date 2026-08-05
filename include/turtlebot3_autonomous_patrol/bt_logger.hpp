// bt_logger.hpp — static singleton that publishes BT node status to /bt_log
// all custom nodes call BTLogger::init(node) once, then BTLogger::log("msg") to report
// echo the topic during demos to show the tree orchestrating in real time

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class BTLogger
{
public:
  // call once per node constructor — safe to call multiple times,
  // only creates the publisher on the first call
  static void init(rclcpp::Node::SharedPtr node)
  {
    if (!publisher_)
    {
      publisher_ = node->create_publisher<std_msgs::msg::String>("/bt_log", 10);
      clock_ = node->get_clock();
    }
  }

  static void log(const std::string& msg)
  {
    if (!publisher_) return;

    auto key = msg.substr(0, msg.find(':'));

    if (last_msgs_[key] != msg)
    {
      auto out = std_msgs::msg::String();
      out.data = msg;
      publisher_->publish(out);
      last_msgs_[key] = msg;
    }
}

private:
  static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  static rclcpp::Clock::SharedPtr clock_;
  static std::map<std::string, std::string> last_msgs_;
};

// header-only static member definitions — inline avoids multiple-definition errors
inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr BTLogger::publisher_ = nullptr;
inline rclcpp::Clock::SharedPtr BTLogger::clock_ = nullptr;
inline std::map<std::string, std::string> BTLogger::last_msgs_;