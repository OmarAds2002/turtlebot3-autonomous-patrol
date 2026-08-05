// check_battery.hpp — BT condition node that monitors battery level
// subscribes to /battery_state, stores latest value in member variable
// tick() reads the stored value and returns SUCCESS (>= 20%) or FAILURE (< 20%)
// key pattern: subscriber stores, tick() decides — they are decoupled execution paths

#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config),
      battery_level_(100.0),
      threshold_(20.0)
  {
    // BT nodes are NOT rclcpp::Nodes — pull the shared handle from the blackboard
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);

    // subscriber only stores the value — never returns NodeStatus
    battery_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/battery_state", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        battery_level_ = msg->data;
      });
  }

  BT::NodeStatus tick() override
  {
    if (battery_level_ >= threshold_)
    {
      BTLogger::log("CheckBattery: SUCCESS (battery " +
                    std::to_string(static_cast<int>(battery_level_)) + "%)");
      return BT::NodeStatus::SUCCESS;
    }
    BTLogger::log("CheckBattery: FAILURE — battery LOW (" +
                  std::to_string(static_cast<int>(battery_level_)) + "%)");
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  double battery_level_;
  double threshold_;
};