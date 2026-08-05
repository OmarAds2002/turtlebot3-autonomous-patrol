// charge_battery.hpp — BT action node stub for battery charging
// in production this would navigate to a charging dock and monitor until full
// for the demo it logs and returns SUCCESS immediately
// uses SyncActionNode because there's nothing to wait for — completes in one tick

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class ChargeBattery : public BT::SyncActionNode
{
public:
  ChargeBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);
  }

  BT::NodeStatus tick() override
  {
    BTLogger::log("ChargeBattery: docking and charging (stub)");
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
};