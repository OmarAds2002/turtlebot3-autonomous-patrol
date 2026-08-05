// wait_and_retry.hpp — BT action node that pauses 2 seconds then returns FAILURE
// the FAILURE is intentional — it causes the parent Fallback to retry from the top,
// giving NavigateToRoom another chance after RecoveryRotate re-localizes
// uses StatefulActionNode (not SyncAction) because blocking in tick() would
// freeze the entire tree and stop all other nodes from ticking

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class WaitAndRetry : public BT::StatefulActionNode
{
public:
  WaitAndRetry(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      wait_duration_(2.0)
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    start_time_ = std::chrono::steady_clock::now();
    BTLogger::log("WaitAndRetry: waiting 2s before retry");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    double seconds = std::chrono::duration<double>(elapsed).count();

    if (seconds >= wait_duration_)
    {
      BTLogger::log("WaitAndRetry: FAILURE (triggers retry)");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point start_time_;
  double wait_duration_;
};