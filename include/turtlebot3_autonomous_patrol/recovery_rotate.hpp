// recovery_rotate.hpp — BT action node that spins the robot 360 degrees
// used as a recovery behavior when navigation fails in featureless areas
// publishes cmd_vel directly, bypassing Nav2's controller
// re-publishes every tick to prevent cmd_vel timeout on the robot

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class RecoveryRotate : public BT::StatefulActionNode
{
public:
  RecoveryRotate(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      angular_speed_(1.0),
      rotation_duration_(2.0 * M_PI)  // 6.28s for 360 degrees at 1.0 rad/s
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node_);
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  // first tick: start the spin
  BT::NodeStatus onStart() override
  {
    start_time_ = std::chrono::steady_clock::now();

    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_speed_;
    publisher_->publish(msg);

    BTLogger::log("RecoveryRotate: starting 360 spin to re-localize");
    return BT::NodeStatus::RUNNING;
  }

  // subsequent ticks: keep publishing and check if 360 degrees done
  BT::NodeStatus onRunning() override
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    double seconds = std::chrono::duration<double>(elapsed).count();

    if (seconds >= rotation_duration_)
    {
      // send zero velocity to stop the robot
      auto msg = geometry_msgs::msg::Twist();
      publisher_->publish(msg);
      BTLogger::log("RecoveryRotate: SUCCESS — spin complete");
      return BT::NodeStatus::SUCCESS;
    }

    // keep publishing to prevent cmd_vel timeout
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = angular_speed_;
    publisher_->publish(msg);
    return BT::NodeStatus::RUNNING;
  }

  // tree preempted — stop the robot immediately
  void onHalted() override
  {
    auto msg = geometry_msgs::msg::Twist();
    publisher_->publish(msg);
    BTLogger::log("RecoveryRotate: halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point start_time_;
  double angular_speed_;
  double rotation_duration_;
};