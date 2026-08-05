// approach_object.hpp — BT action node that navigates 0.5m in front of a detected object
// same two-phase async action client pattern as NavigateToRoom, but:
//   - reads a PoseStamped from input port (not x/y)
//   - applies a 0.5m x-offset so the robot stops in front of the object
// fails gracefully (returns FAILURE) if no target_pose on the blackboard

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class ApproachObject : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ApproachObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node_);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "/navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose",
        "Pose of detected object")
    };
  }

  BT::NodeStatus onStart() override
  {
    geometry_msgs::msg::PoseStamped target;
    if (!getInput<geometry_msgs::msg::PoseStamped>("target_pose", target))
    {
      // no pose on blackboard — fail cleanly instead of crashing the tree
      BTLogger::log("ApproachObject: no target_pose set — skipping");
      return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target;
    goal_msg.pose.pose.position.x -= 0.5;  // stop 0.5m short of the object
    goal_msg.pose.header.frame_id = "map";

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      BTLogger::log("ApproachObject: FAILURE — Nav2 server unavailable");
      return BT::NodeStatus::FAILURE;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    BTLogger::log("ApproachObject: approaching object (0.5m offset)");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // phase 1: goal acceptance
    if (goal_future_.valid() &&
        goal_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = goal_future_.get();

      if (!goal_handle_)
      {
        BTLogger::log("ApproachObject: FAILURE — goal rejected");
        return BT::NodeStatus::FAILURE;
      }

      result_future_ = action_client_->async_get_result(goal_handle_);
      goal_future_ = {};
    }

    // phase 2: navigation result
    if (result_future_.valid() &&
        result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      auto result = result_future_.get();

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        BTLogger::log("ApproachObject: SUCCESS — arrived at object");
        return BT::NodeStatus::SUCCESS;
      }
      BTLogger::log("ApproachObject: FAILURE — navigation failed");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (goal_handle_)
    {
      action_client_->async_cancel_goal(goal_handle_);
      BTLogger::log("ApproachObject: halted — goal canceled");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
};