// navigate_to_room.hpp — BT action node that drives the robot to a map coordinate
// takes x/y input ports, builds a PoseStamped, sends it to Nav2's /navigate_to_pose
// uses the two-phase async pattern:
//   phase 1: async_send_goal → poll goal_future_ in onRunning()
//   phase 2: async_get_result → poll result_future_ in onRunning()
// onHalted() cancels the in-flight goal if the tree preempts (e.g. object detected)

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class NavigateToRoom : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToRoom(const std::string& name, const BT::NodeConfiguration& config)
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
      BT::InputPort<double>("x", "X coordinate in map frame"),
      BT::InputPort<double>("y", "Y coordinate in map frame")
    };
  }

  // first tick: read coordinates, build goal, send to Nav2
  BT::NodeStatus onStart() override
  {
    double x, y;
    if (!getInput<double>("x", x) || !getInput<double>("y", y))
    {
      throw BT::RuntimeError("missing required input [x] or [y]");
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;  // identity quaternion

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20)))
    {
      BTLogger::log("NavigateToRoom: FAILURE — Nav2 server unavailable");
      return BT::NodeStatus::FAILURE;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    BTLogger::log("NavigateToRoom: heading to (" +
                  std::to_string(x) + ", " + std::to_string(y) + ")");
    return BT::NodeStatus::RUNNING;
  }

  // subsequent ticks: poll goal acceptance (phase 1) then result (phase 2)
  BT::NodeStatus onRunning() override
  {
    // phase 1: wait for goal acceptance
    if (goal_future_.valid() &&
        goal_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = goal_future_.get();

      if (!goal_handle_)
      {
        BTLogger::log("NavigateToRoom: FAILURE — goal rejected");
        return BT::NodeStatus::FAILURE;
      }

      // goal accepted — start polling for result
      result_future_ = action_client_->async_get_result(goal_handle_);
      goal_future_ = {};  // invalidate so we don't re-enter this block
    }

    // phase 2: wait for navigation result
    if (result_future_.valid() &&
        result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      auto result = result_future_.get();

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        BTLogger::log("NavigateToRoom: SUCCESS — arrived at room");
        return BT::NodeStatus::SUCCESS;
      }
      BTLogger::log("NavigateToRoom: FAILURE — navigation failed");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  // tree preempted us (e.g. object detected mid-drive) — cancel the Nav2 goal
  void onHalted() override
  {
    if (goal_handle_)
    {
      action_client_->async_cancel_goal(goal_handle_);
      BTLogger::log("NavigateToRoom: halted — goal canceled");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
};