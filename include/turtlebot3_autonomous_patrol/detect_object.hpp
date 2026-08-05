// detect_object.hpp — BT condition node that checks for target objects
// subscribes to /detections (String stand-in for YOLO output)
// when target found: writes a fixed inspection pose to the blackboard via output port,
// clears the detection so it only fires once per message
// in production, the pose would come from the perception pipeline

#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class DetectObject : public BT::ConditionNode
{
public:
  DetectObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);

    detections_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/detections", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        detected_class_ = msg->data;
      });
  }

  BT::NodeStatus tick() override
  {
    std::string target;
    if (!getInput<std::string>("target", target))
    {
      throw BT::RuntimeError("missing required input [target]");
    }

    if (detected_class_ == target)
    {
      // fixed demo pose — in production this comes from perception
      geometry_msgs::msg::PoseStamped obj_pose;
      obj_pose.header.frame_id = "map";
      obj_pose.pose.position.x = 5.0;
      obj_pose.pose.position.y = 0.5;
      obj_pose.pose.orientation.w = 1.0;

      setOutput("object_pose", obj_pose);

      // consume the detection so it fires once per /detections message
      // without this, the tree re-detects on every tick and never resumes patrol
      detected_class_.clear();

      BTLogger::log("DetectObject: SUCCESS — found '" + target + "', pose set");
      return BT::NodeStatus::SUCCESS;
    }

    BTLogger::log("DetectObject: no target ('" + target + "' not seen)");
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "box", "Target object class to detect"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("object_pose",
        "Pose of the detected object")
    };
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detections_sub_;
  std::string detected_class_;
};