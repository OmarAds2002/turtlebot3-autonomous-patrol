// patrol_commander.cpp — mission-level behavior tree executor
//
// this is the core architectural decision of the project:
// Nav2's bt_navigator runs a per-goal point-navigation tree.
// a patrol mission (loop rooms, check battery, detect objects) is NOT
// a "navigate to one pose" behavior — it's a mission-level loop.
//
// so we run our own process that:
//   1. creates a BT::BehaviorTreeFactory and registers all custom nodes
//   2. puts the ROS2 node on the blackboard so BT nodes can access ROS2
//   3. loads patrol_bt.xml
//   4. ticks the tree at 10 Hz
//
// Nav2 becomes a navigation SERVICE our tree calls (via /navigate_to_pose),
// not the HOST of our tree. bt_navigator still runs — it provides the action
// server using Nav2's default point-nav tree internally.
//
// the MultiThreadedExecutor spins the node in a background thread so action
// clients and subscribers stay responsive while wait_for_action_server blocks

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <thread>

#include "turtlebot3_autonomous_patrol/check_battery.hpp"
#include "turtlebot3_autonomous_patrol/detect_object.hpp"
#include "turtlebot3_autonomous_patrol/approach_object.hpp"
#include "turtlebot3_autonomous_patrol/navigate_to_room.hpp"
#include "turtlebot3_autonomous_patrol/recovery_rotate.hpp"
#include "turtlebot3_autonomous_patrol/wait_and_retry.hpp"
#include "turtlebot3_autonomous_patrol/charge_battery.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("patrol_commander");

  // spin the node in a background thread so action clients and subscribers
  // can process messages while the tree ticks on the main thread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // register all custom BT nodes with the factory
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CheckBattery>("CheckBattery");
  factory.registerNodeType<DetectObject>("DetectObject");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<NavigateToRoom>("NavigateToRoom");
  factory.registerNodeType<RecoveryRotate>("RecoveryRotate");
  factory.registerNodeType<WaitAndRetry>("WaitAndRetry");
  factory.registerNodeType<ChargeBattery>("ChargeBattery");

  // put the ROS2 node on the blackboard — every BT node reads it with:
  // config.blackboard->get<rclcpp::Node::SharedPtr>("node")
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // load the patrol tree from XML
  std::string xml_path =
    node->declare_parameter<std::string>("bt_xml_path", "");
  if (xml_path.empty())
  {
    RCLCPP_FATAL(node->get_logger(), "No bt_xml_path parameter set");
    return 1;
  }

  auto tree = factory.createTreeFromFile(xml_path, blackboard);
  RCLCPP_INFO(node->get_logger(), "Patrol tree loaded from %s", xml_path.c_str());

  // tick the tree at 10 Hz — patrol starts immediately
  // try/catch prevents one bad node from crashing the whole commander
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    try
    {
      tree.tickOnce();
    }
    catch (const BT::BehaviorTreeException& e)
    {
      RCLCPP_ERROR(node->get_logger(), "BT tick error: %s", e.what());
    }
    rate.sleep();
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}