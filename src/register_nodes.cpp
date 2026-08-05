// register_nodes.cpp — single registration point for all custom BT nodes
// BT_REGISTER_NODES expands to a function with a fixed symbol name, so it can only
// appear once per shared library — putting it in multiple .cpp files causes linker
// conflicts and silently drops nodes

#include "turtlebot3_autonomous_patrol/check_battery.hpp"
#include "turtlebot3_autonomous_patrol/detect_object.hpp"
#include "turtlebot3_autonomous_patrol/approach_object.hpp"
#include "turtlebot3_autonomous_patrol/navigate_to_room.hpp"
#include "turtlebot3_autonomous_patrol/recovery_rotate.hpp"
#include "turtlebot3_autonomous_patrol/wait_and_retry.hpp"
#include "turtlebot3_autonomous_patrol/charge_battery.hpp"
#include <behaviortree_cpp/bt_factory.h>

// the string passed here (e.g. "CheckBattery") must match the XML tag exactly
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckBattery>("CheckBattery");
  factory.registerNodeType<DetectObject>("DetectObject");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<NavigateToRoom>("NavigateToRoom");
  factory.registerNodeType<RecoveryRotate>("RecoveryRotate");
  factory.registerNodeType<WaitAndRetry>("WaitAndRetry");
  factory.registerNodeType<ChargeBattery>("ChargeBattery");
}