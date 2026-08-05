cmake_minimum_required(VERSION 3.8)
project(turtlebot3_autonomous_patrol)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Library ---
add_library(patrol_bt_nodes SHARED
  src/register_nodes.cpp
)
ament_target_dependencies(patrol_bt_nodes
  behaviortree_cpp
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  geometry_msgs
)
target_include_directories(patrol_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Executable ---
add_executable(patrol_commander src/patrol_commander.cpp)
ament_target_dependencies(patrol_commander
  behaviortree_cpp
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  geometry_msgs
)
target_include_directories(patrol_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Install ---
install(DIRECTORY worlds models launch config
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS patrol_bt_nodes patrol_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()