# nav_launch.py — launches full Nav2 stack + patrol_commander
#
# architecture:
#   bt_navigator  = navigation layer (Nav2's default point-nav tree)
#                   provides /navigate_to_pose action server
#   patrol_commander = mission layer (our custom patrol BT)
#                      calls /navigate_to_pose as an action client
#
# bt_navigator must be in both this launch AND lifecycle_manager's node_names
# in nav2_params.yaml — if it's in one but not the other, lifecycle_manager
# either deadlocks (in list but not launched) or bt_navigator never activates
# (launched but not in list)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_autonomous_patrol')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[nav2_params],
        output='screen',
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params],
        output='screen',
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params],
        output='screen',
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params],
        output='screen',
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params],
        output='screen',
    )

    # navigation layer — provides /navigate_to_pose using Nav2's built-in tree
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params],
        output='screen',
    )

    # mission layer — our custom patrol BT, calls Nav2 as a client
    patrol_commander_node = Node(
        package='turtlebot3_autonomous_patrol',
        executable='patrol_commander',
        name='patrol_commander',
        parameters=[{
            'use_sim_time': True,
            'bt_xml_path': os.path.join(
                pkg_dir, 'config', 'patrol_bt.xml'
            )
        }],
        output='screen',
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[nav2_params],
        output='screen',
    )

    # manages lifecycle transitions for all Nav2 nodes
    # every name in node_names (nav2_params.yaml) must match a launched node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[nav2_params],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(planner_node)
    ld.add_action(controller_node)
    ld.add_action(behavior_node)
    ld.add_action(bt_navigator_node)
    # ld.add_action(patrol_commander_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld