# slam_launch.py — launches slam_toolbox for mapping the 4-room world
# slam_toolbox in ROS2 Jazzy is a lifecycle node — it won't start processing
# until it transitions through configure -> activate
# this launch handles that automatically via LifecycleNode + EmitEvent

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_autonomous_patrol')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    # lifecycle node — starts in 'unconfigured' state
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
        namespace='',
    )

    # auto-configure on launch
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == slam_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # auto-activate once configured (unconfigured -> inactive -> active)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == slam_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(slam_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(rviz_node)

    return ld