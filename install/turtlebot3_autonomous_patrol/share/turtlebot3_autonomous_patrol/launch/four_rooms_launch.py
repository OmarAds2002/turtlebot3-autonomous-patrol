import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('turtlebot3_autonomous_patrol')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Paths
    world_path = os.path.join(pkg_dir, 'worlds', 'four_rooms.sdf')
    bridge_params = os.path.join(pkg_dir, 'config', 'bridge.yaml')

    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
    pkg_dir, 'models', model_folder, 'model.sdf'
)
    # ---- Gazebo server (physics) ----
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---- Gazebo client (GUI) ----
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---- Robot state publisher (TF from URDF) ----
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ---- Spawn TurtleBot3 in Room 1 ----
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', '2.0',
            '-y', '6.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    # ---- ros_gz_bridge (sensor + cmd topics) ----
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # ---- Image bridge (camera) ----
    image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw', '/camera/depth_image'],
        output='screen',
    )

    # ---- Environment: model paths ----
    set_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models')
    )

    set_custom_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, 'models')
    )

    ld = LaunchDescription()

    # Environment must be set before Gazebo starts
    ld.add_action(set_tb3_models)
    ld.add_action(set_custom_models)

    # Launch Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Spawn robot + publishers
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    # Bridges
    ld.add_action(bridge_cmd)
    ld.add_action(image_bridge_cmd)

    return ld