from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).lower() in ("true", "1", "yes", "on")


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_config_path = LaunchConfiguration("robot_config").perform(context)
    controller_delay = LaunchConfiguration("controller_delay")

    with open(robot_config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    robot_cfg = config["robot"]
    links_cfg = robot_cfg.get("links", {})
    base_link = links_cfg.get("base", "base_link")
    tip_link = links_cfg.get("tip", "end_effector_link")

    use_sim_time = as_bool(
        robot_cfg.get("robot_state_publisher", {}).get("use_sim_time", True)
    )

    return [
        TimerAction(
            period=controller_delay,
            actions=[
                Node(
                    package="ik_node",
                    executable="trac_ik_node",
                    name="trac_ik",
                    output="screen",
                    parameters=[
                        {
                            "base_link": base_link,
                            "tip_link": tip_link,
                            "ik_service": "/solve_ik",
                            "robot_description_node": "/robot_state_publisher",
                            "timeout": 0.02,
                            "eps": 1e-5,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
                Node(
                    package="hand_publisher",
                    executable="controller_node",
                    name="controller",
                    output="screen",
                    parameters=[
                        {
                            "base_link": base_link,
                            "ik_service": "/solve_ik",
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
                Node(
                    package="hand_publisher",
                    executable="joint_state_merger",
                    name="joint_state_merger",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        )
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("hand_publisher"))

    default_robot_config = pkg_share / "config" / "panda_description.yaml"
    default_gazebo_config = pkg_share / "config" / "gazebo.yaml"
    default_controllers = pkg_share / "config" / "panda_controllers.yaml"

    robot_config = LaunchConfiguration("robot_config")
    gazebo_config = LaunchConfiguration("gazebo_config")
    controllers_file = LaunchConfiguration("controllers_file")
    headless = LaunchConfiguration("headless")
    gazebo_delay = LaunchConfiguration("gazebo_delay")
    controller_delay = LaunchConfiguration("controller_delay")

    panda_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "panda_bringup.launch.py")
        ),
        launch_arguments={
            "robot_config": robot_config,
            "gazebo_config": gazebo_config,
            "controllers_file": controllers_file,
            "headless": headless,
            "gazebo_delay": gazebo_delay,
            "controller_delay": controller_delay,
        }.items(),
    )

    hand_tracking_nodes = [
        Node(
            package="hand_publisher",
            executable="hand_points_node",
            name="hand_points_node",
            output="screen",
        ),
        Node(
            package="hand_publisher",
            executable="hand_publisher_node",
            name="hand_publisher_node",
            output="screen",
        ),
        Node(
            package="hand_publisher",
            executable="hand_frame_node",
            name="hand_frame_node",
            output="screen",
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config",
                default_value=str(default_robot_config),
                description="Path to robot description YAML config.",
            ),
            DeclareLaunchArgument(
                "gazebo_config",
                default_value=str(default_gazebo_config),
                description="Path to Gazebo YAML config file.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(default_controllers),
                description="Path to ros2_control controller YAML file.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="auto",
                description=(
                    "Override Gazebo GUI mode. "
                    "Use true for headless, false for GUI, auto to use YAML."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_delay",
                default_value="2.0",
                description="Seconds to wait before starting Gazebo.",
            ),
            DeclareLaunchArgument(
                "controller_delay",
                default_value="5.0",
                description="Seconds to wait before spawning ros2_control controllers.",
            ),
            panda_bringup,
            *hand_tracking_nodes,
            OpaqueFunction(function=launch_setup),
        ]
    )
