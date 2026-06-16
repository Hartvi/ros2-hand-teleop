from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    with open(robot_config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    robot_cfg = config["robot"]
    use_sim_time = as_bool(
        robot_cfg.get("robot_state_publisher", {}).get("use_sim_time", True)
    )

    extra_nodes = [
        Node(
            package="hand_publisher",
            executable="mic_trigger_node",
            name="mic_trigger",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="hand_publisher",
            executable="pose_recorder_node",
            name="pose_recorder",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    ]

    gripper_params = {"use_sim_time": use_sim_time}

    # Preserve the Panda-specific gripper defaults that existed in
    # hand_publisher_launch.py when the robot config does not override them.
    if robot_cfg.get("name") == "panda":
        gripper_params.update(
            {
                "joint_names": [
                    "panda_finger_joint1",
                    "panda_finger_joint2",
                ],
                "joint_multipliers": [1.0, 1.0],
                "q_scale": 1.0,
                "q_max": 0.04,
                "closed_distance": 0.07,
            }
        )

    for key in ("joint_names", "joint_multipliers", "q_scale", "q_max", "closed_distance"):
        if key in robot_cfg:
            gripper_params[key] = robot_cfg[key]

    extra_nodes.append(
        Node(
            package="hand_publisher",
            executable="gripper_publisher",
            name="gripper_publisher",
            output="screen",
            parameters=[gripper_params],
        )
    )

    return extra_nodes


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
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
            "rviz": rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    hand_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "hand_tracking_nodes.launch.py")
        )
    )

    trac_ik_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "trac_ik_controller_nodes.launch.py")
        ),
        launch_arguments={
            "robot_config": robot_config,
            "controller_delay": controller_delay,
        }.items(),
    )

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
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Whether to launch RViz2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Optional path to an RViz config file.",
            ),
            panda_bringup,
            hand_tracking_launch,
            trac_ik_controller_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
