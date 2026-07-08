from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import Node


def as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).lower() in ("true", "1", "yes", "on")


def _build_moveit_config():
    return (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


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

    moveit_config = _build_moveit_config()

    return [
        TimerAction(
            period=controller_delay,
            actions=[
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
                    arguments=["--ros-args", "--log-level", "info"],
                ),
                Node(
                    package="ik_node",
                    executable="plan_node",
                    output="screen",
                    parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.planning_pipelines,
                        moveit_config.joint_limits,
                        {
                            "base_link": base_link,
                            "tip_link": tip_link,
                            "ik_service": "/plan_and_execute",
                            "move_group_name": "panda_arm",
                            "robot_description_node": "/robot_state_publisher",
                            "timeout": 0.02,
                            "eps": 1e-5,
                            "use_sim_time": use_sim_time,
                        },
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
                            "service_type": "PlanPose",
                            "ik_service": "/plan_and_execute",
                            "request_mode": "on_stop",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config",
                default_value=str(default_robot_config),
                description="Path to robot description YAML config.",
            ),
            DeclareLaunchArgument(
                "controller_delay",
                default_value="5.0",
                description="Seconds to wait before starting the IK and controller nodes.",
            ),
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="mock_components",
                description="ros2_control hardware interface used by MoveIt Panda config.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
