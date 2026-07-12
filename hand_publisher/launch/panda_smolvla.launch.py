from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    return [
        Node(
            package="hand_publisher",
            executable="smolvla_control_node",
            name="smolvla_control_node",
            output="screen",
            parameters=[{"task_command": LaunchConfiguration("smolvla_task"), "use_sim_time": use_sim_time}],
        ),
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
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    smolvla_repo_id = LaunchConfiguration("smolvla_repo_id")
    smolvla_device = LaunchConfiguration("smolvla_device")
    smolvla_task = LaunchConfiguration("smolvla_task")
    smolvla_state = LaunchConfiguration("smolvla_state")
    smolvla_image_topics = LaunchConfiguration("smolvla_image_topics")

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

    smolvla_node = Node(
        package="hand_publisher",
        executable="smolvla_node",
        name="smolvla_node",
        output="screen",
        parameters=[{
            "repo_id": smolvla_repo_id,
            "device": smolvla_device,
            "task": smolvla_task,
            "state": smolvla_state,
            "image_topics": ParameterValue(smolvla_image_topics, value_type=str),
        }],
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
            DeclareLaunchArgument(
                "smolvla_repo_id",
                default_value="Hartvi/smolvla",
                description="Hugging Face repo ID or local path for the SmolVLA policy.",
            ),
            DeclareLaunchArgument(
                "smolvla_device",
                default_value="auto",
                description="SmolVLA device: auto, cpu, cuda, or mps.",
            ),
            DeclareLaunchArgument(
                "smolvla_task",
                default_value="Teleop task",
                description="Task instruction passed to SmolVLA.",
            ),
            DeclareLaunchArgument(
                "smolvla_state",
                default_value="",
                description="Optional comma-separated fixed state vector.",
            ),
            DeclareLaunchArgument(
                "smolvla_image_topics",
                default_value='{"observation.images.laptop": "/panda/base_camera/image_raw", "observation.images.phone": "/panda/ee_camera/image_raw"}',
                description="JSON map from SmolVLA image keys to ROS image topics.",
            ),
            panda_bringup,
            OpaqueFunction(function=launch_setup),
            smolvla_node,
        ]
    )
