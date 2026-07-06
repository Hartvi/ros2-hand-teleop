import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_context import LaunchContext
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
    return [
        Node(
            package="hand_publisher",
            executable="mic_trigger_node",
            name="mic_trigger",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
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
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")

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

    hand_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "hand_tracking_nodes.launch.py")
        )
    )

    plan_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "plan_controller_nodes.launch.py")
        ),
        launch_arguments={
            "robot_config": robot_config,
            "controller_delay": controller_delay,
            "ros2_control_hardware_type": ros2_control_hardware_type,
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
                "ros2_control_hardware_type",
                default_value="mock_components",
                description="ros2_control hardware interface used by MoveIt Panda config.",
            ),
            panda_bringup,
            hand_tracking_launch,
            plan_controller_launch,
        ]
    )
