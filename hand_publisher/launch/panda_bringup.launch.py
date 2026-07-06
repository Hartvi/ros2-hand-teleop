from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _prepend_env(path: str, current: str) -> str:
    return f"{path}:{current}" if current else path


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).lower() in ("true", "1", "yes", "on")


def launch_setup(context: LaunchContext, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("hand_publisher"))
    robot_config_path = LaunchConfiguration("robot_config").perform(context)
    rviz_enabled = _as_bool(LaunchConfiguration("rviz").perform(context))
    rviz_config_path = LaunchConfiguration("rviz_config").perform(context)

    actions = []

    if rviz_enabled:
        use_sim_time = True

        try:
            import yaml

            with open(robot_config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            use_sim_time = _as_bool(
                config["robot"].get("robot_state_publisher", {}).get("use_sim_time", True)
            )
        except Exception:
            pass

        rviz_arguments = []
        if rviz_config_path:
            rviz_arguments = ["-d", rviz_config_path]

        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=rviz_arguments,
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("hand_publisher"))

    default_robot_config = pkg_share / "config" / "panda_description.yaml"
    default_gazebo_config = pkg_share / "config" / "gazebo.yaml"
    default_controllers = pkg_share / "config" / "panda_controllers.yaml"

    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    ros_lib_path = str(Path("/opt/ros") / ros_distro / "lib")

    robot_config = LaunchConfiguration("robot_config")
    gazebo_config = LaunchConfiguration("gazebo_config")
    controllers_file = LaunchConfiguration("controllers_file")
    headless = LaunchConfiguration("headless")
    gazebo_delay = LaunchConfiguration("gazebo_delay")
    controller_delay = LaunchConfiguration("controller_delay")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "panda_description.launch.py")
        ),
        launch_arguments={"robot_config": robot_config}.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "gazebo_config": gazebo_config,
            "headless": headless,
        }.items(),
    )

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                controllers_file,
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panda_arm_controller",
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                controllers_file,
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                controllers_file,
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=_prepend_env(
                    ros_lib_path, os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
                ),
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=_prepend_env(ros_lib_path, os.environ.get("LD_LIBRARY_PATH", "")),
            ),
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
            description_launch,
            TimerAction(
                period=gazebo_delay,
                actions=[gazebo_launch],
            ),
            TimerAction(
                period=controller_delay,
                actions=controller_spawners,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
