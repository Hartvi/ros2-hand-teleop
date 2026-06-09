from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _prepend_env(path: str, current: str) -> str:
    return f"{path}:{current}" if current else path


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
                "arm_controller",
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
            description_launch,
            TimerAction(
                period=gazebo_delay,
                actions=[gazebo_launch],
            ),
            TimerAction(
                period=controller_delay,
                actions=controller_spawners,
            ),
        ]
    )
