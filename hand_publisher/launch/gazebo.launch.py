from pathlib import Path
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value

    if value is None:
        return False

    return str(value).lower() in ("true", "1", "yes", "on")


def resolve_path(path_str: str) -> str:
    """
    Supports:
      - /absolute/path/world.sdf
      - ~/worlds/my_world.sdf
      - package://my_package/worlds/my_world.sdf
    """
    path_str = os.path.expandvars(os.path.expanduser(path_str))

    if path_str.startswith("package://"):
        without_scheme = path_str.removeprefix("package://")
        package_name, relative_path = without_scheme.split("/", 1)
        return str(Path(get_package_share_directory(package_name)) / relative_path)

    return path_str


def bridge_separator(direction: str) -> str:
    """
    ros_gz_bridge syntax:

      @ = bidirectional
      [ = Gazebo -> ROS
      ] = ROS -> Gazebo
    """
    if direction == "bidirectional":
        return "@"

    if direction == "gz_to_ros":
        return "["

    if direction == "ros_to_gz":
        return "]"

    raise RuntimeError(
        f"Unknown bridge direction '{direction}'. "
        "Use one of: bidirectional, gz_to_ros, ros_to_gz"
    )


def make_bridge_argument(topic_cfg: dict) -> str:
    name = topic_cfg["name"]
    ros_type = topic_cfg["ros_type"]
    gz_type = topic_cfg["gz_type"]
    direction = topic_cfg.get("direction", "bidirectional")

    sep = bridge_separator(direction)

    return f"{name}@{ros_type}{sep}{gz_type}"


def launch_setup(context: LaunchContext, *args, **kwargs):
    config_path = LaunchConfiguration("gazebo_config").perform(context)
    headless_override = LaunchConfiguration("headless").perform(context)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    gz_cfg = config["gazebo"]

    world = resolve_path(gz_cfg["world"])
    run_on_start = as_bool(gz_cfg.get("run_on_start", True))

    # headless:=auto means use YAML value.
    if headless_override == "auto":
        headless = as_bool(gz_cfg.get("headless", False))
    else:
        headless = as_bool(headless_override)

    actions = []

    gz_cmd = ["gz", "sim"]

    if run_on_start:
        gz_cmd.append("-r")

    # -s means server-only, no GUI.
    if headless:
        gz_cmd.append("-s")

    gz_cmd.append(world)

    actions.append(
        ExecuteProcess(
            cmd=gz_cmd,
            output="screen",
        )
    )

    bridges_cfg = gz_cfg.get("bridges", {})

    if as_bool(bridges_cfg.get("clock", True)):
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
            )
        )

    topic_bridge_args = [
        make_bridge_argument(topic_cfg) for topic_cfg in bridges_cfg.get("topics", [])
    ]

    if topic_bridge_args:
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_topic_bridge",
                output="screen",
                arguments=topic_bridge_args,
            )
        )

    spawn_cfg = gz_cfg.get("spawn", {})

    if as_bool(spawn_cfg.get("enabled", True)):
        pose = spawn_cfg.get("pose", {})

        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-name",
                    str(spawn_cfg.get("name", "robot")),
                    "-topic",
                    str(spawn_cfg.get("robot_description_topic", "robot_description")),
                    "-x",
                    str(pose.get("x", 0.0)),
                    "-y",
                    str(pose.get("y", 0.0)),
                    "-z",
                    str(pose.get("z", 0.0)),
                    "-R",
                    str(pose.get("roll", 0.0)),
                    "-P",
                    str(pose.get("pitch", 0.0)),
                    "-Y",
                    str(pose.get("yaw", 0.0)),
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("hand_publisher"))
    default_config = pkg_share / "config" / "gazebo.yaml"

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value="/opt/ros/jazzy/lib:" + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ),
            DeclareLaunchArgument(
                "gazebo_config",
                default_value=str(default_config),
                description="Path to Gazebo YAML config file.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="auto",
                description=(
                    "Override Gazebo GUI mode. "
                    "Use true for headless, false for GUI, auto to use YAML."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
