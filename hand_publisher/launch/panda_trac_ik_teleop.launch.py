from pathlib import Path
import os
import xml.etree.ElementTree as ET

import xacro
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


def resolve_package_path(path_str: str) -> str:
    path_str = os.path.expandvars(os.path.expanduser(path_str))
    if path_str.startswith("package://"):
        without_scheme = path_str.removeprefix("package://")
        package_name, relative_path = without_scheme.split("/", 1)
        return str(Path(get_package_share_directory(package_name)) / relative_path)
    return path_str


def resolve_robot_file(package_name: str, relative_path: str) -> str:
    return str(Path(get_package_share_directory(package_name)) / relative_path)


def weld_urdf_to_world(urdf_xml: str, world_link: str, base_link: str) -> str:
    root = ET.fromstring(urdf_xml)

    existing_links = {
        elem.attrib["name"] for elem in root.findall("link") if "name" in elem.attrib
    }
    existing_joints = {
        elem.attrib["name"] for elem in root.findall("joint") if "name" in elem.attrib
    }

    if world_link not in existing_links:
        root.insert(0, ET.Element("link", name=world_link))

    joint_name = f"{world_link}_to_{base_link}_fixed"
    if joint_name not in existing_joints:
        fixed_joint = ET.Element("joint", name=joint_name, type="fixed")
        ET.SubElement(fixed_joint, "parent", link=world_link)
        ET.SubElement(fixed_joint, "child", link=base_link)
        ET.SubElement(fixed_joint, "origin", xyz="0 0 0", rpy="0 0 0")
        root.append(fixed_joint)

    return ET.tostring(root, encoding="unicode")


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_config_path = LaunchConfiguration("robot_config").perform(context)
    controller_delay = LaunchConfiguration("controller_delay")

    with open(robot_config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    robot_cfg = config["robot"]
    description_cfg = robot_cfg["description"]
    xacro_path = resolve_robot_file(
        description_cfg["package"], description_cfg["xacro"]
    )

    raw_xacro_args = robot_cfg.get("xacro_args", {})
    xacro_args = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in raw_xacro_args.items()
    }
    for key, value in list(xacro_args.items()):
        if value.startswith("package://"):
            xacro_args[key] = resolve_package_path(value)

    doc = xacro.process_file(xacro_path, mappings=xacro_args)
    urdf_xml = doc.toxml()

    links_cfg = robot_cfg.get("links", {})
    base_link = links_cfg.get("base", "base_link")
    tip_link = links_cfg.get("tip", "end_effector_link")
    world_link = links_cfg.get("world", "world")

    if as_bool(robot_cfg.get("weld_to_world", False)):
        urdf_xml = weld_urdf_to_world(urdf_xml, world_link, base_link)

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
                            "timeout": 0.02,
                            "eps": 1e-5,
                            "robot_description": urdf_xml,
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

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "panda_bringup.launch.py")
        ),
        launch_arguments={
            "robot_config": robot_config,
            "gazebo_config": gazebo_config,
            "controllers_file": controllers_file,
            "headless": LaunchConfiguration("headless"),
            "gazebo_delay": LaunchConfiguration("gazebo_delay"),
            "controller_delay": LaunchConfiguration("controller_delay"),
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
            bringup_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
