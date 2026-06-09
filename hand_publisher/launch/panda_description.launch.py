from pathlib import Path
import os
import yaml
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value

    if value is None:
        return False

    return str(value).lower() in ("true", "1", "yes", "on")


def resolve_package_path(path_str: str) -> str:
    """
    Supports:
      - /absolute/path/file.xacro
      - ~/file.xacro
      - package://my_package/path/to/file.xacro
    """
    path_str = os.path.expandvars(os.path.expanduser(path_str))

    if path_str.startswith("package://"):
        without_scheme = path_str.removeprefix("package://")
        package_name, relative_path = without_scheme.split("/", 1)
        return str(Path(get_package_share_directory(package_name)) / relative_path)

    return path_str


def resolve_robot_file(package_name: str, relative_path: str) -> str:
    return str(Path(get_package_share_directory(package_name)) / relative_path)


def weld_urdf_to_world(
    urdf_xml: str,
    world_link: str,
    base_link: str,
) -> str:
    """
    Adds:

      world
        └── fixed joint
              └── base_link

    Do this only if your xacro does not already contain a world link/joint.
    """
    root = ET.fromstring(urdf_xml)

    existing_links = {
        elem.attrib["name"] for elem in root.findall("link") if "name" in elem.attrib
    }

    existing_joints = {
        elem.attrib["name"] for elem in root.findall("joint") if "name" in elem.attrib
    }

    if world_link not in existing_links:
        world = ET.Element("link", name=world_link)
        root.insert(0, world)

    joint_name = f"{world_link}_to_{base_link}_fixed"

    if joint_name not in existing_joints:
        fixed_joint = ET.Element("joint", name=joint_name, type="fixed")

        ET.SubElement(fixed_joint, "parent", link=world_link)
        ET.SubElement(fixed_joint, "child", link=base_link)
        ET.SubElement(fixed_joint, "origin", xyz="0 0 0", rpy="0 0 0")

        root.append(fixed_joint)

    return ET.tostring(root, encoding="unicode")


def launch_setup(context: LaunchContext, *args, **kwargs):
    config_path = LaunchConfiguration("robot_config").perform(context)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    robot_cfg = config["robot"]

    robot_name = robot_cfg["name"]

    description_cfg = robot_cfg["description"]
    package_name = description_cfg["package"]
    xacro_relative_path = description_cfg["xacro"]

    xacro_path = resolve_robot_file(package_name, xacro_relative_path)

    if not Path(xacro_path).exists():
        raise RuntimeError(f"Panda xacro does not exist: {xacro_path}")

    raw_xacro_args = robot_cfg.get("xacro_args", {})

    # xacro expects string mappings.
    xacro_args = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in raw_xacro_args.items()
    }

    # Resolve package:// paths inside xacro args where useful.
    for key, value in list(xacro_args.items()):
        if value.startswith("package://"):
            xacro_args[key] = resolve_package_path(value)

    doc = xacro.process_file(
        xacro_path,
        mappings=xacro_args,
    )

    urdf_xml = doc.toxml()

    links_cfg = robot_cfg.get("links", {})
    world_link = links_cfg.get("world", "world")
    base_link = links_cfg.get("base", "base_link")

    if as_bool(robot_cfg.get("weld_to_world", False)):
        urdf_xml = weld_urdf_to_world(
            urdf_xml=urdf_xml,
            world_link=world_link,
            base_link=base_link,
        )

    rsp_cfg = robot_cfg.get("robot_state_publisher", {})
    use_sim_time = as_bool(rsp_cfg.get("use_sim_time", True))

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": urdf_xml,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("hand_publisher"))
    default_config = pkg_share / "config" / "panda_description.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config",
                default_value=str(default_config),
                description="Path to robot description YAML config.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
