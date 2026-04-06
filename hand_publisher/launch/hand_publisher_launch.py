from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from hand_publisher_node.utils import inject_gz_ros2_control
import xacro
import os

USE_RVIZ = "use_rviz"
ROBOT = "robot"

# New args
USE_GZ = "use_gz"
WORLD = "world"
GZ_GUI = "gz_gui"

# Nodes and Packages
HAND_PUBLISHER_PKG = "hand_publisher"
CONTROLLER_MANAGER_PKG = "controller_manager"
CONTROLLER_MANAGER_NODE = "/controller_manager"

# String Constants
SCREEN = "screen"
ROBOT_DESCRIPTION = "robot_description"
BASE_LINK = "base_link"
TIP_LINK = "tip_link"
URDF_PATH = "urdf_path"
URDF_XML = "urdf_xml"
JOINT_NAMES = "joint_names"
JOINT_MULTIPLIERS = "joint_multipliers"
Q_SCALE = "q_scale"
Q_MAX = "q_max"
WORLD_STR = "world"


def get_moveit_xml(robot_name: str, weld_to_world: bool = False):
    share = Path(get_package_share_directory(HAND_PUBLISHER_PKG))
    xacro_path = share / "urdf" / "urdf" / f"{robot_name}.urdf.xacro"

    urdf_xml = xacro.process_file(str(xacro_path)).toxml()  # type: ignore

    if weld_to_world:
        # Inject a world link and fixed joint
        import xml.etree.ElementTree as ET

        root = ET.fromstring(urdf_xml)

        # Add world link
        world_link = ET.Element("link", name=WORLD_STR)
        root.insert(0, world_link)

        # Add fixed joint from world to base
        fixed_joint = ET.Element("joint", name="world_to_base_fixed", type="fixed")
        parent = ET.SubElement(fixed_joint, "parent", link=WORLD_STR)
        child = ET.SubElement(fixed_joint, "child", link="panda_link0")
        origin = ET.SubElement(fixed_joint, "origin", xyz="0 0 0", rpy="0 0 0")
        root.append(fixed_joint)

        urdf_xml = ET.tostring(root, encoding="unicode")

    return urdf_xml


def generate_launch_description():
    # pkg_share = Path(__file__).resolve().parent.parent

    ROBOT_MAPPINGS = {
        "panda": {
            URDF_XML: lambda: get_moveit_xml("panda", True),
            # URDF_PATH: lambda: get_moveit_xml("panda"),
            BASE_LINK: "panda_link0",
            TIP_LINK: "panda_link8",
            JOINT_NAMES: [
                "panda_finger_joint1",
                "panda_finger_joint2",
            ],
            JOINT_MULTIPLIERS: [1.0, 1.0],
            Q_SCALE: 1.0,
            Q_MAX: 1.0,
            "closed_distance": 0.07,
        },
        # "kinova": {
        #     URDF_PATH: pkg_share
        #     / "urdf"
        #     / "gen3_7dof"
        #     / "urdf"
        #     / "GEN3_URDF_V12.urdf",
        #     BASE_LINK: "base_link",
        #     TIP_LINK: "end_effector_link",
        #     "gripper_finger1": "left_inner_finger_pad",
        #     "gripper_finger2": "right_inner_finger_pad",
        #     JOINT_NAMES: [
        #         "finger_joint",
        #         "left_inner_knuckle_joint",
        #         "right_outer_knuckle_joint",
        #         "right_inner_knuckle_joint",
        #         "left_inner_finger_joint",
        #         "right_inner_finger_joint",
        #     ],
        #     JOINT_MULTIPLIERS: [1.0, 1.0, 1.0, 1.0, -1.0, -1.0],
        #     Q_SCALE: 10.0,
        #     Q_MAX: 1.0,
        # },
    }

    pkg_share = Path(get_package_share_directory(HAND_PUBLISHER_PKG))
    controllers_yaml = pkg_share / "config" / "panda_controllers.yaml"

    # raise RuntimeError(f"CONTROLLER: {controllers_yaml}")

    def launch_setup(context: LaunchContext, *args, **kwargs):
        robot_name = LaunchConfiguration(ROBOT).perform(context)
        use_rviz_val = LaunchConfiguration(USE_RVIZ).perform(context).lower()
        use_gz_val = LaunchConfiguration(USE_GZ).perform(context).lower()
        gz_gui_val = LaunchConfiguration(GZ_GUI).perform(context).lower()
        world_val = LaunchConfiguration(WORLD).perform(context)

        def is_true(v: str) -> bool:
            return v in ("true", "1", "yes", "on")

        use_rviz = is_true(use_rviz_val)
        use_gz = is_true(use_gz_val)
        gz_gui = is_true(gz_gui_val)

        if robot_name not in ROBOT_MAPPINGS:
            raise RuntimeError(
                f"Unknown robot='{robot_name}'. Choose one of: {list(ROBOT_MAPPINGS.keys())}"
            )

        cfg = ROBOT_MAPPINGS[robot_name]

        if cfg.get(URDF_PATH):
            urdf_path = Path(cfg[URDF_PATH])
            if not urdf_path.exists():
                raise RuntimeError(f"URDF path does not exist: {urdf_path}")

            urdf_xml = urdf_path.read_text()
        elif cfg.get(URDF_XML):
            urdf_xml = cfg[URDF_XML]()  # lazy eval

            urdf_xml = inject_gz_ros2_control(
                urdf_xml, robot_name, str(controllers_yaml)
            )

        else:
            raise RuntimeError(
                f"Unknown robot option: {cfg.keys()} must "
                f'contain one of the following: ["{URDF_XML}", "{URDF_PATH}"]'
            )

        # If using Gazebo Sim, prefer sim time across nodes.
        common_params = [{"use_sim_time": use_gz}] if use_gz else []

        nodes = []

        if use_gz:
            # Start Gazebo Sim server (+ GUI optionally) with the chosen world.
            # -r = run immediately
            # -s = server-only (no GUI)
            gz_cmd = ["gz", "sim", "-r"]
            if not gz_gui:
                gz_cmd.append("-s")
            if world_val:
                gz_cmd.append(world_val)

            nodes.append(
                ExecuteProcess(
                    cmd=gz_cmd,
                    output=SCREEN,
                )
            )

            # Bridge clock so ROS uses Gazebo sim time.
            nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_clock_bridge",
                    output=SCREEN,
                    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                )
            )

            # Spawn the robot from /robot_description into Gazebo.
            # This is the simplest way to get your URDF into Gazebo (it will internally convert).
            nodes.append(
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_robot",
                    output=SCREEN,
                    arguments=[
                        "-name",
                        f"{robot_name}",
                        "-topic",
                        ROBOT_DESCRIPTION,
                        "-x",
                        "0",
                        "-y",
                        "0",
                        "-z",
                        "0.5",
                    ],
                )
            )

        # Robot description -> TF tree
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=common_params + [{ROBOT_DESCRIPTION: urdf_xml}],
                output=SCREEN,
            )
        )

        # world -> base_link (static)
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", WORLD_STR, cfg[BASE_LINK]],
                parameters=common_params,
                output=SCREEN,
            )
        )

        # IK node (TRAC-IK)
        nodes.append(
            Node(
                package="ik_node",
                executable="trac_ik_node",
                name="trac_ik",
                output=SCREEN,
                parameters=common_params
                + [
                    {
                        BASE_LINK: cfg[BASE_LINK],
                        TIP_LINK: cfg[TIP_LINK],
                        "timeout": 0.02,
                        "eps": 1e-5,
                        ROBOT_DESCRIPTION: urdf_xml,
                    }
                ],
            )
        )

        nodes += [
            Node(
                package=HAND_PUBLISHER_PKG,
                namespace=HAND_PUBLISHER_PKG,
                executable="hand_points_node",
                name="hand_points",
                parameters=common_params,
                output=SCREEN,
            ),
            Node(
                package=HAND_PUBLISHER_PKG,
                namespace=HAND_PUBLISHER_PKG,
                executable="hand_publisher_node",
                name="hand_marker",
                parameters=common_params,
                output=SCREEN,
            ),
            Node(
                package=HAND_PUBLISHER_PKG,
                namespace=HAND_PUBLISHER_PKG,
                executable="hand_frame_node",
                name="hand_frame",
                parameters=common_params,
                output=SCREEN,
            ),
            Node(
                package=HAND_PUBLISHER_PKG,
                namespace=HAND_PUBLISHER_PKG,
                executable="controller_node",
                name="controller",
                parameters=common_params + [{BASE_LINK: cfg[BASE_LINK]}],
                output=SCREEN,
            ),
            Node(
                package=HAND_PUBLISHER_PKG,
                namespace=HAND_PUBLISHER_PKG,
                executable="joint_state_merger",
                name="joint_state_merger",
                parameters=common_params,
                output=SCREEN,
            ),
            Node(
                package=CONTROLLER_MANAGER_PKG,
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    CONTROLLER_MANAGER_NODE,
                    "--param-file",
                    str(controllers_yaml),
                ],
                output=SCREEN,
            ),
            Node(
                package=CONTROLLER_MANAGER_PKG,
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    CONTROLLER_MANAGER_NODE,
                    "--param-file",
                    str(controllers_yaml),
                ],
                output=SCREEN,
            ),
            Node(
                package=CONTROLLER_MANAGER_PKG,
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    CONTROLLER_MANAGER_NODE,
                    "--param-file",
                    str(controllers_yaml),
                ],
                output=SCREEN,
            ),
        ]

        # Gripper publisher (only if robot config has it)
        if cfg.get(JOINT_NAMES):
            nodes.append(
                Node(
                    package=HAND_PUBLISHER_PKG,
                    namespace=HAND_PUBLISHER_PKG,
                    executable="gripper_publisher",
                    name="gripper_publisher",
                    parameters=common_params
                    + [
                        {
                            JOINT_NAMES: cfg[JOINT_NAMES],
                            JOINT_MULTIPLIERS: cfg[JOINT_MULTIPLIERS],
                            Q_SCALE: cfg[Q_SCALE],
                            Q_MAX: cfg[Q_MAX],
                        }
                    ],
                    output=SCREEN,
                )
            )

        # RViz (conditional)
        if use_rviz:
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output=SCREEN,
                    parameters=common_params,
                )
            )

        return nodes

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value="/opt/ros/jazzy/lib:"
                + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/opt/ros/jazzy/lib:" + os.environ.get("LD_LIBRARY_PATH", ""),
            ),
            DeclareLaunchArgument(
                USE_RVIZ, default_value="true", description="Whether to launch RViz"
            ),
            DeclareLaunchArgument(
                ROBOT, default_value="panda", description="Robot config key"
            ),
            DeclareLaunchArgument(
                USE_GZ,
                default_value="false",
                description="Whether to launch Gazebo Sim (new Gazebo)",
            ),
            DeclareLaunchArgument(
                GZ_GUI,
                default_value="true",
                description="Whether to launch Gazebo GUI (if use_gz:=true)",
            ),
            DeclareLaunchArgument(
                WORLD,
                default_value="",
                description="Path to SDF world file (empty uses Gazebo default)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
