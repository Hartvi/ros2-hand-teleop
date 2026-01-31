from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from pathlib import Path

USE_RVIZ = "use_rviz"
ROBOT = "robot"

# New args
USE_GZ = "use_gz"
WORLD = "world"
GZ_GUI = "gz_gui"


def generate_launch_description():
    pkg_share = Path(__file__).resolve().parent.parent

    ROBOT_MAPPINGS = {
        "panda": {
            "urdf_path": pkg_share / "urdf" / "robots" / "panda_arm.urdf",
            "base_link": "panda_link0",
            "tip_link": "panda_link8",
        },
        "kinova": {
            "urdf_path": pkg_share
            / "urdf"
            / "gen3_7dof"
            / "urdf"
            / "GEN3_URDF_V12.urdf",
            "base_link": "base_link",
            "tip_link": "end_effector_link",
            "gripper_finger1": "left_inner_finger_pad",
            "gripper_finger2": "right_inner_finger_pad",
            "joint_names": [
                "finger_joint",
                "left_inner_knuckle_joint",
                "right_outer_knuckle_joint",
                "right_inner_knuckle_joint",
                "left_inner_finger_joint",
                "right_inner_finger_joint",
            ],
            "joint_multipliers": [1.0, 1.0, 1.0, 1.0, -1.0, -1.0],
            "q_scale": 10.0,
            "q_max": 1.0,
        },
    }

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

        urdf_path = Path(cfg["urdf_path"])
        if not urdf_path.exists():
            raise RuntimeError(f"URDF path does not exist: {urdf_path}")

        urdf_xml = urdf_path.read_text()

        # If using Gazebo Sim, prefer sim time across nodes.
        common_params = [{"use_sim_time": use_gz}] if use_gz else []

        nodes = []

        # ---- Gazebo Sim (new Gazebo) ----
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
                    output="screen",
                )
            )

            # Bridge clock so ROS uses Gazebo sim time.
            nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_clock_bridge",
                    output="screen",
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
                    output="screen",
                    arguments=[
                        "-name",
                        f"{robot_name}",
                        "-topic",
                        "robot_description",
                        "-x",
                        "0",
                        "-y",
                        "0",
                        "-z",
                        "0.5",
                    ],
                )
            )

        # ---- Your existing stack ----

        # Robot description -> TF tree
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=common_params + [{"robot_description": urdf_xml}],
                remappings=[("joint_states", "/joint_states_merged")],
                output="screen",
            )
        )

        # world -> base_link (static)
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", cfg["base_link"]],
                parameters=common_params,
                output="screen",
            )
        )

        # IK node (TRAC-IK)
        nodes.append(
            Node(
                package="ik_node",
                executable="trac_ik_node",
                name="trac_ik",
                output="screen",
                parameters=common_params
                + [
                    {
                        "base_link": cfg["base_link"],
                        "tip_link": cfg["tip_link"],
                        "timeout": 0.02,
                        "eps": 1e-5,
                        "robot_description": urdf_xml,
                    }
                ],
            )
        )

        nodes += [
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_image_node",
                name="hand_image",
                parameters=common_params,
                output="screen",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_points_node",
                name="hand_points",
                parameters=common_params,
                output="screen",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_publisher_node",
                name="hand_marker",
                parameters=common_params,
                output="screen",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_frame_node",
                name="hand_frame",
                parameters=common_params,
                output="screen",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="controller_node",
                name="controller",
                parameters=common_params + [{"base_link": cfg["base_link"]}],
                output="screen",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="joint_state_merger",
                name="joint_state_merger",
                parameters=common_params,
                output="screen",
            ),
        ]

        # Gripper publisher (only if robot config has it)
        if cfg.get("joint_names"):
            nodes.append(
                Node(
                    package="hand_publisher",
                    namespace="hand_publisher",
                    executable="gripper_publisher",
                    name="gripper_publisher",
                    parameters=common_params
                    + [
                        {
                            "joint_names": cfg["joint_names"],
                            "joint_multipliers": cfg["joint_multipliers"],
                            "q_scale": cfg["q_scale"],
                            "q_max": cfg["q_max"],
                        }
                    ],
                    output="screen",
                )
            )

        # RViz (conditional)
        if use_rviz:
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=common_params,
                )
            )

        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                USE_RVIZ, default_value="true", description="Whether to launch RViz"
            ),
            DeclareLaunchArgument(
                ROBOT, default_value="kinova", description="Robot config key"
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
