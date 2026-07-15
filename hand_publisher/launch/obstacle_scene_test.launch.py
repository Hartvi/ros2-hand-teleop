from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


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


def generate_launch_description():
    hand_publisher_share = Path(get_package_share_directory("hand_publisher"))
    default_world_path = hand_publisher_share / "worlds" / "my_world.sdf"

    moveit_config = _build_moveit_config()
    rviz_config_path = Path(str(moveit_config.package_path)) / "launch" / "moveit.rviz"

    world_path = LaunchConfiguration("world_path")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    planning_frame = LaunchConfiguration("planning_frame")
    publish_period = LaunchConfiguration("publish_period")
    use_rviz = LaunchConfiguration("use_rviz")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panda",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    obstacle_scene_node = Node(
        package="ik_node",
        executable="obstacle_scene_node",
        name="obstacle_scene",
        output="screen",
        parameters=[
            {
                "planning_frame": planning_frame,
                "obstacle_topic": obstacle_topic,
            }
        ],
    )

    sdf_obstacle_publisher_node = Node(
        package="hand_publisher",
        executable="sdf_obstacle_publisher",
        name="sdf_obstacle_publisher",
        output="screen",
        parameters=[
            {
                "world_path": world_path,
                "frame_id": planning_frame,
                "obstacle_topic": obstacle_topic,
                "publish_period": publish_period,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config_path)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_path",
                default_value=str(default_world_path),
                description="SDF world file to parse into planning obstacles.",
            ),
            DeclareLaunchArgument(
                "obstacle_topic",
                default_value="/planning_obstacles",
                description="Topic used by the SDF publisher and MoveIt obstacle node.",
            ),
            DeclareLaunchArgument(
                "planning_frame",
                default_value="world",
                description="Frame used for published obstacle poses.",
            ),
            DeclareLaunchArgument(
                "publish_period",
                default_value="10.0",
                description="Seconds between repeated SDF obstacle publications.",
            ),
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="mock_components",
                description="Hardware type passed to the Panda MoveIt xacro.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz to inspect planning scene objects.",
            ),
            static_tf_node,
            robot_state_publisher_node,
            move_group_node,
            TimerAction(period=1.0, actions=[obstacle_scene_node]),
            TimerAction(period=2.0, actions=[sdf_obstacle_publisher_node]),
            rviz_node,
        ]
    )
