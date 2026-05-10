import os
import time
import unittest
import pytest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

try:
    import launch_testing
    import launch_testing.actions
except ImportError:
    launch_testing = None


# Helper function to configure MoveIt 2.
# It loads URDF, SRDF, planning pipelines, and trajectory execution settings.
def _build_moveit_config():
    return (
        MoveItConfigsBuilder("moveit_resources_panda")
        # Load the robot description from URDF/XACRO.
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                # maps the hardware interface type (mock vs real)
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        # Load Semantic Robot Description (SRDF) for planning groups and poses.
        .robot_description_semantic(file_path="config/panda.srdf")
        # Publish the robot description so other nodes can use the model.
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        # Define the controllers used for executing planned trajectories.
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        # Use OMPL as the default motion planning library.
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    )


# Construct the launch entities (nodes and arguments) for the MoveIt stack.
def _build_launch_entities():
    moveit_config = _build_moveit_config()
    ros2_controllers_path = os.path.join(
        str(moveit_config.package_path), "config", "ros2_controllers.yaml"
    )
    rviz_config_path = os.path.join(
        str(moveit_config.package_path), "launch", "moveit.rviz"
    )

    declared_arguments = [
        # Selects the hardware backend: 'mock_components' for simulation, 'panda' for real hardware.
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="mock_components",
            description="ros2_control hardware interface used for Panda MoveIt bring-up",
        ),
        # Sets the default MoveIt planning group (e.g., panda_arm).
        DeclareLaunchArgument(
            "move_group_name",
            default_value="panda_arm",
            description="MoveIt planning group used by the planning service",
        ),
        # Toggles the RViz visualization window.
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz alongside the MoveIt test stack",
        ),
    ]

    # The move_group node is the main executor for MoveIt planning and execution.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Publishes the 3D poses (TF) of the robot links based on joint states and URDF.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Fixes the robot base (panda_link0) to the map/world coordinate frame.
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panda",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # The controller manager handles the lifecycle of ros2_control controllers.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    # Publishes current joint angles to the /joint_states topic.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Controller for the 7-DOF Panda arm.
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Controller for the Panda gripper fingers.
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    # Provides the custom /plan_and_execute service for the teleoperation system.
    plan_node = Node(
        package="ik_node",
        executable="plan_node",
        name="planning_service",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"move_group_name": LaunchConfiguration("move_group_name")},
        ],
    )

    # RViz2 visualization tool, configured with MoveIt and robot model parameters.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    entities = declared_arguments + [
        static_tf_node,
        robot_state_publisher_node,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        plan_node,
        rviz_node,
    ]

    return entities, {"plan_node": plan_node}


def generate_launch_description():
    entities, _ = _build_launch_entities()
    return LaunchDescription(entities)


@pytest.mark.launch_test
def generate_test_description():
    if launch_testing is None:
        raise RuntimeError("launch_testing is required to run this launch test")

    entities, context = _build_launch_entities()
    return LaunchDescription(entities + [launch_testing.actions.ReadyToTest()]), context


if launch_testing is not None:
    import rclpy
    from hand_publisher_interfaces.srv import PlanPose
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformException, TransformListener


class TestPlanAndExecuteService(unittest.TestCase):

    def test_plan_and_execute_service(self, proc_output, plan_node):
        """Ensure /plan_and_execute can plan and execute."""
        z_offset_list = [0.02, 0.0]

        rclpy.init()
        node = rclpy.create_node("moveit_control_launch_test")

        try:
            proc_output.assertWaitFor(
                "MoveIt ready for planning group 'panda_arm'",
                timeout=120,
                process=plan_node,
            )

            # 2. Setup service client
            client = node.create_client(PlanPose, "/plan_and_execute")
            if not client.wait_for_service(timeout_sec=60.0):
                raise AssertionError("Timed out waiting for /plan_and_execute")

            # 3. Wait for joint states
            messages = []
            sub = node.create_subscription(
                JointState, "/joint_states", messages.append, 10
            )

            deadline = time.time() + 60.0
            while time.time() < deadline and not messages:
                rclpy.spin_once(node, timeout_sec=0.1)

            if not messages:
                raise AssertionError("Timed out waiting for /joint_states")
            node.destroy_subscription(sub)

            # 4. TF setup
            tf_buffer = Buffer()
            TransformListener(tf_buffer, node, spin_thread=False)

            # 5. Execute Plan
            last_message = "No response received"
            test_deadline = time.time() + 60.0

            while time.time() < test_deadline:
                # Wait for transform
                transform = None
                tf_deadline = time.time() + 10.0
                while time.time() < tf_deadline:
                    try:
                        transform = tf_buffer.lookup_transform(
                            "world", "panda_link8", rclpy.time.Time()
                        )
                        break
                    except TransformException:
                        rclpy.spin_once(node, timeout_sec=0.1)

                if not transform:
                    raise AssertionError(
                        "Timed out waiting for transform world -> panda_link8"
                    )

                for z_offset in z_offset_list:
                    request = PlanPose.Request()
                    request.target.header.frame_id = "world"
                    request.target.header.stamp = node.get_clock().now().to_msg()
                    request.target.pose.position.x = transform.transform.translation.x
                    request.target.pose.position.y = transform.transform.translation.y
                    request.target.pose.position.z = (
                        transform.transform.translation.z + z_offset
                    )
                    request.target.pose.orientation.x = transform.transform.rotation.x
                    request.target.pose.orientation.y = transform.transform.rotation.y
                    request.target.pose.orientation.z = transform.transform.rotation.z
                    request.target.pose.orientation.w = transform.transform.rotation.w

                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

                    if not future.done():
                        continue

                    response = future.result()
                    if response and response.success:
                        proc_output.assertWaitFor(
                            "Plan executed successfully",
                            timeout=30,
                            process=plan_node,
                        )
                        return

                    if response:
                        last_message = response.message

                time.sleep(1.0)

            raise AssertionError(f"/plan_and_execute never succeeded: {last_message}")

        finally:
            node.destroy_node()
            rclpy.shutdown()
