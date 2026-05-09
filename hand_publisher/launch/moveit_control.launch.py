import os
import time
import unittest

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
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def _build_launch_entities():
    moveit_config = _build_moveit_config()
    ros2_controllers_path = os.path.join(
        str(moveit_config.package_path), "config", "ros2_controllers.yaml"
    )
    rviz_config_path = os.path.join(
        str(moveit_config.package_path), "launch", "moveit.rviz"
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="mock_components",
            description="ros2_control hardware interface used for Panda MoveIt bring-up",
        ),
        DeclareLaunchArgument(
            "move_group_name",
            default_value="panda_arm",
            description="MoveIt planning group used by the planning service",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz alongside the MoveIt test stack",
        ),
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

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

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

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

    class TestMoveItControlLaunch(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            rclpy.init()

        @classmethod
        def tearDownClass(cls):
            rclpy.shutdown()

        def setUp(self):
            self.node = rclpy.create_node("moveit_control_launch_test")

        def tearDown(self):
            self.node.destroy_node()

        def _wait_for_joint_states(self, timeout_sec):
            messages = []
            subscription = self.node.create_subscription(
                JointState, "/joint_states", messages.append, 10
            )

            try:
                deadline = time.time() + timeout_sec
                while time.time() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    if messages:
                        return messages[-1]
            finally:
                self.node.destroy_subscription(subscription)

            self.fail("Timed out waiting for /joint_states from the Panda controllers")

        def _wait_for_transform(
            self, tf_buffer, target_frame, source_frame, timeout_sec
        ):
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                try:
                    return tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                    )
                except TransformException:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

            self.fail(
                f"Timed out waiting for transform from {target_frame} to {source_frame}"
            )

        def _call_plan_service(self, client, request, timeout_sec):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            self.assertTrue(future.done(), "Timed out waiting for /plan_and_execute")
            response = future.result()
            self.assertIsNotNone(response, "PlanPose returned no response")
            return response

        def _make_request(self, transform, z_offset):
            request = PlanPose.Request()
            request.target.header.frame_id = "world"
            request.target.header.stamp = self.node.get_clock().now().to_msg()
            request.target.pose.position.x = transform.transform.translation.x
            request.target.pose.position.y = transform.transform.translation.y
            request.target.pose.position.z = (
                transform.transform.translation.z + z_offset
            )
            request.target.pose.orientation.x = transform.transform.rotation.x
            request.target.pose.orientation.y = transform.transform.rotation.y
            request.target.pose.orientation.z = transform.transform.rotation.z
            request.target.pose.orientation.w = transform.transform.rotation.w
            return request

        def test_plan_and_execute_service(self, proc_output, plan_node):
            proc_output.assertWaitFor(
                "MoveIt ready for planning group 'panda_arm'",
                timeout=120,
                process=plan_node,
            )

            client = self.node.create_client(PlanPose, "/plan_and_execute")
            self.assertTrue(
                client.wait_for_service(timeout_sec=60.0),
                "Timed out waiting for /plan_and_execute",
            )

            self._wait_for_joint_states(timeout_sec=60.0)
            tf_buffer = Buffer()
            TransformListener(tf_buffer, self.node, spin_thread=False)

            last_message = "No response received"
            deadline = time.time() + 60.0

            while time.time() < deadline:
                transform = self._wait_for_transform(
                    tf_buffer,
                    "world",
                    "panda_link8",
                    timeout_sec=10.0,
                )

                for z_offset in (0.02, 0.0):
                    response = self._call_plan_service(
                        client,
                        self._make_request(transform, z_offset),
                        timeout_sec=30.0,
                    )
                    last_message = response.message
                    if response.success:
                        proc_output.assertWaitFor(
                            "Plan executed successfully",
                            timeout=30,
                            process=plan_node,
                        )
                        return

                time.sleep(1.0)

            self.fail(f"/plan_and_execute never succeeded: {last_message}")
