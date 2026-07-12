from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from hand_publisher_interfaces.srv import SmolVLAInference


class SmolVlaControlNode(Node):
    def __init__(self) -> None:
        super().__init__("smolvla_control_node")
        self.declare_parameter("task_command", "Teleop task")
        self.declare_parameter("period", 0.5)
        self.declare_parameter("joint_names", [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6",
        ])
        self.declare_parameter("controller_joint_names", [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
        ])
        self.declare_parameter("inference_service", "/smolvla_inference")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_topic", "/smolvla_joint_commands")
        self.declare_parameter("controller_topic", "/arm_controller/commands")

        self.task_command = str(self.get_parameter("task_command").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.controller_joint_names = list(
            self.get_parameter("controller_joint_names").value
        )
        self.client = self.create_client(
            SmolVLAInference, str(self.get_parameter("inference_service").value)
        )
        self.publisher = self.create_publisher(
            JointState, str(self.get_parameter("output_topic").value), 10
        )
        self.controller_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("controller_topic").value),
            10,
        )
        self.current_positions: dict[str, float] = {}
        self.state_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.state_callback,
            10,
        )
        self.request_in_flight = False
        self.timer = self.create_timer(
            float(self.get_parameter("period").value), self.request_inference
        )

    def state_callback(self, message: JointState) -> None:
        self.current_positions.update(
            {
                name: float(position)
                for name, position in zip(message.name, message.position)
            }
        )

    def request_inference(self) -> None:
        if self.request_in_flight or not self.client.service_is_ready():
            return
        request = SmolVLAInference.Request()
        request.task_command = self.task_command
        self.request_in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future) -> None:
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"SmolVLA service call failed: {exc}")
            return
        if not response.success:
            self.get_logger().warning(response.error_message)
            return
        if len(response.action) != len(self.joint_names):
            self.get_logger().error(
                f"Action length {len(response.action)} does not match "
                f"joint_names length {len(self.joint_names)}"
            )
            return

        commanded = dict(self.current_positions)
        for name, value in zip(self.joint_names, response.action):
            commanded[name] = float(value)
        if any(name not in commanded for name in self.controller_joint_names):
            self.get_logger().warning(
                "Waiting for a complete /joint_states message before publishing "
                "the 7-joint Panda command"
            )
            return

        command = Float64MultiArray()
        command.data = [commanded[name] for name in self.controller_joint_names]
        self.controller_publisher.publish(command)

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.controller_joint_names
        message.position = command.data
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmolVlaControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
