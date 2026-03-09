#!/usr/bin/env python3
import numpy as np
import rclpy
from hand_publisher_interfaces.msg import HandPoints
from rclpy.node import Node
from sensor_msgs.msg import JointState


from rclpy.parameter import Parameter


class GripperPublisher(Node):
    def __init__(self):
        super().__init__("gripper_publisher")

        self.pub = self.create_publisher(JointState, "/gripper_joint_states", 10)
        self.sub = self.create_subscription(
            HandPoints, "hand_points_corrected", self.grip, 10
        )

        # Parameters
        # robotiq 2f 85 defaults
        self.declare_parameter(
            "joint_names",
            [
                "finger_joint",
                "left_inner_knuckle_joint",
                "right_outer_knuckle_joint",
                "right_inner_knuckle_joint",
                "left_inner_finger_joint",
                "right_inner_finger_joint",
            ],
        )
        self.declare_parameter("joint_multipliers", [1.0, 1.0, 1.0, 1.0, -1.0, -1.0])
        self.declare_parameter("q_scale", 1.0)
        self.declare_parameter("q_max", 1.0)

        self.joint_names: list[str] = self.get_parameter("joint_names").value
        self.joint_multipliers: list[float] = self.get_parameter(
            "joint_multipliers"
        ).value
        self.q_scale: float = float(self.get_parameter("q_scale").value)
        self.q_max = float(self.get_parameter("q_max").value)

        if len(self.joint_names) != len(self.joint_multipliers):
            raise ValueError("joint_names and joint_multipliers must have same length")

        self.q = 0.0

    def grip(self, msg: HandPoints):
        hand_points = np.array(msg.points).reshape(21, 3)
        dist = np.linalg.norm(hand_points[4] - hand_points[8])
        self.get_logger().info("Distance: %f" % dist)

        # to limit the noise => closed & open state are further apart with the second power
        q = float(self.q_scale * dist) > 0.1  # magic approximate hand-size constant
        self.q = float(np.clip(q, 0.0, self.q_max))
        self.publish()

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = list(self.joint_names)
        msg.position = [m * self.q for m in self.joint_multipliers]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
