#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from hand_publisher_interfaces.msg import HandPoints
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperPublisher(Node):
    def __init__(self):
        super().__init__("gripper_publisher")

        self.pub = self.create_publisher(JointState, "/gripper_joint_states", 1)
        self.sub = self.create_subscription(
            HandPoints, "hand_points_corrected", self.grip, 1
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
        self.declare_parameter("closed_distance", 0.07)
        self.declare_parameter("debounce_secs", 2.0)

        self.joint_names: list[str] = self.get_parameter("joint_names").value  # type: ignore
        self.joint_multipliers: list[float] = self.get_parameter(  # type: ignore
            "joint_multipliers"
        ).value
        self.q_scale: float = float(self.get_parameter("q_scale").value)  # type: ignore
        self.q_max = float(self.get_parameter("q_max").value)  # type: ignore
        self.closed_distance = float(self.get_parameter("closed_distance").value)  # type: ignore
        self.debounce_secs = float(self.get_parameter("debounce_secs").value)  # type: ignore

        if len(self.joint_names) != len(self.joint_multipliers):
            raise ValueError("joint_names and joint_multipliers must have same length")

        # Toggle state
        self.gripper_closed = False
        self.fingers_were_close = False
        self.last_toggle_time = 0.0
        self.q = 0.0
        self.joint_state_msg = JointState()

    def grip(self, msg: HandPoints):
        hand_points = np.array(msg.points).reshape(21, 3)
        dist = np.linalg.norm(hand_points[4] - hand_points[8])

        fingers_close = float(self.q_scale * dist) < self.closed_distance
        now = time.monotonic()

        # Toggle on rising edge (fingers just came together) with debounce
        if fingers_close and not self.fingers_were_close:
            if (now - self.last_toggle_time) > self.debounce_secs:
                self.gripper_closed = not self.gripper_closed
                self.last_toggle_time = now

        self.fingers_were_close = fingers_close
        self.q = float(
            np.clip(0.0 if self.gripper_closed else self.q_max, 0.0, self.q_max)
        )
        self.publish()

    def publish(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [m * self.q for m in self.joint_multipliers]
        self.pub.publish(self.joint_state_msg)


def main():
    rclpy.init()
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
