#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointCommandMux(Node):
    def __init__(self):
        super().__init__("joint_command_mux")

        # Match your controller YAML joint order exactly:
        self.arm_joint_order = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        self.gripper_joint_order = [
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        self.arm = None
        self.gripper = None

        self.sub_arm = self.create_subscription(
            JointState, "/main_joint_states", self.cb_arm, 10
        )
        self.sub_grip = self.create_subscription(
            JointState, "/gripper_joint_states", self.cb_grip, 10
        )

        self.pub_arm = self.create_publisher(
            Float64MultiArray, "/arm_controller/commands", 10
        )
        self.pub_grip = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )

        self.timer = self.create_timer(1.0 / 60.0, self.tick)  # 60 Hz is fine

        # Optional: also publish desired joint_states for debugging (NOT /joint_states!)
        self.pub_desired = self.create_publisher(
            JointState, "/joint_states_desired", 10
        )

    def cb_arm(self, msg: JointState):
        self.arm = msg

    def cb_grip(self, msg: JointState):
        self.gripper = msg

    @staticmethod
    def to_map(msg: JointState | None) -> dict[str, float]:
        if msg is None:
            return {}
        m: dict[str, float] = {}
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                m[n] = msg.position[i]
        return m

    def tick(self):
        if self.arm is None:
            return

        m = self.to_map(self.arm)
        m.update(self.to_map(self.gripper))

        # Publish arm commands (only if all required joints are present)
        try:
            arm_cmd = [m[j] for j in self.arm_joint_order]
        except KeyError:
            # You can log once per few seconds if you want
            return

        msg_arm = Float64MultiArray()
        msg_arm.data = arm_cmd
        self.pub_arm.publish(msg_arm)

        # Publish gripper commands if present
        gripper_cmds = [m[j] for j in self.gripper_joint_order if j in m]
        if len(gripper_cmds) == len(self.gripper_joint_order):
            msg_grip = Float64MultiArray()
            msg_grip.data = gripper_cmds
            self.pub_grip.publish(msg_grip)

        # Optional debug desired joint states (separate topic)
        desired = JointState()
        desired.header.stamp = self.get_clock().now().to_msg()
        desired.name = list(m.keys())
        desired.position = [m[n] for n in desired.name]
        self.pub_desired.publish(desired)


def main():
    rclpy.init()
    node = JointCommandMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
