import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.pub = self.create_publisher(PoseStamped, "/ik_target", 10)
        self.t = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.lookup_transform)

        self.group_name = "arm"
        self.declare_parameter("base_link", "base_link")
        self.base_link = (
            self.get_parameter("base_link").get_parameter_value().string_value
        )

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_link,
                source_frame="hand_frame",
                time=rclpy.time.Time(),
            )

            target_pose = self.pose_from_transform(transform)
            self.pub.publish(target_pose)

        except TransformException as ex:
            # self.get_logger().warn(f"TF lookup failed: {ex}")
            ...

    def pose_from_transform(self, transform_stamped) -> PoseStamped:
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation
        pose_stamped = PoseStamped()
        pose = pose_stamped.pose
        pose.position.x = t.x
        pose.position.y = t.y
        pose.position.z = t.z
        pose.orientation = q
        pose_stamped.header.frame_id = self.base_link
        return pose_stamped


def main(args=None):
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
