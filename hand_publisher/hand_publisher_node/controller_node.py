import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.pub = self.create_publisher(PoseStamped, "/ik_target", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_stamped = PoseStamped()
        self.declare_parameter("base_link", "base_link")
        self.base_link = (
            self.get_parameter("base_link").get_parameter_value().string_value
        )
        self.timer = self.create_timer(0.1, self.lookup_transform)

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_link,
                source_frame="hand_frame",
                time=rclpy.time.Time(),  # type: ignore
            )
            self.pose_stamped.header.frame_id = self.base_link
            self.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.pose_stamped.pose.position.x = transform.transform.translation.x
            self.pose_stamped.pose.position.y = transform.transform.translation.y
            self.pose_stamped.pose.position.z = transform.transform.translation.z
            self.pose_stamped.pose.orientation = transform.transform.rotation
            self.pub.publish(self.pose_stamped)
        except TransformException:
            pass


def main(args=None):
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
