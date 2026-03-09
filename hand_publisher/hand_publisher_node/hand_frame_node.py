import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from hand_publisher_interfaces.msg import HandPoints

from .hand_utils import hand_to_pose, hand_fingers_to_pose
from scipy.spatial.transform import Rotation as R


class HandFrameNode(Node):

    def __init__(
        self,
        node_name: str = "hand_frame_node",
        topic: str = "hand_points_corrected",
        base_frame: str = "world",
    ):
        super().__init__(node_name=node_name)
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            msg_type=HandPoints,
            topic=topic,
            callback=self.listener_callback,
            qos_profile=10,
        )
        self.static_br = StaticTransformBroadcaster(self)
        self.publish_hand_correction()
        self.publish_camera_pos()
        self.subscription  # prevent unused variable warning
        self.base_frame = base_frame

    def listener_callback(self, msg_in: HandPoints):
        stamp = self.get_clock().now().to_msg()
        self.raw_hand_frame(msg_in, stamp)

    def publish_camera_pos(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "camera_frame"

        # TODO: set smartly somehow
        x, y, z, w = R.from_euler(
            "XYZ", [0, np.pi / 2, -np.pi / 2], degrees=False
        ).as_quat()
        msg.transform.translation.x = -1.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.4
        msg.transform.rotation.x = float(x)
        msg.transform.rotation.y = float(y)
        msg.transform.rotation.z = float(z)
        msg.transform.rotation.w = float(w)
        self.static_br.sendTransform(msg)

    def publish_hand_correction(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "raw_hand_frame"
        msg.child_frame_id = "hand_frame"

        # TODO: set depending on the URDF that is present
        # x, y, z, w = R.from_euler("XYZ", [np.pi / 2, 0, np.pi], degrees=False).as_quat()
        x, y, z, w = R.from_euler(
            "XYZ", [0, np.pi / 2, np.pi / 2], degrees=False
        ).as_quat()
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = float(x)
        msg.transform.rotation.y = float(y)
        msg.transform.rotation.z = float(z)
        msg.transform.rotation.w = float(w)
        self.static_br.sendTransform(msg)

    def raw_hand_frame(self, msg_in: HandPoints, stamp):
        hand_points = np.array(msg_in.points, dtype=float).reshape(21, 3)

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        msg.child_frame_id = "raw_hand_frame"
        # TODO: publish corrected hand_points here

        try:
            R_rot, t = hand_fingers_to_pose(hand_points)
        except ValueError:
            R_rot, t = hand_to_pose(hand_points)
        x, y, z, w = R_rot.as_quat()

        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(x)
        msg.transform.rotation.y = float(y)
        msg.transform.rotation.z = float(z)
        msg.transform.rotation.w = float(w)

        self.br.sendTransform(msg)


def main():
    rclpy.init()
    node = HandFrameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
