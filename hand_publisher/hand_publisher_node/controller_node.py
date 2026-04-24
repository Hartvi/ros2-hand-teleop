import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool

from tf2_ros import TransformException, TransformBroadcaster  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R, Slerp


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.pub = self.create_publisher(PoseStamped, "/ik_target", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.pose_stamped = PoseStamped()
        self.declare_parameter("base_link", "base_link")
        self.base_link = (
            self.get_parameter("base_link").get_parameter_value().string_value
        )
        self.timer = self.create_timer(0.1, self.lookup_transform)

        # Delta tracking state
        self.moving = False
        self.prev_t: np.ndarray | None = None
        # Accumulated IK target pose (start at a safe default in front of the robot)
        self.ik_t = np.array([0.4, 0.0, 0.4])
        # Low-pass filter: alpha=1.0 -> no filtering, 0.1 -> heavy smoothing
        self.declare_parameter("lp_alpha", 0.2)
        self.lp_alpha: float = float(self.get_parameter("lp_alpha").value)  # type: ignore
        self.smooth_t = self.ik_t.copy()
        self.smooth_R = R.identity()

        self.create_subscription(Bool, "moving", self._moving_cb, 1)

    def _moving_cb(self, msg: Bool):
        if msg.data and not self.moving:
            # Just started moving - reset previous pose so first delta is zero
            self.prev_t = None
        self.moving = msg.data

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_link,
                source_frame="hand_frame",
                time=rclpy.time.Time(),  # type: ignore
            )
        except TransformException:
            return

        tf = transform.transform
        cur_t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        cur_R = R.from_quat(
            [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        )

        if self.moving and self.prev_t is not None:
            delta_t = cur_t - self.prev_t
            self.ik_t += delta_t

        self.prev_t = cur_t

        # Low-pass filter: lerp translation, slerp rotation
        a = self.lp_alpha
        self.smooth_t = a * self.ik_t + (1.0 - a) * self.smooth_t
        self.smooth_R = Slerp([0.0, 1.0], R.concatenate([self.smooth_R, cur_R]))([a])[0]
        quat = self.smooth_R.as_quat()  # x, y, z, w

        self.pose_stamped.header.frame_id = self.base_link
        self.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.pose_stamped.pose.position.x = float(self.smooth_t[0])
        self.pose_stamped.pose.position.y = float(self.smooth_t[1])
        self.pose_stamped.pose.position.z = float(self.smooth_t[2])
        self.pose_stamped.pose.orientation.x = float(quat[0])
        self.pose_stamped.pose.orientation.y = float(quat[1])
        self.pose_stamped.pose.orientation.z = float(quat[2])
        self.pose_stamped.pose.orientation.w = float(quat[3])
        self.pub.publish(self.pose_stamped)
        # Also publish as a TF frame
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = self.base_link
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.child_frame_id = "ik_target_frame"
        tf_msg.transform.translation.x = float(self.smooth_t[0])
        tf_msg.transform.translation.y = float(self.smooth_t[1])
        tf_msg.transform.translation.z = float(self.smooth_t[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.br.sendTransform(tf_msg)


def main(args=None):
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
