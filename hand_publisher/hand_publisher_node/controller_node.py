import numpy as np
import rclpy
from hand_publisher_interfaces.srv import SolveIK, PlanPose
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool

from tf2_ros import TransformException, TransformBroadcaster  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R, Slerp

SERVICE_TYPES = {
    "SolveIK": SolveIK,
    "PlanPose": PlanPose,
}


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.target_pub = self.create_publisher(PoseStamped, "/ik_target", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.pose_stamped = PoseStamped()
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("ik_service", "/solve_ik")
        self.declare_parameter("service_type", "SolveIK")
        self.base_link = (
            self.get_parameter("base_link").get_parameter_value().string_value
        )
        self.service_type = SERVICE_TYPES[
            self.get_parameter("service_type").get_parameter_value().string_value
        ]
        self.ik_service_name = (
            self.get_parameter("ik_service").get_parameter_value().string_value
        )
        self.ik_client = self.create_client(self.service_type, self.ik_service_name)
        self.pending_ik_future = None
        self.last_ik_service_warn_ns = 0
        self.timer = self.create_timer(0.1, self.lookup_transform)

        # Delta tracking state
        self.moving = False
        self.prev_t: np.ndarray | None = None
        # Accumulated IK target pose (start at a safe default in front of the robot)
        self.ik_t = np.array([0.4, 0.0, 0.4])
        # Low-pass filter: alpha=1.0 -> no filtering, 0.1 -> heavy smoothing
        self.declare_parameter("lp_alpha", 0.2)
        self.lp_alpha: float = float(self.get_parameter("lp_alpha").value)  # type: ignore
        self.declare_parameter("track_rotation", False)
        self.track_rotation: bool = bool(self.get_parameter("track_rotation").value)  # type: ignore
        self.smooth_t = self.ik_t.copy()
        point_down_R = R.from_euler("X", np.pi, degrees=False)
        local_z_twist_axis = point_down_R.apply([0.0, 0.0, 1.0])
        z_twist_R = R.from_rotvec((np.pi / 4) * local_z_twist_axis)
        self.smooth_R = z_twist_R * point_down_R

        self.create_subscription(Bool, "moving", self._moving_cb, 1)

    def _send_ik_request(self):
        if self.pending_ik_future is not None and not self.pending_ik_future.done():
            return

        if not self.ik_client.wait_for_service(timeout_sec=0.0):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_ik_service_warn_ns >= 5_000_000_000:
                self.get_logger().warning(
                    f"Waiting for IK service {self.ik_service_name}"
                )
                self.last_ik_service_warn_ns = now_ns
            return

        request = self.service_type.Request()
        request.target.header.frame_id = self.pose_stamped.header.frame_id
        request.target.header.stamp = self.pose_stamped.header.stamp
        request.target.pose.position.x = self.pose_stamped.pose.position.x
        request.target.pose.position.y = self.pose_stamped.pose.position.y
        request.target.pose.position.z = self.pose_stamped.pose.position.z
        request.target.pose.orientation.x = self.pose_stamped.pose.orientation.x
        request.target.pose.orientation.y = self.pose_stamped.pose.orientation.y
        request.target.pose.orientation.z = self.pose_stamped.pose.orientation.z
        request.target.pose.orientation.w = self.pose_stamped.pose.orientation.w

        self.pending_ik_future = self.ik_client.call_async(request)
        self.pending_ik_future.add_done_callback(self._handle_ik_response)

    def _handle_ik_response(self, future):
        self.pending_ik_future = None

        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning(f"IK service call failed: {exc}")

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
        if self.track_rotation:
            self.smooth_R = Slerp([0.0, 1.0], R.concatenate([self.smooth_R, cur_R]))(
                [a]
            )[0]
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
        self.target_pub.publish(self.pose_stamped)
        self._send_ik_request()
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
