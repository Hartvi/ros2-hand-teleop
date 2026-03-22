import numpy as np

from . import config
import rclpy
from rclpy.node import Node
from hand_publisher_interfaces.msg import HandPoints


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class HandPublisherNode(Node):

    def __init__(
        self,
        node_name: str = "hand_publisher_node",
        topic: str = "hand_points",
        base_frame: str = "world",
    ):
        super().__init__(node_name=node_name)
        self.subscription = self.create_subscription(
            msg_type=HandPoints,
            topic=topic,
            callback=self.listener_callback,
            qos_profile=10,
        )
        self.subscription  # prevent unused variable warning

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, "hand_points_marker", 10)
        self.finger_pos = self.create_publisher(Point, "finger_dist", 10)
        self.corrected_point_pub = self.create_publisher(
            HandPoints, "hand_points_corrected", 10
        )
        self.base_frame = base_frame

        self.old_points = np.zeros((21, 3))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("max_hand_size", 18.5)
        self.max_hand_size: float = self.get_parameter("max_hand_size").value  # type: ignore

        self.declare_parameter("dist_exponent", 1.145)
        self.dist_exponent: float = self.get_parameter("dist_exponent").value  # type: ignore

        self.declare_parameter("total_scale", 0.0135)
        self.total_scale: float = self.get_parameter("total_scale").value  # type: ignore

        self.declare_parameter("scale", 10.0)
        self.scale: float = self.get_parameter("scale").value  # type: ignore

    def listener_callback(self, msg: HandPoints):
        hand_points = np.array(msg.points).reshape(21, 3)
        dist = self.mix_in_distance(hand_points)
        self.normalize_hand(hand_points, dist)
        # self.get_logger().info('Hand points: "%s"' % str(dist))

        # Publish visualization marker in base_frame
        low_pass_points = self.lerp(0.5, hand_points, self.old_points)
        hand_point_msg = HandPoints()
        hand_point_msg.points = low_pass_points.reshape(-1).tolist()
        self.corrected_point_pub.publish(hand_point_msg)
        self.publish_marker(low_pass_points)
        self.old_points = low_pass_points

    @staticmethod
    def normalize_hand(
        points_21_3: np.ndarray,
        dist: float,
    ):
        points_21_3[:, :2] -= 0.5
        points_21_3[:, :2] *= dist
        points_21_3[:, 2] += dist

    def mix_in_distance(self, xyz: np.ndarray) -> float:
        assert xyz.shape[0] == 21, f"{xyz=}"

        def segment_dist(x: np.ndarray) -> np.floating:
            return np.mean(np.sqrt(np.sum(np.diff(x, axis=0) ** 2, axis=1)))

        thumb_dists = segment_dist(xyz[0:5, 0:2])
        index_dists = segment_dist(xyz[[0] + list(range(5, 9)), 0:2])
        middle_dists = segment_dist(xyz[[0] + list(range(9, 13)), 0:2])
        ring_dists = segment_dist(xyz[[0] + list(range(13, 17)), 0:2])
        pinky_dists = segment_dist(xyz[[0] + list(range(17, 20)), 0:2])
        mean_dist: float = float(
            np.mean([thumb_dists, index_dists, middle_dists, ring_dists, pinky_dists])
        )
        return (
            self.total_scale
            * (self.max_hand_size / (self.scale * mean_dist)) ** self.dist_exponent
        )

    @staticmethod
    def lerp(lerp: float, arr1: np.ndarray, arr2: np.ndarray) -> np.ndarray:
        if lerp > 0.0:
            return lerp * arr1 + (1.0 - lerp) * arr2
        return arr2

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="world",
                source_frame="camera_frame",
                time=rclpy.time.Time(),  # type: ignore
            )

            return transform

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")

    def publish_marker(self, hand_points: np.ndarray):
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "hand_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST  # or Marker.POINTS
        marker.action = Marker.ADD

        # Marker pose (identity – points are already in this frame)
        marker.pose.orientation.w = 1.0

        # Size of each point
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color (opaque green)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for p in hand_points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            marker.points.append(pt)  # type: ignore
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    config.init_caps()
    hand_publisher_node = HandPublisherNode(node_name="hand_publisher_node")
    rclpy.spin(hand_publisher_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
