import numpy as np

from . import config
import rclpy
from rclpy.node import Node
from hand_publisher_interfaces.msg import HandPoints


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


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
            qos_profile=1,
        )
        self.subscription  # prevent unused variable warning

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, "hand_points_marker", 1)
        self.corrected_point_pub = self.create_publisher(
            HandPoints, "hand_points_corrected", 1
        )
        self.base_frame = base_frame

        self.old_points = np.zeros((21, 3))
        self.setup_markers()
        self.lerp_alpha = 0.5
        self.one_minus_lerp_alpha = 1.0 - self.lerp_alpha

        self.declare_parameter("max_hand_size", 18.5)
        self.max_hand_size: float = self.get_parameter("max_hand_size").value  # type: ignore

        self.declare_parameter("dist_exponent", 1.145)
        self.dist_exponent: float = self.get_parameter("dist_exponent").value  # type: ignore

        self.declare_parameter("total_scale", 0.0135)
        self.total_scale: float = self.get_parameter("total_scale").value  # type: ignore

        self.declare_parameter("scale", 10.0)
        self.scale: float = self.get_parameter("scale").value  # type: ignore

    def setup_markers(self):
        # Reuse messages and point objects to avoid per-frame allocation
        self.hand_point_msg = HandPoints()
        self.marker = Marker()
        self.marker.header.frame_id = "camera_frame"
        self.marker.ns = "hand_points"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE_LIST  # or Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker_points: list[Point] = [Point() for _ in range(21)]
        self.marker.points = self.marker_points

    def listener_callback(self, msg: HandPoints):
        hand_points = np.asarray(msg.points, dtype=float).reshape(21, 3)
        dist = self.mix_in_distance(hand_points)
        self.normalize_hand(hand_points, dist)
        # self.get_logger().info('Hand points: "%s"' % str(dist))

        # In-place low-pass filter to avoid extra temporary arrays.
        self.old_points *= self.one_minus_lerp_alpha
        self.old_points += self.lerp_alpha * hand_points

        self.hand_point_msg.points = self.old_points.reshape(-1).tolist()
        self.corrected_point_pub.publish(self.hand_point_msg)
        self.publish_marker(self.old_points)

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

        # thumb_dists = segment_dist(xyz[0:5, 0:2])
        # index_dists = segment_dist(xyz[[0] + list(range(5, 9)), 0:2])
        # middle_dists = segment_dist(xyz[[0] + list(range(9, 13)), 0:2])
        # ring_dists = segment_dist(xyz[[0] + list(range(13, 17)), 0:2])
        # pinky_dists = segment_dist(xyz[[0] + list(range(17, 20)), 0:2])
        # mean_dist: float = float(
        #     np.mean([thumb_dists, index_dists, middle_dists, ring_dists, pinky_dists])
        # )
        d1 = segment_dist(xyz[[0, 5], 0:2])
        d2 = segment_dist(xyz[[0, 9], 0:2])
        d3 = segment_dist(xyz[[0, 13], 0:2])
        d4 = segment_dist(xyz[[0, 17], 0:2])
        mean_dist = float(np.mean([d1, d2, d3, d4])) / 3.0
        return (
            self.total_scale
            * (self.max_hand_size / (self.scale * mean_dist)) ** self.dist_exponent
        )

    def publish_marker(self, hand_points: np.ndarray):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        for idx, p in enumerate(hand_points):
            pt = self.marker_points[idx]
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
        self.marker_pub.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    hand_publisher_node = HandPublisherNode(node_name="hand_publisher_node")
    rclpy.spin(hand_publisher_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
