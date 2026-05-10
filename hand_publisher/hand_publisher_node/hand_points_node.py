import cv2
import mediapipe
import rclpy

from . import config
from hand_publisher_interfaces.msg import HandPoints
from rclpy.node import Node

Seconds = float


class Landmark:
    x: float
    y: float
    z: float


class HandPointsNode(Node):
    """Captures webcam frames, runs MediaPipe hand detection, and publishes hand points."""

    def __init__(
        self,
        node_name: str = "hand_points_node",
        topic_hand_points: str = "hand_points",
        src: str | int = 0,
    ):
        super().__init__(node_name)

        # Webcam capture
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_SIZE[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_SIZE[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # MediaPipe hand detection
        self.declare_parameter("num_hands_per_src", 1)
        self.num_hands_per_src: int = self.get_parameter("num_hands_per_src").value  # type: ignore
        self.mp_hands = mediapipe.solutions.hands  # type: ignore
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.num_hands_per_src,
            model_complexity=0,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )

        # Publisher
        self.publisher_ = self.create_publisher(
            HandPoints, topic=topic_hand_points, qos_profile=10
        )

        # Timer drives the capture + detection loop
        timer_period: Seconds = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ok, img = self.cap.read()
        if not ok:
            return
        img_rgb = cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        if results.multi_hand_landmarks:
            hand_marks = list(results.multi_hand_landmarks)
            hand_points = [list(handLms.landmark) for handLms in hand_marks]
            landmarks = self.landmarks_to_handpoints(hand_points)
            msg_hand = HandPoints()
            msg_hand.points = landmarks
            self.publisher_.publish(msg_hand)

    @staticmethod
    def landmarks_to_handpoints(hand_marks: list[list[Landmark]]) -> list[float]:
        assert len(hand_marks) == 1, f"{len(hand_marks)=}"
        hand = hand_marks[0]
        hand_points = sum(map(lambda x: [x.x, x.y, x.z], hand), start=[])
        return hand_points

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    config.init_caps()
    hand_points_node = HandPointsNode()
    rclpy.spin(hand_points_node)
    hand_points_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
