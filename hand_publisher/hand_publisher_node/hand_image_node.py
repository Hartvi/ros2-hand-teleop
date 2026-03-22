from . import config
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

Seconds = float


class ImagePublisherNode(Node):

    def __init__(
        self,
        node_name: str = "image_publisher",
        topic: str = "webcam",
        src: str | int = 0,
    ):
        super().__init__(node_name=node_name)
        self.publisher_ = self.create_publisher(Image, topic=topic, qos_profile=1)
        timer_period: Seconds = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_SIZE[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_SIZE[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # might be ignored by some backends

    def timer_callback(self):
        ok, img = self.cap.read()
        if not ok:
            return
        img = cv2.flip(img, 1)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    config.init_caps()
    image_publisher_node = ImagePublisherNode(node_name="hand_publisher_node")
    rclpy.spin(image_publisher_node)
    # ensure explicitly the calling of cap release
    image_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# TODO: another node
# win_name = f"Debug source {src}"
# frame_id = 0
# while not stop_event.is_set():
#     cv2.imshow(win_name, img)
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord('q'):
#         break
# cap.release()
# if debug:
#     cv2.destroyWindow(win_name)
