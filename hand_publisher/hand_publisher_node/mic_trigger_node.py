import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import pyaudio
except ImportError:
    pyaudio = None  # type: ignore


class MicTriggerNode(Node):
    """Publishes Bool on 'moving' based on microphone volume level."""

    RATE = 16000
    CHUNK = 1600  # 100 ms of audio at 16 kHz

    def __init__(self):
        super().__init__("mic_trigger_node")

        self.declare_parameter("threshold", 0.02)
        self.threshold: float = self.get_parameter("threshold").value  # type: ignore

        self.pub = self.create_publisher(Bool, "moving", 1)

        if pyaudio is None:
            self.get_logger().error("pyaudio not installed - mic trigger disabled")
            return

        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
        )

        # Poll mic at ~10 Hz (matches controller_node timer rate)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.msg = Bool()

    def timer_callback(self):
        data = self.stream.read(self.CHUNK, exception_on_overflow=False)
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(samples**2)))

        self.get_logger().info(f"Mic RMS: {rms:.4f} (threshold: {self.threshold})")
        loud = rms > self.threshold
        self.msg.data = loud
        self.pub.publish(self.msg)

    def destroy_node(self):
        if hasattr(self, "stream"):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, "pa"):
            self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
