import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import pyaudio
except ImportError:
    pyaudio = None  # type: ignore


class MicTriggerNode(Node):
    """Publishes Bool on 'moving' and 'recording' based on microphone audio.

    - Any loud sound with dominant frequency < 1 kHz → moving = True
    - Any loud sound with dominant frequency >= 1 kHz → toggles recording
    - Quiet → moving = False (recording keeps its toggled state)
    """

    RATE = 16000
    CHUNK = 1600  # 100 ms of audio at 16 kHz

    def __init__(self):
        super().__init__("mic_trigger_node")

        self.declare_parameter("threshold", 0.02)
        self.declare_parameter("freq_boundary", 1000.0)
        self.declare_parameter("recording_debounce_secs", 1.0)
        self.threshold: float = self.get_parameter("threshold").value  # type: ignore
        self.freq_boundary: float = self.get_parameter("freq_boundary").value  # type: ignore
        self.recording_debounce: float = self.get_parameter("recording_debounce_secs").value  # type: ignore

        self.moving_pub = self.create_publisher(Bool, "moving", 1)
        self.recording_pub = self.create_publisher(Bool, "recording", 1)

        self.recording = False
        self.last_recording_toggle = 0.0
        self.was_high = False  # edge detection for high-freq sound

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

        # Poll mic at ~10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.moving_msg = Bool()
        self.recording_msg = Bool()

    def timer_callback(self):
        import time

        data = self.stream.read(self.CHUNK, exception_on_overflow=False)
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(samples**2)))

        loud = rms > self.threshold

        # Dominant frequency via FFT
        if loud:
            fft_mag = np.abs(np.fft.rfft(samples))
            freqs = np.fft.rfftfreq(len(samples), d=1.0 / self.RATE)
            dominant_freq = float(freqs[np.argmax(fft_mag[1:]) + 1])  # skip DC
        else:
            dominant_freq = 0.0

        is_high = loud and dominant_freq >= self.freq_boundary
        is_low = loud and dominant_freq < self.freq_boundary

        # Moving: true while low-frequency sound is loud
        self.moving_msg.data = is_low
        self.moving_pub.publish(self.moving_msg)

        # Recording: toggle on rising edge of high-frequency sound with debounce
        now = time.monotonic()
        if is_high and not self.was_high:
            if (now - self.last_recording_toggle) > self.recording_debounce:
                self.recording = not self.recording
                self.last_recording_toggle = now
                self.get_logger().info(
                    f"Recording {'STARTED' if self.recording else 'STOPPED'}"
                )
        self.was_high = is_high

        self.recording_msg.data = self.recording
        self.recording_pub.publish(self.recording_msg)

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
