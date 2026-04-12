import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class PoseRecorderNode(Node):
    """Records IK target poses and gripper state to CSV while 'recording' is True."""

    def __init__(self):
        super().__init__("pose_recorder_node")

        self.declare_parameter("output_dir", "~/vla_recordings")
        output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        # Latest state
        self.latest_pose: PoseStamped | None = None
        self.latest_gripper: JointState | None = None

        self.create_subscription(Bool, "recording", self._recording_cb, 1)
        self.create_subscription(PoseStamped, "/ik_target", self._pose_cb, 1)
        self.create_subscription(
            JointState, "/gripper_joint_states", self._gripper_cb, 1
        )

        # Write at 10 Hz while recording
        self.timer = self.create_timer(0.1, self._write_tick)

    def _recording_cb(self, msg: Bool):
        if msg.data and not self.recording:
            self._start_recording()
        elif not msg.data and self.recording:
            self._stop_recording()
        self.recording = msg.data

    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def _gripper_cb(self, msg: JointState):
        self.latest_gripper = msg

    def _start_recording(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"episode_{stamp}.csv"
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "t",
                "px",
                "py",
                "pz",
                "qx",
                "qy",
                "qz",
                "qw",
                "gripper_pos",
            ]
        )
        self.get_logger().info(f"Recording started: {path}")

    def _stop_recording(self):
        if self.csv_file is not None:
            path = self.csv_file.name
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info(f"Recording stopped: {path}")

    def _write_tick(self):
        if not self.recording or self.csv_writer is None:
            return
        if self.latest_pose is None:
            return

        p = self.latest_pose.pose.position
        o = self.latest_pose.pose.orientation
        gripper_pos = 0.0
        if self.latest_gripper is not None and self.latest_gripper.position:
            gripper_pos = self.latest_gripper.position[0]

        self.csv_writer.writerow(
            [
                time.time(),
                p.x,
                p.y,
                p.z,
                o.x,
                o.y,
                o.z,
                o.w,
                gripper_pos,
            ]
        )

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
