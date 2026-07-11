import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from hand_publisher_interfaces.msg import Obstacle
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


IGNORED_MODEL_NAMES = {"panda", "sun", "ground_plane", "ground plane"}


def _parse_floats(text: Optional[str], count: int, default: float = 0.0) -> list[float]:
    values = [float(item) for item in (text or "").split()]
    if len(values) < count:
        values.extend([default] * (count - len(values)))
    return values[:count]


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_multiply(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quat: tuple[float, float, float, float], vec: tuple[float, float, float]
) -> tuple[float, float, float]:
    q_vec = (vec[0], vec[1], vec[2], 0.0)
    q_inv = (-quat[0], -quat[1], -quat[2], quat[3])
    rotated = _quat_multiply(_quat_multiply(quat, q_vec), q_inv)
    return rotated[:3]


def _pose_from_sdf_text(text: Optional[str]) -> Pose:
    x, y, z, roll, pitch, yaw = _parse_floats(text, 6)
    qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _compose_pose(parent: Pose, child: Pose) -> Pose:
    parent_q = (
        parent.orientation.x,
        parent.orientation.y,
        parent.orientation.z,
        parent.orientation.w,
    )
    child_q = (
        child.orientation.x,
        child.orientation.y,
        child.orientation.z,
        child.orientation.w,
    )
    rotated_child = _rotate_vector(
        parent_q, (child.position.x, child.position.y, child.position.z)
    )
    out_q = _quat_multiply(parent_q, child_q)
    out = Pose()
    out.position.x = parent.position.x + rotated_child[0]
    out.position.y = parent.position.y + rotated_child[1]
    out.position.z = parent.position.z + rotated_child[2]
    out.orientation.x = out_q[0]
    out.orientation.y = out_q[1]
    out.orientation.z = out_q[2]
    out.orientation.w = out_q[3]
    return out


class SdfObstaclePublisher(Node):
    def __init__(self):
        super().__init__("sdf_obstacle_publisher")

        self.declare_parameter("world_path", "")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("obstacle_topic", "/planning_obstacles")
        self.declare_parameter("gazebo_tf_topic", "")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("ignored_models", list(IGNORED_MODEL_NAMES))

        self.frame_id = self.get_parameter("frame_id").value
        self.ignored_models = {
            str(name).lower() for name in self.get_parameter("ignored_models").value
        }
        obstacle_topic = self.get_parameter("obstacle_topic").value
        self.publisher = self.create_publisher(Obstacle, obstacle_topic, 10)

        self.model_poses: dict[str, Pose] = {}
        self.collision_offsets: dict[str, Pose] = {}
        self.obstacles: dict[str, Obstacle] = {}

        world_path = self._resolve_world_path(self.get_parameter("world_path").value)
        self._load_world(world_path)

        gazebo_tf_topic = self.get_parameter("gazebo_tf_topic").value
        if gazebo_tf_topic:
            self.create_subscription(TFMessage, gazebo_tf_topic, self._tf_cb, 10)
            self.get_logger().info(f"Listening for Gazebo pose updates on {gazebo_tf_topic}")

        period = float(self.get_parameter("publish_period").value)
        self.timer = self.create_timer(period, self._publish_all)
        self._publish_all()

    def _resolve_world_path(self, value: str) -> Path:
        pkg_share = Path(get_package_share_directory("hand_publisher"))
        if value.startswith("package://"):
            package_path = value.removeprefix("package://")
            package_name, relative_path = package_path.split("/", 1)
            return Path(get_package_share_directory(package_name)) / relative_path
        if value:
            candidate = Path(value).expanduser()
            if candidate.exists() or candidate.is_absolute():
                return candidate
            package_world = pkg_share / "worlds" / value
            if package_world.exists():
                return package_world
            return candidate
        return pkg_share / "worlds" / "my_world.sdf"

    def _load_world(self, world_path: Path):
        if not world_path.exists():
            raise RuntimeError(f"SDF world does not exist: {world_path}")

        root = ET.parse(world_path).getroot()
        for model in root.findall(".//world/model"):
            model_name = model.attrib.get("name", "")
            if not model_name or model_name.lower() in self.ignored_models:
                continue

            model_pose = _pose_from_sdf_text(model.findtext("pose"))
            self.model_poses[model_name] = model_pose
            collisions = model.findall("./link/collision")
            for collision in collisions:
                self._add_collision(model_name, model_pose, collision)

        self.get_logger().info(
            f"Loaded {len(self.obstacles)} obstacle primitives from {world_path}"
        )

    def _add_collision(self, model_name: str, model_pose: Pose, collision: ET.Element):
        collision_name = collision.attrib.get("name", "collision")
        obstacle_id = model_name if collision_name in {"collision", "col"} else f"{model_name}::{collision_name}"
        collision_pose = _pose_from_sdf_text(collision.findtext("pose"))
        geometry = collision.find("geometry")
        if geometry is None:
            return

        obstacle_type = ""
        dimensions: list[float] = []
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        sphere = geometry.find("sphere")

        if box is not None:
            obstacle_type = "box"
            dimensions = _parse_floats(box.findtext("size"), 3)
        elif cylinder is not None:
            obstacle_type = "cylinder"
            radius = _parse_floats(cylinder.findtext("radius"), 1)[0]
            length = _parse_floats(cylinder.findtext("length"), 1)[0]
            dimensions = [radius, length]
        elif sphere is not None:
            obstacle_type = "sphere"
            dimensions = [_parse_floats(sphere.findtext("radius"), 1)[0]]
        else:
            self.get_logger().warn(f"Skipping unsupported geometry for {obstacle_id}")
            return

        obstacle = Obstacle()
        obstacle.header.frame_id = self.frame_id
        obstacle.id = obstacle_id
        obstacle.type = obstacle_type
        obstacle.dimensions = dimensions
        obstacle.pose = _compose_pose(model_pose, collision_pose)
        obstacle.remove = False

        self.collision_offsets[obstacle_id] = collision_pose
        self.obstacles[obstacle_id] = obstacle

    def _tf_cb(self, msg: TFMessage):
        changed = False
        for transform in msg.transforms:
            model_name = transform.child_frame_id.split("/")[0]
            if model_name not in self.model_poses:
                continue

            model_pose = Pose()
            model_pose.position.x = transform.transform.translation.x
            model_pose.position.y = transform.transform.translation.y
            model_pose.position.z = transform.transform.translation.z
            model_pose.orientation = transform.transform.rotation
            self.model_poses[model_name] = model_pose

            for obstacle_id, obstacle in self.obstacles.items():
                if obstacle_id == model_name or obstacle_id.startswith(f"{model_name}::"):
                    obstacle.pose = _compose_pose(model_pose, self.collision_offsets[obstacle_id])
                    changed = True

        if changed:
            self._publish_all()

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()
        for obstacle in self.obstacles.values():
            obstacle.header.stamp = stamp
            obstacle.header.frame_id = self.frame_id
            self.publisher.publish(obstacle)


def main(args=None):
    rclpy.init(args=args)
    node = SdfObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
