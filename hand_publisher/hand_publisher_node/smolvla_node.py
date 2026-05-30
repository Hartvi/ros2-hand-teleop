from __future__ import annotations

import sys

print(sys.executable)
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import torch

EE_CAMERA_IMAGE_TOPIC = "/panda/ee_camera/image_raw"
EE_CAMERA_INFO_TOPIC = "/panda/ee_camera/camera_info"
BASE_CAMERA_IMAGE_TOPIC = "/panda/base_camera/image_raw"
BASE_CAMERA_INFO_TOPIC = "/panda/base_camera/camera_info"


def default_policy_loader(policy_path: str) -> Any:
    # Match the smoke script's import order because SmolVLA is not
    # re-exported consistently across LeRobot versions.
    try:
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        return SmolVLAPolicy.from_pretrained(policy_path)
    except ModuleNotFoundError as exc:
        if "transformers" in str(exc):
            raise RuntimeError(
                "SmolVLA requires the 'transformers' dependency, but it is missing."
            ) from exc

    try:
        from lerobot.policies.smolvla import SmolVLAPolicy  # type: ignore[attr-defined]

        return SmolVLAPolicy.from_pretrained(policy_path)
    except Exception:
        pass

    try:
        from lerobot.policies.pretrained import PreTrainedPolicy

        return PreTrainedPolicy.from_pretrained(policy_path)
    except Exception as exc:
        raise RuntimeError(
            "Failed to load SmolVLA policy from repo_id. "
            "Ensure LeRobot and SmolVLA dependencies are installed."
        ) from exc


def choose_device(requested: str) -> torch.device:
    if requested == "cpu":
        return torch.device("cpu")
    if requested == "cuda":
        return torch.device("cuda")
    if requested == "mps":
        return torch.device("mps")

    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def collect_model_stats(model: Any) -> dict[str, Any]:
    total_params = 0
    total_param_bytes = 0

    if hasattr(model, "parameters"):
        for parameter in model.parameters():
            param_count = int(parameter.numel())
            total_params += param_count
            total_param_bytes += int(param_count * parameter.element_size())

    return {
        "model_class": model.__class__.__name__,
        "config_class": getattr(
            getattr(model, "config", None), "__class__", type(None)
        ).__name__,
        "total_parameters": total_params,
        "parameter_memory_mb": round(total_param_bytes / (1024.0 * 1024.0), 2),
    }


class SmolVlaNode(Node):
    def __init__(self) -> None:
        super().__init__("smolvla_node")

        self.declare_parameter("repo_id", "Hartvi/smolvla")
        self.declare_parameter("device", "auto")
        self.declare_parameter(
            "image_topics",
            """{
                "observation.images.base": "/panda/base_camera/image_raw",
                "observation.images.ee": "/panda/ee_camera/image_raw"
            }""",
        )

        self.repo_id = str(self.get_parameter("repo_id").value)
        self.device = choose_device(str(self.get_parameter("device").value))
        self.image_topics = str(self.get_parameter("image_topics").value)

        self.subscription = self.create_subscription(
            msg_type=Image,
            topic=self.image_topics,
            callback=self.listener_callback,
            qos_profile=1,
        )
        self.subscription  # prevent unused variable warning

        if not self.repo_id:
            raise ValueError(
                "The 'repo_id' parameter must be set to a SmolVLA model id."
            )

        self.get_logger().info(
            f"Loading SmolVLA policy from '{self.repo_id}' on device '{self.device}'"
        )
        self.policy = default_policy_loader(self.repo_id)
        if hasattr(self.policy, "to"):
            self.policy.to(self.device)

        stats = collect_model_stats(self.policy)
        self.get_logger().info(
            "Loaded %s (%s), parameters=%s, memory=%s MB"
            % (
                stats["model_class"],
                stats["config_class"],
                f"{stats['total_parameters']:,}",
                stats["parameter_memory_mb"],
            )
        )

    def listener_callback(self, msg: Image) -> None:
        self.get_logger().info(
            f"Received image message on topic '{msg.header.frame_id}' with size {msg.width}x{msg.height}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmolVlaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
