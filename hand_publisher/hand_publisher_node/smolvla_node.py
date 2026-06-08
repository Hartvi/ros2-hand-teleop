from __future__ import annotations

import json
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import torch

from hand_publisher_interfaces.srv import SmolVLAInference

LAPTOP_CAMERA_IMAGE_TOPIC = "/panda/base_camera/image_raw"
LAPTOP_CAMERA_INFO_TOPIC = "/panda/base_camera/camera_info"
PHONE_CAMERA_IMAGE_TOPIC = "/panda/ee_camera/image_raw"
PHONE_CAMERA_INFO_TOPIC = "/panda/ee_camera/camera_info"
JOINT_STATES_TOPIC = "/joint_states"


def default_policy_loader(policy_path: str) -> Any:
    # Try PreTrainedPolicy first to avoid loading all policies and their dataclass issues
    try:
        from lerobot.policies.pretrained import PreTrainedPolicy

        return PreTrainedPolicy.from_pretrained(policy_path)
    except Exception:
        pass

    # Fallback: try direct SmolVLA import if PreTrainedPolicy fails
    try:
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        return SmolVLAPolicy.from_pretrained(policy_path)
    except ModuleNotFoundError as exc:
        if "transformers" in str(exc):
            raise RuntimeError(
                "SmolVLA requires the 'transformers' dependency, but it is missing."
            ) from exc
        raise RuntimeError(
            "Failed to load SmolVLA policy. Ensure LeRobot and SmolVLA dependencies are installed."
        ) from exc
    except Exception as exc:
        raise RuntimeError(
            "Failed to load SmolVLA policy from repo_id. "
            "Ensure LeRobot and SmolVLA dependencies are installed."
        ) from exc


def default_processor_factory(*args: Any, **kwargs: Any):
    try:
        from lerobot.policies import make_pre_post_processors  # type: ignore[attr-defined]
    except ImportError:
        from lerobot.policies.factory import make_pre_post_processors
    return make_pre_post_processors(*args, **kwargs)


def choose_device(requested: str) -> torch.device:
    if requested == "cpu":
        return torch.device("cpu")
    if requested == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError(
                "Device 'cuda' requested but CUDA is not available. "
                "Available devices: CPU"
                + (" + MPS" if torch.backends.mps.is_available() else "")
            )
        return torch.device("cuda")
    if requested == "mps":
        if not torch.backends.mps.is_available():
            raise RuntimeError(
                "Device 'mps' requested but MPS is not available. "
                "Available devices: CPU"
                + (" + CUDA" if torch.cuda.is_available() else "")
            )
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


def _feature_type_text(feature: Any) -> str:
    value = (
        feature.get("type", "")
        if isinstance(feature, dict)
        else getattr(feature, "type", "")
    )
    return str(value).lower()


def infer_feature_keys(config: Any) -> tuple[str, list[str], int]:
    input_features = getattr(config, "input_features", {}) or {}

    state_key = "observation.state"
    state_dim = 0
    image_keys: list[str] = []

    for key, feature in input_features.items():
        feature_type = _feature_type_text(feature)
        if isinstance(feature, dict):
            shape = tuple(feature.get("shape", ()) or ())
        else:
            shape = tuple(getattr(feature, "shape", ()) or ())
        if "state" in feature_type:
            state_key = key
            if shape:
                state_dim = int(shape[0])
        elif (
            "visual" in feature_type
            or "image" in feature_type
            or "video" in feature_type
        ):
            image_keys.append(key)

    if not image_keys:
        print(
            "WARNING: No visual/image features found in policy config; defaulting to '%s'."
            % image_keys
        )
        image_keys = ["observation.images.laptop"]

    if state_dim <= 0:
        state_dim = 9

    return state_key, image_keys, state_dim


def parse_state(raw: str | None, expected_dim: int) -> np.ndarray:
    if raw is None or raw.strip() == "":
        return np.zeros((expected_dim,), dtype=np.float32)

    tokens = [part.strip() for part in raw.split(",") if part.strip()]
    values = np.asarray([float(token) for token in tokens], dtype=np.float32)

    if values.shape[0] != expected_dim:
        raise ValueError(
            f"State length mismatch: got {values.shape[0]}, expected {expected_dim}."
        )

    return values


def tensor_to_list(x: Any) -> list[float]:
    if isinstance(x, torch.Tensor):
        return x.detach().cpu().reshape(-1).tolist()
    if isinstance(x, np.ndarray):
        return x.reshape(-1).tolist()
    return np.asarray(x).reshape(-1).tolist()


def _to_torch_float_tensor(value: Any, device: torch.device) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        return value.to(device=device)
    if isinstance(value, np.ndarray):
        return torch.from_numpy(value).to(device=device)
    return torch.as_tensor(value, device=device)


def ensure_policy_input_shapes(
    batch: dict[str, Any], state_key: str, image_keys: list[str], device: torch.device
) -> dict[str, Any]:
    out = dict(batch)

    if state_key in out:
        state = _to_torch_float_tensor(out[state_key], device=device).float()
        if state.ndim == 1:
            state = state.unsqueeze(0)
        out[state_key] = state

    for key in image_keys:
        if key not in out:
            continue
        image = _to_torch_float_tensor(out[key], device=device).float()
        if image.max().item() > 1.0:
            image = image / 255.0

        if image.ndim == 3 and image.shape[-1] in (1, 3):
            image = image.permute(2, 0, 1).unsqueeze(0)
        elif image.ndim == 3 and image.shape[0] in (1, 3):
            image = image.unsqueeze(0)
        elif image.ndim == 4 and image.shape[-1] in (1, 3):
            image = image.permute(0, 3, 1, 2)
        elif image.ndim == 4 and image.shape[1] in (1, 3):
            pass
        else:
            raise ValueError(
                f"Unsupported image shape for key '{key}': {tuple(image.shape)}"
            )

        out[key] = image

    return out


def ros_image_to_rgb(msg: Image) -> np.ndarray:
    if msg.height == 0 or msg.width == 0:
        raise ValueError("Received empty image message")

    channels = int(msg.step // msg.width) if msg.width > 0 else 0
    if channels <= 0:
        raise ValueError(
            f"Invalid image step/width combination: step={msg.step}, width={msg.width}"
        )

    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, channels
    )
    encoding = msg.encoding.lower()

    if encoding in ("rgb8", "rgba8"):
        return image[:, :, :3]
    if encoding in ("bgr8", "bgra8"):
        return image[:, :, :3][:, :, ::-1]

    if channels >= 3:
        return image[:, :, :3]

    return np.repeat(image[:, :, :1], 3, axis=2)


class SmolVlaNode(Node):
    def __init__(self) -> None:
        super().__init__("smolvla_node")

        self.declare_parameter("repo_id", "Hartvi/smolvla")
        self.declare_parameter("device", "auto")
        self.declare_parameter(
            "image_topics",
            """{
                "observation.images.laptop": "/panda/base_camera/image_raw",
                "observation.images.phone": "/panda/ee_camera/image_raw"
            }""",
        )
        self.declare_parameter("task", "Teleop task")
        self.declare_parameter("state", "")
        self.declare_parameter("joint_state_topic", JOINT_STATES_TOPIC)

        self.repo_id = str(self.get_parameter("repo_id").value)
        self.device = choose_device(str(self.get_parameter("device").value))
        self.image_topics_raw = str(self.get_parameter("image_topics").value)
        self.task = str(self.get_parameter("task").value)
        self.state_raw = str(self.get_parameter("state").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)

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

        self.state_key, self.expected_image_keys, self.state_dim = infer_feature_keys(
            self.policy.config
        )
        self.fixed_state = parse_state(self.state_raw, self.state_dim)
        self.latest_state = self.fixed_state.copy()

        try:
            self.preprocess, self.postprocess = default_processor_factory(
                self.policy.config,
                self.repo_id,
                preprocessor_overrides={
                    "device_processor": {"device": str(self.device)}
                },
            )
        except FileNotFoundError as exc:
            if "policy_preprocessor.json" not in str(exc):
                raise
            self.get_logger().warning(
                "policy_preprocessor.json not found in model repo; using config-derived processors."
            )
            self.preprocess, self.postprocess = default_processor_factory(
                self.policy.config,
                preprocessor_overrides={
                    "device_processor": {"device": str(self.device)}
                },
            )

        self.latest_images: dict[str, np.ndarray] = {}
        self._image_subscriptions = []

        topic_map = self._parse_image_topic_map(self.image_topics_raw)

        # Validate that all expected image keys from policy are in the topic mapping
        missing_keys = [
            key for key in self.expected_image_keys if key not in topic_map
        ]
        if missing_keys:
            raise ValueError(
                f"Policy expects image keys {self.expected_image_keys}, "
                f"but the following are missing from 'image_topics' mapping: {missing_keys}. "
                f"Available topic mappings: {list(topic_map.keys())}"
            )

        # Subscribe to all image topics and just buffer them
        for key, topic in topic_map.items():
            sub = self.create_subscription(
                msg_type=Image,
                topic=topic,
                callback=lambda msg, image_key=key: self.listener_callback(
                    msg, image_key
                ),
                qos_profile=1,
            )
            self._image_subscriptions.append(sub)

        self.joint_state_subscription = self.create_subscription(
            msg_type=JointState,
            topic=self.joint_state_topic,
            callback=self.joint_state_callback,
            qos_profile=10,
        )

        # Create inference service
        self.inference_service = self.create_service(
            SmolVLAInference, "smolvla_inference", self.handle_inference_request
        )

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
        self.get_logger().info(
            "State key=%s dim=%d, image keys=%s"
            % (self.state_key, self.state_dim, self.expected_image_keys)
        )
        self.get_logger().info(
            "SmolVLA inference service ready at ~/smolvla_inference. "
            "Call with no arguments to run inference on buffered images."
        )

    def handle_inference_request(
        self, request: SmolVLAInference.Request, response: SmolVLAInference.Response
    ) -> SmolVLAInference.Response:
        """Handle inference service request.

        Checks if all required images are present in the buffer,
        runs inference if available, and returns action or error.
        """
        try:
            # Check which expected image keys are missing
            missing_keys = [
                key for key in self.expected_image_keys
                if key not in self.latest_images
            ]

            if missing_keys:
                response.success = False
                response.action = []
                response.error_message = (
                    f"Cannot infer: missing image keys: {', '.join(missing_keys)}"
                )
                response.missing_keys = missing_keys
                self.get_logger().warn(response.error_message)
                return response

            # All required images are present; run inference
            raw_obs: dict[str, Any] = {
                "task": self.task,
                self.state_key: self.latest_state.copy(),
            }

            # Consume the latest frame from each expected camera key
            for key in self.expected_image_keys:
                raw_obs[key] = self.latest_images.pop(key)

            processed_obs = self.preprocess(raw_obs)
            processed_obs = ensure_policy_input_shapes(
                processed_obs,
                state_key=self.state_key,
                image_keys=self.expected_image_keys,
                device=self.device,
            )

            with torch.no_grad():
                action = self.policy.select_action(processed_obs)
            action = self.postprocess(action)

            action_values = tensor_to_list(action)

            response.success = True
            response.action = action_values
            response.error_message = ""
            response.missing_keys = []

            preview = [round(value, 4) for value in action_values[:6]]
            self.get_logger().info(
                f"Inference successful: action dim={len(action_values)}, preview={preview}"
            )

            return response

        except Exception as exc:
            response.success = False
            response.action = []
            response.error_message = f"Inference failed: {str(exc)}"
            response.missing_keys = []
            self.get_logger().error(response.error_message)
            return response

    def _parse_image_topic_map(self, raw_value: str) -> dict[str, str]:
        try:
            mapping = json.loads(raw_value)
        except json.JSONDecodeError as exc:
            raise ValueError(
                "Parameter 'image_topics' must be a JSON object mapping model image keys to ROS topics."
            ) from exc

        if not isinstance(mapping, dict):
            raise ValueError("Parameter 'image_topics' must decode to an object/dict.")

        parsed: dict[str, str] = {}
        for key, value in mapping.items():
            if not isinstance(key, str) or not isinstance(value, str):
                raise ValueError(
                    "All keys and values in 'image_topics' must be strings."
                )
            parsed[key] = value

        if not parsed:
            raise ValueError("Parameter 'image_topics' must not be empty.")

        return parsed

    def listener_callback(self, msg: Image, image_key: str) -> None:
        """Buffer incoming image; inference is triggered via service call."""
        try:
            image_rgb = ros_image_to_rgb(msg)
            self.latest_images[image_key] = image_rgb
        except Exception as exc:
            self.get_logger().error(f"Failed to decode image for key '{image_key}': {exc}")

    def joint_state_callback(self, msg: JointState) -> None:
        """Buffer the latest robot state from /joint_states."""
        try:
            positions = np.asarray(msg.position, dtype=np.float32)
            if positions.shape[0] != self.state_dim:
                self.get_logger().warn(
                    "Ignoring joint state with %d positions; expected %d."
                    % (positions.shape[0], self.state_dim)
                )
                return

            self.latest_state = positions
        except Exception as exc:
            self.get_logger().error(f"Failed to process joint state message: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmolVlaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
