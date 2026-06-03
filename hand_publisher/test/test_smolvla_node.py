"""Unit test for SmolVlaNode ROS2 node.

Tests that the node:
1. Starts successfully
2. Subscribes to image topics
3. Processes incoming Image messages
4. Calls policy.select_action()
5. Logs action output
"""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock, patch
from typing import Any

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Image
import torch


# Minimal fake policy that matches SmolVLA interface
class FakeSmolVLAPolicy:
    def __init__(self):
        self.config = MagicMock()
        self.config.input_features = {
            "observation.images.laptop": {"type": "visual"},
            "observation.images.phone": {"type": "visual"},
            "observation.state": {"type": "state", "shape": [6]},
        }
        self.call_count = 0

    def to(self, device):
        return self

    def select_action(self, processed_obs: dict[str, Any]) -> np.ndarray:
        self.call_count += 1
        return np.array([0.1] * 6, dtype=np.float32)


class FakePreprocessor:
    def __init__(self):
        self.call_count = 0

    def __call__(self, obs: dict[str, Any]) -> dict[str, Any]:
        self.call_count += 1
        return obs


class FakePostprocessor:
    def __init__(self):
        self.call_count = 0

    def __call__(self, action: Any) -> Any:
        self.call_count += 1
        return action


@pytest.fixture
def ros_context():
    """Initialize and teardown ROS context for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


def create_image_message(width: int, height: int, encoding: str = "rgb8") -> Image:
    """Create a dummy Image message for testing."""
    msg = Image()
    msg.header.frame_id = "camera_frame"
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = width * 3
    msg.data = (np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)).tobytes()
    return msg


def test_smolvla_node_imports(ros_context):
    """Test that the SmolVlaNode can be imported without errors."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    assert SmolVlaNode is not None


def test_smolvla_node_initialization_basic(ros_context):
    """Test that SmolVlaNode initializes with mocked policy."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()
        assert node.policy is fake_policy
        assert node.preprocess is fake_preprocess
        assert node.postprocess is fake_postprocess
        assert node.state_key == "observation.state"
        assert "observation.images.laptop" in node.expected_image_keys
        assert "observation.images.phone" in node.expected_image_keys
        node.destroy_node()


def test_smolvla_node_callback_waits_for_both_images(ros_context):
    """Test that callback just buffers images; service checks completeness."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()
        image_msg_laptop = create_image_message(width=640, height=480, encoding="rgb8")
        image_msg_phone = create_image_message(width=640, height=480, encoding="rgb8")

        # Callback just buffers; no inference yet
        node.listener_callback(image_msg_laptop, "observation.images.laptop")
        assert fake_policy.call_count == 0

        # Still no inference with second image
        node.listener_callback(image_msg_phone, "observation.images.phone")
        assert fake_policy.call_count == 0

        # Now call the service to trigger inference
        from hand_publisher_interfaces.srv import SmolVLAInference
        request = SmolVLAInference.Request()
        response = SmolVLAInference.Response()
        node.handle_inference_request(request, response)

        assert response.success
        assert len(response.action) == 6
        assert fake_policy.call_count == 1
        assert fake_preprocess.call_count == 1
        assert fake_postprocess.call_count == 1

        node.destroy_node()


def test_smolvla_node_buffers_multiple_images(ros_context):
    """Test that multiple image updates are buffered and latest pair is used on service call."""
    from hand_publisher_node.smolvla_node import SmolVlaNode
    from hand_publisher_interfaces.srv import SmolVLAInference

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()

        # Add first image pair
        image_msg_1 = create_image_message(width=640, height=480)
        node.listener_callback(image_msg_1, "observation.images.laptop")
        assert fake_policy.call_count == 0

        image_msg_2 = create_image_message(width=320, height=240)
        node.listener_callback(image_msg_2, "observation.images.phone")
        assert fake_policy.call_count == 0

        # Request inference - should succeed with buffered pair
        request = SmolVLAInference.Request()
        response = SmolVLAInference.Response()
        node.handle_inference_request(request, response)
        assert response.success
        assert fake_policy.call_count == 1

        # Add another image from only one camera
        image_msg_3 = create_image_message(width=640, height=480)
        node.listener_callback(image_msg_3, "observation.images.laptop")

        # Second request should fail because previous inference consumes both keys.
        response2 = SmolVLAInference.Response()
        node.handle_inference_request(request, response2)
        assert not response2.success
        assert "observation.images.phone" in response2.missing_keys
        assert fake_policy.call_count == 1

        node.destroy_node()


def test_smolvla_node_ros_image_rgb8_encoding(ros_context):
    """Test ROS Image RGB8 encoding handling."""
    from hand_publisher_node.smolvla_node import ros_image_to_rgb

    msg_rgb = create_image_message(640, 480, "rgb8")
    result = ros_image_to_rgb(msg_rgb)

    assert result.shape == (480, 640, 3)
    assert result.dtype == np.uint8


def test_smolvla_node_ros_image_bgr8_encoding(ros_context):
    """Test ROS Image BGR8 encoding handling."""
    from hand_publisher_node.smolvla_node import ros_image_to_rgb

    msg_bgr = create_image_message(640, 480, "bgr8")
    result = ros_image_to_rgb(msg_bgr)

    assert result.shape == (480, 640, 3)
    assert result.dtype == np.uint8


def test_smolvla_node_callback_error_handling(ros_context):
    """Test that callback image decoding errors are logged without crashing."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ), patch(
        "hand_publisher_node.smolvla_node.ros_image_to_rgb",
        side_effect=ValueError("Invalid image format"),
    ):
        node = SmolVlaNode()

        image_msg = create_image_message(640, 480)
        # Callback should catch the error and log it
        node.listener_callback(image_msg, "observation.images.laptop")

        # Node should still be healthy
        assert node is not None
        # Image should not be buffered due to error
        assert "observation.images.laptop" not in node.latest_images

        node.destroy_node()


def test_smolvla_node_parse_image_topics_valid(ros_context):
    """Test JSON parsing of valid image_topics parameter."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()
        mapping = node._parse_image_topic_map(
            '{"obs.img.base": "/topic1", "obs.img.ee": "/topic2"}'
        )
        assert mapping == {"obs.img.base": "/topic1", "obs.img.ee": "/topic2"}
        node.destroy_node()


def test_smolvla_node_parse_image_topics_invalid_json(ros_context):
    """Test that invalid JSON in image_topics raises error."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()
        with pytest.raises(ValueError, match="must be a JSON"):
            node._parse_image_topic_map("not valid json")
        node.destroy_node()


def test_smolvla_node_service_inference_success(ros_context):
    """Test that service handler returns successful inference with buffered images."""
    from hand_publisher_node.smolvla_node import SmolVlaNode
    from hand_publisher_interfaces.srv import SmolVLAInference

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()

        # Buffer both images
        node.listener_callback(
            create_image_message(640, 480),
            "observation.images.laptop"
        )
        node.listener_callback(
            create_image_message(640, 480),
            "observation.images.phone"
        )

        # Call service
        request = SmolVLAInference.Request()
        response = SmolVLAInference.Response()
        result = node.handle_inference_request(request, response)

        assert result.success
        assert len(result.action) == 6
        assert result.error_message == ""
        assert result.missing_keys == []

        node.destroy_node()


def test_smolvla_node_service_inference_missing_images(ros_context):
    """Test that service handler reports missing images correctly."""
    from hand_publisher_node.smolvla_node import SmolVlaNode
    from hand_publisher_interfaces.srv import SmolVLAInference

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()

        # Buffer only one image
        node.listener_callback(
            create_image_message(640, 480),
            "observation.images.laptop"
        )

        # Call service
        request = SmolVLAInference.Request()
        response = SmolVLAInference.Response()
        result = node.handle_inference_request(request, response)

        assert not result.success
        assert len(result.action) == 0
        assert "observation.images.phone" in result.missing_keys
        assert "missing image keys" in result.error_message.lower()

        node.destroy_node()


def test_smolvla_node_config_validation_missing_image_key(ros_context):
    """Test that node fails fast if policy expects image key not in topic mapping."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_preprocess = FakePreprocessor()
    fake_postprocess = FakePostprocessor()

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(fake_preprocess, fake_postprocess),
    ), patch.object(
        SmolVlaNode,
        "_parse_image_topic_map",
        return_value={"observation.images.laptop": "/topic1"},
    ):
        with pytest.raises(ValueError, match="missing from 'image_topics'"):
            SmolVlaNode()


def test_smolvla_device_validation_cuda_unavailable(ros_context):
    """Test that explicit CUDA request fails fast if CUDA is unavailable."""
    from hand_publisher_node.smolvla_node import choose_device

    with patch("torch.cuda.is_available", return_value=False), patch(
        "torch.backends.mps.is_available", return_value=False
    ):
        with pytest.raises(RuntimeError, match="CUDA is not available"):
            choose_device("cuda")


def test_smolvla_device_validation_mps_unavailable(ros_context):
    """Test that explicit MPS request fails fast if MPS is unavailable."""
    from hand_publisher_node.smolvla_node import choose_device

    with patch("torch.cuda.is_available", return_value=False), patch(
        "torch.backends.mps.is_available", return_value=False
    ):
        with pytest.raises(RuntimeError, match="MPS is not available"):
            choose_device("mps")


@pytest.mark.integration
def test_real_model_config_matches_ground_truth_cuda(ros_context):
    """Load Hartvi/smolvla on CUDA and verify config keys/shapes from config.json."""
    if not torch.cuda.is_available():
        pytest.skip("CUDA is required for this integration test")

    from hand_publisher_node.smolvla_node import default_policy_loader

    local_config_path = (
        Path(__file__).resolve().parents[1] / "hand_publisher_node" / "config.json"
    )
    with local_config_path.open("r", encoding="utf-8") as fh:
        expected = json.load(fh)

    policy = default_policy_loader("Hartvi/smolvla")
    policy.to(torch.device("cuda"))

    print("Loaded policy config:", policy.config)
    input_features = policy.config.input_features
    output_features = policy.config.output_features

    assert "observation.images.laptop" in input_features
    assert "observation.images.phone" in input_features
    assert "observation.state" in input_features
    assert "action" in output_features

    laptop_shape = list(
        getattr(input_features["observation.images.laptop"], "shape", [])
    )
    phone_shape = list(getattr(input_features["observation.images.phone"], "shape", []))
    state_shape = list(getattr(input_features["observation.state"], "shape", []))
    action_shape = list(getattr(output_features["action"], "shape", []))

    assert (
        laptop_shape == expected["input_features"]["observation.images.laptop"]["shape"]
    )
    assert (
        phone_shape == expected["input_features"]["observation.images.phone"]["shape"]
    )
    assert state_shape == expected["input_features"]["observation.state"]["shape"]
    assert action_shape == expected["output_features"]["action"]["shape"]


@pytest.mark.integration
def test_real_model_inference_cuda_with_laptop_phone_labels(ros_context):
    """Run one real CUDA inference using laptop/phone observation labels."""
    if not torch.cuda.is_available():
        pytest.skip("CUDA is required for this integration test")

    from hand_publisher_node.smolvla_node import (
        default_policy_loader,
        default_processor_factory,
        ensure_policy_input_shapes,
        infer_feature_keys,
        tensor_to_list,
    )

    device = torch.device("cuda")
    policy = default_policy_loader("Hartvi/smolvla")
    policy.to(device)

    try:
        preprocess, postprocess = default_processor_factory(
            policy.config,
            "Hartvi/smolvla",
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
    except FileNotFoundError as exc:
        if "policy_preprocessor.json" not in str(exc):
            raise
        preprocess, postprocess = default_processor_factory(
            policy.config,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )

    state_key, image_keys, state_dim = infer_feature_keys(policy.config)
    assert state_key == "observation.state"
    assert set(["observation.images.laptop", "observation.images.phone"]).issubset(
        set(image_keys)
    )
    assert state_dim == 6

    raw_obs = {
        "task": "Teleop task",
        state_key: np.zeros((state_dim,), dtype=np.float32),
        "observation.images.laptop": np.zeros((480, 640, 3), dtype=np.uint8),
        "observation.images.phone": np.zeros((480, 640, 3), dtype=np.uint8),
    }

    processed = preprocess(raw_obs)
    processed = ensure_policy_input_shapes(
        processed,
        state_key=state_key,
        image_keys=["observation.images.laptop", "observation.images.phone"],
        device=device,
    )

    with torch.no_grad():
        action = policy.select_action(processed)
    action = postprocess(action)
    action_values = tensor_to_list(action)

    assert len(action_values) == 6
    assert all(np.isfinite(action_values))
    # TODO: log action values, state values, observation keys
    print("Action values:", action_values)
    print("State values:", processed[state_key])
    print("Observation keys:", list(processed.keys()))


@pytest.mark.integration
def test_real_smolvla_node_one_step_inference_cuda(ros_context):
    """Initialize SmolVlaNode and trigger one-step inference with laptop/phone images."""
    if not torch.cuda.is_available():
        pytest.skip("CUDA is required for this integration test")

    from hand_publisher_node.smolvla_node import SmolVlaNode
    from hand_publisher_interfaces.srv import SmolVLAInference

    node = SmolVlaNode()

    # Wrap the real select_action to verify service-triggered forward pass count.
    original_select_action = node.policy.select_action
    call_counter = {"count": 0}

    def counted_select_action(processed_obs: dict[str, Any]):
        call_counter["count"] += 1
        return original_select_action(processed_obs)

    node.policy.select_action = counted_select_action  # type: ignore[method-assign]

    try:
        image_msg_laptop = create_image_message(width=640, height=480, encoding="rgb8")
        image_msg_phone = create_image_message(width=640, height=480, encoding="rgb8")

        # Buffers should not trigger inference
        node.listener_callback(image_msg_laptop, "observation.images.laptop")
        assert call_counter["count"] == 0

        node.listener_callback(image_msg_phone, "observation.images.phone")
        assert call_counter["count"] == 0

        # Service call should trigger inference
        request = SmolVLAInference.Request()
        response = SmolVLAInference.Response()
        node.handle_inference_request(request, response)
        assert response.success
        assert call_counter["count"] == 1

        # Service consumes both buffered keys on successful inference.
        assert "observation.images.laptop" not in node.latest_images
        assert "observation.images.phone" not in node.latest_images
    finally:
        node.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
