"""Unit test for SmolVlaNode ROS2 node.

Tests that the node:
1. Starts successfully
2. Subscribes to image topics
3. Processes incoming Image messages
4. Calls policy.select_action()
5. Logs action output
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch
from typing import Any

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Image


# Minimal fake policy that matches SmolVLA interface
class FakeSmolVLAPolicy:
    def __init__(self):
        self.config = MagicMock()
        self.config.input_features = {
            "observation.images.base": {"type": "video"},
            "observation.images.ee": {"type": "video"},
            "observation.state": {"type": "state", "shape": [6]},
        }
        self.call_count = 0

    def to(self, device):
        return self

    def select_action(self, processed_obs: dict[str, Any]) -> np.ndarray:
        self.call_count += 1
        return np.array([0.1, 0.2, 0.3], dtype=np.float32)


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
        assert "observation.images.base" in node.expected_image_keys
        node.destroy_node()


def test_smolvla_node_callback_waits_for_both_images(ros_context):
    """Test that inference runs only after both expected camera images arrive."""
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
        image_msg_base = create_image_message(width=640, height=480, encoding="rgb8")
        image_msg_ee = create_image_message(width=640, height=480, encoding="rgb8")

        node.listener_callback(image_msg_base, "observation.images.base")
        assert fake_policy.call_count == 0

        node.listener_callback(image_msg_ee, "observation.images.ee")

        assert fake_policy.call_count == 1
        assert fake_preprocess.call_count == 1
        assert fake_postprocess.call_count == 1

        node.destroy_node()


def test_smolvla_node_buffers_multiple_images(ros_context):
    """Test that paired image updates are consumed once per full camera pair."""
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
        image_msg_1 = create_image_message(width=640, height=480)
        node.listener_callback(image_msg_1, "observation.images.base")
        assert fake_policy.call_count == 0

        image_msg_2 = create_image_message(width=320, height=240)
        node.listener_callback(image_msg_2, "observation.images.ee")
        assert fake_policy.call_count == 1

        # One more frame from only one camera should not trigger inference.
        image_msg_3 = create_image_message(width=640, height=480)
        node.listener_callback(image_msg_3, "observation.images.base")
        assert fake_policy.call_count == 1

        # Second camera update completes a fresh pair and triggers one more inference.
        image_msg_4 = create_image_message(width=320, height=240)
        node.listener_callback(image_msg_4, "observation.images.ee")

        assert fake_policy.call_count == 2
        assert "observation.images.base" in node.latest_images
        assert "observation.images.ee" in node.latest_images

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
    """Test that callback errors are caught and logged without crashing."""
    from hand_publisher_node.smolvla_node import SmolVlaNode

    fake_policy = FakeSmolVLAPolicy()
    fake_postprocess = FakePostprocessor()

    # Create a preprocessor that raises an error
    error_preprocess = MagicMock(side_effect=RuntimeError("Preprocess error"))

    with patch(
        "hand_publisher_node.smolvla_node.default_policy_loader",
        return_value=fake_policy,
    ), patch(
        "hand_publisher_node.smolvla_node.default_processor_factory",
        return_value=(error_preprocess, fake_postprocess),
    ):
        node = SmolVlaNode()

        image_msg_base = create_image_message(640, 480)
        image_msg_ee = create_image_message(640, 480)
        node.listener_callback(image_msg_base, "observation.images.base")
        node.listener_callback(image_msg_ee, "observation.images.ee")

        # Node should still exist and error should be logged
        assert node is not None
        # select_action should not have been called due to error in preprocessing
        assert fake_policy.call_count == 0

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
