from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

from smolvla_single_step_smoke import read_first_video_frame, run_single_step_inference


@dataclass
class FakeFeature:
    type: str
    shape: tuple[int, ...]


class FakeConfig:
    def __init__(self):
        self.input_features = {
            "observation.state": FakeFeature(type="STATE", shape=(15,)),
            "observation.images.base": FakeFeature(type="VISUAL", shape=(3, 224, 224)),
        }


class FakePolicy:
    def __init__(self):
        self.config = FakeConfig()

    def to(self, _device: torch.device) -> None:
        return

    def select_action(self, _obs: dict[str, Any]) -> torch.Tensor:
        return torch.tensor([0.1, -0.2, 0.3], dtype=torch.float32)


def fake_preprocess(obs: dict[str, Any]) -> dict[str, Any]:
    return obs


def fake_postprocess(action: Any) -> Any:
    return action


def fake_processor_factory(*_args: Any, **_kwargs: Any):
    return fake_preprocess, fake_postprocess


def test_single_step_inference_with_dataset_video_frame() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    video_path = (
        repo_root
        / "datasets"
        / "test_dataset_out"
        / "videos"
        / "observation.images.base"
        / "chunk-000"
        / "file-000.mp4"
    )
    assert video_path.exists(), f"Expected test video not found: {video_path}"

    image = read_first_video_frame(video_path)
    assert image.ndim == 3
    assert image.shape[2] == 3

    state_values = ",".join(["0.0"] * 15)
    payload = run_single_step_inference(
        policy_path="dummy/smolvla",
        image_rgb=image,
        state_raw=state_values,
        task="Unit test one-step",
        device_raw="cpu",
        policy_loader=lambda _path: FakePolicy(),
        processor_factory=fake_processor_factory,
    )

    assert payload["state_dim"] == 15
    assert payload["action_dim"] == 3
    assert payload["state_key"] == "observation.state"
    assert payload["image_keys"] == ["observation.images.base"]
    assert isinstance(payload["action"], list)
    assert len(payload["action"]) == 3
