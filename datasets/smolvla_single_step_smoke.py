#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Callable

import cv2
import numpy as np
import torch

PolicyFactory = Callable[[str], Any]
ProcessorFactory = Callable[
    ..., tuple[Callable[[dict[str, Any]], dict[str, Any]], Callable[[Any], Any]]
]


def default_policy_loader(policy_path: str) -> Any:
    # Prefer direct module import because some LeRobot builds don't re-export
    # SmolVLAPolicy at lerobot.policies.smolvla package level.
    try:
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        return SmolVLAPolicy.from_pretrained(policy_path)
    except ModuleNotFoundError as exc:
        if "transformers" in str(exc):
            raise RuntimeError(
                "SmolVLA requires the 'transformers' dependency, but it is missing. "
                "Install it in this environment and re-run."
            ) from exc

    try:
        from lerobot.policies.smolvla import SmolVLAPolicy  # type: ignore[attr-defined]

        return SmolVLAPolicy.from_pretrained(policy_path)
    except Exception:
        pass

    # Last resort: generic loader. This may fail for older LeRobot versions that
    # cannot materialize SmolVLA policies from config.
    try:
        from lerobot.policies.pretrained import PreTrainedPolicy

        return PreTrainedPolicy.from_pretrained(policy_path)
    except Exception as exc:
        raise RuntimeError(
            "Failed to load policy. This LeRobot version may not support SmolVLA loading for this model. "
            "Try upgrading LeRobot and ensuring SmolVLA dependencies are installed (including transformers)."
        ) from exc


def default_processor_factory(*args: Any, **kwargs: Any):
    try:
        from lerobot.policies import make_pre_post_processors  # type: ignore[attr-defined]
    except ImportError:
        from lerobot.policies.factory import make_pre_post_processors
    return make_pre_post_processors(*args, **kwargs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Minimal SmolVLA smoke test: load one policy, one image, one state, then run one select_action call."
        )
    )
    parser.add_argument(
        "--policy-path",
        required=True,
        help="Policy path or Hub id, for example lerobot/smolvla_base or your finetuned checkpoint.",
    )
    parser.add_argument(
        "--image-path",
        type=Path,
        required=True,
        help="Path to one RGB image file.",
    )
    parser.add_argument(
        "--state",
        default=None,
        help="Comma-separated robot state values. If omitted, a zero vector matching policy state dimension is used.",
    )
    parser.add_argument(
        "--task",
        default="Test one-step action",
        help="Language instruction passed to the policy preprocessor.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        choices=["auto", "cpu", "cuda", "mps"],
        help="Inference device.",
    )
    return parser.parse_args()


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


def read_rgb_image(path: Path) -> np.ndarray:
    image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f"Could not read image at {path}")
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def read_first_video_frame(path: Path) -> np.ndarray:
    cap = cv2.VideoCapture(str(path))
    if cap.isOpened():
        ok, frame_bgr = cap.read()
        cap.release()
        if ok and frame_bgr is not None:
            return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path is None:
        raise ValueError(
            f"Could not read first frame from {path} with OpenCV and ffmpeg is not available."
        )

    with tempfile.TemporaryDirectory(prefix="smolvla_frame_") as tmp_dir:
        out_path = Path(tmp_dir) / "frame0.png"
        cmd = [
            ffmpeg_path,
            "-v",
            "error",
            "-i",
            str(path),
            "-frames:v",
            "1",
            "-f",
            "image2",
            "-y",
            str(out_path),
        ]
        completed = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if completed.returncode != 0 or not out_path.exists():
            raise ValueError(
                "Could not read first frame from "
                f"{path}. OpenCV decode failed and ffmpeg fallback failed: {completed.stderr.strip()}"
            )
        return read_rgb_image(out_path)


def _feature_type_text(feature: Any) -> str:
    value = getattr(feature, "type", "")
    return str(value).lower()


def infer_feature_keys(config: Any) -> tuple[str, list[str], int]:
    input_features = getattr(config, "input_features", {}) or {}

    state_key = "observation.state"
    state_dim = 0
    image_keys: list[str] = []

    for key, feature in input_features.items():
        feature_type = _feature_type_text(feature)
        shape = tuple(getattr(feature, "shape", ()) or ())
        if "state" in feature_type:
            state_key = key
            if shape:
                state_dim = int(shape[0])
        elif "visual" in feature_type or "image" in feature_type:
            image_keys.append(key)

    if not image_keys:
        image_keys = ["observation.images.base"]

    if state_dim <= 0:
        # Fallback to commonly used 9D panda state (7 arm + 2 gripper) in this workspace.
        state_dim = 9

    return state_key, image_keys, state_dim


def parse_state(raw: str | None, expected_dim: int) -> np.ndarray:
    if raw is None:
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

        # Accept HWC, CHW, BHWC, BCHW and normalize to BCHW.
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


def collect_model_stats(model: Any) -> dict[str, Any]:
    model_class = model.__class__.__name__
    config_class = getattr(
        getattr(model, "config", None), "__class__", type(None)
    ).__name__

    total_params = 0
    trainable_params = 0
    total_param_bytes = 0
    total_buffer_bytes = 0

    if hasattr(model, "parameters"):
        for parameter in model.parameters():
            param_count = int(parameter.numel())
            total_params += param_count
            total_param_bytes += int(param_count * parameter.element_size())
            if bool(getattr(parameter, "requires_grad", False)):
                trainable_params += param_count

    if hasattr(model, "buffers"):
        for buffer in model.buffers():
            buffer_count = int(buffer.numel())
            total_buffer_bytes += int(buffer_count * buffer.element_size())

    estimated_model_size_mb = (total_param_bytes + total_buffer_bytes) / (
        1024.0 * 1024.0
    )

    return {
        "model_class": model_class,
        "config_class": config_class,
        "total_parameters": total_params,
        "trainable_parameters": trainable_params,
        "non_trainable_parameters": total_params - trainable_params,
        "parameter_memory_mb": round(total_param_bytes / (1024.0 * 1024.0), 2),
        "buffer_memory_mb": round(total_buffer_bytes / (1024.0 * 1024.0), 2),
        "estimated_model_size_mb": round(estimated_model_size_mb, 2),
    }


def run_single_step_inference(
    *,
    policy_path: str,
    image_rgb: np.ndarray,
    state_raw: str | None,
    task: str,
    device_raw: str,
    policy_loader: PolicyFactory | None = None,
    processor_factory: ProcessorFactory | None = None,
) -> dict[str, Any]:
    device = choose_device(device_raw)

    load_policy = policy_loader
    if load_policy is None:
        load_policy = default_policy_loader

    make_processors = processor_factory
    if make_processors is None:
        make_processors = default_processor_factory

    model = load_policy(policy_path)
    if hasattr(model, "to"):
        model.to(device)

    model_stats = collect_model_stats(model)

    try:
        preprocess, postprocess = make_processors(
            model.config,
            policy_path,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
    except FileNotFoundError as exc:
        if "policy_preprocessor.json" not in str(exc):
            raise
        print(
            "policy_preprocessor.json not found in model repo; "
            "falling back to config-derived processors."
        )
        preprocess, postprocess = make_processors(
            model.config,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )

    state_key, image_keys, state_dim = infer_feature_keys(model.config)
    state = parse_state(state_raw, state_dim)

    raw_obs: dict[str, Any] = {
        "task": task,
        state_key: state,
    }
    for key in image_keys:
        raw_obs[key] = image_rgb

    processed_obs = preprocess(raw_obs)
    processed_obs = ensure_policy_input_shapes(
        processed_obs, state_key=state_key, image_keys=image_keys, device=device
    )

    with torch.no_grad():
        action = model.select_action(processed_obs)
    action = postprocess(action)

    return {
        "device": str(device),
        "policy_path": policy_path,
        "model": model_stats,
        "state_key": state_key,
        "image_keys": image_keys,
        "state_dim": int(state_dim),
        "action_dim": int(np.asarray(tensor_to_list(action)).shape[0]),
        "action": tensor_to_list(action),
    }


def main() -> None:
    args = parse_args()
    image = read_rgb_image(args.image_path)

    print(f"Loading SmolVLA policy from: {args.policy_path}")
    payload = run_single_step_inference(
        policy_path=args.policy_path,
        image_rgb=image,
        state_raw=args.state,
        task=args.task,
        device_raw=args.device,
    )
    model_stats = payload.get("model", {})
    print(f"Model class: {model_stats.get('model_class', 'unknown')}")
    print(f"Config class: {model_stats.get('config_class', 'unknown')}")
    print(f"Total parameters: {model_stats.get('total_parameters', 0):,}")
    print(f"Trainable parameters: {model_stats.get('trainable_parameters', 0):,}")
    print(f"Estimated model size: {model_stats.get('estimated_model_size_mb', 0.0)} MB")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
