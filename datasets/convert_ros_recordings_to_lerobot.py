#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from lerobot.datasets.lerobot_dataset import LeRobotDataset

LOGGER = logging.getLogger("ros_to_lerobot")

POSE_COLUMNS = ["px", "py", "pz", "qx", "qy", "qz", "qw"]
GRIPPER_COLUMN = "gripper_pos"
OLD_IMAGE_COLUMN = "image_file"
EE_IMAGE_COLUMN = "ee_image_file"
BASE_IMAGE_COLUMN = "base_image_file"


@dataclass
class EpisodeSource:
    episode_id: str
    csv_path: Path
    image_dir: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert ROS teleop CSV/JPEG recordings into a LeRobot v3 dataset."
    )
    parser.add_argument(
        "--repo-id", required=True, help="LeRobot dataset repo id, e.g. user/my_dataset"
    )
    parser.add_argument(
        "--recordings-root",
        type=Path,
        default=Path("~/vla_recordings").expanduser(),
        help="Root directory containing episode recordings",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        required=True,
        help="Directory where the converted LeRobot dataset will be written",
    )
    parser.add_argument(
        "--fps", type=int, default=5, help="Recording fps used by the source logs"
    )
    parser.add_argument(
        "--task-map",
        type=Path,
        default=None,
        help='JSON mapping file: {"episode_<id>": "task text", "__default__": "..."}',
    )
    parser.add_argument(
        "--default-task",
        default="teleop demonstration",
        help="Fallback task text when episode is not in task map",
    )
    parser.add_argument(
        "--missing-base-policy",
        choices=["duplicate_ee", "skip_episode", "error"],
        default="duplicate_ee",
        help="Behavior when base camera image is unavailable",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Optional cap for quick test runs",
    )
    parser.add_argument(
        "--robot-type",
        default="panda_teleop",
        help="robot_type stored in LeRobot metadata",
    )
    return parser.parse_args()


def discover_episodes(recordings_root: Path) -> list[EpisodeSource]:
    episodes: list[EpisodeSource] = []

    for item in sorted(recordings_root.iterdir()):
        if item.is_dir() and item.name.startswith("episode_"):
            csv_path = item / "episode.csv"
            if csv_path.exists():
                episodes.append(EpisodeSource(item.name, csv_path, item))
            continue

        if (
            item.is_file()
            and item.suffix == ".csv"
            and item.stem.startswith("episode_")
        ):
            image_dir = recordings_root / f"{item.stem}_images"
            if image_dir.exists() and image_dir.is_dir():
                episodes.append(EpisodeSource(item.stem, item, image_dir))

    return episodes


def load_task_map(task_map_path: Path | None, default_task: str) -> dict[str, str]:
    mapping = {"__default__": default_task}
    if task_map_path is None:
        return mapping

    with task_map_path.open("r", encoding="utf-8") as file_handle:
        payload = json.load(file_handle)

    if not isinstance(payload, dict):
        raise ValueError("Task map must be a JSON object.")

    for key, value in payload.items():
        if not isinstance(key, str) or not isinstance(value, str):
            raise ValueError("Task map must contain string keys and string values.")
        mapping[key] = value

    return mapping


def get_task_for_episode(task_map: dict[str, str], episode_id: str) -> str:
    if episode_id in task_map:
        return task_map[episode_id]
    return task_map.get("__default__", "teleop demonstration")


def resolve_image_path(image_dir: Path, image_name: str | float | None) -> Path | None:
    if image_name is None or (isinstance(image_name, float) and np.isnan(image_name)):
        return None

    name = str(image_name).strip()
    if not name:
        return None

    candidate = Path(name)
    if candidate.is_absolute():
        return candidate
    return image_dir / candidate


def load_rgb_image(image_path: Path) -> np.ndarray:
    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def image_shape_3d(image: np.ndarray, source: str) -> tuple[int, int, int]:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 image for {source}, got shape {image.shape}")
    return (int(image.shape[0]), int(image.shape[1]), int(image.shape[2]))


def episode_joint_columns(df: pd.DataFrame) -> list[str]:
    return [column for column in df.columns if column.startswith("joint_")]


def state_vector_from_row(row: pd.Series, joint_columns: list[str]) -> np.ndarray:
    values: list[float] = []

    for column in joint_columns:
        values.append(float(row[column]))

    values.append(float(row[GRIPPER_COLUMN]))

    for column in POSE_COLUMNS:
        values.append(float(row[column]))

    return np.asarray(values, dtype=np.float32)


def infer_camera_shapes(
    sample_episode: EpisodeSource, missing_base_policy: str
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    df = pd.read_csv(sample_episode.csv_path)
    if df.empty:
        raise ValueError(f"Sample episode is empty: {sample_episode.csv_path}")

    row = df.iloc[0]
    ee_col = EE_IMAGE_COLUMN if EE_IMAGE_COLUMN in df.columns else OLD_IMAGE_COLUMN
    ee_path = resolve_image_path(sample_episode.image_dir, row.get(ee_col))
    if ee_path is None:
        raise ValueError("Could not infer ee camera shape from sample episode.")

    ee_image = load_rgb_image(ee_path)
    ee_shape = image_shape_3d(ee_image, f"{sample_episode.episode_id}:ee")

    base_shape = ee_shape
    if BASE_IMAGE_COLUMN in df.columns:
        base_path = resolve_image_path(
            sample_episode.image_dir, row.get(BASE_IMAGE_COLUMN)
        )
        if base_path is not None and base_path.exists():
            base_image = load_rgb_image(base_path)
            base_shape = image_shape_3d(base_image, f"{sample_episode.episode_id}:base")
        elif missing_base_policy == "error":
            raise ValueError(
                f"Base image missing in sample episode: {sample_episode.episode_id}"
            )

    return ee_shape, base_shape


def build_features(
    state_names: list[str],
    ee_shape: tuple[int, int, int],
    base_shape: tuple[int, int, int],
) -> dict[str, dict]:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": [f"delta_{name}" for name in state_names],
        },
        "observation.images.ee": {
            "dtype": "video",
            "shape": ee_shape,
            "names": ["height", "width", "channels"],
        },
        "observation.images.base": {
            "dtype": "video",
            "shape": base_shape,
            "names": ["height", "width", "channels"],
        },
    }


def validate_required_columns(df: pd.DataFrame, csv_path: Path) -> None:
    missing = [
        column for column in [*POSE_COLUMNS, GRIPPER_COLUMN] if column not in df.columns
    ]
    if not episode_joint_columns(df):
        missing.append("joint_* columns")
    if missing:
        raise ValueError(f"Missing required columns in {csv_path}: {missing}")

    if EE_IMAGE_COLUMN not in df.columns and OLD_IMAGE_COLUMN not in df.columns:
        raise ValueError(
            f"Missing image column in {csv_path}. Expected '{EE_IMAGE_COLUMN}' or '{OLD_IMAGE_COLUMN}'."
        )


def convert_episode(
    dataset: LeRobotDataset,
    episode: EpisodeSource,
    joint_columns: list[str],
    task: str,
    missing_base_policy: str,
) -> tuple[int, int]:
    df = pd.read_csv(episode.csv_path)
    if df.empty:
        LOGGER.warning("Skipping empty episode: %s", episode.csv_path)
        return 0, 1

    validate_required_columns(df, episode.csv_path)

    if len(df) < 2:
        LOGGER.warning("Skipping short episode (<2 rows): %s", episode.csv_path)
        return 0, 1

    ee_col = EE_IMAGE_COLUMN if EE_IMAGE_COLUMN in df.columns else OLD_IMAGE_COLUMN

    frame_count = 0
    skipped = 0
    for index in range(len(df) - 1):
        row = df.iloc[index]
        next_row = df.iloc[index + 1]

        obs_state = state_vector_from_row(row, joint_columns)
        next_state = state_vector_from_row(next_row, joint_columns)
        action = (next_state - obs_state).astype(np.float32)

        ee_path = resolve_image_path(episode.image_dir, row.get(ee_col))
        if ee_path is None or not ee_path.exists():
            LOGGER.warning("Missing ee image in %s row %d", episode.episode_id, index)
            skipped += 1
            continue

        ee_image = load_rgb_image(ee_path)

        base_image: np.ndarray
        base_path = None
        if BASE_IMAGE_COLUMN in df.columns:
            base_path = resolve_image_path(
                episode.image_dir, row.get(BASE_IMAGE_COLUMN)
            )

        if base_path is not None and base_path.exists():
            base_image = load_rgb_image(base_path)
        elif missing_base_policy == "duplicate_ee":
            base_image = ee_image
        elif missing_base_policy == "skip_episode":
            LOGGER.warning(
                "Skipping episode due to missing base image: %s", episode.episode_id
            )
            dataset.clear_episode_buffer(delete_images=True)
            return 0, 1
        else:
            raise FileNotFoundError(
                f"Missing base image in episode {episode.episode_id} at row {index}"
            )

        frame = {
            "observation.state": obs_state,
            "action": action,
            "observation.images.ee": ee_image,
            "observation.images.base": base_image,
            "task": task,
        }
        dataset.add_frame(frame)
        frame_count += 1

    if frame_count == 0:
        dataset.clear_episode_buffer(delete_images=True)
        LOGGER.warning("Skipping episode with no valid frames: %s", episode.episode_id)
        return 0, 1

    dataset.save_episode()
    return frame_count, skipped


def run_conversion(args: argparse.Namespace) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    recordings_root = args.recordings_root.expanduser()
    output_root = args.output_root.expanduser()

    episodes = discover_episodes(recordings_root)
    if not episodes:
        raise FileNotFoundError(f"No episodes discovered under {recordings_root}")

    if args.max_episodes is not None:
        episodes = episodes[: args.max_episodes]

    task_map = load_task_map(args.task_map, args.default_task)

    sample_df = pd.read_csv(episodes[0].csv_path)
    validate_required_columns(sample_df, episodes[0].csv_path)
    joint_columns = episode_joint_columns(sample_df)

    state_names = [*joint_columns, GRIPPER_COLUMN, *POSE_COLUMNS]
    ee_shape, base_shape = infer_camera_shapes(episodes[0], args.missing_base_policy)
    features = build_features(state_names, ee_shape, base_shape)

    if output_root.exists():
        raise FileExistsError(
            f"Output root already exists: {output_root}. Provide a new path or remove the existing directory."
        )

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=output_root,
        fps=args.fps,
        features=features,
        robot_type=args.robot_type,
        use_videos=True,
    )

    total_frames = 0
    skipped_episodes = 0

    try:
        for episode in episodes:
            task = get_task_for_episode(task_map, episode.episode_id)
            frames_written, skipped = convert_episode(
                dataset=dataset,
                episode=episode,
                joint_columns=joint_columns,
                task=task,
                missing_base_policy=args.missing_base_policy,
            )
            total_frames += frames_written
            skipped_episodes += skipped
            if frames_written > 0:
                LOGGER.info(
                    "Converted %s (%d frames)", episode.episode_id, frames_written
                )
    finally:
        dataset.finalize()

    LOGGER.info("Done. Episodes attempted: %d", len(episodes))
    LOGGER.info("Episodes skipped: %d", skipped_episodes)
    LOGGER.info("Frames written: %d", total_frames)
    LOGGER.info("Output dataset root: %s", output_root)


def main() -> None:
    args = parse_args()
    run_conversion(args)


if __name__ == "__main__":
    main()
