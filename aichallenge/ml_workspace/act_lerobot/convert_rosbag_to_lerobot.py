#!/usr/bin/env python3
"""Convert AI Challenge rosbag demonstrations to a LeRobotDataset.

The resulting dataset is intended for LeRobot ACT training. Each rosbag
directory is saved as one episode with:

  observation.images.front: camera image
  action: [acceleration, steering_tire_angle]
  observation.state: optional vehicle state, depending on --state-mode
"""

from __future__ import annotations

import argparse
import logging
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

try:
    from lerobot.datasets import LeRobotDataset
except ImportError:  # Older LeRobot releases used this path.
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise SystemExit(
            "LeRobot is not installed. Install it in your training environment, "
            "then rerun this converter."
        ) from exc


LOGGER = logging.getLogger("convert_rosbag_to_lerobot")


@dataclass
class Sample:
    timestamp_ns: int
    image: np.ndarray
    action: np.ndarray
    state: np.ndarray | None


def image_msg_to_rgb(msg: Any) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if encoding == "bgr8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img.copy()
    if encoding in ("bgra8", "rgba8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2RGB if encoding == "bgra8" else cv2.COLOR_RGBA2RGB
        return cv2.cvtColor(img, code)
    if encoding == "mono8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def process_image(img: np.ndarray, height: int, width: int, crop_top_ratio: float) -> np.ndarray:
    if crop_top_ratio >= 1.0:
        raise ValueError("--crop-top-ratio must be < 1.0")
    if crop_top_ratio > 0:
        top = int(img.shape[0] * crop_top_ratio)
        img = img[top:, :, :]
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_LINEAR)


def nearest_indices(src_times: np.ndarray, target_times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(target_times) == 0:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)
    idx = np.searchsorted(target_times, src_times)
    idx = np.clip(idx, 0, len(target_times) - 1)
    prev = np.clip(idx - 1, 0, len(target_times) - 1)
    use_prev = np.abs(target_times[prev] - src_times) < np.abs(target_times[idx] - src_times)
    out = np.where(use_prev, prev, idx)
    return out, np.abs(target_times[out] - src_times)


def odometry_state(msg: Any) -> np.ndarray:
    twist = msg.twist.twist
    return np.array(
        [
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        ],
        dtype=np.float32,
    )


def velocity_state(msg: Any) -> np.ndarray:
    return np.array([msg.longitudinal_velocity, msg.heading_rate], dtype=np.float32)


def read_bag_episode(
    bag_path: Path,
    *,
    image_topic: str,
    control_topic: str,
    state_topic: str | None,
    state_mode: str,
    image_height: int,
    image_width: int,
    crop_top_ratio: float,
    max_sync_dt: float,
) -> list[Sample]:
    images: list[np.ndarray] = []
    image_times: list[int] = []
    actions: list[np.ndarray] = []
    action_times: list[int] = []
    states: list[np.ndarray] = []
    state_times: list[int] = []

    topics = [image_topic, control_topic]
    if state_topic:
        topics.append(state_topic)

    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic in topics]
        for conn, timestamp, raw in reader.messages(connections=connections):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception:
                continue

            if conn.topic == image_topic:
                try:
                    images.append(process_image(image_msg_to_rgb(msg), image_height, image_width, crop_top_ratio))
                    image_times.append(timestamp)
                except ValueError as exc:
                    LOGGER.warning("%s: skipped image: %s", bag_path.name, exc)
            elif conn.topic == control_topic:
                actions.append(
                    np.array(
                        [msg.longitudinal.acceleration, msg.lateral.steering_tire_angle],
                        dtype=np.float32,
                    )
                )
                action_times.append(timestamp)
            elif conn.topic == state_topic:
                if state_mode == "odometry":
                    states.append(odometry_state(msg))
                    state_times.append(timestamp)
                elif state_mode == "vehicle_status":
                    states.append(velocity_state(msg))
                    state_times.append(timestamp)

    if not images or not actions:
        return []

    image_times_np = np.asarray(image_times, dtype=np.int64)
    action_times_np = np.asarray(action_times, dtype=np.int64)
    action_order = np.argsort(action_times_np)
    action_times_np = action_times_np[action_order]
    actions_np = np.asarray(actions, dtype=np.float32)[action_order]

    action_idx, action_dt = nearest_indices(image_times_np, action_times_np)
    keep = action_dt <= int(max_sync_dt * 1e9)

    states_np: np.ndarray | None = None
    state_idx: np.ndarray | None = None
    state_keep = np.ones(len(image_times_np), dtype=bool)
    if state_mode != "none":
        if not states:
            LOGGER.warning("%s: no state samples found; skipping episode", bag_path.name)
            return []
        state_times_np = np.asarray(state_times, dtype=np.int64)
        state_order = np.argsort(state_times_np)
        state_times_np = state_times_np[state_order]
        states_np = np.asarray(states, dtype=np.float32)[state_order]
        state_idx, state_dt = nearest_indices(image_times_np, state_times_np)
        state_keep = state_dt <= int(max_sync_dt * 1e9)

    samples: list[Sample] = []
    for i, ok in enumerate(keep & state_keep):
        if not ok:
            continue
        state = None if states_np is None or state_idx is None else states_np[state_idx[i]]
        samples.append(Sample(image_times[i], images[i], actions_np[action_idx[i]], state))
    return samples


def make_features(height: int, width: int, state_mode: str, use_videos: bool) -> dict[str, dict[str, Any]]:
    features: dict[str, dict[str, Any]] = {
        "observation.images.front": {
            "dtype": "video" if use_videos else "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["acceleration", "steering_tire_angle"],
        },
    }
    if state_mode == "odometry":
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (3,),
            "names": ["linear_x", "linear_y", "angular_z"],
        }
    elif state_mode == "vehicle_status":
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (2,),
            "names": ["longitudinal_velocity", "heading_rate"],
        }
    return features


def find_bag_paths(bags_dir: Path) -> list[Path]:
    candidates = [p for p in bags_dir.iterdir() if p.is_dir()]
    if (bags_dir / "metadata.yaml").exists():
        candidates.append(bags_dir)
    return sorted(set(candidates))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags-dir", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, default=Path("./dataset/aichallenge_act"))
    parser.add_argument("--repo-id", default="local/aichallenge_act")
    parser.add_argument("--fps", type=int, default=40)
    parser.add_argument("--image-topic", default="/sensing/camera/image_raw")
    parser.add_argument("--control-topic", default="/control/command/control_cmd")
    parser.add_argument("--state-mode", choices=["none", "odometry", "vehicle_status"], default="none")
    parser.add_argument("--state-topic", default="")
    parser.add_argument("--image-height", type=int, default=200)
    parser.add_argument("--image-width", type=int, default=320)
    parser.add_argument("--crop-top-ratio", type=float, default=0.375)
    parser.add_argument("--max-sync-dt", type=float, default=0.05)
    parser.add_argument("--task", default="drive the racing kart around the course")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--use-videos", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = parse_args()

    if args.outdir.exists():
        if not args.overwrite:
            raise SystemExit(f"{args.outdir} already exists. Pass --overwrite to replace it.")
        shutil.rmtree(args.outdir)

    state_topic = args.state_topic
    if args.state_mode == "odometry" and not state_topic:
        state_topic = "/localization/kinematic_state"
    elif args.state_mode == "vehicle_status" and not state_topic:
        state_topic = "/vehicle/status/velocity_status"
    elif args.state_mode == "none":
        state_topic = None

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=make_features(args.image_height, args.image_width, args.state_mode, args.use_videos),
        root=args.outdir,
        robot_type="aichallenge_racing_kart",
        use_videos=args.use_videos,
        image_writer_threads=4,
    )

    total_samples = 0
    for bag_path in find_bag_paths(args.bags_dir):
        samples = read_bag_episode(
            bag_path,
            image_topic=args.image_topic,
            control_topic=args.control_topic,
            state_topic=state_topic,
            state_mode=args.state_mode,
            image_height=args.image_height,
            image_width=args.image_width,
            crop_top_ratio=args.crop_top_ratio,
            max_sync_dt=args.max_sync_dt,
        )
        if not samples:
            LOGGER.warning("%s: no synchronized samples; skipped", bag_path)
            continue

        first_ts = samples[0].timestamp_ns
        for sample in samples:
            frame = {
                "observation.images.front": sample.image,
                "action": sample.action,
                "timestamp": (sample.timestamp_ns - first_ts) * 1e-9,
                "task": args.task,
            }
            if sample.state is not None:
                frame["observation.state"] = sample.state
            dataset.add_frame(frame)
        dataset.save_episode()
        total_samples += len(samples)
        LOGGER.info("%s: saved %d frames", bag_path.name, len(samples))

    dataset.finalize()
    LOGGER.info("Wrote %d frames to %s", total_samples, args.outdir)


if __name__ == "__main__":
    main()
