import logging
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import torch


class ActLeRobotCore:
    """Inference wrapper around a trained LeRobot ACT policy."""

    def __init__(
        self,
        policy_path: str,
        dataset_repo_id: str = "",
        dataset_root: str = "",
        device: str = "cuda",
        image_height: int = 200,
        image_width: int = 320,
        crop_top_ratio: float = 0.375,
        crop_bottom_ratio: float = 0.0,
        state_mode: str = "none",
        control_mode: str = "ai",
        acceleration: float = 0.6,
    ):
        if not policy_path:
            raise ValueError("model.policy_path is required")
        if crop_top_ratio + crop_bottom_ratio >= 1.0:
            raise ValueError("crop_top_ratio + crop_bottom_ratio must be < 1.0")

        self.logger = logging.getLogger(__name__)
        self.policy_path = policy_path
        self.dataset_repo_id = dataset_repo_id
        self.dataset_root = dataset_root
        self.device = torch.device(device if device != "cuda" or torch.cuda.is_available() else "cpu")
        self.image_height = image_height
        self.image_width = image_width
        self.crop_top_ratio = crop_top_ratio
        self.crop_bottom_ratio = crop_bottom_ratio
        self.state_mode = state_mode
        self.control_mode = control_mode.lower()
        self.acceleration = acceleration

        self.policy = self._load_policy(policy_path)
        self.policy.to(self.device)
        if hasattr(self.policy, "config"):
            self.policy.config.device = str(self.device)
        self.policy.eval()
        if hasattr(self.policy, "reset"):
            self.policy.reset()

        self.preprocessor, self.postprocessor = self._load_processors()
        self.expected_image_hw = self._expected_image_hw()
        self.expected_state_dim = self._expected_state_dim()

    def process(self, image: np.ndarray, state: Optional[np.ndarray] = None) -> Tuple[float, float]:
        batch = {
            "observation.images.front": self._image_tensor(image),
            "task": ["drive the racing kart around the course"],
        }
        if self.expected_state_dim:
            if state is None:
                state = np.zeros((self.expected_state_dim,), dtype=np.float32)
            batch["observation.state"] = torch.from_numpy(state.astype(np.float32)).unsqueeze(0).to(self.device)

        with torch.no_grad():
            if self.preprocessor is not None:
                batch = self.preprocessor(batch)
            action = self.policy.select_action(batch)
            if self.postprocessor is not None:
                action = self.postprocessor(action)

        action_np = action.detach().cpu().numpy().reshape(-1)
        if action_np.shape[0] < 2:
            raise RuntimeError(f"ACT policy returned action with shape {action_np.shape}; expected at least 2 values")

        accel = float(np.clip(action_np[0], -1.0, 1.0))
        steer = float(np.clip(action_np[1], -1.0, 1.0))
        if self.control_mode != "ai":
            accel = self.acceleration
        return accel, steer

    def _load_policy(self, policy_path: str):
        try:
            from lerobot.policies.act import ACTPolicy
        except ImportError:
            from lerobot.policies.act.modeling_act import ACTPolicy
        return ACTPolicy.from_pretrained(policy_path)

    def _load_processors(self):
        try:
            from lerobot.datasets import LeRobotDatasetMetadata
            from lerobot.policies import make_pre_post_processors
        except ImportError:
            self.logger.warning("LeRobot processors are unavailable; using raw tensors")
            return None, None

        dataset_stats = None
        if self.dataset_repo_id:
            kwargs = {}
            if self.dataset_root:
                kwargs["root"] = Path(self.dataset_root)
            try:
                metadata = LeRobotDatasetMetadata(self.dataset_repo_id, **kwargs)
                dataset_stats = metadata.stats
            except Exception as exc:
                self.logger.warning("Failed to load dataset metadata for stats: %s", exc)

        overrides = {"device_processor": {"device": str(self.device)}}
        pretrained_path = self.policy_path
        for kwargs in (
            {"policy_cfg": self.policy.config, "pretrained_path": pretrained_path, "dataset_stats": dataset_stats, "preprocessor_overrides": overrides},
            {"config": self.policy.config, "pretrained_path": pretrained_path, "dataset_stats": dataset_stats, "preprocessor_overrides": overrides},
            {"dataset_stats": dataset_stats},
        ):
            try:
                return make_pre_post_processors(**kwargs)
            except TypeError:
                continue
        return make_pre_post_processors(self.policy.config, dataset_stats=dataset_stats)

    def _expected_image_hw(self) -> tuple[int, int]:
        feature = getattr(self.policy.config, "input_features", {}).get("observation.images.front")
        shape = getattr(feature, "shape", None) if feature is not None else None
        if shape and len(shape) == 3:
            if shape[0] == 3:
                return int(shape[1]), int(shape[2])
            return int(shape[0]), int(shape[1])
        return self.image_height, self.image_width

    def _expected_state_dim(self) -> int:
        feature = getattr(self.policy.config, "input_features", {}).get("observation.state")
        shape = getattr(feature, "shape", None) if feature is not None else None
        if shape:
            return int(shape[0])
        return 0

    def _image_tensor(self, image: np.ndarray) -> torch.Tensor:
        image = self._preprocess_image(image)
        tensor = torch.from_numpy(image).permute(2, 0, 1).contiguous().float() / 255.0
        return tensor.unsqueeze(0).to(self.device)

    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        if self.crop_top_ratio > 0 or self.crop_bottom_ratio > 0:
            h = image.shape[0]
            top = int(h * self.crop_top_ratio)
            bottom = h - int(h * self.crop_bottom_ratio)
            image = image[top:bottom, :, :]
        height, width = self.expected_image_hw
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
