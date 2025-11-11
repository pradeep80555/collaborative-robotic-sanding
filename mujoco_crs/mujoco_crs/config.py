from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml

from . import get_asset_path


DEFAULT_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@dataclass
class MujocoCRSConfig:
    model_xml: str = "ur10e_sanding.xml"
    joint_order: Sequence[str] = field(default_factory=lambda: DEFAULT_JOINT_ORDER)
    home_position: Sequence[float] = field(
        default_factory=lambda: (0.0, -2.5, 1.7, -0.77, -1.57, 0.0)
    )
    waypoint_hold_time: float = 0.15
    interpolation_density: float = 0.04  # meters
    ik_position_tolerance: float = 5e-3
    ik_rotation_tolerance: float = 5e-2
    ik_damping: float = 0.1
    ik_max_iterations: int = 200

    def asset_path(self) -> str:
        return str(get_asset_path(self.model_xml))

    def home_qpos(self) -> np.ndarray:
        # Home position: arm extended forward toward panel
        if hasattr(self, "home_position") and self.home_position is not None and len(self.home_position) == 6:
            return np.asarray(self.home_position, dtype=float)
        # Default: pointing forward and up
        return np.array([0.0, -2.5, 1.7, -0.77, -1.57, 0.0], dtype=float)


def load_crs_yaml(path: Path | str, config: MujocoCRSConfig | None = None) -> MujocoCRSConfig:
    """Merge relevant parameters from a legacy CRS YAML config into MujocoCRSConfig."""
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        params = yaml.safe_load(handle)

    cfg = MujocoCRSConfig() if config is None else config
    crs = params.get("crs", {})
    motion = crs.get("motion_planning", {})
    home = motion.get("home_position", {})
    joint_names = home.get("joint_names")
    joint_position = home.get("joint_position")
    if joint_names and joint_position and len(joint_names) == len(joint_position):
        cfg.joint_order = tuple(joint_names)
        cfg.home_position = tuple(joint_position)
    return cfg

