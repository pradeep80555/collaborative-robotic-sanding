from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class CartesianWaypoint:
    position: np.ndarray  # shape (3,)
    quaternion: np.ndarray  # shape (4,) wxyz order (Mujoco convention)

    @staticmethod
    def from_dict(entry: dict) -> "CartesianWaypoint":
        pos = entry["position"]
        ori = entry["orientation"]
        position = np.array([pos["x"], pos["y"], pos["z"]], dtype=float)
        quaternion = np.array([ori["w"], ori["x"], ori["y"], ori["z"]], dtype=float)
        return CartesianWaypoint(position=position, quaternion=quaternion)


@dataclass
class Toolpath:
    name: str
    waypoints: List[CartesianWaypoint]

    def __iter__(self) -> Iterable[CartesianWaypoint]:
        return iter(self.waypoints)

    def __len__(self) -> int:
        return len(self.waypoints)

    def positions(self) -> np.ndarray:
        return np.stack([wp.position for wp in self.waypoints])

    def quaternions(self) -> np.ndarray:
        return np.stack([wp.quaternion for wp in self.waypoints])


def load_toolpath(path: Path | str, *, name: str | None = None) -> Toolpath:
    """Load a legacy CRS toolpath (YAML) and flatten the embedded path list."""
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        blob = yaml.safe_load(handle)

    if not isinstance(blob, list):
        raise ValueError(f"Unrecognised toolpath schema in {path}")

    waypoints: List[CartesianWaypoint] = []
    for segment in blob:
        poses = segment.get("poses", [])
        for pose_entry in poses:
            waypoints.append(CartesianWaypoint.from_dict(pose_entry))

    if not waypoints:
        raise ValueError(f"No poses found inside toolpath file {path}")

    resolved_name = name or path.stem
    return Toolpath(name=resolved_name, waypoints=waypoints)


def resample_toolpath(
    toolpath: Toolpath,
    max_translation_step: float = 0.02,
) -> Toolpath:
    """Insert additional cartesian samples to keep translation steps bounded."""
    if len(toolpath) < 2:
        return toolpath

    new_points: List[CartesianWaypoint] = []

    def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        dot = np.dot(q0, q1)
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        dot = np.clip(dot, -1.0, 1.0)
        if dot > 0.9995:
            return (q0 + t * (q1 - q0)) / np.linalg.norm(q0 + t * (q1 - q0))
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.sin(theta_0 - theta) / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return (s0 * q0 + s1 * q1) / np.linalg.norm(s0 * q0 + s1 * q1)

    for a, b in zip(toolpath.waypoints[:-1], toolpath.waypoints[1:]):
        delta = b.position - a.position
        distance = np.linalg.norm(delta)
        steps = max(1, int(np.ceil(distance / max_translation_step)))
        for i in range(steps):
            t = i / steps
            interp_pos = a.position + t * delta
            interp_quat = slerp(a.quaternion, b.quaternion, t)
            new_points.append(CartesianWaypoint(interp_pos, interp_quat))
    new_points.append(toolpath.waypoints[-1])
    return Toolpath(name=toolpath.name, waypoints=new_points)


def create_rectangular_toolpath(
    origin: Sequence[float] = (0.4, -0.4, 0.75),
    size: Sequence[float] = (0.5, 0.3),
    spacing: float = 0.05,
    normal_quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
) -> Toolpath:
    """Generate a simple lawn-mower toolpath over a rectangular panel.

    The orientation quaternion is expressed in Mujoco's (w, x, y, z) convention.
    """
    # Panel is at (0.6, 0, 0.43) - generate reachable toolpath
    # Robot tip at home is around (0.08, 0.78, 0.10)
    # Use identity orientation since that's what IK can solve
    center = np.array([0.6, 0.0, 0.43])  # panel center
    width = 0.3   # panel width
    height = 0.2  # panel height
    
    # Tool orientation: identity (natural wrist orientation)
    quat = np.array([1, 0, 0, 0])  # w,x,y,z
    
    num_x = 10
    num_y = 6
    waypoints = []
    
    for i in range(num_y):
        y = center[1] - height / 2 + i * (height / (num_y - 1)) if num_y > 1 else center[1]
        row = []
        for j in range(num_x):
            x = center[0] - width / 2 + j * (width / (num_x - 1)) if num_x > 1 else center[0]
            pos = np.array([x, y, center[2]])
            row.append(CartesianWaypoint(position=pos, quaternion=quat))
        if i % 2 == 1:
            row.reverse()
        waypoints.extend(row)
    
    return Toolpath(name="rectangular_panel", waypoints=waypoints)

