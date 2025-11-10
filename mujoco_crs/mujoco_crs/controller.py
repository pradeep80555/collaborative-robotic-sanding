from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np


def actuator_indices(model: mujoco.MjModel, joint_names: Sequence[str]) -> np.ndarray:
    """Return actuator indices that correspond to the given joint names."""
    indices: list[int] = []
    for name in joint_names:
        actuator_name = f"{name}_pos"
        try:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        except mujoco.Error as exc:  # pragma: no cover - defensive
            raise KeyError(f"Actuator '{actuator_name}' not defined in model") from exc
        indices.append(aid)
    return np.array(indices, dtype=int)


def _joint_qpos_count(model: mujoco.MjModel, joint_id: int) -> int:
    jtype = model.jnt_type[joint_id]
    if jtype in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        return 1
    if jtype == mujoco.mjtJoint.mjJNT_BALL:
        return 4
    if jtype == mujoco.mjtJoint.mjJNT_FREE:
        return 7
    raise ValueError(f"Unsupported joint type {jtype} for joint id {joint_id}")


@dataclass
class JointPositionController:
    model: mujoco.MjModel
    joint_names: Sequence[str]

    def __post_init__(self) -> None:
        self._actuator_indices = actuator_indices(self.model, self.joint_names)
        self._qpos_indices = []
        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            adr = self.model.jnt_qposadr[joint_id]
            count = _joint_qpos_count(self.model, joint_id)
            self._qpos_indices.extend(range(adr, adr + count))
        self._qpos_indices = np.array(self._qpos_indices, dtype=int)

    def set_target(self, data: mujoco.MjData, target: np.ndarray) -> None:
        if target.shape[0] != len(self._actuator_indices):
            raise ValueError("Target length mismatch for joint controller")
        data.ctrl[self._actuator_indices] = target

    def qpos_slice(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self._qpos_indices]

    @property
    def qpos_indices(self) -> np.ndarray:
        return self._qpos_indices

