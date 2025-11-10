from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np


def _joint_qpos_count(model: mujoco.MjModel, joint_id: int) -> int:
    jtype = model.jnt_type[joint_id]
    if jtype in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        return 1
    if jtype == mujoco.mjtJoint.mjJNT_BALL:
        return 4
    if jtype == mujoco.mjtJoint.mjJNT_FREE:
        return 7
    raise ValueError(f"Unsupported joint type {jtype} for joint id {joint_id}")


def _joint_indices(model: mujoco.MjModel, joint_names: Sequence[str]) -> np.ndarray:
    indices: list[int] = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        adr = model.jnt_qposadr[joint_id]
        count = _joint_qpos_count(model, joint_id)
        indices.extend(range(adr, adr + count))
    return np.array(indices, dtype=int)


def _quat_conjugate(quat: np.ndarray) -> np.ndarray:
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


@dataclass
class IKSolver:
    model: mujoco.MjModel
    site: str
    joint_names: Sequence[str]
    damping: float = 1e-4
    position_tolerance: float = 5e-4
    rotation_tolerance: float = 1e-3
    max_iterations: int = 200

    def __post_init__(self) -> None:
        self._site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site)
        self._joint_indices = _joint_indices(self.model, self.joint_names)
        self._scratch_jac_pos = np.zeros((3, self.model.nv))
        self._scratch_jac_rot = np.zeros((3, self.model.nv))
        self._scratch_error = np.zeros(6)

    def solve(
        self,
        target_position: np.ndarray,
        target_quaternion: np.ndarray,
        *,
        initial_qpos: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute a joint configuration that reaches the provided pose."""
        data = mujoco.MjData(self.model)
        if initial_qpos is not None:
            data.qpos[:] = initial_qpos
        mujoco.mj_forward(self.model, data)

        for _ in range(self.max_iterations):
            current_pos = data.site_xpos[self._site_id]
            current_quat = np.zeros(4)
            mujoco.mju_mat2Quat(current_quat, data.site_xmat[self._site_id])

            pos_err = target_position - current_pos
            quat_err = _quat_multiply(target_quaternion, _quat_conjugate(current_quat))
            quat_err /= np.linalg.norm(quat_err)
            rot_vec = np.zeros(3)
            mujoco.mju_quat2Vel(rot_vec, quat_err, 1.0)

            if np.linalg.norm(pos_err) < self.position_tolerance and np.linalg.norm(rot_vec) < self.rotation_tolerance:
                result = data.qpos.copy()
                return result

            mujoco.mj_jacSite(self.model, data, self._scratch_jac_pos, self._scratch_jac_rot, self._site_id)
            J = np.vstack((self._scratch_jac_pos[:, self._joint_indices], self._scratch_jac_rot[:, self._joint_indices]))

            error = np.hstack((pos_err, rot_vec))
            H = J.T @ J + self.damping * np.eye(J.shape[1])
            g = J.T @ error
            step = np.linalg.solve(H, g)
            data.qpos[self._joint_indices] += step
            mujoco.mj_forward(self.model, data)

        raise RuntimeError("IK failed to converge within max iterations")

