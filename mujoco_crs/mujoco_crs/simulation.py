from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
from typing import Iterable
import time

import mujoco
import mujoco.viewer
import numpy as np

from .config import MujocoCRSConfig
from .controller import JointPositionController
from .ik import IKSolver
from .toolpath import Toolpath, resample_toolpath


def _normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero magnitude")
    return q / norm


@dataclass
class SimulationApp:
    config: MujocoCRSConfig

    def __post_init__(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(self.config.asset_path())
        self.data = mujoco.MjData(self.model)
        self.controller = JointPositionController(self.model, self.config.joint_order)
        self.ik_solver = IKSolver(
            model=self.model,
            site="sander_tip",
            joint_names=self.config.joint_order,
            damping=self.config.ik_damping,
            position_tolerance=self.config.ik_position_tolerance,
            rotation_tolerance=self.config.ik_rotation_tolerance,
            max_iterations=self.config.ik_max_iterations,
        )
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self._set_joint_positions(self.config.home_qpos())
        mujoco.mj_forward(self.model, self.data)

    def _set_joint_positions(self, qpos: Iterable[float]) -> None:
        qpos = np.asarray(list(qpos), dtype=float)
        self.data.qpos[self.controller.qpos_indices] = qpos

    def _plan_joint_path(self, toolpath: Toolpath) -> np.ndarray:
        seed = self.data.qpos.copy()
        trajectory: list[np.ndarray] = []
        print(f"Planning joint path for {len(toolpath)} waypoints...")
        for idx, waypoint in enumerate(toolpath):
            pose = _normalize_quaternion(waypoint.quaternion)
            try:
                solution = self.ik_solver.solve(
                    waypoint.position,
                    pose,
                    initial_qpos=seed,
                )
                trajectory.append(solution[self.controller.qpos_indices])
                seed = solution
            except Exception as e:
                print(f"IK failed at waypoint {idx}: {e}")
        print(f"Planned {len(trajectory)} joint positions.")
        if not trajectory:
            print("No valid joint trajectory was generated. Check home position and toolpath reachability.")
        return np.vstack(trajectory) if trajectory else np.zeros((1, len(seed)))

    def run_toolpath(self, toolpath: Toolpath, *, launch_viewer: bool = False) -> None:
        self.reset()
        dense_path = resample_toolpath(toolpath, self.config.interpolation_density)
        joint_traj = self._plan_joint_path(dense_path)
        self._execute_joint_trajectory(joint_traj, launch_viewer=launch_viewer)

    def _execute_joint_trajectory(self, joint_traj: np.ndarray, *, launch_viewer: bool) -> None:
        steps_per_waypoint = max(1, int(self.config.waypoint_hold_time / self.model.opt.timestep))

        viewer_context = (
            mujoco.viewer.launch_passive(self.model, self.data) if launch_viewer else nullcontext()
        )

        print(f"Executing trajectory with {len(joint_traj)} waypoints, {steps_per_waypoint} steps each...")
        with viewer_context as viewer:
            for idx, target in enumerate(joint_traj):
                if idx % 10 == 0:
                    print(f"Executing waypoint {idx}/{len(joint_traj)}")
                for step in range(steps_per_waypoint):
                    self.controller.set_target(self.data, target)
                    mujoco.mj_step(self.model, self.data)
                    if viewer is not None:
                        viewer.sync()
            print("Trajectory execution complete!")
            if viewer is not None:
                print("Viewer running - press ESC to exit...")
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.01)

