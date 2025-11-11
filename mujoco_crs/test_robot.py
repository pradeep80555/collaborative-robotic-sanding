#!/usr/bin/env python
"""Test script to verify robot kinematics and reach"""

import mujoco
import mujoco.viewer
import numpy as np
import time

from mujoco_crs.config import MujocoCRSConfig

config = MujocoCRSConfig()
model = mujoco.MjModel.from_xml_path(config.asset_path())
data = mujoco.MjData(model)

# Set to home position
home_qpos = config.home_qpos()
print(f"Home position: {home_qpos}")

# Find the joint indices
joint_names = config.joint_order
joint_indices = []
for name in joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    adr = model.jnt_qposadr[jid]
    joint_indices.append(adr)

# Set joints to home
for i, idx in enumerate(joint_indices):
    data.qpos[idx] = home_qpos[i]

# Forward kinematics
mujoco.mj_forward(model, data)

# Get sander_tip position
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")
tip_pos = data.site_xpos[site_id].copy()
tip_quat = np.zeros(4)
mujoco.mju_mat2Quat(tip_quat, data.site_xmat[site_id])

print(f"\nSander tip at home position:")
print(f"  Position: {tip_pos}")
print(f"  Quaternion: {tip_quat}")
print(f"  Distance from origin: {np.linalg.norm(tip_pos):.3f}m")

# Panel position
print(f"\nPanel position: (0.6, 0, 0.43)")
print(f"Panel center would be reachable if within ~1.2m from base")

# Launch viewer
print("\nLaunching viewer - robot at home position...")
print("Press ESC to close")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
