#!/usr/bin/env python
"""Test IK solver to find what positions are reachable"""

import mujoco
import numpy as np

from mujoco_crs.config import MujocoCRSConfig
from mujoco_crs.ik import IKSolver

config = MujocoCRSConfig()
model = mujoco.MjModel.from_xml_path(config.asset_path())
data = mujoco.MjData(model)

# Create IK solver
ik_solver = IKSolver(
    model=model,
    site="sander_tip",
    joint_names=config.joint_order,
    damping=0.1,  # Higher damping
    position_tolerance=5e-3,
    rotation_tolerance=5e-2,
    max_iterations=200,
)

# Set to home position
home_qpos = config.home_qpos()
data.qpos[:len(home_qpos)] = home_qpos
mujoco.mj_forward(model, data)

# Get current tip position and orientation
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")
tip_pos = data.site_xpos[site_id].copy()
tip_quat = np.zeros(4)
mujoco.mju_mat2Quat(tip_quat, data.site_xmat[site_id])

print("Starting from home position:")
print(f"  Tip position: {tip_pos}")
print(f"  Tip quaternion: {tip_quat}")

# Test points near the panel
test_points = [
    # Panel is at (0.6, 0, 0.43)
    (0.5, 0.0, 0.43, "center-near"),
    (0.6, 0.0, 0.43, "panel center"),
    (0.7, 0.0, 0.43, "center-far"),
    (0.6, -0.1, 0.43, "left side"),
    (0.6, 0.1, 0.43, "right side"),
    (0.5, 0.0, 0.5, "higher near"),
]

# Tool pointing down: -90° around Y axis
quat_down = np.array([0.7071068, 0, -0.7071068, 0])  # w,x,y,z

print("\nTesting IK for panel positions (tool pointing down):")
print("-" * 70)

success_count = 0
for x, y, z, label in test_points:
    target_pos = np.array([x, y, z])
    try:
        solution = ik_solver.solve(
            target_pos,
            quat_down,
            initial_qpos=data.qpos.copy(),
        )
        
        # Verify by forward kinematics
        data_test = mujoco.MjData(model)
        data_test.qpos[:] = solution
        mujoco.mj_forward(model, data_test)
        
        achieved_pos = data_test.site_xpos[site_id].copy()
        error = np.linalg.norm(achieved_pos - target_pos)
        
        print(f"✓ {label:15s} ({x:.2f}, {y:.2f}, {z:.2f}) - Error: {error*1000:.1f}mm")
        success_count += 1
    except Exception as e:
        print(f"✗ {label:15s} ({x:.2f}, {y:.2f}, {z:.2f}) - FAILED: {str(e)[:40]}")

print(f"\nSuccess rate: {success_count}/{len(test_points)}")

# Try with different orientations
print("\n" + "="*70)
print("Testing different tool orientations at panel center (0.6, 0, 0.43):")
print("-" * 70)

orientations = [
    (np.array([1, 0, 0, 0]), "identity (no rotation)"),
    (np.array([0.7071068, 0, -0.7071068, 0]), "pointing down (-90° Y)"),
    (np.array([0.9238795, 0, -0.3826834, 0]), "angled down (-45° Y)"),
    (np.array([0.9659258, 0, -0.258819, 0]), "slight down (-30° Y)"),
]

target_pos = np.array([0.6, 0.0, 0.43])
for quat, desc in orientations:
    try:
        solution = ik_solver.solve(target_pos, quat, initial_qpos=home_qpos.copy())
        print(f"✓ {desc:30s} - SUCCESS")
    except Exception as e:
        print(f"✗ {desc:30s} - FAILED")

print("\nDone!")
