#!/usr/bin/env python
"""Verify robot and panel positions visually"""

import mujoco
import mujoco.viewer
import numpy as np
import time

from mujoco_crs.config import MujocoCRSConfig
from mujoco_crs.controller import JointPositionController

config = MujocoCRSConfig()
model = mujoco.MjModel.from_xml_path(config.asset_path())
data = mujoco.MjData(model)

controller = JointPositionController(model, config.joint_order)

# Get site IDs
sander_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")
work_surface_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "work_surface")

# Get body ID for workbench
try:
    workbench_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "workbench")
    workpiece_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "workpiece")
except:
    workbench_id = None
    workpiece_id = None

print("="*80)
print("SCENE ANALYSIS")
print("="*80)

# Home position
home = config.home_qpos()
data.qpos[controller.qpos_indices] = home
mujoco.mj_forward(model, data)

sander_pos = data.site_xpos[sander_tip_id].copy()
panel_pos = data.site_xpos[work_surface_id].copy()

print("\nAt HOME position:")
print(f"  Robot joints: {home}")
print(f"  Sander tip position: {sander_pos}")
print(f"  Panel work_surface position: {panel_pos}")
print(f"  Distance: {np.linalg.norm(sander_pos - panel_pos):.3f}m")

if workbench_id is not None:
    wb_pos = data.xpos[workbench_id].copy()
    print(f"  Workbench body position: {wb_pos}")

if workpiece_id is not None:
    wp_pos = data.xpos[workpiece_id].copy()
    print(f"  Workpiece body position: {wp_pos}")

# Best sanding config
best_config = np.array([-1.520, -1.012, -2.114, -0.606, 2.541, -1.731])
data.qpos[controller.qpos_indices] = best_config
mujoco.mj_forward(model, data)

sander_pos = data.site_xpos[sander_tip_id].copy()
distance = np.linalg.norm(sander_pos - panel_pos)

print("\nAt BEST sanding position:")
print(f"  Robot joints: {best_config}")
print(f"  Sander tip position: {sander_pos}")
print(f"  Panel work_surface position: {panel_pos}")
print(f"  Distance: {distance:.3f}m ({distance*100:.1f}cm)")

print("\n" + "="*80)
print("VISUAL TEST - Watch the robot move between positions")
print("="*80)
print("The robot will:")
print("1. Start at HOME (away from panel)")
print("2. Move to SANDING position (should be near panel)")
print("3. Hold there so you can verify visually")
print("\nPress ESC when done checking")
print("="*80)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Hold at home
    print("\n[1/3] Showing HOME position for 3 seconds...")
    for _ in range(1500):
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Move to sanding position
    print("[2/3] Moving to SANDING position...")
    for _ in range(100):
        controller.set_target(data, best_config)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Hold at sanding position
    print("[3/3] Holding at SANDING position - VERIFY: Is the sander near the panel?")
    print("      The sander tip should be within 5cm of the panel surface!")
    
    while viewer.is_running():
        controller.set_target(data, best_config)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
