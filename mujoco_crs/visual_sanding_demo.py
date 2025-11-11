#!/usr/bin/env python
"""CLEAR VISUAL SANDING DEMO - Slow and obvious motion"""

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
sander_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")

home = config.home_qpos()
base_config = np.array([-1.520, -1.012, -2.114, -0.606, 2.541, -1.731])

print("=" * 80)
print("VISUAL SANDING DEMONSTRATION")
print("=" * 80)
print("\nLOOK FOR:")
print("  🟡 YELLOW SPHERE = Sander Tip (robot end effector)")
print("  🔴 RED SPHERE = Panel Center (sanding target)")
print("  🟢 GREEN SPHERES = Panel Corners")
print("\nThe yellow sphere should move VERY CLOSE to the red sphere!")
print("\nThe robot will:")
print("  1. Start away from panel (3 seconds)")
print("  2. Move slowly to panel (5 seconds)")  
print("  3. Perform SLOW sanding motions over panel (20 seconds)")
print("  4. Return home (5 seconds)")
print("\nWatch carefully - the motion is SLOW so you can see it clearly!")
print("=" * 80)
input("\nPress ENTER to start the demonstration...")

# Generate a simple 4x3 grid for slow, visible motion
waypoints = []
for row in range(3):
    cols = range(4) if row % 2 == 0 else range(3, -1, -1)
    for col in cols:
        pan_offset = (col / 3 - 0.5) * 0.25
        row_offset = (row / 2 - 0.5) * 0.25
        
        config_point = base_config.copy()
        config_point[0] += pan_offset
        config_point[1] += row_offset
        waypoints.append(config_point)

print(f"\nGenerated {len(waypoints)} sanding waypoints")

# Slow motion parameters
steps_per_waypoint = 150  # Much slower for visibility

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("\n[1/4] Starting at HOME position (away from panel)...")
    print("      Look at the scene - find the YELLOW and RED spheres")
    for _ in range(1500):
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    print("\n[2/4] Moving SLOWLY to panel...")
    print("      Watch the YELLOW sphere approach the RED sphere!")
    # Slow transition to first waypoint
    for alpha in np.linspace(0, 1, 2500):
        interp_config = home * (1 - alpha) + waypoints[0] * alpha
        controller.set_target(data, interp_config)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    print("\n[3/4] SANDING THE PANEL - Watch the yellow sphere move over the red sphere area!")
    print(f"      Performing {len(waypoints)} passes...")
    for idx, target in enumerate(waypoints):
        if idx % 3 == 0:
            print(f"      ... pass {idx+1}/{len(waypoints)}")
        
        for _ in range(steps_per_waypoint):
            controller.set_target(data, target)
            mujoco.mj_step(model, data)
            viewer.sync()
    
    print("\n[4/4] Returning to HOME position...")
    # Slow transition back
    for alpha in np.linspace(0, 1, 2500):
        interp_config = waypoints[-1] * (1 - alpha) + home * alpha
        controller.set_target(data, interp_config)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    print("\n" + "=" * 80)
    print("DEMONSTRATION COMPLETE!")
    print("=" * 80)
    print("\nDid you see the YELLOW sphere (sander) move over the RED sphere (panel)?")
    print("If not, the panel and robot might be in different areas of the scene.")
    print("\nViewer will stay open - press ESC to exit")
    print("=" * 80)
    
    while viewer.is_running():
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
