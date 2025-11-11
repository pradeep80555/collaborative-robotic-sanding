#!/usr/bin/env python
"""Create a sanding demo using forward kinematics to ensure reachability"""

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

# Get site ID
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")

# Home position
home = config.home_qpos()

# Create a lawnmower pattern by varying joints slightly around a good base config
# Panel is at (0.6, 0, 0.45) - this config reaches (0.627, -0.034, 0.454)
print("Generating sanding trajectory over panel...")

# Best configuration found that reaches the panel (4.3cm away)
base_config = np.array([-1.520, -1.012, -2.114, -0.606, 2.541, -1.731])

# Generate waypoints by scanning across
waypoints = []
positions = []

# Scan pattern: vary joints slightly to cover panel area
num_rows = 6
num_cols = 10

for row in range(num_rows):
    # Alternate direction each row (lawnmower pattern)
    cols = range(num_cols) if row % 2 == 0 else range(num_cols-1, -1, -1)
    
    for col in cols:
        # Vary shoulder_pan and wrist joints for coverage
        pan_offset = (col / (num_cols-1) - 0.5) * 0.3  # ±0.15 rad on joint 0
        row_offset = (row / (num_rows-1) - 0.5) * 0.3  # ±0.15 rad on joint 1
        
        joint_config = base_config.copy()
        joint_config[0] += pan_offset  # shoulder_pan
        joint_config[1] += row_offset  # shoulder_lift
        
        # Compute forward kinematics to see where this reaches
        data.qpos[controller.qpos_indices] = joint_config
        mujoco.mj_forward(model, data)
        
        tip_pos = data.site_xpos[site_id].copy()
        waypoints.append(joint_config)
        positions.append(tip_pos)

print(f"Generated {len(waypoints)} waypoints")
print(f"Position range:")
print(f"  X: [{min(p[0] for p in positions):.3f}, {max(p[0] for p in positions):.3f}]")
print(f"  Y: [{min(p[1] for p in positions):.3f}, {max(p[1] for p in positions):.3f}]")
print(f"  Z: [{min(p[2] for p in positions):.3f}, {max(p[2] for p in positions):.3f}]")
print(f"Panel center is at: (0.6, 0, 0.43)")

# Reset to home
data.qpos[controller.qpos_indices] = home
mujoco.mj_forward(model, data)

steps_per_waypoint = 40

print("\nLaunching sanding demonstration...")
print("Watch the robot perform a sanding motion over the panel!")
print("Press ESC to exit")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Start from home
    for _ in range(50):
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Execute sanding pattern
    for idx, target in enumerate(waypoints):
        if idx % 10 == 0:
            print(f"Sanding... {idx}/{len(waypoints)}")
        
        for _ in range(steps_per_waypoint):
            controller.set_target(data, target)
            mujoco.mj_step(model, data)
            viewer.sync()
    
    # Return to home
    for _ in range(50):
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
    
    print("\nSanding complete! Viewer will stay open - press ESC to exit")
    while viewer.is_running():
        controller.set_target(data, home)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
