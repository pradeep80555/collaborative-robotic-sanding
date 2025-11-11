#!/usr/bin/env python
"""Simple demo - move robot through joint space without IK"""

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

# Define a simple joint space trajectory
home = config.home_qpos()

# Create some waypoints by varying joints slightly
waypoints = [
    home,
    home + np.array([0.0, 0.2, -0.2, 0.0, 0.0, 0.0]),
    home + np.array([0.3, 0.2, -0.2, 0.0, 0.0, 0.0]),
    home + np.array([0.3, 0.4, -0.4, 0.0, 0.0, 0.0]),
    home + np.array([0.0, 0.4, -0.4, 0.0, 0.0, 0.0]),
    home + np.array([-0.3, 0.4, -0.4, 0.0, 0.0, 0.0]),
    home + np.array([-0.3, 0.2, -0.2, 0.0, 0.0, 0.0]),
    home,
]

print(f"Running simple joint-space demo with {len(waypoints)} waypoints")
print("Home position:", home)

# Reset to home
data.qpos[controller.qpos_indices] = home
mujoco.mj_forward(model, data)

# Get sander tip position at home
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")
tip_pos = data.site_xpos[site_id].copy()
print(f"Sander tip at home: {tip_pos}")

steps_per_waypoint = 50

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("\nExecuting trajectory - watch the robot move!")
    for idx, target in enumerate(waypoints):
        print(f"Moving to waypoint {idx+1}/{len(waypoints)}")
        for _ in range(steps_per_waypoint):
            controller.set_target(data, target)
            mujoco.mj_step(model, data)
            viewer.sync()
    
    print("\nTrajectory complete! Viewer will stay open - press ESC to exit")
    while viewer.is_running():
        # Hold final position
        controller.set_target(data, waypoints[-1])
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
