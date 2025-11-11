#!/usr/bin/env python
"""Find a joint configuration that actually reaches the panel"""

import mujoco
import numpy as np

from mujoco_crs.config import MujocoCRSConfig

config = MujocoCRSConfig()
model = mujoco.MjModel.from_xml_path(config.asset_path())
data = mujoco.MjData(model)

# Get IDs
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sander_tip")
work_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "work_surface")

# Joint indices
joint_indices = []
for name in config.joint_order:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    adr = model.jnt_qposadr[jid]
    joint_indices.append(adr)

print("Searching for configurations that reach the panel...")
print(f"Panel work_surface site is our target\n")

# Get panel position
mujoco.mj_forward(model, data)
panel_pos = data.site_xpos[work_site_id].copy()
print(f"Panel position: {panel_pos}")

best_configs = []

# Try many random configurations
np.random.seed(42)
for i in range(10000):
    # Random joint angles
    q = np.random.uniform(-3.14, 3.14, 6)
    
    for idx, val in zip(joint_indices, q):
        data.qpos[idx] = val
    
    mujoco.mj_forward(model, data)
    tip_pos = data.site_xpos[site_id].copy()
    
    # Distance to panel
    dist = np.linalg.norm(tip_pos - panel_pos)
    
    if dist < 0.3:  # Within 30cm
        best_configs.append((dist, q, tip_pos))

# Sort by distance
best_configs.sort(key=lambda x: x[0])

print(f"\nFound {len(best_configs)} configurations within 30cm of panel")
print("\nTop 10 closest configurations:")
print("-" * 80)

for i, (dist, q, pos) in enumerate(best_configs[:10]):
    print(f"{i+1}. Distance: {dist*100:.1f}cm")
    print(f"   Joints: [{', '.join(f'{x:6.3f}' for x in q)}]")
    print(f"   Tip pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    print()

if best_configs:
    print("Best configuration found:")
    dist, q, pos = best_configs[0]
    print(f"  config = np.array([{', '.join(f'{x:.4f}' for x in q)}])")
