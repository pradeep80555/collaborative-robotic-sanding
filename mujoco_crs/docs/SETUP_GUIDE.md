# Mujoco CRS Simulation Setup

The legacy collaborative robotic sanding stack depended on ROS2 Eloquent, Gazebo, and a UR robot driver.  This reimplementation removes all ROS/hardware requirements and runs entirely in Python on top of Mujoco.

## 1. Prepare macOS tooling (run once)

> Skip any item you already have installed. You mentioned the Mujoco app is on your machine; still run the `pip install mujoco` command below so Python bindings match the version on disk.

1. **Install Xcode command-line tools** – provides compilers, `git`, and headers needed by Python wheels.
   ```bash
   xcode-select --install
   ```

2. **Install Homebrew (optional but convenient)** – gives you `brew` for optional utilities like `ffmpeg`.
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

3. **Install/upgrade the Mujoco Python package** – pulls the latest wheels that bundle the native libraries used by the simulator.
   ```bash
   python3 -m pip install --upgrade mujoco
   ```
   - If you rely on the standalone Mujoco viewer apps, keep your existing installation under `~/Library/Application Support/MuJoCo` or update it from [mujoco.org](https://mujoco.org/). The Python package happily coexists with the app.

4. **(Optional) Install `ffmpeg`** – only needed if you plan to capture simulation videos.
   ```bash
   brew install ffmpeg
   ```

5. **Verify Python version** – the project targets Python 3.10+. Confirm with:
   ```bash
   python3 --version
   ```

## 2. Create and populate a virtual environment

Run the following inside the repository; each command is annotated with its purpose.

```bash
cd /Users/pradeep/Downloads/Algorthimic\ HRI/Final_project_code/collaborative-robotic-sanding/mujoco_crs  # enter the Mujoco project directory
python3 -m venv .venv                                                                   # create an isolated Python environment
source .venv/bin/activate                                                               # activate the venv so installs stay local
pip install --upgrade pip wheel                                                         # ensure packaging tools are current
pip install -e .                                                                        # install the mujoco_crs package in editable mode
```

The final command exposes the `mujoco-crs-demo` CLI and links the package to the source tree for rapid iteration.

### Upgrading later

```bash
source .venv/bin/activate                                          # re-activate the existing venv
pip install --upgrade mujoco numpy scipy tyro                      # refresh runtime dependencies
pip install --upgrade -e .                                         # pull in any local source changes
```

## 3. Run the default simulation

Launch the Mujoco viewer and follow the built-in sanding path:

- **Interactive viewer on macOS** – MuJoCo requires you to use its bundled Python interpreter (`mjpython`):
  ```bash
  /Applications/MuJoCo.app/Contents/MacOS/mjpython -m mujoco_crs.run --viewer
  ```
- **Linux/Windows interactive** – the packaged console script works as-is:
  ```bash
  mujoco-crs-demo --viewer
  ```
- **Headless playback (all platforms)** – omit `--viewer` to run without GUI:
  ```bash
  mujoco-crs-demo
  ```

Viewer controls:

- **Space** – pause/resume simulation
- **Right drag** – orbit camera
- **Middle drag** – pan
- **Mouse wheel** – zoom
- **Esc** – close viewer

## 4. Replaying Legacy CRS Toolpaths

Reuse any of the CRS YAML toolpaths directly:

```bash

# macOS interactive
/Applications/MuJoCo.app/Contents/MacOS/mjpython -m mujoco_crs.run \
  --toolpath /Users/pradeep/Downloads/Algorthimic\ HRI/Final_project_code/collaborative-robotic-sanding/crs_process_data/data/main/toolpaths/part_2/job_0_degrees_20200520T192241.997282.yaml \
  --part-config /Users/pradeep/Downloads/Algorthimic\ HRI/Final_project_code/collaborative-robotic-sanding/crs_process_data/data/main/toolpaths/part_2/crs.yaml \
  --viewer

# Linux/Windows interactive (or macOS headless)
mujoco-crs-demo \
  --toolpath /Users/pradeep/Downloads/Algorthimic\ HRI/Final_project_code/collaborative-robotic-sanding/crs_process_data/data/main/toolpaths/part_2/job_0_degrees_20200520T192241.997282.yaml \
  --part-config /Users/pradeep/Downloads/Algorthimic\ HRI/Final_project_code/collaborative-robotic-sanding/crs_process_data/data/main/toolpaths/part_2/crs.yaml \
  --viewer
```

Tips:

- The CRS YAML provides the original home joint configuration; the loader merges it automatically so the Mujoco robot matches the intended start pose.
- Toolpaths are densified before execution (`--interpolation` CLI flag) to keep adjacent positions within a 1.5&nbsp;cm bound.  Increase the density for smoother motion.
- Use `--waypoint-hold 0.02` to speed up execution while keeping stability.

## 5. Headless / Batch Usage

The simulator can run headless (without viewer) which is useful for automated testing:

```bash
mujoco-crs-demo --toolpath ./example.yaml
```

Wrap the CLI in Python if you need programmatic control:

```python
from mujoco_crs.run import Args, main

main(Args(toolpath=Path("./example.yaml"), viewer=False))
```

## 6. Creating Your Own Toolpaths

Two options:

1. **Programmatic generation** via the helper in `toolpath.py`:
   ```python
   from mujoco_crs.toolpath import create_rectangular_toolpath
   toolpath = create_rectangular_toolpath(origin=(0.5, -0.35, 0.75), size=(0.4, 0.25))
   ```

2. **Legacy YAML** – follow the schema in the `crs_process_data` package.  Each file contains a list of `paths`, each with a list of `poses`.

Save your tooling data anywhere and pass the absolute path to the CLI.

## 7. Troubleshooting

- **IK did not converge** – try lowering `--interpolation` to add more intermediate samples, or reduce `--damping` so the solver takes larger steps.
- **Viewer crashes on launch** – ensure `$MUJOCO_GL=egl` is unset on macOS; the default Metal backend works out-of-the-box.
- **Robot oscillates** – the actuators are configured as position servos.  Shorten `--waypoint-hold` or increase the damping coefficient in `run.py`.

## 8. Next Steps

- Integrate contact-sensitive sanding by reading `data.cfrc_ext` and modulating normal force.
- Add perception mock-ups by importing the part meshes from `crs_support/meshes`.
- Wrap the simulator in a reinforcement-learning environment (Gymnasium) for policy training.

With the ROS2/Gazebo dependencies removed, iterating on collaborative sanding strategies should now be much quicker and more portable.

