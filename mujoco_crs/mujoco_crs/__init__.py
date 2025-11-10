"""
Mujoco-based reimplementation of the Collaborative Robotic Sanding (CRS) demo.

All functionality is intentionally dependency-light and ROS-free so the simulator
can run on modern macOS (Apple Silicon) with Python tooling only.
"""

from importlib import resources
from pathlib import Path


def get_asset_path(relative: str) -> Path:
    """Return an absolute path to a packaged asset file."""
    return Path(resources.files("mujoco_crs") / "assets" / relative)


__all__ = ["get_asset_path"]

