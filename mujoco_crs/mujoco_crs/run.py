from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import platform
import sys

import tyro

from .config import MujocoCRSConfig, load_crs_yaml
from .simulation import SimulationApp
from .toolpath import Toolpath, create_rectangular_toolpath, load_toolpath


@dataclass
class Args:
    toolpath: Optional[Path] = None
    part_config: Optional[Path] = None
    viewer: bool = False
    interpolation: float = 0.015
    waypoint_hold: float = 0.04
    damping: float = 1e-3


def build_toolpath(args: Args) -> Toolpath:
    if args.toolpath is not None:
        return load_toolpath(args.toolpath)
    return create_rectangular_toolpath()


def build_config(args: Args) -> MujocoCRSConfig:
    cfg = MujocoCRSConfig()
    cfg.interpolation_density = args.interpolation
    cfg.waypoint_hold_time = args.waypoint_hold
    cfg.ik_damping = args.damping
    if args.part_config is not None:
        cfg = load_crs_yaml(args.part_config, cfg)
    return cfg


def main(cli_args: Args | None = None) -> None:
    args = cli_args or tyro.cli(Args)
    config = build_config(args)
    toolpath = build_toolpath(args)

    app = SimulationApp(config)
    try:
        app.run_toolpath(toolpath, launch_viewer=args.viewer)
    except RuntimeError as exc:
        if args.viewer and platform.system() == "Darwin" and "launch_passive" in str(exc):
            hint = (
                "MuJoCo's macOS viewer needs the MuJoCo-provided `mjpython` interpreter.\n"
                "Launch the demo with the interpreter that ships alongside MuJoCo, e.g.:\n"
                "  /Applications/mujoco-<version>/bin/mjpython -m mujoco_crs.run --viewer\n"
                "or run headless without --viewer."
            )
            raise SystemExit(hint) from exc
        raise


if __name__ == "__main__":
    main()

