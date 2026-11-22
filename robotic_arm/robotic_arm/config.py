"""Tunable settings. Load overrides from config.yaml; every value has a default."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path

import yaml


@dataclass
class Config:
    # Controller (soarm-panel). Tailscale address of ramion-server-01, or an SSH tunnel to localhost.
    controller_url: str = "http://100.85.56.14:8780"  # the server dashboard (same /api routes, keep-alive, faster); the controller itself is :8765
    calibration_file: str = "calibration.json"

    # Heights above the calibrated table-tip height [m].
    hover_height: float = 0.07  # travel height; must clear a block (and a block held in the gripper)
    grasp_offset: float = 0.012  # tip height above the table when grasping: about half a block. TUNE.
    release_offset: float = 0.018  # tip height when placing: slightly above grasp so the block drops softly

    # Simple goto move: rise to travel_height above the table, glide, then lower to goto_offset.
    travel_height: float = 0.03
    goto_offset: float = 0.005  # final tip height above the calibrated table height. TUNE (0 = touch)

    # Gripper in app units (0 closed .. 100 open). TUNE both on the real arm with a real block.
    gripper_open: float = 60.0
    gripper_closed: float = 8.0  # a bit below the block width so the jaws squeeze it

    # Motion. The controller caps every joint at 15 units/s in a model session, so stay under it:
    # a trajectory faster than the cap would lag the command and could trip the tracking-error stop.
    arm_speed: float = 12.0  # deg/s
    gripper_speed: float = 15.0  # units/s
    hz: float = 20.0
    dwell_s: float = 0.6  # pause after closing / opening the gripper
    max_chunk_s: float = 50.0  # a model session lasts at most 60 s; split longer plans at segment boundaries

    # IK: tool approach pitch below horizontal [deg], tried in order (90 = straight down).
    pitches_deg: tuple[float, ...] = (90.0, 75.0, 60.0, 45.0)
    max_rot_err_rad: float = 0.4
    ik_tol_m: float = 0.003

    # Input handling.
    uv_margin: float = 0.0  # accepted range is [-margin, 1 + margin]
    min_pick_place_dist: float = 0.08  # in u,v units; reject a place point on top of the pick point
    auto_execute: bool = True  # run as soon as both points are selected

    # Service.
    listen_host: str = "127.0.0.1"
    listen_port: int = 8790
    dry_run: bool = False  # plan and report, never send motion

    @classmethod
    def load(cls, path: str | Path | None = "config.yaml") -> Config:
        cfg = cls()
        if path and Path(path).exists():
            data = yaml.safe_load(Path(path).read_text()) or {}
            known = {f.name for f in fields(cls)}
            unknown = set(data) - known
            if unknown:
                raise ValueError(f"Unknown config keys: {sorted(unknown)}")
            for key, value in data.items():
                setattr(cfg, key, tuple(value) if isinstance(value, list) else value)
        return cfg
