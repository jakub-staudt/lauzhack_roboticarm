"""Sends a Plan to the controller through the repo's WorkbenchClient (claim, arm, heartbeat, STOP handling)."""

from __future__ import annotations

from soarm_lab.notebooks.client import ARM_CONFIRMATION, WorkbenchClient

from .config import Config
from .planner import Plan, PlanError


def make_client(cfg: Config) -> WorkbenchClient:
    return WorkbenchClient(cfg.controller_url, timeout_s=3.0)


def run_plan(client: WorkbenchClient, plan: Plan, cfg: Config) -> None:
    """Runs each chunk as one armed session. The arm must start within 2 deg of the plan's first row."""
    if cfg.dry_run:
        return
    for chunk in plan.chunks(cfg.hz, cfg.max_chunk_s):
        if not chunk:
            continue
        seconds = len(chunk) / cfg.hz + 2.0
        if seconds > 60:
            raise PlanError("A chunk exceeds the 60 s session limit.")
        client.run_targets(chunk, hz=cfg.hz, confirmation=ARM_CONFIRMATION, seconds=seconds,
                           speed=min(15.0, max(cfg.arm_speed, cfg.gripper_speed)))
