"""Prevalidated explicit soft-limit ingress; never grants hardware authority."""
from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from typing import Any

MAX_SOFT_VIOLATION_DEG = 1.0
MAX_TIP_DRIFT_M = 0.002
MAX_PITCH_CHANGE_DEG = 3.0
JOINT_QUANTUM_DEG = 360.0 / 4096
INGRESS_INNER_MARGIN_DEG = 0.75
PITCH_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")


def ingress_plan(solver: Any, joints: Mapping[str, float], limits: Mapping,
                 hardware: Mapping, quantize: Callable[[dict], dict]) -> list[dict]:
    """Validate the entire constant-tip ingress before any motion is armed."""
    violations = [joint for joint in joints if not limits[joint][0] <= joints[joint] <= limits[joint][1]]
    if not violations:
        return []
    if len(violations) != 1 or violations[0] not in PITCH_JOINTS:
        raise ValueError("Recovery supports one pitch joint outside its software limit at a time.")
    joint = violations[0]
    start = joints[joint]
    low, high = limits[joint]
    violation = max(low - start, start - high)
    if violation > MAX_SOFT_VIOLATION_DEG:
        raise ValueError("Soft-limit overshoot exceeds the one-degree recovery envelope.")
    if any(name not in hardware for name in PITCH_JOINTS):
        raise ValueError("Recovery ingress requires known EEPROM limits for every moving joint.")
    for name, value in joints.items():
        if name in hardware and not hardware[name][0] <= value <= hardware[name][1]:
            raise ValueError("Recovery pose is outside a verified EEPROM limit.")
    # Leave room for measured tracking offset; still prevalidate the entire
    # compensated move and require actual feedback inside the software range.
    margin = min(INGRESS_INNER_MARGIN_DEG, max(JOINT_QUANTUM_DEG, (high - low) / 2))
    end = low + margin if start < low else high - margin
    bounds = dict(limits)
    bounds[joint] = (min(low, start), max(high, start))
    anchor = solver.position(joints)
    pitch = solver.pitch(joints)
    previous = dict(joints)
    previous_violation = violation
    plan = []
    steps = max(1, math.ceil(abs(end - start) / 0.02))
    for index in range(1, steps + 1):
        value = start + (end - start) * index / steps
        candidate = solver.solve_position(previous, anchor, pitch_anchor=pitch,
                                          pitch_budget=math.radians(MAX_PITCH_CHANGE_DEG),
                                          bounds=bounds, fixed={joint: value})
        sent = quantize(candidate)
        current_violation = max(low - sent[joint], sent[joint] - high, 0.0)
        if current_violation > previous_violation + 1e-8:
            raise ValueError("Quantized ingress would worsen a software-limit violation.")
        for name, measured in sent.items():
            if name in hardware and not hardware[name][0] <= measured <= hardware[name][1]:
                raise ValueError("Ingress would cross an EEPROM limit.")
            if name != joint and not limits[name][0] <= measured <= limits[name][1]:
                raise ValueError("Ingress would create another software-limit violation.")
            if name not in PITCH_JOINTS and abs(measured - joints[name]) > 1e-8:
                raise ValueError("Ingress must preserve pan, roll and gripper.")
        if math.dist(solver.position(sent), anchor) > MAX_TIP_DRIFT_M:
            raise ValueError("Ingress exceeds the total tip-deviation budget.")
        if abs(solver._angle(solver.pitch(sent) - pitch)) > math.radians(MAX_PITCH_CHANGE_DEG):
            raise ValueError("Ingress exceeds the total tilt budget.")
        plan.append(sent)
        previous = candidate
        previous_violation = current_violation
    if not low <= plan[-1][joint] <= high:
        raise ValueError("Ingress did not reach the software limits after quantization.")
    return plan
