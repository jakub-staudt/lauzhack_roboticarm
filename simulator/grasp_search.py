"""Search for good grasp settings in the simulator (no learning: a cross-entropy random search over a few numbers).

Each candidate = a set of grasp numbers (gripper opening, approach offset along the jaw axis, grasp height, drop offset, travel height,
settle time, closing speed). It is tried on several randomly placed and rotated 49 mm blocks with position and rotation noise like the real
camera/calibration (about 6 mm, 4 deg). A trial scores 0..4: 0 never got to grasp height, 1 got there, 2 lifted the block, 3 carried it,
4 put it within 15 mm of the target. Each generation keeps the best quarter and narrows around them. Everything is written to
`simulator/grasp_search_out/` (trials.jsonl, generations.jsonl, summary.md). It touches only the simulator, never the real arm.

    python -m simulator.grasp_search --hours 1.9 --workers 8
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent / "grasp_search_out"
REST_Z = 0.7745                      # a 49 mm block resting on the table: centre height [m]

#          name          low     high    start (the settings we use now)
SPACE = {"open_value": (60.0, 100.0, 80.0),
         "pick_offset": (-0.02, 0.03, 0.0084),
         "grip_h": (0.010, 0.034, 0.0185),
         "grasp_shift": (-0.015, 0.015, -0.0044),
         "hover": (0.03, 0.055, 0.045),
         "settle": (0.2, 0.8, 0.3),
         "rate": (20.0, 80.0, 60.0)}
NAMES = list(SPACE)
NOISE_XY_M, NOISE_YAW = 0.006, math.radians(4.0)

_W: dict = {}


def _init() -> None:
    sys.path.insert(0, str(ROOT))
    try:
        os.nice(10)
    except OSError:
        pass
    import mujoco

    import simulator.planner as P
    from simulator.mujoco_backend import MujocoStation

    sim = MujocoStation(render=False)
    _W.update(P=P, planner=P.Planner(), sim=sim, mujoco=mujoco, qpos0=sim.data.qpos.copy(), ctrl0=sim.data.ctrl.copy())


def _place(color: str, x: float, y: float, yaw: float) -> None:
    sim, mj = _W["sim"], _W["mujoco"]
    joint = sim.model.joint(sim.model.body(f"{color}_block").jntadr[0])
    q, v = int(joint.qposadr[0]), int(joint.dofadr[0])
    sim.data.qpos[q:q + 7] = [x, y, REST_Z, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    sim.data.qvel[v:v + 6] = 0


def trial(args: tuple[dict, int]) -> dict:
    params, seed = args
    if not _W:
        _init()
    P, planner, sim, mj = _W["P"], _W["planner"], _W["sim"], _W["mujoco"]
    rng = np.random.default_rng(seed)
    base = planner.base
    with sim.lock:                                            # fresh arm and blocks
        sim.data.qpos[:] = _W["qpos0"]
        sim.data.qvel[:] = 0
        sim.data.ctrl[:] = _W["ctrl0"]
    spot = None
    for _ in range(40):
        w = (float(base[0] + rng.uniform(0.20, 0.32)), float(base[1] + rng.uniform(-0.10, 0.08)))
        if planner.reachable(w):
            spot = w
            break
    if spot is None:
        return {"seed": seed, "skipped": "no reachable spot"}
    yaw = float(rng.uniform(0, math.pi / 2))
    phi = float(rng.uniform(0, 2 * math.pi))
    dst = (spot[0] + 0.10 * math.cos(phi), spot[1] + 0.10 * math.sin(phi))
    with sim.lock:
        _place("red", spot[0], spot[1], yaw)
        _place("blue", -0.42, 0.25, 0.0)
        _place("yellow", -0.42, -0.25, 0.0)
        mj.mj_forward(sim.model, sim.data)
    sim.advance(0.6)
    P.GRIPPER_OPEN, P.PICK_OFFSET, P.GRASP_SHIFT, P.HOVER = params["open_value"], params["pick_offset"], params["grasp_shift"], params["hover"]
    P.GRIP_Z, P.SETTLE_S, P.GRIPPER_RATE = P.TABLE_Z + params["grip_h"], params["settle"], params["rate"]
    yaw_est = yaw + float(rng.normal(0, NOISE_YAW))
    P.YAWS = tuple(yaw_est + k * math.pi / 2 for k in range(4))
    src_est = (spot[0] + float(rng.normal(0, NOISE_XY_M)), spot[1] + float(rng.normal(0, NOISE_XY_M)))
    t0 = time.time()
    try:
        plan = planner.pick_and_place(sim.joints(), src_est, dst, park=None)
    except P.Unreachable as exc:
        return {"seed": seed, "skipped": str(exc)[:60]}
    seen: dict[str, tuple] = {}
    for i, (label, q, dwell) in enumerate(plan):
        sim.command(q)
        sim.advance(dwell)
        nxt = plan[i + 1][0] if i + 1 < len(plan) else None
        if nxt != label:                                      # the last step of each phase
            x, y, z = sim.block("red")
            tip = float(planner.kin.tcp(planner.to_app(sim.joints()))[2, 3] + base[2])
            seen.setdefault(label, (x, y, z, tip))
            seen[label + "_last"] = (x, y, z, tip)
    sim.advance(1.0)
    x, y, z = sim.block("red")
    reached = "grasp" in seen and seen["grasp"][3] - P.TABLE_Z <= params["grip_h"] + 0.008
    lifted = "lift_last" in seen and seen["lift_last"][2] > REST_Z + 0.02
    carried = lifted and "carry_last" in seen and seen["carry_last"][2] > REST_Z + 0.02
    placed = carried and math.hypot(x - dst[0], y - dst[1]) < 0.015 and z < REST_Z + 0.008
    score = 4 if placed else 3 if carried else 2 if lifted else 1 if reached else 0
    return {"seed": seed, "score": score, "reached": bool(reached), "lifted": bool(lifted), "carried": bool(carried), "placed": bool(placed),
            "grasp_tip_mm": None if "grasp" not in seen else round((seen["grasp"][3] - P.TABLE_Z) * 1000, 1),
            "final_err_mm": round(math.hypot(x - dst[0], y - dst[1]) * 1000, 1), "secs": round(time.time() - t0, 1)}


def clip(v: np.ndarray) -> np.ndarray:
    return np.array([min(max(v[i], SPACE[n][0]), SPACE[n][1]) for i, n in enumerate(NAMES)])


def as_params(v) -> dict:
    return {n: float(v[i]) for i, n in enumerate(NAMES)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=1.9)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--trials", type=int, default=4, help="trials per candidate per generation")
    ap.add_argument("--validate", type=int, default=30, help="fresh trials for the final comparison")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    deadline = time.time() + a.hours * 3600
    rng = np.random.default_rng(a.seed)
    mean = np.array([SPACE[n][2] for n in NAMES])
    std = np.array([0.25 * (SPACE[n][1] - SPACE[n][0]) for n in NAMES])
    floor = np.array([0.04 * (SPACE[n][1] - SPACE[n][0]) for n in NAMES])
    history: list[tuple[float, dict, int]] = []
    seed_counter = a.seed * 100000
    tl, gl = open(OUT / "trials.jsonl", "a"), open(OUT / "generations.jsonl", "a")
    reserve = 60 * (a.validate * 8 / max(a.workers, 1) / 4 + 4)         # time kept for the final validation
    with mp.get_context("spawn").Pool(a.workers, initializer=_init) as pool:
        gen = 0
        while time.time() < deadline - reserve:
            cands = [np.array([SPACE[n][2] for n in NAMES])] if gen == 0 else []
            cands += [clip(rng.normal(mean, std)) for _ in range(a.pop - len(cands))]
            jobs = []
            for ci, c in enumerate(cands):
                for _ in range(a.trials):
                    seed_counter += 1
                    jobs.append((ci, as_params(c), seed_counter))
            results: dict[int, list[dict]] = {i: [] for i in range(len(cands))}
            for (ci, params, seed), res in zip(jobs, pool.imap(trial, [(p, s) for _, p, s in jobs]), strict=True):
                results[ci].append(res)
                tl.write(json.dumps({"gen": gen, "cand": ci, "params": params, **res}) + "\n")
            tl.flush()
            fit = []
            for ci, c in enumerate(cands):
                scores = [r["score"] for r in results[ci] if "score" in r]
                fit.append(sum(scores) / (4 * len(scores)) if scores else 0.0)
                history.append((fit[-1], as_params(c), len(scores)))
            order = np.argsort(fit)[::-1]
            elite = np.array([cands[i] for i in order[:max(3, a.pop // 4)]])
            mean, std = elite.mean(axis=0), np.maximum(elite.std(axis=0) * 1.15, floor)
            gl.write(json.dumps({"gen": gen, "best": float(fit[order[0]]), "mean": float(np.mean(fit)), "default": float(fit[0]) if gen == 0 else None,
                                 "best_params": as_params(cands[order[0]]), "std": std.tolist(), "left_s": int(deadline - time.time())}) + "\n")
            gl.flush()
            gen += 1
        # final comparison on fresh trials: the best few candidates found, the settings we use now, and the mean of the last elite
        ranked = sorted([h for h in history if h[2] >= 2], key=lambda h: -h[0])
        picks = [("current settings", as_params(np.array([SPACE[n][2] for n in NAMES])))] + [(f"found #{i + 1}", h[1]) for i, h in enumerate(ranked[:3])]
        picks.append(("elite mean", as_params(clip(mean))))
        final = []
        for name, params in picks:
            base_seed = 9_000_000 + len(final) * 1000
            out = list(pool.imap(trial, [(params, base_seed + k) for k in range(a.validate)]))
            ok = [r for r in out if "score" in r]
            final.append({"name": name, "params": params, "n": len(ok), "mean_score": sum(r["score"] for r in ok) / max(len(ok), 1),
                          "reached": sum(r["reached"] for r in ok), "lifted": sum(r["lifted"] for r in ok), "carried": sum(r["carried"] for r in ok),
                          "placed": sum(r["placed"] for r in ok), "skipped": len(out) - len(ok)})
    json.dump(final, open(OUT / "final.json", "w"), indent=1)
    lines = [f"# Grasp search: {gen} generations, {sum(h[2] for h in history)} scored trials\n",
             "| settings | trials | reached grasp height | lifted | carried | placed (<15 mm) | mean score /4 |", "|---|---|---|---|---|---|---|"]
    for f in final:
        lines.append(f"| {f['name']} | {f['n']} | {f['reached']} | {f['lifted']} | {f['carried']} | {f['placed']} | {f['mean_score']:.2f} |")
    lines.append("\n## Parameters\n")
    lines.append("| settings | " + " | ".join(NAMES) + " |\n|---|" + "---|" * len(NAMES))
    for f in final:
        lines.append(f"| {f['name']} | " + " | ".join(f"{f['params'][n]:.4g}" for n in NAMES) + " |")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
