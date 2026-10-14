#!/usr/bin/env python3
"""Scan local phase-rate candidates without changing the GFR backend.

This is a feasibility diagnostic, not a controller or a training method.  It
uses the same analytic lift, beta schedule and nonlinear endpoint/swept
checker as GFRManifoldEnv, but samples physical r candidates directly in raw
mode.  A positive feasible count shows that the zero-completion smoke cannot
be attributed to local infeasibility at that state.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from experiments.aviator_manifold.gfr_env import GFRManifoldEnv


def run_episode(env: GFRManifoldEnv, trajectory_index: int, steps: int,
                samples: int, seed: int) -> dict:
    env.reset(seed=seed, options={"trajectory_index": trajectory_index})
    rng = np.random.default_rng(seed + trajectory_index)
    rows = []
    for step in range(steps):
        xdot_nom = env._path_state(env._u)[1]
        feasible = []
        total = 0
        for _ in range(samples):
            r = rng.uniform(-env.r_max, env.r_max, size=2)
            for beta in env.governor_schedule:
                total += 1
                info, _ = env._candidate(
                    env._x, env._q, xdot_nom, r, beta,
                    env._previous_n, structured=False,
                )
                if info.get("feasible", False):
                    feasible.append(info)
        row = {
            "step": step,
            "progress": float(env._u / max(len(env._traj["x"]) - 1, 1)),
            "samples": samples,
            "candidate_evaluations": total,
            "feasible_count": len(feasible),
            "feasible_rate": len(feasible) / max(total, 1),
            "best_beta": max((float(x["beta"]) for x in feasible), default=None),
            "best_clearance_m": max((float(x["d_actual_m"]) for x in feasible), default=None),
        }
        rows.append(row)
        if not feasible:
            break
        # Prefer the fastest accepted candidate, breaking ties by clearance.
        chosen = max(feasible, key=lambda x: (float(x["beta"]), float(x["d_actual_m"])))
        env._x = np.asarray(chosen["x_next"], dtype=np.float64)
        env._q = np.asarray(chosen["q_next"], dtype=np.float64)
        env._u = min(env._u + float(chosen["beta"]) * env.dt / env.traj_dt,
                     len(env._traj["x"]) - 1.0)
        env._previous_n = np.asarray(chosen["n_next"], dtype=np.float64)
        if env._u >= len(env._traj["x"]) - 1.0 - 1e-9:
            break
    return {
        "trajectory_index": trajectory_index,
        "steps_requested": steps,
        "steps_completed": len(rows),
        "reached_requested_steps": len(rows) >= steps,
        "first_no_feasible_step": next((r["step"] for r in rows if r["feasible_count"] == 0), None),
        "rows": rows,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--manifold", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--trajectory-dir", required=True)
    p.add_argument("--split", default="val")
    p.add_argument("--episodes", type=int, default=4)
    p.add_argument("--steps", type=int, default=20)
    p.add_argument("--samples", type=int, default=32)
    p.add_argument("--seed", type=int, default=20261012)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    env = GFRManifoldEnv(
        manifold_dir=args.manifold, model_path=args.model,
        trajectory_dir=args.trajectory_dir, split=args.split, mode="raw",
        visible_error=True, dt=0.005, d_safe=0.005, beta_stop=0.125,
        r_max=1.5,
    )
    episodes = [run_episode(env, i, args.steps, args.samples, args.seed)
                for i in range(args.episodes)]
    payload = {
        "kind": "gfr1_local_oracle_feasibility_probe",
        "frozen": {"dt": 0.005, "d_safe": 0.005, "beta_stop": 0.125,
                   "mode": "raw", "samples_per_beta": args.samples},
        "episodes": episodes,
        "interpretation": (
            "This scans local accepted actions only; it does not establish a "
            "globally feasible trajectory or RL effectiveness."
        ),
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({
        "episodes": len(episodes),
        "first_no_feasible_step": [e["first_no_feasible_step"] for e in episodes],
        "steps_completed": [e["steps_completed"] for e in episodes],
    }, indent=2))


if __name__ == "__main__":
    main()
