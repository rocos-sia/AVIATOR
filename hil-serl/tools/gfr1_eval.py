#!/usr/bin/env python3
"""Evaluate a short GFR-1 checkpoint with deterministic actions.

This evaluator keeps the nonlinear checker/governor in the loop and writes
episode-level metrics.  It is intentionally separate from historical RL
evaluation scripts because those use the ideal-tracking LUT environment.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import jax
import numpy as np
from flax.training import checkpoints

from experiments.aviator_gfr.config import TrainConfig


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--mode", choices=("raw", "structured", "fixed"), required=True)
    p.add_argument("--visible-error", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--output", required=True)
    args = p.parse_args()

    import os
    os.environ["AVIATOR_GFR_MODE"] = args.mode
    os.environ["AVIATOR_GFR_VISIBLE_ERROR"] = "1" if args.visible_error else "0"
    # Config reads the mode from the environment at construction time.
    config = TrainConfig()
    env = config.get_environment()
    obs, _ = env.reset(seed=20261011, options={"trajectory_index": 0})
    agent = config.make_sac_agent(
        seed=11,
        sample_obs=env.observation_space.sample(),
        sample_action=env.action_space.sample(),
    )
    state = checkpoints.restore_checkpoint(str(Path(args.checkpoint).resolve()), agent.state)
    agent = agent.replace(state=state)

    rows = []
    for episode in range(args.episodes):
        obs, _ = env.reset(seed=20262000 + episode, options={"trajectory_index": episode})
        done = truncated = False
        steps = 0
        min_clearance = np.inf
        max_closure = 0.0
        interventions = 0
        endpoint_rejections = 0
        started = time.perf_counter()
        last = {}
        while not (done or truncated):
            action = agent.sample_actions(
                observations={"state": np.asarray([obs["state"]])},
                seed=jax.random.PRNGKey(500000 + episode * 10000 + steps),
                argmax=True,
            )
            action = np.asarray(jax.device_get(action))[0].astype(np.float32)
            obs, reward, done, truncated, info = env.step(action)
            steps += 1
            min_clearance = min(min_clearance, float(info.get("d_actual_m", np.inf)))
            max_closure = max(max_closure, float(info.get("closure_residual_inf", 0.0)))
            interventions += int(info.get("intervened", False))
            endpoint_rejections += int(info.get("rejected", False))
            last = info
            if steps > 2000:
                raise RuntimeError("evaluation episode exceeded 2000 steps")
        rows.append({
            "episode": episode,
            "steps": steps,
            "termination": last.get("termination"),
            "completed": bool(last.get("termination") == "end"),
            "progress": float(last.get("progress", np.nan)),
            "min_clearance_m": float(min_clearance),
            "max_closure_residual_inf": float(max_closure),
            "intervention_steps": interventions,
            "endpoint_rejection_steps": endpoint_rejections,
            "elapsed_s": time.perf_counter() - started,
        })

    completed = sum(int(r["completed"]) for r in rows)
    payload = {
        "kind": "gfr1_checkpoint_evaluation",
        "checkpoint": str(Path(args.checkpoint).resolve()),
        "mode": args.mode,
        "visible_error": args.visible_error,
        "device": str(jax.devices()),
        "episodes": rows,
        "summary": {
            "episodes": len(rows),
            "completion_rate": completed / max(len(rows), 1),
            "mean_progress": float(np.mean([r["progress"] for r in rows])),
            "mean_min_clearance_m": float(np.mean([r["min_clearance_m"] for r in rows])),
            "mean_intervention_steps": float(np.mean([r["intervention_steps"] for r in rows])),
            "mean_endpoint_rejection_steps": float(np.mean([r["endpoint_rejection_steps"] for r in rows])),
            "mean_elapsed_s": float(np.mean([r["elapsed_s"] for r in rows])),
        },
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps(payload["summary"], indent=2))


if __name__ == "__main__":
    main()
