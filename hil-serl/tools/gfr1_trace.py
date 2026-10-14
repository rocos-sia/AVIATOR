#!/usr/bin/env python3
"""Write one-step GFR-1 audit traces with nominal and executed actions."""

from __future__ import annotations

import argparse
import json
import os
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
    p.add_argument("--trajectory-index", type=int, default=0)
    p.add_argument("--max-steps", type=int, default=600)
    p.add_argument("--condition", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()

    os.environ["AVIATOR_GFR_MODE"] = args.mode
    os.environ["AVIATOR_GFR_VISIBLE_ERROR"] = "1" if args.visible_error else "0"
    c = TrainConfig()
    env = c.get_environment()
    agent = c.make_sac_agent(11, env.observation_space.sample(), env.action_space.sample())
    state = checkpoints.restore_checkpoint(str(Path(args.checkpoint).resolve()), agent.state)
    agent = agent.replace(state=state)
    obs, _ = env.reset(seed=20263000 + args.trajectory_index,
                       options={"trajectory_index": args.trajectory_index})
    done = truncated = False
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("a") as f:
        step = 0
        while not (done or truncated) and step < args.max_steps:
            action = agent.sample_actions(
                {"state": np.asarray([obs["state"]])},
                seed=jax.random.PRNGKey(700000 + step),
                argmax=True,
            )
            action = np.asarray(jax.device_get(action))[0].astype(np.float32)
            next_obs, reward, done, truncated, info = env.step(action)

            def cv(x):
                if isinstance(x, np.ndarray): return x.tolist()
                if isinstance(x, (np.floating, np.integer, np.bool_)): return x.item()
                return x
            row = {
                "condition": args.condition,
                "trajectory_index": args.trajectory_index,
                "step": step,
                "policy_action_raw": cv(action),
                "action_nominal_r": cv(info.get("action_nominal_r")),
                "action_executed_r": cv(info.get("action_executed_r")),
                "beta_g": cv(info.get("beta_g")),
                "beta_executed": cv(info.get("beta_executed")),
                "tau_requested": 1.0,
                "tau_executed": 1.0,
                "endpoint_validation": bool(info.get("endpoint_validation", False)),
                "swept_validation": bool(info.get("swept_validation", False)),
                "endpoint_check_evaluated": bool(info.get("endpoint_check_evaluated", False)),
                "swept_check_evaluated": bool(info.get("swept_check_evaluated", False)),
                "endpoint_clearance_m": cv(info.get("d_endpoint_m")),
                "swept_clearance_m": cv(info.get("d_swept_m")),
                "closure_residual_inf": cv(info.get("closure_residual_inf")),
                "sigma_min_Aq": cv(info.get("sigma_min_Aq")),
                "joint_limit_margin_rad": cv(info.get("joint_limit_margin_rad")),
                "intervened": cv(info.get("intervened", False)),
                "rejected": cv(info.get("rejected", False)),
                "termination": info.get("termination"),
                "progress": cv(info.get("progress")),
                "reward": cv(reward),
                "done": bool(done or truncated),
            }
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
            obs = next_obs
            step += 1
    print(json.dumps({"condition": args.condition, "steps": step, "output": str(out)}))


if __name__ == "__main__":
    main()
