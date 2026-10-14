#!/usr/bin/env python3
"""Evaluate every immutable RL-N13R checkpoint on held-out trajectories."""

from __future__ import annotations

import argparse
import glob
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from flax.training import checkpoints


ROOT = Path("/home/rocos/sia/AVIATOR")
HIL = ROOT / "hil-serl"
MANIFOLD = HIL / "data/aviator/manifold_phi_stale_pitch-10deg"
TRAJECTORIES = HIL / "data/aviator/trajectory_source"


def load_traj(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {k: np.asarray(data[k], dtype=np.float64) for k in data.files}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, default=None)
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    sys.path.insert(0, str(HIL))
    from examples.experiments.aviator_manifold.config import TrainConfig
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
    from tools.evaluate_policy import roll_out, summarize

    ckpts = sorted(args.run_dir.glob("checkpoint_*"),
                   key=lambda p: int(p.name.split("_")[-1]))
    paths = sorted((TRAJECTORIES / "trajs/val").glob("traj_*.npz"))[:args.episodes]
    if not ckpts or not paths:
        raise SystemExit("need at least one checkpoint and one validation trajectory")
    trajs = [load_traj(path) for path in paths]
    cfg = TrainConfig()
    env = AviatorManifoldEnv(manifold_dir=str(MANIFOLD), trajectory_dir=None,
                             trajectories=trajs, phi_dot_scale=cfg.phi_dot_scale,
                             initial_phi_fraction_range=cfg.initial_phi_fraction_range)
    agent = cfg.make_sac_agent(
        seed=args.seed,
        sample_obs={"state": np.asarray(env.observation_space.sample()["state"])[None, :]},
        sample_action=env.action_space.sample(),
    )
    records = []
    for ckpt in ckpts:
        restored = checkpoints.restore_checkpoint(str(ckpt), agent.state)
        current = agent.replace(state=restored)

        def policy(obs, _env):
            batch = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
            return np.asarray(current.sample_actions(observations=batch, argmax=True))

        summary = summarize(roll_out(env, trajs, policy))
        records.append({"checkpoint": ckpt.name, **summary})
        print(json.dumps(records[-1], sort_keys=True), flush=True)
    result = {
        "node": "RL-N13R",
        "run_dir": str(args.run_dir),
        "split": "val",
        "n_episodes": len(trajs),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "checkpoints": records,
        "convergence_rule": "do not claim convergence unless the final two checkpoints have stable completion and termination rates; report the curve otherwise",
    }
    output = args.output or (args.run_dir / "convergence_eval.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
