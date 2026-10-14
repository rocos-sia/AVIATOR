#!/usr/bin/env python3
"""Measure N15 cold-start/first-JAX-call latency without changing checkpoints."""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path("/home/rocos/sia/AVIATOR")
HIL = ROOT / "hil-serl"
MANIFOLD = HIL / "data/aviator/manifold_phi_stale_pitch-10deg"
VAL = HIL / "data/aviator/trajectory_source/trajs/val/traj_0027.npz"
CKPTS = {
    1: ROOT / "outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_1/checkpoint_60000",
    2: ROOT / "outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_2/checkpoint_60000",
}


def main() -> int:
    sys.path.insert(0, str(HIL))
    import jax
    from flax.training import checkpoints
    from examples.experiments.aviator_manifold.config import TrainConfig
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

    with np.load(VAL) as data:
        trajectory = {key: np.asarray(data[key], dtype=np.float64) for key in data.files}
    env = AviatorManifoldEnv(str(MANIFOLD), trajectories=[trajectory])
    obs, _ = env.reset(seed=10000, options={"trajectory_index": 0})
    sample_obs = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
    cfg = TrainConfig()
    out = {
        "node": "RL-N15",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "device": [str(device) for device in jax.devices()],
        "conditions": [],
    }
    for seed in (1, 2):
        t0 = time.perf_counter_ns()
        agent = cfg.make_sac_agent(seed=seed, sample_obs=sample_obs,
                                   sample_action=np.zeros(2, dtype=np.float32))
        t1 = time.perf_counter_ns()
        restored = checkpoints.restore_checkpoint(str(CKPTS[seed]), agent.state)
        agent = agent.replace(state=restored)
        t2 = time.perf_counter_ns()
        def call():
            action = agent.sample_actions(observations=sample_obs, argmax=True)
            return jax.block_until_ready(action)
        c0 = time.perf_counter_ns(); call(); c1 = time.perf_counter_ns()
        c2 = time.perf_counter_ns(); call(); c3 = time.perf_counter_ns()
        out["conditions"].append({
            "method": f"RL_seed_{seed}", "checkpoint_step": int(np.asarray(restored.step)),
            "agent_construct_ms": (t1 - t0) / 1e6,
            "restore_ms": (t2 - t1) / 1e6,
            "first_synchronized_policy_call_ms": (c1 - c0) / 1e6,
            "second_synchronized_policy_call_ms": (c3 - c2) / 1e6,
            "cold_start_included_in_formal_p95": False,
        })
    # The fixed and greedy policies have no XLA compilation stage. Their first
    # call is represented by the measured policy component in the main CSV.
    out["baseline_note"] = "fixed/greedy have no neural XLA compilation stage; their first policy call is already in the main timing trace"
    output = ROOT / "outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_cold_start.json"
    output.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(json.dumps(out, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
