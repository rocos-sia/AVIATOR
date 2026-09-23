"""Check whether DP replay transitions are feasible in the training environment.

Run this before RLPD: the prior must be replayable under the exact dynamics,
clearance and branch checks used for online actions. A failed audit exits 1.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pickle
from collections import Counter

import numpy as np

from examples.experiments.aviator_manifold.env import AviatorManifoldEnv


def audit(manifold_dir, trajectory_dir, demo_dir, scale):
    paths = sorted(glob.glob(os.path.join(demo_dir, "traj_*.pkl")))
    if not paths:
        raise ValueError(f"no DP demonstrations in {demo_dir}")
    env = AviatorManifoldEnv(manifold_dir, trajectories=[], phi_dot_scale=scale)
    causes = Counter()
    fractions = []
    initial_phase_errors = []
    initial_clearance_errors = []
    missing = []
    max_observation_error = 0.0
    max_reward_error = 0.0
    transition_mismatches = 0

    for path in paths:
        stem = os.path.splitext(os.path.basename(path))[0]
        source = os.path.join(trajectory_dir, "trajs", "dp_train", stem + ".npz")
        if not os.path.isfile(source):
            missing.append(stem)
            continue
        with open(path, "rb") as f:
            transitions = pickle.load(f)
        with np.load(source) as f:
            env._trajectories = [{k: np.asarray(f[k], dtype=np.float64) for k in f.files}]
        # Route A emits the canonical LUT phase, so the DP's φ₀ is the replay
        # start; recovering it from the closed-loop sin/cos avoids the default
        # φ=0 reset integrating the same actions into a shifted phase path.
        demo_start = transitions[0]["observations"]["state"][0]
        demo_phi = np.array([np.arctan2(demo_start[4], demo_start[5]),
                             np.arctan2(demo_start[6], demo_start[7])])
        obs, _ = env.reset(options={"initial_phi": demo_phi})
        initial_phase_errors.append(float(np.max(np.abs(env._phi - demo_phi))))
        initial_clearance_errors.append(float(abs(obs["state"][12] - demo_start[12])))

        for step, transition in enumerate(transitions, start=1):
            recorded_obs = transition["observations"]["state"][0]
            max_observation_error = max(max_observation_error,
                                        float(np.max(np.abs(obs["state"] - recorded_obs))))
            obs, reward, terminated, truncated, info = env.step(transition["actions"])
            recorded_next = transition["next_observations"]["state"][0]
            max_observation_error = max(max_observation_error,
                                        float(np.max(np.abs(obs["state"] - recorded_next))))
            max_reward_error = max(max_reward_error,
                                   abs(float(reward) - float(transition["rewards"])))
            finished = bool(terminated or truncated)
            if (bool(transition["dones"]) != finished or
                    bool(transition["masks"] == 0) != finished):
                transition_mismatches += 1
            if terminated or truncated:
                causes[info["termination"]] += 1
                fractions.append(step / len(transitions))
                break
        else:
            causes["missing_terminal"] += 1
            fractions.append(1.0)

    return {
        "demo_files": len(paths),
        "paired_trajectories": len(fractions),
        "missing_sources": missing,
        "termination_causes": dict(causes),
        "completion_rate": causes["end"] / len(paths),
        "median_replayed_fraction": float(np.median(fractions)),
        "median_initial_phase_error_rad": float(np.median(initial_phase_errors)),
        "median_initial_clearance_error_m": float(np.median(initial_clearance_errors)),
        "max_observation_error": max_observation_error,
        "max_reward_error": max_reward_error,
        "transition_mismatches": transition_mismatches,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifold-dir", default="data/aviator/manifold_phi")
    parser.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    parser.add_argument("--demo-dir", default="data/aviator/dp_demo")
    parser.add_argument("--expected-count", type=int, default=99)
    args = parser.parse_args()
    with open(os.path.join(args.demo_dir, "phi_dot_scale.json")) as f:
        scale = float(json.load(f)["phi_dot_scale"])
    result = audit(args.manifold_dir, args.trajectory_dir, args.demo_dir, scale)
    print(json.dumps(result, indent=2))
    if (result["demo_files"] != args.expected_count
            or result["missing_sources"]
            or result["completion_rate"] < 1.0
            or result["max_observation_error"] > 1e-4
            or result["max_reward_error"] > 1e-4
            or result["transition_mismatches"]):
        raise SystemExit("DP prior is not fully replayable; refusing unshielded RLPD training")


if __name__ == "__main__":
    main()
