"""Evaluate unshielded RLPD (SAC), optionally against a compatible BC baseline.

The BC checkpoint must use the same action scale if one is supplied.

Run from ``hil-serl/``::

    python -m tools.eval_rlpd_policy --split val --n-eps 50 --ckpt-step 20000
"""
from __future__ import annotations

import argparse
import glob
import os

import jax
import numpy as np
from flax.training import checkpoints

from examples.experiments.aviator_manifold.config import TrainConfig
from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from tools.evaluate_policy import (
    make_bc_policy,
    print_table,
    roll_out,
    summarize,
)


def make_sac_policy(sac_agent):
    """Trained SAC agent, deterministic mode (distribution mode = argmax)."""
    def policy(obs, env):
        o = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
        a = sac_agent.sample_actions(observations=o, argmax=True)
        return np.asarray(a)
    return policy


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi/")
    ap.add_argument("--trajectory-dir", default="data/aviator/trajectory_source/")
    ap.add_argument("--split", default="val")
    ap.add_argument("--bc-checkpoint", default=None,
                    help="optional BC checkpoint trained with the same action scale")
    ap.add_argument("--sac-checkpoint", default="examples/experiments/aviator_manifold/unshielded_rlpd_pilot")
    ap.add_argument("--ckpt-step", type=int, default=None, help="restore this SAC step (None = latest)")
    ap.add_argument("--n-eps", type=int, default=None)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg = TrainConfig()
    traj_dir = args.trajectory_dir
    split_paths = sorted(glob.glob(os.path.join(traj_dir, "trajs", args.split, "*.npz")))
    assert split_paths, f"no trajectories in {os.path.join(traj_dir, 'trajs', args.split)}"
    split_trajs = []
    for p in split_paths:
        with np.load(p) as f:
            split_trajs.append({k: np.asarray(f[k], dtype=np.float64) for k in f.files})
    if args.n_eps is not None:
        split_trajs = split_trajs[: args.n_eps]
    print(f"evaluating on {len(split_trajs)} '{args.split}' trajectories")

    env = AviatorManifoldEnv(
        manifold_dir=args.manifold_dir,
        trajectory_dir=None,
        split=args.split,
        phi_dot_scale=cfg.phi_dot_scale,
        trajectories=split_trajs,
    )
    sample_obs = {"state": np.asarray(env.observation_space.sample()["state"])[None, :]}
    sample_action = env.action_space.sample()

    results = {}

    if args.bc_checkpoint:
        bc_agent = cfg.make_bc_agent(seed=args.seed, sample_obs=sample_obs,
                                     sample_action=sample_action)
        bc_agent = bc_agent.replace(state=checkpoints.restore_checkpoint(
            os.path.abspath(args.bc_checkpoint), bc_agent.state))
        results["bc"] = summarize(roll_out(env, split_trajs, make_bc_policy(bc_agent)))

    # RLPD (SAC) policy
    sac_agent = cfg.make_sac_agent(seed=args.seed, sample_obs=sample_obs,
                                   sample_action=sample_action)
    ckpt = checkpoints.restore_checkpoint(
        os.path.abspath(args.sac_checkpoint), sac_agent.state, step=args.ckpt_step)
    sac_agent = sac_agent.replace(state=ckpt)
    results["rlpd"] = summarize(roll_out(env, split_trajs, make_sac_policy(sac_agent)))

    print()
    print_table(results)


if __name__ == "__main__":
    main()
