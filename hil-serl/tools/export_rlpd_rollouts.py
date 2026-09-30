"""Export deterministic RLPD joint trajectories for independent collision audit."""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
from flax.training import checkpoints

from examples.experiments.aviator_manifold.config import TrainConfig
from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
from tools.eval_rlpd_policy import make_sac_policy


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--trajectory-dir", required=True)
    ap.add_argument("--split", default="val")
    ap.add_argument("--checkpoint", default="examples/experiments/aviator_manifold/route_a_lut_native_rlpd_800x64")
    ap.add_argument("--step", type=int, default=60000)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    paths = sorted(glob.glob(f"{args.trajectory_dir}/trajs/{args.split}/*.npz"))
    if not paths:
        ap.error("no trajectories found")
    trajs = []
    for path in paths:
        with np.load(path) as f:
            trajs.append({key: f[key] for key in f.files})
    cfg = TrainConfig()
    env = AviatorManifoldEnv(args.manifold_dir, trajectories=trajs,
                             phi_dot_scale=cfg.phi_dot_scale)
    sample_obs = {"state": np.asarray(env.observation_space.sample()["state"])[None, :]}
    agent = cfg.make_sac_agent(0, sample_obs, env.action_space.sample())
    agent = agent.replace(state=checkpoints.restore_checkpoint(
        str(Path(args.checkpoint).resolve()), agent.state, step=args.step))
    policy = make_sac_policy(agent)
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {}
    for i, path in enumerate(paths):
        obs, _ = env.reset(options={"trajectory_index": i})
        initial = env.lookup.query(env._x[None, :], env._phi[None, :], check_safe=False)
        qs = [np.r_[initial["qL"][0], initial["qR"][0]]]
        xs = [env._x.copy()]
        done = False
        terminal = "end"
        while not done:
            obs, _, terminated, truncated, info = env.step(policy(obs, env))
            if "q" in info:
                qs.append(info["q"])
                xs.append(info["x"])
            done = terminated or truncated
            terminal = info.get("termination", terminal)
        np.savez_compressed(args.output / Path(path).name, q=np.asarray(qs), x=np.asarray(xs))
        summary[Path(path).stem] = terminal
    (args.output / "termination.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"n": len(summary), "end": sum(v == "end" for v in summary.values())}))


if __name__ == "__main__":
    main()
