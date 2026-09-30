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
from tools.audit_static_field import interpolate_q
from tools.eval_rlpd_policy import make_sac_policy


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--trajectory-dir", required=True)
    ap.add_argument("--split", default="val")
    ap.add_argument("--checkpoint", default="examples/experiments/aviator_manifold/route_a_lut_native_rlpd_800x64")
    ap.add_argument("--step", type=int, default=60000)
    ap.add_argument("--initial-phase-field", type=Path,
                    help="start RL at the frozen field's exact initial q; requires grid-aligned x0")
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
    if args.initial_phase_field:
        with np.load(args.initial_phase_field) as field:
            if not (np.array_equal(field["theta"], env.lookup.theta_axis) and
                    np.array_equal(field["s"], env.lookup.s_axis)):
                ap.error("initial phase field axes do not match manifold")
            phi_grid, q_grid = field["phi"], field["q"]
    sample_obs = {"state": np.asarray(env.observation_space.sample()["state"])[None, :]}
    agent = cfg.make_sac_agent(0, sample_obs, env.action_space.sample())
    agent = agent.replace(state=checkpoints.restore_checkpoint(
        str(Path(args.checkpoint).resolve()), agent.state, step=args.step))
    policy = make_sac_policy(agent)
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {}
    for i, path in enumerate(paths):
        options = {"trajectory_index": i}
        if args.initial_phase_field:
            x0 = trajs[i]["x"][0:1]
            options["initial_phi"] = interpolate_q(env.lookup, phi_grid, x0)[0]
        obs, _ = env.reset(options=options)
        initial = env.lookup.query(env._x[None, :], env._phi[None, :], check_safe=False)
        qs = [np.r_[initial["qL"][0], initial["qR"][0]]]
        if args.initial_phase_field:
            field_q0 = interpolate_q(env.lookup, q_grid, env._x[None, :])[0]
            if np.max(np.abs(qs[0] - field_q0)) > 1e-7:
                raise ValueError(f"{Path(path).name}: initial q mismatch; use grid-aligned task starts")
        xs = [env._x.copy()]
        phis = [env._phi.copy()]
        actions = []
        done = False
        terminal = "end"
        while not done:
            action = policy(obs, env)
            obs, _, terminated, truncated, info = env.step(action)
            if "q" in info:
                qs.append(info["q"])
                xs.append(info["x"])
                phis.append(env._phi.copy())
                actions.append(action)
            done = terminated or truncated
            terminal = info.get("termination", terminal)
        np.savez_compressed(args.output / Path(path).name, q=np.asarray(qs), x=np.asarray(xs),
                            phi=np.asarray(phis), action=np.asarray(actions))
        summary[Path(path).stem] = terminal
    (args.output / "termination.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"n": len(summary), "end": sum(v == "end" for v in summary.values())}))


if __name__ == "__main__":
    main()
