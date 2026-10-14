#!/usr/bin/env python3
"""RL-N13 small three-seed training and validation sanity gate.

The learner/actor pair uses the existing RLPD implementation and a stale-
manifold-compatible greedy replay prior generated locally for this node.  The
experiment is deliberately small; it tests training infrastructure and policy
generalization, not safety.  The output directory is immutable once populated.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pickle
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path("/home/rocos/sia/AVIATOR")
HIL = ROOT / "hil-serl"
PYTHON = Path("/home/rocos/miniconda3/envs/serl_clean/bin/python")
MANIFOLD = HIL / "data/aviator/manifold_phi_stale_pitch-10deg"
TRAJECTORIES = HIL / "data/aviator/trajectory_source"
DEFAULT_OUTPUT = ROOT / "outputs/differential_lift_feasibility_20261007/rl_n13_training"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--seeds", default="0,1,2")
    p.add_argument("--learner-steps", type=int, default=20000)
    p.add_argument("--actor-steps", type=int, default=20000)
    p.add_argument("--episodes", type=int, default=16)
    p.add_argument("--actor-envs", type=int, default=2)
    p.add_argument("--val-episodes", type=int, default=50)
    return p.parse_args()


def load_traj(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {key: np.asarray(data[key], dtype=np.float64) for key in data.files}


def write_greedy_demo(output: Path, episodes: int, seed: int = 1307) -> list[Path]:
    """Generate stale-manifold-compatible fixed-trajectory replay transitions."""
    sys.path.insert(0, str(HIL))
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

    traj_paths = sorted((TRAJECTORIES / "trajs/rl_train").glob("traj_*.npz"))
    if not traj_paths:
        raise RuntimeError("no rl_train trajectories found")
    trajs = [load_traj(traj_paths[i % len(traj_paths)]) for i in range(episodes)]
    env = AviatorManifoldEnv(
        manifold_dir=str(MANIFOLD), trajectory_dir=None, trajectories=trajs,
        phi_dot_scale=1.5, dt=0.01,
    )
    demo_dir = output / "greedy_demo"
    demo_dir.mkdir(parents=True, exist_ok=False)
    paths: list[Path] = []
    for index, traj in enumerate(trajs):
        env._trajectories = [traj]
        obs, _ = env.reset(seed=seed + index)
        transitions = []
        done = False
        while not done:
            x_next = env._traj["x"][env._t + 1]
            box = env.lookup.safe_interval(x_next[None, :])
            center = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
            action = np.clip((center - env._phi) / env.dt / env.phi_dot_scale, -1.0, 1.0)
            next_obs, reward, terminated, truncated, info = env.step(action)
            transitions.append({
                "observations": obs,
                "actions": np.asarray(action, dtype=np.float32),
                "next_observations": next_obs,
                "rewards": float(reward),
                "masks": float(not terminated),
                "dones": bool(terminated),
            })
            obs = next_obs
            done = bool(terminated or truncated)
        path = demo_dir / f"traj_{index:04d}.pkl"
        with path.open("wb") as stream:
            pickle.dump(transitions, stream)
        paths.append(path)
    return paths


def base_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{HIL}:{env.get('PYTHONPATH', '')}".rstrip(":")
    env["AVIATOR_MANIFOLD_DIR"] = str(MANIFOLD)
    env["AVIATOR_TRAJECTORY_DIR"] = str(TRAJECTORIES)
    env["AVIATOR_NUM_ENVS"] = "2"
    env["AVIATOR_MAX_ONLINE_EPISODES"] = "16"
    env["AVIATOR_ACTOR_STEP_DELAY"] = "0"
    env["AVIATOR_UPDATES_PER_ONLINE_TRANSITION"] = "1"
    env["AVIATOR_CHECKPOINT_PERIOD"] = "5000"
    env["AVIATOR_TRAINING_STARTS"] = "32"
    env["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
    return env


def run_seed(seed: int, output: Path, demo_paths: list[Path], learner_steps: int,
             actor_steps: int, episodes: int, actor_envs: int) -> dict:
    run_dir = output / f"seed_{seed}"
    run_dir.mkdir(parents=False, exist_ok=False)
    (run_dir / "metadata.json").write_text(json.dumps({
        "seed": seed,
        "manifold": str(MANIFOLD),
        "trajectory_dir": str(TRAJECTORIES),
        "learner_steps": learner_steps,
        "actor_steps": actor_steps,
        "episodes": episodes,
        "actor_envs": actor_envs,
        "action": ["r_L", "r_R"],
        "beta_owner": "governor",
    }, indent=2) + "\n")
    common = [str(PYTHON), str(HIL / "examples/train_rlpd.py"),
              "--exp_name=aviator_manifold", f"--checkpoint_path={run_dir}",
              f"--seed={seed}", "--debug"]
    demo_args = [item for path in demo_paths for item in ("--demo_path", str(path))]
    learner_env = base_env()
    learner_env["AVIATOR_MAX_STEPS"] = str(learner_steps)
    learner_env["AVIATOR_NUM_ENVS"] = str(actor_envs)
    actor_env = base_env()
    actor_env["AVIATOR_MAX_STEPS"] = str(actor_steps)
    actor_env["AVIATOR_MAX_ONLINE_EPISODES"] = str(episodes)
    actor_env["AVIATOR_NUM_ENVS"] = str(actor_envs)
    learner_log = (run_dir / "learner.log").open("w")
    actor_log = (run_dir / "actor.log").open("w")
    learner = subprocess.Popen(common + ["--learner"] + demo_args,
                               cwd=HIL, env=learner_env,
                               stdout=learner_log, stderr=subprocess.STDOUT)
    time.sleep(12)
    actor = subprocess.Popen(common + ["--actor"], cwd=HIL, env=actor_env,
                             stdout=actor_log, stderr=subprocess.STDOUT)
    actor_rc = actor.wait()
    # The learner should stop once it receives actor-done.  Give it a bounded
    # grace period; a stuck learner is a training infrastructure failure.
    try:
        learner_rc = learner.wait(timeout=120)
    except subprocess.TimeoutExpired:
        learner.terminate()
        try:
            learner.wait(timeout=15)
        except subprocess.TimeoutExpired:
            learner.kill()
        learner_rc = "timeout"
    learner_log.close()
    actor_log.close()
    checkpoints = sorted(run_dir.glob("checkpoint_*"))
    return {
        "seed": seed,
        "run_dir": str(run_dir),
        "actor_returncode": actor_rc,
        "learner_returncode": learner_rc,
        "checkpoints": [p.name for p in checkpoints],
        "latest_checkpoint": checkpoints[-1].name if checkpoints else None,
        "training_ok": actor_rc == 0 and learner_rc == 0 and bool(checkpoints),
    }


def evaluate_checkpoints(output: Path, seeds: list[int], val_episodes: int) -> dict:
    sys.path.insert(0, str(HIL))
    import glob as _glob
    from flax.training import checkpoints
    from examples.experiments.aviator_manifold.config import TrainConfig
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv
    from tools.evaluate_policy import make_greedy_policy, make_static_policy, roll_out, summarize

    val_paths = sorted((TRAJECTORIES / "trajs/val").glob("traj_*.npz"))[:val_episodes]
    val_trajs = [load_traj(path) for path in val_paths]
    cfg = TrainConfig()
    env = AviatorManifoldEnv(manifold_dir=str(MANIFOLD), trajectory_dir=None,
                             trajectories=val_trajs, phi_dot_scale=cfg.phi_dot_scale)
    baseline = {
        "fixed_phase": summarize(roll_out(env, val_trajs, make_static_policy())),
        "greedy_phase": summarize(roll_out(env, val_trajs, make_greedy_policy())),
    }
    result = {"baselines": baseline, "seeds": {}}
    for seed in seeds:
        run_dir = output / f"seed_{seed}"
        ckpt_path = checkpoints.latest_checkpoint(str(run_dir))
        if ckpt_path is None:
            result["seeds"][str(seed)] = {"evaluation_ok": False, "reason": "no_checkpoint"}
            continue
        agent = cfg.make_sac_agent(seed=seed,
                                   sample_obs={"state": np.asarray(env.observation_space.sample()["state"])[None, :]},
                                   sample_action=env.action_space.sample())
        agent = agent.replace(state=checkpoints.restore_checkpoint(str(run_dir), agent.state))
        def policy(obs, _env):
            batch = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
            return np.asarray(agent.sample_actions(observations=batch, argmax=True))
        result["seeds"][str(seed)] = {
            "checkpoint": ckpt_path,
            "summary": summarize(roll_out(env, val_trajs, policy)),
            "evaluation_ok": True,
        }
    return result


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=False)
    seeds = [int(item) for item in args.seeds.split(",") if item.strip()]
    demo_paths = write_greedy_demo(output, args.episodes)
    run_records = []
    for seed in seeds:
        run_records.append(run_seed(seed, output, demo_paths, args.learner_steps,
                                     args.actor_steps, args.episodes, args.actor_envs))
    evaluation = evaluate_checkpoints(output, seeds, args.val_episodes)
    summary = {
        "node": "RL-N13",
        "status": "complete",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "training_environment": "AviatorManifoldEnv",
        "manifold": str(MANIFOLD),
        "action": ["r_L", "r_R"],
        "beta_owner": "governor",
        "learner_steps": args.learner_steps,
        "actor_steps": args.actor_steps,
        "episodes": args.episodes,
        "actor_envs": args.actor_envs,
        "seeds": seeds,
        "runs": run_records,
        "evaluation": evaluation,
        "safety_claim_enabled": False,
        "next_node": "RL-N14 paired policy comparison pilot",
    }
    (output / "n13_training_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (output / "n13_policy_eval.json").write_text(json.dumps(evaluation, indent=2) + "\n")
    (output / "n13_report.md").write_text(
        "# RL-N13 training sanity gate\n\n"
        "This node tests SAC/RLPD training and held-out evaluation on the\n"
        "fixed task trajectory. The environment is unshielded; no safety claim\n"
        "is made here. Final safety evaluation belongs to RL-N15.\n"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
