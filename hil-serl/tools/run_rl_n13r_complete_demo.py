#!/usr/bin/env python3
"""RL-N13R: complete stale-manifold demonstrations and convergence pilot.

This node repairs the demonstration side of RL-N13.  It writes each retained
trajectory as a list of step transitions.  The learner later inserts those
transitions into its *separate* ``demo_buffer``; they are never copied into
the online replay buffer.

The demonstration controller is deliberately training-only.  It is a slow
greedy phase-centre controller used to obtain feasible demonstrations on the
stale manifold.  It does not modify the deployment checker or governor.
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
DEFAULT_OUTPUT = ROOT / "outputs/differential_lift_feasibility_20261007/rl_n13r_complete_demo"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--required-demos", type=int, default=64)
    p.add_argument("--max-source", type=int, default=400)
    p.add_argument("--demo-scale", type=float, default=0.10)
    p.add_argument("--seed", type=int, default=1307)
    p.add_argument("--seeds", default="0", help="training seeds; use 0,1,2 for formal validation")
    p.add_argument("--learner-steps", type=int, default=60000)
    p.add_argument("--online-transitions", type=int, default=60000)
    p.add_argument("--actor-envs", type=int, default=8)
    p.add_argument("--checkpoint-period", type=int, default=10000)
    p.add_argument("--val-episodes", type=int, default=50)
    p.add_argument("--demo-manifest", type=Path, default=None,
                   help="Reuse an immutable complete-demo manifest instead of regenerating it")
    p.add_argument("--bc-checkpoint", type=Path, default=None,
                   help="Optional BC checkpoint used to warm-start each SAC actor")
    p.add_argument("--sac-init-checkpoint", type=Path, default=None,
                   help="Optional full SAC checkpoint used to initialize a fresh run")
    p.add_argument("--sac-init-checkpoint-step", type=int, default=None,
                   help="Checkpoint step inside --sac-init-checkpoint")
    p.add_argument("--fixed-demo-fraction", type=float, default=None,
                   help="Fixed demo share in every learner batch, e.g. 0.5")
    p.add_argument("--critic-clip-norm", type=float, default=None,
                   help="Optional critic global gradient clip norm")
    p.add_argument("--diagnostic-period", type=int, default=100,
                   help="Learner update period for local JSONL diagnostics")
    p.add_argument("--historical-extra-prior", action="store_true",
                   help="Reuse the historical fixed 25% extra demo quota")
    p.add_argument("--margin-weight", type=float, default=0.1,
                   help="Clearance warning-layer weight; 0.1 is the frozen baseline")
    p.add_argument("--skip-training", action="store_true")
    return p.parse_args()


def load_traj(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {key: np.asarray(data[key], dtype=np.float64) for key in data.files}


def greedy_demo_action(env, scale: float) -> np.ndarray:
    """Return a safe, slow training-only phase-centre action."""
    x_next = env._traj["x"][env._t + 1]
    box = env.lookup.safe_interval(x_next[None, :])
    center = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
    action = (center - env._phi) / env.dt / env.phi_dot_scale
    return np.clip(scale * action, -1.0, 1.0).astype(np.float32)


def build_complete_demos(output: Path, required: int, max_source: int,
                         scale: float, seed: int) -> tuple[list[Path], dict]:
    sys.path.insert(0, str(HIL))
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

    source_paths = sorted((TRAJECTORIES / "trajs/rl_train").glob("traj_*.npz"))
    if not source_paths:
        raise RuntimeError("no rl_train trajectories found")
    source_paths = source_paths[:max_source]

    demo_dir = output / "complete_demo"
    demo_dir.mkdir(parents=True, exist_ok=False)
    env = AviatorManifoldEnv(
        manifold_dir=str(MANIFOLD), trajectory_dir=None,
        trajectories=[load_traj(source_paths[0])], phi_dot_scale=1.5, dt=0.01,
    )

    retained: list[Path] = []
    records = []
    discard_counts: dict[str, int] = {}
    for source_index, source_path in enumerate(source_paths):
        traj = load_traj(source_path)
        env._trajectories = [traj]
        obs, _ = env.reset(seed=seed + source_index)
        transitions = []
        terminal_info = None
        while True:
            action = greedy_demo_action(env, scale)
            next_obs, reward, terminated, truncated, info = env.step(action)
            episode_done = bool(terminated or truncated)
            # Match the old demo schema exactly.  The terminal mask must be 0
            # for either Gym termination mode, including final truncation.
            transitions.append({
                "observations": obs,
                "actions": np.asarray(action, dtype=np.float32),
                "next_observations": next_obs,
                "rewards": np.asarray(reward, dtype=np.float32),
                "masks": np.asarray(0.0 if episode_done else 1.0, dtype=np.float32),
                "dones": bool(episode_done),
            })
            obs = next_obs
            if episode_done:
                terminal_info = dict(info)
                terminal_info["terminated"] = bool(terminated)
                terminal_info["truncated"] = bool(truncated)
                break

        termination = str(terminal_info.get("termination", "unknown"))
        is_complete = termination == "end" and bool(transitions[-1]["dones"])
        if is_complete:
            path = demo_dir / f"traj_{len(retained):04d}.pkl"
            with path.open("wb") as stream:
                pickle.dump(transitions, stream, protocol=pickle.HIGHEST_PROTOCOL)
            retained.append(path)
        else:
            discard_counts[termination] = discard_counts.get(termination, 0) + 1
        records.append({
            "source_index": source_index,
            "source_path": str(source_path),
            "termination": termination,
            "steps": len(transitions),
            "complete_retained": bool(is_complete),
            "terminal_terminated": bool(terminal_info.get("terminated", False)),
            "terminal_truncated": bool(terminal_info.get("truncated", False)),
            "terminal_d_min": float(terminal_info.get("d_min", np.nan)),
            "terminal_max_qdot": float(terminal_info.get("max_qdot", np.nan)),
        })
        if len(retained) >= required:
            break

    # Independent structural audit: all non-final transitions are nonterminal;
    # exactly the final transition is done=True with mask=0.
    lengths = []
    bad_files = []
    for path in retained:
        with path.open("rb") as stream:
            transitions = pickle.load(stream)
        lengths.append(len(transitions))
        valid = bool(transitions) and all(
            (not bool(t["dones"])) == (i < len(transitions) - 1)
            and float(np.asarray(t["masks"]).reshape(-1)[0]) == (1.0 if i < len(transitions) - 1 else 0.0)
            for i, t in enumerate(transitions)
        )
        if not valid:
            bad_files.append(str(path))
    audit = {
        "node": "RL-N13R",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "manifold": str(MANIFOLD),
        "source_split": "rl_train",
        "source_count_considered": len(records),
        "required_demos": required,
        "retained_demos": len(retained),
        "demo_scale": scale,
        "demo_files": [str(p) for p in retained],
        "length_min": int(min(lengths)) if lengths else 0,
        "length_median": float(np.median(lengths)) if lengths else 0.0,
        "length_max": int(max(lengths)) if lengths else 0,
        "discard_counts": discard_counts,
        "bad_demo_files": bad_files,
        "all_complete": len(retained) >= required and not bad_files,
        "records": records,
    }
    (output / "complete_demo_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    if len(retained) < required or bad_files:
        raise RuntimeError(f"complete demo gate failed: retained={len(retained)} required={required} bad={bad_files}")
    (output / "demo_manifest.json").write_text(json.dumps({
        "demo_paths": [str(p) for p in retained],
        "semantics": "each file is one full trajectory; train_rlpd inserts each transition into independent demo_buffer",
        "terminal_rule": "exactly one final dones=True and masks=0; all earlier dones=False and masks=1",
    }, indent=2) + "\n")
    return retained, audit


def base_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{HIL}:{env.get('PYTHONPATH', '')}".rstrip(":")
    env["AVIATOR_MANIFOLD_DIR"] = str(MANIFOLD)
    env["AVIATOR_TRAJECTORY_DIR"] = str(TRAJECTORIES)
    env["AVIATOR_NUM_ENVS"] = "8"
    env["AVIATOR_MAX_ONLINE_EPISODES"] = "400"
    env["AVIATOR_ACTOR_STEP_DELAY"] = "0"
    env["AVIATOR_UPDATES_PER_ONLINE_TRANSITION"] = "1"
    env["AVIATOR_TRAINING_STARTS"] = "256"
    env["AVIATOR_CHECKPOINT_PERIOD"] = "10000"
    env["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
    return env


def run_training(output: Path, demo_paths: list[Path], seed: int, learner_steps: int,
                 online_transitions: int, actor_envs: int, checkpoint_period: int,
                 bc_checkpoint_path: Path | None = None,
                 extra_demo_paths: list[Path] | None = None,
                 fixed_demo_fraction: float | None = None,
                 critic_clip_norm: float | None = None,
                 margin_weight: float = 0.1,
                 sac_init_checkpoint_path: Path | None = None,
                 sac_init_checkpoint_step: int | None = None,
                 diagnostic_period: int = 100) -> dict:
    run_dir = output / f"train_seed_{seed}"
    run_dir.mkdir(parents=False, exist_ok=False)
    common = [str(PYTHON), str(HIL / "examples/train_rlpd.py"),
              "--exp_name=aviator_manifold", f"--checkpoint_path={run_dir}",
              f"--seed={seed}", "--debug"]
    if bc_checkpoint_path is not None:
        common.append(f"--bc_checkpoint_path={bc_checkpoint_path}")
    if sac_init_checkpoint_path is not None:
        common.append(f"--sac_init_checkpoint_path={sac_init_checkpoint_path}")
    if sac_init_checkpoint_step is not None:
        common.append(f"--sac_init_checkpoint_step={sac_init_checkpoint_step}")
    demo_args = [item for path in demo_paths for item in ("--demo_path", str(path))]
    extra_demo_args = [item for path in (extra_demo_paths or [])
                       for item in ("--extra_demo_path", str(path))]
    learner_env = base_env()
    learner_env.update({"AVIATOR_MAX_STEPS": str(learner_steps),
                        "AVIATOR_NUM_ENVS": str(actor_envs),
                        "AVIATOR_CHECKPOINT_PERIOD": str(checkpoint_period),
                        "AVIATOR_DIAGNOSTIC_LOG": str(run_dir / "learner_diagnostics.jsonl"),
                        "AVIATOR_DIAGNOSTIC_PERIOD": str(diagnostic_period),
                        "AVIATOR_MARGIN_WEIGHT": str(margin_weight)})
    if fixed_demo_fraction is not None:
        learner_env["AVIATOR_FIXED_DEMO_FRACTION"] = str(fixed_demo_fraction)
    if critic_clip_norm is not None:
        learner_env["AVIATOR_CRITIC_CLIP_NORM"] = str(critic_clip_norm)
    actor_env = base_env()
    actor_env.update({"AVIATOR_MAX_STEPS": str(learner_steps),
                      "AVIATOR_MAX_ONLINE_TRANSITIONS": str(online_transitions),
                      "AVIATOR_NUM_ENVS": str(actor_envs),
                      "AVIATOR_MAX_ONLINE_EPISODES": "400",
                      "AVIATOR_CHECKPOINT_PERIOD": str(checkpoint_period),
                      "AVIATOR_MARGIN_WEIGHT": str(margin_weight)})
    (run_dir / "metadata.json").write_text(json.dumps({
        "seed": seed, "learner_steps": learner_steps,
        "online_transitions": online_transitions, "actor_envs": actor_envs,
        "demo_paths": [str(p) for p in demo_paths],
        "bc_checkpoint_path": str(bc_checkpoint_path) if bc_checkpoint_path else None,
        "extra_demo_paths": [str(p) for p in (extra_demo_paths or [])],
        "demo_buffer_is_independent": True,
        "historical_extra_prior": bool(extra_demo_paths),
        "fixed_demo_fraction": fixed_demo_fraction,
        "critic_clip_norm": critic_clip_norm,
        "margin_weight": margin_weight,
        "sac_init_checkpoint_path": str(sac_init_checkpoint_path) if sac_init_checkpoint_path else None,
        "sac_init_checkpoint_step": sac_init_checkpoint_step,
        "diagnostic_period": diagnostic_period,
        "shutdown_protocol": "learner waits for explicit actor-done after cap; server stops after final checkpoint",
    }, indent=2) + "\n")
    learner_log = (run_dir / "learner.log").open("w")
    actor_log = (run_dir / "actor.log").open("w")
    learner = subprocess.Popen(common + ["--learner"] + demo_args + extra_demo_args, cwd=HIL,
                               env=learner_env, stdout=learner_log,
                               stderr=subprocess.STDOUT)
    time.sleep(15)
    actor = subprocess.Popen(common + ["--actor"], cwd=HIL, env=actor_env,
                             stdout=actor_log, stderr=subprocess.STDOUT)
    actor_rc = actor.wait()
    try:
        learner_rc = learner.wait(timeout=300)
    except subprocess.TimeoutExpired:
        learner.terminate()
        try:
            learner.wait(timeout=30)
        except subprocess.TimeoutExpired:
            learner.kill()
        learner_rc = "timeout"
    learner_log.close(); actor_log.close()
    checkpoints = sorted(
        run_dir.glob("checkpoint_*"),
        key=lambda path: int(path.name.rsplit("_", 1)[-1]),
    )
    return {
        "seed": seed, "run_dir": str(run_dir),
        "actor_returncode": actor_rc, "learner_returncode": learner_rc,
        "checkpoints": [p.name for p in checkpoints],
        "latest_checkpoint": checkpoints[-1].name if checkpoints else None,
        "training_process_ok": actor_rc == 0 and learner_rc == 0 and bool(checkpoints),
    }


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=False)
    if args.demo_manifest is not None:
        manifest = json.loads(args.demo_manifest.read_text())
        demo_paths = [Path(path) for path in manifest["demo_paths"]]
        if not demo_paths or not all(path.exists() for path in demo_paths):
            raise SystemExit(f"demo manifest contains missing paths: {args.demo_manifest}")
        audit_path = args.demo_manifest.parent / "complete_demo_audit.json"
        audit = json.loads(audit_path.read_text()) if audit_path.exists() else {
            "source_manifest": str(args.demo_manifest),
            "retained_demos": len(demo_paths),
        }
        (output / "demo_manifest_used.json").write_text(json.dumps({
            "source_manifest": str(args.demo_manifest),
            "demo_paths": [str(path) for path in demo_paths],
            "immutable_reuse": True,
        }, indent=2) + "\n")
    else:
        demo_paths, audit = build_complete_demos(output, args.required_demos,
                                                 args.max_source, args.demo_scale,
                                                 args.seed)
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    runs = []
    if not args.skip_training:
        for seed in seeds:
            runs.append(run_training(output, demo_paths, seed, args.learner_steps,
                                     args.online_transitions, args.actor_envs,
                                     args.checkpoint_period,
                                     bc_checkpoint_path=args.bc_checkpoint,
                                     extra_demo_paths=(demo_paths if args.historical_extra_prior else None),
                                     fixed_demo_fraction=args.fixed_demo_fraction,
                                     critic_clip_norm=args.critic_clip_norm,
                                     margin_weight=args.margin_weight,
                                     sac_init_checkpoint_path=args.sac_init_checkpoint,
                                     sac_init_checkpoint_step=args.sac_init_checkpoint_step,
                                     diagnostic_period=args.diagnostic_period))
    summary = {
        "node": "RL-N13R",
        "status": "complete_demo_generated" if args.skip_training else "training_started_or_finished",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "demo_audit": audit,
        "learner_steps_cap": args.learner_steps,
        "online_transition_budget": args.online_transitions,
        "margin_weight": args.margin_weight,
        "sac_init_checkpoint_path": str(args.sac_init_checkpoint) if args.sac_init_checkpoint else None,
        "sac_init_checkpoint_step": args.sac_init_checkpoint_step,
        "seeds": seeds,
        "runs": runs,
        "safety_claim_enabled": False,
        "next_node": "RL-N14 only after convergence and held-out gate",
    }
    (output / "n13r_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
