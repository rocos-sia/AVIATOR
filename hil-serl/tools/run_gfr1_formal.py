#!/usr/bin/env python3
"""Run one auditable fixed-duration GFR-1 B0--B4 extension sweep.

This runner launches the learner and actor serially per condition on one
device.  Every condition gets a fresh output directory; historical N1--N17
artifacts and checkpoints are never touched.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
HIL = ROOT / "hil-serl"
PYTHON = Path(os.environ.get("GFR_PYTHON", "/home/rocos/miniconda3/envs/gfr_clean/bin/python"))
DEFAULT_DEMO_DIR = ROOT / "outputs/differential_lift_feasibility_20261011/gfr_gfr01/demos"

CONDITIONS = {
    "B0": {"mode": "fixed", "visible": "1", "demo": "structured_visible.pkl", "correction": "0.0"},
    "B1": {"mode": "raw", "visible": "0", "demo": "raw_hidden.pkl", "correction": "0.0"},
    "B2": {"mode": "raw", "visible": "1", "demo": "structured_visible.pkl", "correction": "0.0"},
    "B3": {"mode": "structured", "visible": "1", "demo": "structured_visible.pkl", "correction": "0.0"},
    "B4": {"mode": "structured", "visible": "1", "demo": "structured_visible.pkl", "correction": "0.1"},
}


def run_condition(args, condition: str, seed: int, output: Path) -> dict:
    spec = CONDITIONS[condition]
    run_dir = output / f"{condition.lower()}_seed{seed}"
    run_dir.mkdir(parents=True, exist_ok=False)
    common = [
        str(PYTHON), str(HIL / "examples/train_rlpd.py"),
        "--exp_name=aviator_gfr", f"--checkpoint_path={run_dir}",
        f"--seed={seed}", "--debug",
    ]
    demo = str(args.demo_dir / spec["demo"])
    base = os.environ.copy()
    base.update({
        "PYTHONPATH": f"{HIL}:{base.get('PYTHONPATH', '')}".rstrip(":"),
        "AVIATOR_GFR_MODE": spec["mode"],
        "AVIATOR_GFR_VISIBLE_ERROR": spec["visible"],
        "AVIATOR_GFR_CORRECTION_WEIGHT": spec["correction"],
        "AVIATOR_GFR_SPLIT": "rl_train",
        "AVIATOR_MAX_STEPS": str(args.steps),
        # The smoke/extension budget is a transition budget.  Without this
        # explicit setting the actor interprets max_steps as an episode cap
        # and can report a false step-cap failure after one long episode.
        "AVIATOR_MAX_ONLINE_TRANSITIONS": str(args.steps),
        "AVIATOR_NUM_ENVS": str(args.num_envs),
        "AVIATOR_GFR_NUM_ENVS": str(args.num_envs),
        "AVIATOR_MAX_ONLINE_EPISODES": str(args.episodes),
        "AVIATOR_CHECKPOINT_PERIOD": str(args.checkpoint_period),
        "AVIATOR_UPDATES_PER_ONLINE_TRANSITION": "1",
        "AVIATOR_TRAINING_STARTS": str(args.training_starts),
        "AVIATOR_ACTOR_STEP_DELAY": "0",
        "AVIATOR_FIXED_DEMO_FRACTION": "0.5",
        # Keep learner diagnostics next to the condition logs.  The learner
        # reads this exact variable; the previous runner used a different
        # name and silently produced no diagnostics.
        "AVIATOR_DIAGNOSTIC_LOG": str(run_dir / "diagnostics.jsonl"),
        "AVIATOR_DIAGNOSTIC_PERIOD": str(args.diagnostic_period),
        "XLA_PYTHON_CLIENT_PREALLOCATE": "false",
        "XLA_PYTHON_CLIENT_MEM_FRACTION": str(args.mem_fraction),
        "JAX_PLATFORMS": "cuda",
    })
    learner_log = (run_dir / "learner.log").open("w")
    actor_log = (run_dir / "actor.log").open("w")
    start = time.perf_counter()
    learner = subprocess.Popen(
        common + ["--learner", f"--demo_path={demo}"],
        cwd=HIL, env=base, stdout=learner_log, stderr=subprocess.STDOUT,
    )
    time.sleep(args.learner_startup_s)
    actor = subprocess.Popen(
        common + ["--actor"], cwd=HIL, env=base,
        stdout=actor_log, stderr=subprocess.STDOUT,
    )
    actor_rc = actor.wait()
    try:
        learner_rc = learner.wait(timeout=args.learner_timeout_s)
    except subprocess.TimeoutExpired:
        learner.terminate()
        try:
            learner.wait(timeout=15)
        except subprocess.TimeoutExpired:
            learner.kill()
        learner_rc = "timeout"
    learner_log.close()
    actor_log.close()
    # Orbax writes checkpoints directly below checkpoint_path.  Accept the
    # nested layout too because older local runs used that layout.
    checkpoints = sorted(
        {p for p in run_dir.glob("checkpoint_*")} |
        {p for p in (run_dir / "checkpoint").glob("checkpoint_*")}
    )
    return {
        "condition": condition,
        "seed": seed,
        "mode": spec["mode"],
        "visible_error": spec["visible"] == "1",
        "correction_weight": float(spec["correction"]),
        "demo": demo,
        "steps_requested": args.steps,
        "actor_returncode": actor_rc,
        "learner_returncode": learner_rc,
        "elapsed_s": time.perf_counter() - start,
        "run_dir": str(run_dir),
        "checkpoints": [p.name for p in checkpoints],
        "training_ok": actor_rc == 0 and learner_rc == 0 and bool(checkpoints),
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--output", required=True)
    p.add_argument("--demo-dir", type=Path, default=DEFAULT_DEMO_DIR)
    p.add_argument("--seed", type=int, default=11)
    p.add_argument("--num-envs", type=int, default=16)
    p.add_argument("--steps", type=int, default=1000)
    p.add_argument("--episodes", type=int, default=16)
    p.add_argument("--checkpoint-period", type=int, default=500)
    p.add_argument("--training-starts", type=int, default=32)
    p.add_argument("--learner-startup-s", type=float, default=12.0)
    p.add_argument("--learner-timeout-s", type=float, default=240.0)
    p.add_argument("--mem-fraction", type=float, default=0.3)
    p.add_argument("--diagnostic-period", type=int, default=100)
    p.add_argument("--conditions", default=",".join(CONDITIONS))
    args = p.parse_args()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for condition in [x for x in args.conditions.split(",") if x]:
        if condition not in CONDITIONS:
            raise SystemExit(f"unknown condition {condition}")
        records.append(run_condition(args, condition, args.seed, output))
    summary = {
        "node": "GFR-1-extension",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "seed": args.seed,
        "steps": args.steps,
        "conditions": records,
        "frozen": {"dt": 0.005, "d_safe": 0.005, "beta_stop": 0.125,
                   "demo_fraction": 0.5, "device": "cuda",
                   "num_envs": args.num_envs},
        "interpretation": "training extension and infrastructure evidence; effectiveness requires held-out evaluation and feasible-task reference",
    }
    (output / "gfr1_extension_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
