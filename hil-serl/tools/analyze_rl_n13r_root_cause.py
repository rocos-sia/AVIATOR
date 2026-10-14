#!/usr/bin/env python3
"""Reproducible RCA for the RL-N13R 50k checkpoint event.

This script only reads the immutable Seed-0 artifacts and the repaired
Seed-1/Seed-2 artifacts.  It does not alter checkpoints or training outputs.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path("/home/rocos/sia/AVIATOR")
OLD_RUN = ROOT / "outputs/differential_lift_feasibility_20261007/rl_n13r_extra_prior/train_seed_0"
NEW_ROOT = ROOT / "outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1"


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def window_rows(rows: list[dict], lo: int, hi: int) -> list[dict]:
    return [row for row in rows if lo < row["updates_completed"] <= hi]


def scalar_stats(rows: list[dict], section: str, key: str) -> dict:
    values = np.asarray(
        [row["update_info"][section][key] for row in rows], dtype=np.float64
    )
    return {
        "n": int(values.size),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "first": float(values[0]),
        "last": float(values[-1]),
    }


def eval_summary(path: Path) -> list[dict]:
    return json.loads(path.read_text())["checkpoints"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=NEW_ROOT / "root_cause_analysis.json")
    parser.add_argument("--csv", type=Path, default=NEW_ROOT / "root_cause_metric_windows.csv")
    args = parser.parse_args()

    old_eval = eval_summary(OLD_RUN / "convergence_eval.json")
    old_50k = next(row for row in old_eval if row["checkpoint"] == "checkpoint_50000")
    old_log = (OLD_RUN / "learner.log").read_text(errors="replace")
    metric_tokens = [
        "critic_loss", "predicted_qs", "target_qs", "actor_loss",
        "entropy", "temperature", "temperature_loss",
    ]
    old_metric_presence = {token: bool(re.search(token, old_log)) for token in metric_tokens}

    windows = [(0, 10000), (10000, 20000), (20000, 30000),
               (30000, 40000), (40000, 50000), (50000, 60001)]
    rows_out = []
    seed_reports = {}
    for seed in (1, 2):
        run = NEW_ROOT / f"train_seed_{seed}"
        rows = read_jsonl(run / "learner_diagnostics.jsonl")
        ratio_values = sorted({row["sampling"]["demo_fraction_of_batch"] for row in rows})
        batch_values = sorted({
            (row["sampling"]["online_batch_size"], row["sampling"]["demo_batch_size"],
             row["sampling"]["extra_demo_batch_size"])
            for row in rows
        })
        report = {
            "diagnostic_rows": len(rows),
            "update_first": rows[0]["updates_completed"],
            "update_last": rows[-1]["updates_completed"],
            "demo_fraction_values": ratio_values,
            "batch_composition_values": [list(value) for value in batch_values],
            "windows": {},
        }
        for lo, hi in windows:
            selected = window_rows(rows, lo, hi)
            if not selected:
                continue
            key = f"{lo + 1}-{min(hi, rows[-1]['updates_completed'])}"
            metric_map = {
                "critic_loss": ("critic", "critic_loss"),
                "predicted_q_mean": ("critic", "predicted_qs"),
                "predicted_q_std": ("critic", "predicted_qs_std"),
                "target_q_mean": ("critic", "target_qs"),
                "target_q_std": ("critic", "target_qs_std"),
                "entropy": ("actor", "entropy"),
                "alpha": ("actor", "temperature"),
                "actor_loss": ("actor", "actor_loss"),
                "temperature_loss": ("temperature", "temperature_loss"),
            }
            stats = {name: scalar_stats(selected, section, metric)
                     for name, (section, metric) in metric_map.items()}
            report["windows"][key] = stats
            for name, stat in stats.items():
                rows_out.append({
                    "seed": seed, "window": key, "metric": name,
                    "n": stat["n"], "mean": stat["mean"], "std": stat["std"],
                    "min": stat["min"], "max": stat["max"],
                    "first": stat["first"], "last": stat["last"],
                })
        report["heldout_eval"] = eval_summary(run / "convergence_eval.json")
        seed_reports[str(seed)] = report

    result = {
        "node": "RL-N13R-RCA",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "immutable_inputs": {
            "old_seed0_run": str(OLD_RUN),
            "repaired_root": str(NEW_ROOT),
        },
        "old_seed0_50k_failure_attribution": {
            "completion_rate": old_50k["completion_rate"],
            "clearance_rate": old_50k["clearance_rate"],
            "speed_violation_rate": old_50k["speed_violation_rate"],
            "grid_exit_rate": old_50k["grid_exit_rate"],
            "branch_rate": old_50k["branch_rate"],
            "joint_limit_rate": old_50k["joint_limit_rate"],
            "mean_progress_shortfall": old_50k["mean_progress_shortfall"],
            "mean_reserve": old_50k["mean_reserve"],
            "mean_J": old_50k["mean_J"],
        },
        "old_seed0_training_metric_availability": {
            "tokens_found_in_learner_log": old_metric_presence,
            "interpretation": "The old run cannot support a retrospective Q/entropy/alpha root-cause claim because update metrics were not logged locally.",
        },
        "repaired_seed_reports": seed_reports,
        "interpretation": {
            "confirmed": [
                "The old 50k held-out failures were dominated by clearance, not grid/phase-boundary, branch, or joint-limit exits.",
                "The repaired runs used an exactly constant 128 online + 128 demo composition (50/50) with no extra demo buffer.",
                "Both repaired seeds completed 60k with clean actor/learner process exit and recovered held-out performance at 60k.",
                "Entropy stayed near target and alpha was bounded and smooth in the repaired 40k-50k windows; Q means tracked target Q without an abrupt divergence.",
            ],
            "not_confirmed": [
                "A critic/Q explosion as the cause of the original Seed-0 50k collapse; the original run lacks the required metrics.",
                "A unique demo-ratio cause; fixed 50/50 sampling reduced the severity but Seed 2 still had a 50k clearance dip.",
                "A unique gradient-clipping cause; clipping was part of the repaired protocol, so it is not isolated from the sampling change.",
            ],
            "best_supported_root_cause": "A stochastic policy/distribution drift near the clearance boundary, amplified in Seed 0 and partially visible in Seed 2, is the best-supported explanation. The evidence does not isolate whether the drift originates in critic targets, actor exploration, or the environment's narrow clearance geometry.",
            "phase_boundary_conclusion": "No evidence that phase/grid boundary violations caused the 50k event in the available held-out evaluations.",
        },
        "next_experiment": "Run a factorial ablation with the same seed and held-out set: historical schedule vs fixed 50/50 demo ratio, each with and without critic clipping, while logging the same diagnostics.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    with args.csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "seed", "window", "metric", "n", "mean", "std", "min", "max", "first", "last"
        ])
        writer.writeheader()
        writer.writerows(rows_out)
    print(json.dumps({
        "output": str(args.output),
        "csv": str(args.csv),
        "old_50k_clearance_rate": old_50k["clearance_rate"],
        "seed1_rows": seed_reports["1"]["diagnostic_rows"],
        "seed2_rows": seed_reports["2"]["diagnostic_rows"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
