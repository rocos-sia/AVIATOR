#!/usr/bin/env python3
"""RL-N15: measure phase-scheduler latency and unshielded online execution.

This runner deliberately does not change the frozen environment, governor,
endpoint checker, reward, or checkpoints.  It measures the control cycle that
is actually implemented by ``AviatorManifoldEnv``.  That environment performs
manifold lookup and kinematic termination checks, but it does not invoke the
N7/N8 nonlinear endpoint/swept checker or endpoint-aware governor.  The output
therefore records those stages as unavailable rather than timing a fabricated
implementation.

The same validation trajectories and deterministic fixed reset are used for
every method.  The first 100 steps per method/seed are warm-up; at least 1000
subsequent valid steps are retained in the per-step CSV.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import resource
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np


AVIATOR_ROOT = Path("/home/rocos/sia/AVIATOR")
HIL_SERL = AVIATOR_ROOT / "hil-serl"
DEFAULT_OUT = AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency"
MANIFOLD = HIL_SERL / "data/aviator/manifold_phi_stale_pitch-10deg"
VAL_DIR = HIL_SERL / "data/aviator/trajectory_source/trajs/val"
SEED_CKPTS = {
    1: AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_1/checkpoint_60000",
    2: AVIATOR_ROOT / "outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_2/checkpoint_60000",
}


def load_traj(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {key: np.asarray(data[key], dtype=np.float64) for key in data.files}


def finite_or_none(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def timing_summary(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    vals = [float(row[field]) for row in rows if row.get("measured") and row.get(field) is not None]
    if not vals:
        return {"n": 0, "mean_ms": None, "median_ms": None, "p95_ms": None,
                "p99_ms": None, "max_ms": None, "std_ms": None}
    arr = np.asarray(vals, dtype=np.float64) / 1e6
    return {
        "n": int(arr.size),
        "mean_ms": float(np.mean(arr)),
        "median_ms": float(np.median(arr)),
        "p95_ms": float(np.percentile(arr, 95)),
        "p99_ms": float(np.percentile(arr, 99)),
        "max_ms": float(np.max(arr)),
        "std_ms": float(np.std(arr)),
    }


def summarize_episodes(episodes: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(episodes)
    terms = np.asarray([episode["termination"] for episode in episodes], dtype=object)
    def rate(name: str) -> float:
        return float(np.mean(terms == name)) if n else float("nan")
    mins = np.asarray([episode["min_d"] for episode in episodes], dtype=np.float64)
    qdots = np.asarray([episode["max_qdot"] for episode in episodes], dtype=np.float64)
    progress = np.asarray([episode["progress"] for episode in episodes], dtype=np.float64)
    return {
        "n_episodes": n,
        "completion_rate": rate("end"),
        "clearance_failure_rate": rate("clearance"),
        "speed_failure_rate": rate("speed"),
        "grid_failure_rate": rate("grid_exit"),
        "branch_failure_rate": rate("branch"),
        "joint_limit_failure_rate": rate("joint_limit"),
        "invalid_lookup_rate": rate("invalid_lookup"),
        "acceleration_failure_rate": rate("acceleration"),
        "mean_minimum_clearance_mm": finite_or_none(np.nanmean(mins) * 1e3),
        "minimum_clearance_mm": finite_or_none(np.nanmin(mins) * 1e3),
        "mean_max_qdot": finite_or_none(np.nanmean(qdots)),
        "max_qdot": finite_or_none(np.nanmax(qdots)),
        "mean_task_progress": finite_or_none(np.nanmean(progress)),
        "mean_steps": finite_or_none(np.mean([episode["steps"] for episode in episodes])),
        "termination_counts": {str(name): int(np.sum(terms == name)) for name in sorted(set(terms.tolist()))},
        "governor_trigger_count": 0,
        "endpoint_rejection_count": 0,
        "endpoint_checker_status": "not_invoked_in_AviatorManifoldEnv",
        "governor_status": "not_invoked_in_AviatorManifoldEnv",
    }


def make_greedy_policy():
    def policy(obs: dict[str, np.ndarray], env) -> np.ndarray:
        del obs
        x_next = env._traj["x"][env._t + 1]
        box = env.lookup.safe_interval(x_next[None, :])
        center = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
        phi_dot = (center - env._phi) / env.dt
        return np.clip(phi_dot / env.phi_dot_scale, -1.0, 1.0)
    return policy


def make_fixed_policy():
    def policy(obs: dict[str, np.ndarray], env) -> np.ndarray:
        del obs, env
        return np.zeros(2, dtype=np.float32)
    return policy


def device_snapshot() -> dict[str, Any]:
    result: dict[str, Any] = {
        "platform": platform.platform(),
        "python": sys.version,
        "cpu_count": os.cpu_count(),
    }
    try:
        import jax
        result["jax_devices"] = [str(device) for device in jax.devices()]
        result["jax_default_backend"] = jax.default_backend()
    except Exception as exc:  # pragma: no cover - diagnostic only
        result["jax_devices_error"] = repr(exc)
    try:
        completed = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.used,utilization.gpu",
             "--format=csv,noheader,nounits"],
            check=False, capture_output=True, text=True, timeout=3,
        )
        result["nvidia_smi"] = completed.stdout.strip() if completed.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        result["nvidia_smi"] = None
    return result


def make_rl_policy(agent):
    import jax

    def policy(obs: dict[str, np.ndarray], env) -> np.ndarray:
        del env
        batch = {"state": np.asarray(obs["state"], dtype=np.float32)[None, :]}
        action = agent.sample_actions(observations=batch, argmax=True)
        # The synchronization is intentional: the latency is wall-clock
        # inference latency rather than asynchronous dispatch latency.
        action = jax.block_until_ready(action)
        return action
    return policy


def run_condition(
    *,
    method: str,
    seed_label: int,
    trajectories: list[dict[str, np.ndarray]],
    make_env: Callable[[], Any],
    policy: Callable[[dict[str, np.ndarray], Any], np.ndarray],
    warmup_steps: int,
    min_measured_steps: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    env = make_env()
    episodes: list[dict[str, Any]] = []
    timing_rows: list[dict[str, Any]] = []
    global_step = 0
    measured_step = 0
    for episode_index, trajectory in enumerate(trajectories):
        env._trajectories = [trajectory]
        obs, _ = env.reset(seed=10000 + episode_index, options={"trajectory_index": 0})
        done = False
        step_index = 0
        min_d = float("inf")
        max_qdot = 0.0
        initial_phi = np.asarray(env._phi, dtype=np.float64).copy()
        phi_values: list[np.ndarray] = [initial_phi.copy()]
        safe_stop_reason = "end"
        reward_sum = 0.0
        while not done:
            t0 = time.perf_counter_ns()
            t_policy = time.perf_counter_ns()
            raw_action = policy(obs, env)
            t_policy_end = time.perf_counter_ns()
            t_decode = time.perf_counter_ns()
            action = np.asarray(raw_action, dtype=np.float64).reshape(2)
            if not np.all(np.isfinite(action)):
                raise RuntimeError(f"non-finite action for {method} seed={seed_label}")
            # No action is clipped for the RL policy. Fixed/greedy are already
            # in range; this assertion preserves the frozen env contract.
            if np.any(np.abs(action) > 1.0 + 1e-7):
                raise RuntimeError(f"out-of-range action for {method} seed={seed_label}: {action}")
            t_decode_end = time.perf_counter_ns()
            t_env = time.perf_counter_ns()
            obs_next, reward, terminated, truncated, info = env.step(action)
            t_env_end = time.perf_counter_ns()
            total_ns = t_env_end - t0
            env_step_ns = t_env_end - t_env
            policy_ns = t_policy_end - t_policy
            decode_ns = t_decode_end - t_decode
            measured = global_step >= warmup_steps
            row = {
                "method": method,
                "seed": seed_label,
                "episode": episode_index,
                "step": step_index,
                "global_step": global_step,
                "measured": bool(measured),
                "policy_inference_ns": int(policy_ns),
                "action_decode_ns": int(decode_ns),
                # The environment performs lookup, kinematic checks, reward,
                # and observation construction inside step().  Splitting this
                # without changing the frozen runner would double-execute it.
                "manifold_lookup_ns": None,
                "endpoint_checker_ns": None,
                "governor_ns": None,
                "environment_step_ns": int(env_step_ns),
                "measured_unshielded_cycle_ns": int(total_ns),
                "endpoint_checker_status": "not_invoked_in_AviatorManifoldEnv",
                "governor_status": "not_invoked_in_AviatorManifoldEnv",
                "d_min": finite_or_none(info.get("d_min")),
                "max_qdot": finite_or_none(info.get("max_qdot")),
                "termination": info.get("termination", "running"),
            }
            # Retain all measured rows until the requested minimum is reached;
            # after that keep every row to preserve complete episode alignment.
            timing_rows.append(row)
            global_step += 1
            if measured:
                measured_step += 1
            reward_sum += float(reward)
            d = finite_or_none(info.get("d_min"))
            qdot = finite_or_none(info.get("max_qdot"))
            if d is not None:
                min_d = min(min_d, d)
            if qdot is not None:
                max_qdot = max(max_qdot, qdot)
            phi_values.append(np.asarray(env._phi, dtype=np.float64).copy())
            done = bool(terminated or truncated)
            if terminated or truncated:
                safe_stop_reason = info.get("termination", "terminated")
            obs = obs_next
            step_index += 1
        total_steps = max(1, len(env._traj["x"]) - 1)
        episodes.append({
            "method": method,
            "seed": seed_label,
            "episode": episode_index,
            "termination": safe_stop_reason,
            "steps": step_index,
            "total_steps": total_steps,
            "progress": float(step_index / total_steps),
            "min_d": float(min_d) if np.isfinite(min_d) else float("nan"),
            "max_qdot": float(max_qdot),
            "initial_phi": initial_phi.tolist(),
            "phase_min": np.min(np.asarray(phi_values), axis=0).tolist(),
            "phase_max": np.max(np.asarray(phi_values), axis=0).tolist(),
            "reward_sum": reward_sum,
            "governor_trigger_count": 0,
            "endpoint_rejection_count": 0,
            "endpoint_checker_status": "not_invoked_in_AviatorManifoldEnv",
            "governor_status": "not_invoked_in_AviatorManifoldEnv",
        })
    # Ensure the protocol's minimum timing sample is explicit.
    if measured_step < min_measured_steps:
        raise RuntimeError(
            f"only {measured_step} measured steps for {method}/seed{seed_label}; "
            f"need {min_measured_steps}"
        )
    summary = {
        "method": method,
        "seed": seed_label,
        "episodes": summarize_episodes(episodes),
        "timing": {
            field: timing_summary(timing_rows, field)
            for field in ("policy_inference_ns", "action_decode_ns", "environment_step_ns",
                          "measured_unshielded_cycle_ns")
        },
        "warmup_steps_discarded": warmup_steps,
        "measured_steps": measured_step,
        "timing_scope": "simulation-side unshielded manifold control cycle",
        "endpoint_checker_status": "not_invoked_in_AviatorManifoldEnv",
        "governor_status": "not_invoked_in_AviatorManifoldEnv",
    }
    return summary, episodes, timing_rows


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_report(output: Path, raw: dict[str, Any], summary: dict[str, Any]) -> str:
    lines = [
        "# RL-N15 Online Efficiency (2026-10-10)",
        "",
        "## Scope",
        "",
        "This node measures deterministic RL phase scheduling against fixed-phase and "
        "greedy-phase baselines on the same 50 validation trajectories. It evaluates "
        "simulation-side latency and the unshielded manifold environment only.",
        "",
        "The current `AviatorManifoldEnv` performs manifold lookup and kinematic "
        "termination checks, but does not invoke the N7/N8 nonlinear endpoint/swept "
        "checker or endpoint-aware governor. Those stages were not skipped in a "
        "claimed full pipeline; they were unavailable and are marked as such in every "
        "raw row. Therefore this node cannot establish full endpoint-aware closed-loop "
        "speedup.",
        "",
        "## Timing protocol",
        "",
        f"- Validation episodes per method/seed: {raw['n_episodes']}",
        f"- Warm-up steps discarded from timing: {raw['warmup_steps']}",
        f"- Environment dt: {raw['dt_s']} s",
        "- RL action: deterministic SAC checkpoint at step 60000 (`argmax=True`)",
        "- Checkpoint continuation/training: none",
        "- Device: " + ", ".join(raw["device"].get("jax_devices", [])),
        "",
        "## Measured results",
        "",
        "| method | seed | completion | clear. fail | speed fail | cycle P95 (ms) | cycle P99 (ms) | control Hz (P95) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summary["conditions"]:
        ep = item["episodes"]
        timing = item["timing"]["measured_unshielded_cycle_ns"]
        p95 = timing["p95_ms"]
        p99 = timing["p99_ms"]
        hz = 1000.0 / p95 if p95 and p95 > 0 else float("nan")
        lines.append(
            f"| {item['method']} | {item['seed']} | {ep['completion_rate']*100:.1f}% | "
            f"{ep['clearance_failure_rate']*100:.1f}% | {ep['speed_failure_rate']*100:.1f}% | "
            f"{p95:.3f} | {p99:.3f} | {hz:.1f} |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "The measured cycle includes policy/action processing and `env.step`, which "
        "contains the lookup, reward, observation, and kinematic checks. The endpoint "
        "checker and governor columns are `null` because the frozen RL runner does "
        "not expose those stages. Accordingly, any lower RL latency is evidence only "
        "about the simulation-side unshielded cycle; it is not evidence that the full "
        "endpoint-aware system is faster.",
        "",
        f"The machine-readable efficiency gate is `summary['efficiency_gate']['status']`. "
        "The gate is intentionally false when the full safety stages are unavailable, "
        "even if the unshielded timing is lower.",
        "",
        "## Artifacts",
        "",
        f"- Raw JSON: `{output / 'n15_latency_raw.json'}`",
        f"- Per-step timing: `{output / 'n15_latency_per_step.csv'}`",
        f"- Online execution summary: `{output / 'n15_online_execution_summary.csv'}`",
        f"- Baseline comparison: `{output / 'n15_baseline_comparison.csv'}`",
        "",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--warmup-steps", type=int, default=100)
    parser.add_argument("--min-measured-steps", type=int, default=1000)
    parser.add_argument("--skip-plot", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {output}")
    if args.episodes <= 0 or args.episodes > 50:
        raise SystemExit("--episodes must be in 1..50 for the frozen N15 protocol")
    output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(HIL_SERL))
    from flax.training import checkpoints
    from examples.experiments.aviator_manifold.config import TrainConfig
    from examples.experiments.aviator_manifold.env import AviatorManifoldEnv

    paths = sorted(VAL_DIR.glob("traj_*.npz"))
    if len(paths) < args.episodes:
        raise SystemExit(f"need {args.episodes} val trajectories, found {len(paths)}")
    trajectories = [load_traj(path) for path in paths[:args.episodes]]
    cfg = TrainConfig()
    sample_obs = {"state": np.asarray(
        AviatorManifoldEnv(str(MANIFOLD), trajectories=[trajectories[0]])
        .observation_space.sample()["state"]
    )[None, :]}
    sample_action = np.zeros(2, dtype=np.float32)

    def make_env():
        return AviatorManifoldEnv(
            manifold_dir=str(MANIFOLD), trajectory_dir=None, trajectories=trajectories,
            phi_dot_scale=cfg.phi_dot_scale, initial_phi_fraction_range=None,
        )

    raw: dict[str, Any] = {
        "node": "RL-N15",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": "/home/rocos/miniconda3/envs/serl_clean/bin/python",
        "manifold": str(MANIFOLD),
        "val_directory": str(VAL_DIR),
        "n_episodes": args.episodes,
        "warmup_steps": args.warmup_steps,
        "min_measured_steps": args.min_measured_steps,
        "dt_s": float(cfg.__dict__.get("dt", 0.01) or 0.01),
        "phi_dot_scale": float(cfg.phi_dot_scale),
        "device": device_snapshot(),
        "methods": ["RL_seed_1", "RL_seed_2", "fixed_phase", "greedy_phase"],
        "differential_lift_status": "not_run_current_runner_not_directly_callable",
        "safety_stages": {
            "endpoint_checker": "not_invoked_in_AviatorManifoldEnv",
            "governor": "not_invoked_in_AviatorManifoldEnv",
            "same_safety_checker": False,
            "full_online_efficiency_claim_enabled": False,
        },
        "checkpoint_paths": {str(seed): str(path) for seed, path in SEED_CKPTS.items()},
    }
    all_timing: list[dict[str, Any]] = []
    all_episodes: list[dict[str, Any]] = []
    conditions: list[dict[str, Any]] = []

    # Construct each method in a fresh env/agent context, but run serially on
    # the same process and device, as requested for GPU execution.
    for seed in (1, 2):
        cfg_agent = cfg.make_sac_agent(seed=seed, sample_obs=sample_obs, sample_action=sample_action)
        restored = checkpoints.restore_checkpoint(str(SEED_CKPTS[seed]), cfg_agent.state)
        agent = cfg_agent.replace(state=restored)
        rl_summary, episodes, rows = run_condition(
            method=f"RL_seed_{seed}", seed_label=seed, trajectories=trajectories,
            make_env=make_env, policy=make_rl_policy(agent),
            warmup_steps=args.warmup_steps, min_measured_steps=args.min_measured_steps,
        )
        rl_summary["restored_checkpoint_step"] = int(np.asarray(restored.step))
        conditions.append(rl_summary); all_episodes.extend(episodes); all_timing.extend(rows)
        print(json.dumps({"condition": rl_summary["method"], "p95_ms": rl_summary["timing"]["measured_unshielded_cycle_ns"]["p95_ms"], "completion": rl_summary["episodes"]["completion_rate"]}), flush=True)
    for method, policy in (("fixed_phase", make_fixed_policy()), ("greedy_phase", make_greedy_policy())):
        for seed in (1, 2):
            summary, episodes, rows = run_condition(
                method=method, seed_label=seed, trajectories=trajectories,
                make_env=make_env, policy=policy,
                warmup_steps=args.warmup_steps, min_measured_steps=args.min_measured_steps,
            )
            conditions.append(summary); all_episodes.extend(episodes); all_timing.extend(rows)
            print(json.dumps({"condition": f"{method}/seed{seed}", "p95_ms": summary["timing"]["measured_unshielded_cycle_ns"]["p95_ms"], "completion": summary["episodes"]["completion_rate"]}), flush=True)

    # Compare RL to fixed phase at the same seed.  This is a diagnostic of the
    # measured unshielded cycle; the full efficiency gate remains disabled.
    by_key = {(item["method"], item["seed"]): item for item in conditions}
    comparisons: list[dict[str, Any]] = []
    for seed in (1, 2):
        baseline = by_key[("fixed_phase", seed)]
        for method in (f"RL_seed_{seed}", "greedy_phase"):
            candidate = by_key[(method, seed)]
            b_p95 = baseline["timing"]["measured_unshielded_cycle_ns"]["p95_ms"]
            c_p95 = candidate["timing"]["measured_unshielded_cycle_ns"]["p95_ms"]
            comparisons.append({
                "seed": seed, "candidate": method, "baseline": "fixed_phase",
                "candidate_cycle_p95_ms": c_p95, "baseline_cycle_p95_ms": b_p95,
                "unshielded_cycle_speedup": float(b_p95 / c_p95) if c_p95 else None,
                "completion_difference_pp": float((candidate["episodes"]["completion_rate"] - baseline["episodes"]["completion_rate"]) * 100),
                "full_efficiency_claim_eligible": False,
                "reason": "endpoint checker and governor unavailable in frozen RL env",
            })

    summary = {
        "node": "RL-N15",
        "created_utc": raw["created_utc"],
        "conditions": conditions,
        "baseline_comparison": comparisons,
        "efficiency_gate": {
            "status": "blocked_by_missing_full_safety_stages",
            "full_endpoint_aware_efficiency_claim": False,
            "unshielded_cycle_diagnostic_available": True,
            "criteria": {
                "total_p95_and_p99_lower": "reported diagnostically",
                "endpoint_checker_and_governor_invoked": False,
                "completion_within_5pp": "reported diagnostically",
                "safety_no_worse": "not_comparable_without_same endpoint/swept checker",
                "two_seed_same_direction": "reported diagnostically",
            },
        },
        "instrumentation_unknown": 0,
        "interpretation": "Only simulation-side unshielded manifold-cycle latency is measured; no full online efficiency claim is permitted.",
    }
    raw["conditions"] = conditions
    raw["episodes"] = all_episodes
    raw["timing_rows"] = len(all_timing)
    (output / "n15_latency_raw.json").write_text(json.dumps(raw, indent=2, sort_keys=True) + "\n")
    (output / "n15_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    timing_fields = [
        "method", "seed", "episode", "step", "global_step", "measured",
        "policy_inference_ns", "action_decode_ns", "manifold_lookup_ns",
        "endpoint_checker_ns", "governor_ns", "environment_step_ns",
        "measured_unshielded_cycle_ns", "endpoint_checker_status", "governor_status",
        "d_min", "max_qdot", "termination",
    ]
    write_csv(output / "n15_latency_per_step.csv", all_timing, timing_fields)
    episode_fields = [
        "method", "seed", "episode", "termination", "steps", "total_steps", "progress",
        "min_d", "max_qdot", "initial_phi", "phase_min", "phase_max", "reward_sum",
        "governor_trigger_count", "endpoint_rejection_count", "endpoint_checker_status", "governor_status",
    ]
    write_csv(output / "n15_online_execution_summary.csv", all_episodes, episode_fields)
    comparison_fields = [
        "seed", "candidate", "baseline", "candidate_cycle_p95_ms", "baseline_cycle_p95_ms",
        "unshielded_cycle_speedup", "completion_difference_pp", "full_efficiency_claim_eligible", "reason",
    ]
    write_csv(output / "n15_baseline_comparison.csv", comparisons, comparison_fields)

    if not args.skip_plot:
        try:
            import matplotlib.pyplot as plt
            names = [item["method"] + f"/s{item['seed']}" for item in conditions]
            vals = [item["timing"]["measured_unshielded_cycle_ns"]["p95_ms"] for item in conditions]
            fig, ax = plt.subplots(figsize=(10, 4.5))
            ax.bar(np.arange(len(names)), vals, color=["#4472c4" if name.startswith("RL") else "#8faadc" for name in names])
            ax.set_ylabel("unshielded control-cycle P95 (ms)")
            ax.set_xticks(np.arange(len(names)), names, rotation=35, ha="right")
            ax.grid(axis="y", alpha=0.25)
            fig.tight_layout()
            fig.savefig(output / "n15_latency_summary.png", dpi=180)
            plt.close(fig)
        except Exception as exc:
            (output / "plot_error.txt").write_text(repr(exc) + "\n")

    report = build_report(output, raw, summary)
    report_path = AVIATOR_ROOT / "docs/research/rl_n15_online_efficiency_2026-10-10.md"
    report_path.write_text(report)

    status_path = AVIATOR_ROOT / "docs/research/long_horizon_status.md"
    with status_path.open("a") as stream:
        stream.write("\n## RL-N15 — online efficiency (2026-10-10)\n\n")
        stream.write("- Status: measured simulation-side unshielded manifold cycle; full endpoint-aware efficiency gate blocked.\n")
        stream.write("- Output: `outputs/differential_lift_feasibility_20261010/rl_n15_online_efficiency/n15_summary.json`.\n")
        stream.write("- The frozen RL environment did not invoke N7/N8 nonlinear endpoint/swept checker or governor; no full online efficiency claim is made.\n")
        stream.write("- Differential-lift baseline was not run because no compatible current runner was directly callable.\n")
        stream.write("- Next: integrate the same endpoint-aware checker/governor into a timing runner before claiming full closed-loop efficiency; keep this diagnostic immutable.\n")

    print(json.dumps({
        "output": str(output), "report": str(report_path),
        "conditions": len(conditions), "timing_rows": len(all_timing),
        "efficiency_gate": summary["efficiency_gate"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
