#!/usr/bin/env python3
"""Evaluate a short GFR-1 checkpoint with deterministic actions.

This evaluator keeps the nonlinear checker/governor in the loop and writes
episode-level metrics.  It is intentionally separate from historical RL
evaluation scripts because those use the ideal-tracking LUT environment.
"""

from __future__ import annotations

import argparse
import json
import hashlib
import math
from collections import Counter
import time
from pathlib import Path

import jax
import numpy as np
from flax.training import checkpoints

from experiments.aviator_gfr.config import TrainConfig
from tools.gfr1_bc_support import policy_state


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--mode", choices=("raw", "structured", "fixed"), required=True)
    p.add_argument("--visible-error", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--output", required=True)
    p.add_argument("--agent-kind", choices=("sac", "bc"), default="sac")
    p.add_argument("--mask-previous-action", action="store_true")
    p.add_argument("--reference-demo-dir", type=Path,
                   help="Optional teacher reference for imitation diagnostics, not a test split.")
    p.add_argument("--full-horizon", action="store_true",
                   help="Use the frozen beta floor to derive the complete-path step budget.")
    args = p.parse_args()
    out = Path(args.output)
    if out.exists():
        raise FileExistsError(f"refusing to overwrite evaluation: {out}")
    out.parent.mkdir(parents=True, exist_ok=True)
    checkpoint = Path(args.checkpoint).resolve()
    if not checkpoint.exists():
        raise FileNotFoundError(checkpoint)
    def checkpoint_hash():
        h = hashlib.sha256()
        paths = sorted(p for p in checkpoint.rglob('*') if p.is_file()) if checkpoint.is_dir() else [checkpoint]
        for path in paths:
            h.update(str(path.relative_to(checkpoint) if checkpoint.is_dir() else path.name).encode())
            h.update(path.read_bytes())
        return h.hexdigest()
    original_hash = checkpoint_hash()

    import os
    os.environ["AVIATOR_GFR_MODE"] = args.mode
    os.environ["AVIATOR_GFR_VISIBLE_ERROR"] = "1" if args.visible_error else "0"
    # Config reads the mode from the environment at construction time.
    config = TrainConfig()
    env = config.get_environment()
    obs, _ = env.reset(seed=20261011, options={"trajectory_index": 0})
    factory = config.make_bc_agent if args.agent_kind == 'bc' else config.make_sac_agent
    agent = factory(
        seed=11,
        sample_obs=env.observation_space.sample(),
        sample_action=env.action_space.sample(),
    )
    state = checkpoints.restore_checkpoint(str(Path(args.checkpoint).resolve()), agent.state)
    agent = agent.replace(state=state)

    references = {}
    step_log = None
    if args.reference_demo_dir is not None:
        import pickle
        audit = json.loads((args.reference_demo_dir / 'demo_manifest.json').read_text())
        from tools.gfr1_prepare_teacher_buffer import sha256
        buffer = args.reference_demo_dir / 'structured_visible.pkl'
        if sha256(buffer) != audit['buffer_sha256']:
            raise ValueError('teacher buffer hash mismatch')
        with buffer.open('rb') as stream:
            demos = pickle.load(stream)
        for ep in audit['episodes']:
            start, count = ep['transition_start'], ep['transition_count']
            seq = demos[start:start + count]
            before = np.asarray([t['observations']['state'] for t in seq]).reshape(count, 40)
            references[(ep['model'], ep['trajectory_path'])] = (
                before, np.asarray([t['actions'] for t in seq]))
        del demos
        step_log = out.with_suffix('.steps.jsonl').open('x', buffering=1)

    rows = []
    episode_log = out.with_suffix('.episodes.jsonl').open('x', buffering=1)
    for episode in range(args.episodes):
        obs, _ = env.reset(seed=20262000 + episode, options={"trajectory_index": episode})
        done = truncated = False
        steps = 0
        initial_state = np.asarray(obs['state']).reshape(-1)
        min_clearance = float(initial_state[30])
        max_closure = float(np.max(np.abs(initial_state[18:30])))
        interventions = 0
        endpoint_rejections = 0
        started = time.perf_counter()
        last = {}
        core = env.unwrapped
        step_budget = (math.ceil((len(core._traj['x']) - 1) * core.traj_dt /
                                (core.dt * core.beta_stop)) + 2) if args.full_horizon else 2000
        # The old fixed 2000-step limit truncated valid complete trajectories.
        accepted_min_clearance = float(obs['state'].reshape(-1)[30])
        accepted_clearance_sum = accepted_min_clearance
        accepted_count = 1
        endpoint_failed_candidates = swept_failed_candidates = 0
        alternative_recoveries = boundary_corrections = candidate_evaluations = 0
        accepted_validation_violations = 0
        reference_deviation = []
        reference = references.get((core.model_path, core._traj_paths[episode]))
        if args.reference_demo_dir is not None and reference is None:
            raise ValueError('evaluation task has no matching model-consistent teacher')
        while not (done or truncated):
            before_state = np.asarray(obs['state']).reshape(-1).copy()
            actor_state = policy_state(obs['state'], args.mask_previous_action)
            action = agent.sample_actions(
                observations={"state": np.asarray([actor_state])},
                seed=jax.random.PRNGKey(500000 + episode * 10000 + steps),
                argmax=True,
            )
            action = np.asarray(jax.device_get(action))[0].astype(np.float32)
            obs, reward, done, truncated, info = env.step(action)
            steps += 1
            alternative_recoveries += int(info.get("alternative_recovered", False))
            boundary_corrections += int(info.get("boundary_correction_iterations", 0))
            candidate_evaluations += int(info.get("actual_candidate_evaluations", 0))
            if info.get('beta_executed', 0.) > 0.:
                accepted_validation_violations += int(not (
                    info.get('endpoint_validation', False) and info.get('swept_validation', False)
                    and float(info.get('d_actual_m', -np.inf)) >= core.d_safe - 1e-9
                    and float(info.get('joint_limit_margin_rad', -np.inf)) >= -1e-9
                    and float(info.get('qdot_max_rad_s', np.inf)) <= core.qdot_max + 2e-5))
            if accepted_validation_violations:
                raise RuntimeError('accepted action violates frozen nonlinear checker contract')
            if reference is not None:
                states, labels = reference
                # Nearest task-progress reference, not a claim that the two
                # trajectories have identical states or null-space gauges.
                idx = int(np.searchsorted(states[:, 39], before_state[39]))
                idx = min(idx, len(states) - 1)
                if idx and abs(states[idx - 1, 39] - before_state[39]) < abs(states[idx, 39] - before_state[39]):
                    idx -= 1
                deviation = float(np.linalg.norm(action - labels[idx]))
                reference_deviation.append(deviation)
                log_row = {
                    'episode': episode, 'step': steps, 'progress_before': float(before_state[39]),
                    'previous_executed_r_observed': before_state[33:35].tolist(),
                    'previous_executed_r_policy_input': np.asarray(actor_state).reshape(-1)[33:35].tolist(),
                    'progress_after': float(info['progress']), 'policy_action_raw': action.tolist(),
                    'action_executed_r': info['action_executed_r'], 'beta_executed': info['beta_executed'],
                    'teacher_reference_step': idx, 'teacher_reference_action': labels[idx].tolist(),
                    'raw_action_l2_difference': deviation,
                    'q_reference_l2_difference': float(np.linalg.norm(before_state[4:18] - states[idx, 4:18])),
                    'clearance_before_m': float(before_state[30]),
                    'closure_before_inf': float(np.max(np.abs(before_state[18:30]))),
                    'intervened': bool(info.get('intervened', False)),
                    'projection_or_retry': bool(info.get('rejected', False)),
                    'termination': info.get('termination'),
                    'candidate_failure_reasons': [c.get('reason', 'accepted') for c in info.get('candidate_trace', [])],
                }
                step_log.write(json.dumps(log_row, allow_nan=False) + '\n')
            min_clearance = min(min_clearance, float(info.get("d_actual_m", np.inf)))
            max_closure = max(max_closure, float(info.get("closure_residual_inf", 0.0)))
            interventions += int(info.get("intervened", False))
            endpoint_rejections += int(info.get("rejected", False))
            endpoint_failed_candidates += int(info['actual_endpoint_failed_candidates'])
            swept_failed_candidates += int(info['actual_swept_failed_candidates'])
            current_clearance = float(obs['state'].reshape(-1)[30])
            if not np.isfinite(obs['state']).all():
                raise RuntimeError('non-finite evaluation observation')
            accepted_min_clearance = min(accepted_min_clearance, current_clearance)
            accepted_clearance_sum += current_clearance
            accepted_count += 1
            last = info
            if steps > step_budget:
                raise RuntimeError(f"evaluation exceeded complete-path budget {step_budget}")
        rows.append({
            "episode": episode,
            "alternative_recoveries": alternative_recoveries,
            "boundary_correction_iterations": boundary_corrections,
            "actual_candidate_evaluations": candidate_evaluations,
            "steps": steps,
            "termination": last.get("termination"),
            "completed": bool(last.get("termination") == "end"),
            "progress": float(last.get("progress", np.nan)),
            "min_clearance_m": float(min_clearance),
            "max_closure_residual_inf": float(max_closure),
            "intervention_steps": interventions,
            "endpoint_rejection_steps": endpoint_rejections,
            "elapsed_s": time.perf_counter() - started,
            "step_budget": step_budget,
            "model": core.model_path,
            "trajectory_path": core._traj_paths[episode],
            "minimum_executed_clearance_m": accepted_min_clearance,
            "mean_executed_clearance_m": accepted_clearance_sum / accepted_count,
            "endpoint_failed_candidates": endpoint_failed_candidates,
            "swept_failed_candidates": swept_failed_candidates,
            "accepted_validation_violations": accepted_validation_violations,
            "mean_raw_action_reference_l2": float(np.mean(reference_deviation)) if reference_deviation else None,
        })
        episode_log.write(json.dumps(rows[-1]) + '\n')
        print(json.dumps(rows[-1]), flush=True)
    episode_log.close()
    if step_log is not None:
        step_log.close()
    if checkpoint_hash() != original_hash:
        raise RuntimeError('checkpoint changed during evaluation')

    completed = sum(int(r["completed"]) for r in rows)
    payload = {
        "kind": "gfr1_repaired_checkpoint_evaluation",
        "executor": "combined_repair_v1",
        "evaluation_role": "development gate, not untouched test",
        "agent_kind": args.agent_kind,
        "mask_previous_action": args.mask_previous_action,
        "reference_demo_dir": str(args.reference_demo_dir) if args.reference_demo_dir else None,
        "reference_note": "Teacher-covered imitation reconstruction; progress-aligned action difference is descriptive, not an optimality metric." if references else None,
        "checkpoint": str(Path(args.checkpoint).resolve()),
        "checkpoint_sha256": original_hash,
        "instrumentation_unknown": 0,
        "model_manifest": config.gfr_model_manifest,
        "split": config.trajectory_split,
        "full_horizon": args.full_horizon,
        "metric_note": "legacy endpoint_rejection_steps counts projection or beta retries; actual failed checker candidates and executed-state clearance are reported separately",
        "mode": args.mode,
        "visible_error": args.visible_error,
        "device": str(jax.devices()),
        "episodes": rows,
        "summary": {
            "episodes": len(rows),
            "completion_rate": completed / max(len(rows), 1),
            "completed": completed,
            "termination_counts": dict(Counter(r['termination'] for r in rows)),
            "mean_alternative_recoveries": float(np.mean([r["alternative_recoveries"] for r in rows])),
            "mean_boundary_corrections": float(np.mean([r["boundary_correction_iterations"] for r in rows])),
            "mean_candidate_evaluations": float(np.mean([r["actual_candidate_evaluations"] for r in rows])),
            "mean_progress": float(np.mean([r["progress"] for r in rows])),
            "mean_min_clearance_m": float(np.mean([r["min_clearance_m"] for r in rows])),
            "mean_intervention_steps": float(np.mean([r["intervention_steps"] for r in rows])),
            "mean_endpoint_rejection_steps": float(np.mean([r["endpoint_rejection_steps"] for r in rows])),
            "mean_elapsed_s": float(np.mean([r["elapsed_s"] for r in rows])),
        },
    }
    from tools.gfr1_gate_support import atomic_json
    atomic_json(out, payload)
    print(json.dumps(payload["summary"], indent=2))


if __name__ == "__main__":
    main()
