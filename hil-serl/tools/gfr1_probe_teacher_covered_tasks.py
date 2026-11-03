#!/usr/bin/env python3
"""Bounded frozen-checkpoint diagnosis on the first teacher task per model.

At a policy stop, search the existing teacher candidate set at exactly the
same state. A found action is a one-step witness, not a whole-path guarantee.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np


def encode_solver_diagnostics(value, path='', nonfinite=None):
    """Keep infeasible-solver infinities explicit while producing strict JSON."""
    if nonfinite is None:
        nonfinite = []
    if isinstance(value, dict):
        return {k: encode_solver_diagnostics(v, f'{path}.{k}', nonfinite) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode_solver_diagnostics(v, f'{path}[{i}]', nonfinite) for i, v in enumerate(value)]
    if isinstance(value, float) and not np.isfinite(value):
        nonfinite.append({'path': path, 'value': str(value)})
        return None
    return value


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-root', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--max-steps', type=int, default=1024)
    a = p.parse_args()
    root = Path(a.run_root).resolve()
    out = Path(a.output)
    if out.exists() or out.with_suffix('.episodes.jsonl').exists():
        raise FileExistsError(out)
    os.environ.update(AVIATOR_GFR_MODEL_MANIFEST=str(root / 'demo_buffer/train_manifest.json'),
                      AVIATOR_GFR_SPLIT='rl_train', AVIATOR_GFR_MODE='structured',
                      AVIATOR_GFR_VISIBLE_ERROR='1', JAX_PLATFORMS='cpu')
    import jax
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    from tools.gfr1_teacher_replay_consistency import choose, candidates
    config = TrainConfig()
    env = config.get_environment()
    core = env.unwrapped
    indices, seen = [], set()
    for i, rec in enumerate(core._model_records):
        if rec['model'] not in seen:
            indices.append(i)
            seen.add(rec['model'])
    agent = config.make_sac_agent(11, env.observation_space.sample(), env.action_space.sample())
    checkpoint = root / 'formal_seed11/training/b4_seed11/checkpoint_40000'
    def hashes():
        return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in checkpoint.rglob('*') if p.is_file()}
    before = hashes()
    agent = agent.replace(state=checkpoints.restore_checkpoint(str(checkpoint), agent.state))
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    with out.with_suffix('.episodes.jsonl').open('x', buffering=1) as log:
        for index in indices:
            obs, _ = env.reset(seed=20261029, options={'trajectory_index': index})
            start = time.perf_counter()
            previous_raw = np.zeros(2)
            complete = stopped = False
            for step in range(a.max_steps):
                action = np.asarray(agent.sample_actions(
                    {'state': np.asarray([obs['state']])},
                    seed=jax.random.PRNGKey(step), argmax=True))[0].astype(np.float32)
                obs, reward, done, truncated, info = env.step(action)
                if done or truncated:
                    stopped = done
                    complete = info.get('termination') == 'end'
                    break
                previous_raw = action.copy()
            row = {'model': core.model_path, 'trajectory_path': core._traj_paths[index],
                   'manifest_index': index, 'steps': step + 1,
                   'outcome': info.get('termination') if done or truncated else 'diagnostic_budget',
                   'complete': complete, 'progress': float(core._u / (len(core._traj['x']) - 1)),
                   'policy_action': action.tolist(), 'closure_residual_inf': float(np.max(np.abs(core.fk.error(core._x, core._q)))),
                   'current_clearance_m': float(core.checker.check(core._x, core._q)[0]),
                   'last_info': core._json_safe_candidate(info), 'teacher_one_step_witness': None}
            if stopped:
                # A rejected step leaves q, x, u and transported basis intact.
                snapshot = [v.copy() for v in (core._x, core._q)]
                result = choose(core, candidates(np.random.default_rng(20261029), previous_raw, 25))
                row['teacher_one_step_witness'] = {'found': result is not None, 'candidate_action_count': 25}
                if result is not None:
                    row['teacher_one_step_witness'].update(
                        action=np.asarray(result[2]).tolist(),
                        accepted_candidate=core._json_safe_candidate(result[3]))
                assert np.array_equal(snapshot[0], core._x) and np.array_equal(snapshot[1], core._q)
            row['elapsed_s'] = time.perf_counter() - start
            nonfinite = []
            row = encode_solver_diagnostics(row, nonfinite=nonfinite)
            row['nonfinite_solver_diagnostics'] = nonfinite
            rows.append(row)
            log.write(json.dumps(row, allow_nan=False) + '\n')
            print(json.dumps({k: v for k, v in row.items() if k != 'last_info'}), flush=True)
    payload = {'kind': 'bounded_teacher_covered_checkpoint_diagnosis',
               'selection': 'first manifest record per model, checkpoint 40000, no parameter changes',
               'max_steps': a.max_steps, 'checkpoint': str(checkpoint),
               'checkpoint_unchanged': hashes() == before, 'episodes': rows,
               'limitations': 'Two selected tasks only. Diagnostic budget is not a task failure. Teacher witness certifies only one checked step.'}
    assert payload['checkpoint_unchanged']
    with out.open('x') as f:
        json.dump(payload, f, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
