#!/usr/bin/env python3
"""Resume incomplete cases of a frozen unseen regression, preserving completed evidence."""
from __future__ import annotations

import argparse
import csv
import inspect
import json
import math
import os
import pickle
import time
from collections import Counter
from pathlib import Path

import numpy as np

from tools.gfr1_same_state_diagnosis import digest, safe, snapshot, write_json
from tools.gfr1_recovery_boundary_ablation import independent_check, select_action
from tools.gfr1_full_recovery_regression import full_budget, completed, execute_or_stop

GROUPS = [('original', False, False), ('alternative_only', True, False), ('combined', True, True)]


def load_completed(output):
    rows=[json.loads(line) for line in (output/'episodes.jsonl').read_text().splitlines() if line.strip()]
    keys=set()
    for row in rows:
        key=(row['episode'],row['method'])
        if key in keys: raise ValueError('duplicate completed case')
        result=output/f'episode_{key[0]:02d}_{key[1]}'/'result.json'
        if not result.exists() or json.loads(result.read_text())!=row:
            raise ValueError('completed JSONL and result disagree')
        keys.add(key)
    for result in output.glob('episode_*/result.json'):
        row=json.loads(result.read_text())
        if (row['episode'],row['method']) not in keys:
            raise ValueError('orphan completed case must be reconciled explicitly')
    return rows,keys


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    args = p.parse_args()
    assert args.output.is_dir()
    original_protocol=json.loads((args.output/'protocol.json').read_text())
    for path, expected in original_protocol['input_hashes'].items():
        assert digest(path)==expected, f'Frozen input changed: {path}'
    rows, finished = load_completed(args.output)
    source = json.loads((args.source/'protocol.json').read_text())
    frozen, checkpoint = source['frozen'], Path(source['checkpoint'])
    manifest = json.loads(args.manifest.read_text())
    records = manifest['records']
    assert len(records) == 10 and manifest['split'] == 'test'
    assert all(r['initialization_valid'] for r in records), 'Initialization failures retained in selection audit; do not silently drop cases'
    frozen = dict(frozen, eval_manifest=str(args.manifest.resolve()))
    for path, expected in source['input_hashes'].items():
        assert digest(path) == expected, f'Frozen implementation/source changed: {path}'
    # Set all geometry inputs explicitly, without inheriting training side effects.
    for key in ('AVIATOR_EPISODE_LOG', 'AVIATOR_ASYNC_GATE_DIR',
                'AVIATOR_INITIAL_PHI_FRACTION_MIN', 'AVIATOR_INITIAL_PHI_FRACTION_MAX'):
        os.environ.pop(key, None)
    for key in ('AVIATOR_GFR_MANIFOLD', 'AVIATOR_GFR_TRAJECTORY_DIR', 'AVIATOR_GFR_MODEL'):
        os.environ[key] = frozen['environment'][key]
    os.environ.update(JAX_PLATFORMS='cpu', AVIATOR_GFR_MODEL_MANIFEST=frozen['eval_manifest'],
                      AVIATOR_GFR_SPLIT='test', AVIATOR_GFR_MODE='structured', AVIATOR_GFR_VISIBLE_ERROR='1')
    for key in ('dt', 'd_safe', 'beta_stop', 'r_max', 'qdot_max'):
        os.environ['AVIATOR_GFR_'+key.upper()] = str(frozen[key])
    import jax
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    config = TrainConfig()
    env = config.get_environment()
    core = env.unwrapped
    agent = config.make_sac_agent(11, env.observation_space.sample(), env.action_space.sample())
    agent = agent.replace(state=checkpoints.restore_checkpoint(str(checkpoint), agent.state))
    protected = [Path(path) for path in original_protocol['input_hashes']]
    protected += [Path(__file__), args.output/'protocol.json']
    protected += [args.output/f'episode_{ep:02d}_{name}' for ep,name in sorted(finished)]
    hashes = {str(path):digest(path) for path in protected}
    write_json(args.output/'resume_20261102.json',dict(
        reason='Original runner and finalizer absent; log ended without traceback during episode 8 alternative_only. Cause unknown.',
        completed_cases_preserved=sorted(finished), remaining_cases=[(ep,name) for ep in range(10) for name,_,_ in GROUPS if (ep,name) not in finished],
        protected_hashes=hashes, training_started=False,
        rule='Restart incomplete cases from the trajectory initial state with frozen seeds; archive partial attempt, append new completed rows.'))
    with (args.output/'episodes.jsonl').open('a', buffering=1) as log:
        for episode in range(10):
            first=args.output/f'episode_{episode:02d}_original'/'initial_state.json'
            initial=np.asarray(json.loads(first.read_text())['observation']['state']) if first.exists() else None
            for name, alternative, boundary in GROUPS:
                if (episode,name) in finished:
                    continue
                obs, _ = env.reset(seed=20262000+episode, options={'trajectory_index':episode})
                if initial is None:
                    initial = obs['state'].copy()
                else:
                    np.testing.assert_array_equal(initial, obs['state'])
                assert core.model_path == records[episode]['model']
                assert core._traj_paths[episode] == records[episode]['trajectory_path']
                np.testing.assert_array_equal(core._q, records[episode]['initial_q'])
                case = args.output/f'episode_{episode:02d}_{name}'
                if case.exists():
                    assert not (case/'result.json').exists(), 'Completed case must never be replaced'
                    archive=args.output/'interrupted_attempts'
                    archive.mkdir(exist_ok=True)
                    archived=archive/(case.name+'_before_resume_20261102')
                    assert not archived.exists()
                    case.rename(archived)
                case.mkdir()
                write_json(case/'initial_state.json', dict(observation=obs, q=core._q, x=core._x,
                           model=core.model_path, trajectory=core._traj_paths[episode]))
                budget = full_budget(len(core._traj['x']), core.traj_dt, core.dt, core.beta_stop)
                previous_raw = np.zeros(2, dtype=np.float32)
                counts = Counter({k:0 for k in ('cycles','accepted_steps','alternative_steps',
                    'boundary_correction_steps','candidate_evaluations','correction_iterations',
                    'endpoint_failed_candidates','swept_failed_candidates','intervention_steps',
                    'projection_or_retry_steps','slowed_steps')})
                minimum = float(np.asarray(obs['state']).reshape(-1)[30])
                max_closure = float(np.max(np.abs(np.asarray(obs['state']).reshape(-1)[18:30])))
                min_margin, max_speed = records[episode]['initial_joint_margin_rad'], 0.
                times = []
                started = time.perf_counter()
                stop = 'full_horizon_budget_exhausted'
                print(f'START episode={episode} method={name} budget={budget}', flush=True)
                with (case/'trace.jsonl').open('x', buffering=1) as trace:
                    for cycle in range(budget):
                        t0 = time.perf_counter()
                        raw = np.asarray(jax.device_get(agent.sample_actions(
                            observations={'state':np.asarray([obs['state']])},
                            seed=jax.random.PRNGKey(500000+episode*10000+cycle), argmax=True)))[0].astype(np.float32)
                        chosen, history, policy_exhausted = select_action(
                            core, raw, previous_raw, boundary, alternative,
                            np.random.default_rng(20261102+episode*10000+cycle))
                        trials = [t['result'] for h in history for t in h['trials']]
                        corrections = sum(len(h['trials'])-1 for h in history)
                        counts['cycles'] += 1
                        counts['candidate_evaluations'] += len(trials)
                        counts['correction_iterations'] += corrections
                        counts['endpoint_failed_candidates'] += sum(
                            bool(t.get('endpoint_check_evaluated')) and not t.get('endpoint_validation', False) for t in trials)
                        counts['swept_failed_candidates'] += sum(
                            bool(t.get('swept_check_evaluated')) and not t.get('swept_validation', False) for t in trials)
                        if chosen is None:
                            with (case/'pre_stop_state.pkl').open('xb') as f:
                                pickle.dump(snapshot(env, obs, raw, previous_raw), f)
                            write_json(case/'stop_candidates.json', dict(history=history,
                                       progress=core._u/(len(core._traj['x'])-1), raw=raw))
                        independent = None
                        if chosen is not None:
                            selected_raw, info = chosen
                            independent = independent_check(core, core._x.copy(), core._q.copy(), info)
                        candidates = [h['trials'][-1]['result'] for h in history]
                        obs, _, done, truncated, executed = execute_or_stop(env, obs, raw, chosen, candidates)
                        if not np.isfinite(obs['state']).all():
                            raise RuntimeError('non-finite policy observation')
                        counts['intervention_steps'] += int(executed.get('intervened', False))
                        counts['projection_or_retry_steps'] += int(executed.get('rejected', False))
                        if chosen is not None:
                            counts['accepted_steps'] += 1
                            counts['alternative_steps'] += int(policy_exhausted)
                            counts['boundary_correction_steps'] += int(corrections > 0)
                            counts['slowed_steps'] += int(info['beta'] < 1.-1e-10)
                            previous_raw = selected_raw.copy()
                            minimum = min(minimum, independent['endpoint_m'], independent['swept_m'])
                            max_closure = max(max_closure, info['closure_residual_inf'])
                            min_margin = min(min_margin, independent['margin'])
                            max_speed = max(max_speed, independent['speed'])
                        elapsed = time.perf_counter()-t0
                        times.append(elapsed)
                        trace.write(json.dumps(safe(dict(cycle=cycle, accepted=chosen is not None,
                            policy_raw=raw, selected_raw=None if chosen is None else chosen[0],
                            executed_r=executed['action_executed_r'], beta=executed['beta_executed'],
                            progress=executed['progress'], termination=executed['termination'],
                            candidate_evaluations=len(trials), corrections=corrections,
                            policy_exhausted=policy_exhausted, independent_check=independent,
                            closure_residual_inf=executed.get('closure_residual_inf'),
                            intervened=executed.get('intervened', False),
                            projection_or_retry=executed.get('rejected', False),
                            diagnostic_cycle_s=elapsed)), allow_nan=False)+'\n')
                        if chosen is not None and (policy_exhausted or corrections):
                            write_json(case/f'recovery_{cycle:05d}.json', dict(history=history, chosen=chosen))
                        if cycle % 100 == 0:
                            print(f'PROGRESS episode={episode} method={name} cycle={cycle} progress={executed["progress"]:.5f} elapsed={time.perf_counter()-started:.1f}s', flush=True)
                        if done or truncated:
                            stop = executed['termination']
                            break
                progress = core._u/(len(core._traj['x'])-1)
                row = dict(episode=episode, method=name, completed=completed(stop, progress),
                    progress=progress, stop_reason=stop, **dict(counts), step_budget=budget,
                    minimum_executed_clearance_m=minimum, max_closure_residual_inf=max_closure,
                    minimum_joint_margin_rad=min_margin, maximum_speed_rad_s=max_speed,
                    elapsed_s=time.perf_counter()-started,
                    simulated_execution_s=counts['accepted_steps']*core.dt,
                    nominal_task_duration_s=(len(core._traj['x'])-1)*core.traj_dt,
                    diagnostic_cycle_p50_s=float(np.median(times)), diagnostic_cycle_p95_s=float(np.percentile(times,95)),
                    accepted_validation_violations=0, original_reproduced=None)
                write_json(case/'result.json', row)
                log.write(json.dumps(safe(row), allow_nan=False)+'\n')
                rows.append(row)
                print(json.dumps(safe(row), allow_nan=False), flush=True)
    assert hashes == {str(path):digest(path) for path in protected}, 'protected inputs changed'
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (args.output/'comparison.csv').open('x') as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
    groups = {}
    for name, _, _ in GROUPS:
        rr = [r for r in rows if r['method'] == name]
        groups[name] = dict(episodes=len(rr), completed=sum(r['completed'] for r in rr),
            mean_progress=float(np.mean([r['progress'] for r in rr])),
            stops=dict(Counter(r['stop_reason'] for r in rr)),
            total_candidate_evaluations=sum(r['candidate_evaluations'] for r in rr),
            total_alternative_steps=sum(r.get('alternative_steps',0) for r in rr),
            total_correction_steps=sum(r.get('boundary_correction_steps',0) for r in rr),
            minimum_executed_clearance_m=min(r['minimum_executed_clearance_m'] for r in rr),
            maximum_closure_residual_inf=max(r['max_closure_residual_inf'] for r in rr))
    write_json(args.output/'summary.json', dict(groups=groups, episodes=rows,
        source_hashes_unchanged=True, instrumentation_unknown=0, training_started=False,
        paired_reset_observations_identical=True, unique_test_trajectories=10,
        accepted_validation_violations=0, interpretation='New test-trajectory validation under frozen repairs; no unseen-model or continuous safety claim'))


if __name__ == '__main__':
    main()
