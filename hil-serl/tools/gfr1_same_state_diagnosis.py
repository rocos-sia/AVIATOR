#!/usr/bin/env python3
"""Read-only 40k failure diagnosis, exact-state teacher fork and bounded recovery.

Python line tracing observes the ORIGINAL project_r implementation; it does not
replace its solver, tolerances, action selection or nonlinear checker.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import inspect
import json
import math
import os
import pickle
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np


STATE_FIELDS = ('_u', '_x', '_q', '_previous_n', '_r_prev', '_beta_prev',
                '_last_intervention', '_last_rejection', '_last_recovery', '_last_sigma')


def safe(value):
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json(path, value):
    with Path(path).open('x') as f:
        json.dump(safe(value), f, indent=2, allow_nan=False)


def digest(path):
    path = Path(path)
    h = hashlib.sha256()
    for p in sorted(path.rglob('*')) if path.is_dir() else [path]:
        if p.is_file():
            h.update(str(p.relative_to(path) if path.is_dir() else p.name).encode())
            h.update(p.read_bytes())
    return h.hexdigest()


def snapshot(env, obs, action, previous_raw):
    core = env.unwrapped
    return dict(fields={k: copy.deepcopy(getattr(core, k)) for k in STATE_FIELDS},
                observation=copy.deepcopy(obs), wrapper_history=copy.deepcopy(env.current_obs),
                rng=copy.deepcopy(core.np_random.bit_generator.state),
                action=action.copy(), previous_raw=previous_raw.copy())


def restore(env, state):
    core = env.unwrapped
    for k, v in state['fields'].items():
        setattr(core, k, copy.deepcopy(v))
    env.current_obs = copy.deepcopy(state['wrapper_history'])
    core.np_random.bit_generator.state = copy.deepcopy(state['rng'])
    assert np.array_equal(core._get_obs()['state'], state['observation']['state'].reshape(-1))


def violations(d_endpoint, d_swept, endpoint_contact, swept_contact, speed, margin,
               d_safe, qdot_max):
    reasons = []
    if d_endpoint < d_safe - 1e-9:
        reasons.append('endpoint_clearance')
    if d_swept < d_safe - 1e-9:
        reasons.append('swept_clearance')
    if endpoint_contact:
        reasons.append('endpoint_contact')
    if swept_contact:
        reasons.append('swept_contact')
    if speed > qdot_max + 2e-5:
        reasons.append('joint_speed')
    if margin < -1e-9:
        reasons.append('joint_limit')
    return reasons


class ProjectTrace:
    """Observe original function locals at each Euler checker verdict/return."""

    def __init__(self, fn):
        self.fn = fn
        lines, start = inspect.getsourcelines(fn)
        self.verdict_line = start + next(i for i, s in enumerate(lines)
                                         if s.strip() == 'np.max(np.abs(qdot)) <= qdot_max + 2e-5')

    def call(self, *args, **kwargs):
        record = {'trials': []}

        def trace(frame, event, arg):
            if frame.f_code is not self.fn.__code__:
                return None
            loc = frame.f_locals
            if (event == 'line' and frame.f_lineno == self.verdict_line
                    and (not record['trials'] or record['trials'][-1]['attempt'] != int(loc['_']))):
                speed = float(np.max(np.abs(loc['qdot'])))
                margin = float(min(np.min(loc['q_next'] - loc['lower']),
                                   np.min(loc['upper'] - loc['q_next'])))
                record['trials'].append(dict(
                    attempt=int(loc['_']), r=loc['r'].copy(),
                    q_next=loc['q_next'].copy(), d_endpoint_m=loc['d_next'],
                    d_swept_m=loc['swept_min'], qdot_max_rad_s=speed,
                    joint_limit_margin_rad=margin,
                    violated=violations(loc['d_next'], loc['swept_min'],
                                        loc['contact_next'], loc['swept_contact'],
                                        speed, margin, loc['d_safe'], loc['qdot_max'])))
            if event == 'return':
                labels = [f'joint_{j}_{kind}' for j in range(14)
                          for kind in ('speed_upper', 'speed_lower', 'position_upper', 'position_lower')]
                labels.append('linear_clearance')
                record.update(d0_m=loc['d0'], grad_q=loc['grad'].copy(),
                              polygon=copy.deepcopy(loc['poly']),
                              planes=[dict(label=label, a=a.copy(), b=b)
                                      for label, (a, b) in zip(labels, loc['planes'])],
                              feasible=bool(arg[1]['feasible']))
                if not loc['poly']:
                    idx = next(i for i, (a, b) in enumerate(loc['planes']) if a is loc['aa'])
                    record['empty_at_constraint'] = labels[idx]
                    record['stage'] = 'linear_slice_empty'
                else:
                    record['stage'] = 'euler_accepted' if arg[1]['feasible'] else 'euler_backtracking_exhausted'
            return trace

        previous = sys.gettrace()
        try:
            sys.settrace(trace)
            result = self.fn(*args, **kwargs)
        finally:
            sys.settrace(previous)
        return result, record


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--episodes', type=int, default=10)
    p.add_argument('--continuation-steps', type=int, default=100)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    frozen = json.loads((a.run_root / 'frozen_config.json').read_text())
    manifest = Path(frozen['eval_manifest'])
    checkpoint = a.run_root / 'training/b4_seed11/checkpoint_40000'
    reference_path = a.run_root / 'evaluations/checkpoint_40000.episodes.jsonl'
    reference = [json.loads(s) for s in reference_path.read_text().splitlines()]
    os.environ.update(JAX_PLATFORMS='cpu', AVIATOR_GFR_MODEL_MANIFEST=str(manifest),
                      AVIATOR_GFR_SPLIT='val', AVIATOR_GFR_MODE='structured',
                      AVIATOR_GFR_VISIBLE_ERROR='1')
    for key in ('dt', 'd_safe', 'beta_stop', 'r_max', 'qdot_max'):
        os.environ['AVIATOR_GFR_' + key.upper()] = str(frozen[key])
    import jax
    import mujoco
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    from experiments.aviator_manifold import gfr_env
    from tools.gfr1_teacher_replay_consistency import candidates, choose

    config = TrainConfig()
    env = config.get_environment()
    core = env.unwrapped
    agent = config.make_sac_agent(11, env.observation_space.sample(), env.action_space.sample())
    agent = agent.replace(state=checkpoints.restore_checkpoint(str(checkpoint), agent.state))
    original_project = gfr_env.project_r
    tracer = ProjectTrace(original_project)
    protected = [checkpoint, manifest, reference_path,
                 Path(inspect.getfile(original_project)), Path(inspect.getfile(type(core)))]
    before_hashes = {str(path): digest(path) for path in protected}
    protocol = dict(checkpoint=str(checkpoint), input_hashes=before_hashes, frozen=frozen,
                    episodes=list(range(a.episodes)), teacher_actions=25,
                    teacher_seed=20261101, continuation_steps=a.continuation_steps,
                    continuation_rule='Existing teacher: reuse last accepted raw action; expand to 25 only if it fails.',
                    first_search_rule='25 existing teacher candidates, fixed seed 20261101+episode; max beta then clearance.',
                    recovery_rule='Report accepted steps and exact progress gain. A bounded continuation is not whole-task success.',
                    checker='Unmodified discrete endpoint/swept checker, subdivisions=5; no continuous safety claim.')
    write_json(a.output / 'protocol.json', protocol)
    rows = []
    constraint_rows = []
    with (a.output / 'episodes.jsonl').open('x', buffering=1) as log:
        for episode in range(a.episodes):
            started = time.perf_counter()
            case = a.output / f'episode_{episode:02d}'
            case.mkdir()
            obs, _ = env.reset(seed=20262000 + episode, options={'trajectory_index': episode})
            previous_raw = np.zeros(2)
            budget = math.ceil((len(core._traj['x']) - 1) * core.traj_dt / (core.dt * core.beta_stop)) + 2
            for step in range(budget):
                action = np.asarray(jax.device_get(agent.sample_actions(
                    observations={'state': np.asarray([obs['state']])},
                    seed=jax.random.PRNGKey(500000 + episode * 10000 + step), argmax=True)))[0].astype(np.float32)
                state = snapshot(env, obs, action, previous_raw)
                obs, _, done, truncated, info = env.step(action)
                if done or truncated:
                    break
                previous_raw = action.copy()
            assert info['termination'] == 'governor_exhausted', info['termination']
            assert step + 1 == reference[episode]['steps'], (step + 1, reference[episode]['steps'])
            assert abs(info['progress'] - reference[episode]['progress']) < 1e-12
            restore(env, state)
            # Complete kinematic state plus MuJoCo scratch state and wrapper history.
            state.update(trajectory=copy.deepcopy(core._traj), model=core.model_path,
                         model_sha256=digest(core.model_path), trajectory_path=core._traj_paths[episode],
                         trajectory_sha256=digest(core._traj_paths[episode]), frozen=frozen,
                         simulator={})
            for name in ('fk', 'checker'):
                obj = getattr(core, name)
                spec = mujoco.mjtState.mjSTATE_INTEGRATION
                values = np.empty(mujoco.mj_stateSize(obj.model, spec))
                mujoco.mj_getState(obj.model, obj.data, values, spec)
                state['simulator'][name] = dict(spec=int(spec), values=values)
            with (case / 'pre_stop_state.pkl').open('xb') as f:
                pickle.dump(state, f, protocol=pickle.HIGHEST_PROTOCOL)
            write_json(case / 'pre_stop_state.json', {k: v for k, v in state.items() if k != 'wrapper_history'})

            active_project_records = []
            def instrumented_project(*args, **kwargs):
                result, record = tracer.call(*args, **kwargs)
                active_project_records.append(record)
                return result

            gfr_env.project_r = instrumented_project
            try:
                _, _, pdone, _, pinfo = core.step(action)
                assert pdone and pinfo['termination'] == info['termination']
                assert pinfo['candidate_trace'] == info['candidate_trace'], 'Tracing changed solver output'
                policy_records = copy.deepcopy(active_project_records)
                write_json(case / 'policy_rejection.json', dict(info=pinfo, project_trace=policy_records,
                                                               traced_untraced_identical=True))
                restore(env, state)
                active_project_records.clear()
                searched = []
                original_candidate = core._candidate
                def record_candidate(*args, **kwargs):
                    start = len(active_project_records)
                    result = original_candidate(*args, **kwargs)
                    searched.append(dict(beta=float(args[4]), r_nom=np.asarray(args[3]).copy(),
                                         result=result[0], project_trace=copy.deepcopy(active_project_records[start:])))
                    return result
                core._candidate = record_candidate
                try:
                    pick = choose(core, candidates(np.random.default_rng(20261101 + episode), previous_raw, 25))
                finally:
                    del core._candidate
                write_json(case / 'teacher_search.json', dict(found=pick is not None, candidates=searched,
                                                             selected=None if pick is None else dict(raw=pick[2], result=pick[3])))
            finally:
                gfr_env.project_r = original_project
            assert all(np.array_equal(getattr(core, k), v) for k, v in state['fields'].items())

            for i, (record, candidate) in enumerate(zip(policy_records, pinfo['candidate_trace'])):
                for trial in record['trials']:
                    constraint_rows.append(dict(episode=episode, beta=core.governor_schedule[i], stage='euler',
                                                attempt=trial['attempt'], violated=';'.join(trial['violated']),
                                                d_endpoint_m=trial['d_endpoint_m'], d_swept_m=trial['d_swept_m'],
                                                qdot_max_rad_s=trial['qdot_max_rad_s'],
                                                joint_limit_margin_rad=trial['joint_limit_margin_rad']))
                if record['stage'] == 'linear_slice_empty':
                    constraint_rows.append(dict(episode=episode, beta=core.governor_schedule[i],
                                                stage='linear_slice', attempt=-1,
                                                violated=record['empty_at_constraint']))
                if 'd_endpoint_m' in candidate:
                    reasons = violations(candidate['d_endpoint_m'], candidate['d_swept_m'],
                                         candidate['endpoint_contact'], candidate['swept_contact'],
                                         candidate['qdot_max_rad_s'], candidate['joint_limit_margin_rad'],
                                         core.d_safe, core.qdot_max)
                    constraint_rows.append(dict(episode=episode, beta=core.governor_schedule[i], stage='midpoint',
                                                attempt=0, violated=';'.join(reasons),
                                                **{k: candidate[k] for k in ('d_endpoint_m', 'd_swept_m',
                                                                            'qdot_max_rad_s', 'joint_limit_margin_rad')}))

            progress_before = info['progress']
            continuation = []
            reason = 'teacher_no_candidate_at_stop' if pick is None else 'continuation_budget'
            with (case / 'continuation.jsonl').open('x', buffering=1) as contlog:
                for k in range(a.continuation_steps):
                    if pick is None:
                        break
                    _, _, done, truncated, executed = core.step(
                        pick[2], precomputed={'accepted': pick[3], 'candidates': pick[4]})
                    accepted = dict(step=k + 1, progress=executed['progress'],
                                    beta=executed['beta_executed'], raw_action=pick[2],
                                    executed_r=executed['action_executed_r'],
                                    d_actual_m=executed['d_actual_m'],
                                    closure_residual_inf=executed['closure_residual_inf'],
                                    joint_limit_margin_rad=executed['joint_limit_margin_rad'],
                                    qdot_max_rad_s=executed['qdot_max_rad_s'],
                                    endpoint_validation=executed['endpoint_validation'],
                                    swept_validation=executed['swept_validation'])
                    assert not violations(executed['d_endpoint_m'], executed['d_swept_m'],
                                          executed['endpoint_contact'], executed['swept_contact'],
                                          executed['qdot_max_rad_s'], executed['joint_limit_margin_rad'],
                                          core.d_safe, core.qdot_max)
                    continuation.append(accepted)
                    contlog.write(json.dumps(safe(accepted), allow_nan=False) + '\n')
                    if done or truncated:
                        reason = executed['termination']
                        break
                    if k + 1 < a.continuation_steps:
                        prev = pick[2].copy()
                        pick = choose(core, [prev])
                        if pick is None:
                            pick = choose(core, candidates(np.random.default_rng(20261101 + episode * 10000 + k + 1), prev, 25))
                        if pick is None:
                            reason = 'teacher_search_exhausted_during_continuation'
            progress_after = float(core._u / (len(core._traj['x']) - 1))
            row = dict(episode=episode, model=core.model_path, trajectory=core._traj_paths[episode],
                       policy_steps=step + 1, policy_stop=info['termination'],
                       reproduced_reference=True, traced_untraced_identical=True,
                       policy_project_stages=[r['stage'] for r in policy_records],
                       policy_constraint_counts=dict(Counter(v for r in constraint_rows if r['episode'] == episode
                                                              for v in r['violated'].split(';') if v)),
                       teacher_one_step_witness=bool(continuation),
                       teacher_accepted_steps=len(continuation), teacher_stop=reason,
                       progress_before=progress_before, progress_after=progress_after,
                       progress_gain=progress_after - progress_before,
                       teacher_min_clearance_m=min((c['d_actual_m'] for c in continuation), default=None),
                       bounded_recovery_100_steps=len(continuation) >= 100,
                       completed=reason == 'end', snapshot_sha256=digest(case / 'pre_stop_state.pkl'),
                       elapsed_s=time.perf_counter() - started)
            write_json(case / 'result.json', row)
            rows.append(row)
            log.write(json.dumps(safe(row), allow_nan=False) + '\n')
            print(json.dumps(safe(row), allow_nan=False), flush=True)
    after_hashes = {str(path): digest(path) for path in protected}
    assert after_hashes == before_hashes
    fields = ['episode', 'beta', 'stage', 'attempt', 'violated', 'd_endpoint_m', 'd_swept_m',
              'qdot_max_rad_s', 'joint_limit_margin_rad']
    with (a.output / 'policy_constraints.csv').open('x') as f:
        writer = csv.DictWriter(f, fields)
        writer.writeheader()
        writer.writerows(constraint_rows)
    summary = dict(protocol=protocol, input_hashes_unchanged=True, episodes=rows,
                   one_step_witness_count=sum(r['teacher_one_step_witness'] for r in rows),
                   recovery_100_steps_count=sum(r['bounded_recovery_100_steps'] for r in rows),
                   complete_count=sum(r['completed'] for r in rows),
                   stop_counts=dict(Counter(r['teacher_stop'] for r in rows)))
    write_json(a.output / 'summary.json', summary)


if __name__ == '__main__':
    main()
