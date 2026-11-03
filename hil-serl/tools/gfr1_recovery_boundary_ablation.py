#!/usr/bin/env python3
"""Frozen 2x2 snapshot ablation; never modifies the production GFR backend.

Alternative actions are searched only after the current policy governor fails.
Boundary correction compensates measured Euler-to-midpoint joint defects in
candidate generation; the original midpoint checker retains the original bounds.
"""
from __future__ import annotations

import argparse
import copy
import csv
import inspect
import json
import os
import pickle
import time
from collections import Counter
from pathlib import Path

import numpy as np

from tools.gfr1_same_state_diagnosis import digest, restore, safe, violations, write_json


def rejection(info, core):
    if info.get('feasible', False):
        return []
    if 'd_endpoint_m' not in info:
        return ['linear_or_euler_rejection']
    return violations(info['d_endpoint_m'], info['d_swept_m'], info['endpoint_contact'],
                      info['swept_contact'], info['qdot_max_rad_s'],
                      info['joint_limit_margin_rad'], core.d_safe, core.qdot_max)


def tightened_bounds(lower, upper, defect, lower_offset, upper_offset):
    """Keep all prior sign-specific measured defect compensation, never relax."""
    lower_offset = np.maximum(lower_offset, np.maximum(-defect, 0.))
    upper_offset = np.maximum(upper_offset, np.maximum(defect, 0.))
    return lower + lower_offset, upper - upper_offset, lower_offset, upper_offset


def evaluate_candidate(core, action, beta, boundary=False, max_corrections=3):
    from experiments.aviator_manifold import gfr_env
    base_project = gfr_env.project_r
    x, q, xd = core._x, core._q, core._path_state(core._u)[1]
    r_nom = np.asarray(action, dtype=float) * core.r_max
    lo = np.concatenate(core.lookup.joint_lower)
    hi = np.concatenate(core.lookup.joint_upper)
    lower_offset, upper_offset = np.zeros_like(q), np.zeros_like(q)
    history = []
    info = None
    for iteration in range((max_corrections if boundary else 0) + 1):
        def compensated_project(*args, **kwargs):
            values = list(args)
            values[7] = lo + lower_offset
            values[8] = hi - upper_offset
            return base_project(*values, **kwargs)
        if iteration:
            gfr_env.project_r = compensated_project
        try:
            info, _ = core._candidate(x, q, xd, r_nom, beta, core._previous_n, structured=True)
        finally:
            gfr_env.project_r = base_project
        reason = rejection(info, core)
        item = dict(iteration=iteration, beta=beta, feasible=bool(info['feasible']),
                    violated=reason, lower_offset=lower_offset.copy(), upper_offset=upper_offset.copy(),
                    result=copy.deepcopy(info))
        if 'q_next' in info:
            _, n, he, hx, _ = core._lift(x, q, xd, core._previous_n)
            qe = q + core.dt * (he + beta*hx + n @ np.asarray(info['r_exec']))
            defect = np.asarray(info['q_next']) - qe
            item.update(q_euler=qe, midpoint_minus_euler=defect,
                        euler_joint_margin=float(min(np.min(qe-lo), np.min(hi-qe))))
        history.append(item)
        if info['feasible'] or not boundary or reason != ['joint_limit'] or iteration == max_corrections:
            break
        _, _, lower_offset, upper_offset = tightened_bounds(lo, hi, defect, lower_offset, upper_offset)
    return info, history


def select_action(core, raw, previous_raw, boundary, alternative, rng):
    from tools.gfr1_teacher_replay_consistency import candidates
    history = []

    def attempt(action, source):
        for beta in core.governor_schedule:
            info, trials = evaluate_candidate(core, action, beta, boundary)
            history.append(dict(source=source, raw=np.asarray(action).copy(), beta=beta, trials=trials))
            if info['feasible']:
                return np.asarray(action, dtype=np.float32), info
        return None

    chosen = attempt(raw, 'policy')
    policy_accepted = chosen is not None
    if chosen is None and alternative:
        for action in candidates(rng, previous_raw, 25):
            pick = attempt(action, 'alternative')
            if pick is not None:
                chosen = pick
                break
    return chosen, history, not policy_accepted


def independent_check(core, old_x, old_q, info):
    """Recheck the chosen state without trusting cached acceptance flags."""
    from tools.validate_branch_connector import swept_clearance
    d, contact = core.checker.check(info['x_next'], info['q_next'])
    swept, swept_contact = swept_clearance(core.checker, old_x, old_q,
                                           info['x_next'], info['q_next'], subdivisions=5)
    lo, hi = np.concatenate(core.lookup.joint_lower), np.concatenate(core.lookup.joint_upper)
    qn = np.asarray(info['q_next'])
    speed = float(np.max(np.abs((qn-old_q)/core.dt)))
    margin = float(min(np.min(qn-lo), np.min(hi-qn)))
    failed = violations(d, swept, contact, swept_contact, speed, margin, core.d_safe, core.qdot_max)
    assert not failed, failed
    return dict(endpoint_m=d, swept_m=swept, margin=margin, speed=speed, violations=failed)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--snapshots', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--steps', type=int, default=100)
    p.add_argument('--episodes', type=int, default=10)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reference = json.loads((args.snapshots/'summary.json').read_text())
    frozen = reference['protocol']['frozen']
    checkpoint = Path(reference['protocol']['checkpoint'])
    os.environ.update(JAX_PLATFORMS='cpu', AVIATOR_GFR_MODEL_MANIFEST=frozen['eval_manifest'],
                      AVIATOR_GFR_SPLIT='val', AVIATOR_GFR_MODE='structured', AVIATOR_GFR_VISIBLE_ERROR='1')
    for key in ('dt', 'd_safe', 'beta_stop', 'r_max', 'qdot_max'):
        os.environ['AVIATOR_GFR_'+key.upper()] = str(frozen[key])
    import jax
    import mujoco
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    config = TrainConfig()
    env = config.get_environment()
    core = env.unwrapped
    agent = config.make_sac_agent(11, env.observation_space.sample(), env.action_space.sample())
    agent = agent.replace(state=checkpoints.restore_checkpoint(str(checkpoint), agent.state))
    groups = [('original',False,False), ('alternative_only',True,False),
              ('boundary_only',False,True), ('combined',True,True)]
    protected = [args.snapshots, checkpoint, Path(inspect.getfile(type(core))),
                 Path('hil-serl/tools/validate_branch_connector.py'), Path(frozen['eval_manifest'])]
    hashes = {str(path):digest(path) for path in protected}
    write_json(args.output/'protocol.json', dict(
        source=str(args.snapshots), frozen=frozen, input_hashes=hashes,
        groups=groups, episodes=list(range(args.episodes)), accepted_step_budget=args.steps,
        alternative_candidate_count=25, alternative_seed_formula='20261102+episode*10000+cycle',
        alternative_selection='Only on policy exhaustion; previous accepted nominal action, zero, signed axes/corners, then frozen random candidates; first feasible action in this order, original beta schedule within each action.',
        boundary_correction='Only on joint-limit-only midpoint failures: tighten Euler candidate bounds by measured sign-specific midpoint-minus-Euler defect, monotonically, at most 3 corrections. Original final limits unchanged.',
        policy='40k deterministic policy re-evaluated at every accepted state; no teacher takeover',
        execution_feedback='env.step receives original policy raw action and chosen checked result so correction remains observable',
        interpretation='Snapshot regression, not whole-task success or unbiased heldout generalization; no training.'))
    rows = []
    with (args.output/'episodes.jsonl').open('x',buffering=1) as log:
        for episode in range(args.episodes):
            source = args.snapshots/f'episode_{episode:02d}'
            with (source/'pre_stop_state.pkl').open('rb') as f:
                state = pickle.load(f)
            assert digest(source/'pre_stop_state.pkl') == reference['episodes'][episode]['snapshot_sha256']
            assert digest(state['model']) == state['model_sha256']
            assert digest(state['trajectory_path']) == state['trajectory_sha256']
            for name, alternative, boundary in groups:
                started = time.perf_counter()
                env.reset(seed=20262000+episode, options={'trajectory_index':episode})
                core._traj = copy.deepcopy(state['trajectory'])
                restore(env,state)
                for objname, value in state['simulator'].items():
                    obj = getattr(core,objname)
                    mujoco.mj_setState(obj.model,obj.data,np.asarray(value['values']),value['spec'])
                    mujoco.mj_forward(obj.model,obj.data)
                obs = copy.deepcopy(state['observation'])
                previous_raw = state['previous_raw'].copy()
                case = args.output/f'episode_{episode:02d}_{name}'
                case.mkdir()
                progress0 = core._u/(len(core._traj['x'])-1)
                accepted_steps = fallback_steps = correction_steps = retries = 0
                evaluations = 0
                minimum_clearance = float('inf')
                maximum_closure = 0.
                cycle_times = []
                stop = 'diagnostic_budget'
                first_accepted = False
                with (case/'trace.jsonl').open('x',buffering=1) as trace:
                    for cycle in range(args.steps):
                        t0 = time.perf_counter()
                        raw = np.asarray(jax.device_get(agent.sample_actions(
                            {'state':np.asarray([obs['state']])},
                            seed=jax.random.PRNGKey(800000+episode*10000+cycle),argmax=True)))[0].astype(np.float32)
                        if cycle == 0:
                            np.testing.assert_array_equal(raw,state['action'])
                        chosen, history, policy_exhausted = select_action(
                            core,raw,previous_raw,boundary,alternative,
                            np.random.default_rng(20261102+episode*10000+cycle))
                        evaluations += sum(len(h['trials']) for h in history)
                        corrections = sum(max(0,len(h['trials'])-1) for h in history)
                        retries += corrections
                        if cycle == 0:
                            write_json(case/'first_step.json',dict(policy_raw=raw,history=history,
                                      chosen=None if chosen is None else dict(raw=chosen[0],result=chosen[1])))
                        if chosen is None:
                            stop='candidate_exhausted'
                            trace.write(json.dumps(safe(dict(cycle=cycle,accepted=False,policy_raw=raw,
                                progress=core._u/(len(core._traj['x'])-1),history=history)),allow_nan=False)+'\n')
                            break
                        if cycle == 0:
                            first_accepted=True
                        selected_raw, info = chosen
                        independent = independent_check(core,core._x.copy(),core._q.copy(),info)
                        # Preserve original raw policy action; do not conceal override in observation/reward.
                        all_candidates=[h['trials'][-1]['result'] for h in history]
                        obs,_,done,truncated,executed=env.step(raw,{'accepted':info,'candidates':all_candidates})
                        accepted_steps+=1
                        fallback_steps+=int(policy_exhausted)
                        correction_steps+=int(corrections>0)
                        previous_raw=selected_raw.copy()
                        minimum_clearance=min(minimum_clearance,info['d_actual_m'])
                        maximum_closure=max(maximum_closure,info['closure_residual_inf'])
                        elapsed=time.perf_counter()-t0
                        cycle_times.append(elapsed)
                        trace.write(json.dumps(safe(dict(cycle=cycle,accepted=True,
                            policy_raw=raw,selected_raw=selected_raw,executed_r=info['r_exec'],
                            beta=info['beta'],progress=executed['progress'],
                            d_actual_m=info['d_actual_m'],joint_limit_margin_rad=info['joint_limit_margin_rad'],
                            closure_residual_inf=info['closure_residual_inf'],
                            policy_exhausted=policy_exhausted,correction_iterations=corrections,
                            candidate_evaluations=sum(len(h['trials']) for h in history),
                            independent_check=independent,cycle_elapsed_s=elapsed,
                            history=history if policy_exhausted or corrections else None)),allow_nan=False)+'\n')
                        if done or truncated:
                            stop=executed['termination']
                            break
                progress1=core._u/(len(core._traj['x'])-1)
                row=dict(episode=episode,method=name,first_step_accepted=first_accepted,
                    accepted_steps=accepted_steps,progress_before=progress0,progress_after=progress1,
                    progress_gain=progress1-progress0,stop_reason=stop,
                    alternative_recovery_steps=fallback_steps,boundary_correction_steps=correction_steps,
                    correction_iterations=retries,candidate_evaluations=evaluations,
                    min_clearance_m=minimum_clearance if accepted_steps else None,
                    max_closure_residual_inf=maximum_closure if accepted_steps else None,
                    accepted_validation_violations=0,
                    mean_diagnostic_cycle_s=float(np.mean(cycle_times)) if cycle_times else None,
                    elapsed_s=time.perf_counter()-started)
                write_json(case/'result.json',row)
                rows.append(row)
                log.write(json.dumps(safe(row),allow_nan=False)+'\n')
                print(json.dumps(safe(row),allow_nan=False),flush=True)
    assert hashes == {str(p):digest(p) for p in protected}
    with (args.output/'comparison.csv').open('x') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    summary={}
    for name,_,_ in groups:
        rr=[r for r in rows if r['method']==name]
        summary[name]=dict(cases=len(rr),first_step_accepted=sum(r['first_step_accepted'] for r in rr),
            recovered_100_steps=sum(r['accepted_steps']>=100 for r in rr),
            mean_progress_gain=float(np.mean([r['progress_gain'] for r in rr])),
            total_alternative_steps=sum(r['alternative_recovery_steps'] for r in rr),
            total_boundary_correction_steps=sum(r['boundary_correction_steps'] for r in rr),
            candidate_evaluations=sum(r['candidate_evaluations'] for r in rr),
            stops=dict(Counter(r['stop_reason'] for r in rr)))
    write_json(args.output/'summary.json',dict(groups=summary,episodes=rows,
        source_hashes_unchanged=True,accepted_validation_violations=0,
        whole_task_success_not_evaluated=True,training_started=False))


if __name__=='__main__':
    main()
