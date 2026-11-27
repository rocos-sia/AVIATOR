"""Independent, no-training foresight qualification with a frozen CUDA Actor.

The score is an executor-conditioned candidate-evaluation count, NOT a SAC Q
target or a millisecond speed claim. All accepted steps receive an independent
geometric audit. Existing experiment sources and outputs are read-only inputs.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import time
import traceback

import numpy as np
import torch

from tools.audit_static_collision import CollisionChecker
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import (
    EvaluationEnv, ROOT, RULES, audit_motion, dump, sha, snapshot, restore,
)
from tools.gfr1_scheduler_benchmark import Meter, instrumentation
from tools.gfr2_shared_env import SharedProposalEnv
from tools.gfr3_demo_transitions import unpack_state
from tools.gfr3_sac import GaussianActor
from tools.gfr4_common import require_memory_limit

HORIZONS = {'one_step': .005, 'h005': .05, 'h010': .1,
            'h025': .25, 'h050': .5, 'h100': 1.}
EXTRA_STATE = ('_repair_episode', 'minimum_chart_component', '_last_lift_basis',
               'last_policy_observation')


class ProtectedSharedEnv(SharedProposalEnv):
    # Explicitly bypass DirectOnlyEnv.action_step/step. Preserve its 128-input
    # representation and arm-local coordinates, but use the real protected path.
    action_step = EvaluationEnv.action_step

    def step(self, action):
        return self.action_step(action, True)[0]


def make_env(record):
    data = ROOT/'hil-serl/data/aviator'
    env = ProtectedSharedEnv(str(data/'manifold_phi_stale_pitch-10deg'),
                             record['model'], str(data/'trajectory_source'))
    env._model_records = [record]
    env.reset(seed=11, options={'trajectory_index': 0})
    return env


def freeze(env):
    return dict(fields={**snapshot(env), **{k: copy.deepcopy(getattr(env, k)) for k in EXTRA_STATE}},
                trajectory=copy.deepcopy(env._traj), model=env.model_path,
                rng=copy.deepcopy(env.np_random.bit_generator.state))


def thaw(env, state):
    assert env.model_path == state['model'], 'Cross-model snapshot restore'
    restore(env, state['fields'])
    env._traj = copy.deepcopy(state['trajectory'])
    env.np_random.bit_generator.state = copy.deepcopy(state['rng'])
    before = state['fields']['last_policy_observation']
    np.testing.assert_allclose(env._get_obs(), before, atol=1e-6, rtol=0)


class FrozenActor:
    def __init__(self, checkpoint):
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA required; no CPU inference fallback')
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        self.device = torch.device('cuda:0')
        saved = torch.load(checkpoint, map_location=self.device, weights_only=True)
        self.actor = GaussianActor(np.zeros(128), np.ones(128)).to(self.device)
        self.actor.load_state_dict(saved['actor'])
        self.actor.eval().requires_grad_(False)
        self.original = {k: v.clone() for k, v in self.actor.state_dict().items()}

    @torch.inference_mode()
    def __call__(self, obs):
        return self.actor.sample(torch.as_tensor(obs, dtype=torch.float32,
                                 device=self.device), True)[0].cpu().numpy()

    def unchanged(self):
        return all(torch.equal(v, self.original[k]) for k, v in self.actor.state_dict().items())


def action_set(obs, actor):
    # Baseline first gives deterministic baseline-preferred tie-breaking.
    proposals = [('previous', np.clip(obs[33:35], -1, 1)), ('actor', actor(obs))]
    proposals += [(f'grid_{i}_{j}', np.array([i, j], np.float32))
                  for i in (-1., 0., 1.) for j in (-1., 0., 1.)]
    result = []
    for name, action in proposals:
        action = np.asarray(action, np.float32)
        # Keep alias names, but don't confuse duplicate actions with coverage.
        result.append(dict(name=name, action=action.tolist()))
    return result


def reversal_indices(velocity):
    """Detect direction changes across a zero-speed plateau as well as a sign crossing."""
    events = set()
    for dim in range(velocity.shape[1]):
        nonzero = np.flatnonzero(np.abs(velocity[:, dim]) > 1e-8)
        for left, right in zip(nonzero[:-1], nonzero[1:]):
            if velocity[left, dim]*velocity[right, dim] < 0:
                events.add(int(right))
    return np.asarray(sorted(events), dtype=int)


def event_horizon(env, target_u):
    changes = reversal_indices(env._traj['xdot'])
    future = changes[changes > env._u+1e-8]
    if not len(future):
        return None
    endpoint = float(future[0])+.15/env.traj_dt
    if endpoint > target_u-1e-8:
        return None  # Don't call a clipped pre-event window "next event".
    return (endpoint-env._u)*env.traj_dt


def rollout(env, saved, actor, action, target_u, audit, budget=None):
    thaw(env, saved)
    u0 = env._u
    if budget is None:
        budget = int(np.ceil(2*(target_u-u0)*env.traj_dt/env.dt))+1
    rows = []
    meter = Meter()
    reason = 'progress_budget_exhausted'
    # Counters are scoped to this serial rollout; audit wall time is excluded
    # from each control-step time. Instrumentation itself has overhead.
    with instrumentation(env, meter):
        for step in range(budget):
            x0, q0 = env._x.copy(), env._q.copy()
            counts = meter.counts.copy()
            started = time.perf_counter_ns()
            raw = np.asarray(action, np.float32) if step == 0 else actor(env.last_policy_observation)
            result, direct, first = env.action_step(raw, True)
            elapsed = (time.perf_counter_ns()-started)/1e6
            _, _, terminated, truncated, info = result
            accepted = info['beta_executed'] > 0
            verdict = audit_motion(env, audit, x0, q0) if accepted else None
            violations = verdict['violations'] if verdict else []
            row = dict(step=step, u=float(env._u), q=env._q.tolist(), x=env._x.tolist(),
                       n=None if env._previous_n is None else env._previous_n.tolist(),
                       raw=raw.tolist(), executed=info['action_executed_r'],
                       beta=float(info['beta_executed']), direct=bool(direct),
                       candidate_calls=meter.counts['candidate']-counts['candidate'],
                       projection_calls=meter.counts['projection']-counts['projection'],
                       recovery=bool(info.get('alternative_recovered', False)),
                       boundary_iterations=int(info.get('boundary_correction_iterations', 0)),
                       termination=info['termination'], elapsed_ms=elapsed,
                       clearance_m=None if verdict is None else min(verdict['endpoint_clearance_m'], verdict['swept_clearance_m']),
                       joint_margin_rad=None if verdict is None else verdict['joint_margin_rad'],
                       audit=verdict,
                       violations=violations)
            rows.append(row)
            if violations:
                raise AssertionError(f'Accepted-motion audit failed: {violations}')
            if env._u >= target_u-1e-9:
                reason = 'target_reached'
                break
            if terminated or truncated:
                reason = info['termination']
                break
    return dict(completed=reason == 'target_reached', termination=reason,
                start_u=u0, target_u=target_u, budget=budget, steps=len(rows),
                candidate_calls=sum(r['candidate_calls'] for r in rows),
                projection_calls=sum(r['projection_calls'] for r in rows),
                fallback_steps=sum(not r['direct'] for r in rows),
                slowdown_deficit_s=sum(1-r['beta'] for r in rows)*env.dt,
                all_direct=all(r['direct'] for r in rows),
                total_compute_ms=sum(r['elapsed_ms'] for r in rows), rows=rows)


def deterministic_view(result):
    result = copy.deepcopy(result)
    result.pop('total_compute_ms')
    for row in result['rows']:
        row.pop('elapsed_ms')
    return result


def compare_replay(a, b):
    # Exact equality is intentional within one fixed numeric runtime. Times
    # aren't deterministic; neither are they used to rank qualification labels.
    from tools.gfr1_same_state_diagnosis import safe
    if safe(deterministic_view(a)) != safe(deterministic_view(b)):
        raise AssertionError('D0 deterministic state/action/event replay mismatch')


def prefix_cost(result, endpoint):
    rows = result['rows']
    indices = [i for i, row in enumerate(rows) if row['u'] >= endpoint-1e-9]
    if indices:
        prefix = rows[:indices[0]+1]
        return (0, sum(r['candidate_calls'] for r in prefix), len(prefix))
    # Failed/censored candidates cannot beat candidates reaching the endpoint.
    return (1, 0, 0)


def analyze_state(results, horizons, u0, traj_dt):
    long_cost = [(0, r['candidate_calls'], r['steps']) if r['completed'] else (1, 0, 0)
                 for r in results]
    good = [i for i, r in enumerate(results) if r['completed']]
    best = min(good, key=lambda i: long_cost[i]) if good else None
    selections = {}
    for name, seconds in horizons.items():
        costs = [prefix_cost(r, u0+seconds/traj_dt) for r in results]
        available = [i for i, cost in enumerate(costs) if cost[0] == 0]
        # No fake teacher label when all candidates fail to reach this horizon.
        selected = min(available, key=lambda i: costs[i]) if available else None
        delta = None if best is None or not results[0]['completed'] else long_cost[0][1]-long_cost[best][1]
        selections[name] = dict(index=selected,
            completed=None if selected is None else results[selected]['completed'],
            cost=None if selected is None or not results[selected]['completed'] else long_cost[selected][1],
            regret=None if selected is None or best is None or not results[selected]['completed'] else long_cost[selected][1]-long_cost[best][1],
            eta=None if delta is None or delta < 2 or selected is None or not results[selected]['completed'] else (long_cost[0][1]-long_cost[selected][1])/delta)
    future = [r['candidate_calls']-r['rows'][0]['candidate_calls'] for r in results if r['completed']]
    first_q = np.asarray([r['rows'][0]['q'] for r in results])
    last_q = np.asarray([r['rows'][-1]['q'] for r in results if r['completed']])
    influence = {}
    for seconds in (.005, .05, .1, .25, .5, 1.):
        frame = int(round(seconds/.005))-1
        active = [r['rows'][frame] for r in results if len(r['rows']) > frame]
        if active:
            influence[str(seconds)] = dict(active_candidates=len(active),
                q_range_rad=float(np.max(np.ptp(np.asarray([r['q'] for r in active]), axis=0))),
                progress_index_range=float(np.ptp([r['u'] for r in active])))
    return dict(completed_candidates=len(good), all_failed=not good,
                baseline_completed=results[0]['completed'], best_index=best,
                baseline_cost=results[0]['candidate_calls'] if results[0]['completed'] else None,
                available_gain=None if best is None or not results[0]['completed'] else long_cost[0][1]-long_cost[best][1],
                future_call_range=max(future)-min(future) if future else 0,
                first_q_range_rad=float(np.max(np.ptp(first_q, axis=0))),
                final_q_range_rad=float(np.max(np.ptp(last_q, axis=0))) if len(last_q) else None,
                first_executed_action_range=np.ptp(np.asarray([r['rows'][0]['executed'] for r in results]), axis=0).tolist(),
                first_direct_count=sum(r['rows'][0]['direct'] for r in results),
                influence_at_same_elapsed_time=influence,
                protected_completion_count=len(good), all_direct_to_target_count=sum(r['completed'] and r['all_direct'] for r in results),
                selections=selections)


def pick_anchors(data):
    obs, states = data['observations'], data['states']
    # At least 1.3 seconds of nominal future in every common-target experiment.
    eligible = np.flatnonzero((states[-1, 0]-states[:, 0])*.01 >= 1.3)
    if len(eligible) < 5:
        raise ValueError('Insufficient nominal future for qualification')
    signs = reversal_indices(obs[:, 2:4])
    reversal = int(max(signs[0]-80, 0)) if len(signs) else None
    ranking = [('regular', sorted(eligible, key=lambda i: abs(i-int(.2*len(obs))))),
               ('joint_approach', sorted(eligible, key=lambda i: obs[i,31])),
               ('clearance_approach', sorted(eligible, key=lambda i: obs[i,30])),
               ('pre_reversal' if reversal is not None else 'mid_motion',
                sorted(eligible, key=lambda i: abs(i-(reversal if reversal is not None else int(.4*len(obs)))))),
               ('coordination_change', sorted(eligible, key=lambda i: -float(np.linalg.norm(data['actions'][i]-obs[i,33:35]))))]
    chosen = []
    for label, indices in ranking:
        frame = next((int(i) for i in indices if all(abs(i-j) >= 20 for _, j in chosen)), None)
        if frame is None:
            raise ValueError('Insufficient distinct scene anchors')
        chosen.append((label, frame))
    return chosen


def prepare(a):
    records = json.loads((a.source/'dataset/train.json').read_text())['records']
    jobs, inputs = [], {}
    for record in records:
        installation, trajectory = record['case'].split('/')
        if installation not in ('train_00', 'train_01', 'train_06', 'train_07'):
            continue
        split = 'development' if installation in ('train_00', 'train_01') and trajectory == 'traj_0000' else 'qualification_holdout'
        file = a.artifacts/'demonstrations'/f"{record['case'].replace('/', '__')}.npz"
        inputs[str(file)] = sha(file)
        for key in ('model', 'true_model', 'trajectory_path'):
            inputs[record[key]] = sha(record[key])
        with np.load(file) as data:
            for scene, frame in pick_anchors(data):
                jobs.append(dict(record=record, scene=scene, frame=frame, split=split,
                                 state=data['states'][frame].tolist(), observation=data['observations'][frame].tolist()))
    jobs.sort(key=lambda j: (j['split'] != 'development', j['record']['case'], j['frame']))
    for i, job in enumerate(jobs):
        job['id'] = f's{i:03d}'
    return jobs, inputs


def d0(a, jobs, actor):
    checks = []
    for job in jobs[:1] if a.smoke else [jobs[0], next(j for j in jobs if j['scene'] == 'joint_approach'), next(j for j in jobs if j['scene'] == 'clearance_approach')]:
        env = make_env(job['record']); other = make_env(job['record'])
        audit = CollisionChecker(job['record']['true_model'])
        audit_other = CollisionChecker(job['record']['true_model'])
        try:
            np.testing.assert_allclose(unpack_state(env, job['state']), job['observation'], atol=1e-5, rtol=0)
            saved = freeze(env)
            proposals = action_set(env.last_policy_observation, actor)
            selected = [proposals[i]['action'] for i in (0, 2, -1)]
            target = env._u+.05/env.traj_dt
            reference = [rollout(env, saved, actor, raw, target, audit) for raw in selected]
            for i in reversed(range(len(selected))):
                repeat = rollout(env, saved, actor, selected[i], target, audit)
                compare_replay(reference[i], repeat)
                independent = rollout(other, saved, actor, selected[i], target, audit_other)
                compare_replay(reference[i], independent)
            checks.append(dict(state=job['id'], case=job['record']['case'], passed=True,
                               rollouts=9, fallback_steps=sum(r['fallback_steps'] for r in reference),
                               steps=sum(r['steps'] for r in reference)))
            dump(a.output/'d0.json', dict(passed=True, checks=checks,
                scope='same-model implementation reproducibility, not physical/model-error validation'))
        finally:
            env.close(); other.close()
    if not any(c['fallback_steps'] for c in checks):
        dump(a.output/'d0.json', dict(passed=False, checks=checks, reason='missing fallback coverage'))
        raise RuntimeError('D0 missing fallback coverage; no qualification run allowed')
    return checks


def aggregate(rows):
    informative = [r for r in rows if (r['analysis']['available_gain'] or 0) >= 2]
    future = [r for r in rows if r['analysis']['future_call_range'] >= 2 and r['analysis']['first_q_range_rad'] > 1e-8]
    table = {}
    for h in (*HORIZONS, 'next_event'):
        pairs = []
        losses = gains = recovered = regressions = 0
        previous_pairs = []
        previous_gains = previous_regressions = previous_recoveries = 0
        for row in rows:
            s = row['analysis']['selections']; baseline = s['one_step']; selected = s.get(h)
            if selected is None:
                continue
            previous_ok = row['analysis']['baseline_completed']
            if previous_ok and not selected['completed']:
                previous_regressions += 1
            elif selected['completed'] and not previous_ok:
                previous_recoveries += 1
            elif previous_ok and selected['completed']:
                gain = row['analysis']['baseline_cost']-selected['cost']
                previous_pairs.append(gain)
                previous_gains += gain >= 2
            if baseline['completed'] and not selected['completed']:
                regressions += 1
            elif selected['completed'] and not baseline['completed']:
                recovered += 1
            elif baseline['completed'] and selected['completed']:
                gain = baseline['cost']-selected['cost']; pairs.append(gain)
                gains += gain >= 2; losses += gain <= -2
        table[h] = dict(pairs=len(pairs), net_candidate_calls_saved=sum(pairs),
                        gain_states=gains, loss_states=losses,
                        completion_regressions=regressions, completion_recoveries=recovered,
                        previous_action_gain_states=previous_gains,
                        net_calls_saved_vs_previous=sum(previous_pairs),
                        previous_action_completion_regressions=previous_regressions,
                        previous_action_completion_recoveries=previous_recoveries)
    valid = [h for h, v in table.items() if h != 'one_step' and v['completion_regressions'] == 0
             and ((v['gain_states'] >= 3 and v['gain_states'] > v['loss_states'] and v['net_candidate_calls_saved'] >= 6)
                  or v['completion_recoveries'] >= 3)
             and v['previous_action_completion_regressions'] == 0
             and ((v['previous_action_gain_states'] >= 3 and v['net_calls_saved_vs_previous'] >= 6)
                  or v['previous_action_completion_recoveries'] >= 3)]
    return dict(states=len(rows), informative_states=len(informative), future_influence_states=len(future),
                action_gate=len(future) >= 3,
                foresight_candidates=valid, horizon_comparisons=table)


def reference_pair(env):
    """Identical .35 s prefix, then smooth C2 continue/reverse references.

    A quintic joins current reference p/v/a at the branch to constant +/- v.
    It is an explicit reference intervention, not an alternative installation.
    """
    original = env._traj
    branch = int(np.ceil(env._u+.35/env.traj_dt))
    duration = .35
    end = branch+int(round(duration/env.traj_dt))
    if end+2 >= len(original['x']):
        raise ValueError('Insufficient reference for counterfactual')
    p0, v0 = original['x'][branch].copy(), original['xdot'][branch].copy()
    a0 = original.get('xddot', np.zeros_like(original['x']))[branch].copy()
    dim = int(np.argmax(np.abs(v0)/np.array([1., .1])))
    matrix = np.array([[duration**3, duration**4, duration**5],
                       [3*duration**2, 4*duration**3, 5*duration**4],
                       [6*duration, 12*duration**2, 20*duration**3]])
    variants = []
    for reverse in (False, True):
        target_v = v0.copy()
        if reverse:
            target_v[dim] *= -1
        target_p = p0+.5*(v0+target_v)*duration
        high = np.linalg.solve(matrix, np.stack((target_p-p0-v0*duration-.5*a0*duration**2,
                                                  target_v-v0-a0*duration, -a0)))
        trajectory = copy.deepcopy(original)
        if 'xddot' not in trajectory:
            trajectory['xddot'] = np.zeros_like(trajectory['x'])
        for index in range(branch, len(trajectory['x'])):
            t = (index-branch)*env.traj_dt
            if t <= duration+1e-10:
                c3, c4, c5 = high
                trajectory['x'][index] = p0+v0*t+.5*a0*t*t+c3*t**3+c4*t**4+c5*t**5
                trajectory['xdot'][index] = v0+a0*t+3*c3*t*t+4*c4*t**3+5*c5*t**4
                trajectory['xddot'][index] = a0+6*c3*t+12*c4*t*t+20*c5*t**3
            else:
                trajectory['x'][index] = target_p+target_v*(t-duration)
                trajectory['xdot'][index] = target_v
                trajectory['xddot'][index] = 0
        variants.append(trajectory)
    return variants, dict(branch_u=branch, reverse_dimension=dim, shared_prefix_s=(branch-env._u)*env.traj_dt)


def observation_test(a, jobs, actor, summaries):
    # Preselect by actual decision headroom, not by an observed context conflict.
    eligible = [r['id'] for r in summaries if r['analysis']['future_call_range'] >= 2][:4]
    results = []
    for job in (j for j in jobs if j['id'] in eligible):
        env = make_env(job['record']); audit = CollisionChecker(job['record']['true_model'])
        try:
            unpack_state(env, job['state']); saved = freeze(env)
            variants, metadata = reference_pair(env)
            proposals = action_set(env.last_policy_observation, actor)
            groups, observations, event_features = [], [], []
            for vi, trajectory in enumerate(variants):
                np.savez_compressed(a.output/'context'/f"{job['id']}_{vi}_reference.npz",
                                    **{k: trajectory[k] for k in ('x', 'xdot', 'xddot')})
                thaw(env, saved); env._traj = trajectory
                observations.append(env._get_obs().copy())
                local = freeze(env)
                target = env._u+1.25/env.traj_dt
                event_features.append(dict(next_event_s=event_horizon(env, target),
                                           velocity_at_08s=env._path_state(env._u+.8/env.traj_dt)[1].tolist()))
                records = [rollout(env, local, actor, p['action'], target, audit) for p in proposals]
                dump(a.output/'context'/f"{job['id']}_{vi}.json", dict(meta=metadata, candidates=proposals, rollouts=records))
                success = [i for i, r in enumerate(records) if r['completed']]
                minimum = min((records[i]['candidate_calls'] for i in success), default=None)
                near = [] if minimum is None else [i for i in success if records[i]['candidate_calls'] <= minimum+2]
                groups.append(near)
            same = bool(np.allclose(observations[0], observations[1], atol=1e-6, rtol=0))
            both = bool(groups[0] and groups[1])
            results.append(dict(state=job['id'], same_current_and_existing_03s_preview=same,
                                event_features=event_features, near_optimal_sets=groups,
                                common_actions=sorted(set(groups[0]) & set(groups[1])),
                                context_conflict=bool(same and both and not(set(groups[0]) & set(groups[1])))))
            dump(a.output/'n0_b.json', dict(results=results,
                 interpretation='Can falsify information sufficiency; cannot prove learnability without training'))
        finally:
            env.close()
    return results


def run(a):
    require_memory_limit()
    a.output.mkdir(parents=True, exist_ok=False)
    for name in ('rollouts', 'context'):
        (a.output/name).mkdir()
    started = time.monotonic()
    jobs, inputs = prepare(a)
    inputs[str(a.checkpoint)] = sha(a.checkpoint)
    # Freeze all local Python sources that could affect the executor, not just
    # this new script. If another chat edits one, stop before using mixed code.
    sources = {str(p.resolve()): sha(p) for directory in ('hil-serl/tools', 'hil-serl/examples/experiments/aviator_manifold')
               for p in (ROOT/directory).glob('*.py')}
    if a.smoke:
        jobs = jobs[:1]
    protocol = dict(actor_checkpoint=str(a.checkpoint), actor_sha256=sha(a.checkpoint),
        actor_device='cuda:0', actor_deterministic=True, actor_updates=0, critic_used=False,
        integration_dt_s=.005, first_action_duration_s=.005, continuation='frozen Actor every 5 ms + direct-first combined executor',
        common_target_nominal_progress_s=1.25, progress_budget_ratio=2., horizons_nominal_progress_s=HORIZONS,
        next_event='first reference velocity sign reversal + .15 nominal seconds when inside common target; otherwise .5 nominal seconds',
        progress='monotone accepted reference index _u, not steering angle or wall-clock time',
        score='feasibility first, then actual candidate evaluations, then steps; baseline-first ties',
        cost_is_milliseconds=False, timing='instrumented, concurrent, diagnostic only; no latency qualification',
        short_pruning=False, candidates='previous + frozen actor + 3x3 normalized grid; duplicate aliases retained',
        rules=RULES, action_gate='at least 3 states with >=2 future candidate-call spread and >1e-8 rad first-state spread',
        foresight_gate='Relative to BOTH one-step and previous-action proposals: >=3 gains of >=2 calls, net >=6 calls, no completion regression; OR >=3 completion recoveries and no regression. Also more gains than losses vs one-step.',
        data_scope='40 selected states; 10 development, 30 qualification-heldout. Frozen actor pretraining saw these installs/tasks: NOT unseen-policy generalization.',
        deployment_qualified=False, calibration_residual_tested=False, smoke=a.smoke,
        inputs=inputs, sources=sources, no_training_even_if_passed=True)
    dump(a.output/'protocol.json', protocol)
    dump(a.output/'states.json', jobs)
    actor = FrozenActor(a.checkpoint)
    dump(a.output/'status.json', dict(phase='d0_replay', gpu=torch.cuda.get_device_name(), actor_device=str(next(actor.actor.parameters()).device)))
    with use_projector('combined'):
        d0(a, jobs, actor)
        summaries = []
        chosen_horizon = None
        for index, job in enumerate(jobs):
            if not a.smoke and index == 10:
                development = aggregate(summaries)
                dump(a.output/'development_gate.json', development)
                if not development['action_gate'] or not development['foresight_candidates']:
                    phase = 'stopped_action_gate' if not development['action_gate'] else 'stopped_foresight_gate'
                    dump(a.output/'status.json', dict(phase=phase, execution_completed=True, actor_trained=False, summary=development))
                    break
                chosen_horizon = development['foresight_candidates'][0]
                dump(a.output/'frozen_horizon.json', dict(selected=chosen_horizon, selected_using='10 development states only'))
            if not all(sha(p) == h for p, h in sources.items()):
                raise RuntimeError('Source changed by another writer; refuse mixed-version experiment')
            env = make_env(job['record']); audit = CollisionChecker(job['record']['true_model'])
            try:
                obs = unpack_state(env, job['state'])
                np.testing.assert_allclose(obs, job['observation'], atol=1e-5, rtol=0)
                saved = freeze(env); u0 = env._u
                duration = .05 if a.smoke else 1.25
                target = u0+duration/env.traj_dt
                horizons = {k: v for k, v in HORIZONS.items() if v < duration+1e-9}
                event = event_horizon(env, target)
                horizons['next_event'] = event if event is not None else min(.5, duration)
                proposals = action_set(obs, actor)
                records = []
                for ci, proposal in enumerate(proposals):
                    dump(a.output/'status.json', dict(phase='n0_a_c_rollouts', state=job['id'], case=job['record']['case'],
                         scene=job['scene'], split=job['split'], candidate=ci, candidates=len(proposals),
                         completed_states=index, total_states=len(jobs), elapsed_s=time.monotonic()-started,
                         gpu=str(next(actor.actor.parameters()).device), actor_trained=False))
                    records.append(rollout(env, saved, actor, proposal['action'], target, audit))
                    dump(a.output/'rollouts'/f"{job['id']}_{ci:02d}.json", dict(state=job['id'], proposal=proposal, result=records[-1]))
                analysis = analyze_state(records, horizons, u0, env.traj_dt)
                summary = dict(id=job['id'], case=job['record']['case'], scene=job['scene'], split=job['split'],
                               horizon_endpoints_nominal_s=horizons, known_event_in_window=event is not None, analysis=analysis)
                summaries.append(summary)
                dump(a.output/'results.json', summaries)
                print(json.dumps(dict(state=job['id'], analysis=analysis)), flush=True)
            finally:
                env.close()
        else:
            if a.smoke:
                dump(a.output/'status.json', dict(phase='smoke_completed', execution_completed=True, qualified=False, actor_trained=False))
            else:
                held = aggregate([r for r in summaries if r['split'] == 'qualification_holdout'])
                selected = held['horizon_comparisons'][chosen_horizon]
                held_passed = held['action_gate'] and chosen_horizon in held['foresight_candidates']
                dump(a.output/'holdout_gate.json', dict(passed=held_passed, frozen_horizon=chosen_horizon, selected=selected, summary=held))
                contexts = []
                if held_passed:
                    dump(a.output/'status.json', dict(phase='n0_b_context_counterfactual', actor_trained=False))
                    contexts = observation_test(a, jobs, actor, summaries)
                dump(a.output/'status.json', dict(phase='qualification_completed' if held_passed else 'stopped_holdout_gate',
                     execution_completed=True, model_headroom_qualified=held_passed,
                     observation_conflicts=sum(r['context_conflict'] for r in contexts),
                     actor_trained=False, deployment_qualified=False))
    unchanged = actor.unchanged() and all(sha(p) == h for p, h in inputs.items()) and all(sha(p) == h for p, h in sources.items())
    if not unchanged:
        raise RuntimeError('Frozen input or Actor mutation detected')
    dump(a.output/'completion.json', dict(execution_completed=True, frozen_inputs_unchanged=unchanged,
         actor_updates=0, elapsed_s=time.monotonic()-started, gpu=torch.cuda.get_device_name(),
         peak_cuda_bytes=torch.cuda.max_memory_allocated(), status=json.loads((a.output/'status.json').read_text())))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source', type=Path, default=ROOT/'outputs/gfr_shared_distill_gpu_20261126')
    p.add_argument('--artifacts', type=Path, default=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126')
    p.add_argument('--checkpoint', type=Path, default=ROOT/'outputs/gfr_sac_demo_paired_gpu_20261126/bc/sac_060000.pt')
    p.add_argument('--smoke', action='store_true')
    a = p.parse_args()
    for name in ('output', 'source', 'artifacts', 'checkpoint'):
        setattr(a, name, getattr(a, name).resolve())
    if a.output.exists():
        p.error('Output already exists; never overwrite an earlier experiment')
    try:
        run(a)
    except Exception as exc:
        if a.output.exists():
            dump(a.output/'failure.json', dict(error=repr(exc), traceback=traceback.format_exc()))
            dump(a.output/'status.json', dict(phase='stopped_error', execution_completed=False, actor_trained=False, error=repr(exc)))
        raise


if __name__ == '__main__':
    main()
