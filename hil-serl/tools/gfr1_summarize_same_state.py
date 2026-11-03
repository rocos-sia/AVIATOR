#!/usr/bin/env python3
"""Summarize exact-state witnesses without relabeling bounded recovery as success."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np

from tools.gfr1_same_state_diagnosis import digest, write_json


def write_csv(path, rows):
    with path.open('x') as f:
        fields = list(dict.fromkeys(k for row in rows for k in row))
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('directory', type=Path)
    a = p.parse_args()
    root = a.directory
    result = json.loads((root / 'summary.json').read_text())
    frozen = result['protocol']['frozen']
    dt, d_safe, qdot_max = frozen['dt'], frozen['d_safe'], frozen['qdot_max']
    comparison, trials = [], []
    unknown = 0
    for row in result['episodes']:
        case = root / f"episode_{row['episode']:02d}"
        state = json.loads((case / 'pre_stop_state.json').read_text())
        policy = json.loads((case / 'policy_rejection.json').read_text())
        teacher = json.loads((case / 'teacher_search.json').read_text())
        continuation = [json.loads(line) for line in (case / 'continuation.jsonl').read_text().splitlines()]
        assert digest(case / 'pre_stop_state.pkl') == row['snapshot_sha256']
        selected = teacher['selected']
        comparison.append(dict(episode=row['episode'], model=Path(row['model']).name,
                               trajectory=Path(row['trajectory']).name,
                               progress_before=row['progress_before'],
                               progress_after=row['progress_after'], progress_gain=row['progress_gain'],
                               teacher_accepted_steps=row['teacher_accepted_steps'],
                               teacher_stop=row['teacher_stop'],
                               policy_raw=json.dumps(state['action']),
                               teacher_raw=None if selected is None else json.dumps(selected['raw']),
                               teacher_beta=None if selected is None else selected['result']['beta'],
                               teacher_endpoint_m=None if selected is None else selected['result']['d_endpoint_m'],
                               teacher_swept_m=None if selected is None else selected['result']['d_swept_m'],
                               teacher_feasible_action_count=sum(c['result']['feasible'] for c in teacher['candidates']),
                               policy_constraints=json.dumps(row['policy_constraint_counts']),
                               teacher_last_clearance_m=continuation[-1]['d_actual_m'] if continuation else None,
                               teacher_mean_beta=float(np.mean([c['beta'] for c in continuation])) if continuation else None,
                               closure_before_inf=float(np.max(np.abs(state['observation']['state'][0][18:30])))))
        if 'q_next' in policy['info']:
            # Recover the exact configured joint bounds from the recorded
            # linear planes; no new model assumptions or relaxed tolerances.
            planes = policy['project_trace'][-1]['planes']
            q = np.asarray(state['fields']['_q'])
            qnext = np.asarray(policy['info']['q_next'])
            h = np.array([qdot_max - planes[4*j]['b'] for j in range(14)])
            lo = q - dt*h - np.array([planes[4*j+3]['b'] for j in range(14)])
            hi = q + dt*h + np.array([planes[4*j+2]['b'] for j in range(14)])
            comparison[-1]['last_rejected_joint_names'] = ';'.join(
                f"{'L' if j < 7 else 'R'}{j % 7 + 1}_{'upper' if qnext[j] > hi[j] else 'lower'}"
                for j in range(14) if min(qnext[j]-lo[j], hi[j]-qnext[j]) < -1e-9)
        for beta, trace in zip(result['protocol']['frozen'].get('governor_schedule', [1,.75,.5,.25,.125]),
                               policy['project_trace']):
            plane = trace['planes'][-1]
            for trial in trace['trials']:
                linear = d_safe + plane['b'] - np.dot(plane['a'], trial['r'])
                actual = min(trial['d_endpoint_m'], trial['d_swept_m'])
                max_violation = max(np.dot(p['a'], trial['r']) - p['b'] for p in trace['planes'])
                trials.append(dict(episode=row['episode'], beta=beta, attempt=trial['attempt'],
                                   d0_m=trace['d0_m'], d_linear_m=linear, d_actual_m=actual,
                                   delta_m=actual-linear, maximum_halfspace_violation=max_violation,
                                   endpoint_m=trial['d_endpoint_m'], swept_m=trial['d_swept_m'],
                                   qdot_max=trial['qdot_max_rad_s'], joint_margin=trial['joint_limit_margin_rad'],
                                   violated=';'.join(trial['violated'])))
            if not trace['feasible'] and not trace['trials'] and trace['stage'] != 'linear_slice_empty':
                unknown += 1
    # Only observations; do not infer universal infeasibility from a finite search.
    write_csv(root / 'same_state_comparison.csv', comparison)
    write_csv(root / 'policy_linear_vs_actual.csv', trials)
    write_json(root / 'diagnostic_analysis.json', dict(
        episodes=len(comparison), instrumentation_unknown=unknown,
        teacher_one_step_witnesses=result['one_step_witness_count'],
        teacher_100_step_recoveries=result['recovery_100_steps_count'],
        full_task_completions=result['complete_count'],
        policy_stop_constraint_episode_counts=dict(Counter(k for r in result['episodes']
                                                          for k in r['policy_constraint_counts'])),
        first_projection_trials=sum(t['attempt'] == 0 for t in trials),
        first_projection_optimistic=sum(t['attempt'] == 0 and t['d_linear_m'] >= d_safe - 1e-9
                                        and t['d_actual_m'] < d_safe - 1e-9 for t in trials),
        shrunken_trials=sum(t['attempt'] > 0 for t in trials),
        shrunken_linear_clearance_violations=sum(t['attempt'] > 0 and t['d_linear_m'] < d_safe - 1e-9 for t in trials),
        input_hashes_unchanged=result['input_hashes_unchanged'],
        limitations=[
            'Ten paired heldout failure states, not independent training seeds.',
            'Teacher search cost exceeds one policy inference; this is a feasibility witness, not a latency comparison.',
            '100 accepted steps demonstrate bounded local progress only.',
            'No teacher candidate does not prove that no feasible path exists.',
            'The swept checker is discretely sampled, not a continuous-time collision certificate.',
            'These witnesses do not by themselves identify the training-level cause of policy error.',
            'Heldout recovery labels must not be used for training while retaining the same cases as an untouched test set.',
        ]))


if __name__ == '__main__':
    main()
