#!/usr/bin/env python3
"""Audit and compare original versus masked-previous-r BC, without rollouts."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    protocol = json.loads((root / 'ablation_protocol.json').read_text())
    base, variant = Path(protocol['baseline']), root / 'formal_seed11'
    changed = [p for p, digest in protocol['protected_sha256'].items() if sha256(p) != digest]
    if changed:
        raise RuntimeError(f'protected files changed: {changed}')
    assert json.loads((base / 'teacher_covered_eval_manifest.json').read_text()) == json.loads(
        (variant / 'teacher_covered_eval_manifest.json').read_text())
    a = json.loads((base / 'frozen_protocol.json').read_text())
    b = json.loads((variant / 'frozen_protocol.json').read_text())
    for key in ('seed', 'demo_episodes', 'demo_transitions', 'bc_steps', 'bc_batch_size',
                'bc_demo_fraction', 'bc_loss', 'bc_target', 'learning_rate', 'eval_steps',
                'eval_episodes_per_checkpoint', 'gate', 'sac_frozen', 'safety_reward_backend_unchanged'):
        assert a[key] == b[key], key
    assert b['mask_previous_action'] and b['masked_indices'] == [33, 34]
    assert not (variant / 'sac_from_bc').exists()
    rows, paired, examples = [], [], {}
    reports = {}
    for name, directory in (('original', base), ('mask_previous_r', variant)):
        qualification = json.loads((directory / 'bc_qualification.json').read_text())
        assert qualification['sources_unchanged']
        reports[name] = qualification
        for step in (10000, 20000):
            report = qualification['evaluations'][str(step)]
            assert report['returncode'] == 0 and report['instrumentation_unknown'] == 0
            assert bool(report.get('mask_previous_action', False)) == (name == 'mask_previous_r')
            episodes = report['episodes']
            row = {'method': name, 'checkpoint': step,
                   'completed': sum(e['completed'] for e in episodes), 'episodes': len(episodes),
                   'mean_progress': float(np.mean([e['progress'] for e in episodes])),
                   'mean_interventions': float(np.mean([e['intervention_steps'] for e in episodes])),
                   'endpoint_failed_candidates': sum(e['endpoint_failed_candidates'] for e in episodes),
                   'swept_failed_candidates': sum(e['swept_failed_candidates'] for e in episodes),
                   'accepted_validation_violations': sum(e['accepted_validation_violations'] for e in episodes),
                   'mean_min_clearance_m': float(np.mean([e['minimum_executed_clearance_m'] for e in episodes])),
                   'max_closure_residual_inf': max(e['max_closure_residual_inf'] for e in episodes)}
            rows.append(row)
            first_episode = []
            trace_count = 0
            with (directory / f'evaluations/checkpoint_{step}.steps.jsonl').open() as stream:
                for line in stream:
                    trace = json.loads(line)
                    trace_count += 1
                    if name == 'mask_previous_r':
                        assert trace['previous_executed_r_policy_input'] == [0., 0.]
                    if trace['episode'] == 0 and (trace['step'] in (1, 5, 10, 20, 26) or trace['termination'] not in (None, 'none', '')):
                        first_episode.append(trace)
            assert trace_count == sum(e['steps'] for e in episodes)
            examples[f'{name}_{step}'] = first_episode
    for step in (10000, 20000):
        old = reports['original']['evaluations'][str(step)]['episodes']
        new = reports['mask_previous_r']['evaluations'][str(step)]['episodes']
        for x, y in zip(old, new):
            assert (x['model'], x['trajectory_path'], x['step_budget']) == (y['model'], y['trajectory_path'], y['step_budget'])
            paired.append({'checkpoint': step, 'episode': y['episode'], 'model': y['model'],
                           'trajectory_path': y['trajectory_path'],
                           'original_completed': x['completed'], 'masked_completed': y['completed'],
                           'original_progress': x['progress'], 'masked_progress': y['progress'],
                           'progress_difference': y['progress'] - x['progress'],
                           'original_stop': x['termination'], 'masked_stop': y['termination']})
    for filename, records in (('comparison.csv', rows), ('paired_episodes.csv', paired)):
        with (root / filename).open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    result = {'kind': 'bc_previous_r_observation_ablation', 'protected_files_unchanged': True,
              'masked_inputs_verified_every_evaluation_step': True,
              'gate_decision': reports['mask_previous_r']['decision'], 'sac_started': False,
              'comparison': rows, 'paired_episodes': paired, 'first_episode_examples': examples,
              'scope': 'Single seed, ten teacher-covered tasks, two checkpoints. Absolute intervention counts depend on distance travelled; no generalization or global safety inference.',
              'reconstruction': {name: value['training']['reconstruction'] for name, value in reports.items()}}
    with (root / 'ablation_summary.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({'gate': result['gate_decision'], 'comparison': rows}, indent=2))


if __name__ == '__main__':
    main()
