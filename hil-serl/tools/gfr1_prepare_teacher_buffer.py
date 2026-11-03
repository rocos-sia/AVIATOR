#!/usr/bin/env python3
"""Merge complete, model-consistent teacher episodes without altering sources."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def prepare(source: Path, output: Path):
    output.mkdir(parents=True, exist_ok=False)
    sources = [(source / f'retry_shard_{i}/teacher_replay.json',
                source / f'retry_shard_{i}/demo_buffer.pkl') for i in range(3)]
    sources += [(source / f'repair/replacement_{i}_replay.json',
                 source / f'repair/replacement_{i}_demo.pkl') for i in range(2)]
    merged, episodes, records, excluded, hashes = [], [], [], [], {}
    keys = set()
    minimum = float('inf')
    for report_path, buffer_path in sources:
        report = json.loads(report_path.read_text())
        assert report['summary']['instrumentation_unknown'] == 0
        manifest_path = Path(report['manifest'])
        manifest = json.loads(manifest_path.read_text())
        for path in (report_path, buffer_path, manifest_path):
            hashes[str(path)] = sha256(path)
        with buffer_path.open('rb') as stream:
            transitions = pickle.load(stream)
        assert len(transitions) == len(report['traces'])
        for ep in report['episodes']:
            rec = manifest['records'][ep['trajectory_index']]
            start, count = ep['trace_start'], ep['trace_count']
            if not ep['complete'] or ep['termination'] != 'end':
                excluded.append({'source': str(report_path), 'episode': ep['episode'],
                                 'termination': ep['termination'], 'record': rec,
                                 'excluded_transitions': count})
                continue
            key = (rec['model'], rec['trajectory_path'])
            assert key not in keys, f'duplicate teacher task: {key}'
            keys.add(key)
            assert rec['model'] == ep['model']
            assert count == ep['accepted_steps'] and count > 0
            assert abs(ep['progress'] - 1.) < 1e-7
            selected = transitions[start:start + count]
            trace = report['traces'][start:start + count]
            assert len(selected) == count and trace[-1]['termination'] == 'end'
            for i, (tr, logged) in enumerate(zip(selected, trace)):
                before = np.asarray(tr['observations']['state'])
                after = np.asarray(tr['next_observations']['state'])
                assert before.shape == after.shape == (40,)
                for value in (before, after, tr['actions'], tr['rewards'], tr['masks']):
                    assert np.isfinite(value).all()
                assert np.shape(tr['actions']) == (2,)
                assert np.max(np.abs(tr['actions'])) <= 1.
                terminal = i == count - 1
                assert bool(tr['dones']) == terminal
                assert float(tr['masks']) == float(not terminal)
                assert logged['episode'] == ep['episode'] and logged['step'] == i
                np.testing.assert_array_equal(tr['actions'], np.asarray(logged['action'], np.float32))
                assert logged['d_actual_m'] >= report['frozen']['d_safe'] - 1e-9
                minimum = min(minimum, logged['d_actual_m'])
                np.testing.assert_allclose(after[-1], logged['progress'], atol=1e-7, rtol=0)
                if i:
                    np.testing.assert_array_equal(before, selected[i - 1]['next_observations']['state'])
                else:
                    assert before[-1] == 0.
                    np.testing.assert_allclose(before[4:18], rec['initial_q'], atol=2e-7, rtol=0)
                if terminal:
                    assert abs(float(after[-1]) - 1.) < 1e-7
            episodes.append({'episode': len(episodes), 'source': str(report_path),
                             'source_episode': ep['episode'], 'model': rec['model'],
                             'trajectory_path': rec['trajectory_path'],
                             'transition_start': len(merged), 'transition_count': count,
                             'termination': 'end', 'progress': 1.,
                             'min_clearance_m': ep['min_clearance_m']})
            # Keep the environment's original rewards, actions and terminal flags.
            # The learner's replay insertion accepts these unchunked 40-D arrays.
            merged.extend(selected)
            records.append(rec)
    assert len(episodes) == 100 and len(excluded) == 2
    assert len(merged) == 302125
    counts = Counter(rec['model'] for rec in records)
    assert sorted(counts.values()) == [50, 50]
    for rec in records:
        for path in (rec['model'], rec['trajectory_path']):
            if path not in hashes:
                hashes[path] = sha256(path)
    buffer_path = output / 'structured_visible.pkl'
    with buffer_path.open('xb') as stream:
        pickle.dump(merged, stream, protocol=pickle.HIGHEST_PROTOCOL)
    train_manifest = {'kind': 'gfr1_success_only_training_manifest', 'split': 'rl_train',
                      'models': sorted(counts), 'records': records,
                      'trajectory_count': len({r['trajectory_path'] for r in records}),
                      'record_count': len(records)}
    (output / 'train_manifest.json').write_text(json.dumps(train_manifest, indent=2) + '\n')
    result = {'kind': 'gfr1_complete_teacher_buffer_audit', 'passed': True,
              'episode_count': len(episodes), 'transition_count': len(merged),
              'model_episode_counts': dict(counts), 'instrumentation_unknown': 0,
              'minimum_logged_clearance_m': minimum, 'episodes': episodes,
              'excluded': excluded, 'source_sha256': hashes,
              'buffer_path': str(buffer_path), 'buffer_sha256': sha256(buffer_path),
              'train_manifest_sha256': sha256(output / 'train_manifest.json'),
              'checks': ['full_task_end', 'terminal_masks', 'finite_values',
                         'state_continuity', 'model_consistent_initial_q',
                         'action_matches_accepted_teacher_trace', 'accepted_step_clearance'],
              'safety_provenance': 'Teacher saved only feasible endpoint/swept-accepted candidates; merge verifies their recorded min(endpoint,swept). This is not a new replay of all 302125 steps.',
              'frozen': {'dt': .005, 'd_safe': .005, 'beta_stop': .125,
                         'r_max': 1.5, 'demo_fraction': .5}}
    (output / 'demo_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('passed', 'episode_count', 'transition_count',
                                             'model_episode_counts', 'buffer_sha256')}), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source.resolve(), args.output.resolve())
