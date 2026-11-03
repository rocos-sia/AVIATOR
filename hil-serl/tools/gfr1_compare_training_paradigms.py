#!/usr/bin/env python3
"""Read-only evidence audit of N13R and the October 28 full-teacher GFR run.

Writes a new diagnostic JSON, never changes a training input or checkpoint.
The optional policy probe measures agreement, not action optimality or success.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1'
NEW = ROOT / 'outputs/differential_lift_feasibility_20261028/gfr_complete_teacher_gate'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    finite = a[np.isfinite(a)]
    if not len(finite):
        return {'n': len(a), 'nonfinite': int(len(a))}
    return {'n': len(a), 'nonfinite': int(len(a) - len(finite)),
            'mean': float(finite.mean()), 'min': float(finite.min()),
            'p05': float(np.quantile(finite, .05)), 'median': float(np.median(finite)),
            'p95': float(np.quantile(finite, .95)), 'max': float(finite.max())}


def demo_stats(episodes, is_gfr):
    lengths = [len(ep) for ep in episodes]
    nonterminal_rewards, terminal_rewards, returns, discounted = [], [], [], []
    action_changes = zero_actions = close_boundary = total = corrected = 0
    samples = []
    for ep in episodes:
        reward = np.asarray([t['rewards'] for t in ep], dtype=float)
        dones = np.asarray([t['dones'] for t in ep], dtype=bool)
        assert dones[-1] and not dones[:-1].any()
        nonterminal_rewards.extend(reward[:-1])
        terminal_rewards.append(reward[-1])
        returns.append(float(reward.sum()))
        discounted.append(float(np.dot(reward, .999 ** np.arange(len(ep)))))
        actions = np.asarray([t['actions'] for t in ep])
        zero_actions += int(np.sum(np.max(np.abs(actions), axis=1) < 1e-7))
        action_changes += int(np.sum(np.max(np.abs(np.diff(actions, axis=0)), axis=1) > 1e-6))
        after = np.asarray([t['next_observations']['state'] for t in ep]).reshape(len(ep), 40)
        clearance_col = 30 if is_gfr else 12
        close_boundary += int(np.sum(after[:, clearance_col] <= .00501))
        if is_gfr:
            corrected += int(np.sum((after[:, 36] > .5) & (after[:, 35] > 0) & (after[:, 38] < .5)))
        total += len(ep)
        for idx in np.linspace(0, len(ep) - 1, 8, dtype=int):
            samples.append(ep[idx])
    result = {'episodes': len(episodes), 'transitions': total, 'lengths': stats(lengths),
              'nonterminal_reward': stats(nonterminal_rewards), 'terminal_reward': stats(terminal_rewards),
              'undiscounted_episode_return': stats(returns), 'discounted_return_gamma_0999': stats(discounted),
              'terminal_discount_gamma_0999': stats(.999 ** (np.asarray(lengths) - 1)),
              'demo_terminal_fraction': len(episodes) / total,
              'prob_terminal_in_uniform_128_demo_batch': 1 - (1 - len(episodes) / total) ** 128,
              'zero_raw_action_fraction': zero_actions / total,
              'raw_action_change_fraction_within_episode': action_changes / (total - len(episodes)),
              'fraction_clearance_within_10um_of_threshold': close_boundary / total,
              'correction_supervision_eligible_fraction': corrected / total if is_gfr else None}
    return result, samples


def diagnostic_stats(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    windows = {}
    for lo, hi in [(0, 10000), (10000, 20000), (20000, 30000), (30000, 40000), (40000, 60000)]:
        sub = [r for r in rows if lo < r.get('updates_completed', r['step'] + 1) <= hi]
        if not sub:
            continue
        windows[f'{lo}_{hi}'] = {
            f'{group}.{key}': stats([r['update_info'][group][key] for r in sub if key in r['update_info'][group]])
            for group, keys in [('actor', ['actor_loss', 'correction_loss', 'entropy', 'temperature']),
                                ('critic', ['critic_loss', 'predicted_qs', 'predicted_qs_std', 'rewards'])]
            for key in keys
        }
    return {'records': len(rows), 'windows': windows,
            'demo_buffer_sizes': sorted({r['demo_buffer_size'] for r in rows}),
            'batch_allocations': [json.loads(s) for s in sorted({json.dumps(r['sampling'], sort_keys=True) for r in rows})],
            'last': rows[-1]}


def policy_probe(samples):
    import jax
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    obs = {'state': np.asarray([t['observations']['state'] for t in samples], dtype=np.float32).reshape(-1, 1, 40)}
    actions = np.asarray([t['actions'] for t in samples], dtype=np.float32)
    next_states = np.asarray([t['next_observations']['state'] for t in samples]).reshape(-1, 40)
    config = TrainConfig()
    agent = config.make_sac_agent(11, {'state': np.zeros((1, 40), np.float32)}, np.zeros(2, np.float32))
    result = {}
    for step in (20000, 40000):
        ckpt = NEW / f'formal_seed11/training/b4_seed11/checkpoint_{step}'
        paths = sorted(p for p in ckpt.rglob('*') if p.is_file())
        before = {str(p): sha(p) for p in paths}
        state = checkpoints.restore_checkpoint(str(ckpt), agent.state)
        learner = agent.replace(state=state)
        policy = np.asarray(learner.sample_actions(obs, seed=jax.random.PRNGKey(0), argmax=True))
        teacher_q = np.asarray(learner.forward_critic(obs, actions, rng=None, train=False)).mean(axis=0)
        policy_q = np.asarray(learner.forward_critic(obs, policy, rng=None, train=False)).mean(axis=0)
        agree = np.linalg.norm(policy - actions, axis=1)
        result[str(step)] = {
            'samples': len(samples), 'device': str(jax.devices()),
            'sampling_rule': 'eight evenly spaced indices per teacher episode, including endpoints',
            'raw_policy_minus_teacher_l2': stats(agree),
            'fraction_l2_below_01': float(np.mean(agree < .1)),
            'policy_component_saturation_abs_gt_095': float(np.mean(np.abs(policy) > .95)),
            'critic_policy_minus_teacher_q': stats(policy_q - teacher_q),
            'critic_prefers_policy_fraction': float(np.mean(policy_q > teacher_q)),
            'policy_minus_executed_teacher_label_l2': stats(np.linalg.norm(policy - next_states[:, 33:35] / 1.5, axis=1)),
            'checkpoint_unchanged': before == {str(p): sha(p) for p in paths},
            'limitations': 'Agreement and Q preference are diagnostics; multiple successful actions can exist. No rollout success or action optimality is inferred.',
        }
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--policy-probe', action='store_true')
    a = p.parse_args()
    out = Path(a.output)
    if out.exists():
        raise FileExistsError(out)
    sources = {}
    def load_json(path):
        sources[str(path)] = sha(path)
        return json.loads(path.read_text())
    def load_pickle(path):
        sources[str(path)] = sha(path)
        with path.open('rb') as f:
            return pickle.load(f)
    metadata = load_json(OLD / 'train_seed_1/metadata.json')
    old_episodes = [load_pickle(Path(path)) for path in metadata['demo_paths']]
    old_demo, _ = demo_stats(old_episodes, False)
    del old_episodes
    manifest = load_json(NEW / 'demo_buffer/demo_manifest.json')
    data = load_pickle(NEW / 'demo_buffer/structured_visible.pkl')
    episodes = [data[e['transition_start']:e['transition_start'] + e['transition_count']] for e in manifest['episodes']]
    new_demo, samples = demo_stats(episodes, True)
    del episodes, data
    frozen = load_json(NEW / 'formal_seed11/frozen_config.json')
    report = {'kind': 'gfr_training_paradigm_comparison', 'causality': 'observational audit; causal attribution requires controlled ablation',
              'old_demo': old_demo, 'new_demo': new_demo, 'old_bc_checkpoint': metadata['bc_checkpoint_path'],
              'new_training_command': frozen['training_command'], 'diagnostics': {}}
    for name, path in [('old_seed1', OLD/'train_seed_1/learner_diagnostics.jsonl'),
                       ('new_seed11', NEW/'formal_seed11/training/b4_seed11/diagnostics.jsonl')]:
        sources[str(path)] = sha(path)
        report['diagnostics'][name] = diagnostic_stats(path)
    online_path = NEW / 'formal_seed11/online_episodes.jsonl'
    sources[str(online_path)] = sha(online_path)
    online = [json.loads(l) for l in online_path.read_text().splitlines() if l.strip()]
    report['online_episodes'] = {
        'completed_records': len(online), 'terminations': dict(Counter(r['termination'] for r in online)),
        'progress': stats([r['progress'] for r in online]),
        'by_model': {m: {'n': sum(r['model'] == m for r in online),
                         'progress': stats([r['progress'] for r in online if r['model'] == m])}
                     for m in sorted({r['model'] for r in online})}}
    report['heldout'] = {}
    for step in (20000, 40000):
        result = load_json(NEW / f'formal_seed11/evaluations/checkpoint_{step}.json')
        report['heldout'][str(step)] = {**result['summary'],
            'endpoint_failed_candidates': sum(r['endpoint_failed_candidates'] for r in result['episodes']),
            'swept_failed_candidates': sum(r['swept_failed_candidates'] for r in result['episodes']),
            'max_closure_residual_inf': max(r['max_closure_residual_inf'] for r in result['episodes'])}
    if a.policy_probe:
        report['teacher_state_policy_probe'] = policy_probe(samples)
    report['source_sha256'] = sources
    report['sources_unchanged'] = all(sha(Path(path)) == digest for path, digest in sources.items())
    assert report['sources_unchanged']
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k not in {'source_sha256', 'diagnostics'}}, indent=2))


if __name__ == '__main__':
    main()
