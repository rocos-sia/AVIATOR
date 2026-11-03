#!/usr/bin/env python3
"""Isolate previous-executed-action sensitivity after a failed BC gate.

The recursive observation probe is synthetic: it does not execute the robot
or certify full-system instability. It isolates one policy input channel.
"""
import argparse
import json
import os
from pathlib import Path
import pickle

import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bc-root', type=Path, required=True)
    p.add_argument('--demo-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    os.environ['JAX_PLATFORMS'] = 'cpu'
    import jax
    import jax.numpy as jnp
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig
    from tools.gfr1_bc_support import checkpoint_hash
    from tools.gfr1_prepare_teacher_buffer import sha256
    audit = json.loads((a.demo_dir / 'demo_manifest.json').read_text())
    assert sha256(a.demo_dir / 'structured_visible.pkl') == audit['buffer_sha256']
    with (a.demo_dir / 'structured_visible.pkl').open('rb') as f:
        data = pickle.load(f)
    ep = audit['episodes'][0]
    seq = data[ep['transition_start']:ep['transition_start'] + 26]
    observations = np.asarray([t['observations']['state'] for t in seq], np.float32)
    labels = np.asarray([t['actions'] for t in seq])
    assert np.max(np.abs(labels)) == 0.
    results = {}
    config = TrainConfig()
    for step in (10000, 20000):
        checkpoint = (a.bc_root / f'bc_seed11/checkpoint_{step}').resolve()
        before = checkpoint_hash(checkpoint)
        agent = config.make_bc_agent(11, {'state': np.zeros((1, 40), np.float32)}, np.zeros(2, np.float32))
        agent = agent.replace(state=checkpoints.restore_checkpoint(str(checkpoint), agent.state))
        def policy(obs):
            return agent.sample_actions({'state': obs[None, None, :]}, argmax=True)[0]
        teacher_predictions = np.asarray(jax.vmap(policy)(jnp.asarray(observations)))
        recursive_predictions = []
        previous_raw = np.zeros(2, np.float32)
        for obs in observations:
            counterfactual = obs.copy()
            counterfactual[33:35] = 1.5 * previous_raw
            previous_raw = np.asarray(policy(jnp.asarray(counterfactual)))
            recursive_predictions.append(previous_raw.tolist())
        # Inference can stop encoder gradients. Measure actual input sensitivity
        # with finite differences instead of differentiating that training graph.
        gains = {}
        for epsilon in (1e-3, 1e-4):
            columns = []
            for channel in (33, 34):
                plus, minus = observations[0].copy(), observations[0].copy()
                plus[channel] += epsilon
                minus[channel] -= epsilon
                columns.append((np.asarray(policy(jnp.asarray(plus))) -
                                np.asarray(policy(jnp.asarray(minus)))) / (2 * epsilon))
            gains[str(epsilon)] = (np.stack(columns, axis=1) * 1.5).tolist()
        gain = np.asarray(gains['0.001'])
        results[str(step)] = {
            'checkpoint_sha256': before,
            'checkpoint_unchanged': before == checkpoint_hash(checkpoint),
            'teacher_conditioned_raw_predictions': teacher_predictions.tolist(),
            'recursive_previous_r_only_predictions': recursive_predictions,
            'teacher_conditioned_last_action_l2': float(np.linalg.norm(teacher_predictions[-1])),
            'recursive_previous_r_last_action_l2': float(np.linalg.norm(recursive_predictions[-1])),
            'local_raw_action_feedback_jacobian': gain.tolist(),
            'jacobian_method': 'central finite difference, observation-space epsilon=0.001',
            'finite_difference_sensitivity_check': gains,
            'local_feedback_spectral_radius': float(np.max(np.abs(np.linalg.eigvals(gain)))),
            'local_feedback_operator_norm': float(np.linalg.norm(gain, 2)),
        }
        assert results[str(step)]['checkpoint_unchanged']
    result = {'kind': 'bc_previous_action_channel_probe', 'version': 2,
              'supersedes': 'bc_feedback_probe.json Jacobian fields only: automatic differentiation through inference does not measure input sensitivity when encoder gradients are stopped.',
              'model': ep['model'],
              'trajectory_path': ep['trajectory_path'], 'steps': 26,
              'teacher_raw_actions_all_zero': True,
              'intervention': 'Replace only observation[33:35] by 1.5 times the preceding policy raw output; keep all other fields at recorded teacher values.',
              'limitation': 'Synthetic input-channel probe and local Jacobian, not a physical rollout or global stability theorem.',
              'results': results}
    with a.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    for step, row in results.items():
        print(step, {k: v for k, v in row.items() if not k.endswith('predictions')})


if __name__ == '__main__':
    main()
