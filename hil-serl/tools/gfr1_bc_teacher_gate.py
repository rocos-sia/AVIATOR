#!/usr/bin/env python3
"""Learn the audited GFR teacher before permitting a fresh SAC experiment.

Uses the existing GFR actor factory and BC likelihood loss. The optional
previous-action input ablation leaves reward, governor and checker unchanged.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pickle
import subprocess
import sys
import time

import numpy as np

from tools.gfr1_bc_support import checkpoint_hash, qualify_bc, select_covered_records, policy_state
from tools.gfr1_gate_support import atomic_json
from tools.gfr1_prepare_teacher_buffer import sha256

ROOT = Path(__file__).resolve().parents[2]
HIL = ROOT / 'hil-serl'


def train_bc(demo, out, seed, smoke=False, mask_previous_action=False):
    import jax
    from flax.training import checkpoints
    from experiments.aviator_gfr.config import TrainConfig

    cfg = TrainConfig()
    with (demo / 'structured_visible.pkl').open('rb') as stream:
        transitions = pickle.load(stream)
    obs = np.asarray([t['observations']['state'] for t in transitions], np.float32).reshape(-1, 1, 40)
    nxt = np.asarray([t['next_observations']['state'] for t in transitions], np.float32).reshape(-1, 1, 40)
    actions = np.asarray([t['actions'] for t in transitions], np.float32)
    assert len(transitions) == 302125 and all(np.isfinite(x).all() for x in (obs, nxt, actions))
    assert actions.shape == (302125, 2) and np.max(np.abs(actions)) <= 1.
    obs = policy_state(obs, mask_previous_action)
    nxt = policy_state(nxt, mask_previous_action)
    audit = json.loads((demo / 'demo_manifest.json').read_text())
    probe_indices = np.concatenate([e['transition_start'] + np.linspace(0, e['transition_count'] - 1, 8, dtype=int)
                                    for e in audit['episodes']])
    del transitions
    probe = {'observations': {'state': obs[probe_indices]}, 'actions': actions[probe_indices]}
    arrays = jax.device_put({'observations': {'state': obs}, 'next_observations': {'state': nxt}, 'actions': actions})
    del obs, nxt, actions
    agent = cfg.make_bc_agent(seed, {'state': np.zeros((1, 40), np.float32)}, np.zeros(2, np.float32))
    train_dir = out / 'bc_seed11'
    train_dir.mkdir()
    steps = 256 if smoke else 20000
    save_steps = {128, 256} if smoke else {5000, 10000, 15000, 20000}
    rng = np.random.default_rng(seed)
    started = time.perf_counter()
    def reconstruction(step):
        metrics = jax.device_get(agent.get_debug_metrics(probe))
        if not all(np.isfinite(v).all() for v in metrics.values()):
            raise RuntimeError('non-finite BC reconstruction metrics')
        l2 = np.sqrt(metrics['mse'])
        return {'updates_completed': step, 'mean_raw_action_l2': float(l2.mean()),
                'p95_raw_action_l2': float(np.quantile(l2, .95)),
                'fraction_raw_action_l2_below_01': float(np.mean(l2 < .1)),
                'nll': float(-metrics['log_probs'].mean()),
                'samples': len(l2), 'scope': 'in-sample reconstruction, not held-out generalization'}
    with (out / 'bc_diagnostics.jsonl').open('x', buffering=1) as log:
        log.write(json.dumps(reconstruction(0)) + '\n')
        for step in range(1, steps + 1):
            indices = jax.device_put(rng.integers(0, 302125, 256, dtype=np.int32))
            batch = jax.tree_util.tree_map(lambda x: x[indices], arrays)
            agent, info = agent.update(batch)
            if step % 100 == 0 or step in save_steps:
                metrics = {k: float(v) for k, v in jax.device_get(info).items()}
                if not all(np.isfinite(v) for v in metrics.values()):
                    raise RuntimeError('non-finite BC update')
                row = {'updates_completed': step, 'elapsed_s': time.perf_counter() - started,
                       'batch_size': 256, 'demo_fraction': 1., **metrics}
                if step in save_steps:
                    row['reconstruction'] = reconstruction(step)
                log.write(json.dumps(row, allow_nan=False) + '\n')
                atomic_json(out / 'status.json', {'phase': 'bc_training', 'pid': os.getpid(), **row})
                if step % 1000 == 0 or step in save_steps:
                    print(json.dumps(row), flush=True)
            if step in save_steps:
                checkpoints.save_checkpoint(str(train_dir), agent.state, step=step, keep=10, overwrite=False)
    checkpoint = train_dir / f'checkpoint_{steps}'
    restored = cfg.make_bc_agent(seed, {'state': np.zeros((1, 40), np.float32)}, np.zeros(2, np.float32))
    restored = restored.replace(state=checkpoints.restore_checkpoint(str(checkpoint), restored.state))
    sac = cfg.make_sac_agent(seed, {'state': np.zeros((1, 40), np.float32)}, np.zeros(2, np.float32))
    critic_before = jax.device_get(sac.state.params['modules_critic'])
    params = dict(sac.state.params)
    params['modules_actor'] = restored.state.params['modules_actor']
    sac = sac.replace(state=sac.state.replace(params=params))
    bc_actions = np.asarray(restored.sample_actions(probe['observations'], argmax=True))
    sac_actions = np.asarray(sac.sample_actions(probe['observations'], argmax=True))
    max_diff = float(np.max(np.abs(bc_actions - sac_actions)))
    critic_unchanged = all(np.array_equal(a, b) for a, b in zip(
        jax.tree_util.tree_leaves(critic_before), jax.tree_util.tree_leaves(jax.device_get(sac.state.params['modules_critic']))))
    assert max_diff <= 1e-6 and critic_unchanged
    result = {'checkpoint': str(checkpoint), 'checkpoint_sha256': checkpoint_hash(checkpoint),
              'updates_completed': steps, 'elapsed_s': time.perf_counter() - started,
              'device': str(jax.devices()), 'bc_to_sac_action_max_abs_difference': max_diff,
              'critic_unchanged_during_actor_transfer': critic_unchanged,
              'mask_previous_action': mask_previous_action,
              'reconstruction': reconstruction(steps)}
    atomic_json(out / 'bc_training_summary.json', result)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--demo-dir', type=Path, required=True)
    p.add_argument('--heldout-manifest', type=Path, required=True)
    p.add_argument('--smoke', action='store_true')
    p.add_argument('--continue-sac-if-qualified', action='store_true')
    p.add_argument('--mask-previous-action', action='store_true')
    a = p.parse_args()
    if a.mask_previous_action and a.continue_sac_if_qualified:
        p.error('masked-observation ablation is BC-only; SAC needs a separately matched observation pipeline')
    out, demo = a.output.resolve(), a.demo_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for key in list(os.environ):
        if key.startswith(('AVIATOR_', 'JAX_', 'XLA_')):
            del os.environ[key]
    os.environ.update(JAX_PLATFORMS='cuda', CUDA_VISIBLE_DEVICES='0',
                      XLA_PYTHON_CLIENT_PREALLOCATE='false',
                      AVIATOR_GFR_MODE='structured', AVIATOR_GFR_VISIBLE_ERROR='1',
                      AVIATOR_CRITIC_CLIP_NORM='1.0')
    audit = json.loads((demo / 'demo_manifest.json').read_text())
    assert audit['passed'] and audit['episode_count'] == 100 and audit['transition_count'] == 302125
    assert audit['instrumentation_unknown'] == 0
    assert sha256(demo / 'structured_visible.pkl') == audit['buffer_sha256']
    assert sha256(demo / 'train_manifest.json') == audit['train_manifest_sha256']
    train_manifest = json.loads((demo / 'train_manifest.json').read_text())
    records = select_covered_records(train_manifest['records'])
    manifest = out / 'teacher_covered_eval_manifest.json'
    atomic_json(manifest, {'kind': 'teacher_covered_imitation_reconstruction',
                          'records': records, 'models': sorted({r['model'] for r in records})})
    source_paths = [demo / 'structured_visible.pkl', demo / 'demo_manifest.json', demo / 'train_manifest.json',
                    Path(__file__), HIL / 'tools/gfr1_eval.py', HIL / 'tools/gfr1_bc_support.py',
                    HIL / 'examples/experiments/aviator_manifold/gfr_env.py',
                    HIL / 'examples/experiments/aviator_gfr/config.py',
                    HIL / 'examples/experiments/aviator_manifold/config.py',
                    HIL / 'serl_launcher/serl_launcher/agents/continuous/bc.py']
    hashes = {str(path): sha256(path) for path in source_paths}
    frozen = {'kind': 'gfr_bc_teacher_gate_protocol', 'seed': 11, 'demo_episodes': 100,
              'demo_transitions': 302125, 'bc_steps': 256 if a.smoke else 20000,
              'bc_batch_size': 256, 'bc_demo_fraction': 1., 'bc_loss': 'existing tanh-Gaussian negative log likelihood',
              'bc_target': 'stored raw teacher action; no executed-action relabelling',
              'learning_rate': .0003, 'observations': '40-D GFR observations; optional zero mask recorded below; no LUT weights',
              'mask_previous_action': a.mask_previous_action,
              'masked_indices': [33, 34] if a.mask_previous_action else [],
              'checkpoint_selection': 'final 20k; 10k is a stability check, not best-checkpoint selection',
              'eval_steps': [] if a.smoke else [10000, 20000], 'eval_episodes_per_checkpoint': 10,
              'eval_manifest': str(manifest), 'gate': 'Both checkpoints >=9/10; each model >=4/5; zero accepted-check violations and instrumentation errors.',
              'evidence_scope': 'teacher-covered imitation execution, not generalization or convergence proof',
              'sac_only_if_gate_passes': a.continue_sac_if_qualified,
              'sac_frozen': {'demo_fraction': .5, 'critic_clip': 1., 'correction_weight': .1,
                             'gate_steps': [20000, 40000], 'max_steps': 60000, 'num_envs': 16},
              'safety_reward_backend_unchanged': True, 'source_sha256': hashes}
    atomic_json(out / 'frozen_protocol.json', frozen)
    try:
        training = train_bc(demo, out, 11, a.smoke, a.mask_previous_action)
        reports = {}
        eval_dir = out / 'evaluations'
        eval_dir.mkdir()
        for step in frozen['eval_steps']:
            env = os.environ.copy()
            env.update(JAX_PLATFORMS='cpu', CUDA_VISIBLE_DEVICES='',
                       AVIATOR_GFR_MODEL_MANIFEST=str(manifest), AVIATOR_GFR_SPLIT='rl_train')
            destination = eval_dir / f'checkpoint_{step}.json'
            cmd = [sys.executable, str(HIL / 'tools/gfr1_eval.py'), '--checkpoint',
                   str(out / f'bc_seed11/checkpoint_{step}'), '--agent-kind', 'bc',
                   '--mode', 'structured', '--visible-error', '--full-horizon', '--episodes', '10',
                   '--reference-demo-dir', str(demo), '--output', str(destination)]
            if a.mask_previous_action:
                cmd.append('--mask-previous-action')
            atomic_json(out / 'status.json', {'phase': 'bc_evaluation', 'step': step, 'command': cmd, 'pid': os.getpid()})
            with (eval_dir / f'checkpoint_{step}.log').open('x') as log:
                rc = subprocess.run(cmd, env=env, cwd=HIL, stdout=log, stderr=subprocess.STDOUT).returncode
            report = json.loads(destination.read_text()) if destination.exists() else {'instrumentation_unknown': 1}
            report['returncode'] = rc
            reports[step] = report
            atomic_json(eval_dir / f'checkpoint_{step}_process.json', {'returncode': rc})
            print(json.dumps({'eval_checkpoint': step, 'returncode': rc, 'summary': report.get('summary')}), flush=True)
            if rc != 0 or report.get('instrumentation_unknown') != 0:
                break
        sources_unchanged = all(sha256(path) == value for path, value in hashes.items())
        assert sources_unchanged
        decision = 'smoke_only' if a.smoke else qualify_bc(reports)
        if not a.smoke and decision == 'pending':
            decision = 'instrumentation_error'
        result = {'kind': 'gfr_bc_qualification', 'decision': decision,
                  'demo_buffer_sha256': audit['buffer_sha256'],
                  'checkpoint': training['checkpoint'], 'checkpoint_sha256': training['checkpoint_sha256'],
                  'training': training, 'evaluations': reports, 'sources_unchanged': sources_unchanged,
                  'protocol': str(out / 'frozen_protocol.json')}
        atomic_json(out / 'bc_qualification.json', result)
        atomic_json(out / 'status.json', {'phase': 'bc_finished', 'decision': decision, 'sac_started': False})
        if a.continue_sac_if_qualified and decision == 'pass':
            cmd = [sys.executable, str(HIL / 'tools/gfr1_train_verified_gate.py'),
                   '--output', str(out / 'sac_from_bc'), '--demo-dir', str(demo),
                   '--eval-manifest', str(a.heldout_manifest.resolve()),
                   '--bc-checkpoint', training['checkpoint'],
                   '--bc-qualification', str(out / 'bc_qualification.json')]
            atomic_json(out / 'status.json', {'phase': 'sac_from_qualified_bc', 'command': cmd, 'sac_started': True})
            with (out / 'sac_supervisor.log').open('x') as log:
                rc = subprocess.run(cmd, cwd=HIL, stdout=log, stderr=subprocess.STDOUT).returncode
            atomic_json(out / 'sac_process.json', {'returncode': rc})
            atomic_json(out / 'status.json', {'phase': 'finished', 'bc_decision': decision, 'sac_returncode': rc})
            if rc:
                raise RuntimeError(f'SAC supervisor returned {rc}')
    except BaseException as exc:
        atomic_json(out / 'error.json', {'type': type(exc).__name__, 'message': str(exc)})
        raise


if __name__ == '__main__':
    main()
