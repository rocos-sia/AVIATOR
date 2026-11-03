#!/usr/bin/env python3
"""Launch the audited 100-demo run and independent full-horizon evaluations."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from tools.gfr1_gate_support import atomic_json, evaluate_gate
from tools.gfr1_prepare_teacher_buffer import sha256
from tools.gfr1_bc_support import checkpoint_hash, validate_bc_warmstart_report

ROOT = Path(__file__).resolve().parents[2]
HIL = ROOT / 'hil-serl'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--demo-dir', type=Path, required=True)
    p.add_argument('--eval-manifest', type=Path, required=True)
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--compatibility-audit', type=Path, required=True)
    p.add_argument('--bc-checkpoint', type=Path)
    p.add_argument('--bc-qualification', type=Path)
    p.add_argument('--bc-initialization-only', action='store_true',
                   help='Explicit BC warm-start experiment: completion gate is not a prerequisite; integrity and instrumentation checks remain required.')
    a = p.parse_args()
    if a.bc_checkpoint or a.bc_qualification or a.bc_initialization_only:
        p.error("Fresh SAC only: BC and old SAC initialization forbidden")
    if a.bc_initialization_only and not a.bc_checkpoint:
        p.error('--bc-initialization-only requires a BC checkpoint and its report')
    out, demo = a.output.resolve(), a.demo_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    audit = json.loads((demo / 'demo_manifest.json').read_text())
    compatibility = json.loads(a.compatibility_audit.read_text())
    assert compatibility['passed'] and compatibility['episodes'] == 100
    assert compatibility['buffer_sha256'] == audit['buffer_sha256']
    assert audit['passed'] and audit['episode_count'] == 100
    assert audit['transition_count'] == 302125 and audit['instrumentation_unknown'] == 0
    assert sha256(demo / 'structured_visible.pkl') == audit['buffer_sha256']
    assert sha256(demo / 'train_manifest.json') == audit['train_manifest_sha256']
    bc_provenance = None
    if bool(a.bc_checkpoint) != bool(a.bc_qualification):
        raise ValueError('BC initialization requires both checkpoint and qualification report')
    if a.bc_checkpoint is not None:
        qualification = json.loads(a.bc_qualification.read_text())
        bc_decision = validate_bc_warmstart_report(qualification, a.bc_initialization_only)
        bc_protocol = json.loads(Path(qualification['protocol']).read_text())
        if bc_protocol.get('mask_previous_action', False):
            raise ValueError('masked BC cannot initialize the full-observation SAC pipeline')
        assert qualification['demo_buffer_sha256'] == audit['buffer_sha256']
        assert Path(qualification['checkpoint']).resolve() == a.bc_checkpoint.resolve()
        assert checkpoint_hash(a.bc_checkpoint) == qualification['checkpoint_sha256']
        assert a.bc_checkpoint.name == 'checkpoint_20000'
        bc_provenance = {'checkpoint': str(a.bc_checkpoint.resolve()),
                         'checkpoint_sha256': qualification['checkpoint_sha256'],
                         'qualification': str(a.bc_qualification.resolve()),
                         'qualification_sha256': sha256(a.bc_qualification),
                         'bc_execution_decision': bc_decision,
                         'initialization_only_experiment': a.bc_initialization_only,
                         'completion_gate_required': not a.bc_initialization_only}
    for path, digest in audit['source_sha256'].items():
        assert sha256(path) == digest, f'source changed: {path}'
    train_manifest = demo / 'train_manifest.json'
    eval_manifest = a.eval_manifest.resolve()
    training = json.loads(train_manifest.read_text())
    testing = json.loads(eval_manifest.read_text())
    assert len(testing['records']) == 10
    assert not set(training['models']) & set(testing['models'])
    assert not {r['trajectory_path'] for r in training['records']} & {r['trajectory_path'] for r in testing['records']}
    control = out / 'gate_control'
    control.mkdir()
    eval_dir = out / 'evaluations'
    eval_dir.mkdir()
    env = os.environ.copy()
    # Scope every setting to a fresh run; optionally initialize only its actor
    # from an audited GFR BC checkpoint, never historical LUT weights.
    for key in list(env):
        if key.startswith(('AVIATOR_', 'JAX_', 'XLA_')):
            del env[key]
    env.update({
        'GFR_PYTHON': sys.executable,
        'PYTHONPATH': f'{HIL}/examples:{HIL}:{ROOT}',
        'AVIATOR_GFR_MANIFOLD': str(HIL / 'data/aviator/manifold_phi_stale_pitch-10deg'),
        'AVIATOR_GFR_TRAJECTORY_DIR': str(HIL / 'data/aviator/trajectory_source'),
        'AVIATOR_GFR_MODEL': training['models'][0],
        'AVIATOR_GFR_MODEL_MANIFEST': str(train_manifest),
        'AVIATOR_GFR_SPLIT': 'rl_train',
        'AVIATOR_ASYNC_GATE_DIR': str(control),
        'AVIATOR_EPISODE_LOG': str(out / 'online_episodes.jsonl'),
        'AVIATOR_REPAIRED_TRACE_DIR': str(out / 'executor_traces'),
        'AVIATOR_CRITIC_CLIP_NORM': '1.0',
        'AVIATOR_MARGIN_WEIGHT': '0.1',
        'CUDA_VISIBLE_DEVICES': '0', 'JAX_PLATFORMS': 'cuda',
        'XLA_PYTHON_CLIENT_PREALLOCATE': 'false',
        'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
        'MKL_NUM_THREADS': '1', 'PYTHONUNBUFFERED': '1',
    })
    cmd = [sys.executable, str(HIL / 'tools/run_gfr1_repaired_formal.py'),
           '--output', str(out / 'training'), '--demo-dir', str(demo),
           '--seed', str(a.seed), '--num-envs', '16', '--steps', '60000',
           '--episodes', '400', '--checkpoint-period', '5000', '--conditions', 'B4']
    if a.bc_checkpoint is not None:
        cmd += ['--bc-checkpoint', str(a.bc_checkpoint.resolve())]
    source_files = [HIL / 'tools' / name for name in (
        'gfr1_train_repaired_gate.py', 'gfr1_gate_support.py', 'gfr1_bc_support.py', 'gfr1_eval.py',
        'gfr1_prepare_teacher_buffer.py', 'run_gfr1_repaired_formal.py',
        'gfr1_repaired_entry.py', 'gfr1_repaired_env.py', 'gfr1_repaired_eval.py',
        'gfr1_recovery_boundary_ablation.py', 'gfr1_teacher_replay_consistency.py',
        'gfr1_audit_repaired_demos.py')]
    source_files += [HIL / 'examples/train_rlpd.py',
                     HIL / 'examples/experiments/aviator_manifold/vectorized_actor.py',
                     HIL / 'examples/experiments/aviator_manifold/gfr_env.py',
                     HIL / 'examples/experiments/aviator_gfr/config.py',
                     HIL / 'examples/experiments/aviator_manifold/config.py',
                     HIL / 'serl_launcher/serl_launcher/agents/continuous/sac.py']
    frozen = {'kind': 'gfr1_repaired_executor_scratch_training',
              'executor': 'combined_repair_v1', 'initialization': 'random_actor_critic_temperature_no_checkpoint',
              'development_eval_note': 'Same diagnosed development validation set; not untouched test data',
              'backend_scope': 'Frozen nonlinear checker and thresholds; same reward and 40-D observation semantics; alternative recovery separately logged',
              'demo_note': 'Retained historical accepted teacher transitions, not newly collected under repair; initial-step compatibility audited separately', 'seed': a.seed,
              'bc_initialization': bc_provenance,
              'demo_episodes': 100, 'demo_transitions': 302125,
              'online_transition_cap': 60000, 'online_episode_target': 400,
              'budget_note': '60k transitions and the 20k/40k gate take precedence; report actual completed episodes, never assert that 400 episodes occurred.',
              'num_envs': 16,
              'environment_execution': '16 logical environments stepped sequentially by the existing vectorized actor; GPU policy inference is batched',
              'demo_fraction': .5, 'critic_clip_norm': 1., 'correction_weight': .1,
              'dt': .005, 'd_safe': .005, 'beta_stop': .125, 'r_max': 1.5, 'qdot_max': 1.5,
              'reward': 'unchanged GFR backend reward, not the historical LUT margin reward',
              'train_manifest': str(train_manifest), 'eval_manifest': str(eval_manifest),
              'eval_episodes': 10, 'eval_device': 'cpu', 'train_device': 'cuda:0',
              'checkpoint_period': 5000, 'gate_steps': [20000, 40000],
              'gate_rule': '20k evaluation runs while training continues; at 40k wait for both reports; both zero stops; evaluator error stops as instrumentation error; otherwise continue to 60k without claiming convergence.',
              'demo_buffer_sha256': audit['buffer_sha256'],
              'compatibility_audit': str(a.compatibility_audit.resolve()),
              'compatibility_audit_sha256': sha256(a.compatibility_audit),
              'train_manifest_sha256': sha256(train_manifest),
              'eval_manifest_sha256': sha256(eval_manifest),
              'code_sha256': {str(f): sha256(f) for f in source_files},
              'training_command': cmd,
              'environment': {k: v for k, v in env.items() if k.startswith(('AVIATOR_', 'GFR_', 'JAX_', 'XLA_', 'CUDA_', 'OMP_', 'OPENBLAS_', 'MKL_', 'PYTHON'))}}
    atomic_json(out / 'frozen_config.json', frozen)
    log = (out / 'training_wrapper.log').open('x')
    proc = subprocess.Popen(cmd, cwd=HIL, env=env, stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    jobs, reports, launched = {}, {}, set()
    decision = 'pending'
    run_dir = out / f'training/b4_seed{a.seed}'
    def stop_training():
        (control / 'STOP').touch(exist_ok=True)
    def cleanup():
        stop_training()
        for job, handle in jobs.values():
            if job.poll() is None:
                os.killpg(job.pid, signal.SIGTERM)
            handle.close()
        if proc.poll() is None:
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        while True:
            for step in (20000, 40000, 60000):
                ready = control / f'ready_{step}.json'
                if step in launched or not ready.exists():
                    continue
                ee = env.copy()
                ee.update({'AVIATOR_GFR_MODEL_MANIFEST': str(eval_manifest),
                           'AVIATOR_GFR_SPLIT': 'val', 'JAX_PLATFORMS': 'cpu',
                           'CUDA_VISIBLE_DEVICES': ''})
                ee.pop('AVIATOR_ASYNC_GATE_DIR', None)
                ee.pop('AVIATOR_EPISODE_LOG', None)
                ee['AVIATOR_REPAIRED_TRACE_DIR'] = str(eval_dir / f'checkpoint_{step}_executor_traces')
                checkpoint = json.loads(ready.read_text())['checkpoint']
                ec = [sys.executable, str(HIL / 'tools/gfr1_repaired_entry.py'), 'eval',
                      '--checkpoint', checkpoint, '--mode', 'structured', '--visible-error',
                      '--full-horizon', '--episodes', '10',
                      '--output', str(eval_dir / f'checkpoint_{step}.json')]
                handle = (eval_dir / f'checkpoint_{step}.log').open('x')
                ep = subprocess.Popen(ec, cwd=HIL, env=ee, stdout=handle,
                                      stderr=subprocess.STDOUT, start_new_session=True)
                jobs[step] = (ep, handle)
                launched.add(step)
                print(f'checkpoint {step}: CPU evaluation started, pid={ep.pid}', flush=True)
            for step, (ep, handle) in list(jobs.items()):
                rc = ep.poll()
                if rc is None:
                    continue
                handle.close()
                path = eval_dir / f'checkpoint_{step}.json'
                try:
                    report = json.loads(path.read_text())
                except (OSError, ValueError):
                    report = {'instrumentation_unknown': 1, 'summary': {}}
                report['returncode'] = rc
                reports[step] = report
                del jobs[step]
                atomic_json(eval_dir / f'checkpoint_{step}_process.json',
                            {'returncode': rc, 'instrumentation_unknown': report.get('instrumentation_unknown', 0)})
                print(f'checkpoint {step}: rc={rc}, summary={report.get("summary")}', flush=True)
            if any(r['returncode'] != 0 or r.get('instrumentation_unknown', 0) for r in reports.values()):
                decision = 'instrumentation_error'
                stop_training()
            elif decision == 'pending':
                decision = evaluate_gate(reports)
            if decision != 'pending' and not (control / 'decision.json').exists():
                atomic_json(control / 'decision.json', {'decision': decision})
                print(f'gate decision: {decision}', flush=True)
            state = {'kind': 'gfr1_training_live_status', 'supervisor_pid': os.getpid(),
                     'training_pid': proc.pid, 'training_returncode': proc.poll(),
                     'gate_decision': decision,
                     'checkpoint_ready_steps': sorted(int(f.stem.split('_')[1]) for f in control.glob('ready_*.json')),
                     'running_evaluations': {str(s): p.pid for s, (p, _) in jobs.items()},
                     'completed_evaluations': {str(s): {'returncode': r['returncode'], 'summary': r.get('summary')}
                                               for s, r in reports.items()},
                     'updated_unix': time.time()}
            atomic_json(out / 'run_status.json', state)
            if proc.poll() is not None and not jobs:
                break
            time.sleep(5)
        if proc.returncode != 0:
            decision = 'instrumentation_error'
        summary = {'kind': 'gfr1_verified_training_gate_summary', 'frozen': frozen,
                   'training_returncode': proc.returncode, 'gate_decision': decision,
                   'evaluations': reports, 'training_run_dir': str(run_dir)}
        atomic_json(out / 'gfr1_async_gate_summary.json', summary)
    finally:
        cleanup()
        log.close()


if __name__ == '__main__':
    main()
