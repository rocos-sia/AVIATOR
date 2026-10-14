"""Pair the pre-fix and corrected runtime, preserving all previous evidence."""
import argparse
import hashlib
import importlib.machinery
import importlib.util
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.validate_online_lift import analytic_constraint_matrices

ROOT = Path(__file__).resolve().parents[2]


def build(cls, mode):
    return cls(str(ROOT / 'hil-serl/data/aviator/manifold_phi_stale_pitch-10deg'),
               str(ROOT / 'AviatorRobot/build/bin/model/aviator.xml'),
               str(ROOT / 'hil-serl/data/aviator/trajectory_source'),
               split='val', mode=mode)


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    loader = importlib.machinery.SourceFileLoader(
        'examples.experiments.aviator_manifold.gfr_before', args.before_source)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    old = importlib.util.module_from_spec(spec)
    loader.exec_module(old)
    rows, contracts = [], []
    with (output / 'beta_pair_trace.jsonl').open('x') as trace:
        for version, cls in [('before', old.GFRManifoldEnv), ('after', GFRManifoldEnv)]:
            env = build(cls, 'raw')
            env.reset(seed=0, options={'trajectory_index': 0})
            nominal = np.array([.08, .002])
            original = env._lift
            for beta in env.governor_schedule:
                stages = []

                def capture(x, q, v, previous):
                    result = original(x, q, v, previous)
                    e, aq, ax = analytic_constraint_matrices(env.fk, x, q)
                    _, n, he, hx, sigma = result
                    res = aq @ (he + beta * hx) + ax @ (beta * nominal) + 2 * e
                    stages.append({'residual_inf': float(np.max(np.abs(res))),
                                   'sigma_min': sigma})
                    return result

                env._lift = capture
                env._candidate(env._x, env._q, nominal, np.zeros(2), beta, None, False)
                contracts.append({'version': version, 'beta': beta, 'stages': stages})
            env._lift = original
            for mode in ['fixed', 'structured']:
                env = build(cls, mode)
                for episode in range(2):
                    env.reset(seed=1000 + episode, options={'trajectory_index': episode})
                    initial_q = env._q.copy()
                    initial_x = env._x.copy()
                    closures, accepted_clearance, betas = [], [], []
                    for step in range(args.max_steps):
                        _, reward, term, trunc, info = env.step(np.zeros(2, dtype=np.float32))
                        accepted = info['beta_executed'] > 0
                        closure = float(np.max(np.abs(env.fk.error(env._x, env._q))))
                        state_clearance = float(env.checker.check(env._x, env._q)[0])
                        closures.append(closure)
                        if accepted:
                            accepted_clearance.append(info['d_actual_m'])
                            betas.append(info['beta_executed'])
                        trace.write(json.dumps({
                            'version': version, 'mode': mode, 'episode': episode,
                            'step': step, 'accepted': accepted,
                            'beta': info['beta_executed'], 'progress': info['progress'],
                            'committed_closure_inf': closure,
                            'committed_clearance_m': state_clearance,
                            'candidate_clearance_m': info.get('d_actual_m'),
                            'termination': info['termination'], 'reason': info.get('reason'),
                            'reward': reward,
                        }) + '\n')
                        if term or trunc:
                            break
                    row = {
                        'version': version, 'mode': mode, 'episode': episode,
                        'trajectory': str(env._traj_paths[episode]),
                        'initial_q': initial_q.tolist(), 'initial_x': initial_x.tolist(),
                        'steps': step + 1, 'progress': info['progress'],
                        'termination': info['termination'] if term or trunc else 'probe_budget',
                        'max_committed_closure_inf': max(closures),
                        'min_accepted_clearance_m': min(accepted_clearance, default=None),
                        'slow_accepted_steps': sum(b < 1. for b in betas),
                    }
                    rows.append(row)
                    print(json.dumps({k: v for k, v in row.items()
                                      if k not in ['initial_q', 'initial_x', 'trajectory']}), flush=True)
    manifest = json.loads((output / 'input_hashes.json').read_text())
    changed = [p for p, digest in manifest.items()
               if hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest]
    report = {'scope': 'paired runtime diagnostic, not RL effectiveness',
              'max_steps': args.max_steps, 'contracts': contracts, 'rollouts': rows,
              'protected_files': len(manifest), 'changed_protected_files': changed}
    with (output / 'beta_pair_summary.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    assert not changed, changed


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--before-source', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-steps', type=int, default=200)
    run(parser.parse_args())
