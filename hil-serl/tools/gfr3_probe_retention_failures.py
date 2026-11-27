"""Replay lost-success cases and compare actions at identical failure states.

Diagnostic only: candidate alternatives are never executed as policy rollouts.
No learning or checkpoint changes.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr2_teacher import make_env
from tools.gfr2_train_shared_bc import cuda_device
from tools.gfr3_diagnose_retention import load_nets


def run(root, dataset, output):
    device = cuda_device()
    records = {r['case']: r for r in json.loads((dataset/'train.json').read_text())['records']}
    diagnostic = json.loads((output/'checkpoint_audit.json').read_text())
    results = []
    hashes = {}
    for arm in ('bc', 'random'):
        nets = {}
        for step in (60000, 120000):
            path = root/arm/f'sac_{step:06d}.pt'
            hashes[str(path)] = sha(path)
            nets[step] = load_nets(path, device)[:3]
        for case in diagnostic['arms'][arm]['lost_successes']:
            history = [json.loads(line) for line in
                       (root/arm/'evaluation_120000'/f"{case.replace('/', '__')}.jsonl").open()]
            env = make_env(records[case])
            try:
                for index, original in enumerate(history):
                    observation = env.last_policy_observation.copy()
                    action = np.asarray(original['action'], np.float32)
                    _, _, done, truncated, info = env.step(action)
                    assert env._last_direct_accepted == original['accepted'], (case, index)
                    assert abs(info['progress']-original['progress']) < 1e-10, (case, index)
                    if done or truncated:
                        assert index == len(history)-1 and not env._last_direct_accepted
                        break
                obs = torch.as_tensor(observation, device=device).unsqueeze(0)
                with torch.no_grad():
                    actions = {str(step): nets[step][0].sample(obs, True)[0][0].cpu().numpy()
                               for step in nets}
                assert np.max(np.abs(actions['120000']-action)) < 1e-6
                actions['zero'] = np.zeros(2, np.float32)
                actions['previous'] = np.asarray(env._r_prev/env.r_max, np.float32)
                actions.update({f'grid_{i}_{j}': np.array([a, b], np.float32)
                                for i, a in enumerate(np.linspace(-1, 1, 5))
                                for j, b in enumerate(np.linspace(-1, 1, 5))})
                trials = {}
                for label, candidate in actions.items():
                    trial, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                                              candidate*env.r_max, 1., env._previous_n, structured=False)
                    with torch.no_grad():
                        act = torch.as_tensor(candidate, device=device).unsqueeze(0)
                        values = {str(step): float(torch.minimum(q1(obs, act), q2(obs, act)).item())
                                  for step, (_, q1, q2) in nets.items()}
                    trials[label] = dict(action=candidate.tolist(), feasible=bool(trial['feasible']),
                        critic_q=values, candidate=env._json_safe_candidate(trial))
                row = dict(arm=arm, case=case, failed_step=index, progress=info['progress'],
                    replay_exact=True, trials=trials,
                    feasible_grid_count=sum(t['feasible'] for name, t in trials.items() if name.startswith('grid_')))
                results.append(row)
                dump(output/'failure_state_probes.json', dict(cases=results, complete=False))
                print(json.dumps({k:v for k,v in row.items() if k!='trials'}), flush=True)
            finally:
                env.close()
    assert all(sha(p) == digest for p, digest in hashes.items())
    dump(output/'failure_state_probes.json', dict(cases=results, complete=True,
        checkpoint_sha256=hashes, checkpoints_unchanged=True, source_sha256=sha(__file__),
        scope='same pre-failure observation and physical state; one-step alternatives only; no training'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'dataset', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.root.resolve(), args.dataset.resolve(), args.output.resolve())
