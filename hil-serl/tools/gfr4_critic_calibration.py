"""Frozen-actor critic calibration: uniform versus failure-stratified replay.

Fitting uses only the first 60k transitions. Later 60k failures are evaluation
only (same training installations, different collected states; not a geometry
holdout). Both initializations and both sampling arms use fixed update budgets.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import torch
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr3_train_sac_demo import fingerprint
from tools.gfr4_common import KEYS, restore, demonstrations, replay_prefix, FixedReplaySampler, require_memory_limit


def calibration(learner, probes):
    result = {}
    with torch.no_grad():
        for name, (obs, action, reward) in probes.items():
            q1 = learner.q1(obs, action).ravel()
            q2 = learner.q2(obs, action).ravel()
            q = torch.minimum(q1, q2)
            error = q-reward
            result[name] = dict(count=len(q), q_mean=float(q.mean()), reward_mean=float(reward.mean()),
                rmse=float(error.square().mean().sqrt()), mae=float(error.abs().mean()),
                positive_fraction=float((q>0).float().mean()),
                within_two_fraction=float((error.abs()<=2).float().mean()),
                q1_rmse=float((q1-reward).square().mean().sqrt()),
                q2_rmse=float((q2-reward).square().mean().sqrt()))
    return result


def run(a):
    require_memory_limit()
    out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    inputs = {str(a.artifact_root/arm/name): sha(a.artifact_root/arm/name)
              for arm in ('bc', 'random') for name in ('sac_060000.pt', 'online_replay.npz')}
    inputs.update({str(p):sha(p) for p in (a.artifact_root/'demonstrations').glob('*.npz')})
    protocol = dict(phase='critic_only', actor_frozen=True, alpha_frozen=True,
        initial_checkpoint_step=60000, updates_per_arm=a.updates,
        modes=['uniform', 'stratified'], initializations=['bc', 'random'],
        batch=dict(demonstrations=128, online=128, stratified_online_failures=32),
        stratification='32 failure + 96 nonfailure online samples; no importance correction',
        fitting_data='original first 60000 online transitions and the unchanged teacher pool',
        testing_data='original later 60000 terminal failures; never used in updates',
        scope='single-seed diagnostic on training installations, not unseen-installation reliability',
        optimizer='restore 60k optimizer and target networks; original gamma/tau/lr unchanged',
        factorial_policy='factorial starts from ORIGINAL 60k, never from these fitted critics',
        inputs=inputs, sources={str(p.resolve()):sha(p) for p in Path(__file__).parent.glob('gfr4_*.py')})
    dump(out/'protocol.json', protocol)
    summaries = []
    for arm in ('bc', 'random'):
        for mode in ('uniform', 'stratified'):
            learner, identity = restore(a.artifact_root, a.source_root, arm)
            learner.actor.requires_grad_(False)
            learner.log_alpha.requires_grad_(False)
            demo = demonstrations(a.artifact_root, learner.device)
            replay = replay_prefix(a.artifact_root, arm, learner.device, 60008)
            sampler = FixedReplaySampler(replay, mode)
            probes = {}
            with np.load(a.artifact_root/arm/'online_replay.npz') as data:
                failure = data['rewards'].ravel() < -9
                terminal = data['terminals'].ravel() == 1
                assert terminal[failure].all()
                index = np.arange(len(failure))
                masks = dict(fit_failures=failure & (index<60000),
                             unseen_later_failures=failure & (index>=60000),
                             fit_success_terminals=terminal & (data['rewards'].ravel()>0) & (index<60000))
                for label, mask in masks.items():
                    probes[label] = tuple(torch.as_tensor(data[key][mask], device=learner.device)
                                          for key in ('observations', 'actions', 'rewards'))
                    obs, act, reward = probes[label]; probes[label] = (obs, act, reward.ravel())
            alpha_before = learner.log_alpha.detach().clone()
            directory = out/f'{arm}_{mode}'; directory.mkdir()
            history = [dict(update=0, calibration=calibration(learner, probes))]
            start = time.perf_counter()
            with (directory/'learning.jsonl').open('x', buffering=1) as log:
                for update in range(1, a.updates+1):
                    db = demo.sample(128); ob = sampler.sample(128)
                    batch = tuple(torch.cat((d, o)) for d, o in zip(db, ob))
                    metrics = learner.update(batch, (db[0], db[1]), 0., critic_only=True)
                    if update % 1000 == 0 or update == a.updates:
                        row = dict(update=update, elapsed_s=time.perf_counter()-start,
                                   calibration=calibration(learner, probes), **metrics)
                        history.append(row); log.write(json.dumps(row)+'\n')
                        dump(directory/'evaluations.json', history)
                        dump(out/'status.json', dict(phase='critic_calibration', arm=arm, sampling=mode,
                            update=update, budget=a.updates, latest=row))
                        print(json.dumps(dict(arm=arm, sampling=mode, **row)), flush=True)
            assert fingerprint(learner.actor) == identity['actor']
            assert torch.equal(learner.log_alpha, alpha_before)
            learner.save(directory/'critic_only_final.pt', 60000)
            summary = dict(arm=arm, sampling=mode, updates=a.updates, actor_unchanged=True,
                           alpha_unchanged=True, initial=history[0], final=history[-1])
            summaries.append(summary)
            dump(out/'results.json', summaries)
            del learner, demo, replay, sampler, probes
            torch.cuda.empty_cache()
    assert all(sha(p)==h for p,h in inputs.items())
    dump(out/'status.json', dict(phase='completed', execution_completed=True, actor_unchanged=True,
         input_artifacts_unchanged=True, factorial_initialization='original 60k, unchanged'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('artifact-root', 'source-root', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--updates', type=int, default=20000)
    run(parser.parse_args())
