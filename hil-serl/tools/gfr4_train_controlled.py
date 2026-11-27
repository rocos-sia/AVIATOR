"""One arm of the 60k retention factorial, with full evaluation every 10k."""
import argparse
from functools import partial
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.vec_env import SubprocVecEnv
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr2_train_shared_bc import evaluate
from tools.gfr3_train_sac_demo import terminal_transition
from tools.gfr4_common import KEYS, require_memory_limit, restore, demonstrations, replay_prefix
from tools.gfr4_controlled_env import make_worker, bc_weight
from tools.gfr4_retention_probes import evaluate_probes


def success_retention(baseline_successes, episodes):
    evaluated = {r['case'] for r in episodes}
    completed = {r['case'] for r in episodes if r['completed']}
    return dict(lost_baseline_successes=sorted((baseline_successes & evaluated)-completed),
                gained_successes=sorted(completed-baseline_successes),
                unevaluated_baseline_successes=sorted(baseline_successes-evaluated))


def run(a):
    require_memory_limit()
    out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    learner, identities = restore(a.artifacts, a.source, 'bc')
    demo = demonstrations(a.artifacts, learner.device)
    replay = replay_prefix(a.artifacts, 'bc', learner.device)
    records = []
    for split in ('train', 'development'):
        records.extend(json.loads((a.source/f'dataset/{split}.json').read_text())['records'])
    if a.smoke:
        records = records[:1]
    protocol = json.loads((a.root/'protocol.json').read_text())
    inputs = protocol['inputs']; sources = protocol['sources']
    assert all(sha(p)==h for p,h in {**inputs, **sources}.items())
    config = dict(bc_mode=a.bc_mode, curriculum_mode=a.curriculum_mode, smoke=a.smoke,
        device=str(learner.device), gpu=torch.cuda.get_device_name(), initial_step=60000,
        final_step=a.end_step, initial_networks=identities,
        initial_replay_size=replay.size, inputs=inputs, sources=sources,
        resume_semantics='restore all networks, optimizers, torch RNG and first60k replay; reset both environments identically; not exact old-process continuation',
        online_sampling='unchanged uniform', critic_calibration_weights_used=False)
    dump(out/'config.json', config)
    env = None; step = 60000; curves = []
    started = time.perf_counter()
    baseline = json.loads((a.artifacts/'bc/evaluation_060000/episodes.json').read_text())
    baseline_successes = {r['case'] for r in baseline if r['completed']}
    baseline_probes = json.loads((a.root/'baseline_retention.json').read_text())
    baseline_passed = {(r['origin_arm'],r['case']) for r in baseline_probes['rows'] if r['passed']}
    try:
        factories = [partial(make_worker, str(a.source), str(a.artifacts/'demonstrations'), str(out),
                             rank, 2, a.curriculum_mode) for rank in range(2)]
        env = SubprocVecEnv(factories, start_method='spawn')
        env.seed(11); obs = env.reset()
        np.save(out/'initial_worker_observations.npy', obs)
        with (out/'learning.jsonl').open('x', buffering=1) as log:
            while step < a.end_step:
                actions = learner.predict(obs)
                nxt, rewards, dones, infos = env.step(actions)
                successors, terminals = terminal_transition(nxt, dones, infos)
                replay.add(obs, actions, rewards, successors, terminals)
                obs = nxt; step += 2
                weight = bc_weight(a.bc_mode, step)
                for _ in range(2):
                    db = demo.sample(128); ob = replay.sample(128)
                    batch = tuple(torch.cat((d,o)) for d,o in zip(db,ob))
                    metrics = learner.update(batch, (db[0],db[1]), weight)
                if step % 1000 == 0 or step == a.end_step:
                    row = dict(step=step, additional_interactions=step-60000,
                        elapsed_s=time.perf_counter()-started, **metrics)
                    log.write(json.dumps(row)+'\n')
                    dump(out/'status.json', dict(phase='controlled_training', **row))
                    print(json.dumps(row), flush=True)
                if step % 10000 == 0 or step == a.end_step:
                    checkpoint = out/f'actor_{step:06d}.pt'
                    learner.export(checkpoint); learner.save(out/f'sac_{step:06d}.pt', step)
                    dump(out/'status.json', dict(phase='full_trajectory_evaluation', step=step))
                    gate, episodes = evaluate(records, checkpoint, out/f'evaluation_{step:06d}', 1)
                    if any(r['termination']=='exception' or r.get('audit_violations') for r in episodes):
                        raise RuntimeError('Evaluation instrumentation/audit failure')
                    probes = evaluate_probes(checkpoint, a.root/'retention_states', out/f'retention_{step:06d}.json')
                    passed = {(r['origin_arm'],r['case']) for r in probes['rows'] if r['passed']}
                    curve = dict(step=step, splits={}, retention_passed=probes['passed'],
                        lost_baseline_probe_actions=sorted(baseline_passed-passed),
                        **success_retention(baseline_successes, episodes),
                        checkpoint_sha256=sha(checkpoint))
                    for split in ('train','development'):
                        subset = [r for r in episodes if r['case'].startswith(split+'_')]
                        if subset:
                            curve['splits'][split] = dict(cases=len(subset),completed=sum(r['completed'] for r in subset),
                                median_progress=float(np.median([r['progress'] for r in subset])))
                    curve['retention_alert'] = bool(curve['lost_baseline_successes'] or curve['lost_baseline_probe_actions'])
                    curves.append(curve); dump(out/'evaluations.json',curves)
                    dump(out/'status.json',dict(phase='evaluation_completed', **curve))
                    print(json.dumps(dict(event='evaluation',**curve)),flush=True)
        np.savez_compressed(out/'online_replay.npz', **{k:v[:replay.size] for k,v in zip(KEYS,replay.arrays)})
        assert all(sha(p)==h for p,h in {**inputs,**sources}.items())
        dump(out/'status.json',dict(phase='completed',execution_completed=True,step=step,
             input_artifacts_unchanged=True,scientific_qualification='see full trajectory curves'))
    except BaseException as error:
        dump(out/'status.json',dict(phase='stopped_on_error',execution_completed=False,step=step,error=repr(error)))
        raise
    finally:
        if env is not None:
            env.close()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts','source','root','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--bc-mode',choices=['fixed','decay'],required=True)
    parser.add_argument('--curriculum-mode',choices=['hold','advance'],required=True)
    parser.add_argument('--end-step',type=int,default=120000)
    parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args()
    if args.end_step<60002 or args.end_step>120000 or args.end_step%2:
        parser.error('end-step must be even, greater than 60000 and at most 120000')
    run(args)
