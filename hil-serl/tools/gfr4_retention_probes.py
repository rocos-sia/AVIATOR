"""Fixed historical failure states, isolated from all training replay pools."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr2_teacher import make_env
from tools.gfr2_train_shared_bc import load_actor, cuda_device
from tools.gfr3_demo_transitions import pack_state, unpack_state
from tools.gfr4_common import require_memory_limit


def prepare(artifacts, source, diagnosis, output):
    output.mkdir(parents=True, exist_ok=False)
    cases = json.loads((diagnosis/'failure_state_probes.json').read_text())['cases']
    records = {r['case']:r for r in json.loads((source/'dataset/train.json').read_text())['records']}
    observations, states, actions, metadata = [], [], [], []
    for entry in cases:
        arm, case = entry['arm'], entry['case']
        log = artifacts/arm/'evaluation_120000'/f"{case.replace('/', '__')}.jsonl"
        history = [json.loads(line) for line in log.open()]
        env = make_env(records[case])
        try:
            for i, row in enumerate(history):
                state = pack_state(env)
                obs = env.last_policy_observation.copy()
                action = np.asarray(row['action'], np.float32)
                _, _, done, truncated, info = env.step(action)
                assert env._last_direct_accepted == row['accepted']
                assert abs(info['progress']-row['progress']) < 1e-10
                if done or truncated:
                    assert i == len(history)-1 and not env._last_direct_accepted
                    break
            assert i == entry['failed_step']
            restored = unpack_state(env, state)
            np.testing.assert_allclose(restored, obs, atol=1e-6, rtol=0)
            trial, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                action*env.r_max, 1., env._previous_n, structured=False)
            assert not trial['feasible']
            observations.append(obs); states.append(state); actions.append(action)
            metadata.append(dict(origin_arm=arm, case=case, failed_step=i,
                                 record=records[case], log_sha256=sha(log)))
            print(json.dumps(dict(event='retention_state_prepared', arm=arm, case=case)), flush=True)
        finally:
            env.close()
    np.savez_compressed(output/'states.npz', observations=np.asarray(observations, np.float32),
                        states=np.asarray(states), rejected_actions=np.asarray(actions, np.float32))
    dump(output/'manifest.json', dict(cases=metadata, passed=len(metadata)==9,
         state_sha256=sha(output/'states.npz'), use='diagnostic evaluation only; no training data',
         scope='nine selected historical failures; one-step direct feasibility, not full-task success'))


def evaluate_probes(checkpoint, prepared, destination):
    # Restore RNG even if evaluation shares the learner process.
    cpu_rng = torch.get_rng_state(); cuda_rng = torch.cuda.get_rng_state()
    try:
        device = cuda_device()
        actor = load_actor(checkpoint, device)
        manifest = json.loads((prepared/'manifest.json').read_text())
        assert manifest['passed'] and sha(prepared/'states.npz') == manifest['state_sha256']
        with np.load(prepared/'states.npz') as data:
            observations = data['observations'].copy(); states = data['states'].copy()
        # Single-state inference matches the full-trajectory evaluator exactly.
        with torch.no_grad():
            actions = [actor(torch.as_tensor(obs, device=device)).cpu().numpy() for obs in observations]
        rows = []
        for meta, obs, state, action in zip(manifest['cases'], observations, states, actions):
            env = make_env(meta['record'])
            try:
                np.testing.assert_allclose(unpack_state(env, state), obs, atol=1e-6, rtol=0)
                trial, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                    action*env.r_max, 1., env._previous_n, structured=False)
                rows.append(dict(case=meta['case'], origin_arm=meta['origin_arm'],
                    action=action.tolist(), passed=bool(trial['feasible']),
                    candidate=env._json_safe_candidate(trial)))
            finally:
                env.close()
        result = dict(cases=len(rows), passed=sum(r['passed'] for r in rows), rows=rows,
                      checkpoint_sha256=sha(checkpoint), one_step_only=True)
        dump(destination, result)
        return result
    finally:
        torch.set_rng_state(cpu_rng); torch.cuda.set_rng_state(cuda_rng)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts', 'source', 'diagnosis', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); require_memory_limit()
    prepare(args.artifacts.resolve(), args.source.resolve(), args.diagnosis.resolve(), args.output.resolve())
