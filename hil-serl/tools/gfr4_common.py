"""Controlled continuations of saved SAC checkpoints; original runs stay frozen."""
from pathlib import Path
import numpy as np
import torch
from tools.gfr3_sac import SAC, DemoReplay, OnlineReplay
from tools.gfr3_train_sac_demo import fingerprint

KEYS = ('observations', 'actions', 'rewards', 'next_observations', 'terminals')


def require_memory_limit():
    group = next(line.split(':', 2)[2] for line in Path('/proc/self/cgroup').read_text().splitlines()
                 if line.startswith('0::'))
    directory = Path('/sys/fs/cgroup')/group.lstrip('/')
    limit = (directory/'memory.max').read_text().strip()
    if limit == 'max' or int(limit) > 12*1024**3:
        raise RuntimeError('Requires systemd MemoryMax <= 12G')
    return directory


def restore(artifact_root, source_root, initialization='bc', step=60000):
    checkpoint = Path(artifact_root)/initialization/f'sac_{step:06d}.pt'
    learner = SAC(Path(source_root)/'shared_bc/actor_round_2.pt', initialization, 11)
    saved = torch.load(checkpoint, map_location=learner.device, weights_only=True)
    for name in ('actor', 'q1', 'q2', 'target1', 'target2'):
        getattr(learner, name).load_state_dict(saved[name])
    with torch.no_grad():
        learner.log_alpha.copy_(saved['log_alpha'])
    for name, key in (('actor_opt', 'actor_optimizer'), ('q_opt', 'critic_optimizer'),
                      ('alpha_opt', 'alpha_optimizer')):
        getattr(learner, name).load_state_dict(saved[key])
    learner.updates = int(saved['gradient_updates'])
    learner.critic_warmup_updates = int(saved['critic_warmup_updates'])
    torch.set_rng_state(saved['torch_rng'].cpu())
    torch.cuda.set_rng_state(saved['cuda_rng'].cpu())
    identities = {name: fingerprint(getattr(learner, name))
                  for name in ('actor', 'q1', 'q2', 'target1', 'target2')}
    return learner, identities


def demonstrations(root, device):
    return DemoReplay(sorted((Path(root)/'demonstrations').glob('*.npz')), device)


def replay_prefix(root, initialization, device, capacity=120008):
    replay = OnlineReplay(capacity, device, 11)
    with np.load(Path(root)/initialization/'online_replay.npz') as data:
        replay.add(*(data[key][:60000].copy() for key in KEYS))
    assert replay.size == 60000 and replay.position == 60000
    return replay


class FixedReplaySampler:
    """Same prefix only. Stratification intentionally changes fitting weights."""
    def __init__(self, replay, mode, seed=11):
        if mode not in ('uniform', 'stratified'):
            raise ValueError(mode)
        self.replay, self.mode = replay, mode
        self.rng = np.random.default_rng(seed)
        rejected = replay.arrays[2][:replay.size, 0] < -9
        assert np.all(replay.arrays[4][:replay.size, 0][rejected] == 1)
        self.failures = np.flatnonzero(rejected)
        self.other = np.flatnonzero(~rejected)
        if not len(self.failures) or not len(self.other):
            raise ValueError('Both replay strata required')

    def indices(self, count=128):
        if self.mode == 'uniform':
            return self.rng.integers(self.replay.size, size=count)
        failures = count//4
        return np.r_[self.rng.choice(self.failures, failures, replace=True),
                     self.rng.choice(self.other, count-failures, replace=True)]

    def sample(self, count=128):
        indices = self.indices(count)
        return tuple(torch.as_tensor(a[indices], device=self.replay.device)
                     for a in self.replay.arrays)
