#!/usr/bin/env python3
"""Use the same repaired backend in the existing SAC actor/learner and evaluator.

The override is process-local. No historical experiment registration, model,
checkpoint or source file is rewritten.
"""
from pathlib import Path
import runpy
import sys


def main():
    stage = sys.argv.pop(1)
    if stage not in ('train', 'eval'):
        raise ValueError('stage must be train or eval')
    if stage == 'train' and any(x.startswith(('--bc_checkpoint_path', '--initial_checkpoint_path', '--sac_init_checkpoint', '--resume')) for x in sys.argv[1:]):
        raise ValueError('This experiment trains from scratch; checkpoint initialization is forbidden')
    from experiments.aviator_gfr import config
    from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
    config.GFRManifoldEnv = RepairedGFRManifoldEnv
    root = Path(__file__).resolve().parents[1]
    target = root / ('examples/train_rlpd.py' if stage == 'train' else 'tools/gfr1_repaired_eval.py')
    print('executor=combined_repair_v1; initialization=random SAC; stage='+stage, flush=True)
    sys.argv[0] = str(target)
    runpy.run_path(str(target), run_name='__main__')


if __name__ == '__main__':
    main()
