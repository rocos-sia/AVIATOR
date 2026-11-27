"""The sole curriculum intervention in the retention factorial."""
from unittest.mock import patch
from tools import gfr3_curriculum_env as original
from tools.gfr3_curriculum_env import curriculum as original_schedule
from tools.gfr1_train_direct_only import install_guards


def schedule(mode, step):
    if mode == 'hold':
        return 256, .25  # Curriculum immediately before the original 60k evaluation.
    if mode == 'advance':
        return original_schedule(step)
    raise ValueError(mode)


class ControlledCurriculumEnv(original.CurriculumEnv):
    def __init__(self, source, demos, output, rank, workers, mode):
        self.curriculum_mode = mode
        super().__init__(source, demos, output, rank, workers)
        self.local_transitions = 60000//workers

    def reset(self, *, seed=None, options=None):
        chosen = schedule(self.curriculum_mode, self.local_transitions*self.workers)
        with patch.object(original, 'curriculum', return_value=chosen):
            return super().reset(seed=seed, options=options)


def make_worker(source, demos, output, rank, workers, mode):
    install_guards()
    return ControlledCurriculumEnv(source, demos, output, rank, workers, mode)


def bc_weight(mode, step):
    if mode == 'fixed':
        return .14
    if mode == 'decay':
        return .2-.15*max(0., min(1., (step-20000)/100000.))
    raise ValueError(mode)
