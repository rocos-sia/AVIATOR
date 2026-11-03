"""Opt-in training backend for the frozen combined executor repair.

Geometry selection is shared with the audited recovery regression. Historical
backend files, observations, rewards, and nonlinear tolerances are untouched.
Only sequential environment stepping is supported (the repair scopes a module
function override during midpoint bound compensation).
"""
from __future__ import annotations
import json
import os
import time
from pathlib import Path
import numpy as np
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.gfr1_recovery_boundary_ablation import select_action
from tools.gfr1_same_state_diagnosis import safe


class RepairedGFRManifoldEnv(GFRManifoldEnv):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.mode != 'structured':
            raise ValueError('Frozen combined repair requires structured GFR actions')
        self._repair_episode = -1
        self._repair_stream = None

    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        self._repair_episode += 1
        self._repair_cycle = 0
        self._repair_previous_raw = np.zeros(2, dtype=np.float32)
        index = (options or {}).get('trajectory_index')
        # Match the frozen regression for explicit evaluation indices. Training
        # uses a separate episode RNG; recovery never changes reset sampling.
        self._repair_seed = (20261102 + int(index) * 10000 if index is not None
                             else 20261102 + int(seed or 0)*10000 + self._repair_episode*100000)
        return obs, info

    def _failed_step(self, raw, candidates):
        """Preserve the original failure reward/terminal semantics, not the
        diagnostic runner's zero-reward stop. No rejected motion is executed.
        """
        self._last_intervention = self._last_rejection = self._last_recovery = 1.0
        info = dict(candidates[-1]) if candidates else {'reason': 'no_candidate'}
        info = self._json_safe_candidate(info)
        components = dict(progress=0., intervention=-self.w_intervention,
                          recovery=-self.w_recovery,
                          compute=-self.w_compute * len(candidates),
                          terminal=self.terminal_failure_penalty)
        info.update(termination='governor_exhausted', candidate_count=len(candidates),
                    candidate_trace=[self._json_safe_candidate(c) for c in candidates],
                    policy_action_raw=raw.tolist(), action_nominal_r=(raw*self.r_max).tolist(),
                    action_executed_r=[0., 0.], beta_g=0., beta_executed=0.,
                    progress=float(self._u/max(len(self._traj['x'])-1, 1)),
                    task_progress_delta=0., endpoint_validation=False, swept_validation=False,
                    intervened=True, rejected=True, recovered=False,
                    reward_components=components, reward=float(sum(components.values())),
                    x=self._x.copy(), q=self._q.copy())
        info['d_min'] = info.get('d_actual_m', np.nan)
        info['max_qdot'] = info.get('qdot_max_rad_s', np.nan)
        return self._get_obs(), info['reward'], True, False, info

    def step(self, action, precomputed=None):
        if precomputed is not None:
            raise ValueError('Repaired training backend owns candidate validation')
        raw = np.asarray(action, dtype=np.float64).reshape(2)
        if not np.isfinite(raw).all() or np.any(np.abs(raw) > 1.):
            raise ValueError('action must be finite and within [-1,1]')
        started = time.perf_counter()
        chosen, history, policy_exhausted = select_action(
            self, raw, self._repair_previous_raw, boundary=True, alternative=True,
            rng=np.random.default_rng(self._repair_seed + self._repair_cycle))
        # Identical to the frozen full-start regression. Extra midpoint
        # correction attempts are counted separately; reward weights and the
        # original candidate-count reward definition remain unchanged.
        candidates = [h['trials'][-1]['result'] for h in history]
        trials = [t['result'] for h in history for t in h['trials']]
        if chosen is None:
            result = self._failed_step(raw, candidates)
        else:
            result = super().step(raw, {'accepted': chosen[1], 'candidates': candidates})
            self._repair_previous_raw = np.asarray(chosen[0], dtype=np.float32).copy()
        obs, reward, terminated, truncated, info = result
        info.update(executor='combined_repair_v1', alternative_recovered=bool(policy_exhausted and chosen is not None),
                    policy_schedule_exhausted=bool(policy_exhausted),
                    boundary_correction_iterations=sum(len(h['trials'])-1 for h in history),
                    actual_candidate_evaluations=len(trials),
                    actual_endpoint_failed_candidates=sum(bool(t.get('endpoint_check_evaluated')) and not t.get('endpoint_validation', False) for t in trials),
                    actual_swept_failed_candidates=sum(bool(t.get('swept_check_evaluated')) and not t.get('swept_validation', False) for t in trials),
                    selected_action_raw=chosen[0].tolist() if chosen is not None else None,
                    executor_wall_s=time.perf_counter()-started)
        self._repair_cycle += 1
        directory = os.getenv('AVIATOR_REPAIRED_TRACE_DIR')
        if directory:
            if self._repair_stream is None:
                Path(directory).mkdir(parents=True, exist_ok=True)
                self._repair_stream = (Path(directory)/f'pid{os.getpid()}_env{id(self)}.jsonl').open('x', buffering=1)
            keys = ('policy_action_raw','action_nominal_r','action_executed_r','selected_action_raw',
                    'beta_executed','progress','task_progress_delta','termination','d_actual_m',
                    'endpoint_validation','swept_validation','joint_limit_margin_rad','qdot_max_rad_s',
                    'intervened','rejected','alternative_recovered','policy_schedule_exhausted',
                    'boundary_correction_iterations','actual_candidate_evaluations',
                    'actual_endpoint_failed_candidates','actual_swept_failed_candidates',
                    'reward_components','executor_wall_s')
            row = {k: info.get(k) for k in keys}
            row.update(episode=self._repair_episode, step=self._repair_cycle, model=self.model_path)
            self._repair_stream.write(json.dumps(safe(row), allow_nan=False)+'\n')
        return obs, reward, terminated, truncated, info

    def close(self):
        if self._repair_stream is not None:
            self._repair_stream.close()
            self._repair_stream = None
        return super().close()
