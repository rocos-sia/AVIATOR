"""RL proposes r; one beta=1 lift/check; reject terminates without correction.

No QP, governor, backtracking, alternative-action search, or rejected-state
commit. The analytic closed-chain lift and independent acceptance criteria
define the action/environment interface, not a corrective policy.
"""
import numpy as np
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.gfr1_shared_remount_env import SharedRemountEnv


class DirectOnlyEnv(SharedRemountEnv):
    def action_step(self, raw, direct=True):
        if not direct:
            raise ValueError("Projection is disabled in DirectOnlyEnv")
        raw = np.asarray(raw, dtype=np.float64)
        if raw.shape != (2,) or not np.isfinite(raw).all() or np.any(np.abs(raw)>1):
            raise ValueError("Expected finite normalized RL action in [-1, 1]^2")
        candidate, _ = self._candidate(self._x, self._q, self._path_state(self._u)[1],
            raw*self.r_max, 1., self._previous_n, structured=False)
        passed = bool(candidate["feasible"])
        if passed:
            result = GFRManifoldEnv.step(self, raw, {"accepted":candidate, "candidates":[candidate]})
            self._repair_cycle += 1
            self._repair_previous_raw = raw.astype(np.float32)
        else:
            # Do not reuse the repaired executor's failure helper: it would
            # label the event as governor exhaustion/recovery despite no calls.
            self._last_intervention = 0.
            self._last_rejection = 1.
            self._last_recovery = 0.
            info = self._json_safe_candidate(candidate)
            info.update(termination="direct_rejected", candidate_count=1,
                actual_candidate_evaluations=1, policy_action_raw=raw.tolist(),
                action_nominal_r=(raw*self.r_max).tolist(), action_executed_r=[0.,0.],
                beta_executed=0., beta_g=0., task_progress_delta=0.,
                progress=float(self._u/max(len(self._traj["x"])-1,1)),
                intervened=False, rejected=True, recovered=False,
                x=self._x.copy(), q=self._q.copy())
            result = self._get_obs(), -1000., True, False, info
        result[-1].update(executor="direct_only", projection_calls=0,
            governor_calls=0, recovery_calls=0, fallback_steps=0)
        return result, passed, candidate

    def step(self, action):
        previous = self._r_prev/self.r_max
        (obs, _, terminated, truncated, info), passed, candidate = self.action_step(action)
        components = dict(progress=1000.*info["task_progress_delta"], attempt=-.01,
            action_change=-.01*float(np.sum((np.asarray(action)-previous)**2)),
            terminal=100. if info["termination"]=="end" else (-1000. if not passed else 0.))
        reward = float(sum(components.values()))
        if info["termination"]=="end":
            terminated, truncated = True, False
        info.update(reward=reward, reward_components=components)
        self._last_direct_candidate = candidate
        self._last_direct_accepted = passed
        return obs, reward, terminated, truncated, info

    def method_step(self, method, policy=None):
        if method != "R" or policy is None:
            raise ValueError("DirectOnlyEnv evaluation requires method R and a frozen policy")
        result = self.step(policy(self))
        return result, self._last_direct_accepted, self._last_direct_candidate

    def legacy_qp_step(self):
        raise RuntimeError("QP/governor disabled in DirectOnlyEnv")

    def previous_action_step(self, direct):
        raise RuntimeError("Use step() with the RL proposal, without an action substitute")
