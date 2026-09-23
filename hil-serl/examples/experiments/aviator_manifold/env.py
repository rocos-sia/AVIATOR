"""Aviator v0.1 redundancy-policy Gym environment (Task 1.3, BLOCKER #9/#12).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

The policy controls the per-arm phase velocity; task variables follow a fixed
reference trajectory. Actions are executed without a safety projection:

    phi_dot_exec = action * phi_dot_scale
    phi_next = phi + phi_dot_exec * dt
    q_next = Q(x_next, phi_next)

Clearance, branch, joint and velocity violations terminate with a hard cost.
The lookup is a kinematic approximation, not a fresh physics collision check.

Execution is *kinematic*: ``q`` tracks ``q_ref`` exactly and the clearance is the
manifold's stored ``d_min`` (MuJoCo collision result baked into ``manifold_phi``).
The physical ``mj_step`` position servo is a deployment detail, not part of the
learning env -- v0.1 models perfect tracking.

Observation (40-D, closed-loop phase encoding ``[sin phi, cos phi]``):
    [0:2] x   [2:4] xdot
    [4:6] sin(phi_L), cos(phi_L)   [6:8] sin(phi_R), cos(phi_R)
    [8:12] m_phi_minus, m_phi_plus   [12] d_min   [13] m_q
    [14:16] a_prev   [16:24] hist_x   [24:32] hist_xdot   [32:40] hist_a
The unwrapped ``phi`` is internal only (never reaches the MLP).
"""

from __future__ import annotations

import glob
import os
from collections import deque

import gymnasium as gym
import numpy as np

from .manifold_lookup import ManifoldLookup
from .reward import reward_fn, TERMINAL_PENALTY

# frozen v0.1 constants
_DT = 0.01
_QDOT_MAX = 1.5
_D_SAFE = 0.005
_LAGS = (2, 5, 10, 20)
_OBS_DIM = 40


class AviatorManifoldEnv(gym.Env):
    """Learned redundancy policy over the transported phase manifold."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        manifold_dir: str,
        trajectory_dir: str = None,
        split: str = "dp_train",
        phi_dot_scale: float = 1.0,
        dt: float = _DT,
        qdot_max: float = _QDOT_MAX,
        d_safe: float = _D_SAFE,
        trajectories: list = None,
    ):
        super().__init__()
        self.lookup = ManifoldLookup(manifold_dir)
        self.trajectory_dir = trajectory_dir
        self.split = split
        self.phi_dot_scale = phi_dot_scale
        self.dt = dt
        self.qdot_max = qdot_max
        self.d_safe = d_safe

        # explicit trajectories override disk loading (tests, rollouts)
        self._trajectories = trajectories
        if self._trajectories is None:
            self._traj_paths = sorted(glob.glob(os.path.join(
                trajectory_dir, "trajs", split, "*.npz")))

        self.observation_space = gym.spaces.Dict(
            {"state": gym.spaces.Box(-np.inf, np.inf, shape=(_OBS_DIM,), dtype=np.float32)}
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        # per-episode state
        self._traj = None
        self._t = 0
        self._x = None
        self._phi = None
        self._a_prev = np.zeros(2)
        # Previous executed velocity; the first step has no acceleration cost.
        self._phi_dot_prev = None

        self._hist_x = deque(maxlen=21)
        self._hist_xdot = deque(maxlen=21)
        self._hist_a = deque(maxlen=21)

    # -- trajectory loading --------------------------------------------------
    def _load_trajectory(self, index=None) -> dict:
        if self._trajectories is not None:
            chosen = self.np_random.integers(len(self._trajectories)) if index is None else index
            traj = self._trajectories[chosen]
            return {k: np.asarray(v, dtype=np.float64) for k, v in traj.items()}
        chosen = self.np_random.integers(len(self._traj_paths)) if index is None else index
        path = self._traj_paths[chosen]
        with np.load(path) as f:
            return {k: np.asarray(f[k], dtype=np.float64) for k in f.files}

    # -- reset / step --------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        trajectory_index = None if options is None else options.get("trajectory_index")
        self._traj = self._load_trajectory(trajectory_index)
        self._t = 0
        self._x = self._traj["x"][0].copy()
        # A registered demonstration starts from its own canonical LUT phase.
        # Keep the default online reset policy when no phase is supplied.
        initial_phi = None if options is None else options.get("initial_phi")
        if initial_phi is None:
            box = self.lookup.safe_interval(self._x[None, :])
            self._phi = np.clip(np.zeros(2), box["phi_safe_lo"][0],
                                box["phi_safe_hi"][0])
        else:
            initial_phi = np.asarray(initial_phi, dtype=np.float64).reshape(2)
            if (not np.all(np.isfinite(initial_phi)) or
                    np.any(initial_phi < self.lookup.phi_axis[0]) or
                    np.any(initial_phi > self.lookup.phi_axis[-1])):
                raise ValueError("initial_phi must be finite and inside the LUT phase grid")
            self._phi = initial_phi.copy()
        self._a_prev = np.zeros(2)
        self._phi_dot_prev = None

        self._hist_x.clear()
        self._hist_xdot.clear()
        self._hist_a.clear()
        x0 = self._x.copy()
        xd0 = self._traj["xdot"][0].copy() if "xdot" in self._traj else np.zeros(2)
        a0 = np.zeros(2)
        for _ in range(max(_LAGS) + 1):
            self._hist_x.append(x0)
            self._hist_xdot.append(xd0)
            self._hist_a.append(a0)

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(2)
        if not np.all(np.isfinite(action)) or np.any(np.abs(action) > 1.0):
            raise ValueError("action must be finite and within [-1, 1]; it is never clipped")
        phi_dot_exec = action * self.phi_dot_scale

        x = self._x
        x_next = self._traj["x"][self._t + 1]
        phi = self._phi

        phi_next = phi + phi_dot_exec * self.dt
        info = {"phi_dot_exec": phi_dot_exec.copy()}
        # The lookup cannot represent a phase outside its grid. Treat that
        # candidate as a failed action instead of projecting it onto the edge.
        try:
            res_next = self.lookup.query(x_next[None, :], phi_next[None, :], check_safe=False)
        except ValueError:
            res_next = None
        prev_phi_dot = self._phi_dot_prev

        # --- advance state --------------------------------------------------
        self._t += 1
        self._x = x_next.copy()
        self._phi = phi_next.copy()
        self._a_prev = action
        self._phi_dot_prev = phi_dot_exec.copy()
        self._hist_x.append(self._x)
        self._hist_xdot.append(self._traj["xdot"][self._t])
        self._hist_a.append(action)

        if res_next is None:
            info.update(termination="grid_exit", d_min=float("nan"),
                        max_qdot=float("nan"), m_q=float("nan"))
            return self._get_obs(allow_invalid=True), TERMINAL_PENALTY, True, False, info

        q_next = np.concatenate([res_next["qL"][0], res_next["qR"][0]])
        lo = np.concatenate([self.lookup.joint_lower[0], self.lookup.joint_lower[1]])
        hi = np.concatenate([self.lookup.joint_upper[0], self.lookup.joint_upper[1]])
        res_cur = self.lookup.query(x[None, :], phi[None, :], check_safe=False)
        q_cur = np.concatenate([res_cur["qL"][0], res_cur["qR"][0]])
        max_qdot = float(np.max(np.abs((q_next - q_cur) / self.dt)))
        d_min = float(res_next["d_min"][0])
        m_q = float(res_next["m_q"][0])

        # --- termination ----------------------------------------------------
        terminated = False
        truncated = False
        if not np.all(np.isfinite(q_next)) or not np.isfinite(d_min):
            terminated = True
            info["termination"] = "invalid_lookup"
        elif int(res_next["branch"][0]) >= 2:
            # Branch codes 2 (unreachable chart) and 3 (cycle-inconsistent) name genuinely
            # corrupt φ charts. Code 1 (loop-open) is a short-but-valid self-motion arc near
            # the wrist singularity and is still clearance/joint/speed-checked below.
            terminated = True
            info["termination"] = "branch"
        elif d_min < self.d_safe:
            terminated = True
            info["termination"] = "clearance"
        elif np.any(q_next < lo) or np.any(q_next > hi):
            terminated = True
            info["termination"] = "joint_limit"
        elif max_qdot > self.qdot_max:
            terminated = True
            info["termination"] = "speed"
        elif self._t >= len(self._traj["x"]) - 1:
            truncated = True
            info["termination"] = "end"
        reward, rinfo = reward_fn(
            x_next, phi_dot_exec, phi_dot_exec, d_min,
            res_next["m_phi_minus"][0], res_next["m_phi_plus"][0], m_q,
            self.lookup,
            a_prev=prev_phi_dot,
            a_prev2=None, dt=self.dt, d_safe=self.d_safe,
        )
        info.update(rinfo)
        info.update({"d_min": d_min, "max_qdot": max_qdot, "m_q": m_q,
                     "branch": int(res_next["branch"][0]),
                     "q": q_next, "x": x_next})

        # one-time hard penalty on terminal violations (d < d_safe or speed):
        # the shaped reward only rewards *surviving* steps, so the terminal step
        # pays TERMINAL_PENALTY instead -- the policy learns to avoid it.
        if terminated:
            reward = TERMINAL_PENALTY
            info["reward"] = reward

        obs = self._get_obs()
        return obs, reward, terminated, truncated, info

    # -- observation ---------------------------------------------------------
    def _get_obs(self, allow_invalid=False) -> dict:
        try:
            res = self.lookup.query(self._x[None, :], self._phi[None, :], check_safe=False)
        except ValueError:
            if not allow_invalid:
                raise
            res = None
        d_min = float(res["d_min"][0]) if res is not None else 0.0
        m_q = float(res["m_q"][0]) if res is not None else 0.0
        m_minus = res["m_phi_minus"][0] if res is not None else np.zeros(2)
        m_plus = res["m_phi_plus"][0] if res is not None else np.zeros(2)

        x = self._x
        xdot = self._traj["xdot"][self._t]

        def lag(hist, k):
            return hist[max(0, len(hist) - 1 - k)]

        obs = np.zeros(_OBS_DIM, dtype=np.float32)
        obs[0:2] = x
        obs[2:4] = xdot
        obs[4:6] = [np.sin(self._phi[0]), np.cos(self._phi[0])]
        obs[6:8] = [np.sin(self._phi[1]), np.cos(self._phi[1])]
        obs[8:10] = m_minus
        obs[10:12] = m_plus
        obs[12] = d_min
        obs[13] = m_q
        obs[14:16] = self._a_prev
        o = 16
        for k in _LAGS:
            obs[o:o + 2] = lag(self._hist_x, k)
            o += 2
        for k in _LAGS:
            obs[o:o + 2] = lag(self._hist_xdot, k)
            o += 2
        for k in _LAGS:
            obs[o:o + 2] = lag(self._hist_a, k)
            o += 2
        return {"state": obs}
