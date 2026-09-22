"""Aviator v0.1 redundancy-policy Gym environment (Task 1.3, BLOCKER #9/#12).

Doc: docs/superpowers/plans/2026-09-22-aviator-v01-rl-training.md (Task 1.3)

The policy controls the per-arm *phase velocity* ``phi_dot = (phi_dot_L,
phi_dot_R)``; the task variables ``x = (theta, s)`` follow a fixed reference
trajectory (the "task controller" is assumed perfect).  Each ``step`` runs the
full chain

    phi_dot_nom = action * phi_dot_scale
    phi_dot_safe = project_phi_dot(phi_dot_nom, x, x_next, phi, lookup)   # hard shield
    phi_next = phi + phi_dot_safe * dt
    q_ref = Q(x_next, phi_next)        (the manifold point IS the task projection)
    q_ref <- clamp to joint limits
    x <- x_next,  phi <- phi_next

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
from .reward import reward_fn
from .safety_filter import project_phi_dot

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

        self._hist_x = deque(maxlen=21)
        self._hist_xdot = deque(maxlen=21)
        self._hist_a = deque(maxlen=21)

    # -- trajectory loading --------------------------------------------------
    def _load_trajectory(self) -> dict:
        if self._trajectories is not None:
            traj = self._trajectories[self.np_random.integers(len(self._trajectories))]
            return {k: np.asarray(v, dtype=np.float64) for k, v in traj.items()}
        path = self._traj_paths[self.np_random.integers(len(self._traj_paths))]
        with np.load(path) as f:
            return {k: np.asarray(f[k], dtype=np.float64) for k in f.files}

    # -- reset / step --------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._traj = self._load_trajectory()
        self._t = 0
        self._x = self._traj["x"][0].copy()
        # Initialize phi to a *safe* configuration at x[0].  The transported
        # anchor phase phi=0 is only safe near (theta=0, s=0); elsewhere the
        # continuation of the anchor config can collide (the per-arm safe
        # interval does not contain 0).  Project 0 onto the per-arm safe box so
        # the episode starts feasible -- the policy's job is to keep it feasible.
        box = self.lookup.safe_interval(self._x[None, :])
        self._phi = np.clip(
            np.zeros(2),
            box["phi_safe_lo"][0],
            box["phi_safe_hi"][0],
        )
        self._a_prev = np.zeros(2)

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
        action = np.clip(action, -1.0, 1.0)
        phi_dot_nom = action * self.phi_dot_scale

        x = self._x
        x_next = self._traj["x"][self._t + 1]
        phi = self._phi

        # --- hard shield ----------------------------------------------------
        phi_dot_safe, filt = project_phi_dot(
            phi_dot_nom, x, x_next, phi, self.lookup,
            qdot_max=self.qdot_max, dt=self.dt, d_safe=self.d_safe,
        )

        info = {"filter": filt, "phi_dot_nom": phi_dot_nom, "phi_dot_safe": phi_dot_safe}

        if not filt["feasible"]:
            # safety filter could not find a safe step -> terminate
            self._a_prev = action
            obs = self._get_obs()
            return obs, 0.0, True, True, info

        # --- advance (kinematic execution) ----------------------------------
        phi_next = phi + phi_dot_safe * self.dt
        # check_safe=False: the safety filter's hard shield already verified the
        # *trilinear* clearance d >= d_safe (authoritative); the bilinear safe
        # interval in safe.bin can disagree with it by interpolation error at the
        # boundary, so re-checking it here would spuriously reject safe steps.
        # The explicit d_min < d_safe termination check below is the real guard.
        res_next = self.lookup.query(x_next[None, :], phi_next[None, :], check_safe=False)
        q_ref = np.concatenate([res_next["qL"][0], res_next["qR"][0]])
        # clamp to joint limits (the manifold point is the task projection)
        lo = np.concatenate([self.lookup.joint_lower[0], self.lookup.joint_lower[1]])
        hi = np.concatenate([self.lookup.joint_upper[0], self.lookup.joint_upper[1]])
        q_ref = np.clip(q_ref, lo, hi)

        res_cur = self.lookup.query(x[None, :], phi[None, :], check_safe=False)
        q_t = np.concatenate([res_cur["qL"][0], res_cur["qR"][0]])
        q_t = np.clip(q_t, lo, hi)   # previous actual config is the *clamped* q_ref
        q_dot_actual = (q_ref - q_t) / self.dt
        max_qdot = float(np.max(np.abs(q_dot_actual)))

        d_min = float(res_next["d_min"][0])

        # --- advance state --------------------------------------------------
        self._t += 1
        self._x = x_next.copy()
        self._phi = phi_next.copy()
        self._a_prev = action
        self._hist_x.append(self._x)
        self._hist_xdot.append(self._traj["xdot"][self._t])
        self._hist_a.append(action)

        # --- termination ----------------------------------------------------
        terminated = False
        truncated = False
        if d_min < self.d_safe:
            terminated = True
            info["termination"] = "collision"
        elif max_qdot > self.qdot_max:
            terminated = True
            info["termination"] = "speed"
        elif self._t >= len(self._traj["x"]) - 1:
            truncated = True
            info["termination"] = "end"
        done = terminated or truncated

        # --- reward ---------------------------------------------------------
        reward, rinfo = reward_fn(
            x_next, phi_dot_nom, phi_dot_safe, d_min,
            res_next["m_phi_minus"][0], res_next["m_phi_plus"][0], res_next["m_q"][0],
            self.lookup,
            a_prev=self._a_prev, a_prev2=None, dt=self.dt,
        )
        info.update(rinfo)
        info.update({"d_min": d_min, "max_qdot": max_qdot})

        obs = self._get_obs()
        return obs, reward, terminated, truncated, info

    # -- observation ---------------------------------------------------------
    def _get_obs(self) -> dict:
        res = self.lookup.query(self._x[None, :], self._phi[None, :], check_safe=False)
        d_min = float(res["d_min"][0])
        m_q = float(res["m_q"][0])
        m_minus = res["m_phi_minus"][0]
        m_plus = res["m_phi_plus"][0]

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
