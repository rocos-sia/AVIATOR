"""GFR-1 fixed-duration environment with the online closed-chain backend.

This module is intentionally separate from ``env.py``.  The historical
``AviatorManifoldEnv`` is a fast LUT/kinematic environment and does not call
the nonlinear endpoint/swept checker.  GFR-1 must use the same backend as the
online governor, so this environment integrates the local differential lift
from a manifold initial state and lets the governor own ``beta``.

The actor action is always the two-vector ``r=[r_L,r_R]``.  A structured mode
projects it into the fixed-beta linear slice before the independent endpoint
and swept checks.  A raw mode skips that projection but keeps the same
governor/checker.  The latter is the B1 control needed to measure the value of
the structured action interface.
"""

from __future__ import annotations

import glob
import json
import os
from pathlib import Path

import gymnasium as gym
import numpy as np

from .manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.validate_branch_connector import (
    clearance_gradient,
    project_r,
    swept_clearance,
)
from tools.validate_differential_lift import AviatorClosure
from tools.validate_online_lift import analytic_constraint_matrices, orient_basis


class GFRManifoldEnv(gym.Env):
    """Fixed-duration GFR-1 rollout environment.

    ``mode`` is one of ``raw``, ``structured`` or ``fixed``.  ``visible_error``
    controls whether the 12-D closure residual is exposed to the policy; the
    residual is always computed internally by the deterministic lift.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        manifold_dir: str,
        model_path: str,
        trajectory_dir: str,
        split: str = "rl_train",
        trajectories: list[dict] | None = None,
        mode: str = "structured",
        visible_error: bool = True,
        dt: float = 0.005,
        d_safe: float = 0.005,
        qdot_max: float = 1.5,
        beta_stop: float = 0.125,
        r_max: float = 1.5,
        w_progress: float = 10.0,
        w_intervention: float = 0.02,
        w_recovery: float = 0.05,
        w_compute: float = 0.001,
        terminal_reward: float = 100.0,
        terminal_failure_penalty: float = -100.0,
        model_manifest: str | None = None,
    ):
        super().__init__()
        if mode not in {"raw", "structured", "fixed"}:
            raise ValueError("mode must be raw, structured, or fixed")
        self.lookup = ManifoldLookup(manifold_dir)
        self.model_manifest = model_manifest
        self._model_cache = {}
        self.model_path = str(model_path)
        if model_manifest is not None:
            payload = json.loads(Path(model_manifest).read_text())
            self._model_records = payload.get("records", payload)
            if not self._model_records:
                raise ValueError("model_manifest contains no records")
            self._traj_paths = []
            for rec in self._model_records:
                if "trajectory_path" not in rec or "model" not in rec:
                    raise ValueError("each model manifest record needs trajectory_path and model")
                self._traj_paths.append(str(rec["trajectory_path"]))
        else:
            self._model_records = None
        self._set_model(self.model_path)
        self.trajectory_dir = trajectory_dir
        self.split = split
        self._trajectories = trajectories
        if self._model_records is None:
            self._traj_paths = sorted(glob.glob(os.path.join(
                trajectory_dir, "trajs", split, "*.npz"))) if trajectories is None else []
        self.mode = mode
        self.visible_error = bool(visible_error)
        self.dt = float(dt)
        self.traj_dt = 0.01
        self.d_safe = float(d_safe)
        self.qdot_max = float(qdot_max)
        self.beta_stop = float(beta_stop)
        self.r_max = float(r_max)
        # GFR-1 reward coefficients are fixed for the ablation.  They are
        # explicit constructor values rather than per-run tuning knobs so a
        # result cannot be improved by silently changing the objective.
        self.w_progress = float(w_progress)
        self.w_intervention = float(w_intervention)
        self.w_recovery = float(w_recovery)
        self.w_compute = float(w_compute)
        self.terminal_reward = float(terminal_reward)
        self.terminal_failure_penalty = float(terminal_failure_penalty)
        self.governor_schedule = (1.0, 0.75, 0.5, 0.25, 0.125)

        # x(2), xdot(2), q(14), e(12), clearance, joint margin, sigma,
        # previous r(2), beta, intervention, rejection, recovery, progress.
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Box(-np.inf, np.inf, shape=(40,), dtype=np.float32)
        })
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        self._traj = None
        self._u = 0.0
        self._x = None
        self._q = None
        self._previous_n = None
        self._r_prev = np.zeros(2, dtype=np.float64)
        self._beta_prev = 1.0
        self._last_intervention = 0.0
        self._last_rejection = 0.0
        self._last_recovery = 0.0
        self._last_sigma = 0.0

    def _set_model(self, model_path: str) -> None:
        """Switch all geometry-dependent objects together for one episode."""
        model_path = str(model_path)
        if model_path not in self._model_cache:
            self._model_cache[model_path] = (
                AviatorClosure(model_path), CollisionChecker(model_path)
            )
        self.model_path = model_path
        self.fk, self.checker = self._model_cache[model_path]

    def _load_trajectory(self, index=None):
        if self._model_records is not None:
            chosen = self.np_random.integers(len(self._model_records)) if index is None else index
            rec = self._model_records[int(chosen)]
            self._set_model(rec["model"])
            with np.load(rec["trajectory_path"]) as data:
                traj = {k: np.asarray(data[k], dtype=np.float64) for k in data.files}
            if "initial_q" in rec:
                traj["_initial_q"] = np.asarray(rec["initial_q"], dtype=np.float64)
            traj["_model"] = rec["model"]
            return traj
        if self._trajectories is not None:
            chosen = self.np_random.integers(len(self._trajectories)) if index is None else index
            return {k: np.asarray(v, dtype=np.float64) for k, v in self._trajectories[chosen].items()}
        chosen = self.np_random.integers(len(self._traj_paths)) if index is None else index
        with np.load(self._traj_paths[chosen]) as data:
            return {k: np.asarray(data[k], dtype=np.float64) for k in data.files}

    def _path_state(self, u: float):
        n = len(self._traj["x"])
        u = float(np.clip(u, 0.0, n - 1.0))
        i = min(int(np.floor(u)), n - 2)
        f = u - i
        x = (1.0 - f) * self._traj["x"][i] + f * self._traj["x"][i + 1]
        if "xdot" in self._traj:
            xd = (1.0 - f) * self._traj["xdot"][i] + f * self._traj["xdot"][i + 1]
        else:
            xd = (self._traj["x"][i + 1] - self._traj["x"][i]) / self.traj_dt
        return x, xd

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        index = None if options is None else options.get("trajectory_index")
        self._traj = self._load_trajectory(index)
        self._u = 0.0
        self._x, _ = self._path_state(self._u)
        box = self.lookup.safe_interval(self._x[None, :])
        phi = 0.5 * (box["phi_safe_lo"][0] + box["phi_safe_hi"][0])
        if "_initial_q" in self._traj:
            self._q = np.asarray(self._traj["_initial_q"], dtype=np.float64).copy()
        else:
            q0 = self.lookup.query(self._x[None, :], phi[None, :], check_safe=False)
            self._q = np.concatenate([q0["qL"][0], q0["qR"][0]]).astype(np.float64)
        self._previous_n = None
        self._r_prev = np.zeros(2, dtype=np.float64)
        self._beta_prev = 1.0
        self._last_intervention = 0.0
        self._last_rejection = 0.0
        self._last_recovery = 0.0
        self._last_sigma = 0.0
        return self._get_obs(), {}

    def _lift(self, x, q, xdot, previous_n):
        e, aq, ax = analytic_constraint_matrices(self.fk, x, q)
        _, singular, vh = np.linalg.svd(aq, full_matrices=True)
        n = orient_basis(vh[-2:].T, previous_n)
        pinv = np.linalg.pinv(aq, rcond=1e-10)
        h_e = pinv @ (-2.0 * e)
        h_x = pinv @ (-ax @ xdot)
        return e, n, h_e, h_x, float(singular[-1])

    def _candidate(self, x, q, xdot_nom, r_nom, beta, previous_n, structured):
        xdot = beta * xdot_nom
        # h_x is the lift of the nominal velocity; beta is applied once below.
        e, n, h_e, h_x, sigma = self._lift(x, q, xdot_nom, previous_n)
        h = h_e + beta * h_x
        x_next = x + self.dt * xdot
        if structured:
            r, info = project_r(
                self.checker, x, x_next, q, h, n, r_nom,
                np.concatenate([self.lookup.joint_lower[0], self.lookup.joint_lower[1]]),
                np.concatenate([self.lookup.joint_upper[0], self.lookup.joint_upper[1]]),
                self.dt, qdot_max=self.qdot_max, d_safe=self.d_safe,
                r_max=self.r_max,
            )
            if not info.get("feasible", False):
                failed = {
                    "sigma_min_Aq": sigma,
                    "reason": "linear_slice_or_endpoint_reject",
                    "endpoint_validation": False,
                    "swept_validation": False,
                    "endpoint_check_evaluated": bool("d_next" in info),
                    "swept_check_evaluated": bool("swept_min" in info),
                    **info,
                }
                return failed, failed
        else:
            r = np.zeros(2) if self.mode == "fixed" else np.asarray(r_nom, dtype=np.float64)
            info = {"feasible": True, "intervened": False, "backtracked": 0}

        # Midpoint re-evaluation keeps the endpoint/swept check independent of
        # the linear candidate generation.
        q_mid = q + 0.5 * self.dt * (h + n @ r)
        x_mid = x + 0.5 * self.dt * xdot
        e_mid, n_mid, h_e_mid, h_x_mid, sigma_mid = self._lift(x_mid, q_mid, xdot_nom, n)
        qdot_mid = h_e_mid + beta * h_x_mid + n_mid @ r
        q_next = q + self.dt * qdot_mid
        d_next, contact_next = self.checker.check(x_next, q_next)
        swept_min, swept_contact = swept_clearance(
            self.checker, x, q, x_next, q_next, subdivisions=5
        )
        lower = np.concatenate([self.lookup.joint_lower[0], self.lookup.joint_lower[1]])
        upper = np.concatenate([self.lookup.joint_upper[0], self.lookup.joint_upper[1]])
        limit = bool(np.any(q_next < lower - 1e-9) or np.any(q_next > upper + 1e-9))
        valid = bool(
            np.max(np.abs(qdot_mid)) <= self.qdot_max + 2e-5
            and not limit
            and not contact_next and not swept_contact
            and d_next >= self.d_safe - 1e-9
            and swept_min >= self.d_safe - 1e-9
        )
        info.update({
            "feasible": valid,
            "r_nom": np.asarray(r_nom, dtype=float).tolist(),
            "r_exec": np.asarray(r, dtype=float).tolist(),
            "beta": float(beta),
            "sigma_min_Aq": min(sigma, sigma_mid),
            "d_endpoint_m": float(d_next),
            "d_swept_m": float(swept_min),
            "d_actual_m": float(min(d_next, swept_min)),
            "closure_residual_inf": float(np.max(np.abs(self.fk.error(x_next, q_next)))),
            "qdot_max_rad_s": float(np.max(np.abs(qdot_mid))),
            "joint_limit_margin_rad": float(min(np.min(q_next - lower), np.min(upper - q_next))),
            "endpoint_contact": bool(contact_next),
            "swept_contact": bool(swept_contact),
            "endpoint_validation": bool(
                not contact_next and d_next >= self.d_safe - 1e-9
            ),
            "swept_validation": bool(
                not swept_contact and swept_min >= self.d_safe - 1e-9
            ),
            "endpoint_check_evaluated": True,
            "swept_check_evaluated": True,
            "n_next": n_mid,
            "q_next": q_next,
            "x_next": x_next,
            "e_next": self.fk.error(x_next, q_next),
        })
        if not valid:
            info["reason"] = "endpoint_validation"
        return info, info

    def step(self, action):
        action = np.asarray(action, dtype=np.float64).reshape(2)
        if not np.all(np.isfinite(action)) or np.any(np.abs(action) > 1.0):
            raise ValueError("action must be finite and within [-1,1]")
        r_nom = action * self.r_max
        xdot_nom = self._path_state(self._u)[1]
        candidates = []
        accepted = None
        for beta in self.governor_schedule:
            info, _ = self._candidate(
                self._x, self._q, xdot_nom, r_nom, beta,
                self._previous_n,
                structured=self.mode == "structured",
            )
            candidates.append(info)
            if info.get("feasible", False):
                accepted = info
                break

        previous_u = self._u
        if accepted is None:
            self._last_intervention = 1.0
            self._last_rejection = 1.0
            self._last_recovery = 1.0
            info = candidates[-1] if candidates else {"reason": "no_candidate"}
            info = dict(info)
            info.update({
                "termination": "governor_exhausted",
                "candidate_count": len(candidates),
                "candidate_trace": [self._json_safe_candidate(c) for c in candidates],
                "policy_action_raw": action.tolist(),
                "action_nominal_r": r_nom.tolist(),
                "action_executed_r": [0.0, 0.0],
                "beta_g": 0.0,
                "beta_executed": 0.0,
                "progress": float(self._u / max(len(self._traj["x"]) - 1, 1)),
                "task_progress_delta": 0.0,
            })
            info["d_min"] = info.get("d_actual_m", np.nan)
            info["max_qdot"] = info.get("qdot_max_rad_s", np.nan)
            info["endpoint_validation"] = False
            info["swept_validation"] = False
            info["x"] = self._x.copy()
            info["q"] = self._q.copy()
            reward_components = {
                "progress": 0.0,
                "intervention": -self.w_intervention,
                "recovery": -self.w_recovery,
                "compute": -self.w_compute * len(candidates),
                "terminal": self.terminal_failure_penalty,
            }
            info["reward_components"] = reward_components
            info["reward"] = float(sum(reward_components.values()))
            return self._get_obs(), info["reward"], True, False, info

        beta = float(accepted["beta"])
        self._x = np.asarray(accepted["x_next"], dtype=np.float64)
        self._q = np.asarray(accepted["q_next"], dtype=np.float64)
        self._u = min(self._u + beta * self.dt / self.traj_dt,
                      len(self._traj["x"]) - 1.0)
        self._previous_n = np.asarray(accepted["n_next"], dtype=np.float64)
        r_exec = np.asarray(accepted["r_exec"], dtype=np.float64)
        intervention = float(np.linalg.norm(r_exec - r_nom) > 1e-8 or beta < 1.0 - 1e-10)
        rejection = float(len(candidates) > 1 or not np.allclose(r_exec, r_nom))
        self._last_intervention = intervention
        self._last_rejection = rejection
        self._last_recovery = 0.0
        self._r_prev = r_exec
        self._beta_prev = beta
        self._last_sigma = float(accepted["sigma_min_Aq"])

        progress = float(self._u / max(len(self._traj["x"]) - 1, 1))
        old_progress = float(previous_u / max(len(self._traj["x"]) - 1, 1))
        delta_progress = progress - old_progress
        terminated = False
        truncated = False
        if self._u >= len(self._traj["x"]) - 1.0 - 1e-9:
            truncated = True
            accepted["termination"] = "end"
        reward_components = {
            "progress": self.w_progress * delta_progress,
            "intervention": -self.w_intervention * intervention,
            "recovery": -self.w_recovery * float(self._last_recovery),
            "compute": -self.w_compute * len(candidates),
            "terminal": self.terminal_reward if truncated else 0.0,
        }
        reward = float(sum(reward_components.values()))
        info = {
            **self._json_safe_candidate(accepted),
            "termination": "end" if truncated else "none",
            "candidate_count": len(candidates),
            "candidate_trace": [self._json_safe_candidate(c) for c in candidates],
            "policy_action_raw": action.tolist(),
            "action_nominal_r": r_nom.tolist(),
            "action_executed_r": r_exec.tolist(),
            "beta_g": beta,
            "beta_executed": beta,
            "progress": progress,
            "task_progress_delta": delta_progress,
            "intervened": bool(intervention),
            "rejected": bool(rejection),
            "recovered": False,
            "reward_components": reward_components,
            "reward": reward,
        }
        # Keep the legacy actor statistics names as aliases.  This lets the
        # isolated GFR runner reuse the vectorized actor without changing the
        # historical manifold environment or its result files.
        info["d_min"] = info.get("d_actual_m", np.nan)
        info["max_qdot"] = info.get("qdot_max_rad_s", np.nan)
        info["x"] = self._x.copy()
        info["q"] = self._q.copy()
        return self._get_obs(), float(reward), terminated, truncated, info

    @staticmethod
    def _json_safe_candidate(info):
        out = {}
        for key, value in info.items():
            if key in {"n_next", "q_next", "x_next", "e_next"}:
                continue
            if isinstance(value, np.ndarray):
                out[key] = value.tolist()
            elif isinstance(value, (np.floating, np.integer, np.bool_)):
                out[key] = value.item()
            else:
                out[key] = value
        return out

    def _get_obs(self):
        e = self.fk.error(self._x, self._q)
        d, _ = self.checker.check(self._x, self._q)
        qlo = np.concatenate([self.lookup.joint_lower[0], self.lookup.joint_lower[1]])
        qhi = np.concatenate([self.lookup.joint_upper[0], self.lookup.joint_upper[1]])
        qmargin = float(min(np.min(self._q - qlo), np.min(qhi - self._q)))
        _, _, _, _, sigma = self._lift(self._x, self._q, self._path_state(self._u)[1], self._previous_n)
        e_obs = e if self.visible_error else np.zeros_like(e)
        # Keep a mask so the B1/B2 distinction is explicit in the transition.
        state = np.r_[
            self._x,
            self._path_state(self._u)[1],
            self._q,
            e_obs,
            float(d), qmargin, float(sigma),
            self._r_prev, self._beta_prev,
            self._last_intervention, self._last_rejection,
            self._last_recovery,
            float(self._u / max(len(self._traj["x"]) - 1, 1)),
        ].astype(np.float32)
        if state.shape != (40,):
            raise RuntimeError(f"unexpected GFR observation shape {state.shape}")
        return {"state": state}
