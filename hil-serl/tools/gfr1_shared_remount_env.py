"""Shared-installation policy interface with explicit, arm-local action axes.

At reset positive action 0 increases the left arm's third joint along its unit
null direction, and action 1 does the same for the right arm. Each arm's axis is
then transported continuously. Its current joint-space direction is also in
the observation, so the policy never has to infer the orientation from history.
All geometry supplied here is the controller's estimate, never an audit model.
"""
from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from tools.gfr1_remount_evaluate import EvaluationEnv, ROOT
from tools.validate_online_lift import analytic_constraint_matrices

INSTALL_BODIES = ("steering_wheel", "AR5-5_07L-W4C4A2_base",
                  "AR5-5_07R-W4C4A2_base", "aviator_wall_left", "aviator_wall_right")


def arm_local_basis(aq):
    if np.max(np.abs(aq[:6, 7:])) > 1e-10 or np.max(np.abs(aq[6:, :7])) > 1e-10:
        raise ValueError("Arm-local action chart requires separate arm constraint blocks")
    basis = np.zeros((14, 2))
    minimum_component = 1.
    for side in range(2):
        block = aq[side*6:(side+1)*6, side*7:(side+1)*7]
        _, singular, vh = np.linalg.svd(block, full_matrices=True)
        if singular[-1] < 1e-8:
            raise ValueError("Rank deficient closure Jacobian")
        vector = vh[-1]
        minimum_component = min(minimum_component, abs(vector[2]))
        if abs(vector[2]) < 1e-5:
            raise ValueError("Arm-local action chart is singular at joint 3")
        basis[side*7:(side+1)*7, side] = vector * (1 if vector[2] > 0 else -1)
    return basis, minimum_component


class SharedRemountEnv(EvaluationEnv):
    def __init__(self, *args, **kwargs):
        self._nominal_installation = None
        self._install_features = np.zeros(30, dtype=np.float32)
        self.minimum_chart_component = 1.
        self.last_policy_observation = None
        self._last_lift_basis = np.zeros((14, 2))
        super().__init__(*args, **kwargs)
        # Read reference transforms from the compiled nominal model once. Its
        # body indices/names are checked explicitly; no episode model is truth.
        nominal = mujoco.MjModel.from_xml_path(str(ROOT/"reference/rocos-mujoco/model/aviator.xml"))
        self._nominal_installation = self._poses(nominal)
        del nominal
        self._update_installation()
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(98,), dtype=np.float32)

    @staticmethod
    def _poses(model):
        result = []
        for name in INSTALL_BODIES:
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if index < 0:
                raise ValueError("Missing installation body: "+name)
            # These bodies are world children or children of the fixed aircraft
            # frame. The experiment never changes that reference frame.
            result.append((model.body_pos[index].copy(), model.body_quat[index].copy()))
        return result

    def _update_installation(self):
        if self._nominal_installation is None:
            return
        features = []
        for (p, q), (p0, q0) in zip(self._poses(self.fk.model), self._nominal_installation):
            rotation = Rotation.from_quat(q[[1,2,3,0]])
            reference = Rotation.from_quat(q0[[1,2,3,0]])
            features.extend((p-p0)/.005)
            features.extend((reference.inv()*rotation).as_rotvec()/np.deg2rad(1.))
        self._install_features = np.asarray(features, dtype=np.float32)

    def _set_model(self, model_path):
        # Limit per-worker memory to one cached model; geometry changes only
        # between episodes. Never evict a model during a measured control cycle.
        if getattr(self, "model_path", None) != str(model_path):
            self._model_cache.clear()
        super()._set_model(model_path)
        self._update_installation()

    def _lift(self, x, q, xdot, previous_n):
        error, aq, ax = analytic_constraint_matrices(self.fk, x, q)
        if previous_n is None:
            basis, component = arm_local_basis(aq)
            self.minimum_chart_component = min(self.minimum_chart_component, component)
        else:
            basis = np.zeros((14, 2))
            for side in range(2):
                _, values, vh = np.linalg.svd(aq[side*6:(side+1)*6, side*7:(side+1)*7], full_matrices=True)
                if values[-1] < 1e-8:
                    raise ValueError("Rank deficient closure Jacobian")
                direction = vh[-1]
                overlap = float(direction @ previous_n[side*7:(side+1)*7, side])
                if abs(overlap) < 1e-4:
                    raise ValueError("Ambiguous null-axis transport")
                basis[side*7:(side+1)*7, side] = direction * (1 if overlap > 0 else -1)
        self._last_lift_basis = basis.copy()
        singular = np.linalg.svd(aq, compute_uv=False)
        inverse = np.linalg.pinv(aq, rcond=1e-10)
        return error, basis, inverse @ (-2.*error), inverse @ (-ax@xdot), float(singular[-1])

    def _get_obs(self):
        state = super()._get_obs()["state"].copy()
        # Fixed physical scales, frozen across training/development/test.
        scales = np.ones(40, dtype=np.float32)
        scales[0:4] = [1., .1, 1., .1]
        scales[4:18] = np.pi
        scales[18:30] = np.tile([.001]*3+[np.deg2rad(1.)]*3, 2)
        scales[30:33] = [.01, 1., .1]
        scales[33:35] = self.r_max
        self.last_policy_observation = np.r_[state/scales, self._install_features, self._last_lift_basis.ravel()].astype(np.float32)
        return self.last_policy_observation.copy()

