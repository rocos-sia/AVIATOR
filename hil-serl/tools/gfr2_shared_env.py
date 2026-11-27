"""Installation/grasp-conditioned direct proposals. No policy-side true model.

128 observations: normalized state 40, calibrated relative geometry 24,
transported null axes 28, prescribed trajectory preview 8, joint margins 28.
Geometry is cached per installation; preview uses the commanded trajectory.
"""
import gymnasium as gym
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from tools.gfr1_direct_only_env import DirectOnlyEnv
from tools.gfr1_remount_evaluate import ROOT

BASES = ("AR5-5_07L-W4C4A2_base", "AR5-5_07R-W4C4A2_base")
HANDLES = ("left_handle", "right_handle")


def relative_geometry(model):
    data = mujoco.MjData(model)
    # Neutral task pose defines the installation frame, not the rotating rim.
    data.qpos[:] = 0.
    mujoco.mj_forward(model, data)
    wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "steering_wheel")
    poses = []
    for name in BASES:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        rotation = data.xmat[index].reshape(3, 3)
        poses.append((rotation.T @ (data.xpos[wheel]-data.xpos[index]),
                      rotation.T @ data.xmat[wheel].reshape(3, 3)))
    for name in HANDLES:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        poses.append((model.site_pos[index].copy(),
                      Rotation.from_quat(model.site_quat[index][[1, 2, 3, 0]]).as_matrix()))
    return poses


def geometry_features(poses, reference):
    result = []
    for (p, r), (p0, r0) in zip(poses, reference):
        result.extend((p-p0)/.005)
        result.extend(Rotation.from_matrix(r0.T @ r).as_rotvec()/np.deg2rad(1.))
    return np.asarray(result, np.float32)


class SharedProposalEnv(DirectOnlyEnv):
    def __init__(self, *args, **kwargs):
        self._geometry_reference = None
        self._geometry_features = np.zeros(24, np.float32)
        super().__init__(*args, **kwargs)
        nominal = mujoco.MjModel.from_xml_path(str(ROOT/"reference/rocos-mujoco/model/aviator.xml"))
        self._geometry_reference = relative_geometry(nominal)
        self._refresh_geometry()
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (128,), np.float32)

    def _refresh_geometry(self):
        if self._geometry_reference is not None:
            self._geometry_features = geometry_features(relative_geometry(self.fk.model), self._geometry_reference)

    def _set_model(self, path):
        changed = getattr(self, "model_path", None) != str(path)
        super()._set_model(path)
        if changed:
            self._refresh_geometry()

    def _get_obs(self):
        original = super()._get_obs()
        preview = []
        for seconds in (.1, .3):
            x, velocity = self._path_state(self._u+seconds/self.traj_dt)
            preview.extend((x-self._x)/[1., .1])
            preview.extend(velocity/[1., .1])
        lower = self.lookup.joint_lower.ravel()
        upper = self.lookup.joint_upper.ravel()
        margins = np.r_[self._q-lower, upper-self._q]/np.pi
        result = np.r_[original[:40], self._geometry_features,
                       self._last_lift_basis.ravel(), preview, margins].astype(np.float32)
        if result.shape != (128,) or not np.isfinite(result).all():
            raise ValueError("Invalid shared proposal observation")
        self.last_policy_observation = result.copy()
        return result
