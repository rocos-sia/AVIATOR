"""Geometry and execution invariants for the new shared proposal interface."""
import unittest
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from tools.gfr1_remount_evaluate import ROOT
from tools.gfr2_shared_env import relative_geometry, geometry_features, BASES, HANDLES


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=mujoco.MjModel.from_xml_path(str(ROOT/"reference/rocos-mujoco/model/aviator.xml"))

    def test_common_world_transform_does_not_change_relative_encoding(self):
        model=self.model
        reference=relative_geometry(model)
        body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"aircraft")
        oldp,oldq=model.body_pos[body].copy(),model.body_quat[body].copy()
        try:
            model.body_pos[body]=[.2,-.3,.1]
            model.body_quat[body]=Rotation.from_rotvec([.1,.2,-.1]).as_quat()[[3,0,1,2]]
            np.testing.assert_allclose(geometry_features(relative_geometry(model),reference),0.,atol=1e-5)
        finally:
            model.body_pos[body]=oldp
            model.body_quat[body]=oldq

    def test_independent_base_motion_changes_its_condition(self):
        model=self.model
        reference=relative_geometry(model)
        body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,BASES[0])
        old=model.body_pos[body].copy()
        try:
            model.body_pos[body,0]+=.0005
            feature=geometry_features(relative_geometry(model),reference)
            self.assertGreater(np.linalg.norm(feature[:6]),.01)
            np.testing.assert_allclose(feature[6:],0.,atol=1e-6)
        finally:
            model.body_pos[body]=old

    def test_left_grasp_change_is_visible_without_changing_right(self):
        model=self.model
        reference=relative_geometry(model)
        site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,HANDLES[0])
        old=model.site_pos[site].copy()
        try:
            model.site_pos[site,0]+=.0005
            feature=geometry_features(relative_geometry(model),reference)
            self.assertGreater(np.linalg.norm(feature[12:18]),.01)
            np.testing.assert_allclose(feature[:12],0.,atol=1e-6)
            np.testing.assert_allclose(feature[18:],0.,atol=1e-6)
        finally:
            model.site_pos[site]=old


if __name__=="__main__":
    unittest.main()
