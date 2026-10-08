"""Check URDF/MJCF visuals, kinematics, mimic constraints and lossless STL chunks."""
import importlib.util
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = ET.parse(ROOT / "models/urdf/aviator.urdf").getroot()
        cls.xml = ET.parse(ROOT / "models/mjcf/aviator.xml").getroot()
        cls.model = mj.MjModel.from_xml_path(str(ROOT / "models/mjcf/aviator.xml"))

    def test_visuals_and_lossless_chunks(self):
        assets = {mesh.get("name"): mesh for mesh in self.xml.findall("asset/mesh")}
        bodies = {body.get("name"): body for body in self.xml.findall(".//body")}
        for link in self.urdf.findall("link"):
            for i, visual in enumerate(link.findall("visual")):
                name = link.get("name")
                geoms = bodies[name].findall("geom")
                geoms = [g for g in geoms if g.get("name", "").startswith(name + "_visual_")]
                mesh = visual.find("geometry/mesh")
                source = (ROOT / "models/urdf" / mesh.get("filename")).resolve()
                expected_scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                paths = []
                for geom in geoms:
                    asset = assets[geom.get("mesh")]
                    paths.append((ROOT / "models/meshes" / asset.get("file")).resolve())
                    np.testing.assert_allclose(
                        np.fromstring(asset.get("scale", "1 1 1"), sep=" "), expected_scale)
                if name == "aircraft":
                    original = source.read_bytes()
                    chunks = [path.read_bytes() for path in paths]
                    self.assertEqual(b"".join(chunk[84:] for chunk in chunks), original[84:])
                    self.assertEqual(sum(struct.unpack_from("<I", c, 80)[0] for c in chunks),
                                     struct.unpack_from("<I", original, 80)[0])
                    for chunk in chunks:
                        count = struct.unpack_from("<I", chunk, 80)[0]
                        self.assertLessEqual(count, 200000)
                        self.assertEqual(len(chunk), 84 + 50 * count)
                else:
                    self.assertEqual(paths, [source], name)
                for geom in geoms:
                    material = visual.find("material")
                    if material is not None and material.get("name"):
                        self.assertEqual(geom.get("material"), material.get("name"), name)
        for name in ("aircraft", "shell", "steering_wheel_2"):
            self.assertFalse(bodies[name].findall("geom[@class='collision']"))
        self.assertFalse(self.xml.findall(".//geom[@group='1']"))
        self.assertFalse([name for name in assets if name.startswith("realsense_d436_mesh")])
        self.assertEqual(self.model.nu, 0)
        self.assertEqual(self.model.ncam, 2)

    def test_joints_and_mimics(self):
        equalities = {e.get("joint1"): e for e in self.xml.findall("equality/joint")}
        for joint in self.urdf.findall("joint"):
            if joint.get("type") == "fixed":
                continue
            actual = self.model.joint(joint.get("name"))
            expected_type = mj.mjtJoint.mjJNT_SLIDE if joint.get("type") == "prismatic" else mj.mjtJoint.mjJNT_HINGE
            self.assertEqual(self.model.jnt_type[actual.id], expected_type)
            limit = joint.find("limit")
            np.testing.assert_allclose(self.model.jnt_range[actual.id],
                                       [float(limit.get("lower")), float(limit.get("upper"))])
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            np.testing.assert_allclose(self.model.jnt_axis[actual.id], axis / np.linalg.norm(axis))
            mimic = joint.find("mimic")
            if mimic is not None:
                equality = equalities[joint.get("name")]
                self.assertEqual(equality.get("joint2"), mimic.get("joint"))
                np.testing.assert_allclose(np.fromstring(equality.get("polycoef"), sep=" "),
                                           [float(mimic.get("offset", "0")),
                                            float(mimic.get("multiplier", "1")), 0, 0, 0])
        data = mj.MjData(self.model)
        mj.mj_resetDataKeyframe(self.model, data, self.model.key("aviator_home").id)
        for joint in self.urdf.findall("joint"):
            mimic = joint.find("mimic")
            if mimic is not None:
                q = data.qpos[self.model.joint(joint.get("name")).qposadr]
                leader = data.qpos[self.model.joint(mimic.get("joint")).qposadr]
                np.testing.assert_allclose(q, float(mimic.get("offset", "0")) +
                                           float(mimic.get("multiplier", "1")) * leader)

    def test_apriltag_physical_dimensions(self):
        config = yaml.safe_load((ROOT / "config/camera.yaml").read_text(encoding="utf-8"))
        self.assertEqual(config["apriltag"]["family"], "tag36h11")
        self.assertEqual(config["apriltag"]["tag_id"], 0)
        self.assertAlmostEqual(config["apriltag"]["tag_size_m"], .12)
        tag = self.model.geom("yoke_apriltag").id
        np.testing.assert_allclose(self.model.geom_size[tag], [.075, .075, .0005], atol=1e-12)
        np.testing.assert_allclose(self.model.geom_size[tag, :2] * 2 * .8,
                                   config["apriltag"]["tag_size_m"], atol=1e-12)
        self.assertTrue((ROOT / "docs/Apriltag120id0.pdf").is_file())

    def test_forward_kinematics(self):
        model = self.model
        data = mj.MjData(model)
        for roll, pitch in [(0, -.085), (-.87266, -.17), (.87266, 0)]:
            mj.mj_resetDataKeyframe(model, data, model.key("aviator_home").id)
            for suffix in ("", "_2"):
                data.qpos[model.joint("roll_input_joint" + suffix).qposadr] = roll
                data.qpos[model.joint("pitch_input_joint" + suffix).qposadr] = pitch
            mj.mj_forward(model, data)
            poses = {"aircraft": np.eye(4)}
            remaining = list(self.urdf.findall("joint"))
            while remaining:
                ready = [j for j in remaining if j.find("parent").get("link") in poses]
                self.assertTrue(ready, "URDF graph has an unresolved parent or cycle")
                for joint in ready:
                    pose = poses[joint.find("parent").get("link")] @ origin(joint.find("origin"))
                    if joint.get("type") != "fixed":
                        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                        q = float(data.qpos[model.joint(joint.get("name")).qposadr][0])
                        motion = np.eye(4)
                        if joint.get("type") == "prismatic":
                            motion[:3, 3] = axis * q
                        else:
                            motion[:3, :3] = rotation(axis, q)
                        pose = pose @ motion
                    poses[joint.find("child").get("link")] = pose
                    remaining.remove(joint)
            for name, pose in poses.items():
                body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
                if name in ("dummy_link", "dummy_link_2"):
                    self.assertEqual(body, -1)  # Coaxial massless bodies are combined.
                    continue
                self.assertGreaterEqual(body, 0, name)
                # Existing arm mounts use rounded quaternions in MJCF.
                np.testing.assert_allclose(data.xpos[body], pose[:3, 3], atol=2e-4, err_msg=name)
                np.testing.assert_allclose(data.xmat[body].reshape(3, 3), pose[:3, :3], atol=2e-4, err_msg=name)

    def test_second_wheel_constraint_stability(self):
        model = self.model
        data = mj.MjData(model)
        mj.mj_resetDataKeyframe(model, data, model.key("aviator_home").id)
        mj.mj_step(model, data, nstep=500)
        self.assertFalse(data.warning.number.any())
        self.assertTrue(np.isfinite(data.qpos).all())
        for name in ("roll_input_joint", "pitch_input_joint"):
            leader = data.qpos[model.joint(name).qposadr]
            follower = data.qpos[model.joint(name + "_2").qposadr]
            np.testing.assert_allclose(follower, leader, atol=1e-4)


class SplitSTLTests(unittest.TestCase):
    def test_split_validation_and_overwrite_protection(self):
        spec = importlib.util.spec_from_file_location("split_stl", ROOT / "scripts/split_stl.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "mesh.STL"
            source.write_bytes(b"invalid")
            with self.assertRaises(ValueError):
                module.split_stl(source, root / "parts")
            source.write_bytes(bytes(80) + struct.pack("<I", 1) + bytes(49))
            with self.assertRaises(ValueError):
                module.split_stl(source, root / "parts")
            source.write_bytes(bytes(80) + struct.pack("<I", 1) + bytes(50))
            outputs = module.split_stl(source, root / "parts")
            self.assertEqual(outputs[0].read_bytes(), source.read_bytes())
            with self.assertRaises(FileExistsError):
                module.split_stl(source, root / "parts")


if __name__ == "__main__":
    unittest.main()
