"""Offline geometry and file-safety tests; no SDK or camera required."""
import copy
import json
import math
from pathlib import Path
import stat
import tempfile
import unittest

import numpy as np

from calibration import (CalibrationError, calibrate, camera_wheel, matrix_pose, pose_matrix,
                         save_grasp, validate_grasp)


def transform(axis, angle, position):
    # Rodrigues construction independent of calibration's quaternion formulas.
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross
    result[:3, 3] = position
    return result


LIMITS = dict(max_joint_span_rad=0.005, max_position_span_m=0.002,
              max_rotation_span_rad=0.02, max_wheel_angle_span_rad=0.01,
              max_wheel_displacement_span_m=0.002)


def fixture(angle=0.3, displacement=-0.06):
    origin = transform([1, 2, -1], 0.8, [0.2, -0.3, 0.7])
    handles = {"left": transform([1, 3, 2], 0.4, [-0.14, 0.03, 0.42]),
               "right": transform([-2, 1, 3], -0.6, [0.14, -0.02, 0.41])}
    truth = {"left": transform([1, 2, 3], 0.9, [0.01, 0.05, 0.13]),
             "right": transform([2, -3, 1], -0.5, [-0.02, -0.04, 0.14])}
    grasp = {"wheel_origin": matrix_pose(origin),
             **{side: matrix_pose(t) for side, t in handles.items()},
             "tool": {"left": matrix_pose(np.eye(4)), "right": matrix_pose(np.eye(4)),
                      "radius": 0.028, "custom": {"untouched": [1, 2]}},
             "unrelated": {"preserve": True}, "approach_distance": 0.18}
    grasp["tool"]["left"]["note"] = "keep per-arm metadata"
    wheel = transform([0, 0, 1], angle, [0, 0, displacement])
    sample = {"angle_rad": angle, "displacement_m": displacement}
    for side in handles:
        flange = origin @ wheel @ handles[side] @ np.linalg.inv(truth[side])
        sample[side] = {"joint_position": [0.1 * j for j in range(7)], "flange": matrix_pose(flange)}
    return grasp, [copy.deepcopy(sample) for _ in range(5)], truth


class GeometryTests(unittest.TestCase):
    def test_camera_coordinates(self):
        self.assertEqual(camera_wheel(0.2, 0), (-0.2, -0.085))
        self.assertEqual(camera_wheel(0, -0.085), (0, 0))
        self.assertEqual(camera_wheel(0, 0.085), (0, -0.17))
        self.assertEqual(camera_wheel(-0.87266, 0), (0.87266, -0.085))
        for theta, translation in [(True, 0), (0, False), (float("nan"), 0),
                                   (0, float("inf")), (0.872661, 0), (0, 0.085001),
                                   (0, -0.085001), ("0.1", 0), (None, 0)]:
            with self.subTest(theta=theta, translation=translation), self.assertRaises(CalibrationError):
                camera_wheel(theta, translation)

    def test_wxyz_and_round_trip_at_pi(self):
        result = pose_matrix({"position": [1, 2, 3],
                              "quaternion": [math.sqrt(0.5), 0, math.sqrt(0.5), 0]})
        np.testing.assert_allclose(result[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-14)
        for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3]):
            for angle in (0, math.pi, math.pi - 1e-10, -1.6):
                expected = transform(axis, angle, [0.1, 0.2, 0.3])
                np.testing.assert_allclose(pose_matrix(matrix_pose(expected)), expected, atol=1e-14)

    def test_invalid_poses(self):
        for q in ([0, 0, 0, 0], [2, 0, 0, 0], [True, 0, 0, 0],
                  [1, 0, 0], [float("nan"), 0, 0, 0], ["1", 0, 0, 0]):
            with self.subTest(q=q), self.assertRaises(CalibrationError):
                pose_matrix({"position": [0, 0, 0], "quaternion": q})
        near_unit = pose_matrix({"position": [0, 0, 0], "quaternion": [1 + 1e-7, 0, 0, 0]})
        np.testing.assert_array_equal(near_unit, np.eye(4))
        for position in ([0, float("inf"), 0], [0, False, 0], [0, 0], None):
            with self.subTest(position=position), self.assertRaises(CalibrationError):
                pose_matrix({"position": position, "quaternion": [1, 0, 0, 0]})
        invalid = []
        m = np.eye(4); m[0, 0] = 2; invalid.append(m)
        m = np.eye(4); m[0, 0] = -1; invalid.append(m)
        m = np.eye(4); m[3, 0] = 1; invalid.append(m)
        invalid.extend([np.eye(3), np.full((4, 4), float("nan"))])
        for m in invalid:
            with self.subTest(matrix=m), self.assertRaises(CalibrationError):
                matrix_pose(m)

    def test_validate_config_before_collection(self):
        grasp, _, _ = fixture()
        self.assertIsNone(validate_grasp(grasp))
        for invalid in (None, [], {**grasp, "unrelated": float("nan")},
                        {**grasp, "wheel_origin": {"position": [0, 0, 0],
                                                  "quaternion": [0, 0, 0, 0]}},
                        {**grasp, "right": None}):
            with self.subTest(grasp=invalid), self.assertRaises(CalibrationError):
                validate_grasp(invalid)

    def test_recover_known_tools_at_several_wheel_poses(self):
        for angle, displacement in [(0.3, -0.06), (-0.6, -0.14), (0, 0)]:
            with self.subTest(angle=angle, displacement=displacement):
                grasp, samples, truth = fixture(angle, displacement)
                original = copy.deepcopy(grasp)
                saved_samples = copy.deepcopy(samples)
                updated, report = calibrate(grasp, samples, **LIMITS)
                self.assertEqual(grasp, original)
                self.assertEqual(samples, saved_samples)
                self.assertEqual(updated["unrelated"], original["unrelated"])
                self.assertEqual(updated["wheel_origin"], original["wheel_origin"])
                self.assertEqual(updated["left"], original["left"])
                self.assertEqual(updated["tool"]["custom"], original["tool"]["custom"])
                self.assertEqual(updated["tool"]["left"]["note"], "keep per-arm metadata")
                self.assertEqual(report["sample_count"], 5)
                self.assertAlmostEqual(report["wheel"]["mean_angle_rad"], angle)
                for side in truth:
                    np.testing.assert_allclose(pose_matrix(updated["tool"][side]), truth[side], atol=1e-14)
                    self.assertLess(report["arms"][side]["reconstruction_max_position_error_m"], 1e-14)
                    self.assertLess(report["arms"][side]["reconstruction_max_rotation_error_rad"], 1e-14)
                    self.assertAlmostEqual(report["arms"][side]["delta_position_norm_m"],
                                           float(np.linalg.norm(truth[side][:3, 3])))
                json.dumps(report, allow_nan=False)

    def test_quaternion_signs_do_not_change_mean(self):
        grasp, samples, truth = fixture()
        for i, sample in enumerate(samples):
            for side in ("left", "right"):
                if i % 2:
                    sample[side]["flange"]["quaternion"] = [-v for v in sample[side]["flange"]["quaternion"]]
        updated, _ = calibrate(grasp, samples, **LIMITS)
        for side in truth:
            np.testing.assert_allclose(pose_matrix(updated["tool"][side]), truth[side], atol=1e-14)

    def test_shared_tool_upgrade_and_invalid_mixture(self):
        grasp, samples, truth = fixture()
        grasp["tool"] = {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0], "mass": 0.15}
        updated, _ = calibrate(grasp, samples, **LIMITS)
        self.assertEqual(set(updated["tool"]), {"left", "right", "mass"})
        self.assertEqual(updated["tool"]["mass"], 0.15)
        for side in truth:
            np.testing.assert_allclose(pose_matrix(updated["tool"][side]), truth[side], atol=1e-14)
        grasp["tool"]["left"] = matrix_pose(np.eye(4))
        with self.assertRaises(CalibrationError):
            calibrate(grasp, samples, **LIMITS)
        grasp["tool"]["right"] = matrix_pose(np.eye(4))
        with self.assertRaises(CalibrationError):
            calibrate(grasp, samples, **LIMITS)

    def test_motion_rejected(self):
        for kind in ("joint", "position", "rotation", "angle", "displacement"):
            with self.subTest(kind=kind):
                grasp, samples, _ = fixture()
                last = samples[-1]
                if kind == "joint":
                    last["left"]["joint_position"][3] += 0.01
                elif kind == "position":
                    last["right"]["flange"]["position"][1] += 0.003
                elif kind == "rotation":
                    rotated = pose_matrix(last["left"]["flange"]) @ transform([0, 1, 0], 0.03, [0, 0, 0])
                    last["left"]["flange"] = matrix_pose(rotated)
                elif kind == "angle":
                    last["angle_rad"] += 0.02
                else:
                    last["displacement_m"] += 0.003
                with self.assertRaisesRegex(CalibrationError, "not static"):
                    calibrate(grasp, samples, **LIMITS)

    def test_small_noise_has_reported_residual(self):
        grasp, samples, truth = fixture()
        samples[0]["left"]["flange"]["position"][0] += 0.001
        updated, report = calibrate(grasp, samples, **LIMITS)
        self.assertAlmostEqual(report["arms"]["left"]["position_span_m"], 0.001)
        residual = report["arms"]["left"]["reconstruction_max_position_error_m"]
        self.assertAlmostEqual(residual, 0.0008)
        np.testing.assert_allclose(pose_matrix(updated["tool"]["right"]), truth["right"], atol=1e-14)

    def test_invalid_sample_data_and_limits(self):
        grasp, samples, _ = fixture()
        for invalid_samples in ([], samples[:2], [None, *samples], {}):
            with self.subTest(samples=invalid_samples), self.assertRaises(CalibrationError):
                calibrate(grasp, invalid_samples, **LIMITS)
        for field, value in [("angle_rad", True), ("angle_rad", 1), ("displacement_m", -0.18)]:
            modified = copy.deepcopy(samples)
            modified[-1][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(CalibrationError):
                calibrate(grasp, modified, **LIMITS)
        for value in ([0] * 6, [0] * 6 + [True], [0] * 6 + [float("nan")]):
            modified = copy.deepcopy(samples)
            modified[-1]["right"]["joint_position"] = value
            with self.subTest(q=value), self.assertRaises(CalibrationError):
                calibrate(grasp, modified, **LIMITS)
        for value in (-1, True, float("inf"), "1"):
            bad_limits = dict(LIMITS, max_joint_span_rad=value)
            with self.subTest(limit=value), self.assertRaises(CalibrationError):
                calibrate(grasp, samples, **bad_limits)


class SaveTests(unittest.TestCase):
    def test_exact_backup_atomic_json_and_permissions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp.json"
            original = b'{ "preserve exact spacing": true }\n'
            path.write_bytes(original)
            path.chmod(0o640)
            replacement = {"left": [1, 2, 3], "unicode": "标定"}
            backup = save_grasp(path, replacement, original)
            self.assertEqual(backup.read_bytes(), original)
            self.assertEqual(json.loads(path.read_text()), replacement)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o640)
            self.assertEqual(stat.S_IMODE(backup.stat().st_mode), 0o640)
            self.assertEqual(len(list(Path(directory).iterdir())), 2)

    def test_stale_file_rejected_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp.json"
            path.write_bytes(b'{"external":true}')
            with self.assertRaisesRegex(CalibrationError, "changed"):
                save_grasp(path, {"overwrite": True}, b"{}")
            self.assertEqual(path.read_bytes(), b'{"external":true}')
            self.assertEqual(len(list(Path(directory).iterdir())), 1)

    def test_nonfinite_missing_or_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp.json"
            path.write_bytes(b"{}")
            with self.assertRaises(CalibrationError):
                save_grasp(path, {"bad": float("nan")}, b"{}")
            self.assertEqual(path.read_bytes(), b"{}")
            with self.assertRaises(CalibrationError):
                save_grasp(path.with_name("missing.json"), {}, b"{}")
            link = path.with_name("symlink.json")
            link.symlink_to(path)
            with self.assertRaises(CalibrationError):
                save_grasp(link, {}, b"{}")
            self.assertEqual(path.read_bytes(), b"{}")


if __name__ == "__main__":
    unittest.main()
