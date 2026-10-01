import math
import unittest

import numpy as np

from geometry import (average_pose_matrices, calibrate_from_poses, evaluate_pose,
                      matrix_to_pose, pose_to_matrix)


def transform(axis, angle, translation):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    r = np.array([
        [c + x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)],
    ])
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = translation
    return out


class GeometryTests(unittest.TestCase):
    def test_pose_round_trip(self):
        matrix = transform([0, 0, 1], .4, [0.1, -0.2, 1.2])
        self.assertTrue(np.allclose(pose_to_matrix(matrix_to_pose(matrix)), matrix))

    def test_calibration_and_validation(self):
        zero = np.eye(4)
        second = transform([0, 0, 1], math.radians(30), [0.02, 0.0, 0.1])
        calibration = calibrate_from_poses(zero, second)
        self.assertTrue(np.allclose(np.abs(calibration["axis_direction"]), [0, 0, 1]))
        self.assertAlmostEqual(abs(calibration["rotation_angle_rad"]), math.radians(30), places=6)
        self.assertAlmostEqual(calibration["translation_along_axis_m"], .1, places=6)
        result = evaluate_pose(zero, second, calibration["axis_direction"])
        self.assertAlmostEqual(result["translation_along_axis_m"], .1, places=6)

    def test_pure_translation_uses_translation_axis(self):
        zero = np.eye(4)
        second = np.eye(4)
        second[:3, 3] = [0, .2, 0]
        calibration = calibrate_from_poses(zero, second)
        self.assertTrue(np.allclose(np.abs(calibration["axis_direction"]), [0, 1, 0]))
        self.assertAlmostEqual(calibration["translation_along_axis_m"], .2, places=6)
        self.assertAlmostEqual(calibration["rotation_angle_rad"], 0, places=6)

    def test_pose_batch_averages_translation_and_orientation(self):
        matrices = []
        for index in range(5):
            matrices.append(transform([0, 0, 1], .4 + (index - 2) * 1e-4,
                                      [.1 + index * 1e-3, .2, 1.0]))
        average = average_pose_matrices(matrices)
        self.assertTrue(np.allclose(average[:3, 3], [.102, .2, 1.0]))
        self.assertAlmostEqual(calibrate_from_poses(np.eye(4), average)["rotation_angle_rad"], .4, places=5)

    def test_validation_recomputes_axis_and_compares_with_calibration(self):
        zero = np.eye(4)
        current = transform([0, 0, 1], math.radians(30), [0.02, 0.0, 0.1])
        result = evaluate_pose(zero, current, [0, 0, 1])
        self.assertTrue(np.allclose(result["measured_axis_direction"], [0, 0, 1]))
        self.assertAlmostEqual(result["axis_error_rad"], 0.0, places=8)

        mismatched = transform([0, 1, 0], math.radians(30), [0.0, 0.02, 0.1])
        result = evaluate_pose(zero, mismatched, [0, 0, 1])
        self.assertTrue(np.allclose(result["measured_axis_direction"], [0, 1, 0]))
        self.assertAlmostEqual(result["axis_error_rad"], math.pi / 2, places=8)

    def test_validation_reports_unknown_axis_without_motion(self):
        result = evaluate_pose(np.eye(4), np.eye(4), [0, 0, 1])
        self.assertIsNone(result["measured_axis_direction"])
        self.assertIsNone(result["axis_error_rad"])

    def test_validation_reports_unknown_axis_for_small_rotation_noise(self):
        current = transform([0, 1, 0], math.radians(0.3), [-0.004, -0.001, 0.001])
        result = evaluate_pose(np.eye(4), current, [0, 0, 1])
        self.assertIsNone(result["measured_axis_direction"])
        self.assertIsNone(result["axis_error_rad"])
        self.assertAlmostEqual(result["rotation_magnitude_rad"], math.radians(0.3))

    def test_validation_does_not_use_translation_as_rotation_axis(self):
        current = np.eye(4)
        current[:3, 3] = [0.2, 0, 0]
        result = evaluate_pose(np.eye(4), current, [0, 0, 1])
        self.assertIsNone(result["measured_axis_direction"])
        self.assertIsNone(result["axis_error_rad"])

    def test_axis_threshold_uses_full_rotation_instead_of_projected_theta(self):
        current = transform([0, 1, 0], math.radians(30), [0, 0, 0])
        result = evaluate_pose(np.eye(4), current, [0, 0, 1])
        self.assertAlmostEqual(result["rotation_angle_rad"], 0.0)
        self.assertAlmostEqual(result["rotation_magnitude_rad"], math.radians(30))
        self.assertAlmostEqual(result["axis_error_rad"], math.pi / 2)

    def test_axis_threshold_is_configurable(self):
        current = transform([0, 0, 1], math.radians(2), [0, 0, 0])
        self.assertIsNone(evaluate_pose(np.eye(4), current, [0, 0, 1])["axis_error_rad"])
        result = evaluate_pose(np.eye(4), current, [0, 0, 1],
                               min_axis_rotation_rad=math.radians(1))
        self.assertAlmostEqual(result["axis_error_rad"], 0.0)

    def test_axis_comparison_handles_canonical_sign_flip(self):
        stored = np.array([0, 0.71, -0.70])
        measured = np.array([0, 0.70, -0.71])
        current = transform(measured, math.radians(30), [0, 0, 0])
        result = evaluate_pose(np.eye(4), current, stored)
        expected = math.acos(abs(np.dot(stored, measured)) /
                             (np.linalg.norm(stored) * np.linalg.norm(measured)))
        self.assertAlmostEqual(result["axis_error_rad"], expected)
        self.assertGreater(np.dot(result["measured_axis_direction"], stored), 0)

    def test_validation_keeps_signed_angle_for_reverse_and_half_turn(self):
        for angle in [math.radians(30), math.radians(-30), math.pi]:
            with self.subTest(angle=angle):
                current = transform([0, 0, 1], angle, [0, 0, 0])
                result = evaluate_pose(np.eye(4), current, [0, 0, 1])
                self.assertAlmostEqual(result["rotation_angle_rad"], angle)
                self.assertAlmostEqual(result["rotation_magnitude_rad"], abs(angle))
                self.assertAlmostEqual(result["axis_error_rad"], 0.0)

    def test_camera_frame_axis_with_offset_and_nonidentity_zero_pose(self):
        zero = transform([1, 2, 3], 0.7, [-0.03, -0.13, 0.81])
        axis = np.array([-0.0158, -0.6139, 0.7893])
        axis /= np.linalg.norm(axis)
        point = np.array([0.1, 0.3, 0.2])
        relative = transform(axis, math.radians(-20), [0, 0, 0])
        relative[:3, 3] = (np.eye(3) - relative[:3, :3]) @ point + 0.04 * axis
        current = relative @ zero
        calibration = calibrate_from_poses(zero, current)
        result = evaluate_pose(zero, current, axis)
        self.assertTrue(np.allclose(calibration["axis_direction"], axis))
        self.assertTrue(np.allclose(result["relative_transform"], relative))
        self.assertAlmostEqual(result["rotation_angle_rad"], math.radians(-20))
        self.assertAlmostEqual(result["translation_along_axis_m"], 0.04)
        self.assertAlmostEqual(result["axis_error_rad"], 0.0)
        self.assertGreater(result["translation_perpendicular_norm_m"], 0.05)


if __name__ == "__main__":
    unittest.main()
