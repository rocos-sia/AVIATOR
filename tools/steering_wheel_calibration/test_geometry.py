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


if __name__ == "__main__":
    unittest.main()
