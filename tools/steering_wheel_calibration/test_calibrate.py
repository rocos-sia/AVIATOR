import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import calibrate
from geometry import matrix_to_pose
from test_geometry import transform
from validate import load_calibration


class CalibrationWriteTests(unittest.TestCase):
    def test_main_writes_calibration_loadable_by_validation(self):
        zero = np.eye(4)
        second = transform([0, 0, 1], math.radians(30), [0.02, 0.0, 0.1])

        def records(matrix):
            return [{"frame_id": 1, "sample_mono_us": 1000,
                     "confidence": 1.0, "pose": matrix_to_pose(matrix)}]

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "calibration.yaml"
            with (mock.patch.dict(sys.modules, {"zmq": mock.MagicMock()}),
                  mock.patch("builtins.input", return_value=""),
                  mock.patch("builtins.print"),
                  mock.patch.object(calibrate.time, "sleep"),
                  mock.patch.object(calibrate, "receive_poses", side_effect=[
                      (records(zero), zero), (records(second), second)])):
                self.assertEqual(calibrate.main(["--output", str(output), "--samples", "1"]), 0)

            result, stored_zero, axis = load_calibration(output)
            self.assertTrue(np.allclose(stored_zero, zero))
            self.assertTrue(np.allclose(axis, [0, 0, 1]))
            motion = result["motion"]
            self.assertTrue(np.allclose(
                motion["translation_perpendicular_between_samples_m"], [0.02, 0.0, 0.0]))
            self.assertAlmostEqual(motion["translation_perpendicular_norm_m"], 0.02)
            self.assertAlmostEqual(motion["translation_along_axis_between_samples_m"], 0.1)
            self.assertAlmostEqual(motion["rotation_between_samples_rad"], math.radians(30))


if __name__ == "__main__":
    unittest.main()
