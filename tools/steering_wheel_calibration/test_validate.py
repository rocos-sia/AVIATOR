import contextlib
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from calibrate import write_document
from geometry import matrix_to_pose
from test_geometry import transform
import validate


class ValidationStreamTests(unittest.TestCase):
    def test_stream_reports_unknown_match_and_mismatch(self):
        stored_axis = np.array([-0.0158, -0.6139, 0.7893])
        stored_axis /= np.linalg.norm(stored_axis)
        poses = [
            transform([-0.0447, 0.7936, 0.6068], math.radians(0.4),
                      [-0.004452, -0.001316, 0.001186]),
            transform(stored_axis, math.radians(-15), [0, 0, 0]),
            transform(np.cross(stored_axis, [1, 0, 0]), math.radians(30), [0, 0, 0]),
            transform(stored_axis, math.radians(2), [0, 0, 0]),
        ]
        messages = [[b"camera.detection", json.dumps({
            "msg_type": "CameraDetection", "status": "TRACKING", "valid": True,
            "camera_id": "cockpit", "tag_id": 0, "frame_id": index,
            "pose": matrix_to_pose(pose),
        }).encode()] for index, pose in enumerate(poses)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.yaml"
            write_document(path, {"steering_wheel_calibration": {
                "status": "calibrated", "source": {"camera_id": "cockpit", "tag_id": 0},
                "zero": {"pose": {"T_camera_tag": np.eye(4).tolist()}},
                "motion": {"axis_direction": stored_axis.tolist()},
            }})
            for threshold_args, last_match in [([], None), (["--min-axis-rotation-deg", "1"], True)]:
                with self.subTest(threshold_args=threshold_args):
                    zmq = mock.MagicMock()
                    sub = zmq.Context.return_value.socket.return_value
                    sub.poll.return_value = 1
                    sub.recv_multipart.side_effect = messages
                    stdout = io.StringIO()
                    with (mock.patch.dict(sys.modules, {"zmq": zmq}),
                          contextlib.redirect_stdout(stdout)):
                        self.assertEqual(validate.main([
                            "--config", str(path), "--json", "--max-msgs", "4", *threshold_args]), 0)
                    results = [json.loads(line) for line in stdout.getvalue().splitlines()]
                    self.assertIsNone(results[0]["axis_match"])
                    self.assertIsNone(results[0]["axis_direction_measured"])
                    self.assertAlmostEqual(results[0]["rotation_magnitude_deg"], 0.4)
                    self.assertIs(results[1]["axis_match"], True)
                    self.assertAlmostEqual(results[1]["theta_deg"], -15)
                    self.assertIs(results[2]["axis_match"], False)
                    self.assertAlmostEqual(results[2]["rotation_magnitude_deg"], 30)
                    self.assertIs(results[3]["axis_match"], last_match)
                    sub.close.assert_called_once_with(0)
                    zmq.Context.return_value.term.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
