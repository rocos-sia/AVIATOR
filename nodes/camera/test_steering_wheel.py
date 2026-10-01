"""Camera publication and standalone calibration compatibility, without hardware."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

from main import make_message
from detection_subscriber import fmt_steering_wheel
from steering_wheel import SteeringWheelObservation, steering_wheel_options
from tools.steering_wheel_calibration import calibrate, validate
from tools.steering_wheel_calibration.geometry import matrix_to_pose, quaternion_to_rotation


def transform(axis, angle, translation):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation([*(axis * math.sin(angle / 2)), math.cos(angle / 2)])
    matrix[:3, 3] = translation
    return matrix


class SteeringWheelTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "calibration.yaml"
        self.zero = transform([1, 2, 3], .7, [-.03, -.13, .81])
        self.axis = np.array([-.0158, -.6139, .7893])
        self.axis /= np.linalg.norm(self.axis)
        self.document = {"steering_wheel_calibration": {
            "status": "calibrated", "source": {"camera_id": "cockpit", "tag_id": 0,
                                                 "frame_id": "camera_color_optical_frame"},
            "zero": {"pose": {"T_camera_tag": self.zero.tolist()}},
            "motion": {"axis_direction": self.axis.tolist()}}}
        self.write_document()
        self.observation = SteeringWheelObservation(self.path)

    def write_document(self):
        self.path.write_text(yaml.safe_dump(self.document), encoding="utf-8")

    def message(self, pose=None, **overrides):
        options = dict(camera_id="cockpit", frame_id=1, sequence=1, sample_mono_us=1000000,
                       session="11111111-1111-4111-8111-111111111111", clock="host-boot",
                       width=1280, height=720, status="TRACKING", confidence=1.0,
                       pose=matrix_to_pose(self.zero) if pose is None else pose,
                       detector="apriltag", tag_id=0, steering_wheel=self.observation)
        options.update(overrides)
        return make_message(**options)

    def test_zero_and_signed_motion_in_radians_and_metres_preserve_raw_pose(self):
        zero = self.message()["steering_wheel"]
        self.assertTrue(zero["valid"])
        self.assertAlmostEqual(zero["theta_rad"], 0)
        self.assertAlmostEqual(zero["translation_along_axis_m"], 0)
        point = np.array([.1, .3, .2])
        for angle, distance in ((-.3, .04), (.3, -.04), (0, .06)):
            with self.subTest(angle=angle, distance=distance):
                relative = transform(self.axis, angle, [0, 0, 0])
                relative[:3, 3] = (np.eye(3) - relative[:3, :3]) @ point + distance * self.axis
                pose = matrix_to_pose(relative @ self.zero)
                message = self.message(pose)
                wire = json.loads(json.dumps(message, allow_nan=False))
                self.assertEqual(wire["pose"], pose)
                wheel = wire["steering_wheel"]
                self.assertTrue(wheel["valid"])
                self.assertAlmostEqual(wheel["theta_rad"], angle)
                self.assertAlmostEqual(wheel["translation_along_axis_m"], distance)
                np.testing.assert_allclose(wheel["translation_vector_m"], relative[:3, 3], atol=1e-12)
                self.assertEqual(wheel["axis_frame"], "camera_color_optical_frame")
                if angle:
                    self.assertIn(f"theta={angle:.6f} rad", fmt_steering_wheel(wheel))
                self.assertIn(f"translation_axis={distance:.6f} m", fmt_steering_wheel(wheel))

    def test_loss_source_mismatch_and_invalid_pose_never_reuse_motion(self):
        self.assertTrue(self.message()["steering_wheel"]["valid"])
        for overrides, reason in ((dict(status="SEARCHING", pose=None, tag_id=None), "target_not_tracking"),
                                  (dict(camera_id="other"), "calibration_source_mismatch"),
                                  (dict(tag_id=1), "calibration_source_mismatch"),
                                  (dict(pose={}), "invalid_target_pose")):
            with self.subTest(overrides=overrides):
                message = self.message(**overrides)
                wheel = message["steering_wheel"]
                self.assertFalse(wheel["valid"])
                self.assertEqual(wheel["reason"], reason)
                self.assertIsNone(wheel["theta_rad"])
                self.assertIsNone(wheel["translation_along_axis_m"])
                self.assertIn(f"reason={reason}", fmt_steering_wheel(wheel))
        self.assertTrue(self.message()["steering_wheel"]["valid"])

    def test_missing_uncalibrated_or_corrupt_yaml_preserves_valid_raw_detection(self):
        for contents in (None, "[invalid]", "steering_wheel_calibration:\n  status: uncalibrated\n",
                         "steering_wheel_calibration: [\n"):
            with self.subTest(contents=contents):
                if contents is None:
                    self.path.unlink(missing_ok=True)
                else:
                    self.path.write_text(contents, encoding="utf-8")
                self.observation = SteeringWheelObservation(self.path)
                message = self.message()
                self.assertTrue(message["valid"])
                self.assertIsInstance(message["pose"], dict)
                self.assertFalse(message["steering_wheel"]["valid"])
                self.assertEqual(message["steering_wheel"]["reason"], "calibration_unavailable")
                json.dumps(message, allow_nan=False)

    def test_reload_changes_zero_and_invalid_new_model_does_not_use_old_calibration(self):
        with patch("steering_wheel.time.monotonic", return_value=0):
            first = self.message()["steering_wheel"]
        self.zero[:3, 3] += .1 * self.axis
        self.document["steering_wheel_calibration"]["zero"]["pose"]["T_camera_tag"] = self.zero.tolist()
        self.write_document()
        with patch("steering_wheel.time.monotonic", return_value=2):
            second = self.message()["steering_wheel"]
        self.assertNotEqual(first["calibration_id"], second["calibration_id"])
        self.assertAlmostEqual(second["translation_along_axis_m"], 0)
        self.path.write_text("invalid: [", encoding="utf-8")
        with patch("steering_wheel.time.monotonic", return_value=4):
            self.assertFalse(self.message()["steering_wheel"]["valid"])
        self.write_document()
        with patch("steering_wheel.time.monotonic", return_value=6):
            self.assertTrue(self.message()["steering_wheel"]["valid"])

    def test_config_paths_disabled_mode_and_option_validation(self):
        config = self.path.parent / "camera.yaml"
        options = steering_wheel_options({}, config)
        self.assertEqual(options["calibration_file"], config.parent / "steering_wheel_calibration.yaml")
        self.assertEqual(steering_wheel_options({}, config, str(self.path))["calibration_file"], self.path)
        for settings in ([], {"enabled": "false"}, {"calibration_file": ""}):
            with self.assertRaises(ValueError):
                steering_wheel_options(settings, config)
        self.observation = SteeringWheelObservation(self.path, enabled=False)
        self.assertEqual(self.message()["steering_wheel"]["reason"], "disabled")

    def test_calibration_receives_raw_poses_even_when_wheel_observation_is_unavailable(self):
        self.path.unlink()
        args = calibrate.parse_args(["--samples", "2", "--tag-id", "0"])
        messages = [self.message(frame_id=i, sample_mono_us=1000000 + i * 20000) for i in (1, 2)]
        sub = MagicMock()
        sub.recv_multipart.side_effect = [[b"camera.detection", json.dumps(m).encode()] for m in messages]
        with patch.object(calibrate.time, "monotonic_ns", return_value=1000000000), \
                contextlib.redirect_stdout(io.StringIO()):
            records, matrix = calibrate.receive_poses(sub, MagicMock(), args, "zero")
        self.assertEqual(len(records), 2)
        np.testing.assert_allclose(matrix, self.zero, atol=1e-12)

    def test_validation_cli_recomputes_raw_pose_and_agrees_with_camera_publication(self):
        relative = transform(self.axis, -.25, .04 * self.axis)
        message = self.message(matrix_to_pose(relative @ self.zero))
        expected = dict(message["steering_wheel"])
        # Ensure validation continues to independently use raw pose, rather
        # than trusting the newly published derived fields.
        message["steering_wheel"]["theta_rad"] = 999
        zmq = MagicMock()
        sub = zmq.Context.return_value.socket.return_value
        sub.poll.return_value = 1
        sub.recv_multipart.return_value = [b"camera.detection", json.dumps(message).encode()]
        output = io.StringIO()
        with patch.dict(sys.modules, {"zmq": zmq}), contextlib.redirect_stdout(output):
            self.assertEqual(validate.main(["--config", str(self.path), "--json", "--max-msgs", "1"]), 0)
        result = json.loads(output.getvalue())
        self.assertAlmostEqual(result["theta_rad"], expected["theta_rad"])
        self.assertAlmostEqual(result["translation_along_axis_m"], expected["translation_along_axis_m"])

    def test_shared_loader_rejects_nonfinite_or_nonrigid_zero_and_invalid_axis(self):
        for zero, axis in ((np.zeros((4, 4)), self.axis), (np.eye(4) * float("nan"), self.axis),
                           (np.diag([2., 1., 1., 1.]), self.axis), (self.zero, [0, 0, 0])):
            with self.subTest(zero=zero, axis=axis):
                self.document["steering_wheel_calibration"]["zero"]["pose"]["T_camera_tag"] = zero.tolist()
                self.document["steering_wheel_calibration"]["motion"]["axis_direction"] = np.asarray(axis).tolist()
                self.write_document()
                with self.assertRaises(ValueError):
                    validate.load_calibration(self.path)
                self.observation = SteeringWheelObservation(self.path)
                self.assertFalse(self.message()["steering_wheel"]["valid"])


if __name__ == "__main__":
    unittest.main()
