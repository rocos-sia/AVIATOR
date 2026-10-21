"""Offline sampling, persistence, transport and owned process lifecycle checks."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import yaml
import zmq

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.steering_wheel_calibration.ui import parse_args
from tools.steering_wheel_calibration.ui_backend import CalibrationSession, Worker, commands, mono_us
from tools.steering_wheel_calibration.calibration import load_calibration
from tools.steering_wheel_calibration.geometry import CalibrationError, matrix_to_pose


def detection(stamp, angle=0, depth=0, **overrides):
    matrix = np.eye(4)
    c, s = math.cos(angle), math.sin(angle)
    matrix[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    matrix[2, 3] = depth
    return dict(msg_type="CameraDetection", camera_id="cockpit", tag_id=0,
                session_id="test-session", status="TRACKING", valid=True, confidence=1.0,
                sample_mono_us=stamp, frame_id=stamp, pose=matrix_to_pose(matrix), **overrides)


class SamplingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.args = parse_args(["--output", str(self.directory / "wheel.yaml"), "--samples", "2"])
        self.session = CalibrationSession(self.args, "test-session")

    def tearDown(self):
        self.temp.cleanup()

    def feed(self, stamp, angle=0, depth=0):
        self.session.ingest(detection(stamp, angle, depth), stamp)

    def capture(self, point, start, angle=0, depth=0):
        self.feed(start, angle, depth)
        self.session.begin(point, start + 1)
        self.feed(start + 20000, angle, depth)
        self.feed(start + 40000, angle, depth)

    def test_two_points_backup_and_live_signed_readout(self):
        self.args.output.write_text("unrelated: preserved\n")
        original = self.args.output.read_bytes()
        self.capture(1, 1000000)
        self.assertFalse(self.session.pending)
        self.capture(2, 1100000, math.pi / 6, .08)
        _, zero, axis = load_calibration(self.args.output)
        np.testing.assert_allclose(zero, np.eye(4))
        np.testing.assert_allclose(axis, [0, 0, 1])
        self.assertEqual(yaml.safe_load(self.args.output.read_text())["unrelated"], "preserved")
        self.assertEqual(next(self.directory.glob("*.bak.*")).read_bytes(), original)
        self.feed(1200000, -math.pi / 9, -.03)
        state = self.session.snapshot(1200000)
        self.assertAlmostEqual(state["theta_deg"], -20)
        self.assertAlmostEqual(state["depth_mm"], -30)
        self.assertIsNone(self.session.snapshot(1800000)["theta_deg"])
        self.feed(1900000)
        invalid = detection(1920000)
        invalid["valid"] = False
        self.session.ingest(invalid, 1920000)
        self.assertIsNone(self.session.snapshot(1920000)["depth_mm"])

    def test_click_filters_preclick_duplicate_wrong_source_stale_future(self):
        with self.assertRaises(CalibrationError):
            self.session.begin(1, 1000000)
        self.feed(1000000)
        with self.assertRaises(CalibrationError):
            self.session.begin(2, 1000001)
        self.session.begin(1, 1020000)
        with self.assertRaises(CalibrationError):
            self.session.begin(1, 1020001)
        self.session.ingest(detection(1010000), 1030000)  # queued before click
        for field, value in (("session_id", "other"), ("camera_id", "other"), ("tag_id", 9)):
            data = detection(1030000)
            data[field] = value
            self.session.ingest(data, 1030000)
        self.session.ingest(detection(1040000), 2000000)  # stale
        self.session.ingest(detection(3000000), 2000000)  # future
        self.assertEqual(len(self.session.records), 0)
        self.feed(2010000)
        self.feed(2010000)
        self.feed(2010010)  # too close
        self.assertEqual(len(self.session.records), 1)
        self.feed(2030000)
        self.assertIsNotNone(self.session.zero)

    def test_retake_cancels_second_and_timeout_preserves_file(self):
        self.capture(1, 1000000)
        self.capture(2, 1100000, .5, .1)
        saved = self.args.output.read_bytes()
        self.feed(1200000)
        self.session.begin(1, 1200001)
        self.assertIsNone(self.session.zero)
        self.session.snapshot(20000000)
        self.assertIsNone(self.session.pending)
        self.assertEqual(self.args.output.read_bytes(), saved)
        self.assertIn("超时", self.session.message)

    def test_identical_points_do_not_overwrite(self):
        self.capture(1, 1000000)
        self.args.output.write_text("keep: original\n")
        with self.assertRaises(CalibrationError):
            self.capture(2, 1100000)
        self.assertEqual(self.args.output.read_text(), "keep: original\n")

    def test_commands_and_arbitrary_working_directory(self):
        plans, endpoint = commands(self.args, "unique-session")
        camera = plans[1][1]
        self.assertEqual(camera[camera.index("--session") + 1], "unique-session")
        self.assertEqual(camera[camera.index("--steering-wheel-calibration") + 1], str(self.args.output))
        result = subprocess.run([str(ROOT / "scripts/start_steering_wheel_calibration_ui.sh"), "--dry-run"],
                                cwd=self.directory, env={**os.environ, "UI_PYTHON": sys.executable},
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("启动 bus:", result.stdout)
        self.assertFalse(list(self.directory.iterdir()))


class WorkerTests(unittest.TestCase):
    def test_real_zmq_and_owned_child_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            args = parse_args(["--output", str(directory / "result.yaml"), "--samples", "1"])
            context = zmq.Context()
            pub = context.socket(zmq.PUB)
            port = pub.bind_to_random_port("tcp://127.0.0.1")
            child = directory / "fake_node.py"
            child.write_text("import os, pathlib, sys, time\npathlib.Path(sys.argv[1]).write_text(str(os.getpid()))\nprint('READY input=fake', flush=True)\ntime.sleep(60)\n")
            plans = [(name, [sys.executable, str(child), str(directory / (name + '.pid'))])
                     for name in ("bus", "camera")]
            worker = Worker(args, "test-session")
            try:
                with patch("tools.steering_wheel_calibration.ui_backend.commands", return_value=(plans, f"tcp://127.0.0.1:{port}")), \
                        patch.dict(os.environ, {"XDG_RUNTIME_DIR": str(directory)}):
                    worker.start()
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        pub.send_multipart([b"camera.detection", json.dumps(detection(mono_us())).encode()])
                        if worker.snapshot().get("fresh"):
                            break
                        time.sleep(.03)
                    self.assertTrue(worker.snapshot().get("fresh"), worker.snapshot())
                    worker.actions.put(1)
                    while time.monotonic() < deadline and not worker.snapshot().get("has_zero"):
                        pub.send_multipart([b"camera.detection", json.dumps(detection(mono_us())).encode()])
                        time.sleep(.03)
                    self.assertTrue(worker.snapshot().get("has_zero"))
                    worker.stop_requested.set()
                    worker.join(10)
                    self.assertFalse(worker.is_alive())
                    self.assertFalse(worker.snapshot().get("error"), worker.snapshot())
                    for name in ("bus", "camera"):
                        pid = int((directory / (name + '.pid')).read_text())
                        self.assertFalse(Path(f"/proc/{pid}").exists())
            finally:
                worker.stop_requested.set()
                worker.join(15)
                pub.close(0)
                context.term()


if __name__ == "__main__":
    unittest.main()
