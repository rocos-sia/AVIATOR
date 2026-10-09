"""Calibration acquisition tests using synthetic frames and a local fake helper.

No robot addresses, camera devices, or native SDK connections are used.
"""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

import calibrate as app
from calibration import CalibrationError


def options(*extra):
    return app.parse_args(["--samples", "6", "--interval-ms", "100", "--timeout-s", "5",
                           "--max-age-ms", "1000", "--max-sync-ms", "300", *extra])


def frame(stamp=1_100_000, sequence=1, **changes):
    data = dict(msg_type="CameraDetection", camera_id="cockpit", publisher_id="camera",
                clock_id="test-clock", session_id="test-session", sequence=sequence,
                sample_mono_us=stamp, valid=True, status="TRACKING", confidence=.95,
                steering_wheel=dict(valid=True, axis_match=True, calibration_id="wheel-calibration",
                                    theta_rad=.2, translation_along_axis_m=.01))
    data.update(changes)
    return data


def state(mid=1_100_000, begin=None, end=None, source="robot"):
    arm = {"joint_position": [0.] * 7,
           "flange": {"position": [0., 0., 0.], "quaternion": [1., 0., 0., 0.]}}
    return dict(frame="aircraft", source=source,
                sample_mono_us=mid, sample_start_mono_us=mid - 100 if begin is None else begin,
                sample_end_mono_us=mid + 100 if end is None else end,
                left=copy.deepcopy(arm), right=copy.deepcopy(arm))


class CameraTrackerTests(unittest.TestCase):
    def tracker(self):
        return app.CameraTracker(options(), 1_000_000, "test-clock")

    def test_mapping_and_metadata(self):
        tracker = self.tracker()
        result = tracker.accept(frame(), 1_120_000)
        self.assertAlmostEqual(result["angle_rad"], -.2)
        self.assertAlmostEqual(result["displacement_m"], -.095)
        self.assertEqual(result["sample_mono_us"], 1_100_000)
        self.assertEqual(result["sequence"], 1)
        self.assertEqual(tracker.session, "test-session")
        self.assertEqual(tracker.calibration_id, "wheel-calibration")
        self.assertEqual(tracker.records, [result])

    def test_rejects_invalid_envelopes(self):
        cases = [None, [], frame(msg_type="ArmState"), frame(camera_id="other-camera"),
                 frame(publisher_id="other-source"), frame(clock_id="different-host"),
                 frame(session_id=""), frame(session_id=3), frame(sequence=True),
                 frame(sequence=-1), frame(sequence=1.5), frame(sample_mono_us=True),
                 frame(sample_mono_us=1_100_000.0)]
        for data in cases:
            with self.subTest(data=data):
                tracker = self.tracker()
                self.assertIsNone(tracker.accept(data, 1_120_000))
                self.assertFalse(tracker.records)
                self.assertEqual(sum(tracker.rejected.values()), 1)

    def test_rejects_pre_start_expired_and_future_frames(self):
        for stamp, now in [(999_999, 1_100_000), (1_100_000, 2_100_001),
                           (1_100_001, 1_100_000)]:
            with self.subTest(stamp=stamp, now=now):
                tracker = self.tracker()
                self.assertIsNone(tracker.accept(frame(stamp=stamp), now))
                self.assertEqual(tracker.last_sequence, -1)
                self.assertFalse(tracker.records)

    def test_rejects_invalid_tracking_and_confidence(self):
        changes = [{"valid": False}, {"valid": 1}, {"status": "LOST"},
                   *({"confidence": v} for v in [None, True, "0.9", .49, 1.01,
                                                  float("nan"), float("inf"), 10**400]),
                   {"steering_wheel": None},
                   {"steering_wheel": dict(frame()["steering_wheel"], valid=False)},
                   {"steering_wheel": dict(frame()["steering_wheel"], axis_match=False)},
                   {"steering_wheel": dict(frame()["steering_wheel"], calibration_id="")}]
        for change in changes:
            with self.subTest(change=change):
                tracker = self.tracker()
                self.assertIsNone(tracker.accept(frame(**change), 1_120_000))
                self.assertFalse(tracker.records)
                self.assertEqual(tracker.last_sequence, 1)

    def test_rejects_duplicate_sequence_or_nonincreasing_time(self):
        for seq, stamp in [(10, 1_101_000), (9, 1_101_000), (11, 1_100_000), (11, 1_099_000)]:
            with self.subTest(sequence=seq, stamp=stamp):
                tracker = self.tracker()
                self.assertIsNotNone(tracker.accept(frame(sequence=10), 1_120_000))
                self.assertIsNone(tracker.accept(frame(sequence=seq, stamp=stamp), 1_120_000))
                self.assertEqual(len(tracker.records), 1)
                self.assertEqual(tracker.last_sequence, 10)

    def test_new_invalid_frame_advances_watermark(self):
        tracker = self.tracker()
        self.assertIsNotNone(tracker.accept(frame(sequence=10), 1_120_000))
        self.assertIsNone(tracker.accept(frame(sequence=12, stamp=1_102_000, valid=False), 1_120_000))
        self.assertEqual((tracker.last_sequence, tracker.last_sample), (12, 1_102_000))
        self.assertIsNone(tracker.accept(frame(sequence=11, stamp=1_101_000), 1_120_000))
        self.assertIsNotNone(tracker.accept(frame(sequence=13, stamp=1_103_000), 1_120_000))
        self.assertEqual([record["sequence"] for record in tracker.records], [10, 13])

    def test_session_and_calibration_changes_abort(self):
        for update in [{"session_id": "restarted-session"},
                       {"steering_wheel": dict(frame()["steering_wheel"], calibration_id="new") }]:
            with self.subTest(update=update):
                tracker = self.tracker()
                tracker.accept(frame(), 1_120_000)
                with self.assertRaises(CalibrationError):
                    tracker.accept(frame(stamp=1_101_000, sequence=2, **update), 1_120_000)

    def test_invalid_geometry_cannot_be_silently_clamped(self):
        for theta, travel in [(1., 0), (0, .1), (True, 0), (0, None), (float("nan"), 0)]:
            tracker = self.tracker()
            data = frame(steering_wheel=dict(frame()["steering_wheel"], theta_rad=theta,
                                            translation_along_axis_m=travel))
            with self.subTest(theta=theta, travel=travel), self.assertRaises(CalibrationError):
                tracker.accept(data, 1_120_000)
            self.assertFalse(tracker.records)


class StateAndPairingTests(unittest.TestCase):
    def test_state_accepts_matching_source_and_monotonic_span(self):
        for offline in (False, True):
            sample = state(source="offline" if offline else "robot")
            self.assertIs(app.validate_state(sample, 1_120_000, 1_000_000, 1_090_000,
                                             offline, 250), sample)

    def test_state_rejects_sources_frames_and_bad_timestamps(self):
        base = state()
        variants = [None, [], dict(base, frame="world"), dict(base, source="reference"),
                    dict(base, source="offline"), dict(base, sample_start_mono_us=True),
                    dict(base, sample_mono_us=1_100_000.0),
                    dict(base, sample_start_mono_us=999_999),
                    dict(base, sample_end_mono_us=1_200_001),
                    dict(base, sample_start_mono_us=1_100_001),
                    dict(base, sample_end_mono_us=1_099_999)]
        for sample in variants:
            with self.subTest(sample=sample), self.assertRaises(CalibrationError):
                app.validate_state(sample, 1_120_000, 1_000_000, 1_090_000, False, 250)
        for now, previous in [(1_400_000, 1_090_000), (1_120_000, 1_100_000),
                              (1_120_000, 1_100_001)]:
            with self.subTest(now=now, previous=previous), self.assertRaises(CalibrationError):
                app.validate_state(base, now, 1_000_000, previous, False, 250)

    def test_pairing_uses_nearest_frame_and_preserves_raw_observation(self):
        states = [state(mid=1_100_000 + i * 100_000) for i in range(3)]
        camera = [dict(sample_mono_us=1_110_000 + i * 100_000, sequence=i,
                       angle_rad=i / 10, displacement_m=-.085) for i in range(3)]
        paired = app.pair_samples(states, camera, 11)
        self.assertEqual([s["camera"]["sequence"] for s in paired], [0, 1, 2])
        self.assertEqual([s["angle_rad"] for s in paired], [0., .1, .2])
        self.assertTrue(all(s["sync_gap_ms"] == 10.1 for s in paired))
        self.assertNotIn("camera", states[0])

    def test_pairing_checks_both_ends_of_dual_arm_read_span(self):
        camera = [dict(sample_mono_us=1_100_000 + i * 100_000, sequence=i,
                       angle_rad=0., displacement_m=-.085) for i in range(3)]
        states = [state(mid=c["sample_mono_us"]) for c in camera]
        states[0]["sample_start_mono_us"] = 1_000_000
        with self.assertRaisesRegex(CalibrationError, "未对齐"):
            app.pair_samples(states, camera, 50)

    def test_pairing_requires_three_independent_camera_frames(self):
        camera = [dict(sample_mono_us=1_100_000, sequence=1,
                       angle_rad=0., displacement_m=-.085)]
        states = [state(mid=1_100_000 + i * 1000) for i in range(6)]
        with self.assertRaisesRegex(CalibrationError, "独立相机帧不足"):
            app.pair_samples(states, camera, 100)


HELPER_SOURCE = '''#!{python}
import argparse, json, os, sys, time
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument('--robot-config')
p.add_argument('--joints-file')
p.add_argument('--read-robot', action='store_true')
p.add_argument('--samples', type=int)
p.add_argument('--interval-ms', type=int)
a = p.parse_args()
Path(__file__).with_suffix('.args.json').write_text(json.dumps(vars(a)))
Path(__file__).with_suffix('.pid').write_text(str(os.getpid()))
mode = {mode!r}
if mode == 'fail':
    print('synthetic SDK reader diagnostic', file=sys.stderr, flush=True)
    sys.exit(7)
if mode == 'timeout':
    time.sleep(60)
if mode == 'bad_json':
    print('this is not JSON', flush=True)
    time.sleep(60)
for i in range(a.samples):
    stamp = time.monotonic_ns() // 1000
    arm = {{'joint_position': [0.] * 7,
           'flange': {{'position': [0., 0., 0.], 'quaternion': [1., 0., 0., 0.]}}}}
    value = {{'sample_mono_us': stamp, 'sample_start_mono_us': stamp,
             'sample_end_mono_us': stamp, 'frame': 'aircraft',
             'source': 'offline' if a.joints_file else 'robot', 'left': arm, 'right': arm}}
    print(json.dumps(value), flush=True)
    if i + 1 < a.samples:
        time.sleep(a.interval_ms / 1000)
'''


class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="grasp-acquisition-test.")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def helper(self, mode="normal"):
        path = self.root / "state_helper.py"
        path.write_text(HELPER_SOURCE.format(python=sys.executable, mode=mode))
        path.chmod(0o700)
        return path

    def manual_args(self, *extra):
        return options("--wheel-source", "manual", "--angle-deg", "10", "--displacement-m", "-.07",
                       "--joints-file", str(self.root / "offline.json"), *extra)

    def assert_reaped(self, helper):
        pid = int(helper.with_suffix(".pid").read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_collect_manual_offline_helper(self):
        helper = self.helper()
        args = self.manual_args()
        samples, metadata = app.collect(args, helper)
        self.assertEqual(len(samples), 6)
        self.assertEqual(metadata, {})
        self.assertTrue(all(s["source"] == "offline" for s in samples))
        self.assertTrue(all(s["displacement_m"] == -.07 for s in samples))
        self.assertTrue(all(s["angle_rad"] == args.manual_angle for s in samples))
        invocation = json.loads(helper.with_suffix(".args.json").read_text())
        self.assertFalse(invocation["read_robot"])
        self.assertEqual(invocation["joints_file"], str((self.root / "offline.json").resolve()))
        self.assert_reaped(helper)

    def test_helper_failure_preserves_stderr(self):
        helper = self.helper("fail")
        with self.assertRaisesRegex(CalibrationError, "退出码 7.*") as caught:
            app.collect(self.manual_args(), helper)
        self.assertIn("synthetic SDK reader diagnostic", str(caught.exception))
        self.assert_reaped(helper)

    def test_bad_json_terminates_and_reaps_helper(self):
        helper = self.helper("bad_json")
        with self.assertRaisesRegex(CalibrationError, "有效 JSONL"):
            app.collect(self.manual_args(), helper)
        self.assert_reaped(helper)

    def test_timeout_terminates_and_reaps_helper(self):
        helper = self.helper("timeout")
        args = self.manual_args("--samples", "3", "--interval-ms", "1", "--timeout-s", ".5")
        with self.assertRaisesRegex(CalibrationError, "超时"):
            app.collect(args, helper)
        self.assert_reaped(helper)

    def test_collect_camera_from_local_publisher(self):
        try:
            import zmq
        except ImportError:
            self.skipTest("pyzmq is needed for the local publisher integration test")
        helper = self.helper()
        # Real PUB/SUB framing over inproc works in sandboxes that disallow even
        # local IPC/TCP binds. Both sockets must share this test-owned context.
        endpoint = "inproc://grasp-camera-acquisition"
        context = zmq.Context()
        ready, stop = threading.Event(), threading.Event()
        failures = []

        class SharedContext:
            def socket(self, *args, **kwargs):
                return context.socket(*args, **kwargs)

            def term(self):
                # collect owns the SUB, but the test also owns a live PUB.
                # Terminate the real context after that publisher has closed.
                pass

        def publish():
            publisher = context.socket(zmq.PUB)
            publisher.setsockopt(zmq.LINGER, 0)
            try:
                publisher.bind(endpoint)
                ready.set()
                sequence = 0
                while not stop.is_set():
                    sequence += 1
                    data = frame(stamp=app.mono_us(), sequence=sequence, clock_id=app.local_clock_id())
                    publisher.send_multipart([app.TOPIC, json.dumps(data).encode()])
                    stop.wait(.005)
            except BaseException as exc:
                failures.append(exc)
                ready.set()
            finally:
                publisher.close()

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        try:
            self.assertTrue(ready.wait(3), "local publisher did not start")
            if failures:
                raise failures[0]
            args = options("--endpoint", endpoint)
            # No --joints-file exercises the robot-source validation, but the
            # state helper is a synthetic Python process with no SDK imports.
            with mock.patch.object(zmq, "Context", return_value=SharedContext()):
                samples, metadata = app.collect(args, helper)
            self.assertEqual(len(samples), args.samples)
            self.assertEqual(metadata["camera_session"], "test-session")
            self.assertEqual(metadata["calibration_id"], "wheel-calibration")
            self.assertGreaterEqual(len({s["camera"]["sequence"] for s in samples}), 3)
            for sample in samples:
                self.assertAlmostEqual(sample["angle_rad"], -.2)
                self.assertAlmostEqual(sample["displacement_m"], -.095)
                self.assertLessEqual(sample["sync_gap_ms"], args.max_sync_ms)
            self.assert_reaped(helper)
        finally:
            stop.set()
            thread.join(timeout=3)
            context.term()
        self.assertFalse(thread.is_alive(), "local publisher failed to close")
        self.assertFalse(failures)


if __name__ == "__main__":
    unittest.main()
