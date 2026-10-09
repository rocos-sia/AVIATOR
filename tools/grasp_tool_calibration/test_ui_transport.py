"""Offline process ownership and real inproc ZMQ tests; no robot or TCP listeners."""
import copy
import io
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch
import uuid

import ui_transport as app


class FakeProcess:
    def __init__(self, text="", pid=41001):
        self.pid, self.returncode = pid, None
        self.stdout = io.StringIO(text)

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("fake", timeout)
        return self.returncode


class ManagedNodesTests(unittest.TestCase):
    def test_owned_groups_readiness_logs_and_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = app.ManagedNodes(Path(directory), Path(directory) / "logs", lambda _: None)
            process = FakeProcess("before\nREADY input=tcp://test\nafter\n")

            def terminate(pid, sig):
                self.assertEqual(pid, process.pid)
                self.assertEqual(sig, signal.SIGTERM)
                process.returncode = -sig

            with patch.object(app.subprocess, "Popen", return_value=process) as popen, \
                    patch.object(app.os, "killpg", side_effect=terminate) as kill:
                manager.start("bus", ["/build/aviator_bus", "--input", "tcp://test"])
                args = popen.call_args
                self.assertTrue(args.kwargs["start_new_session"])
                self.assertIs(args.kwargs["stdin"], subprocess.DEVNULL)
                self.assertNotIn("shell", args.kwargs)
                manager._nodes["bus"]["reader"].join(timeout=1)
                status = manager.snapshot()[0]
                self.assertTrue(status["owned"] and status["running"])
                self.assertEqual(status["tail"], ["before", "READY input=tcp://test", "after"])
                self.assertIn("after", Path(status["log_path"]).read_text())
                with self.assertRaisesRegex(RuntimeError, "already running"):
                    manager.start("bus", ["other"])
                manager.stop("foreign")
                kill.assert_not_called()
                manager.stop_all()
                kill.assert_called_once()
                self.assertFalse(manager.snapshot()[0]["running"])

    def test_bus_timeout_cleans_only_owned_process(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = app.ManagedNodes(Path(directory), Path(directory), lambda _: None)
            process = FakeProcess("bind failed: address already in use\n")
            with patch.object(app.subprocess, "Popen", return_value=process), \
                    patch.object(app.os, "killpg", side_effect=lambda *_: setattr(process, "returncode", -15)) as kill:
                with self.assertRaisesRegex(RuntimeError, "address already in use"):
                    manager.start("bus", ["aviator_bus"], timeout=.04)
                kill.assert_called_once_with(process.pid, signal.SIGTERM)

    def test_start_failure_and_bounded_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = app.ManagedNodes(Path(directory), Path(directory), lambda _: None)
            with patch.object(app.subprocess, "Popen", side_effect=FileNotFoundError("missing")):
                with self.assertRaises(FileNotFoundError):
                    manager.start("missing", ["missing"])
            self.assertEqual(manager.snapshot(), [])
            process = FakeProcess("line\n" * 250 + "READY\n")
            with patch.object(app.subprocess, "Popen", return_value=process), \
                    patch.object(app.os, "killpg", side_effect=lambda *_: setattr(process, "returncode", -15)):
                manager.start("helper", ["helper"], ready_text="READY")
                self.assertEqual(len(manager.snapshot()[0]["tail"]), 120)
                manager.stop_all()

    def test_conflict_matches_executable_and_script_not_unrelated_arguments(self):
        cases = [
            (["/a/aviator_core_managed"], True),
            (["/a/manipulator", "--config", "x"], True),
            (["/a/change_stiffness_simple"], True),
            (["/a/aviator_grasp_tool_session"], True),
            (["/a/python3", "/a/rh56ftp_hand/rh56ftp_node.py"], True),
            (["python", "-u", "nodes/rh56ftp_hand/fake_rh56ftp_hand.py"], True),
            (["python", "-W", "ignore", "tools/grasp_tool_calibration/ui.py"], True),
            (["python", "tools/grasp_tool_calibration/ui.py", "--dry-run"], False),
            (["python", "nodes/rh56ftp_hand/rh56ftp_node.py", "--feedback-only"], False),
            (["python", "test.py", "tools/grasp_tool_calibration/ui.py"], False),
            (["python", "-c", "print('rh56ftp_node.py')"], False),
            (["python", "-m", "unittest", "rh56ftp_node.py"], False),
            (["rg", "aviator_core", "."], False),
            (["bash", "-c", "./aviator_core_managed"], False),
            (["python", "/unrelated/ui.py"], False),
        ]
        for argv, wanted in cases:
            with self.subTest(argv=argv):
                self.assertEqual(app._conflicting_command(argv), wanted)


class HandCameraTests(unittest.TestCase):
    def setUp(self):
        try:
            import zmq
        except ImportError:
            self.skipTest("pyzmq unavailable")
        self.zmq = zmq
        self.context = zmq.Context()
        token = uuid.uuid4().hex
        self.commands = self.context.socket(zmq.SUB)
        self.commands.subscribe(b"hand.command")
        self.commands.bind("inproc://commands-" + token)
        self.states = self.context.socket(zmq.PUB)
        self.states.bind("inproc://states-" + token)
        self.opened = {side: [.5, 1, 1, 1, 1, 1] for side in ("left", "right")}
        self.closed = {side: [.5, 0, 0, 0, 0, 0] for side in ("left", "right")}
        self.link = app.HandCameraLink("inproc://commands-" + token, "inproc://states-" + token,
                                      self.opened, self.closed, lambda _: None, context=self.context)
        self.sequence = 0
        self.received = []
        self.link.start()

    def tearDown(self):
        if hasattr(self, "link"):
            self.link.stop()
            self.commands.close(linger=0)
            self.states.close(linger=0)
            self.context.term()

    def hand_state(self, command):
        self.sequence += 1
        stamp = app._mono_us()
        return dict(msg_type="HandState", version="1.0", publisher_id="rh56ftp_hand",
                    session_id="test-hand-node", sequence=self.sequence, sample_mono_us=stamp,
                    timestamp=time.time_ns() // 1000, clock_id=self.link.clock_id,
                    valid=True, command_valid=True, feedback_only=False, hold_control_error="",
                    accepted_command={k: command[k] for k in (
                        "publisher_id", "session_id", "sequence", "sample_mono_us", "control_epoch")},
                    hands={side: dict(valid=True, status="ACTIVE", feedback_available=True,
                                      sample_mono_us=stamp, error_code=0, error_codes=[0] * 6,
                                      drive_position_normalized=list(self.opened[side]), grasp_verified=False)
                           for side in ("left", "right")})

    def pump(self, predicate=lambda status: status["ready"], transform=None, timeout=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.commands.poll(5):
                topic, data = self.commands.recv_multipart()
                self.assertEqual(topic, b"hand.command")
                command = json.loads(data)
                self.received.append(command)
                state = self.hand_state(command)
                if transform:
                    transform(state)
                self.states.send_multipart([b"hand.state", json.dumps(state).encode()])
            status = self.link.snapshot()
            if predicate(status):
                return status
        self.fail("Timed out waiting for expected link status: " + str(self.link.snapshot()))

    def test_continuous_targets_current_target_ack_and_stop_release(self):
        self.assertFalse(self.link.snapshot()["ready"])
        status = self.pump()
        self.assertTrue(status["fresh"] and status["ack"]["valid"])
        self.assertFalse(status["hands"]["left"]["grasp_verified"])
        self.link.set_hand("left", "close")
        self.assertFalse(self.link.snapshot()["ready"], "old ACK must not accept new targets")
        self.pump()
        current = self.received[-1]
        self.assertEqual(current["hands"]["left"]["drive_position_normalized"], self.closed["left"])
        self.assertEqual(current["hands"]["right"]["drive_position_normalized"], self.opened["right"])
        self.assertEqual(current["origin"]["sequence"], current["sequence"])
        count = len(self.received)
        self.pump(predicate=lambda _: len(self.received) >= count + 4)
        recent = self.received[-4:]
        self.assertEqual(len({m["sequence"] for m in recent}), 4)
        self.assertTrue(all(m["hands"] == current["hands"] for m in recent))
        self.link.wait_ready(timeout=.1)
        self.link.stop()
        released = []
        while self.commands.poll(20):
            released.append(json.loads(self.commands.recv_multipart()[1]))
        self.assertEqual([m["valid"] for m in released[-6:]], [True] * 3 + [False] * 3)
        self.assertEqual(released[-6]["hands"]["left"]["drive_position_normalized"], self.opened["left"])
        self.assertFalse(self.link.snapshot()["running"])

    def test_stale_ack_and_measurements_do_not_remain_ready(self):
        status = self.pump()
        with patch.object(app, "_mono_us", return_value=status["state"]["sample_mono_us"] + 600_000):
            status = self.link.snapshot()
            self.assertFalse(status["ready"] or status["fresh"] or status["ack"]["valid"])

    def test_foreign_ack_latches_conflict_and_stops_publication(self):
        status = self.pump(predicate=lambda s: bool(s["conflict"]),
                           transform=lambda s: s["accepted_command"].update(session_id="foreign-session"))
        self.assertFalse(status["ready"])
        self.link.stop()
        while self.commands.poll(0):
            self.commands.recv_multipart()
        self.assertFalse(self.commands.poll(60), "must not invalidate somebody else's session")
        with self.assertRaises(RuntimeError):
            self.link.set_hand("both", "close")

    def test_invalid_ack_seq_and_wrong_clock_do_not_pass(self):
        status = self.pump(predicate=lambda s: s["state"] is not None,
                           transform=lambda s: s["accepted_command"].update(sequence=999999))
        self.assertFalse(status["ack"]["valid"] or status["ready"])
        self.pump()
        before = self.link.snapshot()["state"]["sequence"]
        state = self.hand_state(self.received[-1])
        state["clock_id"] = "wrong-host"
        self.link._accept_hand(state, app._mono_us())
        self.assertEqual(self.link.snapshot()["state"]["sequence"], before)

    def test_fault_and_missing_side_never_claim_success(self):
        status = self.pump(predicate=lambda s: bool(s["fault"]),
                           transform=lambda s: s["hands"]["left"].update(error_code=2))
        self.assertFalse(status["ready"])
        self.assertTrue(status["ack"]["valid"], "ACK alone is not healthy feedback")
        with self.assertRaises(RuntimeError):
            self.link.wait_ready(timeout=.1)
        status = self.pump(predicate=lambda s: "right" not in s["hands"],
                           transform=lambda s: s["hands"].pop("right"))
        self.assertFalse(status["fresh"] or status["ready"])

    def test_camera_filter_and_deep_snapshot(self):
        self.pump()
        other = dict(msg_type="CameraDetection", camera_id="other", sequence=1)
        wanted = dict(msg_type="CameraDetection", camera_id="cockpit", sequence=2,
                      steering_wheel=dict(theta_rad=.2, translation_along_axis_m=.01))
        self.states.send_multipart([b"camera.detection", json.dumps(other).encode()])
        self.states.send_multipart([b"camera.detection", json.dumps(wanted).encode()])
        status = self.pump(predicate=lambda s: s["camera"]["raw"] is not None)
        self.assertEqual(status["camera"]["raw"], wanted)
        self.assertIsInstance(status["camera"]["received_mono_us"], int)
        status["camera"]["raw"]["sequence"] = 100
        self.assertEqual(self.link.snapshot()["camera"]["raw"]["sequence"], 2)

    def test_preset_validation_and_malformed_feedback(self):
        for target in ({"left": [0] * 5, "right": [0] * 6},
                       {"left": [float("nan")] * 6, "right": [0] * 6},
                       {"left": [True] * 6, "right": [0] * 6}):
            with self.assertRaises(ValueError):
                app.HandCameraLink("a", "b", target, self.closed, lambda _: None)
        self.pump()
        state = self.hand_state(self.received[-1])
        state["hands"]["left"]["error_codes"] = 5
        state["hands"]["right"] = []
        self.link._accept_hand(state, app._mono_us())
        self.assertFalse(self.link.snapshot()["ready"])
        with self.assertRaises(ValueError):
            self.link.set_hand("left", "half")


if __name__ == "__main__":
    unittest.main()
