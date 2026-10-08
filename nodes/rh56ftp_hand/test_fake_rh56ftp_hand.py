"""Deterministic device tests and real-process wire/watchdog checks."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

import rh56ftp_node as real
from fake_rh56ftp_hand import FakeHandLink, main


class FakeDeviceTest(unittest.TestCase):
    def test_motion_partial_stop_and_speed(self):
        now = [0.0]
        link = FakeHandLink(clock=lambda: now[0])
        link.connect()
        link.write_angle_set([0, 0, 0, 0, 0, 500])
        now[0] = .1
        self.assertEqual(link.read_state()["angle"], [800] * 5 + [500])
        link.write_angle_set([-1, None, None, None, None, None])
        link.write_speed_set([None, 250, None, None, None, None])
        now[0] = .2
        self.assertEqual(link.read_state()["angle"], [800, 700, 600, 600, 600, 500])
        link.write_angle_set([1000, None, None, None, None, None])
        now[0] = 1
        self.assertEqual(link.read_state()["angle"], [1000, 0, 0, 0, 0, 500])
        link.write_speed_set([0] * 6)
        link.write_angle_set([500] * 6)
        now[0] = 2
        self.assertEqual(link.read_state()["angle"], [1000, 0, 0, 0, 0, 500])

    def test_config_no_hardware_dependency_and_both_hands_default(self):
        with patch.object(real, "load_handlink", side_effect=AssertionError("hardware imported")), \
                patch.object(real, "run_node", return_value=0) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            main([])
            node = run.call_args.args[0]
            self.assertTrue(all(node.links.values()))
            self.assertIs(node.read_links, node.links)
            self.assertEqual(node.publisher_id, "rh56ftp_hand")
            main(["--config", str(Path(__file__).parents[2] / "config/rh56ftp_hand.yaml"),
                  "--speed", "300", "--left-host", ""])
            node = run.call_args.args[0]
            self.assertEqual(node.speed, 300)
            self.assertIsNone(node.links["left"])
            node.connect()
            node.read_states()
            state = node.make_state()
            self.assertTrue(state["valid"] and state["hands"]["right"]["valid"])
            self.assertFalse(state["hands"]["left"]["valid"])
            node.close()

    def test_invalid_motion_rate(self):
        for value in ("0", "-1", "nan", "inf"):
            with patch("sys.stderr"), self.assertRaises(SystemExit) as error:
                main(["--motion-rate", value])
            self.assertEqual(error.exception.code, 2)


class FakeProcessTest(unittest.TestCase):
    def test_commands_feedback_watchdog_and_readonly(self):
        import zmq
        for readonly in (False, True):
            with self.subTest(readonly=readonly), tempfile.TemporaryFile(mode="w+") as log:
                context = zmq.Context()
                commands, states = context.socket(zmq.PUB), context.socket(zmq.SUB)
                command_port = commands.bind_to_random_port("tcp://127.0.0.1")
                state_port = states.bind_to_random_port("tcp://127.0.0.1")
                states.subscribe(b"hand.state")
                child = subprocess.Popen([
                    sys.executable, str(Path(__file__).with_name("fake_rh56ftp_hand.py")),
                    "--endpoint", f"tcp://127.0.0.1:{command_port}",
                    "--state-endpoint", f"tcp://127.0.0.1:{state_port}",
                    "--state-hz", "50", *(["--feedback-only"] if readonly else []),
                ], stdout=log, stderr=subprocess.STDOUT)
                sequence = state_sequence = 0
                epoch, session = str(uuid.uuid4()), str(uuid.uuid4())
                last_command = None

                def send(mode, targets, valid=True, stale=False):
                    nonlocal sequence, last_command
                    sequence += 1
                    stamp = real.monotonic_us() - (1000000 if stale else 0)
                    last_command = dict(
                        msg_type="HandCommand", version="1.0", sequence=sequence,
                        timestamp=time.time_ns() // 1000, sample_mono_us=stamp,
                        clock_id=real._clock_id(), publisher_id="aviator_core", session_id=session,
                        control_epoch=epoch, valid=valid, mode=mode,
                        origin=dict(publisher_id="aviator_core", session_id=session, sequence=sequence,
                                    sample_mono_us=stamp, clock_id=real._clock_id()), hands=targets)
                    commands.send_multipart([b"hand.command", json.dumps(last_command).encode()])

                def wait_for(predicate, publish=None, timeout=3):
                    nonlocal state_sequence
                    deadline, next_send = time.monotonic() + timeout, 0
                    while time.monotonic() < deadline:
                        self.assertIsNone(child.poll(), "fake node exited unexpectedly")
                        if publish and time.monotonic() >= next_send:
                            publish()
                            next_send = time.monotonic() + .02
                        if not states.poll(5):
                            continue
                        topic, data = states.recv_multipart()
                        state = json.loads(data)
                        self.assertEqual(topic, b"hand.state")
                        self.assertGreater(state["sequence"], state_sequence)
                        state_sequence = state["sequence"]
                        self.assertEqual(state["publisher_id"], "rh56ftp_hand")
                        self.assertEqual(state["clock_id"], real._clock_id())
                        self.assertTrue(state["simulated"])
                        if predicate(state):
                            return state
                    self.fail("timed out waiting for simulated hand state")

                def at(state, poses):
                    return state["valid"] and all(
                        state["hands"][s]["drive_position_normalized"] == poses[s]
                        for s in real.SIDES)

                try:
                    opened = {s: list(real.DEFAULT_SAFE_POSE) for s in real.SIDES}
                    startup = wait_for(lambda s: at(s, opened))
                    self.assertIsNone(startup["accepted_command"])
                    self.assertFalse(startup["command_valid"])
                    poses = dict(left=[.2, .3, .4, .5, .6, .7], right=[.8, .7, .6, .5, .4, .3])
                    targets = {s: dict(drive_position_normalized=poses[s]) for s in real.SIDES}
                    if readonly:
                        end = time.monotonic() + .3
                        state = wait_for(lambda s: time.monotonic() >= end,
                                         lambda: send("NORMALIZED_POSITION", targets))
                        self.assertTrue(at(state, opened))
                        self.assertTrue(state["feedback_only"])
                        self.assertFalse(state["command_valid"])
                        self.assertIsNone(state["accepted_command"])
                        continue
                    state = wait_for(lambda s: at(s, poses) and s["command_valid"],
                                     lambda: send("NORMALIZED_POSITION", targets))
                    ack = state["accepted_command"]
                    self.assertEqual(ack["publisher_id"], "aviator_core")
                    self.assertEqual(ack["control_epoch"], epoch)
                    self.assertEqual(ack["session_id"], session)
                    self.assertLessEqual(ack["sequence"], sequence)
                    self.assertLess(real.monotonic_us() - ack["sample_mono_us"], 100000)
                    for s in real.SIDES:
                        self.assertEqual(state["hands"][s]["drive_position_raw"],
                                         real.normalized_to_raw(poses[s]))
                        self.assertFalse(state["hands"][s]["grasp_verified"])
                    # An old command must not keep a moving target alive.
                    wait_for(lambda s: at(s, opened) and not s["command_valid"],
                             lambda: send("NORMALIZED_POSITION", targets, stale=True))
                    grasp = {s: dict(grasp=dict(closure=1)) for s in real.SIDES}
                    closed = {s: [0] * 6 for s in real.SIDES}
                    wait_for(lambda s: at(s, closed), lambda: send("GRASP_SETPOINT", grasp))
                    wait_for(lambda s: at(s, opened) and not s["command_valid"],
                             lambda: send("NORMALIZED_POSITION", targets, valid=False))
                finally:
                    child.terminate()
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                    commands.close(0)
                    states.close(0)
                    context.term()
                    if child.returncode != 0:
                        log.seek(0)
                        self.fail(log.read())


if __name__ == "__main__":
    unittest.main()
