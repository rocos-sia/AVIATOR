"""Publisher tests use private loopback ports and mock CAN; never the live bus."""

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import hand_command as command

SCRIPT = str(Path(__file__).with_name("hand_command.py"))
HAS_ZMQ = importlib.util.find_spec("zmq") is not None


class PublisherTests(unittest.TestCase):
    def test_inputs_and_single_hand_updates(self):
        for value in ("nan", "inf", "-0.1", "1.1", "0 0", "open close", ""):
            with self.assertRaises(ValueError):
                command.positions(value)
        with self.assertRaises(ValueError):
            command.update_targets("left open", None)
        original = command.update_targets("both open", None)
        changed = command.update_targets("right 1,1,0.8,1,1,1", original)
        self.assertEqual(changed["left"], [1] * 6)
        self.assertEqual(changed["right"], [1, 1, .8, 1, 1, 1])
        self.assertEqual(original["right"], [1] * 6)

    def test_dry_run_and_invalid_cli(self):
        result = subprocess.run([sys.executable, SCRIPT, "--pose", "open", "--dry-run"],
                                capture_output=True, text=True, check=True)
        msg = json.loads(result.stdout)
        self.assertEqual(msg["hands"]["left"]["drive_position_normalized"], [1] * 6)
        for args in (["--left", "1"], ["--pose", "open", "--duration", "nan"],
                     ["--left", "nan", "--right", "1"]):
            self.assertNotEqual(subprocess.run([sys.executable, SCRIPT, *args],
                                              capture_output=True).returncode, 0)

    @unittest.skipUnless(HAS_ZMQ, "pyzmq required for private loopback integration")
    def test_timed_command_and_missing_or_rejected_ack(self):
        import zmq
        for reply in ("accepted", "silent", "readonly", "foreign"):
            with self.subTest(reply=reply):
                context = zmq.Context()
                receive = context.socket(zmq.SUB)
                receive.subscribe(b"hand.command")
                port = receive.bind_to_random_port("tcp://127.0.0.1")
                state = context.socket(zmq.PUB)
                state_port = state.bind_to_random_port("tcp://127.0.0.1")
                process = subprocess.Popen([sys.executable, SCRIPT, "--pose", "open",
                    "--duration", "3" if reply == "silent" else "0.6",
                    "--endpoint", f"tcp://127.0.0.1:{port}",
                    "--state-endpoint", f"tcp://127.0.0.1:{state_port}"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                messages = []
                try:
                    deadline = time.monotonic() + 6
                    while time.monotonic() < deadline:
                        if receive.poll(30):
                            msg = json.loads(receive.recv_multipart()[1])
                            messages.append(msg)
                            ack = {k: msg[k] for k in ("publisher_id", "session_id", "sequence", "sample_mono_us")}
                            if reply == "foreign":
                                ack["publisher_id"] = "another_controller"
                            if reply != "silent":
                                state.send_multipart([b"hand.state", json.dumps(dict(
                                    msg_type="HandState", command_valid=True, valid=False,
                                    feedback_only=reply == "readonly", accepted_command=ack,
                                    hands={})).encode()])
                        elif process.poll() is not None:
                            break
                    stdout, stderr = process.communicate(timeout=1)
                    self.assertEqual(process.returncode, 0 if reply == "accepted" else 1, stderr)
                    self.assertTrue(any(m["valid"] for m in messages))
                    self.assertFalse(messages[-1]["valid"], "exit did not invalidate commands")
                    if reply == "accepted":
                        self.assertIn("已接受", stdout)
                    elif reply == "silent":
                        self.assertIn("2 秒", stderr)
                    elif reply == "readonly":
                        self.assertIn("只读", stderr)
                    else:
                        self.assertIn("已绑定", stderr)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
                    receive.close(0)
                    state.close(0)
                    context.term()

    @unittest.skipUnless(HAS_ZMQ, "pyzmq required for private loopback integration")
    def test_zmq_interactive_and_actual_controller(self):
        import zmq
        context = zmq.Context()
        receive = context.socket(zmq.SUB)
        receive.subscribe(b"hand.command")
        command_port = receive.bind_to_random_port("tcp://127.0.0.1")
        state = context.socket(zmq.PUB)
        state_port = state.bind_to_random_port("tcp://127.0.0.1")
        process = subprocess.Popen([sys.executable, SCRIPT, "--interactive",
                                    "--endpoint", f"tcp://127.0.0.1:{command_port}",
                                    "--state-endpoint", f"tcp://127.0.0.1:{state_port}"],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        messages = []

        def enter(text):
            process.stdin.write(text + "\n")
            process.stdin.flush()

        def collect_until(predicate):
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                if not receive.poll(100):
                    continue
                frames = receive.recv_multipart()
                self.assertEqual(frames[0], b"hand.command")
                self.assertEqual(len(frames), 2)
                msg = json.loads(frames[1])
                messages.append(msg)
                self.assertLess(time.monotonic_ns() // 1000 - msg["sample_mono_us"], 100000)
                state.send_multipart([b"hand.state", json.dumps(dict(
                    msg_type="HandState", valid=False, command_valid=msg["valid"],
                    accepted_command={k: msg[k] for k in ("publisher_id", "session_id", "sequence", "sample_mono_us")},
                    hands={})).encode()])
                if predicate(msg):
                    return
            self.fail("publisher did not emit expected command")

        try:
            time.sleep(0.7)
            self.assertFalse(receive.poll(50), "interactive startup sent unsolicited command")
            enter("both open")
            collect_until(lambda m: len(messages) >= 12)
            enter("right 1 1 0.8 1 1 1")
            collect_until(lambda m: m.get("hands", {}).get("right", {}).get("drive_position_normalized", [0]*6)[2] == .8)
            self.assertEqual(messages[-1]["hands"]["left"]["drive_position_normalized"], [1] * 6)
            enter("stop")
            collect_until(lambda m: not m["valid"])
            enter("both half")
            collect_until(lambda m: m["valid"] and m["hands"]["left"]["drive_position_normalized"] == [.5] * 6)
            enter("quit")
            collect_until(lambda m: not m["valid"])
            stdout, stderr = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn("已接受", stdout)
            first = messages[0]
            for previous, msg in zip(messages, messages[1:]):
                self.assertGreater(msg["sequence"], previous["sequence"])
                for key in ("publisher_id", "session_id", "control_epoch", "clock_id"):
                    self.assertEqual(msg[key], first[key])
                self.assertEqual(msg["origin"]["sample_mono_us"], msg["sample_mono_us"])
            # Continuous publishing, even while terminal input is idle.
            elapsed = (messages[11]["sample_mono_us"] - first["sample_mono_us"]) / 1e6
            self.assertGreater(elapsed, .15)
            self.assertLess(elapsed, .7)
            controller = os.environ.get("HAND_CONTROLLER_TEST")
            if controller:
                with tempfile.TemporaryDirectory() as directory:
                    fixture = Path(directory) / "commands.json"
                    fixture.write_text(json.dumps(messages))
                    result = subprocess.run([controller, "--publisher-fixture", str(fixture)],
                                            capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
            receive.close(0)
            state.close(0)
            context.term()


if __name__ == "__main__":
    unittest.main()
