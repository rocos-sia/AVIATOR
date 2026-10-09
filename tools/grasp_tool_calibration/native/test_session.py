#!/usr/bin/env python3
"""Offline protocol regression tests. This suite ALWAYS uses --dry-run.

python3 tools/grasp_tool_calibration/native/test_session.py \
    --binary build/release/bin/aviator_grasp_tool_session
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import selectors
import signal
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]
BINARY = ROOT / "build/release/bin/aviator_grasp_tool_session"
CONFIG = ROOT / "config/robot.yaml"


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.process = subprocess.Popen(
            [str(BINARY), "--robot-config", str(CONFIG), "--dry-run"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        self.next_id = 0

    def tearDown(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
                self.fail("Dry-run session did not clean up on EOF")
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            stream.close()

    def send(self, payload):
        self.process.stdin.write(json.dumps(payload) + "\n")
        self.process.stdin.flush()
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            self.assertTrue(selector.select(10), "Timed out waiting for native JSON response")
        line = self.process.stdout.readline()
        self.assertTrue(line, "Native session exited without a response")
        return json.loads(line)

    def request(self, op, ok=True, **fields):
        self.next_id += 1
        response = self.send({"id": self.next_id, "op": op, **fields})
        self.assertEqual(response["id"], self.next_id)
        self.assertEqual(response["ok"], ok, response)
        return response.get("result") if ok else response["error"]

    def test_sample_uses_real_fk_and_monotonic_acquisition_times(self):
        before = self.request("status")
        self.assertTrue(before["dry_run"])
        self.assertFalse(before["left"]["connected"])
        self.request("sample", ok=False)
        self.request("connect")
        sample = self.request("sample")
        self.assertEqual(sample["source"], "offline")
        self.assertEqual(sample["frame"], "aircraft")
        self.assertTrue(sample["dry_run"])
        self.assertLessEqual(sample["sample_start_mono_us"], sample["sample_mono_us"])
        self.assertLessEqual(sample["sample_mono_us"], sample["sample_end_mono_us"])
        for side in ("left", "right"):
            self.assertEqual(sample[side]["joint_position"], [0, 1.5, 0, 0, 0, 0, 0])
            pose = sample[side]["flange"]
            self.assertEqual(len(pose["position"]), 3)
            self.assertTrue(all(math.isfinite(value) for value in pose["position"]))
            self.assertEqual(len(pose["quaternion"]), 4)
            self.assertAlmostEqual(sum(value**2 for value in pose["quaternion"]), 1.0)
        self.assertNotEqual(sample["left"]["flange"]["position"], sample["right"]["flange"]["position"])
        following = self.request("sample")
        self.assertGreater(following["sample_index"], sample["sample_index"])
        self.assertGreaterEqual(following["sample_start_mono_us"], sample["sample_end_mono_us"])

    def test_one_arm_still_dragging_blocks_dual_arm_sample(self):
        self.request("drag_start", ok=False, side="both")
        self.request("connect")
        both = self.request("drag_start", side="both")
        self.assertTrue(both["left"]["drag_owned"])
        self.assertTrue(both["right"]["dragging"])
        self.request("sample", ok=False)
        left_only = self.request("drag_stop", side="left")
        self.assertFalse(left_only["left"]["dragging"])
        self.assertTrue(left_only["right"]["dragging"])
        self.request("sample", ok=False)
        stopped = self.request("drag_stop", side="both")
        self.assertFalse(stopped["right"]["dragging"])
        self.request("sample")

    def test_disconnect_ends_owned_drag_and_allows_reconnect(self):
        self.request("connect")
        self.request("drag_start", side="both")
        disconnected = self.request("disconnect")
        for side in ("left", "right"):
            self.assertFalse(disconnected[side]["connected"])
            self.assertFalse(disconnected[side]["dragging"])
            self.assertIsNone(disconnected[side]["powered"])
        self.request("sample", ok=False)
        self.request("connect")
        self.request("sample")

    def test_malformed_requests_do_not_kill_service(self):
        self.assertFalse(self.send({"id": True, "op": "status"})["ok"])
        self.assertFalse(self.send([])["ok"])
        self.request("unknown", ok=False)
        self.request("drag_start", ok=False, side="feet")
        self.request("drag_start", ok=False)
        self.request("connect")
        self.request("sample")

    def test_eof_cleans_up_while_dragging(self):
        self.request("connect")
        self.request("drag_start", side="both")
        self.process.stdin.close()
        self.assertEqual(self.process.wait(timeout=10), 0)
        self.assertIn("Session cleanup complete (dry run)", self.process.stderr.read())

    def test_sigterm_cleans_up_while_waiting_for_input(self):
        self.request("connect")
        self.request("drag_start", side="both")
        self.process.send_signal(signal.SIGTERM)
        self.assertEqual(self.process.wait(timeout=10), 128 + signal.SIGTERM)
        self.assertIn("Session cleanup complete (dry run)", self.process.stderr.read())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=BINARY)
    parser.add_argument("--robot-config", type=Path, default=CONFIG)
    args, unittest_args = parser.parse_known_args()
    BINARY = args.binary.resolve()
    CONFIG = args.robot_config.resolve()
    unittest.main(argv=[__file__, *unittest_args])
