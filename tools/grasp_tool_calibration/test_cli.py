"""Exercise the native Core FK helper and CLI using only offline joint files."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

from calibration import pose_matrix

ROOT = Path(__file__).resolve().parents[2]
HELPER = Path(os.environ.get("AVIATOR_GRASP_TOOL_STATE", ROOT / "build/bin/aviator_grasp_tool_state"))


@unittest.skipUnless(HELPER.is_file(), "build aviator_grasp_tool_state to run native offline tests")
class NativeCliTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="aviator-grasp-cli-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.original = (ROOT / "config/grasp.json").read_bytes()
        (self.directory / "grasp.json").write_bytes(self.original)
        config = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
        for name in ("posture", "urdf"):
            config[name] = str((ROOT / "config" / config[name]).resolve())
        config["grasp"] = "grasp.json"
        self.robot = self.directory / "robot.yaml"
        self.robot.write_text(yaml.safe_dump(config))
        self.joints = self.directory / "joints.json"
        self.joints.write_text(json.dumps({"left": [0, 1.5, 0, 0, 0, 0, 0],
                                           "right": [0, 1.5, 0, 0, 0, 0, 0]}))
        self.report = self.directory / "report.json"

    def run_cli(self, *extra):
        return subprocess.run([
            sys.executable, str(ROOT / "tools/grasp_tool_calibration/calibrate.py"),
            "--robot-config", str(self.robot), "--state-helper", str(HELPER),
            "--joints-file", str(self.joints), "--wheel-source", "manual",
            "--angle-deg", "12", "--displacement-m", "-0.06", "--samples", "3",
            "--interval-ms", "2", "--report", str(self.report), *extra],
            capture_output=True, text=True, timeout=15)

    def test_preview_reconstructs_flange_with_native_fk(self):
        result = self.run_cli()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((self.directory / "grasp.json").read_bytes(), self.original)
        self.assertEqual(list(self.directory.glob("*.bak")), [])
        report = json.loads(self.report.read_text())
        self.assertEqual(report["joint_source"], "offline")
        self.assertFalse(report["write_requested"])
        self.assertEqual(report["sample_count"], 3)
        # Check the recovered tool against independently assembled rigid transforms.
        grasp = json.loads(self.original)
        angle = math.radians(12)
        motion = np.eye(4)
        c, s = math.cos(angle), math.sin(angle)
        motion[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        motion[2, 3] = -0.06
        for side in ("left", "right"):
            tool = pose_matrix(report["arms"][side]["new"])
            expected = pose_matrix(grasp["wheel_origin"]) @ motion @ pose_matrix(grasp[side])
            for sample in report["samples"]:
                self.assertEqual(sample["source"], "offline")
                actual = pose_matrix(sample[side]["flange"]) @ tool
                np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_write_preserves_fields_and_original_byte_backup(self):
        result = self.run_cli("--write")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        backups = list(self.directory.glob("grasp.json.*.bak"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_bytes(), self.original)
        original = json.loads(self.original)
        updated = json.loads((self.directory / "grasp.json").read_text())
        for side in ("left", "right"):
            self.assertNotEqual(updated["tool"][side], original["tool"][side])
            updated["tool"][side] = original["tool"][side]
        self.assertEqual(updated, original)

    def test_native_rejects_bad_joint_file_without_write(self):
        self.joints.write_text(json.dumps({"left": [100] * 7, "right": [0] * 7}))
        result = self.run_cli("--write")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("URDF", result.stderr)
        self.assertEqual((self.directory / "grasp.json").read_bytes(), self.original)
        self.assertFalse(self.report.exists())

    def test_invalid_grasp_fails_before_starting_helper(self):
        original = json.loads(self.original)
        original["tool"]["left"]["quaternion"] = [0, 0, 0, 0]
        (self.directory / "grasp.json").write_text(json.dumps(original))
        marker = self.directory / "helper-started"
        helper = self.directory / "never-run"
        helper.write_text("#!/usr/bin/env python3\nfrom pathlib import Path\n"
                          f"Path({str(marker)!r}).write_text('started')\n")
        helper.chmod(0o755)
        result = self.run_cli("--state-helper", str(helper))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("quaternion", result.stderr)
        self.assertFalse(marker.exists())

    def test_report_cannot_replace_input_file(self):
        result = self.run_cli("--report", str(self.directory / "grasp.json"))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.directory / "grasp.json").read_bytes(), self.original)


if __name__ == "__main__":
    unittest.main()
