"""Capture/write regressions with fake SDK-helper output; never connect to robots."""
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class GetHomeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="aviator get home ")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.resource = self.directory / "source"
        self.prepare_resource(self.resource)
        self.bin = self.resource / "build/bin"
        self.bin.mkdir(parents=True)
        self.helper = self.bin / "aviator_grasp_tool_state"
        self.sample = {"source": "robot", "left": {"joint_position": [math.pi / 2] * 7},
                       "right": {"joint_position": [-math.pi / 4] * 7}}
        self.set_helper([self.sample, self.sample])
        self.env = dict(os.environ, UI_PYTHON=sys.executable, XDG_RUNTIME_DIR=str(self.directory))
        self.env.pop("AVIATOR_BIN", None)

    def prepare_resource(self, root):
        (root / "scripts").mkdir(parents=True)
        for name in ("get_home.sh", "get_home.py", "runtime_env.sh"):
            shutil.copy2(ROOT / "scripts" / name, root / "scripts" / name)
        (root / "config").mkdir()
        (root / "config/robot.yaml").write_text("posture: posture.json\n")
        self.posture = root / "config/posture.json"
        self.original = (ROOT / "config/posture.json").read_bytes()
        self.posture.write_bytes(self.original)
        self.posture.chmod(0o640)

    def set_helper(self, samples, status=0):
        self.helper.write_text(
            f"#!{sys.executable}\nimport json, sys\nfrom pathlib import Path\n"
            f"Path({str(self.directory / 'args.json')!r}).write_text(json.dumps(sys.argv[1:]))\n"
            f"print({chr(10).join(json.dumps(sample) for sample in samples)!r})\n"
            f"sys.exit({status})\n")
        self.helper.chmod(0o755)

    def run_capture(self, *args, ok=True):
        result = subprocess.run([str(self.resource / "scripts/get_home.sh"), *map(str, args)],
                                cwd=self.directory, env=self.env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode == 0, ok, result.stdout + result.stderr)
        return result

    def assert_unchanged(self):
        self.assertEqual(self.posture.read_bytes(), self.original)
        self.assertEqual(list(self.posture.parent.glob("*.bak")), [])

    def test_source_capture_converts_latest_sample_and_preserves_other_fields(self):
        first = {"source": "robot", "left": {"joint_position": [0] * 7},
                 "right": {"joint_position": [0] * 7}}
        self.set_helper([first, self.sample])
        self.run_capture()
        actual = json.loads(self.posture.read_bytes())
        expected = json.loads(self.original)
        expected.update(left_home_deg=[90.0] * 7, right_home_deg=[-45.0] * 7)
        self.assertEqual(actual, expected)
        backups = list(self.posture.parent.glob("posture.json.*.bak"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_bytes(), self.original)
        self.assertEqual(self.posture.stat().st_mode & 0o777, 0o640)
        arguments = json.loads((self.directory / "args.json").read_text())
        self.assertEqual(arguments, ["--robot-config", str(self.resource / "config/robot.yaml"),
                                     "--read-robot", "--samples", "2", "--interval-ms", "50"])

    def test_deployed_layout_uses_bundled_python_and_bin_without_source_tree(self):
        prefix = self.directory / "installed"
        self.resource = prefix / "share/aviator"
        self.prepare_resource(self.resource)
        (prefix / "release-version.json").write_text("{}")
        (prefix / "bin").mkdir()
        shutil.copy2(self.helper, prefix / "bin" / self.helper.name)
        python = prefix / "cache/python/bin/python"
        python.parent.mkdir(parents=True)
        python.symlink_to(sys.executable)
        self.env.pop("UI_PYTHON")
        self.run_capture()
        self.assertEqual(json.loads(self.posture.read_bytes())["left_home_deg"], [90.0] * 7)

    def test_partial_output_or_failed_helper_never_writes(self):
        for samples, status in (([self.sample], 0), ([self.sample, self.sample], 1), ([], 0)):
            with self.subTest(status=status, count=len(samples)):
                self.set_helper(samples, status)
                self.run_capture(ok=False)
                self.assert_unchanged()

    def test_invalid_feedback_never_writes(self):
        cases = [dict(self.sample, source="offline"), dict(self.sample, right={})]
        for value in (True, None, "1.0", float("nan"), float("inf")):
            cases.append(dict(self.sample, right={"joint_position": [value] * 7}))
        for sample in cases:
            with self.subTest(sample=sample):
                self.set_helper([self.sample, sample])
                self.run_capture(ok=False)
                self.assert_unchanged()

    def test_explicit_binary_directory_does_not_fall_back(self):
        self.env["AVIATOR_BIN"] = str(self.directory / "missing")
        self.run_capture(ok=False)
        self.assert_unchanged()
        self.assertFalse((self.directory / "args.json").exists())

    def test_launch_lock_blocks_acquisition(self):
        lock = self.directory / f"aviator-start-{os.getuid()}.lock"
        with lock.open("a") as guard:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.run_capture(ok=False)
        self.assert_unchanged()
        self.assertFalse((self.directory / "args.json").exists())

    def test_custom_relative_config_and_posture_path(self):
        custom = self.directory / "custom"
        custom.mkdir()
        (custom / "robot.yaml").write_text("posture: ../source/config/posture.json\n")
        self.run_capture("--robot-config", "custom/robot.yaml")
        self.assertEqual(json.loads(self.posture.read_bytes())["right_home_deg"], [-45.0] * 7)

    def test_help_does_not_read_robot(self):
        self.run_capture("--help")
        self.assert_unchanged()
        self.assertFalse((self.directory / "args.json").exists())


if __name__ == "__main__":
    unittest.main()
