"""Home teaching UI/controller checks; real native sessions ALWAYS use dry-run."""
import copy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from get_home_ui import HomeController, HomeWindow, NativeSession

HELPER = Path(os.environ.get("AVIATOR_GRASP_TOOL_SESSION", ROOT / "build/bin/aviator_grasp_tool_session"))


class FakeSession:
    def __init__(self, *args):
        self.calls = []
        self.closed = False
        self.fail = None
        self.close_error = False
        self.count = 0
        self.state = {side: dict(connected=True, dragging=False, operation_state="idle")
                      for side in ("left", "right")}

    def request(self, op, **fields):
        self.calls.append((op, fields))
        if op == self.fail:
            raise RuntimeError("injected failure")
        if op in ("drag_start", "drag_stop"):
            assert fields["side"] == "both"
            for arm in self.state.values():
                arm.update(dragging=op == "drag_start", operation_state="drag" if op == "drag_start" else "idle")
        if op == "sample":
            assert not any(arm["dragging"] for arm in self.state.values())
            self.count += 1
            return dict(source="robot", **{side: dict(joint_position=[math.radians(self.count)] * 7)
                                          for side in ("left", "right")})
        return copy.deepcopy(self.state)

    def interrupt(self):
        self.calls.append(("interrupt", {}))

    def close(self):
        self.closed = True
        if self.close_error:
            raise RuntimeError("cleanup failed")


class HomeUiTests(unittest.TestCase):
    def setUp(self):
        import yaml
        self.temp = tempfile.TemporaryDirectory(prefix="aviator-home-ui-test-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.posture = self.directory / "posture.json"
        self.original = (ROOT / "config/posture.json").read_bytes()
        self.posture.write_bytes(self.original)
        config = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
        config.update(posture="posture.json", urdf=str(ROOT / "models/urdf/aviator.urdf"))
        self.robot = self.directory / "robot.yaml"
        self.robot.write_text(yaml.safe_dump(config))
        self.fake = FakeSession()
        self.controller = HomeController(self.robot, session_factory=lambda *args: self.fake)
        self.addCleanup(self.controller.close)
        helper_patch = patch("get_home_ui.state_helper", return_value=HELPER)
        helper_patch.start()
        self.addCleanup(helper_patch.stop)

    def test_connect_start_record_stops_both_and_saves_fresh_sample(self):
        self.controller.perform("connect")
        self.assertEqual(self.controller.angles["left_home_deg"], [1] * 7)
        self.controller.perform("start")
        self.assertIsNone(self.controller.angles)
        self.controller.perform("record")
        self.assertEqual([op for op, _ in self.fake.calls],
                         ["connect", "sample", "drag_start", "drag_stop", "sample"])
        actual = json.loads(self.posture.read_bytes())
        expected = json.loads(self.original)
        expected.update(left_home_deg=[2] * 7, right_home_deg=[2] * 7)
        self.assertEqual(actual, expected)
        self.assertEqual(self.controller.backup.read_bytes(), self.original)
        self.controller.perform("record")
        for value in json.loads(self.posture.read_bytes())["left_home_deg"]:
            self.assertAlmostEqual(value, 3)

    def test_stop_failure_never_samples_or_writes_and_closes_session(self):
        self.controller.perform("connect")
        self.controller.perform("start")
        self.fake.fail = "drag_stop"
        with self.assertRaisesRegex(RuntimeError, "injected"):
            self.controller.perform("record")
        self.assertEqual(self.fake.count, 1)
        self.assertTrue(self.fake.closed)
        self.assertEqual(self.posture.read_bytes(), self.original)

    def test_partial_connection_and_drag_start_failures_cleanup(self):
        for op in ("connect", "drag_start", "sample"):
            with self.subTest(op=op):
                self.fake = FakeSession()
                if op == "drag_start":
                    self.controller.perform("connect")
                self.fake.fail = op
                with self.assertRaisesRegex(RuntimeError, "injected"):
                    self.controller.perform("start" if op == "drag_start" else "connect")
                self.assertTrue(self.fake.closed)
                self.assertIsNone(self.controller.session)

    def test_external_config_change_does_not_get_overwritten(self):
        self.controller.perform("connect")
        changed = self.original + b"\n"
        self.posture.write_bytes(changed)
        with self.assertRaisesRegex(RuntimeError, "其他程序修改"):
            self.controller.perform("record")
        self.assertEqual(self.posture.read_bytes(), changed)
        self.assertTrue(self.fake.closed)

    def test_cleanup_failure_remains_visible_on_close(self):
        self.controller.perform("connect")
        self.fake.fail = "drag_start"
        self.fake.close_error = True
        with self.assertRaisesRegex(RuntimeError, "清理失败"):
            self.controller.perform("start")
        with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
            self.controller.close()
        self.controller.cleanup_error = None

    def test_deployed_ui_imports_and_help_without_source_tree(self):
        prefix = self.directory / "installed"
        resource = prefix / "share/aviator"
        scripts = resource / "scripts"
        tools = resource / "tools/grasp_tool_calibration"
        scripts.mkdir(parents=True)
        tools.mkdir(parents=True)
        for name in ("get_home.sh", "get_home.py", "get_home_ui.py", "runtime_env.sh"):
            shutil.copy2(ROOT / "scripts" / name, scripts / name)
        shutil.copy2(ROOT / "tools/grasp_tool_calibration/native_session.py", tools)
        (prefix / "release-version.json").write_text("{}")
        env = dict(os.environ, UI_PYTHON=sys.executable)
        result = subprocess.run([str(scripts / "get_home.sh"), "--ui", "--help"],
                                env=env, cwd=self.directory, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([sys.executable, "-c",
                                 "import get_home_ui, native_session; print(native_session.__file__)"],
                                cwd=scripts, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(Path(result.stdout.strip()), tools / "native_session.py")

    @unittest.skipUnless(HELPER.is_file(), "native helper not built")
    def test_real_native_demo_lifecycle_and_write_guard(self):
        self.controller = HomeController(self.robot, dry_run=True)
        self.addCleanup(self.controller.close)
        self.controller.perform("connect")
        session = self.controller.session
        self.controller.perform("start")
        self.assertTrue(self.controller.status["left"]["dragging"])
        self.controller.perform("stop")
        self.assertFalse(self.controller.status["right"]["dragging"])
        with self.assertRaisesRegex(RuntimeError, "禁止写入"):
            self.controller.perform("record")
        self.assertEqual(self.posture.read_bytes(), self.original)
        self.assertEqual(session.process.returncode, 0)
        self.assertIn("Session cleanup complete", session.log_path.read_text())

    @unittest.skipUnless(HELPER.is_file(), "native helper not built")
    def test_native_signal_cleanup_during_drag(self):
        self.controller = HomeController(self.robot, dry_run=True)
        self.addCleanup(self.controller.close)
        self.controller.perform("connect")
        session = self.controller.session
        self.controller.perform("start")
        self.controller.cancel()
        self.controller.close()
        self.assertIn("Session cleanup complete", session.log_path.read_text())
        self.assertEqual(self.posture.read_bytes(), self.original)

    @unittest.skipUnless(os.environ.get("DISPLAY") and HELPER.is_file(), "desktop/native helper unavailable")
    def test_desktop_buttons_and_close_during_drag(self):
        import tkinter as tk
        controller = HomeController(self.robot, dry_run=True)
        root = tk.Tk()
        root.withdraw()
        window = HomeWindow(root, controller)
        def settle():
            deadline = time.monotonic() + 10
            while window.future is not None and time.monotonic() < deadline:
                root.update()
                time.sleep(.01)
            self.assertIsNone(window.future, "UI operation did not finish")
        try:
            self.assertEqual(str(window.buttons["record"]["state"]), "disabled")
            window.buttons["connect"].invoke()
            settle()
            session = controller.session
            self.assertIsNotNone(session, window.message.get())
            window.buttons["start"].invoke()
            settle()
            self.assertTrue(window.view["status"]["left"]["dragging"])
            self.assertEqual(str(window.buttons["record"]["state"]), "disabled")
            self.assertEqual(window.table.item("0")["values"][2], "—")
            window.request_close()
            settle()
            self.assertIsNone(controller.session)
            self.assertIn("Session cleanup complete", session.log_path.read_text())
            self.assertEqual(self.posture.read_bytes(), self.original)
        finally:
            controller.cancel()
            window.pool.shutdown(wait=True)
            controller.close()
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == "__main__":
    unittest.main()
