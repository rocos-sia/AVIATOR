"""Startup log directory regressions; no servers or hardware are started."""
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from calibration import CalibrationError
import ui
import ui_backend


class InterpreterTests(unittest.TestCase):
    def test_virtualenv_survives_defaults_cli_and_executable_validation(self):
        with tempfile.TemporaryDirectory(prefix="grasp-ui-python-") as directory:
            environment = Path(directory) / "venv"
            subprocess.run([sys.executable, "-m", "venv", "--without-pip", "--symlinks", str(environment)], check=True)
            python = environment / "bin/python"
            self.assertTrue(python.is_symlink())
            for arguments, variables in (
                ([], {"CAMERA_PYTHON": str(python), "HAND_PYTHON": str(python)}),
                (["--camera-python", str(python), "--hand-python", str(python)], {}),
            ):
                with self.subTest(arguments=arguments), patch.dict(os.environ, variables, clear=True):
                    options = ui.parse_args(arguments)
                    for name in ("camera_python", "hand_python"):
                        selected = ui_backend.executable(getattr(options, name), name)
                        result = subprocess.check_output([str(selected), "-c", "import sys; print(sys.prefix)"], text=True).strip()
                        self.assertEqual(Path(result), environment)
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(ui.python_default("CAMERA_PYTHON", python), python)


class LogDirectoryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="grasp-ui-startup-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "repository"
        self.root.mkdir()
        self.root_patch = patch.object(ui, "ROOT", self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)

    def deny_writes(self, path):
        path.chmod(0o500)
        self.addCleanup(path.chmod, 0o700)
        if os.access(path, os.W_OK):
            self.skipTest("This user can bypass directory permission bits")

    def assert_writable(self, path):
        self.assertIsInstance(path, Path)
        self.assertTrue(path.is_dir())
        probe = path / "test-write.txt"
        probe.write_text("writable", encoding="utf-8")
        self.assertEqual(probe.read_text(encoding="utf-8"), "writable")
        probe.unlink()

    def test_default_creates_writable_repository_log_directory(self):
        stderr = StringIO()
        with redirect_stderr(stderr):
            directory = ui.prepare_log_directory(None)
        self.assertEqual(directory.parent, self.root / "logs/grasp_calibration_ui")
        self.assert_writable(directory)
        self.assertEqual(list(directory.iterdir()), [], "The permission probe must be removed")
        self.assertEqual(stderr.getvalue(), "")

    def test_default_unwritable_parent_falls_back_and_reports_location(self):
        parent = self.root / "logs"
        parent.mkdir()
        self.deny_writes(parent)
        stderr = StringIO()
        with redirect_stderr(stderr):
            directory = ui.prepare_log_directory(None)
        self.addCleanup(shutil.rmtree, directory)
        self.assertNotIn(self.root, directory.parents)
        self.assert_writable(directory)
        self.assertIn(str(directory), stderr.getvalue())
        self.assertEqual(list(parent.iterdir()), [], "Do not change the inaccessible parent")

    def test_explicit_directory_is_respected_and_writable(self):
        requested = Path(self.temporary.name) / "custom-logs" / "session"
        with patch.object(ui.tempfile, "mkdtemp", side_effect=AssertionError("unexpected fallback")):
            directory = ui.prepare_log_directory(requested)
        self.assertEqual(directory, requested)
        self.assert_writable(directory)

    def test_explicit_unwritable_parent_fails_without_fallback(self):
        parent = Path(self.temporary.name) / "private-logs"
        parent.mkdir()
        self.deny_writes(parent)
        requested = parent / "session"
        with patch.object(ui.tempfile, "mkdtemp", side_effect=AssertionError("unexpected fallback")):
            with self.assertRaises(CalibrationError) as raised:
                ui.prepare_log_directory(requested)
        self.assertIn(str(requested), str(raised.exception))
        self.assertFalse(requested.exists())

    def test_existing_directory_requires_write_probe_not_just_mkdir(self):
        requested = Path(self.temporary.name) / "existing-logs"
        requested.mkdir()
        self.deny_writes(requested)
        # This succeeds even when creating any log file inside it would fail.
        requested.mkdir(parents=True, exist_ok=True)
        with patch.object(ui.tempfile, "mkdtemp", side_effect=AssertionError("unexpected fallback")):
            with self.assertRaises(CalibrationError) as raised:
                ui.prepare_log_directory(requested)
        self.assertIn(str(requested), str(raised.exception))
        self.assertEqual(list(requested.iterdir()), [])

    def test_native_sdk_logs_follow_fallback_directory_and_config_stays_absolute(self):
        repository_logs = self.root / "logs"
        repository_logs.mkdir()
        self.deny_writes(repository_logs)
        helper = self.root / "fake-sdk-session"
        helper.write_text("#!" + sys.executable + "\n" + """
import json, pathlib, sys
config = pathlib.Path(sys.argv[sys.argv.index('--robot-config') + 1])
for line in sys.stdin:
    request = json.loads(line)
    log = pathlib.Path('logs/global_logger_test.log')
    log.write_text('SDK connected without touching repository logs')
    print(json.dumps(dict(id=request['id'], ok=True, result=dict(
        cwd=str(pathlib.Path.cwd()), config=str(config), content=config.read_text()))), flush=True)
""")
        helper.chmod(0o700)
        config = self.root / "robot config.yaml"
        config.write_text("test config")
        stderr = StringIO()
        with redirect_stderr(stderr):
            log_dir = ui.prepare_log_directory(None)
        self.addCleanup(shutil.rmtree, log_dir)
        with patch.object(ui_backend, "ROOT", self.root):
            session = ui_backend.NativeSession(Path(helper.name), Path(config.name), False, log_dir, lambda _: None)
        try:
            state = session.request("connect")
            self.assertEqual(Path(state["config"]), config)
            self.assertEqual(state["content"], "test config")
            self.assertEqual(Path(state["cwd"]), session.sdk_work_dir)
            self.assertEqual(session.sdk_work_dir.parent, log_dir)
            self.assertTrue((session.sdk_work_dir / "logs/global_logger_test.log").is_file())
            self.assertIn(str(session.sdk_work_dir), session.log_path.read_text())
            self.assertEqual(list(repository_logs.iterdir()), [])
        finally:
            session.close()
        self.assertEqual(session.process.returncode, 0)

    def test_help_does_not_prepare_or_create_directories(self):
        with patch.object(ui, "prepare_log_directory") as prepare, redirect_stdout(StringIO()):
            with self.assertRaises(SystemExit) as raised:
                ui.parse_args(["--help"])
        self.assertEqual(raised.exception.code, 0)
        prepare.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])

    def test_argument_parsing_keeps_default_implicit_without_writing(self):
        with patch.object(ui, "prepare_log_directory") as prepare:
            options = ui.parse_args(["--dry-run", "--no-browser"])
        self.assertIsNone(options.log_dir)
        prepare.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
