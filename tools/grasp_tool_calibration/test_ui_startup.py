"""Startup log directory regressions; no servers or hardware are started."""
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from calibration import CalibrationError
import ui


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
