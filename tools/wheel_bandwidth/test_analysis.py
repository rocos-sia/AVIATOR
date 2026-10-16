import csv
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from analyze import analyze, bandwidth, frequency_result
from launch import parser, prepare_config, require_regular_user, select_output, ROOT


class AnalysisTest(unittest.TestCase):
    def test_sudo_rejected_before_user_environment_lookup(self):
        with patch("launch.os.geteuid", return_value=0):
            with self.assertRaisesRegex(RuntimeError, "去掉命令前的 sudo"):
                require_regular_user()

    @unittest.skipIf(os.geteuid() == 0, "Permission test requires an unprivileged user")
    def test_default_output_falls_back_when_project_logs_are_readonly(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "repo"
            home = Path(directory) / "home"
            logs = root / "logs"
            logs.mkdir(parents=True)
            home.mkdir()
            logs.chmod(0o555)
            try:
                output = select_output(None, root, home)
                self.assertEqual(output.parent, home / "aviator_logs")
                self.assertFalse(output.parent.exists())  # dry-run has no side effects
                output.mkdir(parents=True)
                (output / "probe.txt").write_text("writeable")
                with self.assertRaisesRegex(PermissionError, "--output"):
                    select_output(logs / "explicit-run", root, home)
            finally:
                logs.chmod(0o755)

    def test_output_keeps_writable_project_logs_and_explicit_destination(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(select_output(None, root).parent, root / "logs")
            chosen = root / "custom" / "run"
            self.assertEqual(select_output(chosen, root), chosen)
            self.assertFalse(chosen.parent.exists())
            chosen.mkdir(parents=True)
            with self.assertRaises(FileExistsError):
                select_output(chosen, root)

    def rows(self, f, gain, lag):
        rng = np.random.default_rng(23)
        t = np.arange(0, 5/f, .01)
        t[1:-1] += rng.uniform(-.002, .002, len(t)-2)
        return [dict(segment_t_s=str(x), command_rad=str(.17*np.sin(2*np.pi*f*x)),
                     estimated_rad=str(.012 + gain*.17*np.sin(2*np.pi*f*x-lag)),
                     arm_difference_rad="0.001", left_transverse_m="0.001", right_transverse_m="0.001",
                     left_tilt_rad="0.001", right_tilt_rad="0.001") for x in t]

    def test_known_response_irregular_samples_and_offset(self):
        result = frequency_result(self.rows(.7, .6, .8), .7, 5)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["gain"], .6, places=10)
        self.assertAlmostEqual(result["phase_deg"], -.8*180/math.pi, places=9)

    def test_incomplete_or_inconsistent_measurement(self):
        rows = self.rows(1, 1, 0)
        self.assertFalse(frequency_result(rows[:100], 1, 5)["valid"])
        rows[10]["arm_difference_rad"] = "0.2"
        self.assertIn("inconsistent_grasp_geometry", frequency_result(rows, 1, 5)["warnings"])

    def test_bandwidth_crossing_and_missing_crossing(self):
        results = [dict(frequency_hz=f, gain_db=g, valid=True) for f, g in ((.1, -1), (.5, -2), (1, -6))]
        self.assertAlmostEqual(bandwidth(results, True)[0], .5*2**.5)
        self.assertIsNone(bandwidth(results[:2], True)[0])
        self.assertIsNone(bandwidth(results, False)[0])
        results[1]["valid"] = False
        self.assertIsNone(bandwidth(results, True)[0])

    def test_ignore_geometry_preserves_other_rejections(self):
        results = []
        for f, gain in ((.5, 1), (1, .5)):
            rows = self.rows(f, gain, .1)
            rows[10]["left_transverse_m"] = ".04"
            results.append(frequency_result(rows, f, 5))
        self.assertEqual(bandwidth(results, True)[1], "invalid_frequency_measurements")
        self.assertEqual(bandwidth(results, True, True)[1], "crossing_estimated")
        self.assertFalse(results[0]["valid"])
        self.assertEqual(bandwidth(results, False, True)[1], "run_incomplete")
        for warning in ("incomplete_measurement_cycles", "measurement_gap",
                        "large_non_sinusoidal_residual", "insufficient_or_invalid_samples"):
            with self.subTest(warning=warning):
                results[0]["warnings"].append(warning)
                self.assertEqual(bandwidth(results, True, True)[1], "invalid_frequency_measurements")
                results[0]["warnings"].pop()

    def test_ignore_geometry_outputs_separate_results(self):
        self.assertFalse(parser().parse_args([]).ignore_geometry)
        self.assertTrue(parser().parse_args(["--ignore-geometry"]).ignore_geometry)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            rows = []
            for f, gain in ((.5, 1), (1, .5)):
                rows.extend(dict(r, frequency_hz=str(f), measure="1", left_transverse_m=".04")
                            for r in self.rows(f, gain, .1))
            with (path / "samples.csv").open("w") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            (path / "run.json").write_text(json.dumps(dict(
                completed=True, frequencies_hz=[.5, 1], measure_cycles=5)))
            analyze(path, plots=False)
            strict = (path / "summary.json").read_bytes()
            summary = analyze(path, plots=False, ignore_geometry=True)
            self.assertEqual(summary["bandwidth_status"], "crossing_estimated")
            self.assertTrue(summary["ignore_geometry"])
            self.assertTrue(summary["frequencies"][0]["usable_for_bandwidth"])
            self.assertFalse(summary["frequencies"][0]["valid"])
            self.assertEqual((path / "summary.json").read_bytes(), strict)
            self.assertTrue((path / "summary_geometry_ignored.json").exists())
            self.assertTrue((path / "frequency_response_geometry_ignored.csv").exists())

    def test_config_isolation_and_absolute_paths(self):
        source = ROOT / "config/system.yaml"
        robot_path = ROOT / "config/robot.yaml"
        before = (source.read_bytes(), robot_path.read_bytes())
        with tempfile.TemporaryDirectory() as directory:
            system, robot, original = prepare_config(source, directory)
            expected = yaml.safe_load(robot_path.read_text())
            self.assertEqual(robot["wheel_initial"], expected["wheel_initial"])
            self.assertEqual(robot["grasp_wheel"]["source"], "wheel_initial")
            self.assertTrue(Path(system["robot"]).is_absolute())
            for key in ("model", "urdf", "grasp", "posture"):
                self.assertTrue(Path(robot[key]).is_absolute())
                robot[key] = expected[key]
            expected["grasp_wheel"]["source"] = "wheel_initial"
            self.assertEqual(robot, expected)
            self.assertEqual(list(Path(directory).iterdir()), [])
        self.assertEqual(before, (source.read_bytes(), robot_path.read_bytes()))


if __name__ == "__main__":
    unittest.main()
