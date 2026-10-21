import csv
import contextlib
import io
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from analyze import analyze, bandwidth, frequency_result, phase_crossings, unwrapped_phases
from launch import (parser, prepare_config, require_regular_user, select_output,
                    load_sweep_config, validate_motion, main, ROOT)


class AnalysisTest(unittest.TestCase):
    def options(self, *values):
        args = parser().parse_args(list(values))
        load_sweep_config(args)
        return args

    def test_sweep_config_defaults_and_cli_override(self):
        config = yaml.safe_load((ROOT / "config/wheel_bandwidth.yaml").read_text())
        args = self.options()
        self.assertEqual(args.axis, config["axis"])
        self.assertEqual(args.amplitude_deg, config["amplitude_deg"] if args.axis == "rotation" else None)
        self.assertEqual(args.amplitude_mm, config["amplitude_mm"] if args.axis == "translation" else None)
        for key in ("settle_cycles", "measure_cycles", "ramp_cycles", "fake_hand", "ignore_geometry"):
            self.assertEqual(getattr(args, key), config[key])
        args = self.options("--axis", "translation", "--amplitude-mm", "12",
                            "--frequencies", "0.2,0.4", "--measure-cycles", "3", "--fake-hand")
        self.assertEqual((args.axis, args.amplitude_deg, args.amplitude_mm), ("translation", None, 12))
        self.assertEqual((args.frequencies, args.measure_cycles), ("0.2,0.4", 3))
        self.assertTrue(args.fake_hand)

    def test_custom_sweep_config_and_invalid_values(self):
        config = yaml.safe_load((ROOT / "config/wheel_bandwidth.yaml").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep.yaml"
            config.update(axis="translation", fake_hand=True)
            path.write_text(yaml.safe_dump(config))
            args = self.options("--sweep-config", str(path))
            self.assertEqual(args.amplitude_mm, 20)
            self.assertTrue(args.fake_hand)
            self.assertFalse(self.options("--sweep-config", str(path), "--no-fake-hand").fake_hand)
            for key, bad in (("axis", "both"), ("fake_hand", "false"), ("ramp_cycles", 1.5),
                             ("amplitude_mm", float("nan")), ("frequencies_hz", "0.1,0.2")):
                with self.subTest(key=key):
                    path.write_text(yaml.safe_dump(dict(config, **{key: bad})))
                    with self.assertRaises(ValueError):
                        self.options("--sweep-config", str(path))

    def test_translation_range_and_axis_validation(self):
        robot = {"wheel_initial": {"displacement": -.085}}
        validate_motion(self.options("--axis", "translation"), robot)
        for values in (("--amplitude-mm", "20"), ("--axis", "translation", "--amplitude-deg", "3"),
                       ("--axis", "translation", "--amplitude-mm", "nan"),
                       ("--axis", "translation", "--amplitude-mm", "0"),
                       ("--axis", "translation", "--amplitude-mm", "86")):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_motion(self.options(*values), robot)
        for center in (-.160, -.010):
            with self.assertRaises(ValueError):
                validate_motion(self.options("--axis", "translation"),
                                {"wheel_initial": {"displacement": center}})

    def test_dry_run_translation_fake_hand_and_simulation_exclusion(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            argv = ["launch.py", "--axis", "translation", "--fake-hand", "--dry-run",
                    "--output", str(output)]
            text = io.StringIO()
            with patch("sys.argv", argv), patch("launch.require_regular_user"), \
                    patch("launch.subprocess.Popen") as popen, contextlib.redirect_stdout(text):
                self.assertEqual(main(), 0)
                popen.assert_not_called()
            self.assertFalse(output.exists())
            self.assertIn("--axis translation --amplitude-mm 20", text.getvalue())
            self.assertIn("--fake-hand", text.getvalue())
            self.assertIn("-105..-65 mm", text.getvalue())
            with patch("sys.argv", argv + ["--simulation"]), patch("launch.require_regular_user"):
                with self.assertRaisesRegex(ValueError, "不能同时使用"):
                    main()

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

    def translation_rows(self, f, gain, lag):
        rows = self.rows(f, gain, lag)
        for r in rows:
            t = float(r["segment_t_s"])
            r.update(command_displacement_m=str(-.085 + .020*np.sin(2*np.pi*f*t)),
                     estimated_displacement_m=str(-.083 + gain*.020*np.sin(2*np.pi*f*t-lag)),
                     reference_displacement_m="-.085", arm_difference_m=".001", t_s=str(t),
                     frequency_hz=str(f), measure="1")
        return rows

    def test_translation_response_units_offset_and_geometry(self):
        rows = self.translation_rows(.7, .6, .8)
        result = frequency_result(rows, .7, 5, "translation")
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["gain"], .6, places=10)
        self.assertAlmostEqual(result["phase_deg"], -.8*180/math.pi, places=9)
        self.assertAlmostEqual(result["input_amplitude_mm"], 20, places=9)
        self.assertAlmostEqual(result["output_amplitude_mm"], 12, places=9)
        self.assertNotIn("input_amplitude_deg", result)
        rows[10]["arm_difference_m"] = ".03"
        self.assertIn("inconsistent_grasp_geometry",
                      frequency_result(rows, .7, 5, "translation")["warnings"])

    def test_translation_analysis_and_interrupted_run_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            rows = self.translation_rows(.5, 1, .1) + self.translation_rows(1, .5, .1)
            with (path / "samples.csv").open("w") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            (path / "run.json").write_text(json.dumps(dict(
                completed=True, axis="translation", frequencies_hz=[.5, 1], measure_cycles=5)))
            result = analyze(path, plots=True)
            self.assertEqual(result["axis"], "translation")
            self.assertEqual(result["bandwidth_status"], "crossing_estimated")
            self.assertTrue((path / "tracking.png").exists())
            self.assertTrue((path / "frequency_response.png").exists())
            (path / "run.json").unlink()
            (path / "launch.json").write_text(json.dumps({"options": {
                "axis": "translation", "frequencies": ".5,1", "measure_cycles": 5}}))
            result = analyze(path, plots=False)
            self.assertEqual(result["axis"], "translation")
            self.assertEqual(result["bandwidth_status"], "run_incomplete")
            self.assertFalse(result["has_estimated_crossing"])
            for crossing in result["phase_crossings"].values():
                self.assertEqual(crossing["status"], "run_incomplete")

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

    def response(self, phases, gains=None):
        gains = gains if gains is not None else [0] * len(phases)
        return [dict(frequency_hz=2**i, phase_deg=phase, gain_db=gain, valid=True, warnings=[])
                for i, (phase, gain) in enumerate(zip(phases, gains))]

    def test_phase_crossings_log_interpolation_and_first_downward_crossing(self):
        results = self.response([-10, -50, -20, -100])
        crossings = phase_crossings(results, True)
        self.assertAlmostEqual(crossings["minus_30_deg"]["frequency_hz"], 2**.5)
        self.assertAlmostEqual(crossings["minus_45_deg"]["frequency_hz"], 2**.875)
        self.assertAlmostEqual(crossings["minus_90_deg"]["frequency_hz"], 4 * 2**.875)
        # A measured endpoint on the threshold is a crossing, with no extrapolation.
        self.assertAlmostEqual(phase_crossings(self.response([0, -90]), True)
                               ["minus_90_deg"]["frequency_hz"], 2)

    def test_phase_unwrap_and_missing_low_frequency_bracket(self):
        results = self.response([-10, -170, 100])
        np.testing.assert_allclose(unwrapped_phases(results), [-10, -170, -260])
        self.assertAlmostEqual(phase_crossings(results, True)["minus_90_deg"]["frequency_hz"], 2**.5)
        # First frequency is already beyond -30/-45: do not invent an earlier
        # crossing or substitute the later downward recrossing.
        crossings = phase_crossings(self.response([-60, -10, -100]), True)
        for key in ("minus_30_deg", "minus_45_deg"):
            self.assertIsNone(crossings[key]["frequency_hz"])
            self.assertEqual(crossings[key]["status"],
                             "phase_threshold_already_reached_at_first_frequency")
        self.assertEqual(crossings["minus_90_deg"]["status"], "crossing_estimated")
        for crossing in phase_crossings(self.response([10, 50]), True).values():
            self.assertEqual(crossing["status"], "phase_crossing_not_observed")

    def test_phase_quality_gates_and_geometry_policy(self):
        results = self.response([-10, -50, -100])
        for subset, completed, expected in (([], True, "need_multiple_frequencies"),
                                            (results[:1], True, "need_multiple_frequencies"),
                                            (results, False, "run_incomplete")):
            for crossing in phase_crossings(subset, completed).values():
                self.assertIsNone(crossing["frequency_hz"])
                self.assertEqual(crossing["status"], expected)
        results[1].update(valid=False, warnings=["inconsistent_grasp_geometry"])
        phases = unwrapped_phases(results)
        self.assertIsNone(phases[1])
        self.assertAlmostEqual(phases[2], -100)
        for crossing in phase_crossings(results, True).values():
            self.assertEqual(crossing["status"], "invalid_frequency_measurements")
        for crossing in phase_crossings(results, True, True).values():
            self.assertEqual(crossing["status"], "crossing_estimated")
        for warning in ("incomplete_measurement_cycles", "measurement_gap",
                        "large_non_sinusoidal_residual", "insufficient_or_invalid_samples"):
            results[1]["warnings"].append(warning)
            for crossing in phase_crossings(results, True, True).values():
                self.assertEqual(crossing["status"], "invalid_frequency_measurements")
            results[1]["warnings"].pop()

    def test_independent_crossing_reports_for_both_axes(self):
        # Gain-only, phase-only (partial/all), both, and neither must all produce
        # JSON/CSV/terminal reports without making one metric depend on another.
        for axis in ("rotation", "translation"):
            for gain, lag_deg in ((.5, 20), (1, 40), (1, 100), (.5, 100), (1, 20)):
                with self.subTest(axis=axis, gain=gain, lag_deg=lag_deg), \
                        tempfile.TemporaryDirectory() as directory:
                    path = Path(directory)
                    rows = []
                    for f, g, lag in ((.5, 1, 10), (1, gain, lag_deg)):
                        if axis == "translation":
                            rows.extend(self.translation_rows(f, g, np.deg2rad(lag)))
                        else:
                            rows.extend(dict(r, t_s=r["segment_t_s"], reference_rad=r["command_rad"],
                                             frequency_hz=str(f), measure="1")
                                        for r in self.rows(f, g, np.deg2rad(lag)))
                    with (path / "samples.csv").open("w") as file:
                        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                        writer.writeheader()
                        writer.writerows(rows)
                    (path / "run.json").write_text(json.dumps(dict(
                        completed=True, axis=axis, frequencies_hz=[.5, 1], measure_cycles=5)))
                    output = io.StringIO()
                    with contextlib.redirect_stdout(output):
                        summary = analyze(path, plots=(gain == 1 and lag_deg == 40))
                    self.assertEqual(summary["bandwidth_hz"] is not None, gain == .5)
                    self.assertEqual(summary["has_estimated_crossing"], gain == .5 or lag_deg >= 30)
                    for threshold in (30, 45, 90):
                        crossing = summary["phase_crossings"][f"minus_{threshold}_deg"]
                        self.assertEqual(crossing["frequency_hz"] is not None, lag_deg >= threshold)
                        self.assertIn(f"-{threshold}°相位交叉频率", output.getvalue())
                    self.assertEqual(json.loads((path / "summary.json").read_text()), summary)
                    with (path / "crossings.csv").open() as file:
                        metrics = list(csv.DictReader(file))
                    self.assertEqual(len(metrics), 4)
                    self.assertEqual(bool(metrics[0]["frequency_hz"]), gain == .5)
                    self.assertEqual(bool(metrics[1]["frequency_hz"]), lag_deg >= 30)
                    with (path / "frequency_response.csv").open() as file:
                        response = list(csv.DictReader(file))
                    self.assertAlmostEqual(float(response[-1]["phase_unwrapped_deg"]), -lag_deg)
                    if not summary["has_estimated_crossing"]:
                        self.assertIn("未观测到交叉", output.getvalue())

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
                            for r in self.rows(f, gain, f * 2))
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
            self.assertEqual(summary["phase_crossings"]["minus_90_deg"]["status"], "crossing_estimated")
            self.assertEqual(json.loads(strict)["phase_crossings"]["minus_90_deg"]["status"],
                             "invalid_frequency_measurements")
            self.assertEqual((path / "summary.json").read_bytes(), strict)
            self.assertTrue((path / "summary_geometry_ignored.json").exists())
            self.assertTrue((path / "frequency_response_geometry_ignored.csv").exists())
            self.assertTrue((path / "crossings_geometry_ignored.csv").exists())

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
