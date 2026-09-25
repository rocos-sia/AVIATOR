#!/usr/bin/env python3
"""Run the t6 joystick controller against a private MuJoCo bus.

Unlike run_t6_sine.py this does not require USB hardware: it runs with a
non-existent joystick path, exercising the "disconnected -> neutral hold" path.
It validates that the driver starts, acquires phase, produces a well-formed CSV
and never exceeds the joint-speed gate; interactive rejections (holds) are
expected and are reported rather than treated as failures.
"""
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

simulator, controller, actor, lut = [str(Path(p).resolve()) for p in sys.argv[1:5]]
duration = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
joystick = sys.argv[6] if len(sys.argv) > 6 else "/dev/input/js_nonexistent"
bus = os.getpid()
with tempfile.TemporaryDirectory(prefix="aviator-t6-joystick-") as work:
    work = Path(work)
    sim_log = work / "simulator.log"
    control_log = work / "controller.log"
    result_csv = work / "joystick.csv"
    with sim_log.open("w") as sim_output, control_log.open("w") as control_output:
        sim = subprocess.Popen(
            [simulator, "--model", "aviator", "--headless", "--ecat-id", str(bus),
             "--duration", "60"], cwd=work, stdout=sim_output,
            stderr=subprocess.STDOUT)
        try:
            for _ in range(200):
                if (Path("/dev/shm") / f"aviator{bus}").exists():
                    break
                if sim.poll() is not None:
                    raise RuntimeError("Simulator exited during startup")
                time.sleep(.05)
            else:
                raise RuntimeError("Simulator did not start")
            result = subprocess.run(
                [controller, actor, lut, str(result_csv), str(duration),
                 joystick, str(bus)],
                cwd=work, stdout=control_output, stderr=subprocess.STDOUT, timeout=50)
            control_output.flush()
            output = control_log.read_text()
            if result.returncode or "TEST COMPLETED" not in output:
                raise RuntimeError(output)
            with result_csv.open(newline="") as file:
                rows = list(csv.DictReader(file))
            expected = round(duration / .01) + 1
            if len(rows) != expected:
                raise RuntimeError(f"Unexpected joystick samples: {len(rows)} != {expected}")
            if joystick == "/dev/input/js_nonexistent":
                max_center_error = max(abs(float(row["s_ref"]) + .08) for row in rows)
                if max_center_error > .001:
                    raise RuntimeError(f"Disconnected joystick left the -80 mm centre: {max_center_error}")
            holds = sum(1 for row in rows if row["status"] != "ok")
            max_feedback_step = 0.0
            for row in rows:
                if row["has_command"] != "1":
                    continue
                q = [float(row[f"q_cmd_{axis}"]) for axis in range(14)]
                measured = [float(row[f"q_meas_{axis}"]) for axis in range(14)]
                max_feedback_step = max(max_feedback_step, *(
                    abs(a - b) / .01 for a, b in zip(q, measured)))
            if max_feedback_step > 1.5001:
                raise RuntimeError(f"Joint targets exceed measured-step limit: {max_feedback_step}")
            print(f"MuJoCo t6 joystick: {expected} samples, {holds} hold cycles, "
                  f"max target-feedback step={max_feedback_step:.4f} rad/s")
            for line in output.splitlines():
                if line.startswith(("theta_rms=", "slide_rms=", "joint_tracking_rms=",
                                    "phase_reconstruction_max=", "control_compute_mean=")):
                    print(line)
        except Exception:
            sim_output.flush()
            control_output.flush()
            print(sim_log.read_text())
            print(control_log.read_text())
            raise
        finally:
            sim.terminate()
            try:
                sim.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sim.kill()
                sim.wait()
            for name in [f"aviator{bus}", f"ecm{bus}", f"pd_input{bus}",
                         f"pd_output{bus}", *[f"sem.sync{bus}_{i}" for i in range(10)]]:
                (Path("/dev/shm") / name).unlink(missing_ok=True)
