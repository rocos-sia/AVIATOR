#!/usr/bin/env python3
"""Run the t6 sine controller against a private MuJoCo bus."""
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

simulator, controller, actor, lut = [str(Path(p).resolve()) for p in sys.argv[1:5]]
duration = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
theta_amp = float(sys.argv[6]) if len(sys.argv) > 6 else .02
slide_amp = float(sys.argv[7]) if len(sys.argv) > 7 else .003
frequency = float(sys.argv[8]) if len(sys.argv) > 8 else .1
bus = os.getpid()
with tempfile.TemporaryDirectory(prefix="aviator-t6-sine-") as work:
    work = Path(work)
    sim_log = work / "simulator.log"
    control_log = work / "controller.log"
    result_csv = work / "sine.csv"
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
                [controller, actor, lut, str(result_csv), str(duration), str(theta_amp),
                 str(slide_amp), str(frequency), str(bus)],
                cwd=work, stdout=control_output, stderr=subprocess.STDOUT, timeout=50)
            control_output.flush()
            output = control_log.read_text()
            if result.returncode or "TEST COMPLETED" not in output:
                raise RuntimeError(output)
            with result_csv.open(newline="") as file:
                rows = list(csv.DictReader(file))
            expected = round(duration / .01) + 1
            if len(rows) != expected or any(row["status"] != "ok" for row in rows):
                raise RuntimeError(f"Unexpected sine samples/status: {len(rows)}")
            center = (float(rows[0]["theta_ref"]), float(rows[0]["s_ref"]))
            for row in rows:
                t = float(row["t"])
                phase = 2 * math.pi * frequency * t
                for axis, name, amplitude in ((0, "theta", theta_amp), (1, "s", slide_amp)):
                    if abs(float(row[f"{name}_ref"]) -
                           (center[axis] + amplitude * math.sin(phase))) > 1e-7 or \
                       abs(float(row[f"{name}_dot_ref"]) -
                           amplitude * 2 * math.pi * frequency * math.cos(phase)) > 1e-7:
                        raise RuntimeError("Wheel position/velocity reference is inconsistent")
            if len({row["wheel_knot"] for row in rows}) != round(duration / .02) + 1:
                raise RuntimeError("50 Hz wheel knot count is wrong")
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
            print(f"MuJoCo t6 sine: {expected} samples, all safe, "
                  f"max target-feedback step={max_feedback_step:.4f} rad/s")
            for line in output.splitlines():
                if line.startswith(("theta_rms=", "slide_rms=", "joint_tracking_rms=",
                                    "phase_reconstruction_max=", "t6_compute_mean=")):
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
