#!/usr/bin/env python3
"""Measure controller send, simulator actuator receipt and joint response."""
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

simulator, probe, config, output = map(lambda p: str(Path(p).resolve()), sys.argv[1:5])
lut = str(Path(sys.argv[5]).resolve()) if len(sys.argv) > 5 else None
wheel_step = sys.argv[6] if len(sys.argv) > 6 else None
output = Path(output)
output.mkdir(parents=True, exist_ok=True)
bus = os.getpid()
with tempfile.TemporaryDirectory(prefix="aviator-joint-step-") as tmp:
    tmp = Path(tmp)
    sim_trace = tmp / "sim_trace.csv"
    send_trace = tmp / "send_trace.csv"
    feedback = output / "joint_step_feedback.csv"
    sim_log = tmp / "sim.log"
    probe_log = tmp / "probe.log"
    sim_env = os.environ.copy()
    sim_env.update(AVIATOR_STEP_TRACE_FILE=str(sim_trace),
                   AVIATOR_STEP_TRACE_JOINT="AR5-5_07L-W4C4A2_joint_1")
    probe_env = os.environ.copy()
    probe_env["AVIATOR_JOINT_SEND_TRACE_FILE"] = str(send_trace)
    with sim_log.open("w") as sim_out, probe_log.open("w") as probe_out:
        sim_command = [simulator, "--model", "aviator", "--headless",
                       "--ecat-id", str(bus), "--duration", "20"]
        if os.environ.get("AVIATOR_STEP_MODEL"):
            sim_command += ["--urdf", str(Path(os.environ["AVIATOR_STEP_MODEL"]).resolve())]
        sim = subprocess.Popen(sim_command,
                               cwd=tmp, env=sim_env, stdout=sim_out, stderr=subprocess.STDOUT)
        try:
            for _ in range(200):
                if (Path("/dev/shm") / f"aviator{bus}").exists():
                    break
                if sim.poll() is not None:
                    raise RuntimeError("Simulator exited during startup")
                time.sleep(.05)
            else:
                raise RuntimeError("Simulator did not start")
            command = [probe, config, str(bus), str(feedback)]
            if lut:
                command.append(lut)
            if wheel_step:
                command.append(wheel_step)
            result = subprocess.run(command,
                                    cwd=tmp, env=probe_env, stdout=probe_out,
                                    stderr=subprocess.STDOUT, timeout=40)
            if result.returncode:
                raise RuntimeError(probe_log.read_text())
            sim.wait(timeout=30)
            if sim.returncode:
                raise RuntimeError(sim_log.read_text())
            with feedback.open() as f:
                fb = list(csv.DictReader(f))
            with send_trace.open() as f:
                sent = list(csv.DictReader(f))
            with sim_trace.open() as f:
                physics = list(csv.DictReader(f))
            base = float(sent[0]["q0_target"])
            step = next(x for x in sent if abs(float(x["q0_target"]) - base) > 1e-6)
            t_send = float(step["monotonic_time"])
            commanded_target = float(step["q0_target"])
            direction = 1 if commanded_target > base else -1
            requested_change = abs(commanded_target - base)
            received = next(x for x in physics if float(x["monotonic_time"]) >= t_send - .005
                            and direction * (float(x["target"]) - base) >=
                                .5 * requested_change)
            t_receive = float(received["monotonic_time"])
            target = float(received["target"])
            q_start = float(received["q_before"])
            response = [x for x in physics if t_receive <= float(x["monotonic_time"]) <= t_receive + .5]
            t90 = next((float(x["monotonic_time"]) - t_receive for x in response
                        if direction * (float(x["q_after"]) - q_start) >=
                            .9 * abs(target - q_start)), None)
            settled_5 = next((float(x["monotonic_time"]) - t_receive
                              for i, x in enumerate(response)
                              if all(abs(float(y["q_after"]) - target) <=
                                     .05 * abs(target - q_start) for y in response[i:])), None)
            q10 = min(response, key=lambda x: abs(float(x["monotonic_time"]) - (t_receive + .01)))
            wheel_before = [x for x in fb if float(x["callback_monotonic_time"]) < t_send]
            wheel_base = float(wheel_before[-1]["theta"])
            wheel_response = [x for x in fb if t_send <= float(x["callback_monotonic_time"]) <= t_send + .5]
            wheel_target = wheel_base + float(wheel_step) if wheel_step else None
            if wheel_target is not None:
                wheel10 = min(wheel_response, key=lambda x:
                              abs(float(x["callback_monotonic_time"]) - (t_send + .01)))
                wheel_settled_5 = next(
                    (float(x["callback_monotonic_time"]) - t_send
                     for i, x in enumerate(wheel_response)
                     if all(abs(float(y["theta"]) - wheel_target) <= .05 * abs(float(wheel_step))
                            for y in wheel_response[i:])), None)
            window = [x for x in physics if t_send - .03 <= float(x["monotonic_time"]) <= t_receive + .5]
            trace_out = output / "joint_step_sim_1ms.csv"
            with trace_out.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=physics[0].keys())
                writer.writeheader()
                writer.writerows(window)
            send_out = output / "joint_step_send.csv"
            send_out.write_text(send_trace.read_text())
            print(next((line for line in probe_log.read_text().splitlines()
                        if line.startswith("step_probe_completed")), "step_probe_completed"))
            print(f"send_monotonic={t_send:.6f} receive_monotonic={t_receive:.6f} "
                  f"delivery_delay_ms={(t_receive-t_send)*1000:.3f}")
            print(f"q_start={q_start:.8f} target={target:.8f} "
                  f"q_at_10ms={float(q10['q_after']):.8f} "
                  f"remaining_10ms_rad={target-float(q10['q_after']):.8f} "
                  f"t90_ms={None if t90 is None else round(t90*1000, 3)} "
                  f"settled_5pct_ms={None if settled_5 is None else round(settled_5*1000, 3)}")
            if wheel_target is not None:
                print(f"wheel_start={wheel_base:.8f} target={wheel_target:.8f} "
                      f"wheel_at_10ms={float(wheel10['theta']):.8f} "
                      f"wheel_10ms_completion={(float(wheel10['theta'])-wheel_base)/float(wheel_step):.3f} "
                      f"wheel_settled_5pct_ms={None if wheel_settled_5 is None else round(wheel_settled_5*1000, 3)}")
            print(f"csv={feedback} {send_out} {trace_out}")
        except Exception:
            sim_out.flush()
            probe_out.flush()
            print(sim_log.read_text())
            print(probe_log.read_text())
            raise
        finally:
            if sim.poll() is None:
                sim.terminate()
                try:
                    sim.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    sim.kill()
                    sim.wait()
            for name in [f"aviator{bus}", f"ecm{bus}", f"pd_input{bus}",
                         f"pd_output{bus}", *[f"sem.sync{bus}_{i}" for i in range(10)]]:
                (Path("/dev/shm") / name).unlink(missing_ok=True)
