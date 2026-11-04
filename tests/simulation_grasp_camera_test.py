"""Exercise device discovery and Core grasp acquisition using only MuJoCo on private ports."""
from contextlib import ExitStack
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml

driver, bus, simulation, root = map(lambda p: Path(p).resolve(), sys.argv[1:])
with tempfile.TemporaryDirectory(prefix="aviator-sim-grasp-camera-") as tmp, ExitStack() as stack:
    directory = Path(tmp)
    reservations = [stack.enter_context(socket.socket()) for _ in range(3)]
    for sock in reservations:
        sock.bind(("127.0.0.1", 0))
    endpoints = [f"tcp://127.0.0.1:{sock.getsockname()[1]}" for sock in reservations]
    system = yaml.safe_load((root / "config/system-simulation-rs422.yaml").read_text())
    system.update(robot=str(directory / "robot.yaml"), manipulator_service=endpoints[2])
    system["bus"] = dict(publish=endpoints[0], subscribe=endpoints[1])
    system["core_hand"]["enabled"] = False
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    for key in ("model", "urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    robot["grasp_wheel"]["source"] = "camera"
    robot["wheel_initial"] = dict(angle=.12, displacement=-.06)
    robot["viewer"] = False
    camera = yaml.safe_load((root / "config/camera.yaml").read_text())
    camera["preview"]["enabled"] = False
    for name, value in (("system", system), ("robot", robot), ("camera", camera)):
        (directory / f"{name}.yaml").write_text(yaml.safe_dump(value))
    for sock in reservations:
        sock.close()
    processes = []
    logs = []
    try:
        commands = [
            [str(bus), "--input", endpoints[0], "--output", endpoints[1],
             "--lock-file", str(directory / "bus.lock")],
            [str(simulation), "--config", str(directory / "system.yaml"), "--headless",
             "--camera-config", str(directory / "camera.yaml")],
        ]
        for i, command in enumerate(commands):
            log = stack.enter_context((directory / f"process-{i}.log").open("w+"))
            logs.append(log)
            processes.append(subprocess.Popen(command, cwd=directory, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 20
        while True:
            logs[1].seek(0)
            if "READY simulation" in logs[1].read():
                break
            assert all(p.poll() is None for p in processes), "Simulation or bus exited"
            assert time.monotonic() < deadline, "Simulation startup timed out"
            time.sleep(.05)
        result = subprocess.run([str(driver), str(directory / "system.yaml")],
                                capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "publisher=simulation" in result.stdout, result.stdout
        print(result.stdout)
    except Exception:
        for log in logs:
            log.seek(0)
            print(log.read(), file=sys.stderr)
        raise
    finally:
        for process in reversed(processes):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
