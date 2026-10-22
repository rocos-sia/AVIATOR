"""Real UI controller + gateway + Managed Core + MuJoCo, on isolated ports."""
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml


def main():
    bus, simulation, core, gateway, root = sys.argv[1:6]
    root = Path(root)
    spec = importlib.util.spec_from_file_location("wheel_ui", root / "tools/wheel_accuracy/ui.py")
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    # UI validation includes NaN/Inf, both units, exact backend boundary and zero.
    for a, d in (("nan", 0), (0, "inf"), (50, 0), (0, -170.1), (0, .1)):
        try:
            ui.target(a, d)
            raise AssertionError(f"Accepted invalid input {a}, {d}")
        except ValueError:
            pass
    assert ui.target(0, 0) == dict(angle=0., displacement=0.)

    directory = Path(tempfile.mkdtemp(prefix="wheel-accuracy-test-"))
    print(f"Wheel accuracy test logs: {directory}", flush=True)
    reservations = []
    for _ in range(4):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        reservations.append(sock)
    endpoints = [f"tcp://127.0.0.1:{s.getsockname()[1]}" for s in reservations]
    for sock in reservations:
        sock.close()
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    config["photoelectric_emergency"] = {"enabled": False}
    config.update(robot="robot.yaml", bus=dict(publish=endpoints[0], subscribe=endpoints[1]),
                  manipulator_service=endpoints[2])
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    for key in ("model", "urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    robot["viewer"] = False
    flight = yaml.safe_load((root / "config/flight.yaml").read_text())
    flight.update(publish=endpoints[0], subscribe=endpoints[1], service=endpoints[3],
                  lock_file=str(directory / "gateway.lock"), service_timeout_ms=1000)
    for name, value in (("system", config), ("robot", robot), ("flight", flight)):
        (directory / f"{name}.yaml").write_text(yaml.safe_dump(value))
    children, logs = [], []
    controller = ui.Controller(str(directory / "ui.sock"))

    def start(name, command):
        log = (directory / f"{name}.log").open("w")
        logs.append(log)
        children.append(subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                         env={**os.environ, "SDL_VIDEODRIVER": "invalid-no-sdl-in-test-mode"}))

    def wait(predicate, timeout=40, allow_error=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            controller.tick()
            assert all(p.poll() is None for p in children), "Child exited; see logs"
            if not allow_error:
                assert controller.phase not in ("ERROR", "EMERGENCY_STOP"), controller.state
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError(f"Timed out: {controller.message}; {controller.status}")

    def operation(name, phase):
        wait(lambda: controller.ready and controller.pending is None)
        controller.operation(name)
        wait(lambda: controller.pending is None and controller.phase == phase, timeout=80)
        assert controller.message.startswith("操作完成"), controller.message
        print(f"PASS {name} -> {phase}", flush=True)

    def raw_target(point, expected):
        identity = controller.command("target", **point)
        wait(lambda: (controller.status.get("result") or {}).get("id") == identity)
        assert controller.status["result"]["status"] == expected, controller.status["result"]

    try:
        start("bus", [bus, "--input", endpoints[0], "--output", endpoints[1],
                      "--lock-file", str(directory / "bus.lock")])
        start("simulation", [simulation, "--config", str(directory / "system.yaml"), "--headless", "--no-camera"])
        start("gateway", [gateway, "--config", str(directory / "flight.yaml"),
                          "--test-ui-socket", controller.path])
        start("core", [core, "--config", str(directory / "system.yaml"), "--operation-service", endpoints[3]])
        wait(lambda: controller.ready and controller.phase == "READY", timeout=60)
        raw_target(ui.target(0, 0), "REJECTED")
        operation("enter_standby", "STANDBY")
        operation("grasp_wheel", "FOLLOWING")
        for index, (angle, displacement) in enumerate(ui.DEFAULT_POINTS, 1):
            point = ui.target(angle, displacement)
            controller.move(point, f"P{index}")
            wait(lambda: controller.pending is None, timeout=65)
            assert "指令参考已到点" in controller.message, controller.message
            assert controller.phase == "CONTROL"
            reference = controller.state["wheel_reference"]
            assert abs(reference["angle"] - point["angle"]) < math.radians(.05)
            assert abs(reference["displacement"] - point["displacement"]) < .0001
            print(f"PASS P{index} {angle} deg / {displacement} mm, holding", flush=True)
        before = controller.status["target"].copy()
        raw_target(dict(angle=1., displacement=-.085), "REJECTED")
        assert controller.status["target"] == before
        raw_target(dict(angle=0., displacement=.01), "REJECTED")
        assert controller.status["target"] == before
        # An old/replayed target cannot renew the input lease or move the robot.
        stale = dict(kind="target", id="stale", sample_us=time.monotonic_ns() // 1000 - 200000,
                     angle=0., displacement=0.)
        controller.sock.send(json.dumps(stale).encode())
        wait(lambda: (controller.status.get("result") or {}).get("id") == "stale")
        assert controller.status["result"]["status"] == "REJECTED"
        assert controller.status["target"] == before
        operation("exit_control", "FOLLOWING")
        # Reenter CONTROL with an edited point, then release directly from CONTROL.
        controller.move(ui.target(7, -90), "edited")
        wait(lambda: controller.pending is None, timeout=65)
        assert "指令参考已到点" in controller.message
        operation("leave_wheel", "STANDBY")
        operation("grasp_wheel", "FOLLOWING")
        controller.move(ui.target(0, -85), "lease test")
        wait(lambda: controller.pending is None, timeout=65)
        assert "指令参考已到点" in controller.message
        # Keep the socket open but freeze the UI event loop: no synthetic keepalive.
        time.sleep(.7)
        wait(lambda: controller.phase == "ERROR", allow_error=True)
        print("PASS frozen UI expires CONTROL; no automatic motion restart", flush=True)
    finally:
        controller.close()
        # Let Core release/disable while the device and bus are still alive.
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
