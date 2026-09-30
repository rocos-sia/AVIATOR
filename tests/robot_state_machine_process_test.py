"""Isolated MuJoCo process regression; never uses deployment endpoints or real hardware."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

import yaml
import zmq


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    bus, manipulator, core, root = sys.argv[1:]
    root = Path(root)
    directory = Path(tempfile.mkdtemp(prefix="aviator-fsm-test-"))
    print(f"FSM process logs: {directory}", flush=True)
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    robot["backend"] = "mujoco"
    robot["viewer"] = False
    robot["settle_duration"] = .15
    for key in ("model", "urdf", "collision_urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    endpoints = [f"tcp://127.0.0.1:{port()}" for _ in range(3)]
    assert len(set(endpoints)) == 3
    config["bus"] = dict(publish=endpoints[0], subscribe=endpoints[1])
    config["manipulator_service"] = endpoints[2]
    config["robot"] = "robot.yaml"
    (directory / "system.yaml").write_text(yaml.safe_dump(config))
    (directory / "robot.yaml").write_text(yaml.safe_dump(robot))
    children, logs = [], []
    safety = directory / "safety.json"
    stop_evidence = threading.Event()
    clear_of_wheel = threading.Event()

    def refresh_evidence():
        while not stop_evidence.is_set():
            data = dict(clock_id=socket.gethostname() + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                        sample_mono_us=time.monotonic_ns() // 1000, emergency_latched=False,
                        clear_of_wheel=clear_of_wheel.is_set(), following_authorized=True, release_authorized=True,
                        fault_cleared=True, source_authorized=True, input_ready=True)
            temporary = safety.with_suffix(".tmp")
            temporary.write_text(json.dumps(data))
            temporary.replace(safety)
            stop_evidence.wait(.01)
    evidence_thread = threading.Thread(target=refresh_evidence)
    evidence_thread.start()
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.subscribe(b"flight.state")
    sub.connect(endpoints[1])

    def start(name, args):
        log = (directory / (name + ".log")).open("w")
        logs.append(log)
        child = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=log, stderr=log,
                                 text=True, env={**os.environ, "LD_BIND_NOW": "1"})
        children.append(child)
        return child

    def wait_state(expected, timeout=100):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if sub.poll(100):
                message = json.loads(sub.recv_multipart()[1])
                state = message.get("system", {})
                if state.get("state") == expected:
                    return state
                if state.get("state") in ("ERROR", "EMERGENCY_STOP"):
                    raise AssertionError(f"Expected {expected}, got {state}")
            if process.poll() is not None:
                raise AssertionError("Child process exited; inspect logs")
        raise AssertionError(f"Timed out waiting for {expected}")

    def command(line):
        process.stdin.write(line + "\n")
        process.stdin.flush()

    try:
        start("bus", [bus, "--input", endpoints[0], "--output", endpoints[1],
                      "--lock-file", str(directory / "bus.lock")])
        start("manipulator", [manipulator, "--config", str(directory / "system.yaml"), "--headless"])
        deadline = time.monotonic() + 2
        while not safety.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        arguments = [core, "--config", str(directory / "system.yaml"), "--state-machine", "--safety-file", str(safety)]
        process = start("core", arguments)
        wait_state("ERROR", 30)  # Connected is not enough: no initial clearance evidence.
        command("RESET_ERROR")
        state = wait_state("SAFE", 3)
        assert state["current_error_code"] == 0 and state["last_error_code"] != 0
        command("ENTER_STANDBY")
        time.sleep(.1)
        clear_of_wheel.set()
        time.sleep(.05)
        command("ENTER_STANDBY")
        wait_state("STANDBY", 3)
        command("START_CONTROL")
        time.sleep(.1)
        command("GRASP_WHEEL")
        wait_state("GRASPING")
        wait_state("FOLLOWING", 120)
        command("START_CONTROL")
        wait_state("CONTROL", 3)
        command("EXIT_CONTROL")
        wait_state("FOLLOWING", 3)
        time.sleep(.2)
        command("LEAVE_WHEEL")
        wait_state("RELEASING", 3)
        wait_state("STANDBY", 30)
        command("emergency")
        state = wait_state("EMERGENCY_STOP", 3)
        assert state["brake_requested"] and not state["brake_confirmed"]
        command("RESET_ERROR\nENTER_STANDBY\nGRASP_WHEEL")
        time.sleep(.2)
        wait_state("EMERGENCY_STOP", 3)
        command("quit")
        assert process.wait(timeout=10) == 0
        assert Path(str(safety) + ".emergency").exists()
        assert Path(str(safety) + ".brake-request").exists()
        # Drain the old publisher before checking a new process starts latched.
        while sub.poll(0):
            sub.recv_multipart()
        process = start("core-restarted", arguments)
        wait_state("EMERGENCY_STOP", 15)
        command("quit")
        assert process.wait(timeout=10) == 0
        print("PASS: real bus/executor FSM cycle, input gate, release and emergency latch", flush=True)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        for log in logs:
            log.close()
        stop_evidence.set()
        evidence_thread.join()
        sub.close(0)
        context.term()


if __name__ == "__main__":
    main()
