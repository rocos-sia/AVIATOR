"""Isolated MuJoCo regression of the managed Aviator API; no deployment endpoints/hardware."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml
import zmq

from fake_rh56ftp_hand import FakeRH56FTPHand


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    bus, manipulator, core, root = sys.argv[1:5]
    hand_timeout = '--hand-timeout' in sys.argv[5:]
    latched_error = '--hand-latched-error' in sys.argv[5:]
    hand_fault = (4 if latched_error or '--hand-overcurrent' in sys.argv[5:] else
                  2 if '--hand-overtemperature' in sys.argv[5:] else 0)
    assert not (hand_fault and hand_timeout), 'hand fault and message timeout are separate scenarios'
    root = Path(root)
    directory = Path(tempfile.mkdtemp(prefix="aviator-fsm-test-"))
    print(f"FSM process logs: {directory}", flush=True)
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    if hand_timeout:
        config['core_hand'].update(enabled=True, publisher_id='unavailable_test_hand', completion_timeout_ms=500)
    if hand_fault:
        # Require actual simulated opening, rather than the deployment's permissive tolerance.
        config['core_hand'].update(enabled=True, publisher_id='rh56ftp_hand', open_tolerance=.03)
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    robot["viewer"] = False
    robot["settle_duration"] = .15
    for key in ("model", "urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    endpoints = [f"tcp://127.0.0.1:{port()}" for _ in range(3)]
    assert len(set(endpoints)) == 3
    config["bus"] = dict(publish=endpoints[0], subscribe=endpoints[1])
    config["manipulator_service"] = endpoints[2]
    config["robot"] = "robot.yaml"
    (directory / "system.yaml").write_text(yaml.safe_dump(config))
    device_config = dict(config)
    if hand_timeout or hand_fault:
        # Simulation uses its configured hand publisher; use a distinct identity
        # so Core really receives no matching hand feedback during the full cycle.
        device_config['core_hand'] = {**config['core_hand'], 'publisher_id': 'simulated_test_hand'}
    (directory / "device-system.yaml").write_text(yaml.safe_dump(device_config))
    (directory / "robot.yaml").write_text(yaml.safe_dump(robot))
    children, logs = [], []
    context = zmq.Context()
    hand = FakeRH56FTPHand(context, endpoints, config['core_hand'], directory, latched_error) if hand_fault else None
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
            if hand:
                hand.check()
            if sub.poll(100):
                message = json.loads(sub.recv_multipart()[1])
                state = message.get("system", {})
                if state.get("state") == expected and (expected != "FOLLOWING" or state.get("settled")):
                    return state
                if state.get("state") in ("ERROR", "EMERGENCY_STOP"):
                    raise AssertionError(f"Expected {expected}, got {state}")
                if hand and state.get('state') == 'SAFE':
                    raise AssertionError(f'Hand fault unexpectedly triggered SAFE: {state}')
            if process.poll() is not None:
                raise AssertionError("Child process exited; inspect logs")
        raise AssertionError(f"Timed out waiting for {expected}")

    def command(line):
        process.stdin.write(line + "\n")
        process.stdin.flush()

    try:
        start("bus", [bus, "--input", endpoints[0], "--output", endpoints[1],
                      "--lock-file", str(directory / "bus.lock")])
        start("manipulator", [manipulator, "--config", str(directory / "device-system.yaml"), "--headless", "--no-camera"])
        if hand:
            hand.start()
        arguments = [core, "--config", str(directory / "system.yaml"), "--console"]
        process = start("core", arguments)
        if hand_timeout:
            state = wait_state("ERROR", 10)
            assert 'hand.state message timeout' in state['current_error'], state
            command("RESET_ERROR")
            time.sleep(.2)
            assert 'hand.state message timeout' in wait_state("ERROR", 3)['current_error']
            command("quit")
            assert process.wait(timeout=10) == 0
            assert '[error]' in (directory / 'core.log').read_text()
            print("PASS: missing hand messages cause managed FSM ERROR", flush=True)
            return
        wait_state("READY", 30)  # Init/enable completed; no automatic home.
        command("GRASP_WHEEL")
        time.sleep(.1)
        wait_state("READY", 3)
        command("ENTER_STANDBY")
        wait_state("HOMING", 3)
        wait_state("STANDBY", 120)
        command("START_CONTROL")
        time.sleep(.1)
        command("GRASP_WHEEL")
        wait_state("GRASPING")
        wait_state("FOLLOWING", 120)
        command("START_CONTROL")
        wait_state("CONTROL", 3)
        # Target calls go through Aviator::ServoWheel, not through the console's own FSM.
        for _ in range(25):
            command("servo 0.01 -0.001 0.5")
            time.sleep(.02)
        wait_state("CONTROL", 3)
        command("EXIT_CONTROL")
        wait_state("FOLLOWING", 3)
        deadline = time.monotonic() + 15
        while not wait_state("FOLLOWING", 3).get("settled"):
            assert time.monotonic() < deadline, "executor did not acknowledge normal stop"
        if hand:
            hand.inject(hand_fault)
        command("LEAVE_WHEEL")
        wait_state("RELEASING", 3)
        wait_state("STANDBY", 30)
        if hand:
            # Let the independent hand peer observe the final flight.state too.
            time.sleep(.1)
            hand.verify_release()
        command("emergency")
        state = wait_state("EMERGENCY_STOP", 3)
        assert state["brake_requested"] and not state["brake_confirmed"]
        command("RESET_ERROR\nENTER_STANDBY\nGRASP_WHEEL")
        time.sleep(.2)
        wait_state("EMERGENCY_STOP", 3)
        command("quit")
        assert process.wait(timeout=10) == 0
        assert not list(directory.glob("*.emergency"))
        assert not list(directory.glob("*.brake-request"))
        print("PASS: file-free FSM cycle, input gate, release and process-local emergency latch", flush=True)
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
        if hand and hand.thread.ident is not None:
            hand.finish()
        sub.close(0)
        context.term()


if __name__ == "__main__":
    main()
