"""Managed Core + Gateway wire protocol + real MuJoCo executor, isolated from deployment."""
import copy
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import urllib.request

import yaml
import zmq
from robot_state_machine_process_test import port


def main():
    bus, manipulator, core, root = sys.argv[1:5]
    monitor = sys.argv[5] if len(sys.argv) > 5 else None
    root = Path(root)
    directory = Path(tempfile.mkdtemp(prefix="aviator-managed-gateway-"))
    print(f"Gateway process logs: {directory}", flush=True)
    import math
    posture = json.loads((root / "config/posture.json").read_text())
    home = [math.radians(x) for x in posture["left_home_deg"] + posture["right_home_deg"]]
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    robot.update(viewer=False, settle_duration=.15)
    for key in ("model", "urdf", "collision_urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    endpoints = [f"tcp://127.0.0.1:{port()}" for _ in range(4)]
    assert len(set(endpoints)) == 4
    config.update(bus=dict(publish=endpoints[0], subscribe=endpoints[1]),
                  manipulator_service=endpoints[2], robot="robot.yaml")
    (directory / "system.yaml").write_text(yaml.safe_dump(config))
    (directory / "robot.yaml").write_text(yaml.safe_dump(robot))
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.subscribe(b"flight.state")
    sub.connect(endpoints[1])
    dealer = context.socket(zmq.DEALER)
    dealer.connect(endpoints[3])
    gateway_session = str(uuid.uuid4())
    core_session = None
    clock = socket.gethostname() + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    stopped = threading.Event()
    transmitting = threading.Event()
    checking = threading.Event()
    checking.set()
    flight_target = (0.0, 0.0)
    children, logs = [], []

    def publish():
        pub = context.socket(zmq.PUB)
        pub.connect(endpoints[0])
        sequence = 0
        # Deliberately keep the original hardware event unchanged across StartControl.
        sampled = checked = time.monotonic_ns() // 1000
        while not stopped.wait(.02):
            if not transmitting.is_set():
                continue
            sequence += 1
            if checking.is_set():
                checked = time.monotonic_ns() // 1000
            roll, pitch = flight_target
            msg = dict(msg_type="FlightCommand", version="1.0", sequence=sequence,
                       timestamp=time.time_ns() // 1000, sample_mono_us=sampled,
                       clock_id=clock, publisher_id="flight_gateway", session_id=gateway_session,
                       valid=True, source="JOYSTICK", control=dict(roll=roll, pitch=pitch),
                       input_state=dict(mode="POSITION_HOLD", device_connected=True,
                                        checked_mono_us=checked))
            pub.send_multipart([b"flight.command", json.dumps(msg).encode()])
        pub.close(0)

    publisher = threading.Thread(target=publish)
    publisher.start()

    def start(name, args):
        log = (directory / (name + ".log")).open("w")
        logs.append(log)
        child = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                 env={**os.environ, "LD_BIND_NOW": "1"})
        children.append(child)
        return child

    def wait_for(predicate, timeout=10):
        nonlocal core_session
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            if sub.poll(50):
                last = json.loads(sub.recv_multipart()[1])
                core_session = last["session_id"]
                assert last["system"]["state"] != "ERROR", last["system"]
                if predicate(last):
                    return last
            assert process.poll() is None, "Managed Core exited; inspect logs"
        raise AssertionError(f"Timed out; last state: {last}")

    def state(name, timeout=10):
        message = wait_for(lambda m: m["system"]["state"] == name, timeout)
        if name == "STANDBY":
            actual = message["arms"]["left"]["joint_position"] + message["arms"]["right"]["joint_position"]
            assert max(abs(a - b) for a, b in zip(actual, home)) <= robot.get("home_position_tolerance", .02), message
        return message

    def request(op):
        return dict(msg_type="ServiceRequest", version="1.0", request_id=str(uuid.uuid4()),
                    client_id="flight_gateway", client_session_id=gateway_session,
                    target="aviator_core", operation=op, timestamp=time.time_ns() // 1000,
                    issued_mono_us=time.monotonic_ns() // 1000, clock_id=clock, deadline_ms=1000,
                    parameters=dict(source="JOYSTICK", button=1, server_session_id=core_session))

    def call(req, expected):
        dealer.send_json(req)
        assert dealer.poll(2000), f"Missing reply: {req}"
        response = dealer.recv_json()
        assert response["request_id"] == req["request_id"] and response["server_session_id"] == core_session
        assert response["status"] == expected, response
        return response

    def check_monitor_control():
        if not monitor:
            return
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        samples = []
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            with http.open(monitor_url + "/api/state", timeout=2) as reply:
                snapshot = json.load(reply)
            with http.open(monitor_url + "/api/overview", timeout=2) as reply:
                overview = json.load(reply)
            samples.append(dict(streams=[r for r in snapshot["streams"]
                                        if r["topic"] in ("arm.state", "hand.state", "arm.command")],
                                arms=overview["arms"], hands=overview["hands"]))
            time.sleep(.02)  # Exercise the browser's 50 Hz overview polling load.
        (directory / "monitor-control.json").write_text(json.dumps(samples, indent=2))
        for topic in ("arm.state", "hand.state"):
            rows = [next(r for r in s["streams"] if r["topic"] == topic) for s in samples]
            assert rows[-1]["sequence"] > rows[0]["sequence"], (topic, rows)
            fresh = sum(r["status"] == "FRESH" for r in rows)
            assert fresh >= len(rows) * .9, (topic, "stale monitor feedback",
                                           "see", directory / "monitor-control.json")
        for group in ("arms", "hands"):
            for side in ("left", "right"):
                fresh = sum(s[group][side]["measurement_state"] == "VALID" for s in samples)
                assert fresh >= len(samples) * .9, (group, side, "see", directory / "monitor-control.json")
        print("PASS: Monitor arm/hand feedback remains fresh during sustained CONTROL", flush=True)

    try:
        start("bus", [bus, "--input", endpoints[0], "--output", endpoints[1],
                      "--lock-file", str(directory / "bus.lock")])
        start("manipulator", [manipulator, "--config", str(directory / "system.yaml"), "--headless", "--no-camera"])
        if monitor:
            monitor_config = yaml.safe_load((root / "config/monitor.yaml").read_text())
            monitor_config["sources"]["hand.state"] = config["core_hand"]["publisher_id"]
            (directory / "monitor.yaml").write_text(yaml.safe_dump(monitor_config))
            monitor_port = port()
            monitor_url = f"http://127.0.0.1:{monitor_port}"
            start("monitor", [monitor, "--bind", "127.0.0.1", "--port", str(monitor_port),
                              "--subscribe", endpoints[1], "--preview", "off",
                              "--config", str(directory / "monitor.yaml")])
        process = start("core", [core, "--config", str(directory / "system.yaml"),
                                 "--operation-service", endpoints[3]])
        initial_state = state("READY", 30)  # Startup enables but waits for external home request, even with stdin EOF.
        assert abs(initial_state["wheel_reference"]["displacement"] + .085) < 1e-9, initial_state
        assert call(request("enter_standby"), "REJECTED")["result"]["reason"] == "GATEWAY_NOT_BOUND"
        transmitting.set()
        # Gateway enables button submission only after fresh Core binding confirmation.
        wait_for(lambda m: m["valid"] and m["system"].get("source_authorized") and
                 m["system"].get("input_ready"))
        call(request("grasp_wheel"), "REJECTED")
        call(request("enter_standby"), "ACCEPTED")
        state("HOMING", 3)
        homed = state("STANDBY", 120)
        assert abs(homed["wheel_reference"]["displacement"] + .085) < 1e-9, homed
        call(request("enter_standby"), "COMPLETED")
        req = request("enter_standby")
        del req["parameters"]["server_session_id"]
        call(req, "COMPLETED")
        req = request("enter_standby")
        req["client_session_id"] = "restarted-gateway"
        req["parameters"]["server_session_id"] = "ignored-old-core"
        call(req, "COMPLETED")
        req["client_session_id"] = "another-text-marker"
        del req["parameters"]["server_session_id"]
        call(req, "COMPLETED")  # Same request ID still deduplicates across labels.
        for field, value in (("clock_id", "foreign"),
                             ("target", "manipulator"), ("operation", "emergency"),
                             ("issued_mono_us", time.monotonic_ns() // 1000 + 10000000)):
            req = request("grasp_wheel")
            req[field] = value
            call(req, "REJECTED")
        for key, value in (("source", None), ("button", "1"), ("button", 12)):
            req = request("grasp_wheel")
            req["parameters"][key] = value
            call(req, "REJECTED")
        req = request("grasp_wheel")
        req["issued_mono_us"] -= 2000000
        call(req, "EXPIRED")
        req = request("grasp_wheel")
        req["gateway_observation"] = "QUEUED"
        call(req, "REJECTED")  # Recording copies are never executable.
        dealer.send_multipart([b"", json.dumps(request("grasp_wheel")).encode()])
        call(request("start_control"), "REJECTED")  # Malformed multipart did not run an action.
        grasp = request("grasp_wheel")
        accepted = call(grasp, "ACCEPTED")
        assert call(grasp, "ACCEPTED") == accepted
        req = copy.deepcopy(grasp)
        req["operation"] = "leave_wheel"
        assert call(req, "REJECTED")["result"]["reason"] == "REQUEST_ID_CONFLICT"
        call(request("grasp_wheel"), "REJECTED")  # BUSY during the original task.
        grasped = state("FOLLOWING", 120)
        assert abs(grasped["wheel_reference"]["displacement"] + .085) < 1e-9, grasped
        assert call(grasp, "ACCEPTED") == accepted  # Cached even beyond the original deadline.
        transmitting.clear()
        time.sleep(.15)
        call(request("start_control"), "REJECTED")  # Input guard required at entry.
        transmitting.set()
        time.sleep(.15)
        call(request("start_control"), "COMPLETED")
        state("CONTROL")
        wait_for(lambda m: m["system"]["state"] == "CONTROL" and
                 m["system"]["control_source"] == "JOYSTICK" and
                 abs(m["wheel_reference"]["displacement"] + .085) < .0004, 10)
        flight_target = (.04, -.01)
        # A static hardware sample must reach the target using only fresh device checks.
        wait_for(lambda m: m["system"]["state"] == "CONTROL" and
                 m["system"]["control_source"] == "JOYSTICK" and
                 abs(m["wheel_reference"]["angle"] + .04 * .87266) < .002 and
                 abs(m["wheel_reference"]["displacement"] + .08585) < .0004, 20)
        check_monitor_control()
        call(request("leave_wheel"), "REJECTED")
        call(request("exit_control"), "COMPLETED")
        wait_for(lambda m: m["system"]["state"] == "FOLLOWING" and m["system"].get("settled"), 15)
        call(request("leave_wheel"), "ACCEPTED")
        state("STANDBY", 120)
        print("PASS: six-operation service, validation/dedup, static input motion, normal release", flush=True)
        call(request("grasp_wheel"), "ACCEPTED")
        state("FOLLOWING", 120)
        call(request("start_control"), "COMPLETED")
        control_start = state("CONTROL")["wheel_reference"]["angle"]
        flight_target = (.3, -.1)
        wait_for(lambda m: m["system"]["state"] == "CONTROL" and
                 abs(m["wheel_reference"]["angle"] - control_start) > .005 and
                 max(abs(v) for side in ("left", "right")
                     for v in m["arms"][side]["joint_velocity"]) > .1, 10)
        checking.clear()  # Keep increasing message sequence but freeze the device evidence.
        state("SAFE", 5)
        stopped_state = wait_for(lambda m: m["system"]["state"] == "SAFE" and
                                m["system"].get("settled"), 5)
        # Stay beyond the old one-second enable/stop deadline. Inspect every update,
        # including after fresh input returns: SAFE must hold without auto-resume.
        checking.set()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            message = wait_for(lambda m: True, 1)
            assert message["system"]["state"] == "SAFE", message["system"]
            assert message["system"]["settled"], message["system"]
            assert message["wheel_reference"] == stopped_state["wheel_reference"]
            assert all(message["arms"][side]["enabled"] for side in ("left", "right"))
        call(request("start_control"), "REJECTED")
        # Sixth request reaches the FSM, with the existing recovery guard result left authoritative.
        req = request("reset_error")
        dealer.send_json(req)
        assert dealer.poll(2000)
        response = dealer.recv_json()
        assert response["request_id"] == req["request_id"]
        assert response["status"] in ("COMPLETED", "REJECTED"), response
        call(request("leave_wheel"), "ACCEPTED")
        state("STANDBY", 120)
        print("PASS: moving SAFE stop, stable enabled hold beyond 1 s, no auto-resume, explicit release", flush=True)
    finally:
        stopped.set()
        publisher.join()
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
        sub.close(0)
        dealer.close(0)
        context.term()


if __name__ == "__main__":
    main()
