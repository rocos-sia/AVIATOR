"""FOLLOWING holds the rebased impedance target despite camera/joystick input."""
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

import yaml
import zmq
from robot_state_machine_process_test import port


def main():
    bus, simulation, core, root = sys.argv[1:5]
    root = Path(root)
    folder = Path(tempfile.mkdtemp(prefix="aviator-following-"))
    print(f"Following process logs: {folder}", flush=True)
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    robot.update(viewer=False, settle_duration=.15)
    # Stable lifecycle fixture, independent of later deployment stiffness tuning.
    robot["rokae"]["joint_stiffness"] = [500, 500, 500, 500, 50, 50, 50]
    robot["rokae"]["following_joint_stiffness"] = [500, 500, 500, 500, 50, 50, .1]
    for key in ("model", "urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    endpoints = [f"tcp://127.0.0.1:{port()}" for _ in range(4)]
    assert len(set(endpoints)) == 4
    config.update(robot="robot.yaml", bus=dict(publish=endpoints[0], subscribe=endpoints[1]),
                  manipulator_service=endpoints[2])
    (folder / "system.yaml").write_text(yaml.safe_dump(config))
    (folder / "robot.yaml").write_text(yaml.safe_dump(robot))
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.subscribe(b"flight.state")
    sub.connect(endpoints[1])
    arm_sub = context.socket(zmq.SUB)
    arm_sub.subscribe(b"arm.state")
    arm_sub.connect(endpoints[1])
    dealer = context.socket(zmq.DEALER)
    dealer.connect(endpoints[3])
    clock = socket.gethostname() + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    quit_publish = threading.Event()
    camera_on = threading.Event()
    flight_target = (0., 0.)
    camera_samples = 0
    session = str(uuid.uuid4())

    def publish():
        nonlocal camera_samples
        pub = context.socket(zmq.PUB)
        pub.connect(endpoints[0])
        sequence = 0
        while not quit_publish.wait(.02):
            sequence += 1
            now = time.monotonic_ns() // 1000
            envelope = dict(version="1.0", sequence=sequence, timestamp=time.time_ns() // 1000,
                            sample_mono_us=now, clock_id=clock, session_id=session, valid=True)
            roll, pitch = flight_target
            flight = dict(envelope, msg_type="FlightCommand", publisher_id="flight_gateway",
                          source="JOYSTICK", control=dict(roll=roll, pitch=pitch),
                          input_state=dict(mode="POSITION_HOLD", device_connected=True, checked_mono_us=now))
            pub.send_multipart([b"flight.command", json.dumps(flight).encode()])
            if camera_on.is_set():
                # Fresh, valid targets change throughout both hold phases. None may
                # reach ServoWheel, even after CONTROL has moved away from the grasp pose.
                sign = 1 if (sequence // 4) % 2 else -1
                detection = dict(envelope, msg_type="CameraDetection", publisher_id="camera", camera_id="cockpit",
                                 steering_wheel=dict(valid=True, theta_rad=sign * .25,
                                                     translation_along_axis_m=sign * .03,
                                                     axis_match=True))
                pub.send_multipart([b"camera.detection", json.dumps(detection).encode()])
                camera_samples += 1
        pub.close(0)

    children, logs = [], []
    publisher = threading.Thread(target=publish)
    publisher.start()

    def start(name, command):
        log = (folder / f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=log, stderr=log,
                                   env={**os.environ, "LD_BIND_NOW": "1"})
        children.append(process)
        return process

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            if sub.poll(50):
                last = json.loads(sub.recv_multipart()[1])
                assert last["system"]["state"] != "ERROR", last["system"]
                if predicate(last):
                    return last
            assert all(child.poll() is None for child in children), "process exited; inspect logs"
        raise AssertionError(f"Timeout; last={last}")

    def state(name, timeout=15):
        return wait(lambda m: m["system"]["state"] == name and
                    (name != "FOLLOWING" or m["system"]["settled"]) and
                    (name != "CONTROL" or m["system"]["control_source"] == "JOYSTICK"), timeout)

    def operation(name):
        request = dict(msg_type="ServiceRequest", version="1.0", request_id=str(uuid.uuid4()),
                       client_id="flight_gateway", client_session_id=session, target="aviator_core",
                       operation=name, timestamp=time.time_ns() // 1000, issued_mono_us=time.monotonic_ns() // 1000,
                       clock_id=clock, deadline_ms=1000, parameters=dict(source="JOYSTICK", button=1))
        dealer.send_json(request)
        assert dealer.poll(2000), name
        reply = dealer.recv_json()
        assert reply["status"] in ("ACCEPTED", "COMPLETED"), reply

    def arm_after(sample):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if arm_sub.poll(50):
                message = json.loads(arm_sub.recv_multipart()[1])
                if message["sample_mono_us"] >= sample:
                    assert not message["execution"]["fault"], message
                    assert all(message["arms"][side]["enabled"] for side in ("left", "right")), message
                    return message
        raise AssertionError("No fresh arm command snapshot")

    def held(message, duration=1.):
        assert message["system"]["settled"], message
        reference = message["wheel_reference"]
        arm = arm_after(message["sample_mono_us"])
        assert arm["execution"]["impedance_profile"] == "following", arm
        assert not arm["execution"]["impedance_switching"], arm
        target = arm["execution"]["target"]
        assert arm["wheel_reference"] == reference, arm
        deadline = time.monotonic() + duration
        samples = 0
        camera_samples_before = camera_samples
        while time.monotonic() < deadline:
            message = wait(lambda m: True, 2)
            assert message["system"]["state"] == "FOLLOWING", message["system"]
            assert message["system"]["settled"] and message["system"]["task_phase"] == "LOCKED", message
            assert message["system"]["control_source"] == "NONE", message
            assert message["wheel_reference"] == reference, (message["wheel_reference"], reference)
            arm = arm_after(message["sample_mono_us"])
            assert arm["execution"]["target"] == target, (arm["execution"]["target"], target)
            assert arm["wheel_reference"] == reference, arm
            samples += 1
        assert samples >= 10, "Too few samples to verify continuous hold"
        assert camera_samples - camera_samples_before >= 10, "Camera publisher did not exercise hold"
        return target, reference

    try:
        start("bus", [bus, "--input", endpoints[0], "--output", endpoints[1], "--lock-file", str(folder / "bus.lock")])
        start("simulation", [simulation, "--config", str(folder / "system.yaml"), "--headless", "--no-camera"])
        start("core", [core, "--config", str(folder / "system.yaml"), "--operation-service", endpoints[3]])
        state("READY", 30)
        wait(lambda m: m["system"].get("input_ready"))
        operation("enter_standby"); state("STANDBY", 120)
        assert "to=STIFFNESS" not in (folder / "core.log").read_text(), "Init/home changed stiffness"
        operation("grasp_wheel")
        grasped = state("FOLLOWING", 120)
        assert grasped["wheel_reference"] == robot["wheel_initial"], grasped
        camera_on.set()
        grasp_target, grasp_reference = held(grasped)
        flight_target = (-.2, .06)
        operation("start_control")
        controlled = state("CONTROL")
        assert arm_after(controlled["sample_mono_us"])["execution"]["impedance_profile"] == "default"
        wait(lambda m: m["system"]["state"] == "CONTROL" and
             m["system"]["control_source"] == "JOYSTICK" and
             .02 < m["wheel_reference"]["angle"] < .15 and
             max(abs(v) for side in ("left", "right")
                 for v in m["arms"][side]["joint_velocity"]) > .1)
        # Exit while moving: decelerate, then rebase the hold to measured joints.
        # Do not replay the last displayed CONTROL sample or original grasp target.
        operation("exit_control")
        stopped = state("FOLLOWING")
        flight_target = (.2, -.06)  # Neither input source may move the held target now.
        stopped_target, stopped_reference = held(stopped)
        assert abs(stopped_reference["angle"] - grasp_reference["angle"]) > .02, stopped_reference
        assert max(abs(a - b) for a, b in zip(stopped_target, grasp_target)) > .001, stopped_target
        operation("leave_wheel")
        released = state("STANDBY", 120)
        assert arm_after(released["sample_mono_us"])["execution"]["impedance_profile"] == "default"
        for name in ("core", "simulation"):
            output = (folder / f"{name}.log").read_text()
            if name == "core":
                assert output.count("to=STIFFNESS") == 4, output
            else:
                assert output.count("Joint stiffness update profile=following") == 2, output
                assert output.count("Joint stiffness update profile=default") == 2, output
        print("PASS: FOLLOWING holds rebased targets, ignores camera, switches following/default stiffness and returns to STANDBY", flush=True)
    finally:
        quit_publish.set(); publisher.join()
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired: child.kill(); child.wait()
        for log in logs: log.close()
        sub.close(0); arm_sub.close(0); dealer.close(0); context.term()


if __name__ == "__main__":
    main()
