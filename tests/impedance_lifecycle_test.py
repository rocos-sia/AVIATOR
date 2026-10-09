"""Exercise planned RT feedback gaps, failed restart, cancellation and owner heartbeat loss."""
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import yaml
from robot_state_machine_process_test import port

bus, driver, root = sys.argv[1:]
root = Path(root)
for mode in ("success", "failure", "cancel", "heartbeat", "timeout", "unchanged", "fallback", "latest",
             "latest_impedance", "latest_heartbeat", "latest_cancel", "rebase"):
    folder = Path(tempfile.mkdtemp(prefix=f"aviator-impedance-{mode}-"))
    print(folder, flush=True)
    config = yaml.safe_load((root / "config/system.yaml").read_text())
    robot = yaml.safe_load((root / "config/robot.yaml").read_text())
    for key in ("model", "urdf", "grasp", "posture"):
        robot[key] = str((root / "config" / robot[key]).resolve())
    robot["rokae"]["joint_stiffness"] = [500, 500, 500, 500, 50, 50, 50]
    robot["rokae"]["following_joint_stiffness"] = [500, 500, 500, 500, 50, 50, .1]
    if mode == "unchanged":
        robot["rokae"]["following_joint_stiffness"] = robot["rokae"]["joint_stiffness"]
    if mode == "fallback":
        del robot["rokae"]["following_joint_stiffness"]
    endpoints = [f"tcp://127.0.0.1:{port()}" for _ in range(3)]
    config.update(robot="robot.yaml", bus=dict(publish=endpoints[0], subscribe=endpoints[1]), manipulator_service=endpoints[2])
    # This test exercises arms only, with no physical or simulated hand service.
    config["core_hand"]["enabled"] = False
    (folder / "robot.yaml").write_text(yaml.safe_dump(robot))
    path = folder / "system.yaml"
    path.write_text(yaml.safe_dump(config))
    children = []
    logs = []
    try:
        for name, command in (
            ("bus", [bus, "--input", endpoints[0], "--output", endpoints[1], "--lock-file", str(folder / "bus.lock")]),
            ("server", [driver, "server", str(path), mode]),
        ):
            log = (folder / f"{name}.log").open("w"); logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=log))
        time.sleep(.3)
        subprocess.run([driver, "client", str(path), mode], check=True, timeout=15)
        server_log = (folder / "server.log").read_text()
        expected_resumes = 2 if mode in ("success", "latest_impedance", "rebase") else (1 if mode in ("unchanged", "fallback") else 0)
        assert server_log.count("RESUMED") == expected_resumes, server_log
        if mode == "latest":
            assert server_log.count("SERVO_STEP_FAILURE") == 1, server_log
    finally:
        for child in reversed(children):
            child.terminate()
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: child.kill(); child.wait()
        for log in logs: log.close()
