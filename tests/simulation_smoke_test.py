"""Finite headless startup on private endpoints; never publish to the deployment bus."""
from pathlib import Path
import socket
import subprocess
import sys
import tempfile

simulation, root = sys.argv[1:]
with tempfile.TemporaryDirectory(prefix="aviator-simulation-smoke-") as directory:
    sockets = [socket.socket() for _ in range(3)]
    try:
        for sock in sockets:
            sock.bind(("127.0.0.1", 0))
        endpoints = [f"tcp://127.0.0.1:{s.getsockname()[1]}" for s in sockets]
    finally:
        for sock in sockets:
            sock.close()
    path = Path(directory) / "system.yaml"
    path.write_text(f"robot: {Path(root).resolve() / 'config/robot.yaml'}\n"
                    f"config_id: aviator-control-v1\n"
                    f"bus:\n  publish: {endpoints[0]}\n  subscribe: {endpoints[1]}\n"
                    f"manipulator_service: {endpoints[2]}\n"
                    "command_timeout_ms: 50\norigin_timeout_ms: 100\n")
    result = subprocess.run([simulation, "--config", str(path), "--headless", "--no-camera",
                             "--duration", "0.1"], cwd=directory, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "READY simulation" in result.stdout, result.stdout
    print("PASS isolated headless simulation startup/shutdown")
