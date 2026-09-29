"""Process lifecycle and storage failures; no third-party Python dependencies."""
import pathlib
import resource
import selectors
import signal
import socket
import subprocess
import sys
import tempfile
import time

executable = sys.argv[1]


def ready(process):
    deadline = time.monotonic() + 5
    data = b""
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdout, selectors.EVENT_READ)
        while time.monotonic() < deadline:
            if selector.select(0.1):
                block = process.stdout.read1(4096)
                if not block:
                    raise AssertionError(f"logger exited before startup: {data!r}")
                data += block
                if b"Ctrl+C" in data:
                    return
    raise AssertionError(f"logger startup timed out: {data!r}")


def fail_storage():
    signal.signal(signal.SIGXFSZ, signal.SIG_IGN)


with tempfile.TemporaryDirectory(prefix="aviator-logger-process-") as root:
    root = pathlib.Path(root)
    for sig in (signal.SIGINT, signal.SIGTERM):
        output = root / f"signal-{sig}.mcap"
        process = subprocess.Popen([executable, "--output", str(output)],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            ready(process)
            process.send_signal(sig)
            data, _ = process.communicate(timeout=5)
            assert process.returncode == 0, data
            assert output.is_file()
            assert not pathlib.Path(str(output) + ".partial").exists()
            assert b"recorded 0 messages" in data
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    output = root / "disk-error.mcap"
    process = subprocess.Popen([executable, "--output", str(output)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               preexec_fn=fail_storage)
    try:
        ready(process)
        # Set the limit after the header is written, so closing must fail.
        limit = pathlib.Path(str(output) + ".partial").stat().st_size + 1
        resource.prlimit(process.pid, resource.RLIMIT_FSIZE, (limit, limit))
        process.send_signal(signal.SIGTERM)
        data, _ = process.communicate(timeout=5)
        assert process.returncode == 1, data
        assert not output.exists()
        assert pathlib.Path(str(output) + ".partial").exists()
        assert b"recorded " not in data, data
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()

    config = root / "recording.yaml"
    yaml_output = root / "from-yaml.mcap"
    override_output = root / "from-cli.mcap"
    config.write_text(f'config_version: 1\noutput: {{path: "{yaml_output}"}}\ncamera: {{mode: disabled}}\n')
    process = subprocess.Popen([executable, "--output", str(override_output), "--config", str(config)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        ready(process)
        process.send_signal(signal.SIGTERM)
        data, _ = process.communicate(timeout=5)
        assert process.returncode == 0, data
        assert override_output.exists() and not yaml_output.exists(), "CLI must override YAML regardless of argument order"
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    # CLI image path overrides YAML regardless of option order; files finalize separately.
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        camera_endpoint = f"tcp://127.0.0.1:{reserve.getsockname()[1]}"
    image_yaml = root / "yaml-images.mcap"
    image_cli = root / "cli-images.mcap"
    split_data = root / "split-data.mcap"
    config.write_text(f'config_version: 1\noutput: {{image_path: "{image_yaml}"}}\n'
                      f'camera: {{mode: raw, record_endpoint: "{camera_endpoint}"}}\n')
    process = subprocess.Popen([executable, "--image-output", str(image_cli), "--config", str(config),
                                "--output", str(split_data)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        ready(process)
        process.send_signal(signal.SIGTERM)
        data, _ = process.communicate(timeout=5)
        assert process.returncode == 0, data
        assert split_data.exists() and image_cli.exists() and not image_yaml.exists()
        assert not pathlib.Path(str(image_cli) + ".partial").exists()
        assert b"images: 0 messages" in data and b"DEGRADED" in data
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    for conflict in (root / "conflict.mcap", root / "conflict.mcap.partial"):
        result = subprocess.run([executable, "--config", str(config), "--output",
                                 str(root / "conflict.mcap"), "--image-output", str(conflict)],
                                capture_output=True, timeout=5)
        assert result.returncode == 1 and b"paths conflict" in result.stderr
        assert not (root / "conflict.mcap.partial").exists()
    for image in (image_cli, root / "missing" / "images.mcap"):
        result = subprocess.run([executable, "--config", str(config), "--output",
                                 str(root / f"failed-{image.parent.name}.mcap"),
                                 "--image-output", str(image)], capture_output=True, timeout=5)
        assert result.returncode == 1 and b"Ctrl+C" not in result.stdout
    # An explicit image path does not enable image recording by itself.
    disabled_data = root / "disabled-data.mcap"
    process = subprocess.Popen([executable, "--output", str(disabled_data),
                                "--image-output", str(root / "disabled-images.mcap")],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        ready(process)
        process.send_signal(signal.SIGINT)
        data, _ = process.communicate(timeout=5)
        assert process.returncode == 0, data
        assert disabled_data.exists() and not (root / "disabled-images.mcap").exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    config.write_text('config_version: 1\ncamera: {mode: invalid}\n')
    result = subprocess.run([executable, "--config", str(config)], capture_output=True, timeout=5)
    assert result.returncode == 1 and b"Ctrl+C" not in result.stdout

    result = subprocess.run([executable, "--output", str(root / "missing" / "file.mcap")],
                            capture_output=True, timeout=5)
    assert result.returncode == 1
    assert b"Ctrl+C" not in result.stdout
    assert b"recorded " not in result.stdout
print("logger process tests passed")
