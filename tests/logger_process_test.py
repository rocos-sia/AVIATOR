"""Process lifecycle and storage failures; no third-party Python dependencies."""
import pathlib
import resource
import selectors
import signal
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
    config.write_text('config_version: 1\ncamera: {mode: invalid}\n')
    result = subprocess.run([executable, "--config", str(config)], capture_output=True, timeout=5)
    assert result.returncode == 1 and b"Ctrl+C" not in result.stdout

    result = subprocess.run([executable, "--output", str(root / "missing" / "file.mcap")],
                            capture_output=True, timeout=5)
    assert result.returncode == 1
    assert b"Ctrl+C" not in result.stdout
    assert b"recorded " not in result.stdout
print("logger process tests passed")
