"""A blocked GPU driver must not block monitor HTTP or fabricate new samples."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


with tempfile.TemporaryDirectory(prefix='monitor-resource-latency-') as directory:
    root = Path(directory)
    config = root / 'monitor.yaml'
    config.write_text("version: 1\npreview:\n  endpoint: ''\n")
    env = dict(os.environ, AVIATOR_TEST_GPU_GATE=directory)
    env['LD_LIBRARY_PATH'] = str(Path(sys.argv[2]).parent) + ':' + env.get('LD_LIBRARY_PATH', '')
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    http_port = port()
    base = f'http://127.0.0.1:{http_port}'

    def fetch(path, timeout=1):
        with http.open(base + path, timeout=timeout) as response:
            return json.load(response)

    process = subprocess.Popen([sys.argv[1], '--bind', '127.0.0.1', '--port', str(http_port),
                                '--subscribe', f'tcp://127.0.0.1:{port()}',
                                '--config', str(config)], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        deadline = time.monotonic() + 5
        while True:
            try:
                first = fetch('/api/system')
                break
            except OSError:
                assert time.monotonic() < deadline, 'monitor did not start'
                time.sleep(.02)
        assert first['gpu']['percent'] == 37, 'test GPU library was not loaded'
        # No HTTP request initiates the second collection: sampling is independent.
        while not (root / 'sampling').exists():
            assert time.monotonic() < deadline, 'resource collection is not running in background'
            time.sleep(.02)
        for _ in range(3):
            assert fetch('/api/overview')['snapshot_generated_mono_us'] > 0
            assert fetch('/api/system')['sample_mono_us'] == first['sample_mono_us']
            assert not (root / 'release').exists(), 'collector must still be blocked'
        (root / 'release').touch()
        deadline = time.monotonic() + 3
        while fetch('/api/system')['sample_mono_us'] == first['sample_mono_us']:
            assert time.monotonic() < deadline, 'resource sampling did not recover'
            time.sleep(.02)
        print('Blocked GPU collection: overview responsive, cached timestamp preserved, recovery passed')
    finally:
        (root / 'release').touch()
        process.terminate()
        try:
            _, errors = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            _, errors = process.communicate()
        if errors:
            print(errors.decode(), file=sys.stderr)
