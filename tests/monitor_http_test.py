"""Local process integration; Python is test-only, production remains C++17."""
import json
from http.client import HTTPResponse
import hashlib
import pathlib
import tempfile
import copy
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
import urllib.parse
import os


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


port, zmq_port = free_port(), free_port()
while port == zmq_port:
    zmq_port = free_port()
base = f'http://127.0.0.1:{port}'
endpoint = f'tcp://127.0.0.1:{zmq_port}'
children = []
slow = None
temporary = tempfile.TemporaryDirectory(prefix="monitor-http-")
config_path = pathlib.Path(temporary.name) / "monitor.yaml"
config_path.write_text("version: 1\npreview:\n  endpoint: ''\n")
http = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def fetch(path, method='GET', origin=base, payload=None, headers=None):
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(origin + path, method=method, data=data,
                                 headers=headers or {})
    with http.open(req, timeout=2) as response:
        return response.read()


def until(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (urllib.error.URLError, ConnectionError):
            pass
        time.sleep(.02)
    raise AssertionError('condition timed out')


try:
    web = subprocess.Popen([sys.argv[1], '--port', str(port), '--subscribe', endpoint,
                            '--preview', 'off', '--config', str(config_path),
                            '--log-dir', temporary.name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    children.append(web)
    until(lambda: fetch('/'))
    # A second local IPv4 address catches a server bound only to 127.0.0.1.
    alternate = f'http://127.0.0.2:{port}'
    assert b'AVIATOR' in fetch('/', origin=alternate)
    assert json.loads(fetch('/api/state', origin=alternate))['streams'] == []
    assert b'textContent' in fetch('/assets/app.js', origin=alternate)
    assert json.loads(fetch('/api/state'))['streams'] == []
    assert b'AVIATOR' in fetch('/') and b'textContent' in fetch('/assets/app.js')
    assert len(json.loads(fetch('/api/overview'))['publishers']) == 7
    manifest = json.loads(fetch('/api/model-manifest'))
    assert manifest['resource_errors'] == []
    assert b'<robot' in fetch(manifest['model_url'])
    assert b'export' in fetch('/assets/vendor.js')
    assert json.loads(fetch('/api/camera/latest'))['state'] in ['WAITING', 'UNCONFIGURED']
    assert b'REQ / REP' in fetch('/') and b'service-rows' in fetch('/')
    for path, method, expected in [('/bad', 'GET', 404), ('/api/state', 'POST', 405),
                                   ('/api/message?id=no', 'GET', 400),
                                   ('/models/../config/flight.yaml', 'GET', 404),
                                   ('/api/camera/frame/missing', 'GET', 404)]:
        try:
            fetch(path, method)
            raise AssertionError('request should fail')
        except urllib.error.HTTPError as error:
            assert error.code == expected
    assert b'tab-logs' in fetch('/') and b'/api/logs' in fetch('/assets/logs.js')
    assert json.loads(fetch('/api/logs'))['files'] == []
    log = pathlib.Path(temporary.name) / 'core.log'
    stamp = '[2026-10-05 12:34:56.123]'
    log.write_text(''.join(f'{stamp} [aviator] [{level}] [thread 1] message\n'
                           for level in ['trace', 'debug', 'info', 'warning', 'error', 'critical']) +
                   f'{stamp} [warning] default spdlog\n' +
                   f'{stamp} [aviator] [\x1b[31merror\x1b[0m] coloured\n' +
                   'external software [error] is message text\n' +
                   '<script>alert(1)</script>\n', encoding='utf-8')
    def read_log(name='core.log'):
        return json.loads(fetch('/api/logs?file=' + urllib.parse.quote(name, safe='')))
    result = read_log()
    assert [entry['level'] for entry in result['entries']] == [
        'trace', 'debug', 'info', 'warn', 'error', 'critical', 'warn', 'error', 'info', 'info']
    assert '\x1b' not in result['entries'][7]['text']
    assert not result['truncated']
    with log.open('ab') as stream:
        stream.write(b'new output\ninvalid utf8: \xff\npartial')
    assert read_log()['entries'][-1]['text'] == 'partial'
    assert '\ufffd' in read_log()['entries'][-2]['text']
    with log.open('a') as stream:
        stream.write(' completed\n')
    assert read_log()['entries'][-1]['text'] == 'partial completed'
    log.write_text('')
    assert read_log()['entries'] == []
    log.write_text('reset after truncation\n')
    assert len(read_log()['entries']) == 1
    replacement = pathlib.Path(temporary.name) / 'replacement'
    replacement.write_text('rotated\n')
    replacement.replace(log)
    assert read_log()['entries'][0]['text'] == 'rotated'
    log.write_text(''.join(f'line {i}\n' for i in range(2500)))
    result = read_log()
    assert result['truncated'] and len(result['entries']) == 2000
    assert result['entries'][0]['text'] == 'line 500'
    log.write_text('x' * (600 * 1024) + '\nlatest\n')
    result = read_log()
    assert result['truncated'] and result['entries'] == [{'level': 'info', 'text': 'latest'}]
    named = pathlib.Path(temporary.name) / '相机 output.log'
    named.write_text('camera output\n')
    assert read_log(named.name)['entries'][0]['level'] == 'info'
    (pathlib.Path(temporary.name) / 'linked.log').symlink_to(config_path)
    os.mkfifo(pathlib.Path(temporary.name) / 'pipe.log')
    assert [f['name'] for f in json.loads(fetch('/api/logs'))['files']] == ['core.log', named.name]
    for name, expected in [('../monitor.yaml', 400), ('../core.log', 400),
                           ('/tmp/outside.log', 400), ('bad\x00.log', 400),
                           ('linked.log', 503), ('pipe.log', 503), ('missing.log', 503)]:
        try:
            read_log(name)
            raise AssertionError('unsafe or missing log should fail')
        except urllib.error.HTTPError as error:
            assert error.code == expected
    named.unlink()
    assert len(json.loads(fetch('/api/logs'))['files']) == 1
    assert b'tab-settings' in fetch('/') and b'config-save' in fetch('/')
    assert b'/api/config' in fetch('/assets/settings.js')
    settings = json.loads(fetch('/api/config'))
    headers = {'Content-Type': 'application/json', 'X-Monitor-Config': '1'}
    def save(config=None, yaml=None, revision=None):
        payload = {'revision': revision or settings['revision']}
        payload.update({'yaml': yaml} if yaml is not None else {'config': config})
        return json.loads(fetch('/api/config', 'PUT', payload=payload, headers=headers))
    for bad in ['timeouts_ms: {arm.state: -1}', 'sources: {unknown: camera}',
                'preview: {endpoint: "tcp://"}', 'version: 1\nversion: 1']:
        before = config_path.read_bytes()
        try:
            save(yaml=bad)
            raise AssertionError('invalid configuration accepted')
        except urllib.error.HTTPError as error:
            assert error.code == 400
        assert json.loads(fetch('/api/config')) == settings
        assert config_path.read_bytes() == before
    for extra in [{}, {'Content-Type':'text/plain'},
                  {**headers, 'Origin':'http://untrusted.example'}]:
        try:
            fetch('/api/config', 'PUT', payload={'revision':settings['revision'],
                  'config':settings['config']}, headers=extra)
            raise AssertionError('cross-origin/untyped write accepted')
        except urllib.error.HTTPError as error:
            assert error.code == 403
    # A fragmented body larger than the old 4 KiB header-only request limit.
    payload = json.dumps({'revision':settings['revision'], 'config':settings['config']}, indent=4).encode()
    with socket.create_connection(('127.0.0.1', port)) as fragmented:
        fragmented.sendall((f'PUT /api/config HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n'
                            'Content-Type: application/json\r\nX-Monitor-Config: 1\r\n'
                            f'Content-Length: {len(payload)}\r\n\r\n').encode())
        for offset in range(0, len(payload), 400):
            fragmented.sendall(payload[offset:offset+400])
        result = b''
        while True:
            part = fragmented.recv(8192)
            if not part: break
            result += part
        assert result.startswith(b'HTTP/1.1 200'), result
    old_revision = settings['revision']
    settings = json.loads(fetch('/api/config'))
    assert settings['revision'] != old_revision
    try:
        save(settings['config'], revision=old_revision)
        raise AssertionError('stale revision accepted')
    except urllib.error.HTTPError as error:
        assert error.code == 409
    # A persistence error must leave the active configuration intact.
    config_path.rename(config_path.with_suffix('.backup'))
    config_path.mkdir()
    try:
        save(settings['config'])
        raise AssertionError('directory replaced by configuration')
    except urllib.error.HTTPError as error:
        assert error.code == 400
    assert json.loads(fetch('/api/config')) == settings
    config_path.rmdir()
    config_path.with_suffix('.backup').rename(config_path)
    # A remote client can take longer than two seconds to receive a large mesh.
    # Keep its receive window small so the kernel cannot buffer the entire STL.
    mesh_path = '/models/meshes/Cessna/steering_wheel.STL'
    with socket.socket() as download:
        download.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        download.settimeout(10)
        download.connect(('127.0.0.1', port))
        download.sendall(f'GET {mesh_path} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n\r\n'.encode())
        time.sleep(2.3)
        assert json.loads(fetch('/api/state'))['streams'] == []
        response = HTTPResponse(download)
        response.begin()
        assert response.status == 200
        mesh = response.read()
        expected = (pathlib.Path(__file__).resolve().parents[1] / 'models/meshes/Cessna/steering_wheel.STL').read_bytes()
        assert len(mesh) == len(expected)
        assert hashlib.sha256(mesh).digest() == hashlib.sha256(expected).digest()
        response.close()
    slow = socket.create_connection(('127.0.0.1', port))
    slow.sendall(b'GET / HTTP/1.1\r\n')  # Incomplete headers must not block other clients.
    producer = subprocess.Popen([sys.argv[2], '--publish', endpoint], stdout=subprocess.DEVNULL)
    children.append(producer)
    def fresh():
        rows = json.loads(fetch('/api/state'))['streams']
        return rows and rows[0]['status'] == 'FRESH' and rows[0]
    row = until(fresh)
    detail = json.loads(fetch('/api/message?id=' + str(row['id'])))
    assert detail['system']['state'] == 'CONTROL'
    assert row['summary']['source'] == 'JOYSTICK'
    assert json.loads(fetch('/api/overview'))['system']['current']['state'] == 'CONTROL'
    candidate = copy.deepcopy(settings['config'])
    candidate['sources']['flight.state'] = 'other_core'
    candidate['timeouts_ms']['flight.state'] = 2000
    candidate['preview'].update(endpoint='tcp://127.0.0.1:1', camera_id='new_camera', publisher_id='new_publisher')
    settings = save(candidate)
    assert json.loads(fetch('/api/overview'))['system']['current'] is None
    assert json.loads(fetch('/api/camera/latest'))['camera_id'] == 'new_camera'
    candidate['sources']['flight.state'] = 'aviator_core'
    candidate['preview']['endpoint'] = ''
    candidate['arm_joints']['left'][0]['offset_rad'] = .25
    settings = save(candidate)
    assert json.loads(fetch('/api/overview'))['system']['current']['state'] == 'CONTROL'
    assert json.loads(fetch('/api/model-manifest'))['arm_joints']['left'][0]['offset_rad'] == .25
    assert json.loads(fetch('/api/camera/latest'))['state'] == 'UNCONFIGURED'
    assert 'sources:' in config_path.read_text() and 'other_core' not in config_path.read_text()
    def service():
        rows = json.loads(fetch('/api/state'))['services']
        return rows and rows[0]['status'] == 'ACCEPTED' and rows[0]
    transaction = until(service)
    assert transaction['operation'] == 'grasp_wheel'
    assert transaction['observation'] == 'QUEUED'
    details = json.loads(fetch('/api/message?id=' + str(transaction['id'])))
    assert details['request']['request_id'] == details['reply']['request_id']
    assert details['reply']['status'] == 'ACCEPTED'
    producer.terminate()
    producer.wait(timeout=2)
    until(lambda: json.loads(fetch('/api/state'))['streams'][0]['status'] == 'STALE')
    web.send_signal(signal.SIGTERM)
    assert web.wait(timeout=2) == 0
    for address, excluded in [('127.0.0.1', '127.0.0.2'), ('127.0.0.2', '127.0.0.1')]:
        web = subprocess.Popen([sys.argv[1], '--bind', address, '--port', str(port),
                                '--subscribe', endpoint, '--preview', 'off', '--config', str(config_path)],
                               stdout=subprocess.DEVNULL)
        children.append(web)
        until(lambda: fetch('/', origin=f'http://{address}:{port}'))
        restored = json.loads(fetch('/api/config', origin=f'http://{address}:{port}'))
        assert restored['config'] == settings['config']
        assert not json.loads(fetch('/api/logs', origin=f'http://{address}:{port}'))['configured']
        with socket.socket() as sock:
            sock.settimeout(1)
            assert sock.connect_ex((excluded, port)) != 0, '--bind must restrict the listener'
        web.send_signal(signal.SIGTERM)
        assert web.wait(timeout=2) == 0
    for args in [ ['--bind'], ['--bind', 'localhost'], ['--bind', '256.0.0.1'],
                  ['--bind', '::1'] ]:
        result = subprocess.run([sys.argv[1], *args], capture_output=True, timeout=2)
        assert result.returncode != 0
        assert b'missing option value' in result.stderr or b'IPv4 address' in result.stderr
    print('monitor HTTP/TCP tests passed')
finally:
    if slow:
        slow.close()
    for child in children:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=2)

    temporary.cleanup()
