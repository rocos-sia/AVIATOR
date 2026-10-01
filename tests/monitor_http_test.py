"""Local process integration; Python is test-only, production remains C++17."""
import json
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request


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
http = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def fetch(path, method='GET', origin=base):
    req = urllib.request.Request(origin + path, method=method)
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
                            '--preview', 'off'],
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
                                '--subscribe', endpoint, '--preview', 'off'],
                               stdout=subprocess.DEVNULL)
        children.append(web)
        until(lambda: fetch('/', origin=f'http://{address}:{port}'))
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
