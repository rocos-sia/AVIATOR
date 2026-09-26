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


def fetch(path, method='GET'):
    req = urllib.request.Request(base + path, method=method)
    with urllib.request.urlopen(req, timeout=2) as response:
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
    web = subprocess.Popen([sys.argv[1], '--port', str(port), '--subscribe', endpoint],
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    children.append(web)
    until(lambda: fetch('/'))
    assert json.loads(fetch('/api/state'))['streams'] == []
    assert b'AVIATOR' in fetch('/') and b'textContent' in fetch('/')
    for path, method, expected in [('/bad', 'GET', 404), ('/api/state', 'POST', 405),
                                   ('/api/message?id=no', 'GET', 400)]:
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
    producer.terminate()
    producer.wait(timeout=2)
    until(lambda: json.loads(fetch('/api/state'))['streams'][0]['status'] == 'STALE')
    web.send_signal(signal.SIGTERM)
    assert web.wait(timeout=2) == 0
    print('monitor HTTP/TCP tests passed')
finally:
    if slow:
        slow.close()
    for child in children:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=2)
