"""Bounded, same-user local control protocol."""
import json
import socket
import struct
from paths import runtime_dir

MAX_BYTES = 262144


def receive(sock):
    data = bytearray()
    while b'\n' not in data:
        part = sock.recv(min(8192, MAX_BYTES + 1 - len(data)))
        if not part:
            raise ValueError('控制连接提前关闭')
        data.extend(part)
        if len(data) > MAX_BYTES:
            raise ValueError('控制消息过大')
    return json.loads(data.split(b'\n', 1)[0])


def send(sock, value):
    data = json.dumps(value, ensure_ascii=False).encode() + b'\n'
    if len(data) > MAX_BYTES:
        raise ValueError('控制消息过大')
    sock.sendall(data)


def peer_uid(sock):
    return struct.unpack('3i', sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]


def request(action, **values):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(3)
        sock.connect(str(runtime_dir() / 'control.sock'))
        send(sock, {'version': 1, 'action': action, **values})
        result = receive(sock)
    if not result.get('ok'):
        raise ValueError(result.get('error', '控制请求失败'))
    return result['state']
