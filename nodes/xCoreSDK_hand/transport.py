"""RH56 HandLink interface backed solely by SDK RS485 transactions.

Values use RH device order, just like the Modbus HandLink. Policy is owned by
rh56ftp_node; this module adds no watchdog, automatic motion or recovery rules.
"""
import ctypes
import threading
import time

class SdkConnection:
    def __init__(self, library, host, timeout_ms):
        self.library, self.host, self.timeout_ms = library, host, timeout_ms
        self.handle = None
        self.attempted = False
        self.failure = ''
        self.ready = threading.Condition()
        self.next_ticket = self.serving = 0

    def connect(self):
        if self.handle:
            return
        if self.attempted:
            raise RuntimeError(self.failure or 'SDK connection unavailable')
        self.attempted = True
        self.sdk = ctypes.CDLL(str(self.library))
        self.sdk.hand_sdk_connect.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.sdk.hand_sdk_connect.restype = ctypes.c_void_p
        self.sdk.hand_sdk_error.restype = ctypes.c_char_p
        self.sdk.hand_sdk_transfer.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8),
                                               ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
        self.sdk.hand_sdk_transfer.restype = ctypes.c_int
        self.handle = self.sdk.hand_sdk_connect(self.host.encode(), self.timeout_ms)
        if not self.handle:
            self.failure = self.sdk.hand_sdk_error().decode()
            raise RuntimeError(self.failure)

    def transfer(self, tx, count):
        if not self.handle:
            raise RuntimeError(self.failure or 'SDK not connected')
        # FIFO only for this physical RS485 port; never shared with an arm.
        with self.ready:
            ticket = self.next_ticket
            self.next_ticket += 1
            self.ready.wait_for(lambda: ticket == self.serving)
        try:
            raw = (ctypes.c_uint8 * len(tx))(*tx)
            rx = (ctypes.c_uint8 * count)()
            if self.sdk.hand_sdk_transfer(self.handle, raw, len(tx), rx, count):
                raise RuntimeError(self.sdk.hand_sdk_error().decode())
            return bytes(rx)
        finally:
            with self.ready:
                self.serving += 1
                self.ready.notify_all()

class HandLink:
    def __init__(self, connection, hand_id=1):
        if type(hand_id) is not int or not 1 <= hand_id <= 254:
            raise ValueError('hand ID must be 1..254')
        self.connection = connection
        self.host, self.port, self.hand_id = connection.host, None, hand_id
        self.connected = False

    def connect(self):
        self.connection.connect()
        self.connected = True

    def close(self):
        # No SDK disconnect or object destruction. This closes only the adapter.
        self.connected = False

    def exchange(self, operation, address, data, count):
        if not self.connected:
            raise RuntimeError('Hand SDK link is closed')
        tx = bytes([0xeb, 0x90, self.hand_id, len(data) + 3, operation,
                    address & 255, address >> 8]) + bytes(data)
        tx += bytes([sum(tx[2:]) & 255])
        started = time.monotonic()
        try:
            rx = self.connection.transfer(tx, count + 8)
            if (len(rx) != count + 8 or rx[:2] != b'\x90\xeb' or
                rx[2] != self.hand_id or rx[3] != count + 3 or
                rx[4:7] != tx[4:7] or rx[-1] != sum(rx[2:-1]) & 255):
                raise RuntimeError('Invalid RS485 response header/length/checksum')
            payload = rx[7:-1]
            if operation == 0x12 and payload != b'\x01':
                raise RuntimeError('RS485 write rejected')
            return payload
        except Exception as exc:
            raise RuntimeError(f'RS485 host={self.host} id={self.hand_id} '
                               f'op={operation:#x} register={address} '
                               f'elapsed_ms={(time.monotonic()-started)*1000:.2f}: {exc}') from exc

    def read_state(self):
        # Two transactions instead of six; all non-tactile feedback is retained.
        return dict(t=time.time(), **self.read_position(), **self.read_telemetry())

    def read_position(self):
        angles = self.exchange(0x11, 1546, [12], 12)
        return {'angle': [int.from_bytes(angles[i:i+2], 'little', signed=True)
                          for i in range(0, 12, 2)]}

    def read_errors(self):
        return {'err': list(self.exchange(0x11, 1606, [6], 6))}

    def read_telemetry(self):
        telemetry = self.exchange(0x11, 1582, [42], 42)
        def words(data, signed=False):
            return [int.from_bytes(data[i:i+2], 'little', signed=signed) for i in range(0,len(data),2)]
        return dict(force=words(telemetry[:12], True),
                    current=words(telemetry[12:24]), err=list(telemetry[24:30]),
                    status=list(telemetry[30:36]), temp=list(telemetry[36:42]))

    def write_words(self, address, values):
        data = b''.join((v & 65535).to_bytes(2, 'little') for v in values)
        self.exchange(0x12, address, data, 1)

    def write_angle_set(self, values):
        if len(values) != 6 or any(v is not None and (type(v) is not int or not -1 <= v <= 1000) for v in values):
            raise ValueError('Six angle values in -1..1000 or None required')
        start = 0
        while start < 6:
            if values[start] is None:
                start += 1
                continue
            end = start + 1
            while end < 6 and values[end] is not None:
                end += 1
            self.write_words(1486 + start * 2, values[start:end])
            start = end

    def write_pause(self):
        self.exchange(0x12, 1634, [1], 1)

    def write_mode_set(self, values):
        if len(values) != 6 or any(type(v) is not int or v not in (0,1,2) for v in values):
            raise ValueError('Six mode bytes required')
        self.exchange(0x12, 1625, values, 1)

    def write_speed_set(self, values):
        self._setting(1522, values, 1000)

    def write_force_set(self, values):
        self._setting(1498, values, 3000)

    def _setting(self, address, values, maximum):
        if len(values) != 6 or any(type(v) is not int or not 0 <= v <= maximum for v in values):
            raise ValueError('Invalid six-channel setting')
        self.write_words(address, values)
