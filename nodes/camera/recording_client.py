"""Non-real-time, bounded raw-camera recording adapter for the camera node.

CameraPacket protobuf encoding has two length-delimited fields (see checked-in
.proto); no generated Python bindings or image Base64 conversion are needed.
Logger owns codec selection. This module never touches the business PUB socket.
"""

# Shared logging module in source and installed share/aviator layouts.
import sys as _log_sys
from pathlib import Path as _LogPath
_log_sys.path.insert(0, str(_LogPath(__file__).resolve().parents[1] / "common")
                     if (_LogPath(__file__).resolve().parents[1] / "common").is_dir()
                     else str(_LogPath(__file__).resolve().parents[2] / "common"))
from aviator_logger import Logger

import json
import queue
import threading



def _varint(value):
    output = bytearray()
    while value > 127:
        output.append((value & 127) | 128)
        value >>= 7
    output.append(value)
    return bytes(output)


def camera_packet(metadata, data):
    header = json.dumps(metadata, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return b"\x0a" + _varint(len(header)) + header + b"\x12" + _varint(len(data)) + data


class RecordingClient:
    def __init__(self, path, camera_id):
        self.enabled = False
        self.record_depth = False
        self.dropped = 0
        self.sent = 0
        self._pending_bytes = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._queue = queue.Queue()
        self._thread = None
        self.error = None
        if not path:
            return
        import yaml
        with open(path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if config.get("config_version") != 1:
            raise ValueError("unsupported recording config_version")
        camera = config.get("camera", {})
        mode = camera.get("mode", "disabled")
        if mode not in ("disabled", "raw", "compressed"):
            raise ValueError("invalid camera.mode")
        if mode == "disabled":
            return
        source = next((source for source in camera.get("sources", [{"camera_id": "cockpit"}])
                       if source["camera_id"] == camera_id), None)
        if source is None:
            raise ValueError("--camera-id must match recording camera.sources")
        self.record_depth = source.get("record_depth", True)
        if not isinstance(self.record_depth, bool):
            raise ValueError("record_depth must be a boolean")
        self.endpoint = camera.get("record_endpoint", "tcp://127.0.0.1:5557")
        self.limit = camera.get("max_record_bytes", 16777216)
        self.budget = camera.get("queue_bytes", 268435456)
        self.hwm = camera.get("receive_hwm", 16)
        if (not self.endpoint.startswith("tcp://") or self.limit <= 0 or
                self.budget < self.limit or self.hwm <= 0):
            raise ValueError("invalid camera recording endpoint/limits")
        self.enabled = True

    def _run(self):
        import zmq
        try:
            with zmq.Context() as context:
                with context.socket(zmq.PUSH) as socket:
                    socket.setsockopt(zmq.SNDHWM, self.hwm)
                    socket.setsockopt(zmq.LINGER, 0)
                    socket.setsockopt(zmq.IMMEDIATE, 1)
                    socket.connect(self.endpoint)
                    while not self._stop.is_set() or not self._queue.empty():
                        try:
                            packet = self._queue.get(timeout=0.05)
                        except queue.Empty:
                            continue
                        with self._lock:
                            self._pending_bytes -= len(packet)
                        try:
                            socket.send(packet, flags=zmq.DONTWAIT)
                            self.sent += 1
                        except zmq.Again:
                            with self._lock:
                                self.dropped += 1
        except Exception as error:
            self.error = error

    def submit(self, metadata, data):
        if not self.enabled:
            return
        if self._stop.is_set():
            raise RuntimeError("camera recording sender is closed")
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        if self.error:
            raise RuntimeError("camera recording sender failed") from self.error
        packet = camera_packet(metadata, data)
        with self._lock:
            if len(packet) > self.limit or self._pending_bytes + len(packet) > self.budget:
                self.dropped += 1
                return
            self._pending_bytes += len(packet)
            self._queue.put_nowait(packet)

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join()
        if self.enabled:
            Logger.info(f"camera recording: queued-to-zmq={self.sent}, dropped={self.dropped}")
            if self.error:
                raise RuntimeError("camera recording sender failed") from self.error
