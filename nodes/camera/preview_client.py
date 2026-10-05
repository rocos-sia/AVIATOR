"""Independent latest-only JPEG PUB for the read-only monitor, never Logger PULL."""

# Shared logging module in source and installed share/aviator layouts.
import sys as _log_sys
from pathlib import Path as _LogPath
_log_sys.path.insert(0, str(_LogPath(__file__).resolve().parents[1] / "common")
                     if (_LogPath(__file__).resolve().parents[1] / "common").is_dir()
                     else str(_LogPath(__file__).resolve().parents[2] / "common"))
from aviator_logger import Logger

import json
import sys
import threading
import time

import cv2
import zmq


class PreviewClient:
    def __init__(self, settings, camera_id, publisher_id="camera", *, start=True):
        if not isinstance(settings, dict):
            raise ValueError("preview must be a mapping")
        self.enabled = settings.get("enabled", False)
        if not isinstance(self.enabled, bool):
            raise ValueError("preview.enabled must be boolean")
        self.endpoint = settings.get("endpoint", "tcp://127.0.0.1:5561")
        self.width = settings.get("width", 640)
        self.height = settings.get("height", 360)
        self.fps = settings.get("fps", 15)
        self.quality = settings.get("jpeg_quality", 80)
        for value, minimum, maximum in [(self.width, 16, 1920), (self.height, 16, 1080),
                                        (self.fps, 1, 30), (self.quality, 30, 95)]:
            if not isinstance(value, int) or isinstance(value, bool) or not minimum <= value <= maximum:
                raise ValueError("invalid preview size/rate/JPEG quality")
        if not isinstance(self.endpoint, str) or not self.endpoint.startswith("tcp://127.0.0.1:"):
            raise ValueError("preview endpoint must bind a loopback TCP address")
        self.camera_id, self.publisher_id = camera_id, publisher_id
        self.error = None
        self._latest = None
        self._stopping = False
        self._condition = threading.Condition()
        self._thread = None
        if start:
            self.start()

    def start(self):
        if self.enabled and self._thread is None:
            self._thread = threading.Thread(target=self._run, name="camera-preview", daemon=True)
            self._thread.start()

    def submit(self, image, *, session_id, clock_id, frame_id, sample_mono_us):
        if not self.enabled or self.error is not None:
            return
        # Detector/display code may mutate the source; the worker owns this copy.
        snapshot = image.copy()
        metadata = dict(version=1, encoding="jpeg", publisher_id=self.publisher_id,
                        session_id=session_id, camera_id=self.camera_id, clock_id=clock_id,
                        frame_id=frame_id, sequence=frame_id, sample_mono_us=sample_mono_us)
        with self._condition:
            self._latest = (snapshot, metadata)
            self._condition.notify()

    def _run(self):
        context = zmq.Context()
        pub = context.socket(zmq.PUB)
        pub.setsockopt(zmq.SNDHWM, 2)
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(zmq.SNDTIMEO, 0)
        try:
            pub.bind(self.endpoint)
            next_frame = 0.0
            while True:
                with self._condition:
                    while not self._stopping and (self._latest is None or time.monotonic() < next_frame):
                        self._condition.wait(timeout=max(0.001, next_frame - time.monotonic())
                                             if self._latest is not None else None)
                    if self._stopping:
                        break
                    image, metadata = self._latest
                    self._latest = None
                next_frame = time.monotonic() + 1.0 / self.fps
                h, w = image.shape[:2]
                scale = min(self.width / w, self.height / h, 1.0)
                size = (max(1, round(w * scale)), max(1, round(h * scale)))
                if size != (w, h):
                    image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok or len(encoded) > 2 * 1024 * 1024:
                    continue
                metadata.update(width=size[0], height=size[1], original_width=w, original_height=h,
                                resize_mode="fit", preview_fps=self.fps)
                try:
                    pub.send_multipart([f"camera.rgb.{self.camera_id}".encode(),
                                        json.dumps(metadata).encode(), encoded.tobytes()], flags=zmq.DONTWAIT)
                except zmq.Again:
                    pass  # Optional previews may drop; acquisition/recording continues.
        except Exception as error:
            self.error = error
            Logger.warn(f"camera RGB preview disabled: {error}", file=sys.stderr)
        finally:
            pub.close()
            context.term()

    def close(self):
        with self._condition:
            self._stopping = True
            self._latest = None
            self._condition.notify_all()
        if self._thread and self._thread.ident is not None:
            self._thread.join(timeout=2)
