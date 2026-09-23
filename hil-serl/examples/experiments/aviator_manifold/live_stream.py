"""ZMQ PUB publisher for streaming one env's live kinematic state to a viewer.

Training (serl_clean) publishes; the MuJoCo viewer (mujo env) subscribes.  PUB
never blocks when no subscriber is connected, and CONFLATE keeps only the latest
frame, so enabling the stream cannot stall the actor or build a backlog.

Enable in the launch shell with either the address or ``1`` for the default::

    AVIATOR_LIVE_VIEWER=1 bash launch_rlpd.sh
    AVIATOR_LIVE_VIEWER=tcp://127.0.0.1:5557 bash launch_rlpd.sh

Frame layout (18 float64, native little-endian):
    [theta, s, qL(7), qR(7), d_min, max_qdot]
"""

from __future__ import annotations

import os
import warnings

import numpy as np

try:
    import zmq
except ImportError:  # pragma: no cover - the stream is optional
    zmq = None

DEFAULT_ADDR = "tcp://127.0.0.1:5557"


class LiveStreamPublisher:
    """Fire-and-forget publisher of per-step kinematic frames."""

    def __init__(self, addr: str):
        self.addr = addr
        self._ctx = None
        self._sock = None
        if zmq is None:
            return
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        try:
            self._sock.bind(addr)
        except zmq.ZMQError as exc:
            self._sock.close(0)
            self._sock = None
            warnings.warn(f"AVIATOR live stream disabled: cannot bind {addr}: {exc}",
                          RuntimeWarning, stacklevel=2)

    def publish(self, x, q, d_min, max_qdot):
        if self._sock is None:
            return
        frame = np.concatenate([
            np.asarray(x, dtype=np.float64).reshape(-1),
            np.asarray(q, dtype=np.float64).reshape(-1),
            np.asarray([d_min, max_qdot], dtype=np.float64),
        ])
        try:
            self._sock.send(frame.tobytes(), zmq.NOBLOCK)
        except zmq.Again:
            # Viewer is absent or cannot keep up; drop this telemetry frame.
            pass
        except zmq.ZMQError as exc:
            warnings.warn(f"AVIATOR live stream disabled after send error: {exc}",
                          RuntimeWarning, stacklevel=2)
            self.close()

    def close(self):
        if self._sock is not None:
            self._sock.close(0)
            self._sock = None


def publisher_from_env(key: str = "AVIATOR_LIVE_VIEWER"):
    """Return a LiveStreamPublisher if the env var is set, else None."""
    val = os.environ.get(key)
    if not val:
        return None
    addr = DEFAULT_ADDR if val == "1" else val
    return LiveStreamPublisher(addr)
