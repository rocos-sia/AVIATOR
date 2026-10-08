#!/usr/bin/env python3
"""Standalone, hardware-free RH56FTP replacement using the production bus adapter."""

import math
import sys
import threading
import time

from rh56ftp_node import (
    DEFAULT_SAFE_POSE, DEFAULT_SPEED, Logger, Rh56FtpNode,
    canonical_to_rh, main as node_main, normalized_to_raw,
)


class FakeHandLink:
    """In-memory HandLink in device order, shared by control and reader threads.

    Positions follow targets at a bounded rate proportional to the speed setting.
    Force is a stored limit, not a contact model. None leaves a channel unchanged;
    -1 stops it at its current position, matching the adapter's hold writes.
    """

    def __init__(self, host="simulated", port=6000, timeout=0.2, *,
                 motion_rate=2.0, clock=time.monotonic):
        if not math.isfinite(motion_rate) or motion_rate <= 0:
            raise ValueError("motion_rate must be finite and positive")
        self.host, self.port = host, port
        self._clock = clock
        self._motion_rate = motion_rate
        self._lock = threading.Lock()
        self._connected = False
        self._position = canonical_to_rh(normalized_to_raw(list(DEFAULT_SAFE_POSE)))
        self._target = list(self._position)
        self._speed = [DEFAULT_SPEED] * 6
        self._force = [0] * 6
        self._mode = [0] * 6
        self._updated = clock()

    def connect(self):
        with self._lock:
            self._connected = True
            self._updated = self._clock()

    def close(self):
        with self._lock:
            self._connected = False

    def _advance(self):
        if not self._connected:
            raise RuntimeError("simulated hand is disconnected")
        now = self._clock()
        elapsed = max(0.0, now - self._updated)
        self._updated = now
        for i in range(6):
            step = elapsed * self._motion_rate * 1000 * self._speed[i] / DEFAULT_SPEED
            delta = self._target[i] - self._position[i]
            self._position[i] += max(-step, min(step, delta))

    @staticmethod
    def _validate(values, maximum, *, stop=False):
        if len(values) != 6 or any(
            value is not None and (type(value) is not int or
                                   not (-1 if stop else 0) <= value <= maximum)
            for value in values
        ):
            raise ValueError("expected six valid register settings (or None)")

    def write_angle_set(self, values):
        self._validate(values, 1000, stop=True)
        with self._lock:
            self._advance()
            for i, value in enumerate(values):
                if value is not None:
                    self._target[i] = self._position[i] if value == -1 else value

    def _write_setting(self, values, target, maximum):
        self._validate(values, maximum)
        with self._lock:
            self._advance()
            for i, value in enumerate(values):
                if value is not None:
                    target[i] = value

    def write_speed_set(self, values):
        self._write_setting(values, self._speed, 1000)

    def write_force_set(self, values):
        self._write_setting(values, self._force, 3000)

    def write_mode_set(self, values):
        # The bus adapter uses mode 0 exclusively; no force-control simulation.
        self._write_setting(values, self._mode, 0)

    def read_state(self):
        with self._lock:
            self._advance()
            return {"angle": [int(math.floor(v + 0.5)) for v in self._position],
                    "force": [0] * 6, "current": [0] * 6, "err": [0] * 6,
                    "status": [0] * 6, "temp": [30] * 6}


class FakeRh56FtpNode(Rh56FtpNode):
    def make_state(self, now=None):
        state = super().make_state(now)
        state["simulated"] = True
        for hand in state["hands"].values():
            hand["position_source"] = "simulated"
            hand["sample_time_basis"] = "host_simulation"
        return state


def main(argv=None):
    return node_main(argv, simulated=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        Logger.error(f"fake_rh56ftp_hand: {exc}", file=sys.stderr)
        raise SystemExit(1)
