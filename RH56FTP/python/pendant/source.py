"""数据源抽象：真实灵巧手 / 模拟数据。

界面和轮询线程只依赖 Source 接口，因此没有硬件时用 --mock 即可开发。
"""
from __future__ import annotations

import math
import time

from .handlink import FINGERS, N_FINGERS, TOUCH_ZONES, HandLink


class Source:
    """数据源接口。"""

    name = "?"

    def open(self) -> None: ...
    def close(self) -> None: ...
    def read_state(self) -> dict: ...
    def read_touch(self) -> dict[str, list[int]]: ...
    def write_angle_set(self, values: list[int]) -> None: ...
    def write_force_set(self, values: list[int]) -> None: ...
    def write_speed_set(self, values: list[int]) -> None: ...
    def clear_error(self) -> None: ...
    def save_flash(self) -> None: ...


class HardwareSource(Source):
    name = "实机"

    def __init__(self, host: str, port: int, timeout: float = 1.0):
        self.link = HandLink(host, port, timeout)

    def open(self): self.link.connect()
    def close(self): self.link.close()
    def read_state(self): return self.link.read_state()
    def read_touch(self): return self.link.read_touch()
    def write_angle_set(self, v): self.link.write_angle_set(v)
    def write_force_set(self, v): self.link.write_force_set(v)
    def write_speed_set(self, v): self.link.write_speed_set(v)
    def clear_error(self): self.link.clear_error()
    def save_flash(self): self.link.save_flash()


class MockSource(Source):
    """脱机模拟：状态缓慢摆动，触觉有一个在手上游走的压痕。"""

    name = "模拟"

    def __init__(self):
        self._t0 = time.time()
        self._angle = [500.0] * N_FINGERS
        self._target = [500.0] * N_FINGERS
        self._force = [0.0] * N_FINGERS
        self._speed = [1000] * N_FINGERS
        self._force_th = [1000] * N_FINGERS

    def open(self): self._t0 = time.time()
    def close(self): pass

    def _t(self) -> float:
        return time.time() - self._t0

    def read_state(self) -> dict:
        t = self._t()
        for i in range(N_FINGERS):
            self._angle[i] += (self._target[i] - self._angle[i]) * 0.15
            self._force[i] = max(0.0, 220 * math.sin(2 * math.pi * (t / 4 + i / 6)))
        return {
            "t": time.time(),
            "angle": [int(a) for a in self._angle],
            "force": [int(f) for f in self._force],
            "current": [int(120 + 60 * abs(math.sin(t / 3 + i))) for i in range(N_FINGERS)],
            "err": [0] * N_FINGERS,
            "status": [2] * N_FINGERS,
            "temp": [int(34 + 3 * math.sin(t / 10 + i)) for i in range(N_FINGERS)],
        }

    def read_touch(self) -> dict[str, list[int]]:
        t = self._t()
        out: dict[str, list[int]] = {}
        for zi, (name, start, end) in enumerate(TOUCH_ZONES):
            n = end - start + 1
            # 每 6 秒在下一个部位出现压痕
            phase = (t / 6.0) % len(TOUCH_ZONES)
            act = phase if name != "掌心" else phase
            amp = max(0.0, 1.0 - min(1.0, abs(zi - act) / 0.6))
            vals = []
            for k in range(n):
                # 用地址做伪随机底噪，中心位置给一个高斯压痕
                base = 8 + 6 * math.sin(k * 0.7 + zi)
                d = (k / n) - 0.45
                blob = 2600 * amp * math.exp(-(d * d) / 0.008)
                vals.append(int(max(0, min(4095, base + blob))))
            out[name] = vals
        return out

    def write_angle_set(self, values):
        for i, v in enumerate(values[:N_FINGERS]):
            if v >= 0:
                self._target[i] = float(v)

    def write_force_set(self, values):
        self._force_th = list(values[:N_FINGERS])

    def write_speed_set(self, values):
        self._speed = list(values[:N_FINGERS])

    def clear_error(self): pass
    def save_flash(self): pass


def make_source(mock: bool, host: str, port: int) -> Source:
    return MockSource() if mock else HardwareSource(host, port)


__all__ = ["Source", "HardwareSource", "MockSource", "make_source", "FINGERS"]
