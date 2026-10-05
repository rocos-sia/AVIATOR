"""后台轮询线程。

状态和触觉用两个独立周期：状态轻（约 3ms/轮）可以跑快些，
触觉重（约 20ms/轮）跑慢些，两者互不阻塞界面。
"""
from __future__ import annotations

import time
from collections import deque

from PyQt5.QtCore import QThread, pyqtSignal

from .source import Source


class Poller(QThread):
    stateReady = pyqtSignal(dict)
    touchReady = pyqtSignal(dict)
    failed = pyqtSignal(str)

    def __init__(self, source: Source, state_hz: float = 20.0, touch_hz: float = 10.0,
                 parent=None):
        super().__init__(parent)
        self.source = source
        self.state_hz = state_hz
        self.touch_hz = touch_hz
        self._stop = False
        self._t_state = time.monotonic()
        self._t_touch = time.monotonic()

    def stop(self) -> None:
        self._stop = True
        self.wait(2000)

    def run(self) -> None:
        self._stop = False
        stamps: dict[str, deque[float]] = {"state": deque(), "touch": deque()}

        def rate(kind: str) -> float:
            q = stamps[kind]
            now = time.monotonic()
            q.append(now)
            while q and now - q[0] > 2.0:   # 2 秒滑动窗口
                q.popleft()
            span = q[-1] - q[0]
            return (len(q) - 1) / span if span > 0.2 else 0.0

        # 截止时刻按固定节拍推进，而不是「读完再 +周期」——
        # 否则实际周期会变成 读取耗时 + 周期（触觉 35ms 读一次就掉到 7Hz）。
        while not self._stop:
            now = time.monotonic()

            if now >= self._t_state:
                self._t_state += 1.0 / self.state_hz
                if self._t_state <= now:      # 落后太多则重排，避免追赶风暴
                    self._t_state = now + 1.0 / self.state_hz
                try:
                    d = self.source.read_state()
                    d["hz"] = round(rate("state"), 1)
                    self.stateReady.emit(d)
                except Exception as e:        # 单次失败不杀线程
                    self.failed.emit(str(e))

            if now >= self._t_touch:
                self._t_touch += 1.0 / self.touch_hz
                if self._t_touch <= now:
                    self._t_touch = now + 1.0 / self.touch_hz
                try:
                    d = self.source.read_touch()
                    d["hz"] = round(rate("touch"), 1)
                    self.touchReady.emit(d)
                except Exception as e:
                    self.failed.emit(str(e))

            self.msleep(2)
