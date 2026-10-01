"""示教器入口。"""
from __future__ import annotations

import argparse
import sys

from PyQt5.QtWidgets import QApplication

from .handlink import HAND_IP, HAND_PORT
from .poller import Poller
from .source import make_source
from .ui import MainWindow


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="RH56E2 灵巧手示教器")
    ap.add_argument("--mock", action="store_true", help="使用模拟数据源，不连硬件")
    ap.add_argument("--host", default=HAND_IP)
    ap.add_argument("--port", type=int, default=HAND_PORT)
    ap.add_argument("--state-hz", type=float, default=20.0, help="状态轮询频率")
    ap.add_argument("--touch-hz", type=float, default=10.0, help="触觉轮询频率")
    args = ap.parse_args(argv)

    app = QApplication(sys.argv[:1])
    source = make_source(args.mock, args.host, args.port)
    win = MainWindow(source)

    try:
        source.open()
        win.set_connected(True)
    except Exception as e:
        win.set_connected(False)
        win.statusBar().showMessage(f"打开 {args.host}:{args.port} 失败: {e}")

    win.show()

    poller = Poller(source, args.state_hz, args.touch_hz)
    poller.stateReady.connect(win.on_state)
    poller.touchReady.connect(win.on_touch)
    poller.failed.connect(win.on_failed)
    win.poller = poller
    poller.start()

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
