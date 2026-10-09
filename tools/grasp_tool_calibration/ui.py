#!/usr/bin/env python3
"""本机静态抓握校准 UI；点击准备环境后才连接设备。"""
from __future__ import annotations

import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import sys
import tempfile
import threading
import time
from urllib.parse import urlsplit
import webbrowser

from calibrate import ROOT
from calibration import CalibrationError
from ui_backend import CalibrationController

WEB = Path(__file__).resolve().parent / "web"


def python_default(env, candidate):
    return Path(os.environ.get(env, str(candidate if candidate.is_file() else Path(sys.executable)))).expanduser().resolve()


def prepare_log_directory(requested):
    """Keep an explicit path strict; a default path must not require sudo."""
    directory = (requested if requested is not None else
                 ROOT / "logs/grasp_calibration_ui" / f"{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}")
    directory = Path(directory).expanduser().resolve()
    try:
        directory.mkdir(parents=True, exist_ok=True)
        # mkdir(exist_ok=True) succeeds for an existing read-only directory.
        # Check that node logs and capture reports can actually be created.
        with tempfile.TemporaryFile(dir=directory):
            pass
    except OSError as exc:
        if requested is not None:
            raise CalibrationError(f"无法写入指定日志目录 {directory}：{exc}。"
                                   "请用 --log-dir 指定可写目录，或省略此参数以自动选择。") from exc
        fallback = Path(tempfile.mkdtemp(prefix="aviator-grasp-calibration-ui-"))
        print(f"默认日志目录不可写：{directory}（{exc}）；已改用 {fallback}", file=sys.stderr, flush=True)
        return fallback
    return directory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="仅模拟设备；禁止写回 grasp.json")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--port", type=int, default=8766, help="仅绑定 127.0.0.1；0 由系统分配")
    parser.add_argument("--robot-config", type=Path, default=ROOT / "config/robot.yaml")
    parser.add_argument("--system-config", type=Path, default=ROOT / "config/system.yaml")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/camera.yaml")
    parser.add_argument("--hand-config", type=Path, default=ROOT / "config/rh56ftp_hand.yaml")
    binary_dir = Path(os.environ.get("AVIATOR_BIN", ROOT / "build/bin"))
    parser.add_argument("--session-helper", type=Path, default=binary_dir / "aviator_grasp_tool_session")
    parser.add_argument("--bus-helper", type=Path, default=binary_dir / "aviator_bus")
    conda = Path(os.environ.get("CONDA_ROOT", str(Path.home() / "miniconda3")))
    parser.add_argument("--hand-python", type=Path,
                        default=python_default("HAND_PYTHON", conda / "envs/rh56-pendant/bin/python3"))
    parser.add_argument("--camera-python", type=Path,
                        default=python_default("CAMERA_PYTHON", conda / "envs/apriltag_realsense/bin/python"))
    parser.add_argument("--camera-id", default="cockpit")
    parser.add_argument("--log-dir", type=Path,
                        help="指定日志目录；默认优先使用仓库 logs，无法写入时自动使用临时目录")
    parser.add_argument("--lease-timeout", type=float, default=10, help="浏览器失联清理等待秒数（3..60）")
    options = parser.parse_args(argv)
    if not 0 <= options.port <= 65535 or not 3 <= options.lease_timeout <= 60:
        parser.error("port 必须为 0..65535，lease-timeout 必须为 3..60")
    for name, value in vars(options).items():
        if isinstance(value, Path):
            setattr(options, name, value.expanduser().resolve())
    return options


class Handler(BaseHTTPRequestHandler):
    server_version = "GraspCalibration/1"

    def log_message(self, *_):
        pass

    def _reply(self, code, data, content_type="application/json; charset=utf-8"):
        if isinstance(data, (dict, list)):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        elif isinstance(data, str):
            data = data.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; "
                         "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _local_request(self):
        port = self.server.server_port
        if self.headers.get("Host") != f"127.0.0.1:{port}":
            self._reply(403, {"error": "请使用启动时显示的 127.0.0.1 地址"})
            return False
        origin = self.headers.get("Origin")
        if origin and origin != f"http://127.0.0.1:{port}":
            self._reply(403, {"error": "不接受跨站请求"})
            return False
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            self._reply(403, {"error": "不接受跨站请求"})
            return False
        return True

    def _authorized(self, body=None):
        supplied = self.headers.get("X-Calibration-Token")
        if supplied is None and isinstance(body, dict) and body.get("action") == "stop":
            supplied = body.get("token")  # pagehide sendBeacon cannot set a header.
        if not isinstance(supplied, str) or not secrets.compare_digest(supplied, self.server.token):
            self._reply(403, {"error": "会话令牌无效，请重新打开本机界面"})
            return False
        return True

    def do_GET(self):
        if not self._local_request():
            return
        path = urlsplit(self.path).path
        if path == "/api/state":
            if self._authorized():
                self.server.controller.heartbeat()
                self._reply(200, self.server.controller.snapshot())
            return
        assets = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/index.html": ("index.html", "text/html; charset=utf-8"),
                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8")}
        if path not in assets:
            self._reply(404, {"error": "Not found"})
            return
        name, kind = assets[path]
        content = (WEB / name).read_text(encoding="utf-8")
        if name == "index.html":
            content = content.replace("__CALIBRATION_TOKEN__", self.server.token)
        self._reply(200, content, kind)

    def do_POST(self):
        if not self._local_request():
            return
        if urlsplit(self.path).path != "/api/action":
            self._reply(404, {"error": "Not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("请求大小不合法")
            self.connection.settimeout(3)
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("请求必须是 JSON 对象")
        except (ValueError, UnicodeError, TimeoutError) as exc:
            self._reply(400, {"error": str(exc)})
            return
        if not self._authorized(body):
            return
        try:
            self.server.controller.submit(body)
        except (CalibrationError, ValueError) as exc:
            self._reply(409, {"error": str(exc)})
            return
        self._reply(202, {"accepted": True})


def main(argv=None):
    options = parse_args(argv)
    controller = server = None
    try:
        # Bind before constructing the controller, so a port collision never
        # creates a second controller or touches devices.
        server = ThreadingHTTPServer(("127.0.0.1", options.port), Handler)
        server.daemon_threads = True
        options.log_dir = prepare_log_directory(options.log_dir)
        controller = CalibrationController(options)
        server.controller, server.token = controller, secrets.token_urlsafe(32)
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"抓握校准 UI：{url}" + ("（模拟模式）" if options.dry_run else ""), flush=True)
        print(f"日志目录：{options.log_dir}", flush=True)
        print("点击准备环境后才连接设备；结束会话/关闭页面会请求结束拖动并张开双手。", flush=True)
        stop = threading.Event()
        def interrupt(*_):
            stop.set()
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        server.timeout = .25
        if not options.no_browser:
            threading.Thread(target=partial(webbrowser.open, url), daemon=True).start()
        while not stop.is_set():
            server.handle_request()
        return 0
    except (OSError, CalibrationError, ValueError) as exc:
        print(f"校准 UI 启动失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if controller:
            print("正在结束设备会话…", flush=True)
            controller.close()
        if server:
            server.server_close()


if __name__ == "__main__":
    raise SystemExit(main())
