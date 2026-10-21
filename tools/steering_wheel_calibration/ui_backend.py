"""Sampling and owned Bus/Camera session for the native calibration window."""
from __future__ import annotations

import fcntl
import json
import math
import os
from pathlib import Path
import queue
import shutil
import tempfile
import threading
import time

import yaml
import zmq

from tools.grasp_tool_calibration.ui_transport import ManagedNodes
from tools.steering_wheel_calibration.calibrate import (
    build_calibration, load_document, valid_detection, write_document)
from tools.steering_wheel_calibration.geometry import (
    CalibrationError, average_pose_matrices, pose_to_matrix)
from nodes.camera.steering_wheel import SteeringWheelObservation

ROOT = Path(__file__).resolve().parents[2]


def mono_us():
    return time.monotonic_ns() // 1000


class CalibrationSession:
    """Only fresh, post-click samples from this camera process may enter a point."""

    def __init__(self, args, session_id):
        self.args, self.session_id = args, session_id
        self.observer = SteeringWheelObservation(args.output)
        self.zero = None
        self.pending = None
        self.records, self.matrices = [], []
        self.latest = None
        self.last_sample = -1
        self.message = "等待相机检测 AprilTag"
        self.first_text = "未采集"
        self.second_text = "未采集"

    def fresh(self, now):
        return self.latest is not None and 0 <= now - self.latest["sample_mono_us"] <= self.args.stale_ms * 1000

    def begin(self, point, now):
        if self.pending:
            raise CalibrationError("正在采集，请保持稳定或点击取消采样")
        if point not in (1, 2) or (point == 2 and self.zero is None):
            raise CalibrationError("请先采集第一个点（零位）")
        if not self.fresh(now):
            raise CalibrationError("没有新鲜有效的检测，请让 AprilTag 保持可见")
        if point == 1:
            self.zero = None
            self.first_text = self.second_text = "未采集"
        self.pending, self.started = point, now
        self.records, self.matrices = [], []
        self.last_record = now - self.args.sample_interval_ms * 1000
        self.message = f"正在采集第 {point} 个点，请保持方向盘静止"

    def cancel(self):
        self.pending = None
        self.records, self.matrices = [], []
        self.message = "采样已取消，可重新采集"

    def ingest(self, data, now):
        if not isinstance(data, dict) or data.get("session_id") != self.session_id:
            return
        if data.get("camera_id") != self.args.camera_id:
            return
        sample = data.get("sample_mono_us")
        if (isinstance(sample, bool) or not isinstance(sample, int)
                or not 0 <= now - sample <= self.args.stale_ms * 1000
                or sample <= self.last_sample):
            return
        self.last_sample = sample
        if not valid_detection(data, self.args):
            self.latest = None
            return
        try:
            matrix = pose_to_matrix(data["pose"])
        except (ValueError, TypeError, KeyError):
            self.latest = None
            return
        self.latest = data
        if (not self.pending or sample < self.started
                or sample < self.last_record + self.args.sample_interval_ms * 1000):
            return
        self.records.append(data)
        self.matrices.append(matrix)
        self.last_record = sample
        if len(self.records) < self.args.samples:
            return
        point, self.pending = self.pending, None
        averaged = average_pose_matrices(self.matrices)
        if point == 1:
            self.zero = (list(self.records), averaged)
            self.first_text = f"已采集 {len(self.records)} 帧（零位）"
            self.message = "请转动方向盘到第二个位置，保持稳定后采集第二点"
            return
        payload = build_calibration(self.args, *self.zero, self.records, averaged)
        document = load_document(self.args.output)
        document["steering_wheel_calibration"] = payload
        backup = None
        if self.args.output.exists():
            backup = self.args.output.with_name(self.args.output.name + f".bak.{time.time_ns()}")
            shutil.copy2(self.args.output, backup)
        write_document(self.args.output, document)
        # Use precisely the camera's geometry/loader; reload immediately in this UI.
        self.observer = SteeringWheelObservation(self.args.output)
        self.second_text = f"已采集 {len(self.records)} 帧，标定已保存"
        self.message = f"已保存：{self.args.output}"
        if backup:
            self.message += f"\n原文件备份：{backup}"

    def snapshot(self, now):
        if self.pending and now - self.started > self.args.capture_timeout_s * 1e6:
            self.cancel()
            self.message = "采样超时，未保存；请检查标签可见性并重新采集"
        fresh = self.fresh(now)
        observation = self.observer.observe(self.latest) if fresh else {}
        valid = fresh and observation.get("valid", False)
        return dict(message=self.message, fresh=fresh, pending=self.pending,
                    count=len(self.records), first=self.first_text, second=self.second_text,
                    has_zero=self.zero is not None,
                    theta_deg=math.degrees(observation["theta_rad"]) if valid else None,
                    depth_mm=observation["translation_along_axis_m"] * 1000 if valid else None,
                    axis_match=observation.get("axis_match"),
                    reason=observation.get("reason", "等待检测 / 数据已过期"),
                    calibration_id=observation.get("calibration_id"),
                    confidence=float(self.latest.get("confidence", 0)) if fresh else None)


def commands(args, session_id):
    config = load_document(args.system_config)
    bus = config.get("bus", {})
    publish = bus.get("publish", "tcp://127.0.0.1:5555")
    subscribe = bus.get("subscribe", "tcp://127.0.0.1:5556")
    camera = [str(args.camera_python), "-u", str(ROOT / "nodes/camera/main.py"),
              "--config", str(args.camera_config), "--recording-config", str(args.recording_config),
              "--camera-id", args.camera_id, "--session", session_id,
              "--endpoint", publish, "--steering-wheel-calibration", str(args.output),
              "--show" if args.show else "--no-show", "--print-pose"]
    return [("bus", [str(args.bus_bin), "--config", str(args.system_config)]),
            ("camera", camera)], subscribe


class Worker(threading.Thread):
    """All ZMQ and process operations run off the Tk thread."""

    def __init__(self, args, session_id):
        super().__init__(daemon=False)
        self.args, self.session_id = args, session_id
        self.actions = queue.Queue()
        self.stop_requested = threading.Event()
        self.lock = threading.Lock()
        self.state = dict(running=False, message="正在启动 Bus 和相机…")

    def publish(self, **state):
        with self.lock:
            self.state = state

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def run(self):
        nodes = context = sub = launch_lock = None
        log_dir = None
        failure = None
        try:
            argv, endpoint = commands(self.args, self.session_id)
            lock_path = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / f"aviator-start-{os.getuid()}.lock"
            launch_lock = lock_path.open("a")
            try:
                fcntl.flock(launch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("已有 AVIATOR 启动脚本运行，请先停止后再启动标定") from exc
            log_dir = Path(tempfile.mkdtemp(prefix="aviator-wheel-calibration-"))
            (log_dir / "session.uuid").write_text(self.session_id + "\n")
            nodes = ManagedNodes(ROOT, log_dir, lambda line: print(line, flush=True))
            context = zmq.Context()
            sub = context.socket(zmq.SUB)
            sub.setsockopt(zmq.LINGER, 0)
            sub.setsockopt(zmq.RCVHWM, 32)
            sub.setsockopt(zmq.MAXMSGSIZE, 65536)
            sub.setsockopt(zmq.SUBSCRIBE, self.args.topic.encode())
            sub.connect(endpoint)
            for name, command in argv:
                if self.stop_requested.is_set():
                    return
                nodes.start(name, command)
            session = CalibrationSession(self.args, self.session_id)
            while not self.stop_requested.is_set():
                for node in nodes.snapshot():
                    if not node["running"]:
                        raise RuntimeError(f"{node['name']} 已退出 ({node['returncode']})\n" + "\n".join(node["tail"][-8:]))
                try:
                    action = self.actions.get_nowait()
                    if action == "cancel":
                        session.cancel()
                    else:
                        session.begin(action, mono_us())
                except queue.Empty:
                    pass
                except CalibrationError as exc:
                    session.message = str(exc)
                if sub.poll(30):
                    # Bound each batch so a fast publisher cannot starve stop/clicks.
                    for _ in range(32):
                        try:
                            frames = sub.recv_multipart(zmq.NOBLOCK)
                        except zmq.Again:
                            break
                        try:
                            if len(frames) == 2 and frames[0] == self.args.topic.encode():
                                session.ingest(json.loads(frames[1]), mono_us())
                        except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
                            session.pending = None
                            session.message = f"采样 / 保存失败：{exc}"
                self.publish(**session.snapshot(mono_us()), running=True, log_dir=str(log_dir))
        except Exception as exc:
            failure = str(exc)
        finally:
            self.publish(running=False, message="正在停止相机和 Bus…", log_dir=str(log_dir or ""))
            try:
                if nodes:
                    nodes.stop_all()
            except Exception as exc:
                failure = f"{failure or ''}\n清理失败：{exc}"
            finally:
                if sub is not None:
                    sub.close(0)
                if context is not None:
                    context.term()
                if launch_lock is not None:
                    launch_lock.close()
            self.publish(running=False, message=failure or "已停止相机和 Bus，可关闭窗口",
                         log_dir=str(log_dir or ""), error=bool(failure))
