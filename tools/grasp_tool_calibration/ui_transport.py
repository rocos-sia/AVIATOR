"""Owned subprocesses and the calibration UI's hand/camera bus connection.

Importing this module does not start processes or connect to hardware. ZMQ sockets
are created, used and closed by one worker thread; HTTP threads only change targets.
"""
from __future__ import annotations

from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
import uuid


def _mono_us():
    return time.monotonic_ns() // 1000


def _clock_id():
    return socket.gethostname()[:80] + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()


class ManagedNodes:
    """Own process groups, retaining bounded UI log tails and full disk logs."""

    def __init__(self, root: Path, log_dir: Path, log):
        self.root, self.log_dir, self.log = Path(root), Path(log_dir), log
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._nodes = {}
        self._lock = threading.RLock()

    def start(self, name, argv, ready_text=None, timeout=10):
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", name):
            raise ValueError("invalid node name")
        if not argv or not all(isinstance(x, str) and x for x in argv):
            raise ValueError("node command must be a nonempty argument list")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("startup timeout must be positive")
        # A successful bind is necessary: never silently attach to someone else's bus.
        if Path(argv[0]).name == "aviator_bus" and ready_text is None:
            ready_text = "READY input="
        with self._lock:
            previous = self._nodes.get(name)
            if previous and previous["process"].poll() is None:
                raise RuntimeError(f"{name} is already running")
            log_path = self.log_dir / f"{name}-{time.time_ns()}.log"
            output = log_path.open("w", encoding="utf-8")
            try:
                process = subprocess.Popen(argv, cwd=self.root, stdin=subprocess.DEVNULL,
                                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, encoding="utf-8", errors="replace",
                                           start_new_session=True, bufsize=1)
            except BaseException:
                output.close()
                raise
            node = dict(process=process, log_path=str(log_path), tail=deque(maxlen=120),
                        ready=threading.Event(), reader=None, output=output)
            self._nodes[name] = node

            def read_output():
                try:
                    # Bounded reads prevent a malformed no-newline message using arbitrary RAM.
                    while True:
                        line = process.stdout.readline(8192)
                        if not line:
                            break
                        output.write(line)
                        output.flush()
                        with self._lock:
                            node["tail"].append(line.rstrip())
                        if ready_text and ready_text in line:
                            node["ready"].set()
                finally:
                    output.close()
                    process.stdout.close()

            reader = threading.Thread(target=read_output, name=f"calibration-log-{name}", daemon=True)
            node["reader"] = reader
            reader.start()
        try:
            start = time.monotonic()
            while True:
                code = process.poll()
                if code is not None:
                    reader.join(timeout=.2)
                    raise RuntimeError(f"{name} exited ({code}): " + " | ".join(node["tail"])[-3000:])
                if ready_text is None and time.monotonic() - start >= .15:
                    break
                if ready_text is not None and node["ready"].is_set():
                    break
                if time.monotonic() - start >= timeout:
                    raise RuntimeError(f"{name} startup timed out waiting for {ready_text!r}: "
                                       + " | ".join(node["tail"])[-3000:])
                node["ready"].wait(.02)
            self.log(f"{name} started (PID {process.pid}); log: {log_path}")
        except BaseException:
            self.stop(name)
            raise

    def stop(self, name):
        with self._lock:
            node = self._nodes.get(name)
        if node is None:
            return
        process = node["process"]
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2)
        if node["reader"]:
            node["reader"].join(timeout=1)
        self.log(f"{name} stopped (exit {process.returncode})")

    def stop_all(self):
        with self._lock:
            names = list(self._nodes)
        errors = []
        for name in reversed(names):
            try:
                self.stop(name)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def snapshot(self):
        with self._lock:
            return [dict(name=name, pid=node["process"].pid,
                         running=node["process"].poll() is None,
                         returncode=node["process"].returncode, owned=True,
                         log_path=node["log_path"], tail=list(node["tail"]))
                    for name, node in self._nodes.items()]


def _conflicting_command(argv):
    """Inspect the executable/script, never unrelated test command arguments."""
    if not argv:
        return False
    executable = Path(argv[0]).name
    if executable.startswith("python"):
        script = None
        skip = False
        for arg in argv[1:]:
            if skip:
                skip = False
                continue
            if arg in ("-c", "-m"):
                return False
            if arg in ("-W", "-X"):
                skip = True
                continue
            if not arg.startswith("-"):
                script = Path(arg)
                break
        if script is None:
            return False
        if script.name in ("rh56ftp_node.py", "fake_rh56ftp_hand.py", "hand_command.py"):
            return "--feedback-only" not in argv and "--dry-run" not in argv
        return (script.parent.name == "grasp_tool_calibration"
                and script.name in ("ui.py", "ui_server.py", "calibration_ui.py", "calibrate.py")
                and "--dry-run" not in argv)
    return (executable.startswith("aviator_core")
            or executable in ("manipulator", "aviator_manipulator", "aviator_grasp_calibration",
                              "aviator_grasp_calibration_session", "aviator_grasp_tool_session")
            or executable.startswith("change_stiffness")) and "--dry-run" not in argv


def check_conflicting_processes():
    """Best-effort same-user local scan; not a remote robot ownership guarantee."""
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            argv = [v.decode("utf-8", "replace") for v in (entry / "cmdline").read_bytes().split(b"\0") if v]
            if _conflicting_command(argv):
                result.append(f"PID {entry.name}: " + " ".join(argv)[:1000])
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return result


class HandCameraLink:
    """Continuously publish held targets; expose measured feedback and precise ACKs.

    ``ready`` means fresh measurements and accepted current targets. It does not
    mean physical contact or a successful grasp. Context injection is for inproc tests.
    """

    def __init__(self, publish_endpoint, subscribe_endpoint, open_targets, close_targets, log,
                 camera_id="cockpit", hand_publisher="rh56ftp_hand", *, context=None,
                 feedback_timeout_ms=500, ack_timeout_ms=200):
        self.publish_endpoint, self.subscribe_endpoint = publish_endpoint, subscribe_endpoint
        self.camera_id, self.hand_publisher, self.log = camera_id, hand_publisher, log
        self._presets = {"open": self._targets(open_targets), "close": self._targets(close_targets)}
        self._targets_now = copy.deepcopy(self._presets["open"])
        self._poses = dict(left="open", right="open")
        self.publisher_id = "grasp_calibration_ui"
        self.session_id, self.control_epoch = str(uuid.uuid4()), str(uuid.uuid4())
        self.clock_id = _clock_id()
        self._context = context
        if feedback_timeout_ms <= 0 or ack_timeout_ms <= 0:
            raise ValueError("feedback and ACK timeouts must be positive")
        self._feedback_us, self._ack_us = int(feedback_timeout_ms * 1000), int(ack_timeout_ms * 1000)
        self._lock = threading.RLock()
        self._stop, self._started, self._changed = threading.Event(), threading.Event(), threading.Event()
        self._thread = None
        self._sequence, self._target_sequence = 0, 1
        self._sent = {}
        self._state = None
        self._state_session, self._state_sequence = None, 0
        self._state_sample = 0
        self._camera, self._camera_received = None, None
        self._error, self._conflict = None, None

    @staticmethod
    def _targets(value):
        result = {}
        for side in ("left", "right"):
            values = value.get(side) if isinstance(value, dict) else None
            if not isinstance(values, (list, tuple)) or len(values) != 6 or any(
                    isinstance(x, bool) or not isinstance(x, (float, int))
                    or not math.isfinite(x) or not 0 <= x <= 1 for x in values):
                raise ValueError(f"{side} hand preset must contain six finite values in [0,1]")
            result[side] = list(values)
        return result

    def start(self):
        if self._thread is not None:
            raise RuntimeError("HandCameraLink cannot be restarted; create a new session")
        self._thread = threading.Thread(target=self._run, name="calibration-hand-camera", daemon=True)
        self._thread.start()
        if not self._started.wait(3):
            self.stop()
            raise RuntimeError("hand/camera bus thread did not start")
        if self._error:
            raise RuntimeError(self._error)

    def set_hand(self, side, pose):
        if side not in ("both", "left", "right") or pose not in self._presets:
            raise ValueError("hand command must be left/right/both and open/close")
        with self._lock:
            if not self._thread or not self._thread.is_alive() or self._stop.is_set():
                raise RuntimeError("hand connection is not running")
            if self._conflict or self._error:
                raise RuntimeError(self._conflict or self._error)
            for name in (("left", "right") if side == "both" else (side,)):
                self._targets_now[name] = list(self._presets[pose][name])
                self._poses[name] = pose
            self._target_sequence = self._sequence + 1
        self._changed.set()

    def _message(self, targets):
        with self._lock:
            self._sequence += 1
            seq, stamp = self._sequence, _mono_us()
            self._sent[seq] = stamp
            while len(self._sent) > 256:
                del self._sent[next(iter(self._sent))]
            value = dict(msg_type="HandCommand", version="1.0", sequence=seq,
                         timestamp=time.time_ns() // 1000, sample_mono_us=stamp,
                         clock_id=self.clock_id, publisher_id=self.publisher_id,
                         session_id=self.session_id, control_epoch=self.control_epoch,
                         valid=targets is not None, mode="NORMALIZED_POSITION")
            value["origin"] = {key: value[key] for key in (
                "publisher_id", "session_id", "sequence", "sample_mono_us", "clock_id")}
            if targets is not None:
                value["hands"] = {s: dict(drive_position_normalized=list(targets[s])) for s in ("left", "right")}
            return value

    def _accept_hand(self, data, now):
        if (data.get("msg_type") != "HandState" or data.get("publisher_id") != self.hand_publisher
                or data.get("clock_id") != self.clock_id):
            return
        seq, stamp, session = data.get("sequence"), data.get("sample_mono_us"), data.get("session_id")
        if (not isinstance(session, str) or not session or type(seq) is not int or seq <= 0
                or type(stamp) is not int or not 0 <= now - stamp < self._feedback_us
                or not isinstance(data.get("valid"), bool)):
            return
        with self._lock:
            if self._state_session and self._state_session != session:
                self._conflict = "手节点会话已变化，请结束当前校准并重新启动环境"
                return
            if seq <= self._state_sequence or stamp < self._state_sample:
                return
            self._state_session, self._state_sequence, self._state_sample = session, seq, stamp
            self._state = data
            ack = data.get("accepted_command")
            if isinstance(ack, dict) and ack and any(ack.get(key) != getattr(self, key) for key in (
                    "publisher_id", "session_id", "control_epoch")):
                self._conflict = "手节点已绑定其他发布者或会话；请先停止原控制程序并重启手节点"

    def snapshot(self):
        now = _mono_us()
        with self._lock:
            state = self._state or {}
            hands = state.get("hands") if isinstance(state.get("hands"), dict) else {}
            sample = state.get("sample_mono_us", 0)
            fresh = bool(state.get("valid") is True and 0 <= now - sample < self._feedback_us)
            fault = self._conflict or self._error or state.get("hold_control_error") or None
            if state.get("feedback_only"):
                fault = "手节点处于 feedback-only，只读模式不能控制"
            for side in ("left", "right"):
                hand = hands.get(side, {})
                if not isinstance(hand, dict):
                    hand = {}
                stamp = hand.get("sample_mono_us")
                try:
                    self._targets({"left": hand.get("drive_position_normalized"),
                                   "right": hand.get("drive_position_normalized")})
                    positions_valid = True
                except ValueError:
                    positions_valid = False
                fresh &= (hand.get("valid") is True and hand.get("feedback_available") is True
                          and positions_valid and type(stamp) is int
                          and 0 <= now - stamp < self._feedback_us)
                protection = hand.get("fault_protection") or {}
                error_codes = hand.get("error_codes")
                if error_codes is not None and not isinstance(error_codes, list):
                    fresh = False
                    error_codes = []
                if (hand.get("status") == "ERROR" or hand.get("error_code")
                        or any(error_codes or [])
                        or (isinstance(protection, dict) and protection.get("fault_count"))):
                    fault = fault or f"{side} 机械手故障"
            ack = state.get("accepted_command") or {}
            if not isinstance(ack, dict):
                ack = {}
            ack_seq, ack_stamp = ack.get("sequence"), ack.get("sample_mono_us")
            ack_valid = bool(type(ack_seq) is int and type(ack_stamp) is int
                             and ack_seq >= self._target_sequence
                             and self._sent.get(ack_seq) == ack_stamp
                             and 0 <= now - ack_stamp < self._ack_us
                             and state.get("command_valid") is True
                             and all(ack.get(key) == getattr(self, key) for key in (
                                 "publisher_id", "session_id", "control_epoch")))
            running = bool(self._thread and self._thread.is_alive() and not self._stop.is_set())
            return copy.deepcopy(dict(
                running=running, ready=bool(running and fresh and ack_valid and not fault),
                error=self._error, conflict=self._conflict, fault=fault, fresh=bool(fresh),
                publisher_id=self.publisher_id, session_id=self.session_id,
                control_epoch=self.control_epoch, clock_id=self.clock_id, sequence=self._sequence,
                targets=self._targets_now, poses=self._poses, hands=hands, state=self._state,
                ack=dict(valid=ack_valid, sequence=ack_seq, sample_mono_us=ack_stamp,
                         age_ms=(now - ack_stamp) / 1000 if type(ack_stamp) is int else None),
                camera=dict(raw=self._camera, received_mono_us=self._camera_received)))

    def wait_ready(self, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.snapshot()
            if status["ready"]:
                return status
            if status["fault"]:
                raise RuntimeError(status["fault"])
            if not status["running"]:
                raise RuntimeError("hand/camera link stopped before receiving feedback")
            self._changed.wait(.02)
            self._changed.clear()
        raise RuntimeError("等待手节点超时：需要双手新鲜有效反馈及本会话指令 ACK；ACK 不代表已握住把手")

    def stop(self):
        self._stop.set()
        self._changed.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                raise RuntimeError("hand/camera thread did not stop")

    def _run(self):
        context = pub = sub = None
        sent_any = False
        try:
            import zmq
            context = self._context if self._context is not None else zmq.Context()
            pub, sub = context.socket(zmq.PUB), context.socket(zmq.SUB)
            pub.setsockopt(zmq.SNDHWM, 5)
            sub.setsockopt(zmq.RCVHWM, 100)
            sub.setsockopt(zmq.SUBSCRIBE, b"hand.state")
            sub.setsockopt(zmq.SUBSCRIBE, b"camera.detection")
            pub.connect(self.publish_endpoint)
            sub.connect(self.subscribe_endpoint)
            self._started.set()
            next_send = 0.0
            while not self._stop.is_set():
                # Bound receive work so camera traffic cannot starve hand keepalives.
                for _ in range(32):
                    if not sub.poll(0):
                        break
                    parts = sub.recv_multipart()
                    if len(parts) != 2 or len(parts[1]) > 1024 * 1024:
                        continue
                    try:
                        data = json.loads(parts[1])
                    except (ValueError, UnicodeError):
                        continue
                    if not isinstance(data, dict):
                        continue
                    if parts[0] == b"hand.state":
                        self._accept_hand(data, _mono_us())
                    elif (parts[0] == b"camera.detection" and data.get("camera_id") == self.camera_id
                          and data.get("msg_type") == "CameraDetection"):
                        with self._lock:
                            self._camera, self._camera_received = data, _mono_us()
                    self._changed.set()
                if self._conflict:
                    self.log(self._conflict)
                    break
                if time.monotonic() >= next_send:
                    with self._lock:
                        # Snapshot and assign the sent sequence atomically with set_hand().
                        message = self._message(self._targets_now)
                    pub.send_multipart([b"hand.command", json.dumps(message, allow_nan=False).encode()], zmq.NOBLOCK)
                    sent_any = True
                    next_send = time.monotonic() + .02
                self._stop.wait(.002)
        except Exception as exc:
            with self._lock:
                self._error = f"手/相机通信失败: {type(exc).__name__}: {exc}"
            self.log(self._error)
        finally:
            self._started.set()
            # Only our accepted session may release. A foreign ACK never triggers
            # an open/invalidation command against somebody else's hand controller.
            if pub is not None and sent_any and not self._conflict:
                try:
                    for targets in [self._presets["open"]] * 3 + [None] * 3:
                        pub.send_multipart([b"hand.command", json.dumps(self._message(targets)).encode()], zmq.NOBLOCK)
                        time.sleep(.02)
                except Exception as exc:
                    self.log(f"发送松手/退出指令失败: {exc}")
            if pub is not None:
                pub.close(linger=0)
            if sub is not None:
                sub.close(linger=0)
            if context is not None and self._context is None:
                context.term()
            self._changed.set()
