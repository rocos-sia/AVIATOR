"""Calibration session coordinator. No hardware is touched until prepare()."""
from __future__ import annotations

from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

import yaml

from calibrate import (ROOT, CameraTracker, local_clock_id, mono_us, pair_samples,
                       parse_args as calibration_args, read_inputs, validate_state, write_report)
from calibration import CalibrationError, calibrate, save_grasp
from ui_transport import HandCameraLink, ManagedNodes, check_conflicting_processes


class NativeSession:
    """One SDK-owning child for the whole UI session; JSON requests are serialized."""
    def __init__(self, executable, robot_config, dry_run, log_dir, log):
        self.log = log
        self._lock = threading.Lock()
        self._responses = queue.Queue()
        self._id = 0
        self._broken = False
        self.log_path = Path(log_dir) / f"arms-{time.time_ns()}.log"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._errors = self.log_path.open("w", encoding="utf-8")
        argv = [str(executable), "--robot-config", str(robot_config)]
        if dry_run:
            argv.append("--dry-run")
        try:
            self.process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=self._errors,
                                            text=True, encoding="utf-8", bufsize=1,
                                            start_new_session=True)
        except BaseException:
            self._errors.close()
            raise
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(262144)
                if not line:
                    break
                if not line.endswith("\n"):
                    raise ValueError("native response too large or incomplete")
                self._responses.put(json.loads(line))
        except Exception as exc:
            self._responses.put(exc)
        finally:
            self._responses.put(RuntimeError(f"机械臂会话已退出，请查看 {self.log_path}"))

    def request(self, op, timeout=60, **fields):
        with self._lock:
            if self._broken or self.process.poll() is not None:
                raise CalibrationError(f"机械臂连接不可用，请结束会话；日志 {self.log_path}")
            self._id += 1
            try:
                self.process.stdin.write(json.dumps(dict(id=self._id, op=op, **fields)) + "\n")
                self.process.stdin.flush()
                response = self._responses.get(timeout=timeout)
                if isinstance(response, Exception):
                    raise response
                if response.get("id") != self._id:
                    raise ValueError("机械臂响应序号不匹配")
            except Exception as exc:
                self._broken = True
                raise CalibrationError(f"机械臂通信失败（{op}）：{exc}") from exc
            if response.get("ok") is not True:
                raise CalibrationError(response.get("error", "机械臂操作失败"))
            return response["result"]

    def close(self):
        # EOF lets the child disable owned drag before disconnecting. SIGTERM has
        # the same orderly path; forced termination is reported, never silent.
        errors = []
        try:
            if self.process.poll() is None:
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=3)
                        errors.append("机械臂 SDK 无响应，已终止自有会话进程；请现场确认机器人状态")
            if self.process.returncode and not (self.process.returncode == 128 + signal.SIGTERM
                    and "Session cleanup complete" in self.log_path.read_text(errors="replace")):
                errors.append(f"机械臂会话退出码 {self.process.returncode}；日志 {self.log_path}")
        finally:
            self._reader.join(timeout=1)
            self.process.stdout.close()
            if not self.process.stdin.closed:
                self.process.stdin.close()
            self._errors.close()
        if errors:
            raise CalibrationError("；".join(errors))

    def interrupt(self):
        """Request SDK-owned cleanup even when a JSON request is still waiting."""
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass


class DemoLink:
    """In-memory hand/camera simulator; never creates sockets or device nodes."""
    def __init__(self, open_targets, close_targets, camera_id):
        self.presets = dict(open=copy.deepcopy(open_targets), close=copy.deepcopy(close_targets))
        self.targets = copy.deepcopy(open_targets)
        self.poses = dict(left="open", right="open")
        self.camera_id, self.session_id = camera_id, str(uuid.uuid4())
        self.running = False

    def start(self):
        self.running = True

    def set_hand(self, side, pose):
        for key in ("left", "right") if side == "both" else (side,):
            self.targets[key] = list(self.presets[pose][key])
            self.poses[key] = pose

    def snapshot(self):
        now = mono_us()
        # A simulated camera really has a frame cadence: repeated polls must not
        # fabricate extra independent camera frames for the stability test.
        sample = now // 33333 * 33333
        raw = dict(msg_type="CameraDetection", publisher_id="camera", camera_id=self.camera_id,
                   clock_id=local_clock_id(), session_id=self.session_id, sequence=sample // 33333,
                   sample_mono_us=sample, valid=True, status="TRACKING", confidence=1.0,
                   steering_wheel=dict(valid=True, axis_match=True, calibration_id="demo-only",
                                       theta_rad=0.0, translation_along_axis_m=0.0))
        return dict(running=self.running, ready=self.running, fresh=self.running, error=None,
                    conflict=None, fault=None, targets=copy.deepcopy(self.targets), poses=dict(self.poses),
                    ack=dict(valid=self.running, age_ms=0), camera=dict(raw=raw, received_mono_us=now),
                    hands={s: dict(valid=True, status="模拟", drive_position_normalized=list(t))
                           for s, t in self.targets.items()})

    def wait_ready(self, timeout=5):
        return self.snapshot()

    def stop(self):
        self.set_hand("both", "open")
        self.running = False


def executable(path, label):
    path = Path(path).expanduser().resolve()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise CalibrationError(f"找不到 {label}：{path}")
    return path


class CalibrationController:
    def __init__(self, options, *, native_factory=NativeSession, nodes_factory=ManagedNodes,
                 link_factory=HandCameraLink, conflict_check=check_conflicting_processes):
        self.options = options
        self.native_factory, self.link_factory = native_factory, link_factory
        self.conflict_check = conflict_check
        self.lock = threading.RLock()
        self.logs = deque(maxlen=120)
        self.native = self.link = None
        self.nodes = nodes_factory(ROOT, options.log_dir, self.log)
        self._pending = queue.Queue(maxsize=1)
        self._cancel = threading.Event()
        self._shutdown = threading.Event()
        self._last_client = time.monotonic()
        self._updated = self._snapshots = None
        self._input_snapshots = None
        self._monitor_at = 0
        self._bad_hand_since = None
        self._lease_expired = False
        self.args = calibration_args(["--robot-config", str(options.robot_config),
                                      "--camera-id", options.camera_id])
        self.grasp_path, _, _ = read_inputs(self.args)
        self.state = dict(dry_run=options.dry_run, phase="idle", busy=False, error="", message="尚未连接设备",
                          prepared=False, wheel_source="camera", arms=self._empty_arms(),
                          progress=dict(current=0, total=self.args.samples), result=None, backup=None,
                          config={"robot": str(options.robot_config), "system": str(options.system_config),
                                  "camera": str(options.camera_config), "hand": str(options.hand_config),
                                  "grasp": str(self.grasp_path), "log_dir": str(options.log_dir)})
        robot = yaml.safe_load(options.robot_config.read_bytes())
        self.state["config"].update({k: robot.get("rokae", {}).get(k, "") for k in ("left_ip", "right_ip")})
        self.worker = threading.Thread(target=self._work, name="calibration-actions", daemon=True)
        self.watchdog = threading.Thread(target=self._watch, name="calibration-lease", daemon=True)
        self.worker.start()
        self.watchdog.start()

    @staticmethod
    def _empty_arms():
        return {s: dict(connected=False, dragging=False, drag_owned=False, operation_state="disconnected")
                for s in ("left", "right")}

    def log(self, text):
        with self.lock:
            self.logs.append(dict(time=time.strftime("%H:%M:%S"), text=str(text)))

    def _set(self, **values):
        with self.lock:
            self.state.update(values)

    def heartbeat(self):
        with self.lock:
            self._last_client = time.monotonic()

    def _invalidate(self):
        self._updated = self._snapshots = None
        self._set(result=None, backup=None)

    def _camera(self, hand):
        raw = hand.get("camera", {}).get("raw")
        result = dict(valid=False, reason="等待相机数据")
        if raw:
            try:
                now = mono_us()
                tracker = CameraTracker(self.args, now - int(self.args.max_age_ms * 1000), local_clock_id())
                record = tracker.accept(raw, now)
                if record:
                    result.update(record, valid=True, reason="跟踪有效", age_ms=(now - record["sample_mono_us"]) / 1000)
                else:
                    result["reason"] = "；".join(tracker.rejected)
            except (CalibrationError, TypeError, ValueError) as exc:
                result["reason"] = str(exc)
        return result

    def snapshot(self):
        with self.lock:
            data = copy.deepcopy(self.state)
            data["logs"] = list(self.logs)
            link = self.link
        hand = link.snapshot() if link else dict(ready=False, fresh=False, hands={}, poses={})
        camera = self._camera(hand)
        data.update(hand=hand, camera=camera, nodes=self.nodes.snapshot())
        still = self._arms_still(data["arms"])
        data["can_sample"] = bool(data["prepared"] and not data["busy"] and still and hand.get("ready")
                                  and (data["wheel_source"] == "manual" or camera["valid"]))
        data["can_save"] = bool(data["can_sample"] and self._updated is not None and not data["dry_run"])
        return data

    def submit(self, request):
        if not isinstance(request, dict):
            raise CalibrationError("请求必须是 JSON 对象")
        action = request.get("action")
        if action not in ("prepare", "drag_start", "drag_stop", "hand", "sample", "save", "stop"):
            raise CalibrationError("未知操作")
        if action in ("drag_start", "drag_stop", "hand") and request.get("side") not in ("left", "right", "both"):
            raise CalibrationError("请选择左侧、右侧或双侧")
        if action == "hand" and request.get("pose") not in ("open", "close"):
            raise CalibrationError("未知手部目标")
        if action == "prepare" and request.get("wheel_source") not in ("camera", "manual"):
            raise CalibrationError("请选择 camera 或 manual")
        with self.lock:
            if self._shutdown.is_set():
                raise CalibrationError("界面服务正在退出")
            self._last_client = time.monotonic()
            if action == "stop":
                self._cancel.set()
                self.state.update(busy=True, message="正在结束会话…")
                if self.native and self.state["phase"] != "stopping":
                    self.native.interrupt()
                return
            if self.state["busy"] or self._cancel.is_set():
                raise CalibrationError("当前操作尚未完成")
            if action == "prepare" and self.state["prepared"]:
                raise CalibrationError("请先结束当前会话")
            if action != "prepare" and not self.state["prepared"]:
                raise CalibrationError("请先准备校准环境")
            if action == "save" and (self.options.dry_run or self._updated is None):
                raise CalibrationError("没有可写回的实测预览；模拟模式禁止修改 grasp.json")
            self.state.update(busy=True, error="")
            self._pending.put_nowait(copy.deepcopy(request))

    def _check_cancel(self):
        if self._cancel.is_set() or self._shutdown.is_set():
            raise CalibrationError("操作已取消")

    def _watch(self):
        while not self._shutdown.wait(.25):
            with self.lock:
                active = self.state["prepared"] or self.state["busy"]
                if active and time.monotonic() - self._last_client > self.options.lease_timeout:
                    if not self._lease_expired:
                        self._lease_expired = True
                        self.log("浏览器心跳超时，将结束拖动并张开双手")
                        if self.native:
                            self.native.interrupt()
                    self._cancel.set()

    def _work(self):
        while not self._shutdown.is_set():
            if self._cancel.is_set():
                # Drop an action accepted just before stop; it must never execute
                # after the cancellation has already closed the session.
                try:
                    self._pending.get_nowait()
                except queue.Empty:
                    pass
                self._stop()
                continue
            try:
                request = self._pending.get(timeout=.1)
            except queue.Empty:
                try:
                    self._monitor()
                except Exception as exc:
                    self._stop("" if self._cancel.is_set() else str(exc))
                continue
            try:
                self._check_cancel()
                action = request["action"]
                if action == "prepare":
                    self._prepare(request["wheel_source"])
                elif action in ("drag_start", "drag_stop"):
                    self._invalidate()
                    self._check_inputs()
                    if action == "drag_start":
                        self._require_hand()
                    self.native.request(action, side=request["side"])
                    self._refresh_arms()
                    dragging = any(a["dragging"] for a in self.state["arms"].values())
                    self._set(phase="dragging" if dragging else "ready", message="拖动状态已更新")
                elif action == "hand":
                    self._invalidate()
                    self.link.set_hand(request["side"], request["pose"])
                    self.link.wait_ready(timeout=5)
                    self._set(message="手节点已接受目标；是否握住把手请现场确认", phase="ready")
                elif action == "sample":
                    self._sample(request)
                elif action == "save":
                    self._save()
                self.log(self.state["message"])
            except Exception as exc:
                if self._cancel.is_set():
                    self.log("当前操作已取消，正在清理设备会话")
                    self._stop()
                elif request["action"] in ("prepare", "drag_start", "drag_stop", "hand"):
                    self.log(f"操作失败：{exc}")
                    self._stop(str(exc))
                else:
                    self.log(f"操作失败：{exc}")
                    self._invalidate()
                    self._set(phase="error", error=str(exc), message="操作失败，请检查提示后重试")
            finally:
                if not self._cancel.is_set():
                    self._set(busy=False)
        if self.native or self.link or any(n["running"] for n in self.nodes.snapshot()):
            self._stop()

    def _prepare(self, source):
        self._invalidate()
        self._set(phase="preparing", wheel_source=source, message="准备环境：启动手节点会张开双手")
        self._lease_expired = False
        self._bad_hand_since = None
        self.args.wheel_source = source
        self.grasp_path, _, snapshots = read_inputs(self.args)
        robot = yaml.safe_load(snapshots[self.options.robot_config])
        with self.lock:
            self.state["config"].update(grasp=str(self.grasp_path),
                                        **{k: robot.get("rokae", {}).get(k, "")
                                           for k in ("left_ip", "right_ip")})
        for path in (self.options.system_config, self.options.hand_config, self.options.camera_config):
            snapshots[path] = path.read_bytes()
        self._input_snapshots = snapshots
        system = yaml.safe_load(snapshots[self.options.system_config])
        hand_config = yaml.safe_load(snapshots[self.options.hand_config])
        if not isinstance(system, dict) or not isinstance(hand_config, dict):
            raise CalibrationError("system / hand 配置必须是映射")
        presets = system.get("core_hand", {})
        opened, closed = (HandCameraLink._targets(presets.get(p)) for p in ("open", "close"))
        helper = executable(self.options.session_helper, "aviator_grasp_tool_session（请先编译）")
        if not self.options.dry_run:
            conflicts = self.conflict_check()
            if conflicts:
                raise CalibrationError("请先退出现有控制节点：" + "；".join(conflicts))
            if not hand_config.get("left_host") or not hand_config.get("right_host"):
                raise CalibrationError("rh56ftp_hand.yaml 必须配置左右两只手")
            bus = executable(self.options.bus_helper, "aviator_bus")
            hand_python = executable(self.options.hand_python, "机械手 Python")
            if source == "camera":
                camera_python = executable(self.options.camera_python, "相机 Python")
            publish, subscribe = system["bus"]["publish"], system["bus"]["subscribe"]
            self.nodes.start("bus", [str(bus), "--config", str(self.options.system_config)])
            self._check_cancel()
            self.link = self.link_factory(publish, subscribe, opened, closed, self.log,
                                          camera_id=self.options.camera_id,
                                          hand_publisher=presets.get("publisher_id", "rh56ftp_hand"))
            self.link.start()
            self.nodes.start("hand", [str(hand_python), "-u", str(ROOT / "nodes/rh56ftp_hand/rh56ftp_node.py"),
                                      "--config", str(self.options.hand_config), "--endpoint", subscribe,
                                      "--state-endpoint", publish, "--publisher-id",
                                      presets.get("publisher_id", "rh56ftp_hand")])
            self.link.wait_ready(timeout=15)
            self._check_cancel()
            if source == "camera":
                self.nodes.start("camera", [str(camera_python), "-u", str(ROOT / "nodes/camera/main.py"),
                                            "--config", str(self.options.camera_config), "--endpoint", publish,
                                            "--camera-id", self.options.camera_id, "--session", str(uuid.uuid4()),
                                            "--no-show", "--no-print-pose", "--preview-endpoint", "off"])
        else:
            self.link = DemoLink(opened, closed, self.options.camera_id)
            self.link.start()
        self._check_cancel()
        self._check_inputs()
        self.native = self.native_factory(helper, self.options.robot_config, self.options.dry_run,
                                          self.options.log_dir, self.log)
        self.native.request("connect")
        self._check_cancel()
        self._refresh_arms()
        self._set(prepared=True, phase="ready", message="校准环境已就绪；可拖动、握紧，停稳后采样")

    def _refresh_arms(self):
        status = self.native.request("status")
        self._set(arms={s: status[s] for s in ("left", "right")})

    @staticmethod
    def _arms_still(arms):
        return all(a.get("connected") and not a.get("dragging") and not a.get("drag_owned")
                   and a.get("operation_state") in ("idle", "jog") for a in arms.values())

    def _require_hand(self):
        status = self.link.snapshot()
        if not status.get("ready"):
            raise CalibrationError(status.get("fault") or status.get("error") or "机械手反馈或本会话 ACK 不新鲜")

    def _check_inputs(self):
        for path, original in (self._input_snapshots or {}).items():
            if path.read_bytes() != original:
                raise CalibrationError(f"会话期间配置发生改变，请结束并重新准备：{path}")

    def _monitor(self):
        if not self.state["prepared"]:
            return
        now = time.monotonic()
        for node in self.nodes.snapshot():
            if not node["running"]:
                raise CalibrationError(f"{node['name']} 节点已退出；日志 {node['log_path']}")
        hand = self.link.snapshot()
        if not hand.get("ready"):
            self._bad_hand_since = self._bad_hand_since or now
            if hand.get("fault") or now - self._bad_hand_since > .5:
                raise CalibrationError(hand.get("fault") or "机械手反馈/ACK 持续失效，结束校准会话")
        else:
            self._bad_hand_since = None
        if now >= self._monitor_at:
            self._monitor_at = now + 1
            self._refresh_arms()
            if any(not a.get("connected") for a in self.state["arms"].values()):
                raise CalibrationError("机械臂连接已中断")

    def _sample(self, request):
        self._invalidate()
        self._check_inputs()
        self._require_hand()
        self._refresh_arms()
        if not self._arms_still(self.state["arms"]):
            raise CalibrationError("请先结束双臂拖动，保持静止后再采样")
        manual = self.state["wheel_source"] == "manual"
        if manual:
            angle, displacement = request.get("angle_deg"), request.get("displacement_m")
            if (type(angle) not in (int, float) or not math.isfinite(angle) or abs(math.radians(angle)) > .87266
                    or type(displacement) not in (int, float) or not math.isfinite(displacement)
                    or not -.170 <= displacement <= 0):
                raise CalibrationError("手动角度必须在 ±0.87266 rad 内，位移必须为 [-0.170,0] m")
            angle = math.radians(angle)
        grasp_path, grasp, snapshots = read_inputs(self.args)
        self._set(phase="sampling", progress=dict(current=0, total=self.args.samples), message="请保持轮盘和双臂静止…")
        start = mono_us()
        tracker = None if manual else CameraTracker(self.args, start, local_clock_id())
        states = []
        deadline = time.monotonic() + self.args.timeout_s

        def observe():
            self._check_cancel()
            self._require_hand()
            if time.monotonic() > deadline:
                reasons = dict(tracker.rejected) if tracker else {}
                raise CalibrationError(f"采样超时：{reasons}")
            if tracker:
                raw = self.link.snapshot().get("camera", {}).get("raw")
                if raw:
                    tracker.accept(raw, mono_us())

        while tracker and not tracker.records:
            observe()
            self._cancel.wait(.01)
        for i in range(self.args.samples):
            observe()
            state = self.native.request("sample", timeout=10)
            validate_state(state, mono_us(), start, states[-1]["sample_mono_us"] if states else start - 1,
                           self.options.dry_run, self.args.max_age_ms)
            states.append(state)
            self._set(progress=dict(current=i + 1, total=self.args.samples))
            end = time.monotonic() + self.args.interval_ms / 1000
            while time.monotonic() < end:
                observe()
                self._cancel.wait(.01)
        if tracker:
            while tracker.records[-1]["sample_mono_us"] < states[-1]["sample_end_mono_us"]:
                observe()
                self._cancel.wait(.01)
            samples = pair_samples(states, tracker.records, self.args.max_sync_ms)
        else:
            samples = [dict(s, angle_rad=angle, displacement_m=displacement) for s in states]
        updated, report = calibrate(grasp, samples,
                                    max_joint_span_rad=math.radians(self.args.max_joint_span_deg),
                                    max_position_span_m=self.args.max_position_span_mm / 1000,
                                    max_rotation_span_rad=math.radians(self.args.max_rotation_span_deg),
                                    max_wheel_angle_span_rad=math.radians(self.args.max_wheel_angle_span_deg),
                                    max_wheel_displacement_span_m=self.args.max_wheel_displacement_span_mm / 1000)
        self._check_cancel()
        self._check_inputs()
        self._updated, self._snapshots = updated, snapshots
        self.grasp_path = grasp_path
        report.update(dry_run=self.options.dry_run, wheel_source=self.state["wheel_source"],
                      grasp_path=str(grasp_path), samples=samples)
        path = self.options.log_dir / f"preview-{time.time_ns()}.json"
        write_report(path, report)
        self.log(f"采样记录：{path}")
        self._set(result=report, phase="preview", message="校准预览已生成" + ("（模拟结果，禁止写回）" if self.options.dry_run else "，确认数值后可备份并写回"))

    def _save(self):
        if self.options.dry_run or self._updated is None:
            raise CalibrationError("没有可写回的实测校准结果")
        self._require_hand()
        self._check_cancel()
        self._check_inputs()
        self._refresh_arms()
        if not self._arms_still(self.state["arms"]):
            raise CalibrationError("机械臂状态已改变，请结束拖动并重新采样")
        self._check_cancel()
        self._check_inputs()
        backup = save_grasp(self.grasp_path, self._updated, self._snapshots[self.grasp_path])
        self._input_snapshots[self.grasp_path] = self.grasp_path.read_bytes()
        self._updated = self._snapshots = None
        self._set(backup=str(backup), phase="preview", message=f"已更新 tool.left/right；备份：{backup}")

    def _stop(self, reason=""):
        self._set(phase="stopping", busy=True, message="正在结束拖动并张开双手…")
        self._invalidate()
        errors = []
        arm_errors = []
        if self.native:
            # A stop/lease cancellation already signals the child. Wait for its
            # orderly cleanup instead of writing more requests to a closing pipe.
            if not self._cancel.is_set():
                try:
                    self.native.request("disconnect")
                except Exception as exc:
                    arm_errors.append(str(exc))
            try:
                self.native.close()
            except Exception as exc:
                arm_errors.append(str(exc))
            self.native = None
        errors.extend(arm_errors)
        if self.link:
            try:
                self.link.stop()
            except Exception as exc:
                errors.append(str(exc))
            self.link = None
        try:
            self.nodes.stop_all()
        except Exception as exc:
            errors.append(str(exc))
        error = "；".join(([reason] if reason else []) + errors)
        arms = self._empty_arms()
        if arm_errors:
            arms = copy.deepcopy(self.state["arms"])
            for arm in arms.values():
                arm.update(operation_state="unknown_cleanup_failed", cleanup_failed=True, powered=None)
        with self.lock:
            self._set(prepared=False, busy=False, arms=arms, phase="error" if error else "idle",
                      error=error, message="清理出现错误，请查看日志并现场确认" if errors else "会话已结束（已请求结束拖动和松手）")
            self._cancel.clear()
        self.log(error or self.state["message"])

    def close(self):
        self._cancel.set()
        self._shutdown.set()
        if self.native:
            self.native.interrupt()
        self.worker.join(timeout=150)
        self.watchdog.join(timeout=1)
        if self.worker.is_alive():
            raise CalibrationError("设备清理尚未完成；请检查机械臂和日志")
