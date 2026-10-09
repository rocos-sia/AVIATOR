#!/usr/bin/env python3
"""静止抓握姿态校准：实测关节 FK + 相机/手动轮盘位姿 → 左右 tool。"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import selectors
import socket
import subprocess
import sys
import tempfile
import time

import yaml

if __package__:
    from .calibration import CalibrationError, calibrate, camera_wheel, save_grasp, validate_grasp
else:
    from calibration import CalibrationError, calibrate, camera_wheel, save_grasp, validate_grasp

ROOT = Path(__file__).resolve().parents[2]
TOPIC = b"camera.detection"


def mono_us():
    return time.monotonic_ns() // 1000


def local_clock_id():
    return socket.gethostname()[:80] + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def number(value):
    try:
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
    except (OverflowError, TypeError):
        return False


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--robot-config", type=Path, default=ROOT / "config/robot.yaml")
    p.add_argument("--state-helper", type=Path, help="aviator_grasp_tool_state 可执行文件")
    p.add_argument("--joints-file", type=Path, help="离线双臂关节角 JSON（rad），使用时不连接机器人")
    p.add_argument("--wheel-source", choices=("camera", "manual"), default="camera")
    angle = p.add_mutually_exclusive_group()
    angle.add_argument("--angle-deg", type=float, help="手动 Core 轮盘角度，度")
    angle.add_argument("--angle-rad", type=float, help="手动 Core 轮盘角度，弧度")
    p.add_argument("--displacement-m", type=float, help="手动 Core 轮盘位移，[-0.170, 0] m")
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5556", help="camera.detection SUB connect 端点")
    p.add_argument("--camera-id", default="cockpit")
    p.add_argument("--min-confidence", type=float, default=0.5)
    p.add_argument("--samples", type=int, default=20)
    p.add_argument("--interval-ms", type=int, default=50)
    p.add_argument("--timeout-s", type=float, default=20, help="等待相机/采集的各阶段超时")
    p.add_argument("--max-age-ms", type=float, default=250, help="相机和关节样本最大接收年龄")
    p.add_argument("--max-sync-ms", type=float, default=100, help="相机与双臂读取时间的最大差")
    p.add_argument("--max-joint-span-deg", type=float, default=0.2)
    p.add_argument("--max-position-span-mm", type=float, default=2)
    p.add_argument("--max-rotation-span-deg", type=float, default=0.5)
    p.add_argument("--max-wheel-angle-span-deg", type=float, default=0.5)
    p.add_argument("--max-wheel-displacement-span-mm", type=float, default=3)
    p.add_argument("--report", type=Path, help="保存结果及原始采样 JSON")
    p.add_argument("--write", action="store_true", help="备份后写回 robot.yaml 指向的 grasp.json；默认只预览")
    a = p.parse_args(argv)
    if not 3 <= a.samples <= 500 or not 1 <= a.interval_ms <= 10000:
        p.error("samples 应为 3..500，interval-ms 应为 1..10000")
    positive = ("timeout_s", "max_age_ms", "max_sync_ms", "max_joint_span_deg",
                "max_position_span_mm", "max_rotation_span_deg", "max_wheel_angle_span_deg",
                "max_wheel_displacement_span_mm")
    if any(not math.isfinite(getattr(a, name)) or getattr(a, name) <= 0 for name in positive):
        p.error("超时、时效、同步和静止阈值必须为有限正数")
    duration = (a.samples - 1) * a.interval_ms / 1000
    if duration > 300 or a.timeout_s <= duration:
        p.error("采样跨度必须 <=300 s，timeout-s 必须大于采样跨度")
    if not math.isfinite(a.min_confidence) or not 0 <= a.min_confidence <= 1:
        p.error("min-confidence 必须在 [0,1]")
    manual_angle = a.angle_rad if a.angle_rad is not None else (
        math.radians(a.angle_deg) if a.angle_deg is not None else None)
    if a.wheel_source == "manual":
        if manual_angle is None or a.displacement_m is None:
            p.error("manual 模式需要 angle-deg/angle-rad 和 displacement-m")
        if (not number(manual_angle) or abs(manual_angle) > .87266 or
                not number(a.displacement_m) or not -.170 <= a.displacement_m <= 0):
            p.error("手动角度需在 ±0.87266 rad，位移需在 [-0.170,0] m")
    elif manual_angle is not None or a.displacement_m is not None:
        p.error("手动角度/位移仅用于 --wheel-source manual，避免坐标约定混用")
    a.manual_angle = manual_angle
    return a


class CameraTracker:
    """Pin one camera session/calibration and retain fresh monotonic observations."""
    def __init__(self, args, start_us, clock_id):
        self.args, self.start_us, self.clock_id = args, start_us, clock_id
        self.session = self.calibration_id = None
        self.last_sequence = self.last_sample = -1
        self.records = []
        self.rejected = Counter()

    def reject(self, reason):
        self.rejected[reason] += 1
        return None

    def accept(self, data, now):
        if not isinstance(data, dict) or data.get("msg_type") != "CameraDetection":
            return self.reject("消息类型错误")
        if data.get("camera_id") != self.args.camera_id or data.get("publisher_id") != "camera":
            return self.reject("不是指定相机")
        if data.get("clock_id") != self.clock_id:
            return self.reject("相机与工具不是同一主机单调时钟")
        session, seq, sample = data.get("session_id"), data.get("sequence"), data.get("sample_mono_us")
        if not isinstance(session, str) or not session or not integer(seq) or seq < 0 or not integer(sample):
            return self.reject("无效会话/序号/时间戳")
        if sample < self.start_us or not 0 <= now - sample <= self.args.max_age_ms * 1000:
            return self.reject("相机数据过期或时间在未来")
        if self.session is not None and session != self.session:
            raise CalibrationError("采样期间相机会话改变，请确保只运行一个相机发布者并重新采样")
        self.session = session
        if seq <= self.last_sequence or sample <= self.last_sample:
            return self.reject("重复或倒序相机帧")
        # Invalid newer frames also advance the watermark.
        self.last_sequence, self.last_sample = seq, sample
        wheel = data.get("steering_wheel")
        confidence = data.get("confidence")
        if (data.get("valid") is not True or data.get("status") != "TRACKING" or
                not number(confidence) or not self.args.min_confidence <= confidence <= 1 or
                not isinstance(wheel, dict) or wheel.get("valid") is not True or
                wheel.get("axis_match") is False):
            return self.reject("相机/轮盘未有效跟踪或置信度不足")
        calibration_id = wheel.get("calibration_id")
        if not isinstance(calibration_id, str) or not calibration_id:
            return self.reject("缺少轮盘标定标识")
        if self.calibration_id is not None and calibration_id != self.calibration_id:
            raise CalibrationError("采样期间相机标定发生改变，请重新采样")
        theta, travel = wheel.get("theta_rad"), wheel.get("translation_along_axis_m")
        angle, displacement = camera_wheel(theta, travel)
        self.calibration_id = calibration_id
        result = dict(sample_mono_us=sample, sequence=seq, session_id=session,
                      calibration_id=calibration_id, angle_rad=angle, displacement_m=displacement,
                      theta_rad=theta, translation_along_axis_m=travel, confidence=confidence)
        self.records.append(result)
        if len(self.records) > 30000:
            raise CalibrationError("相机采样数量异常，终止采集")
        return result

    def drain(self, sub):
        for _ in range(64):
            if not sub.poll(0):
                break
            frames = sub.recv_multipart()
            if len(frames) != 2 or frames[0] != TOPIC:
                self.reject("无效相机消息帧")
                continue
            try:
                data = json.loads(frames[1])
            except (ValueError, UnicodeError):
                self.reject("相机 JSON 无效")
                continue
            self.accept(data, mono_us())


def validate_state(state, now, start_us, previous, offline, max_age_ms):
    if not isinstance(state, dict) or state.get("frame") != "aircraft":
        raise CalibrationError("关节采样必须提供 aircraft 坐标系法兰位姿")
    if state.get("source") != ("offline" if offline else "robot"):
        raise CalibrationError("关节数据来源与采样模式不一致")
    begin, end, mid = (state.get(name) for name in
                       ("sample_start_mono_us", "sample_end_mono_us", "sample_mono_us"))
    if (not all(integer(v) for v in (begin, mid, end)) or
            not start_us <= begin <= mid <= end <= now or
            mid <= previous or now - begin > max_age_ms * 1000):
        raise CalibrationError("双臂关节样本过期、读取耗时过长或时间戳倒序")
    return state


def pair_samples(states, camera, max_sync_ms):
    result = []
    for state in states:
        matched = min(camera, key=lambda c: abs(c["sample_mono_us"] - state["sample_mono_us"]))
        stamp = matched["sample_mono_us"]
        gap = max(abs(stamp - state["sample_start_mono_us"]), abs(stamp - state["sample_end_mono_us"]))
        if gap > max_sync_ms * 1000:
            raise CalibrationError(f"相机/双臂采样未对齐：最大时间差 {gap / 1000:.1f} ms")
        result.append(dict(state, angle_rad=matched["angle_rad"], displacement_m=matched["displacement_m"],
                           camera=matched, sync_gap_ms=gap / 1000))
    if len({s["camera"]["sequence"] for s in result}) < 3:
        raise CalibrationError("匹配到的独立相机帧不足 3 帧，请增加采样间隔/数量")
    return result


def helper_path(requested):
    candidates = [requested] if requested else [ROOT / "build/bin/aviator_grasp_tool_state",
                                               ROOT / "build/release/bin/aviator_grasp_tool_state"]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return path.resolve()
    raise CalibrationError("找不到 aviator_grasp_tool_state；先运行 "
                           "cmake --build build --target aviator_grasp_tool_state -j2，"
                           "或指定 --state-helper")


def collect(args, helper):
    start = mono_us()
    tracker = sub = context = process = None
    states = []
    with tempfile.TemporaryFile() as errors, selectors.DefaultSelector() as selector:
        try:
            if args.wheel_source == "camera":
                import zmq
                tracker = CameraTracker(args, start, local_clock_id())
                context = zmq.Context()
                sub = context.socket(zmq.SUB)
                sub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.RCVHWM, 64)
                sub.setsockopt(zmq.MAXMSGSIZE, 65536)
                sub.setsockopt(zmq.SUBSCRIBE, TOPIC)
                sub.connect(args.endpoint)
                deadline = time.monotonic() + args.timeout_s
                print("等待新鲜有效的相机轮盘数据…", file=sys.stderr, flush=True)
                while not tracker.records:
                    if time.monotonic() >= deadline:
                        raise CalibrationError(f"等待相机超时：{dict(tracker.rejected)}")
                    sub.poll(20)
                    tracker.drain(sub)
            command = [str(helper), "--robot-config", str(args.robot_config.resolve()),
                       "--samples", str(args.samples), "--interval-ms", str(args.interval_ms)]
            command += (["--joints-file", str(args.joints_file.resolve())] if args.joints_file else ["--read-robot"])
            print(f"采集 {args.samples} 组静止数据（{'离线关节' if args.joints_file else 'SDK 实测关节'}）…",
                  file=sys.stderr, flush=True)
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors)
            selector.register(process.stdout, selectors.EVENT_READ)
            buffer = b""
            eof = False
            deadline = time.monotonic() + args.timeout_s
            while True:
                if time.monotonic() >= deadline:
                    raise CalibrationError("关节/相机同步采样超时，请检查辅助程序和数据源")
                if tracker:
                    tracker.drain(sub)
                for key, _ in selector.select(0.01):
                    chunk = os.read(key.fd, 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        eof = True
                        if buffer:
                            raise CalibrationError("辅助程序输出了不完整的 JSONL 样本")
                        continue
                    buffer += chunk
                    if len(buffer) > 131072:
                        raise CalibrationError("辅助程序输出过大")
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            state = json.loads(line)
                        except (ValueError, UnicodeError) as exc:
                            raise CalibrationError("辅助程序输出不是有效 JSONL") from exc
                        previous = states[-1]["sample_mono_us"] if states else start - 1
                        validate_state(state, mono_us(), start, previous, bool(args.joints_file), args.max_age_ms)
                        states.append(state)
                        if len(states) > args.samples:
                            raise CalibrationError("辅助程序返回了多余样本")
                code = process.poll()
                if code is not None and code != 0:
                    errors.seek(0)
                    detail = errors.read(65536).decode("utf-8", "replace").strip()
                    raise CalibrationError(f"关节采样失败（退出码 {code}）：\n{detail}")
                if eof and code == 0:
                    if len(states) != args.samples:
                        raise CalibrationError(f"关节样本不足：{len(states)}/{args.samples}")
                    if not tracker or tracker.records[-1]["sample_mono_us"] >= states[-1]["sample_end_mono_us"]:
                        break
            if tracker:
                samples = pair_samples(states, tracker.records, args.max_sync_ms)
                metadata = dict(camera_session=tracker.session, calibration_id=tracker.calibration_id,
                                clock_id=tracker.clock_id, rejected_camera_frames=dict(tracker.rejected),
                                max_sync_gap_ms=max(s["sync_gap_ms"] for s in samples))
            else:
                samples = [dict(s, angle_rad=args.manual_angle, displacement_m=args.displacement_m) for s in states]
                metadata = {}
            return samples, metadata
        finally:
            if process:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                if process.stdout:
                    process.stdout.close()
            if sub is not None:
                sub.close()
            if context is not None:
                context.term()


def read_inputs(args):
    robot_path = args.robot_config.resolve()
    snapshots = {robot_path: robot_path.read_bytes()}
    config = yaml.safe_load(snapshots[robot_path])
    if not isinstance(config, dict):
        raise CalibrationError("robot.yaml 必须是映射")
    paths = {}
    for name in ("grasp", "urdf", "posture"):
        value = config.get(name)
        if not isinstance(value, str) or not value:
            raise CalibrationError(f"robot.yaml 缺少有效的 {name} 路径")
        paths[name] = (robot_path.parent / value).resolve()
        snapshots[paths[name]] = paths[name].read_bytes()
    if args.joints_file:
        joints = args.joints_file.resolve()
        snapshots[joints] = joints.read_bytes()
    if args.report and args.report.resolve() in snapshots:
        raise CalibrationError("report 不能覆盖任何输入配置/关节文件")
    grasp = json.loads(snapshots[paths["grasp"]])
    validate_grasp(grasp)
    return paths["grasp"], grasp, snapshots


def write_report(path, report):
    path = path.resolve()
    data = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main(argv=None):
    args = parse_args(argv)
    try:
        grasp_path, grasp, snapshots = read_inputs(args)
        helper = helper_path(args.state_helper)
        samples, acquisition = collect(args, helper)
        updated, report = calibrate(
            grasp, samples, max_joint_span_rad=math.radians(args.max_joint_span_deg),
            max_position_span_m=args.max_position_span_mm / 1000,
            max_rotation_span_rad=math.radians(args.max_rotation_span_deg),
            max_wheel_angle_span_rad=math.radians(args.max_wheel_angle_span_deg),
            max_wheel_displacement_span_m=args.max_wheel_displacement_span_mm / 1000)
        for path, original in snapshots.items():
            if path.read_bytes() != original:
                raise CalibrationError(f"采样期间输入文件改变，请重新校准：{path}")
        report.update(wheel_source=args.wheel_source, joint_source="offline" if args.joints_file else "robot",
                      grasp_path=str(grasp_path), state_helper=str(helper), acquisition=acquisition,
                      input_sha256={str(p): hashlib.sha256(b).hexdigest() for p, b in snapshots.items()},
                      samples=samples, write_requested=args.write)
        print(f"校准预览：{grasp_path}")
        for side in ("left", "right"):
            arm = report["arms"][side]
            print(f"{side}: 平移变化 {arm['delta_position_norm_m'] * 1000:.3f} mm，"
                  f"姿态变化 {math.degrees(arm['delta_rotation_rad']):.3f} deg")
            print(f"  原 tool: {json.dumps(arm['old'], ensure_ascii=False)}")
            print(f"  新 tool: {json.dumps(arm['new'], ensure_ascii=False)}")
            print(f"  本组最大法兰重建残差：{arm['reconstruction_max_position_error_m'] * 1000:.3f} mm / "
                  f"{math.degrees(arm['reconstruction_max_rotation_error_rad']):.3f} deg")
        print("四元数顺序 [w,x,y,z]；同组重建残差不代表独立标定精度。")
        if args.report:
            write_report(args.report, report)
            print(f"采样及计算报告：{args.report.resolve()}")
        if args.write:
            backup = save_grasp(grasp_path, updated, snapshots[grasp_path])
            print(f"已写回：{grasp_path}\n原文件备份：{backup}\n重启控制程序后使用新 tool。")
        else:
            print("本次仅预览；加 --write 可备份后写回。")
        return 0
    except (CalibrationError, OSError, ValueError, yaml.YAMLError, ImportError) as exc:
        print(f"校准失败：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("校准已中断；如请求了写回，请核对目标文件及备份。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
