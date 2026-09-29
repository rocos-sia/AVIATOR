#!/usr/bin/env python3
"""RealSense fiducial node: publish detections and send raw frames to Logger."""

import argparse
import hashlib
import json
import os
import signal
import socket
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import zmq
from detectors import create_detector
from recording_client import RecordingClient
from visualization import CameraVisualization, visualization_options


DEFAULT_CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "config", "camera.yaml"))

# =========================
# 公共 Header 辅助（与 common/runtime.cpp 对齐）
# =========================

def read_proc(path):
    with open(path) as f:
        return f.read().strip()


def session_id():
    """与 aviator::new_session_id() 一致：读 /proc/sys/kernel/random/uuid。"""
    return read_proc("/proc/sys/kernel/random/uuid")


def clock_id():
    """与 aviator::local_clock_id() 一致：hostname[:80] + '-' + boot_id。"""
    return socket.gethostname()[:80] + "-" + read_proc("/proc/sys/kernel/random/boot_id")


def utc_us():
    return time.time_ns() // 1000          # CLOCK_REALTIME


def monotonic_us():
    return time.monotonic_ns() // 1000     # CLOCK_MONOTONIC


def stop_on_sigterm(_signum, _frame):
    raise KeyboardInterrupt


# =========================
# 配置加载
# =========================

def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        raise SystemExit(f"配置文件 {path} 顶层必须是映射")
    camera = cfg.get("camera", {})
    detector_config = cfg.get("detector", {"type": "charuco"})
    if not isinstance(camera, dict) or not isinstance(detector_config, dict):
        raise SystemExit("配置文件的 camera / detector 必须是映射")
    kind = detector_config.get("type", "charuco")
    if kind not in ("charuco", "apriltag"):
        raise SystemExit("detector.type 必须为 charuco 或 apriltag")
    settings = cfg.get(kind, {})
    if not isinstance(settings, dict):
        raise SystemExit(f"配置文件的 {kind} 必须是映射")
    return camera, kind, settings, cfg.get("visualization", {})


def required(mapping, key, where):
    """取必填配置项，缺失即报错（不写死回退值）。"""
    if key not in mapping:
        raise SystemExit(f"配置文件缺少 {where}.{key}")
    return mapping[key]


def resolve_device(context, model, serial):
    """按 serial(精确) -> model(名称子串) -> 第一台 选择设备，返回其序列号。"""
    devices = context.query_devices()
    if len(devices) == 0:
        raise RuntimeError("未检测到 RealSense 设备")
    if serial:
        for d in devices:
            if d.get_info(rs.camera_info.serial_number) == serial:
                return serial
        raise RuntimeError(f"未找到序列号 {serial!r} 的设备")
    if model:
        for d in devices:
            name = d.get_info(rs.camera_info.name)
            if model.lower() in name.lower():
                chosen = d.get_info(rs.camera_info.serial_number)
                print(f"aviator_camera: 按型号 {model!r} 选中 {name} ({chosen})")
                return chosen
        present = [d.get_info(rs.camera_info.name) for d in devices]
        raise RuntimeError(f"未找到型号包含 {model!r} 的设备；已连接: {present}")
    chosen = devices[0].get_info(rs.camera_info.serial_number)
    print(f"aviator_camera: 未指定型号/序列号，使用第一台 "
          f"{devices[0].get_info(rs.camera_info.name)} ({chosen})")
    return chosen


# =========================
# 消息组装
# =========================

def make_message(camera_id, frame_id, sequence, sample_mono_us,
                 session, clock, width, height, status, confidence, pose,
                 detector=None, tag_id=None, decision_margin=None):
    """构造 CameraDetection JSON（公共头部 + 业务字段 + 补充 pose 块）。

    时间戳规则（见 docs/AVIATOR_ZMQ协议格式说明.md §12）：
    - sample_mono_us = 主机收到 frameset 的单调时刻
    - timestamp      = 检测快照生成时刻（UTC）
    """
    message = {
        "msg_type": "CameraDetection",
        "version": "1.0",
        "sequence": sequence,
        "timestamp": utc_us(),
        "sample_mono_us": sample_mono_us,
        "clock_id": clock,
        "publisher_id": "camera",
        "session_id": session,
        "valid": status == "TRACKING",
        "camera_id": camera_id,
        "frame_id": frame_id,
        "image_width": width,
        "image_height": height,
        "status": status,
        "confidence": confidence,
        "pose": pose,
    }
    if detector is not None:
        message["detector"] = detector
    if tag_id is not None:
        message["tag_id"] = tag_id
    if decision_margin is not None:
        message["decision_margin"] = decision_margin
    return message


def make_record_frame(camera_id, session, clock, config_id, frame_id, sample_mono_us,
                      timestamp_us, fps, stream, sensor_frame, image, calibration,
                      depth_scale=None):
    """Prepare one raw CameraPacket payload for the Logger's 5557 ingress."""
    if stream == "rgb":
        pixels = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pixel_format, bytes_per_pixel = "RGB8", 3
    elif stream == "depth":
        if image.dtype != np.dtype("<u2"):
            raise ValueError("depth image must be little-endian Z16")
        pixels = np.ascontiguousarray(image)
        pixel_format, bytes_per_pixel = "Z16", 2
    else:
        raise ValueError(f"unsupported stream: {stream}")
    height, width = pixels.shape[:2]
    if width <= 0 or height <= 0:
        raise ValueError("empty camera frame")
    metadata = {
        "version": 1, "camera_id": camera_id, "stream": stream,
        "publisher_id": "camera", "session_id": session, "clock_id": clock,
        "config_id": config_id, "sequence": int(sensor_frame.get_frame_number()) or frame_id,
        "frame_id": frame_id, "timestamp_us": timestamp_us,
        "sample_mono_us": sample_mono_us, "sample_time_basis": "host_frameset_receive",
        "sensor_timestamp_ms": sensor_frame.get_timestamp(),
        "sensor_timestamp_domain": str(sensor_frame.get_frame_timestamp_domain()),
        "width": width, "height": height, "stride_bytes": width * bytes_per_pixel,
        "fps": fps, "pixel_format": pixel_format, "byte_order": "little",
        "encoding": "raw", "calibration": calibration,
    }
    if stream == "depth":
        metadata["depth_scale"] = depth_scale
    return metadata, pixels.tobytes()


def parse_args(argv):
    p = argparse.ArgumentParser(description="RealSense ChArUco/AprilTag 姿态 -> camera.detection 发布")
    p.add_argument("--config", default=DEFAULT_CONFIG, help="YAML 配置文件路径")
    p.add_argument("--recording-config", help="Logger recording.yaml；省略则不发送图像记录")
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5555",
                   help="PUB 连接端点（AVIATOR publish_endpoint）")
    p.add_argument("--bind", action="store_true",
                   help="PUB 改为 bind 而非 connect（直连测试，绕过总线）")
    p.add_argument("--camera-id", default="cockpit", help="camera_id 字段")
    p.add_argument("--session", default="", help="与 Logger --session 相同的记录会话 UUID")
    p.add_argument("--show", action=argparse.BooleanOptionalAction, default=None,
                   help="开启实时识别与位姿预览（默认关闭；--no-show 强制关闭）")
    p.add_argument("--print-pose", action=argparse.BooleanOptionalAction, default=None,
                   help="在终端打印位姿与检测状态（默认关闭）")
    p.add_argument("--pose-print-interval", type=float, default=None,
                   help="位姿打印间隔秒数，默认 0.5；状态变化立即打印")
    p.add_argument("--warmup-s", type=float, default=None,
                   help="启动后跳过发布的热身秒数（0 禁用；默认以 YAML 为准）")
    # 以下几何/采集参数默认 None = 以 YAML 为准，命令行显式给出时才覆盖。
    p.add_argument("--width", type=int, default=None)
    p.add_argument("--height", type=int, default=None)
    p.add_argument("--fps", type=int, default=None)
    p.add_argument("--square-length", type=float, default=None)
    p.add_argument("--marker-length", type=float, default=None)
    p.add_argument("--board-size", default=None, help="如 5x5（覆盖 YAML board_size）")
    p.add_argument("--min-corners", type=int, default=None)
    return p.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    camera, detector_kind, detector_settings, visual_settings = load_config(args.config)
    visual = CameraVisualization(**visualization_options(visual_settings, args))
    visual.check_available()
    recorder = RecordingClient(args.recording_config, args.camera_id)

    # 采集格式（CLI 覆盖 YAML；必填键缺失即报错，不写死回退值）
    width = args.width if args.width is not None else required(camera, "width", "camera")
    height = args.height if args.height is not None else required(camera, "height", "camera")
    fps = args.fps if args.fps is not None else required(camera, "fps", "camera")
    warmup_s = float(args.warmup_s if args.warmup_s is not None
                     else required(camera, "warmup_s", "camera"))
    model = camera.get("model", "")    # 空 = 自动选择
    serial = camera.get("serial", "")  # 空 = 自动选择

    charuco_overrides = {
        "board_size": args.board_size, "square_length": args.square_length,
        "marker_length": args.marker_length, "min_corners": args.min_corners,
    }
    if detector_kind == "charuco":
        detector_settings = dict(detector_settings)
        detector_settings.update({key: value for key, value in charuco_overrides.items()
                                  if value is not None})
        for key in ("board_size", "square_length", "marker_length", "min_corners",
                    "dictionary"):
            required(detector_settings, key, "charuco")
    elif any(value is not None for value in charuco_overrides.values()):
        raise SystemExit("ChArUco 命令行参数仅在 detector.type=charuco 时可用")
    else:
        for key in ("tag_id", "tag_size_m"):
            required(detector_settings, key, "apriltag")

    # -------------------------
    # Camera（按型号/序列号选设备）
    # -------------------------
    context_rs = rs.context()
    device_serial = resolve_device(context_rs, model, serial)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    if recorder.record_depth:
        depth = camera.get("depth", {})
        config.enable_stream(rs.stream.depth, depth.get("width", width),
                             depth.get("height", height), rs.format.z16, fps)
    context = None
    pub = None
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        device = profile.get_device()
        device_name = device.get_info(rs.camera_info.name)
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.asarray(intrinsics.coeffs, dtype=np.float32).reshape(-1, 1)
        depth_scale = device.first_depth_sensor().get_depth_scale() if recorder.record_depth else None
        calibration = {}
        if recorder.enabled:
            streams = [("rgb", rs.stream.color)]
            if recorder.record_depth:
                streams.append(("depth", rs.stream.depth))
            for name, kind in streams:
                video_profile = profile.get_stream(kind).as_video_stream_profile()
                k = video_profile.get_intrinsics()
                extrinsics = video_profile.get_extrinsics_to(profile.get_stream(rs.stream.color))
                calibration[name] = {
                    "serial": device_serial, "width": k.width, "height": k.height,
                    "fx": k.fx, "fy": k.fy, "ppx": k.ppx, "ppy": k.ppy,
                    "distortion_model": str(k.model), "coeffs": list(k.coeffs),
                    "to_color_rotation": list(extrinsics.rotation),
                    "to_color_translation_m": list(extrinsics.translation)}
        config_id = hashlib.sha256(
            json.dumps(calibration, sort_keys=True).encode()).hexdigest()[:16]

        # Only the selected detector is initialized.
        selected_distortion = dist_coeffs
        if detector_kind == "apriltag":
            use_distortion = detector_settings.get("use_distortion", False)
            if not isinstance(use_distortion, bool):
                raise ValueError("apriltag.use_distortion must be boolean")
            if not use_distortion:
                selected_distortion = np.zeros_like(dist_coeffs)
        detector = create_detector(detector_kind, detector_settings,
                                   camera_matrix, selected_distortion)

        session = args.session or session_id()
        clock = clock_id()
        context = zmq.Context()
        pub = context.socket(zmq.PUB)
        pub.setsockopt(zmq.SNDHWM, 8)
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(zmq.SNDTIMEO, 0)
        pub.setsockopt(zmq.MAXMSGSIZE, 65536)
        if args.bind:
            pub.bind(args.endpoint)
        else:
            pub.connect(args.endpoint)
    except BaseException:
        if pub is not None:
            pub.close()
        if context is not None:
            context.term()
        if started:
            pipeline.stop()
        raise
    print(f"aviator_camera: {device_name} ({device_serial}) {width}x{height}@{fps}")
    print(f"aviator_camera: detector={detector.kind} settings={detector_settings}")
    print(f"aviator_camera: endpoint={args.endpoint} bind={args.bind}")
    print(f"aviator_camera: show={visual.show} print_pose={visual.print_pose} "
          f"pose_print_interval={visual.interval}s")
    print(f"aviator_camera: session={session} clock={clock} warmup_s={warmup_s} "
          f"recording={'enabled' if recorder.enabled else 'disabled'} "
          f"record_depth={recorder.record_depth}")

    warmup_until = monotonic_us() + int(warmup_s * 1e6)
    sequence = 0
    frame_id = 0
    old_sigterm = signal.signal(signal.SIGTERM, stop_on_sigterm)
    try:
        while True:
            if recorder.error is not None:
                raise RuntimeError("camera recording sender failed") from recorder.error
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame_id += 1
            sample_mono_us = monotonic_us()
            timestamp_us = utc_us()
            image = np.asanyarray(color_frame.get_data())
            w, h = color_frame.get_width(), color_frame.get_height()

            if recorder.enabled and sample_mono_us >= warmup_until:
                record_sources = [("rgb", color_frame, image)]
                if recorder.record_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        record_sources.append(
                            ("depth", depth_frame, np.asanyarray(depth_frame.get_data())))
                    else:
                        print("aviator_camera DEGRADED: missing depth frame", file=sys.stderr)
                for stream, sensor_frame, pixels in record_sources:
                    metadata, data = make_record_frame(
                        args.camera_id, session, clock, config_id, frame_id,
                        sample_mono_us, timestamp_us, fps, stream, sensor_frame,
                        pixels, calibration[stream], depth_scale)
                    recorder.submit(metadata, data)

            detection = detector.detect(image)

            if sample_mono_us >= warmup_until:
                sequence += 1
                message = make_message(args.camera_id, frame_id, sequence, sample_mono_us,
                                       session, clock, w, h, detection.status,
                                       detection.confidence, detection.pose,
                                       detector.kind, detection.tag_id,
                                       detection.decision_margin)
                payload = json.dumps(message)
                pub.send_multipart([b"camera.detection", payload.encode("utf-8")])

            if not visual.update(image, detector, detection, frame_id,
                                 warming_up=sample_mono_us < warmup_until):
                break
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            recorder.close()
        finally:
            pipeline.stop()
            visual.close()
            pub.close()
            context.term()
            signal.signal(signal.SIGTERM, old_sigterm)
            print("aviator_camera: stopped")


if __name__ == "__main__":
    main(sys.argv[1:])
