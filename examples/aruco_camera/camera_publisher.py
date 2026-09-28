#!/usr/bin/env python3
"""RealSense + ChArUco 板 6DOF 姿态检测 -> ZMQ camera.detection 发布节点。

复用 examples/get_Aruco.py 的检测逻辑，把结果按项目公共 Header + CameraDetection
消息格式发布到 AVIATOR 总线（默认 PUB connect tcp://127.0.0.1:5555）。

相机型号/分辨率与 ChArUco 尺寸从 YAML 读取（默认 config/camera.yaml），
命令行参数只作为覆盖。运行环境：miniconda env `apriltag_realsense`。
"""

import argparse
import json
import os
import socket
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import zmq
from cv2 import aruco

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "camera.yaml")

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


# =========================
# Rodrigues -> Quaternion（qx, qy, qz, qw）
# =========================

def rvec_to_quaternion(rvec):
    R, _ = cv2.Rodrigues(rvec)
    qw = np.sqrt(max(0, 1 + np.trace(R))) / 2
    qx = np.sign(R[2, 1] - R[1, 2]) * np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    qy = np.sign(R[0, 2] - R[2, 0]) * np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    qz = np.sign(R[1, 0] - R[0, 1]) * np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


# =========================
# 配置加载
# =========================

def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        raise SystemExit(f"配置文件 {path} 顶层必须是映射")
    camera = cfg.get("camera", {})
    charuco = cfg.get("charuco", {})
    if not isinstance(camera, dict) or not isinstance(charuco, dict):
        raise SystemExit("配置文件的 camera / charuco 必须是映射")
    return camera, charuco


def required(mapping, key, where):
    """取必填配置项，缺失即报错（不写死回退值）。"""
    if key not in mapping:
        raise SystemExit(f"配置文件缺少 {where}.{key}")
    return mapping[key]


def normalize_board_size(value):
    """接受 [列, 行] 列表或 'NxM' 字符串，返回 (cols, rows)。"""
    if isinstance(value, (list, tuple)):
        if len(value) != 2:
            raise SystemExit("board_size 列表必须是 [列, 行]")
        return int(value[0]), int(value[1])
    if isinstance(value, str):
        try:
            w, h = value.lower().split("x")
            return int(w), int(h)
        except ValueError:
            raise SystemExit(f"invalid board_size: {value!r}")
    raise SystemExit(f"invalid board_size type: {type(value).__name__}")


def resolve_dictionary(name):
    try:
        return aruco.getPredefinedDictionary(getattr(aruco, name))
    except AttributeError:
        samples = [n for n in dir(aruco) if n.startswith("DICT_")][:8]
        raise SystemExit(f"未知字典 {name!r}；可用如 {', '.join(samples)} ...")


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
                print(f"camera_publisher: 按型号 {model!r} 选中 {name} ({chosen})")
                return chosen
        present = [d.get_info(rs.camera_info.name) for d in devices]
        raise RuntimeError(f"未找到型号包含 {model!r} 的设备；已连接: {present}")
    chosen = devices[0].get_info(rs.camera_info.serial_number)
    print(f"camera_publisher: 未指定型号/序列号，使用第一台 "
          f"{devices[0].get_info(rs.camera_info.name)} ({chosen})")
    return chosen


# =========================
# 消息组装
# =========================

def make_message(camera_id, frame_id, sequence, sample_mono_us,
                 session, clock, width, height, status, confidence, pose):
    """构造 CameraDetection JSON（公共头部 + 业务字段 + 补充 pose 块）。

    时间戳规则（见 docs/AVIATOR_ZMQ协议格式说明.md §12）：
    - sample_mono_us = 图像采集时刻（单调时钟）
    - timestamp      = 检测快照生成时刻（UTC）
    """
    return {
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


def parse_args(argv):
    p = argparse.ArgumentParser(description="RealSense ChArUco 姿态 -> camera.detection 发布")
    p.add_argument("--config", default=DEFAULT_CONFIG, help="YAML 配置文件路径")
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5555",
                   help="PUB 连接端点（AVIATOR publish_endpoint）")
    p.add_argument("--bind", action="store_true",
                   help="PUB 改为 bind 而非 connect（直连测试，绕过总线）")
    p.add_argument("--camera-id", default="cockpit_camera", help="camera_id 字段")
    p.add_argument("--show", action="store_true", help="开启 OpenCV 可视化窗口")
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
    camera, charuco = load_config(args.config)

    # 采集格式（CLI 覆盖 YAML；必填键缺失即报错，不写死回退值）
    width = args.width if args.width is not None else required(camera, "width", "camera")
    height = args.height if args.height is not None else required(camera, "height", "camera")
    fps = args.fps if args.fps is not None else required(camera, "fps", "camera")
    warmup_s = float(args.warmup_s if args.warmup_s is not None
                     else required(camera, "warmup_s", "camera"))
    model = camera.get("model", "")    # 空 = 自动选择
    serial = camera.get("serial", "")  # 空 = 自动选择

    # ChArUco 几何（CLI 覆盖 YAML；必填键缺失即报错，不写死回退值）
    bx, by = normalize_board_size(
        args.board_size if args.board_size is not None else required(charuco, "board_size", "charuco"))
    square_length = (args.square_length if args.square_length is not None
                     else required(charuco, "square_length", "charuco"))
    marker_length = (args.marker_length if args.marker_length is not None
                     else required(charuco, "marker_length", "charuco"))
    min_corners = (args.min_corners if args.min_corners is not None
                   else required(charuco, "min_corners", "charuco"))
    dictionary = resolve_dictionary(required(charuco, "dictionary", "charuco"))
    total_corners = bx * by

    # -------------------------
    # Camera（按型号/序列号选设备）
    # -------------------------
    context_rs = rs.context()
    device_serial = resolve_device(context_rs, model, serial)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)

    device = profile.get_device()
    device_name = device.get_info(rs.camera_info.name)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.asarray(intrinsics.coeffs, dtype=np.float32).reshape(-1, 1)

    # -------------------------
    # ChArUco
    # -------------------------
    board = aruco.CharucoBoard((bx, by), square_length, marker_length, dictionary)
    detector = aruco.CharucoDetector(board)

    # -------------------------
    # ZMQ PUB
    # -------------------------
    session = session_id()
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

    print(f"camera_publisher: {device_name} ({device_serial}) {width}x{height}@{fps}")
    print(f"camera_publisher: board={bx}x{by} square={square_length} marker={marker_length} "
          f"dict={charuco['dictionary']} min_corners={min_corners}")
    print(f"camera_publisher: endpoint={args.endpoint} bind={args.bind}")
    print(f"camera_publisher: session={session} clock={clock} warmup_s={warmup_s}")

    warmup_until = monotonic_us() + int(warmup_s * 1e6)
    sequence = 0
    frame_id = 0
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame_id += 1
            sample_mono_us = monotonic_us()
            image = np.asanyarray(color_frame.get_data())
            w, h = color_frame.get_width(), color_frame.get_height()

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            status = "SEARCHING"
            confidence = 0.0
            pose = None
            if charuco_ids is not None:
                confidence = float(min(len(charuco_ids), total_corners)) / total_corners

            if charuco_ids is not None and len(charuco_ids) >= min_corners:
                obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
                success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
                if success:
                    status = "TRACKING"
                    x, y, z = (tvec.flatten() * 1000.0).astype(float)
                    qx, qy, qz, qw = rvec_to_quaternion(rvec).astype(float)
                    pose = {
                        "position": {"x": float(x), "y": float(y), "z": float(z)},
                        "orientation": {"qx": float(qx), "qy": float(qy),
                                        "qz": float(qz), "qw": float(qw)},
                    }

            if monotonic_us() >= warmup_until:
                sequence += 1
                message = make_message(args.camera_id, frame_id, sequence, sample_mono_us,
                                       session, clock, w, h, status, confidence, pose)
                payload = json.dumps(message)
                pub.send_multipart([b"camera.detection", payload.encode("utf-8")])

            if args.show:
                display = image.copy()
                if marker_ids is not None:
                    cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
                if charuco_ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)
                if pose is not None:
                    cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                    cv2.putText(display, f"{status} seq={sequence} conf={confidence:.2f}",
                                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.imshow("aruco_camera publisher", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        pub.close()
        context.term()
        print("camera_publisher: stopped")


if __name__ == "__main__":
    main(sys.argv[1:])
