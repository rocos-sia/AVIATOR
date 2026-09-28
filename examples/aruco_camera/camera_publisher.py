#!/usr/bin/env python3
"""RealSense + ChArUco 板 6DOF 姿态检测 -> ZMQ camera.detection 发布节点。

复用 examples/get_Aruco.py 的检测逻辑，把结果按项目公共 Header + CameraDetection
消息格式发布到 AVIATOR 总线（默认 PUB connect tcp://127.0.0.1:5555）。

运行环境：miniconda env `apriltag_realsense`（已装 zmq + cv2 + pyrealsense2）。
"""

import argparse
import json
import socket
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq
from cv2 import aruco

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
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5555",
                   help="PUB 连接端点（AVIATOR publish_endpoint）")
    p.add_argument("--bind", action="store_true",
                   help="PUB 改为 bind 而非 connect（直连测试，绕过总线）")
    p.add_argument("--camera-id", default="cockpit_camera", help="camera_id 字段")
    p.add_argument("--show", action="store_true", help="开启 OpenCV 可视化窗口")
    p.add_argument("--min-corners", type=int, default=6,
                   help="solvePnP 所需最少角点（默认 6，与 get_Aruco.py 一致）")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--square-length", type=float, default=0.016)
    p.add_argument("--marker-length", type=float, default=0.015)
    p.add_argument("--board-size", default="5x5", help="ChArUco 板格子数，如 5x5")
    return p.parse_args(argv)


def parse_board_size(text):
    try:
        w, h = text.lower().split("x")
        return int(w), int(h)
    except ValueError:
        raise SystemExit("invalid --board-size, expected NxM")


def main(argv):
    args = parse_args(argv)
    bx, by = parse_board_size(args.board_size)
    total_corners = bx * by

    # -------------------------
    # Camera
    # -------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    profile = pipeline.start(config)

    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.asarray(intrinsics.coeffs, dtype=np.float32).reshape(-1, 1)

    # -------------------------
    # ChArUco
    # -------------------------
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    board = aruco.CharucoBoard((bx, by), args.square_length, args.marker_length, dictionary)
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

    print(f"camera_publisher: {args.width}x{args.height}@{args.fps} "
          f"board={bx}x{by} endpoint={args.endpoint} bind={args.bind}")
    print(f"camera_publisher: session={session} clock={clock}")

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
            width, height = color_frame.get_width(), color_frame.get_height()

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            status = "SEARCHING"
            confidence = 0.0
            pose = None
            if charuco_ids is not None:
                confidence = float(min(len(charuco_ids), total_corners)) / total_corners

            if charuco_ids is not None and len(charuco_ids) >= args.min_corners:
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

            sequence += 1
            message = make_message(args.camera_id, frame_id, sequence, sample_mono_us,
                                   session, clock, width, height, status, confidence, pose)
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
