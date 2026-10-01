#!/usr/bin/env python3
"""Record two camera.detection poses and write steering-wheel calibration YAML."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

if __package__:
    from .geometry import (CalibrationError, average_pose_matrices, calibrate_from_poses,
                           matrix_as_lists, matrix_to_pose, pose_to_matrix)
else:
    from geometry import (CalibrationError, average_pose_matrices, calibrate_from_poses,
                          matrix_as_lists, matrix_to_pose, pose_to_matrix)


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "steering_wheel_calibration.yaml"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5556",
                        help="camera.detection SUB 端点（默认 AVIATOR 总线输出）")
    parser.add_argument("--bind", action="store_true",
                        help="SUB bind 端点，用于不经过 bus 的直连测试")
    parser.add_argument("--topic", default="camera.detection")
    parser.add_argument("--camera-id", default="cockpit")
    parser.add_argument("--tag-id", type=int, default=None,
                        help="只接受指定 AprilTag ID；省略则不按 tag_id 过滤")
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--samples", type=int, default=20,
                        help="每个标定位姿采集的有效帧数，默认 20")
    parser.add_argument("--sample-interval-ms", type=int, default=20,
                        help="同一标定位姿两次采样的最小间隔，默认 20 ms")
    parser.add_argument("--frame-id", default="camera_color_optical_frame",
                        help="写入 YAML 的轴坐标系名称；pose 本身使用相机坐标系")
    parser.add_argument("--output", type=Path, default=DEFAULT_CONFIG)
    return parser.parse_args(argv)


def valid_detection(data: dict[str, Any], args: argparse.Namespace) -> bool:
    try:
        confidence = float(data.get("confidence", 0))
    except (TypeError, ValueError):
        return False
    return (data.get("msg_type") == "CameraDetection"
            and data.get("camera_id") == args.camera_id
            and data.get("status") == "TRACKING"
            and data.get("valid") is True
            and isinstance(data.get("pose"), dict)
            and math.isfinite(confidence)
            and confidence >= args.min_confidence
            and (args.tag_id is None or data.get("tag_id") == args.tag_id))


def receive_poses(sub: Any, zmq: Any, args: argparse.Namespace,
                  prompt: str) -> tuple[list[dict[str, Any]], np.ndarray]:
    print(prompt, flush=True)
    not_before = time.monotonic_ns() // 1000
    records: list[dict[str, Any]] = []
    matrices: list[np.ndarray] = []
    last_sample = not_before - args.sample_interval_ms * 1000
    while len(records) < args.samples:
        if sub.poll(args.timeout_ms) == 0:
            print("等待有效 TRACKING 位姿...", flush=True)
            continue
        frames = sub.recv_multipart()
        if len(frames) != 2 or frames[0].decode("utf-8", "replace") != args.topic:
            continue
        try:
            data = json.loads(frames[1].decode("utf-8"))
            if not isinstance(data, dict) or not valid_detection(data, args):
                continue
            sample = data.get("sample_mono_us")
            if (isinstance(sample, bool) or not isinstance(sample, int) or
                    sample < not_before or sample < last_sample + args.sample_interval_ms * 1000):
                continue
            matrix = pose_to_matrix(data["pose"])
        except (UnicodeError, json.JSONDecodeError, CalibrationError, TypeError, ValueError):
            continue
        records.append(data)
        matrices.append(matrix)
        last_sample = sample
        print(f"已记录 {len(records)}/{args.samples} frame_id={data.get('frame_id')} "
              f"confidence={data.get('confidence', 0):.3f}", flush=True)
    average = average_pose_matrices(matrices)
    return records, average


def pose_record(records: list[dict[str, Any]], matrix: np.ndarray) -> dict[str, Any]:
    confidences = [float(item.get("confidence", 0)) for item in records]
    samples = [item.get("sample_mono_us") for item in records]
    return {
        "sample_count": len(records),
        "frame_id_first": records[0].get("frame_id"),
        "frame_id_last": records[-1].get("frame_id"),
        "sample_mono_us_first": samples[0],
        "sample_mono_us_last": samples[-1],
        "confidence_mean": float(np.mean(confidences)),
        "confidence_min": float(np.min(confidences)),
        "average_method": {
            "translation": "componentwise_arithmetic_mean",
            "orientation": "markley_quaternion_mean",
        },
        "pose": matrix_to_pose(matrix),
        "T_camera_tag": matrix_as_lists(matrix),
    }


def load_document(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise CalibrationError("标定 YAML 顶层必须是映射")
    return document


def write_document(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
    os.replace(temporary, path)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if (args.timeout_ms <= 0 or args.samples <= 0 or args.sample_interval_ms < 0 or
            not 0 <= args.min_confidence <= 1):
        raise SystemExit("--timeout-ms 必须为正数，--samples 必须为正数，"
                         "--sample-interval-ms 不能为负数，--min-confidence 必须在 [0,1]")
    try:
        import zmq
    except ImportError as exc:
        raise SystemExit("需要 pyzmq；请使用 camera 节点的 Python 环境运行") from exc

    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 8)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.MAXMSGSIZE, 65536)
    sub.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
    (sub.bind if args.bind else sub.connect)(args.endpoint)
    print(f"calibrate: topic={args.topic} endpoint={args.endpoint} bind={args.bind}")
    try:
        time.sleep(0.3)
        input("保持方向盘在零位，按回车后记录第一帧... ")
        zero_records, zero_matrix = receive_poses(sub, zmq, args, "正在等待零位有效位姿")
        input("把方向盘移动到第二个标定位姿，保持稳定后按回车... ")
        second_records, second_matrix = receive_poses(sub, zmq, args, "正在等待第二个有效位姿")
        motion = calibrate_from_poses(zero_matrix, second_matrix)
        document = load_document(args.output)
        document["steering_wheel_calibration"] = {
            "schema_version": 1,
            "status": "calibrated",
            "source": {
                "topic": args.topic,
                "camera_id": args.camera_id,
                "tag_id": args.tag_id,
                "frame_id": args.frame_id,
                "pose_convention": "T_camera_tag = target_to_camera",
                "position_unit": "m",
                "samples_per_pose": args.samples,
                "sample_interval_ms": args.sample_interval_ms,
                "average_method": {
                    "translation": "componentwise_arithmetic_mean",
                    "orientation": "markley_quaternion_mean",
                },
            },
            "zero": {
                "theta_rad": 0.0,
                "translation_along_axis_m": 0.0,
                "pose": pose_record(zero_records, zero_matrix),
            },
            "second": {"pose": pose_record(second_records, second_matrix)},
            "motion": {
                "axis_frame": args.frame_id,
                "axis_direction": [float(value) for value in motion["axis_direction"]],
                "axis_point_m": None if motion["axis_point"] is None else
                    [float(value) for value in motion["axis_point"]],
                "rotation_between_samples_rad": float(motion["rotation_angle_rad"]),
                "translation_vector_between_samples_m":
                    [float(value) for value in motion["translation_vector"]],
                "translation_along_axis_between_samples_m":
                    float(motion["translation_along_axis_m"]),
                "translation_perpendicular_between_samples_m":
                    [float(value) for value in motion["translation_perpendicular_m"]],
                "translation_perpendicular_norm_m":
                    float(motion["translation_perpendicular_norm_m"]),
            },
        }
        write_document(args.output, document)
        print(f"标定已写入 {args.output}")
        print("axis_direction=", document["steering_wheel_calibration"]["motion"]["axis_direction"])
        print("rotation_between_samples_rad=", motion["rotation_angle_rad"])
        print("translation_along_axis_m=", motion["translation_along_axis_m"])
        return 0
    finally:
        sub.close(0)
        context.term()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CalibrationError, OSError, ValueError) as exc:
        print(f"steering_wheel_calibration: {exc}", file=sys.stderr)
        raise SystemExit(1)
