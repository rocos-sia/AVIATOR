#!/usr/bin/env python3
"""Read camera.detection and report steering-wheel angle and axial translation."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from geometry import CalibrationError, evaluate_pose, pose_to_matrix


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] /
                        "config" / "steering_wheel_calibration.yaml")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--bind", action="store_true")
    parser.add_argument("--topic", default="camera.detection")
    parser.add_argument("--camera-id", default=None)
    parser.add_argument("--tag-id", type=int, default=None)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--max-msgs", type=int, default=0,
                        help="处理 N 个有效位姿后退出，0 表示持续运行")
    parser.add_argument("--axis-tolerance-deg", type=float, default=5.0,
                        help="轴向量一致性允许的夹角（度），默认 5 度")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出每个结果")
    return parser.parse_args(argv)


def load_calibration(path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    calibration = document.get("steering_wheel_calibration")
    if not isinstance(calibration, dict) or calibration.get("status") != "calibrated":
        raise CalibrationError(f"{path} 尚未包含 calibrated 标定结果")
    zero_record = calibration.get("zero", {}).get("pose")
    zero = zero_record.get("T_camera_tag") if isinstance(zero_record, dict) else None
    axis = calibration.get("motion", {}).get("axis_direction")
    if zero is None or axis is None:
        raise CalibrationError("标定 YAML 缺少 zero.pose.T_camera_tag 或 motion.axis_direction")
    matrix = np.asarray(zero, dtype=float)
    if matrix.shape != (4, 4):
        raise CalibrationError("zero pose matrix must be 4x4")
    axis_array = np.asarray(axis, dtype=float)
    if axis_array.shape != (3,) or not np.all(np.isfinite(axis_array)) or np.linalg.norm(axis_array) < 1e-8:
        raise CalibrationError("motion.axis_direction must be a non-zero finite vector")
    return calibration, matrix, axis_array / np.linalg.norm(axis_array)


def accepts(data: dict[str, Any], args: argparse.Namespace, calibration: dict[str, Any]) -> bool:
    source = calibration.get("source", {})
    camera_id = args.camera_id if args.camera_id is not None else source.get("camera_id")
    tag_id = args.tag_id if args.tag_id is not None else source.get("tag_id")
    return (
        data.get("msg_type") == "CameraDetection"
        and (camera_id is None or data.get("camera_id") == camera_id)
        and (tag_id is None or data.get("tag_id") == tag_id)
        and data.get("status") == "TRACKING"
        and data.get("valid") is True
        and isinstance(data.get("pose"), dict)
    )


def output_result(result: dict[str, Any], data: dict[str, Any], as_json: bool) -> None:
    measured_axis = result.get("measured_axis_direction")
    axis_error_rad = result.get("axis_error_rad")
    axis_error_deg = None if axis_error_rad is None else math.degrees(float(axis_error_rad))
    payload = {
        "frame_id": data.get("frame_id"),
        "sequence": data.get("sequence"),
        "theta_rad": float(result["rotation_angle_rad"]),
        "theta_deg": math.degrees(float(result["rotation_angle_rad"])),
        "translation_along_axis_m": float(result["translation_along_axis_m"]),
        "translation_vector_m": [float(value) for value in result["translation_vector"]],
        "translation_perpendicular_norm_m": float(result["translation_perpendicular_norm_m"]),
        "axis_direction_stored": [float(value) for value in result["axis_direction"]],
        "axis_direction_measured": (None if measured_axis is None else
                                     [float(value) for value in measured_axis]),
        "axis_error_deg": axis_error_deg,
        "axis_consistent": result.get("axis_consistent"),
    }
    if as_json:
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)
    else:
        translation = payload["translation_vector_m"]
        measured = payload["axis_direction_measured"]
        if payload["axis_error_deg"] is None:
            axis_check = "axis_match=unknown (insufficient motion)"
        else:
            axis_check = (f"axis_measured=[{measured[0]:.4f},{measured[1]:.4f},{measured[2]:.4f}] "
                          f"axis_error={payload['axis_error_deg']:.3f} deg "
                          f"axis_match={str(payload['axis_consistent']).lower()}")
        print(f"frame={payload['frame_id']} seq={payload['sequence']} "
              f"theta={payload['theta_deg']:.3f} deg "
              f"translation=[{translation[0]:.6f},{translation[1]:.6f},{translation[2]:.6f}] m "
              f"translation_axis={payload['translation_along_axis_m']:.6f} m "
              f"translation_norm={payload['translation_perpendicular_norm_m']:.6f} m "
              f"{axis_check}", flush=True)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.timeout_ms <= 0 or args.max_msgs < 0 or args.axis_tolerance_deg < 0:
        raise SystemExit("--timeout-ms 必须为正数，--max-msgs 不能为负数，"
                         "--axis-tolerance-deg 不能为负数")
    calibration, zero, axis = load_calibration(args.config)
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
    count = 0
    try:
        while args.max_msgs == 0 or count < args.max_msgs:
            if sub.poll(args.timeout_ms) == 0:
                continue
            frames = sub.recv_multipart()
            if len(frames) != 2 or frames[0].decode("utf-8", "replace") != args.topic:
                continue
            try:
                data = json.loads(frames[1].decode("utf-8"))
                if not isinstance(data, dict) or not accepts(data, args, calibration):
                    continue
                current = pose_to_matrix(data["pose"])
                result = evaluate_pose(zero, current, axis)
            except (UnicodeError, json.JSONDecodeError, CalibrationError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
                print(f"[skip] invalid detection: {exc}", file=sys.stderr)
                continue
            count += 1
            if result["axis_error_rad"] is not None:
                result["axis_consistent"] = (
                    math.degrees(float(result["axis_error_rad"])) <= args.axis_tolerance_deg)
            else:
                result["axis_consistent"] = None
            output_result(result, data, args.json)
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0)
        context.term()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CalibrationError, OSError, ValueError) as exc:
        print(f"steering_wheel_validation: {exc}", file=sys.stderr)
        raise SystemExit(1)
