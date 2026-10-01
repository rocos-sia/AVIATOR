"""Shared calibration YAML loader for the camera publisher and validation CLI."""
from pathlib import Path
from typing import Any

import numpy as np
import yaml

if __package__:
    from .geometry import CalibrationError
else:
    from geometry import CalibrationError


def load_calibration(path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    calibration = document.get("steering_wheel_calibration") if isinstance(document, dict) else None
    if not isinstance(calibration, dict) or calibration.get("status") != "calibrated":
        raise CalibrationError(f"{path} 尚未包含 calibrated 标定结果")
    zero_section, motion = calibration.get("zero"), calibration.get("motion")
    if not isinstance(zero_section, dict) or not isinstance(motion, dict):
        raise CalibrationError("标定 YAML 缺少 zero 或 motion 对象")
    zero_record = zero_section.get("pose")
    zero = zero_record.get("T_camera_tag") if isinstance(zero_record, dict) else None
    axis = motion.get("axis_direction")
    if zero is None or axis is None:
        raise CalibrationError("标定 YAML 缺少 zero.pose.T_camera_tag 或 motion.axis_direction")
    matrix = np.asarray(zero, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise CalibrationError("zero pose matrix must be finite and 4x4")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7):
        raise CalibrationError("zero pose matrix has an invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1):
        raise CalibrationError("zero pose must contain a proper rotation")
    axis_array = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis_array))
    if axis_array.shape != (3,) or not np.all(np.isfinite(axis_array)) or not np.isfinite(norm) or norm < 1e-8:
        raise CalibrationError("motion.axis_direction must be a non-zero finite vector")
    if not isinstance(calibration.get("source", {}), dict):
        raise CalibrationError("calibration.source must be an object")
    return calibration, matrix, axis_array / norm
