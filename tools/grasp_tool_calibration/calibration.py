"""Static flange-to-tool calibration; distances in metres, rotations in radians.

All quaternions in this module use grasp.json's [w, x, y, z] convention.
No robot I/O or file mutation occurs until save_grasp is explicitly called.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
from numbers import Real
import os
from pathlib import Path
import stat
import tempfile
from typing import Any

import numpy as np


class CalibrationError(ValueError):
    """Invalid, moving, or incomplete calibration data, or a failed save."""


def _number(value: Any, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise CalibrationError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise CalibrationError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise CalibrationError(f"{name} must be a finite number")
    return result


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    if (not isinstance(value, (list, tuple, np.ndarray))
            or (isinstance(value, np.ndarray) and value.ndim != 1)
            or len(value) != size):
        raise CalibrationError(f"{name} must contain {size} finite numbers")
    return np.array([_number(item, name) for item in value], dtype=float)


def _wheel(angle: Any, displacement: Any) -> tuple[float, float]:
    angle = _number(angle, "angle_rad")
    displacement = _number(displacement, "displacement_m")
    if not -0.87266 <= angle <= 0.87266:
        raise CalibrationError("angle_rad must be in [-0.87266, 0.87266]")
    if not -0.170 <= displacement <= 0:
        raise CalibrationError("displacement_m must be in [-0.170, 0]")
    return angle, displacement


def camera_wheel(theta_rad: Any, translation_along_axis_m: Any) -> tuple[float, float]:
    """Convert camera measurements to Core coordinates without hiding outliers."""
    return _wheel(-_number(theta_rad, "theta_rad"),
                  -_number(translation_along_axis_m, "translation_along_axis_m") - 0.085)


def pose_matrix(pose: dict) -> np.ndarray:
    if not isinstance(pose, dict):
        raise CalibrationError("pose must be an object with position and quaternion")
    p = _vector(pose.get("position"), 3, "position")
    q = _vector(pose.get("quaternion"), 4, "quaternion [w,x,y,z]")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1) > 1e-6:
        raise CalibrationError("quaternion must have unit norm (tolerance 1e-6)")
    w, x, y, z = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ]
    result[:3, 3] = p
    return result


def matrix_pose(matrix: Any) -> dict:
    try:
        value = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise CalibrationError("pose matrix must be numeric") from error
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise CalibrationError("pose matrix must be finite 4x4")
    r = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0)
            or abs(float(np.linalg.det(r)) - 1) > 1e-8):
        raise CalibrationError("pose matrix must be a rigid transform with proper rotation")
    # Choose the largest quaternion component to remain stable near 180 degrees.
    candidates = np.array([1 + np.trace(r), 1 + r[0, 0] - r[1, 1] - r[2, 2],
                           1 - r[0, 0] + r[1, 1] - r[2, 2],
                           1 - r[0, 0] - r[1, 1] + r[2, 2]])
    index = int(np.argmax(candidates))
    scale = 2 * math.sqrt(max(0, float(candidates[index])))
    if index == 0:
        q = np.array([scale / 4, (r[2, 1] - r[1, 2]) / scale,
                      (r[0, 2] - r[2, 0]) / scale, (r[1, 0] - r[0, 1]) / scale])
    elif index == 1:
        q = np.array([(r[2, 1] - r[1, 2]) / scale, scale / 4,
                      (r[0, 1] + r[1, 0]) / scale, (r[0, 2] + r[2, 0]) / scale])
    elif index == 2:
        q = np.array([(r[0, 2] - r[2, 0]) / scale, (r[0, 1] + r[1, 0]) / scale,
                      scale / 4, (r[1, 2] + r[2, 1]) / scale])
    else:
        q = np.array([(r[1, 0] - r[0, 1]) / scale, (r[0, 2] + r[2, 0]) / scale,
                      (r[1, 2] + r[2, 1]) / scale, scale / 4])
    q /= np.linalg.norm(q)
    if q[int(np.argmax(np.abs(q)))] < 0:
        q = -q
    return {"position": value[:3, 3].tolist(), "quaternion": q.tolist()}


def _rotation_distance(a: np.ndarray, b: np.ndarray) -> float:
    relative = a.T @ b
    sin_angle = 0.5 * float(np.linalg.norm([
        relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1]]))
    cos_angle = max(-1.0, min(1.0, (float(np.trace(relative)) - 1) / 2))
    return math.atan2(sin_angle, cos_angle)


def _average(matrices: list[np.ndarray]) -> np.ndarray:
    quaternions = np.array([matrix_pose(t)["quaternion"] for t in matrices])
    _, vectors = np.linalg.eigh(quaternions.T @ quaternions)
    result = pose_matrix({"position": np.mean([t[:3, 3] for t in matrices], axis=0),
                          "quaternion": vectors[:, -1]})
    return result


def _old_tools(grasp: dict) -> dict[str, np.ndarray]:
    tool = grasp.get("tool")
    if not isinstance(tool, dict):
        raise CalibrationError("grasp.tool must be an object")
    if "left" in tool or "right" in tool:
        if "left" not in tool or "right" not in tool:
            raise CalibrationError("grasp.tool requires both left and right transforms")
        if "position" in tool or "quaternion" in tool:
            raise CalibrationError("grasp.tool cannot mix shared and per-arm transforms")
        return {side: pose_matrix(tool[side]) for side in ("left", "right")}
    shared = pose_matrix(tool)
    return {"left": shared, "right": shared}


def validate_grasp(grasp: dict) -> None:
    """Validate configuration before opening SDK/camera data sources."""
    if not isinstance(grasp, dict):
        raise CalibrationError("grasp must be an object")
    try:
        json.dumps(grasp, allow_nan=False)
    except (TypeError, ValueError, OverflowError) as error:
        raise CalibrationError(f"grasp must contain only finite JSON data: {error}") from error
    _old_tools(grasp)
    pose_matrix(grasp.get("wheel_origin"))
    for side in ("left", "right"):
        pose_matrix(grasp.get(side))


def calibrate(grasp: dict, samples: list[dict], *, max_joint_span_rad: float,
              max_position_span_m: float, max_rotation_span_rad: float,
              max_wheel_angle_span_rad: float,
              max_wheel_displacement_span_m: float) -> tuple[dict, dict]:
    """Infer independent tool poses from stationary measured flanges and wheel.

    Each sample contains left/right objects with seven joint_position values and
    a flange pose, plus angle_rad and displacement_m in Core coordinates.
    Span limits are nonnegative; all samples must pass before returning results.
    """
    limits = {
        "max_joint_span_rad": max_joint_span_rad,
        "max_position_span_m": max_position_span_m,
        "max_rotation_span_rad": max_rotation_span_rad,
        "max_wheel_angle_span_rad": max_wheel_angle_span_rad,
        "max_wheel_displacement_span_m": max_wheel_displacement_span_m,
    }
    for key, value in limits.items():
        limits[key] = _number(value, key)
        if limits[key] < 0:
            raise CalibrationError(f"{key} must be nonnegative")
    validate_grasp(grasp)
    if not isinstance(samples, list) or len(samples) < 3:
        raise CalibrationError("at least three paired samples are required")
    old = _old_tools(grasp)
    origin = pose_matrix(grasp.get("wheel_origin"))
    handles = {side: pose_matrix(grasp.get(side)) for side in ("left", "right")}
    wheels = []
    targets = {side: [] for side in handles}
    flanges = {side: [] for side in handles}
    joints = {side: [] for side in handles}
    tools = {side: [] for side in handles}
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise CalibrationError(f"sample {index} must be an object")
        angle, displacement = _wheel(sample.get("angle_rad"), sample.get("displacement_m"))
        wheels.append([angle, displacement])
        moving = pose_matrix({"position": [0, 0, displacement],
                              "quaternion": [math.cos(angle/2), 0, 0, math.sin(angle/2)]})
        for side in handles:
            arm = sample.get(side)
            if not isinstance(arm, dict):
                raise CalibrationError(f"sample {index}.{side} must be an object")
            joints[side].append(_vector(arm.get("joint_position"), 7,
                                        f"sample {index}.{side}.joint_position"))
            flange = pose_matrix(arm.get("flange"))
            target = origin @ moving @ handles[side]
            flanges[side].append(flange)
            targets[side].append(target)
            tools[side].append(np.linalg.solve(flange, target))
    wheels = np.array(wheels)
    wheel_span = np.ptp(wheels, axis=0)
    if wheel_span[0] > limits["max_wheel_angle_span_rad"]:
        raise CalibrationError(f"wheel angle is not static: span {wheel_span[0]:.6g} rad")
    if wheel_span[1] > limits["max_wheel_displacement_span_m"]:
        raise CalibrationError(f"wheel displacement is not static: span {wheel_span[1]:.6g} m")
    result = copy.deepcopy(grasp)
    # Upgrade the legacy shared representation, preserving physical metadata.
    result["tool"].pop("position", None)
    result["tool"].pop("quaternion", None)
    mean_wheel = np.mean(wheels, axis=0)
    report = {
        "sample_count": len(samples), "limits": limits,
        "wheel": {"mean_angle_rad": float(mean_wheel[0]),
                  "mean_displacement_m": float(mean_wheel[1]),
                  "angle_span_rad": float(wheel_span[0]),
                  "displacement_span_m": float(wheel_span[1])},
        "arms": {},
    }
    for side in handles:
        joint_span = np.ptp(joints[side], axis=0)
        position_span = 0.0
        rotation_span = 0.0
        for i, a in enumerate(flanges[side]):
            for b in flanges[side][i + 1:]:
                position_span = max(position_span, float(np.linalg.norm(a[:3, 3] - b[:3, 3])))
                rotation_span = max(rotation_span, _rotation_distance(a[:3, :3], b[:3, :3]))
        for value, limit_key, label in (
            (float(np.max(joint_span)), "max_joint_span_rad", "joint positions (rad)"),
            (position_span, "max_position_span_m", "flange positions (m)"),
            (rotation_span, "max_rotation_span_rad", "flange rotations (rad)"),
        ):
            if value > limits[limit_key]:
                raise CalibrationError(f"{side} {label} are not static: span {value:.6g}, "
                                       f"limit {limits[limit_key]:.6g}")
        mean_tool = _average(tools[side])
        new_pose = matrix_pose(mean_tool)
        # Preserve any per-arm metadata that accompanied the original pose.
        if side not in result["tool"]:
            result["tool"][side] = {}
        result["tool"][side].update(new_pose)
        delta = mean_tool[:3, 3] - old[side][:3, 3]
        reconstructed = [target @ np.linalg.inv(mean_tool) for target in targets[side]]
        report["arms"][side] = {
            "old": matrix_pose(old[side]), "new": new_pose,
            "delta_position_m": delta.tolist(),
            "delta_position_norm_m": float(np.linalg.norm(delta)),
            "delta_rotation_rad": _rotation_distance(old[side][:3, :3], mean_tool[:3, :3]),
            "joint_span_rad": joint_span.tolist(),
            "max_joint_span_rad": float(np.max(joint_span)),
            "position_span_m": position_span, "rotation_span_rad": rotation_span,
            "reconstruction_max_position_error_m": max(
                float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
                for a, b in zip(reconstructed, flanges[side])),
            "reconstruction_max_rotation_error_rad": max(
                _rotation_distance(a[:3, :3], b[:3, :3])
                for a, b in zip(reconstructed, flanges[side])),
        }
    return result, report


def save_grasp(path: Path, updated: dict, original_bytes: bytes) -> Path:
    """Back up the exact input and atomically replace it; reject stale input.

    This protects against edits made during collection. Like ordinary atomic
    saves, it does not provide coordination with another simultaneous writer.
    """
    path = Path(path)
    temporary = None
    try:
        encoded = (json.dumps(updated, ensure_ascii=False, indent=2, allow_nan=False)
                   + "\n").encode("utf-8")
        if not path.is_file() or path.is_symlink():
            raise CalibrationError("grasp path must be an existing regular file, not a symlink")
        if path.read_bytes() != original_bytes:
            raise CalibrationError("grasp file changed since it was loaded; refusing to overwrite")
        mode = stat.S_IMODE(path.stat().st_mode)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        backup = path.with_name(f"{path.name}.{stamp}.bak")
        with backup.open("xb") as stream:
            os.chmod(backup, mode)
            stream.write(original_bytes)
            stream.flush()
            os.fsync(stream.fileno())
        fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
        with os.fdopen(fd, "wb") as stream:
            os.fchmod(stream.fileno(), mode)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if path.read_bytes() != original_bytes:
            raise CalibrationError("grasp file changed during save; refusing to overwrite")
        os.replace(temporary, path)
        temporary = None
        return backup
    except (OSError, TypeError, ValueError) as error:
        if isinstance(error, CalibrationError):
            raise
        raise CalibrationError(f"cannot save grasp configuration: {error}") from error
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except OSError:
                pass
