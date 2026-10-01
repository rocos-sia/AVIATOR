"""Rigid-transform geometry used by the steering-wheel calibration tools.

Camera detections use a target-to-camera pose.  Therefore the relative motion
from a zero pose to a later pose is ``T_current @ inverse(T_zero)``.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np


class CalibrationError(ValueError):
    """Raised when a pose or calibration is incomplete or geometrically invalid."""


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise CalibrationError(f"{name} must be a finite vector of length {size}")
    return array


def quaternion_to_rotation(quaternion: Any) -> np.ndarray:
    qx, qy, qz, qw = _finite_vector(quaternion, 4, "orientation")
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm < 1e-12:
        raise CalibrationError("orientation quaternion has zero norm")
    qx, qy, qz, qw = np.array([qx, qy, qz, qw]) / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=float)


def rotation_to_quaternion(rotation: Any) -> list[float]:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise CalibrationError("rotation must be a finite 3x3 matrix")
    u, _, vh = np.linalg.svd(matrix)
    matrix = u @ vh
    if np.linalg.det(matrix) < 0:
        u[:, -1] *= -1
        matrix = u @ vh
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = 2 * math.sqrt(trace + 1)
        qw = scale / 4
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2 * math.sqrt(max(0, 1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
            qx = scale / 4
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
        elif index == 1:
            scale = 2 * math.sqrt(max(0, 1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = scale / 4
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = 2 * math.sqrt(max(0, 1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = scale / 4
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return [float(value) for value in quaternion]


def pose_to_matrix(pose: dict[str, Any]) -> np.ndarray:
    if not isinstance(pose, dict):
        raise CalibrationError("pose must be an object")
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise CalibrationError("pose requires position and orientation objects")
    translation = _finite_vector([position.get(axis) for axis in ("x", "y", "z")], 3, "position")
    quaternion = [orientation.get(axis) for axis in ("qx", "qy", "qz", "qw")]
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_rotation(quaternion)
    matrix[:3, 3] = translation
    return matrix


def matrix_to_pose(matrix: Any) -> dict[str, Any]:
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise CalibrationError("pose matrix must be a finite 4x4 matrix")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7):
        raise CalibrationError("pose matrix has an invalid homogeneous row")
    return {
        "position": {axis: float(value[i, 3]) for i, axis in enumerate(("x", "y", "z"))},
        "orientation": dict(zip(("qx", "qy", "qz", "qw"), rotation_to_quaternion(value[:3, :3]))),
    }


def matrix_as_lists(matrix: Any) -> list[list[float]]:
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise CalibrationError("pose matrix must be a finite 4x4 matrix")
    return [[float(item) for item in row] for row in value]


def average_pose_matrices(matrices: Any) -> np.ndarray:
    """Average translations and rotations from a batch of pose matrices.

    Translation is averaged component-wise.  Orientation uses the Markley
    quaternion mean, which treats q and -q as the same rotation and avoids
    averaging Euler angles or matrix entries directly.
    """
    values = np.asarray(matrices, dtype=float)
    if values.ndim != 3 or values.shape[1:] != (4, 4) or values.shape[0] == 0:
        raise CalibrationError("pose batch must contain one or more 4x4 matrices")
    if not np.all(np.isfinite(values)):
        raise CalibrationError("pose batch contains non-finite values")
    translations = values[:, :3, 3]
    quaternions = np.asarray([rotation_to_quaternion(value[:3, :3]) for value in values])
    accumulator = np.einsum("ni,nj->ij", quaternions, quaternions)
    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = np.real(eigenvectors[:, int(np.argmax(eigenvalues))])
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise CalibrationError("orientation batch has no stable mean")
    quaternion /= norm
    if quaternion[3] < 0:
        quaternion = -quaternion
    result = np.eye(4, dtype=float)
    result[:3, :3] = quaternion_to_rotation(quaternion)
    result[:3, 3] = np.mean(translations, axis=0)
    return result


def relative_transform(zero: Any, current: Any) -> np.ndarray:
    zero_matrix = np.asarray(zero, dtype=float)
    current_matrix = np.asarray(current, dtype=float)
    if zero_matrix.shape != (4, 4) or current_matrix.shape != (4, 4):
        raise CalibrationError("relative transforms require two 4x4 matrices")
    return current_matrix @ np.linalg.inv(zero_matrix)


def _canonical_axis(axis: np.ndarray) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    # A calibration needs a stable sign.  The largest component is positive;
    # this removes the arbitrary sign returned by eigenvector calculations.
    pivot = int(np.argmax(np.abs(axis)))
    if axis[pivot] < 0:
        axis = -axis
    return axis


def rotation_axis(rotation: Any, fallback_translation: Any | None = None) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise CalibrationError("rotation must be a finite 3x3 matrix")
    skew = np.array([matrix[2, 1] - matrix[1, 2],
                     matrix[0, 2] - matrix[2, 0],
                     matrix[1, 0] - matrix[0, 1]], dtype=float) / 2
    sine = float(np.linalg.norm(skew))
    if sine > 1e-8:
        return _canonical_axis(skew)
    # Near pi the skew vector vanishes.  The eigenvector with eigenvalue 1 is
    # the rotation axis; this also handles a numerically exact half-turn.
    cosine = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1) / 2))
    if cosine < -1 + 1e-6:
        eigenvalues, eigenvectors = np.linalg.eig(matrix)
        index = int(np.argmin(np.abs(eigenvalues - 1)))
        if abs(eigenvalues[index].imag) < 1e-7 and abs(eigenvalues[index].real - 1) < 1e-5:
            candidate = np.real(eigenvectors[:, index])
            if np.linalg.norm(candidate) > 1e-8:
                return _canonical_axis(candidate)
    if fallback_translation is not None:
        translation = _finite_vector(fallback_translation, 3, "translation")
        if np.linalg.norm(translation) > 1e-8:
            return _canonical_axis(translation)
    raise CalibrationError("cannot determine a motion axis from this transform")


def screw_parameters(relative: Any, axis_hint: Any | None = None) -> dict[str, Any]:
    transform = np.asarray(relative, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise CalibrationError("relative transform must be a finite 4x4 matrix")
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    axis = _canonical_axis(_finite_vector(axis_hint, 3, "axis_hint")) if axis_hint is not None \
        else rotation_axis(rotation, translation)
    skew = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]], dtype=float) / 2
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1) / 2))
    angle = math.atan2(float(np.dot(axis, skew)), cosine)
    axial_translation = float(np.dot(axis, translation))
    perpendicular = translation - axial_translation * axis
    point = None
    if abs(angle) > 1e-8:
        # Solve (I-R)p = t - d*u and choose the point closest to the origin.
        point = np.linalg.lstsq(np.eye(3) - rotation,
                                translation - axial_translation * axis, rcond=None)[0]
        point = point - np.dot(point, axis) * axis
    return {
        "axis_direction": axis,
        "axis_point": point,
        "rotation_angle_rad": angle,
        "translation_vector": translation,
        "translation_along_axis_m": axial_translation,
        "translation_perpendicular_m": perpendicular,
        "translation_perpendicular_norm_m": float(np.linalg.norm(perpendicular)),
    }


def calibrate_from_poses(zero: np.ndarray, second: np.ndarray) -> dict[str, Any]:
    relative = relative_transform(zero, second)
    motion = screw_parameters(relative)
    motion["relative_transform"] = relative
    return motion


def evaluate_pose(zero: np.ndarray, current: np.ndarray, axis: Any) -> dict[str, Any]:
    relative = relative_transform(zero, current)
    motion = screw_parameters(relative, axis_hint=axis)
    # Estimate the axis independently from the current point and the zero
    # point.  Keep the stored calibration axis as the hint for the reported
    # screw parameters, but expose this independent estimate so validation can
    # detect a tag/camera setup that no longer follows the calibrated axis.
    try:
        measured_axis = rotation_axis(relative[:3, :3], relative[:3, 3])
    except CalibrationError:
        measured_axis = None
    stored_axis = _canonical_axis(_finite_vector(axis, 3, "axis_hint"))
    axis_error_rad = None
    if measured_axis is not None:
        cosine = float(np.clip(np.dot(measured_axis, stored_axis), -1.0, 1.0))
        axis_error_rad = float(math.acos(cosine))
    motion["axis_direction"] = stored_axis
    motion["measured_axis_direction"] = measured_axis
    motion["axis_error_rad"] = axis_error_rad
    motion["relative_transform"] = relative
    return motion
