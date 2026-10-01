"""Add calibrated physical steering-wheel motion to a raw detection message."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

# The camera runs directly from this source tree. Import the same geometry as
# the standalone tools, without loading their CLI or making a second copy.
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))
from tools.steering_wheel_calibration.calibration import load_calibration
from tools.steering_wheel_calibration.geometry import evaluate_pose, pose_to_matrix


def steering_wheel_options(settings, camera_config, override=None):
    if not isinstance(settings, dict):
        raise ValueError("steering_wheel must be a mapping")
    enabled = settings.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("steering_wheel.enabled must be boolean")
    name = override if override is not None else settings.get("calibration_file", "steering_wheel_calibration.yaml")
    if not isinstance(name, str) or not name:
        raise ValueError("steering_wheel.calibration_file must be a nonempty path")
    path = Path(name).expanduser()
    # Explicit CLI paths follow the caller's cwd; YAML paths follow the config.
    if not path.is_absolute() and override is None:
        path = Path(camera_config).resolve().parent / path
    return dict(calibration_file=path.resolve(), enabled=enabled)


class SteeringWheelObservation:
    def __init__(self, calibration_file, enabled=True):
        self.path = Path(calibration_file)
        self.enabled = enabled
        self.calibration = self.zero = self.axis = None
        self.calibration_id = None
        self.error = ""
        self._signature = None
        self._next_check = 0.0

    def _refresh(self):
        now = time.monotonic()
        if now < self._next_check:
            return
        self._next_check = now + 1.0
        try:
            stat = self.path.stat()
            signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            if signature == self._signature:
                return
            self._signature = signature
            calibration, zero, axis = load_calibration(self.path)
            identity = {"source": calibration.get("source", {}), "zero": zero.tolist(), "axis": axis.tolist()}
            calibration_id = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:16]
            self.calibration, self.zero, self.axis = calibration, zero, axis
            self.calibration_id = calibration_id
            self.error = ""
        except (OSError, ValueError, TypeError, AttributeError, yaml.YAMLError) as exc:
            # A missing/invalid new calibration must not silently reuse the old
            # model, and must not prevent publishing poses for recalibration.
            self.calibration = self.zero = self.axis = None
            self.calibration_id = None
            self.error = str(exc)
            if isinstance(exc, OSError):
                self._signature = None

    def observe(self, detection):
        result = {"valid": False, "reason": "disabled", "theta_rad": None,
                  "translation_along_axis_m": None, "translation_vector_m": None,
                  "axis_direction": None, "axis_frame": None,
                  "axis_error_rad": None, "axis_match": None, "calibration_id": None}
        if not self.enabled:
            return result
        self._refresh()
        if self.calibration is None:
            result.update(reason="calibration_unavailable", calibration_error=self.error)
            return result
        source = self.calibration.get("source", {})
        result.update(calibration_id=self.calibration_id,
                      axis_frame=self.calibration.get("motion", {}).get("axis_frame",
                                                                       source.get("frame_id")),
                      axis_direction=self.axis.tolist())
        if detection.get("status") != "TRACKING" or detection.get("valid") is not True:
            result["reason"] = "target_not_tracking"
            return result
        if ((source.get("camera_id") is not None and source["camera_id"] != detection.get("camera_id")) or
                (source.get("tag_id") is not None and source["tag_id"] != detection.get("tag_id"))):
            result["reason"] = "calibration_source_mismatch"
            return result
        try:
            motion = evaluate_pose(self.zero, pose_to_matrix(detection.get("pose")), self.axis)
        except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
            result.update(reason="invalid_target_pose", pose_error=str(exc))
            return result
        error = motion["axis_error_rad"]
        result.update(valid=True, reason="", theta_rad=float(motion["rotation_angle_rad"]),
                      translation_along_axis_m=float(motion["translation_along_axis_m"]),
                      translation_vector_m=motion["translation_vector"].tolist(),
                      axis_direction=motion["axis_direction"].tolist(),
                      axis_error_rad=error,
                      axis_match=None if error is None else error <= math.radians(5.0))
        return result
