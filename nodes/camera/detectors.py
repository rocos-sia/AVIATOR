"""One active fiducial detector per camera session."""

from dataclasses import dataclass
import math

import cv2
import numpy as np
from cv2 import aruco


@dataclass
class DetectionResult:
    status: str = "SEARCHING"
    confidence: float = 0.0
    pose: dict | None = None
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    charuco_corners: np.ndarray | None = None
    charuco_ids: np.ndarray | None = None
    marker_corners: list | None = None
    marker_ids: np.ndarray | None = None
    tag_corners: np.ndarray | None = None
    tag_id: int | None = None
    decision_margin: float | None = None


def pose_from_pnp(rvec, tvec):
    """Target-to-camera pose: translation in metres, unit quaternion (qx, qy, qz, qw)."""
    rotation, _ = cv2.Rodrigues(rvec)
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = 2 * math.sqrt(trace + 1)
        qw = scale / 4
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2 * math.sqrt(max(0, 1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]))
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = scale / 4
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2 * math.sqrt(max(0, 1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]))
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = scale / 4
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2 * math.sqrt(max(0, 1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]))
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = scale / 4
    quaternion = np.array([qx, qy, qz, qw])
    quaternion /= np.linalg.norm(quaternion)
    x, y, z = tvec.flatten().astype(float)
    return {
        "position": {"x": float(x), "y": float(y), "z": float(z)},
        "orientation": dict(zip(("qx", "qy", "qz", "qw"),
                                map(float, quaternion))),
    }


def board_size(value):
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return int(value[0]), int(value[1])
    if isinstance(value, str):
        parts = value.lower().split("x")
        if len(parts) == 2:
            return int(parts[0]), int(parts[1])
    raise ValueError("charuco.board_size must be [columns, rows] or NxM")


class CharucoDetector:
    kind = "charuco"

    def __init__(self, settings, camera_matrix, dist_coeffs):
        columns, rows = board_size(settings["board_size"])
        square_length = float(settings["square_length"])
        marker_length = float(settings["marker_length"])
        self.min_corners = int(settings["min_corners"])
        if columns < 2 or rows < 2 or not (0 < marker_length < square_length):
            raise ValueError("invalid ChArUco board geometry")
        dictionary_name = settings["dictionary"]
        if not isinstance(dictionary_name, str) or not dictionary_name.startswith("DICT_"):
            raise ValueError("invalid ChArUco dictionary")
        try:
            dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        except AttributeError as error:
            raise ValueError(f"unknown ChArUco dictionary: {dictionary_name}") from error
        self.board = aruco.CharucoBoard(
            (columns, rows), square_length, marker_length, dictionary)
        self.total_corners = len(self.board.getChessboardCorners())
        if not 4 <= self.min_corners <= self.total_corners:
            raise ValueError("charuco.min_corners outside board corner count")
        self.detector = aruco.CharucoDetector(self.board)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect(self, image):
        corners, ids, marker_corners, marker_ids = self.detector.detectBoard(image)
        result = DetectionResult(
            marker_corners=marker_corners, marker_ids=marker_ids,
            charuco_corners=corners, charuco_ids=ids)
        if ids is not None:
            result.confidence = min(len(ids) / self.total_corners, 1.0)
        if ids is not None and len(ids) >= self.min_corners:
            obj_points, img_points = self.board.matchImagePoints(corners, ids)
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, self.camera_matrix, self.dist_coeffs)
            if success:
                result.status = "TRACKING"
                result.rvec, result.tvec = rvec, tvec
                result.pose = pose_from_pnp(rvec, tvec)
        return result

    def draw(self, image, result):
        if result.marker_ids is not None:
            aruco.drawDetectedMarkers(image, result.marker_corners, result.marker_ids)
        if result.charuco_ids is not None:
            # OpenCV 5 may return Nx2/N arrays; the drawing API requires
            # two-channel corners (Nx1x2), also accepted by OpenCV 4.
            aruco.drawDetectedCornersCharuco(
                image, np.asarray(result.charuco_corners, dtype=np.float32).reshape(-1, 1, 2),
                np.asarray(result.charuco_ids, dtype=np.int32).reshape(-1, 1))
        if result.pose is not None:
            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs,
                              result.rvec, result.tvec, 0.1)


class AprilTagDetector:
    kind = "apriltag"

    def __init__(self, settings, camera_matrix, dist_coeffs):
        import apriltag  # Only required when this detector is selected.

        self.tag_id = int(settings["tag_id"])
        self.tag_size_m = float(settings["tag_size_m"])
        self.min_decision_margin = float(settings.get("min_decision_margin", 0.0))
        self.confidence_margin = float(settings.get("confidence_margin", 50.0))
        family = settings.get("family", "tag36h11")
        if (self.tag_id < 0 or not math.isfinite(self.tag_size_m) or
                self.tag_size_m <= 0 or not math.isfinite(self.min_decision_margin) or
                self.min_decision_margin < 0 or not math.isfinite(self.confidence_margin) or
                self.confidence_margin <= 0 or not isinstance(family, str) or not family):
            raise ValueError("invalid AprilTag settings")
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families=family))
        half = self.tag_size_m / 2
        # Match the corner order in the supplied AprilTag example.
        self.object_points = np.array([
            [-half, -half, 0], [half, -half, 0],
            [half, half, 0], [-half, half, 0]], dtype=np.float32)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        matches = [d for d in self.detector.detect(gray)
                   if int(d.tag_id) == self.tag_id]
        if not matches:
            return DetectionResult()
        match = max(matches, key=lambda d: float(d.decision_margin))
        margin = float(match.decision_margin)
        if not math.isfinite(margin):
            return DetectionResult()
        result = DetectionResult(
            confidence=max(0.0, min(1.0, margin / self.confidence_margin)),
            tag_corners=np.asarray(match.corners, dtype=np.float32),
            tag_id=self.tag_id, decision_margin=margin)
        if margin < self.min_decision_margin:
            return result
        if result.tag_corners.shape != (4, 2):
            return result
        success, rvec, tvec = cv2.solvePnP(
            self.object_points, result.tag_corners,
            self.camera_matrix, self.dist_coeffs)
        if success and float(tvec[2, 0]) > 0:
            result.status = "TRACKING"
            result.rvec, result.tvec = rvec, tvec
            result.pose = pose_from_pnp(rvec, tvec)
        return result

    def draw(self, image, result):
        if result.tag_corners is not None and result.tag_corners.shape == (4, 2):
            cv2.polylines(image, [result.tag_corners.astype(np.int32)],
                          True, (0, 255, 0), 2)
            center = result.tag_corners.mean(axis=0).astype(int)
            cv2.putText(image, str(result.tag_id), tuple(center),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if result.pose is not None:
            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs,
                              result.rvec, result.tvec, self.tag_size_m)


def create_detector(kind, settings, camera_matrix, dist_coeffs):
    if kind == "charuco":
        return CharucoDetector(settings, camera_matrix, dist_coeffs)
    if kind == "apriltag":
        return AprilTagDetector(settings, camera_matrix, dist_coeffs)
    raise ValueError(f"unsupported detector.type: {kind}")
