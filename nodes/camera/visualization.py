"""Optional camera diagnostics. Drawing uses a copy of the recording frame."""

# Shared logging module in source and installed share/aviator layouts.
import sys as _log_sys
from pathlib import Path as _LogPath
_log_sys.path.insert(0, str(_LogPath(__file__).resolve().parents[1] / "common")
                     if (_LogPath(__file__).resolve().parents[1] / "common").is_dir()
                     else str(_LogPath(__file__).resolve().parents[2] / "common"))
from aviator_logger import Logger


import math
import os
import shutil
import sys
import time

import cv2
import numpy as np


def visualization_options(settings, args):
    if not isinstance(settings, dict):
        raise ValueError("visualization 必须是映射")
    options = {}
    for key in ("show", "print_pose"):
        value = settings.get(key, False)
        if not isinstance(value, bool):
            raise ValueError(f"visualization.{key} 必须是布尔值")
        override = getattr(args, key)
        options[key] = value if override is None else override
    interval = settings.get("print_interval_s", 0.5)
    if args.pose_print_interval is not None:
        interval = args.pose_print_interval
    if isinstance(interval, bool) or not isinstance(interval, (int, float)) or not math.isfinite(interval) or interval <= 0:
        raise ValueError("visualization.print_interval_s 必须是有限正数")
    options["print_interval_s"] = float(interval)
    return options


def pose_lines(result):
    if result.status != "TRACKING" or result.pose is None or result.rvec is None:
        return ["No valid pose (target missing or PnP rejected)"]
    position = result.pose["position"]  # Same metre values as camera.detection.
    rotation, _ = cv2.Rodrigues(result.rvec)
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    if sy >= 1e-6:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    pitch = math.atan2(-rotation[2, 0], sy)
    roll, pitch, yaw = np.degrees([roll, pitch, yaw])
    return [f"Position (m): X={position['x']:.4f} Y={position['y']:.4f} Z={position['z']:.4f}",
            f"RPY (deg): roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"]


def terminal_pose_lines(result, steering_wheel):
    """Show the raw tag pose and the same physical motion sent on the bus."""
    if result.status == "TRACKING" and result.pose is not None:
        position, quaternion = result.pose["position"], result.pose["orientation"]
        lines = [f"Tag position (m): X={position['x']:+.6f} Y={position['y']:+.6f} Z={position['z']:+.6f}",
                 "Tag quaternion (xyzw): " + " ".join(
                     f"{quaternion[key]:+.6f}" for key in ("qx", "qy", "qz", "qw"))]
    else:
        lines = ["No valid pose (target missing or PnP rejected)", "Tag quaternion: unavailable"]
    if steering_wheel and steering_wheel.get("valid"):
        vector = steering_wheel["translation_vector_m"]
        lines += [f"Wheel rotation (rad): {steering_wheel['theta_rad']:+.6f}",
                  f"Along axis (m): {steering_wheel['translation_along_axis_m']:+.6f}",
                  f"Relative translation (m): X={vector[0]:+.6f} Y={vector[1]:+.6f} Z={vector[2]:+.6f}"]
    else:
        reason = steering_wheel.get("reason", "unavailable") if steering_wheel else "unavailable"
        lines += [f"Wheel rotation (rad): unavailable ({reason})",
                  "Along axis (m): unavailable", "Relative translation (m): unavailable"]
    return lines


def render_preview(image, detector, result, frame_id, fps, warming_up):
    display = image.copy()
    detector.draw(display, result)
    identity = f"tag={result.tag_id}" if result.tag_id is not None else ""
    lines = [f"{detector.kind} {identity} {result.status} conf={result.confidence:.2f}",
             f"frame={frame_id} FPS={fps:.1f} {'WARMUP' if warming_up else 'LIVE'} | q/ESC: exit",
             *pose_lines(result), "Camera frame: +X right, +Y down, +Z forward"]
    panel_width = max(cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0]
                      for line in lines) + 24
    cv2.rectangle(display, (6, 5), (min(display.shape[1] - 1, panel_width),
                  min(display.shape[0] - 1, 25 * len(lines) + 10)), (20, 20, 20), -1)
    for index, line in enumerate(lines):
        point = (12, 25 + index * 25)
        cv2.putText(display, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if result.status == "TRACKING" else (0, 200, 255),
                    1, cv2.LINE_AA)
    return display


class CameraVisualization:
    def __init__(self, show=False, print_pose=False, print_interval_s=0.5):
        self.show = show
        self.print_pose = print_pose
        self.interval = print_interval_s
        self.window = "aviator_camera"
        self.opened = False
        self.last_print = None
        self.last_status = None
        self.last_frame = None
        self.fps = 0.0
        self._terminal_lines = 0

    def _print_diagnostics(self, lines):
        if not sys.stdout.isatty():
            # Keep redirected logs readable, without cursor escape sequences.
            Logger.info(" | ".join(lines), flush=True)
            return
        columns = max(1, shutil.get_terminal_size().columns - 1)
        # A fixed-height panel and clipped lines avoid wrapping into extra rows.
        prefix = f"\x1b[{self._terminal_lines}F" if self._terminal_lines else ""
        Logger.output(prefix + "".join("\x1b[2K" + line[:columns] + "\n" for line in lines),
                      end="", flush=True)
        self._terminal_lines = len(lines)

    def check_available(self):
        if not self.show:
            return
        if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("--show 需要桌面显示环境；无桌面时使用 --no-show --print-pose")
        for line in cv2.getBuildInformation().splitlines():
            if line.strip().startswith("GUI:") and line.split(":", 1)[1].strip() == "NONE":
                raise RuntimeError("当前 OpenCV 不支持 GUI；请使用带 GUI 的 OpenCV，或 --no-show --print-pose")

    def update(self, image, detector, result, frame_id, warming_up=False, now=None, steering_wheel=None):
        """Return False if the user closes the preview or presses q/ESC."""
        if not self.show and not self.print_pose:
            return True
        now = time.monotonic() if now is None else now
        print_state = (result.status, bool(steering_wheel and steering_wheel.get("valid")),
                       steering_wheel.get("reason") if steering_wheel else None, warming_up)
        if self.print_pose and (self.last_print is None or now - self.last_print >= self.interval
                                or print_state != self.last_status):
            identity = f" tag={result.tag_id}" if result.tag_id is not None else ""
            header = (f"aviator_camera: frame={frame_id} {detector.kind}{identity} "
                      f"{result.status} conf={result.confidence:.2f} {'WARMUP' if warming_up else ''}")
            self._print_diagnostics([header, *terminal_pose_lines(result, steering_wheel)])
            self.last_print = now
            self.last_status = print_state
        if self.show:
            if self.last_frame is not None and now > self.last_frame:
                instant = 1 / (now - self.last_frame)
                self.fps = instant if self.fps == 0 else 0.9 * self.fps + 0.1 * instant
            self.last_frame = now
            if not self.opened:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                self.opened = True
            else:
                try:
                    visible = cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE)
                except cv2.error as error:
                    # Qt can destroy its GUI receiver when the last window is
                    # closed, raising StsNullPtr instead of returning invisible.
                    if error.code != cv2.Error.StsNullPtr:
                        raise
                    Logger.info("aviator_camera: preview window no longer available; stopping")
                    return False
                if visible < 1:
                    Logger.info("aviator_camera: preview window closed; stopping")
                    return False
            cv2.imshow(self.window, render_preview(
                image, detector, result, frame_id, self.fps, warming_up))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                Logger.info("aviator_camera: preview q/ESC pressed; stopping")
                return False
        return True

    def close(self):
        self._terminal_lines = 0
        if self.opened:
            try:
                cv2.destroyWindow(self.window)
            except cv2.error:
                pass  # The user may already have closed the window.
            self.opened = False
