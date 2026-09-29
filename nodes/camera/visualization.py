"""Optional camera diagnostics. Drawing uses a copy of the recording frame."""

import math
import os
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
    position = result.pose["position"]  # Same mm values as camera.detection.
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
    return [f"Position (mm): X={position['x']:.1f} Y={position['y']:.1f} Z={position['z']:.1f}",
            f"RPY (deg): roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"]


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

    def check_available(self):
        if not self.show:
            return
        if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("--show 需要桌面显示环境；无桌面时使用 --no-show --print-pose")
        for line in cv2.getBuildInformation().splitlines():
            if line.strip().startswith("GUI:") and line.split(":", 1)[1].strip() == "NONE":
                raise RuntimeError("当前 OpenCV 不支持 GUI；请使用带 GUI 的 OpenCV，或 --no-show --print-pose")

    def update(self, image, detector, result, frame_id, warming_up=False, now=None):
        """Return False if the user closes the preview or presses q/ESC."""
        if not self.show and not self.print_pose:
            return True
        now = time.monotonic() if now is None else now
        if self.print_pose and (self.last_print is None or now - self.last_print >= self.interval
                                or result.status != self.last_status):
            identity = f" tag={result.tag_id}" if result.tag_id is not None else ""
            print(f"aviator_camera pose: frame={frame_id} {detector.kind}{identity} "
                  f"{result.status} conf={result.confidence:.2f} "
                  f"{'WARMUP ' if warming_up else ''}" + " | ".join(pose_lines(result)), flush=True)
            self.last_print = now
            self.last_status = result.status
        if self.show:
            if self.last_frame is not None and now > self.last_frame:
                instant = 1 / (now - self.last_frame)
                self.fps = instant if self.fps == 0 else 0.9 * self.fps + 0.1 * instant
            self.last_frame = now
            if not self.opened:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                self.opened = True
            elif cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                return False
            cv2.imshow(self.window, render_preview(
                image, detector, result, frame_id, self.fps, warming_up))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                return False
        return True

    def close(self):
        if self.opened:
            try:
                cv2.destroyWindow(self.window)
            except cv2.error:
                pass  # The user may already have closed the window.
            self.opened = False
