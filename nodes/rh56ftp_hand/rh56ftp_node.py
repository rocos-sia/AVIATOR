#!/usr/bin/env python3
"""RH56FTP Modbus TCP backend for the AVIATOR ``hand.command`` bus.

The RH56FTP pendant is the source of truth for the Modbus register map.  This
node only uses its non-tactile ``read_state`` path and publishes the result as
AVIATOR ``hand.state`` messages.  One RH56FTP device can be used as the right
hand; pass ``--left-host`` as well when two devices are present.
"""

# Shared logging module in source and installed share/aviator layouts.
import sys as _log_sys
from pathlib import Path as _LogPath
_log_sys.path.insert(0, str(_LogPath(__file__).resolve().parents[1] / "common")
                     if (_LogPath(__file__).resolve().parents[1] / "common").is_dir()
                     else str(_LogPath(__file__).resolve().parents[2] / "common"))
from aviator_logger import Logger

import argparse
import json
import math
import re
import signal
import socket
import sys
import threading
import time
import os
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any


MAX_JSON_INTEGER = (1 << 53) - 1
SIDES = ("left", "right")
CHANNEL_NAMES = ("thumb_rotation", "thumb_bend", "index", "middle", "ring", "little")
DEVICE_ERRORS = ((1, "stall"), (2, "overtemperature"), (4, "overcurrent"),
                 (8, "motor_error"), (16, "communication_error"))
# AVIATOR order: keep thumb rotation at the handle-clearance position; open bends.
DEFAULT_SAFE_POSE = (0.5, 1.0, 1.0, 1.0, 1.0, 1.0)
DEFAULT_SPEED = 500
DEFAULT_FORCE = 1000
DEFAULT_HOLD_SPEED = 100
DEFAULT_HOLD_FORCE = 100
POSITION_FORCE_PROTECTION_MODE = 0
DEFAULT_STATE_HZ = 1000.0 / 60.0
UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$")


def _pendant_import_path() -> None:
    """Make the source tree and an installed sibling package importable."""
    here = Path(__file__).resolve().parent
    candidates = (here, here.parent.parent / "third_party" / "RH56FTP" / "python")
    for candidate in candidates:
        if (candidate / "pendant" / "handlink.py").is_file():
            value = str(candidate)
            if value not in sys.path:
                sys.path.insert(0, value)
            return


def load_handlink():
    _pendant_import_path()
    try:
        from pendant.handlink import HAND_IP, HAND_PORT, HandLink
    except ImportError as exc:  # pragma: no cover - depends on deployment env
        raise RuntimeError("需要 pymodbus，并且 third_party/RH56FTP/python/pendant 可导入") from exc
    return HAND_IP, HAND_PORT, HandLink


def _clock_id() -> str:
    try:
        boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    except OSError:
        boot = "unknown-boot"
    return f"{socket.gethostname()}-{boot}"


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0 or value > MAX_JSON_INTEGER:
        raise ValueError(f"{name} must be a positive uint53")
    return value


def _register_setting(value: Any, name: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [0,{maximum}]")
    return value


def _identifier(value: Any, name: str, uuid_value: bool = False) -> str:
    if not isinstance(value, str) or not value or len(value) > 128 or "\x00" in value:
        raise ValueError(f"invalid {name}")
    if uuid_value and not UUID_RE.fullmatch(value):
        raise ValueError(f"invalid {name} UUID")
    return value


def _object_pairs_no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _six_numbers(value: Any, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{name} must contain six values")
    out: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{name} must be numeric")
        number = float(item)
        if not math.isfinite(number) or not 0 <= number <= 1:
            raise ValueError(f"{name} outside [0,1]")
        out.append(number)
    return out


def decode_command(topic: str, payload: bytes | str) -> dict[str, Any]:
    """Decode and validate the hand.command envelope before any Modbus write."""
    if topic != "hand.command":
        raise ValueError("unexpected topic")
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    if not payload or len(payload.encode("utf-8")) > 65536:
        raise ValueError("invalid payload size")
    message = json.loads(payload, object_pairs_hook=_object_pairs_no_duplicates)
    if not isinstance(message, dict) or message.get("msg_type") != "HandCommand":
        raise ValueError("msg_type != HandCommand")
    version = _identifier(message.get("version"), "version")
    if not re.fullmatch(r"1\.\d+", version):
        raise ValueError("unsupported version")
    for key in ("sequence", "timestamp", "sample_mono_us"):
        _positive_int(message.get(key), key)
    _identifier(message.get("clock_id"), "clock_id")
    _identifier(message.get("publisher_id"), "publisher_id")
    _identifier(message.get("session_id"), "session_id")
    _identifier(message.get("control_epoch"), "control_epoch", True)
    if not isinstance(message.get("valid"), bool):
        raise ValueError("valid must be boolean")
    _identifier(message.get("mode"), "mode")

    origin = message.get("origin")
    if not isinstance(origin, dict):
        raise ValueError("origin must be an object")
    _identifier(origin.get("publisher_id"), "origin.publisher_id")
    _identifier(origin.get("session_id"), "origin.session_id")
    _positive_int(origin.get("sequence"), "origin.sequence")
    _positive_int(origin.get("sample_mono_us"), "origin.sample_mono_us")
    _identifier(origin.get("clock_id"), "origin.clock_id")

    if message["valid"]:
        mode = message["mode"]
        if mode not in ("NORMALIZED_POSITION", "GRASP_SETPOINT"):
            raise ValueError("unsupported hand mode")
        hands = message.get("hands")
        if not isinstance(hands, dict):
            raise ValueError("hands must be an object")
        for side in SIDES:
            hand = hands.get(side)
            if not isinstance(hand, dict):
                raise ValueError(f"hands.{side} must be an object")
            if mode == "NORMALIZED_POSITION":
                if "grasp" in hand or "joint_position" in hand:
                    raise ValueError("mixed hand target modes")
                hand["drive_position_normalized"] = _six_numbers(
                    hand.get("drive_position_normalized"), f"hands.{side}.drive_position_normalized")
            else:
                if "drive_position_normalized" in hand or "joint_position" in hand:
                    raise ValueError("mixed hand target modes")
                grasp = hand.get("grasp")
                if not isinstance(grasp, dict):
                    raise ValueError(f"hands.{side}.grasp must be an object")
                closure = grasp.get("closure")
                if isinstance(closure, bool) or not isinstance(closure, (int, float)):
                    raise ValueError("closure must be numeric")
                closure = float(closure)
                if not math.isfinite(closure) or not 0 <= closure <= 1:
                    raise ValueError("closure outside [0,1]")
                hand["grasp"] = {"closure": closure}
    return message


def canonical_to_rh(values: list[int | None] | tuple[int, ...]) -> list[int | None]:
    """AVIATOR [thumb_rot..pinky] -> RH56FTP [pinky..thumb_rot]."""
    if len(values) != 6:
        raise ValueError("six hand channels required")
    return list(reversed(values))


def rh_to_canonical(values: list[int] | tuple[int, ...]) -> list[int]:
    if len(values) != 6:
        raise ValueError("six hand channels required")
    return list(reversed(values))


def normalized_to_raw(values: list[float]) -> list[int]:
    return [min(1000, max(0, int(math.floor(value * 1000 + 0.5)))) for value in values]


class DiagnosticLog:
    """Concise console events, with opt-in structured troubleshooting output."""

    def __init__(self, log_format: str = "text"):
        if log_format not in ("text", "json"):
            raise ValueError("log_format must be text or json")
        self.log_format = log_format
        self._lock = threading.Lock()
        self._events: dict[str, tuple[int, str, int]] = {}

    @staticmethod
    def _text(event: str, reason: str, fields: dict[str, Any]) -> str:
        titles = {
            "configuration": "启动配置", "feedback_status_changed": "反馈异常",
            "feedback_recovered": "反馈已恢复",
            "modbus_connect_failed": "设备连接失败", "feedback_read_failed": "读取反馈失败",
            "modbus_write_failed": "写入设备失败", "feedback_reader_failed": "反馈线程退出",
            "hold_control_failed": "握持保护触发，已锁存故障，按各手故障范围执行保护",
            "fault_safe_pose_applied": "已确认本手同轮六路故障，已发送安全张开目标（拇指侧摆 500，其余五路 1000）",
            "fault_hold_applied": "故障保护保持，未满足本手同轮六路故障条件，已请求六路停止（-1）",
            "device_error": "设备故障", "reject command": "指令被拒绝",
            "command_ignored": "仅反馈模式，忽略运动指令",
            "command_watchdog_expired": "指令超时，准备执行安全姿态",
            "safe_pose_failed": "安全姿态写入失败", "startup_safe_pose_failed": "启动安全姿态写入失败",
            "shutdown_safe_pose_failed": "退出安全姿态写入失败",
            "feedback_poll_gap": "反馈轮询延迟", "feedback_read_slow": "反馈读取耗时过长",
            "feedback_sample_timestamp_offset": "反馈采集耗时过长",
            "modbus_write_slow": "设备写入耗时过长", "command_processing_slow": "指令处理耗时过长",
            "command_batch_slow": "批量指令处理耗时过长", "state_publish_gap": "状态发布间隔过长",
            "state_publish_dropped": "状态发送队列已满，丢弃本次状态",
            "invalid_wire_message": "收到无效总线消息",
        }
        phases = {"tracking": "执行运动指令", "holding": "满足抓握完成判据，切换到保持状态",
            "failed": "故障"}
        sides = {"left": "左手", "right": "右手"}
        channels = dict(zip(CHANNEL_NAMES, ("拇指侧摆", "拇指弯曲", "食指", "中指", "无名指", "小指")))
        title = phases.get(fields.get("phase"), "握持阶段变化") if event == "hold_phase_changed" else titles.get(event, event)
        if event == "feedback_recovered" and fields.get("hold_control_error"):
            title = "反馈已恢复，但握持故障仍锁存"
        location = sides.get(fields.get("side"), "") + channels.get(fields.get("channel_name"), "")
        parts = [f"{location} {title}".strip()]

        def value_text(value):
            if isinstance(value, (list, tuple)):
                if value and all(v == value[0] for v in value):
                    return f"{value[0]}（全部通道）"
                return "/".join(str(v) for v in value)
            return str(value).replace("\n", " | ")

        def add(label, value):
            if value is not None and value != "":
                parts.append(f"{label}={value_text(value)}")

        if event == "configuration_loaded":
            rows = [(name, json.dumps(value, ensure_ascii=False))
                    for name, value in fields["parameters"].items()]
            width = max(len(name) for name, _ in rows)
            lines = ["启动配置加载完成（命令行覆盖后的最终值）", f"{'Parameter':<{width}} | Value",
                     "-" * width + "-+-" + "-" * 48]
            lines.extend(f"{name:<{width}} | {value}" for name, value in rows)
            return "\n".join(lines)
        elif event == "setting_write":
            setting = {"speed": "速度 speed", "force": "力阈值 force"}[fields["phase"]]
            parts = [f"{location} 发送{setting}设置命令".strip()]
            add("通道顺序", [channels[name] for name in fields["channel_names"]])
            add("寄存器设定值", fields["target_raw"])
        elif event == "configuration":
            for side, endpoint in fields.get("endpoints", {}).items():
                add(sides.get(side, side), f"{endpoint.get('host')}:{endpoint.get('port')}")
            add("模式", fields.get("configured_mode"))
            add("抓取速度", fields.get("speed_raw"))
            add("抓取力指令", fields.get("force_raw"))
            add("保持速度", fields.get("hold_speed_raw"))
            add("保持力指令", fields.get("hold_force_raw"))
            add("握紧位置误差阈值", fields.get("closing_motion_raw"))
            add("持续时间(ms)", fields.get("closing_hold_ms"))
            parts.append("数值为寄存器刻度；配置不代表设备读回")
        elif event == "hold_phase_changed":
            for key, label in (("actual_raw", "当前位置"), ("requested_raw", "请求位置"),
                               ("threshold", "位置误差阈值"), ("closing_age_ms", "偏差持续(ms)"),
                               ("held_raw", "保持位置"), ("hold_speed_raw", "保持速度"),
                               ("hold_force_raw", "保持力阈值")):
                add(label, fields.get(key))
            reasons = {"position_error": "持续位置偏差", "position_stable": "位置稳定"}
            add("保持原因", reasons.get(fields.get("hold_reason")))
            add("稳定观察窗口(ms)", fields.get("stable_window_ms"))
        elif event == "device_error":
            if "command_valid" in fields:
                add("控制状态", "有有效指令" if fields["command_valid"] else "无有效指令")
            errors = {"stall": "堵转", "overtemperature": "过温", "overcurrent": "过流",
                      "motor_error": "电机异常", "communication_error": "通信异常"}
            for channel in fields.get("channels", []):
                names = [errors.get(name, name) for name in channel["error_names"]]
                if channel.get("unknown_bits"):
                    names.append(f"未知故障位 {channel['unknown_bits']}")
                context = "".join(f"，{label}={channel[key]}" for key, label in (
                    ("actual_raw", "当前位置"), ("requested_raw", "最近请求位置"),
                    ("applied_raw", "最近成功写入目标")) if channel.get(key) is not None)
                parts.append(f"{channels.get(channel['channel_name'], channel['channel_name'])}: "
                             f"{'、'.join(names)}（故障码={channel['code']}，"
                             f"电流={channel['current_raw']}，力={channel['force_raw']}，"
                             f"温度={channel['temperature_c']}{context}）")
        else:
            if event == "feedback_status_changed":
                reason = (fields.get("last_error") or fields.get("invalid_reason") or
                          ("设备报告故障" if fields.get("status") == "ERROR" else "反馈读取或更新延迟"))
            translations = {
                "invalid position feedback": "位置反馈无效",
                "missing/stale/future feedback": "反馈缺失、过期或时间戳超前",
                "no_feedback": "尚未收到反馈", "feedback_timestamp_expired_or_future": "反馈过期或时间戳超前",
                "angle_register_out_of_range": "位置寄存器超出范围", "feedback_read_error": "反馈读取失败",
                "current_raw=": "电流原始值=", "force_raw=": "受力原始值=",
                "temperature_raw=": "温度原始值=", "outside ": "超出手册范围 ",
            }
            for raw, translated in translations.items():
                reason = reason.replace(raw, translated)
            add("原因", reason)
            for key, label in (
                ("role", "连接用途"), ("phase", "阶段"), ("duration_ms", "耗时(ms)"),
                ("feedback_age_ms", "反馈年龄(ms)"), ("feedback_timeout_ms", "反馈超时阈值(ms)"),
                ("read_duration_ms", "读取耗时(ms)"), ("sample_timestamp_offset_ms", "采集耗时(ms)"),
                ("start_gap_ms", "轮询间隔(ms)"), ("wait_overrun_ms", "轮询等待超期(ms)"),
                ("gap_ms", "发布间隔(ms)"), ("receive_age_ms", "指令接收间隔(ms)"),
                ("command_age_ms", "指令年龄(ms)"), ("origin_age_ms", "源指令年龄(ms)"),
                ("command_timeout_ms", "指令超时阈值(ms)"), ("sequence", "指令序号"),
                ("accepted_sequence", "已接受序号"), ("frame_count", "消息帧数"), ("dropped", "累计丢弃"),
                ("actual_raw", "当前位置"), ("requested_raw", "请求位置"),
                ("held_raw", "记录保持位置"), ("hold_drift_raw", "位置偏移"),
                ("applied_raw", "最近成功写入目标"),
                ("target_raw", "写入目标"), ("previous_target_raw", "上次写入目标"),
                ("current_raw", "电流"), ("current_register_raw", "电流原始字"),
                ("force_raw", "力"), ("temperature_c", "温度"), ("error_code", "设备故障码"),
                ("closing_age_ms", "偏差持续(ms)"), ("threshold", "位置误差阈值"),
                ("last_write_error", "最近写入错误"),
                ("fault_count", "本轮故障自由度数"), ("fault_channels", "本轮故障通道"),
            ):
                value = fields.get(key)
                if key == "phase":
                    value = phases.get(value, {"angle": "位置写入", "force": "力阈值写入",
                                               "speed": "速度写入", "mode": "模式写入"}.get(value, value))
                elif key == "role":
                    value = {"command": "指令", "feedback": "反馈"}.get(value, value)
                add(label, value)
        if fields.get("host"):
            add("设备", f"{fields['host']}:{fields.get('port')}")
        if fields.get("suppressed"):
            add("期间省略同类日志", fields["suppressed"])
        return "；".join(parts)

    def emit(self, event: str, *, key: str | None = None, reason: str = "",
             throttle_us: int = 1000000, **fields: Any) -> None:
        if self.log_format == "text":
            # The initiating fault already reports the cause and all-channel stop.
            # Per-channel failed phases are software propagation, not 12 motor faults.
            if event == "health" or (event == "hold_phase_changed" and fields.get("phase") == "failed"):
                return
            if throttle_us:
                throttle_us = max(throttle_us, 10000000)
        now = monotonic_us()
        with self._lock:
            name = key or event
            previous, old_reason, suppressed = self._events.get(name, (0, "", 0))
            if previous and reason == old_reason and now - previous < throttle_us:
                self._events[name] = (previous, old_reason, suppressed + 1)
                return
            self._events[name] = (now, reason, 0)
            record = {"event": event, "mono_us": now, **fields}
            if reason:
                record["reason"] = reason
            if suppressed:
                record["suppressed"] = suppressed
            emit = (Logger.error if event.endswith("_failed") or event == "device_error" or
                    (event == "hold_phase_changed" and fields.get("phase") == "failed") else
                    Logger.info if event in ("health", "configuration", "configuration_loaded", "setting_write",
                                             "feedback_recovered", "hold_phase_changed")
                    else Logger.warn)
            message = (json.dumps(record, ensure_ascii=False, allow_nan=False) if self.log_format == "json"
                       else self._text(event, reason, record))
            emit("rh56ftp_hand: " + message,
                  file=sys.stderr, flush=True)


def age_ms(now: int, stamp: int | None) -> float | None:
    return round((now - stamp) / 1000.0, 3) if stamp else None


@dataclass
class Snapshot:
    data: dict[str, Any] | None = None
    sample_mono_us: int | None = None
    error: str = ""
    reads: int = 0
    errors: int = 0
    read_started_us: int = 0
    read_in_progress: bool = False
    read_duration_ms: float = 0
    read_finished_us: int = 0
    last_success_finished_us: int = 0
    read_start_gap_ms: float = 0
    max_read_start_gap_ms: float = 0
    read_idle_gap_ms: float = 0
    sample_timestamp_offset_ms: float = 0

    def fresh(self, now: int, timeout_us: int) -> bool:
        return self.data is not None and self.sample_mono_us is not None and 0 <= now - self.sample_mono_us < timeout_us


@dataclass
class ClosingHold:
    requested_raw: int | None = None
    target_since_us: int = 0
    last_command_us: int = 0
    last_sample_us: int = 0
    feedback_errors: int = 0
    held_raw: int | None = None
    close_since_us: int = 0
    residual_direction: int = 0
    stability_samples: tuple[tuple[int, int], ...] = ()
    held_at_target: bool = False
    hold_reason: str = ""
    phase: str = "tracking"


@dataclass
class CommandGuard:
    authorized: bool = False
    publisher: str = ""
    session: str = ""
    epoch: str = ""
    origin_publisher: str = ""
    origin_session: str = ""
    last_sequence: int = 0
    last_recv_mono_us: int = 0
    sample_mono_us: int = 0
    origin_mono_us: int = 0

    def accept(self, command: dict[str, Any], now: int, timeout_us: int, clock_id: str) -> None:
        def fresh(sample: int) -> bool:
            return 0 < sample <= now and now - sample < timeout_us

        origin = command["origin"]
        if command["clock_id"] != clock_id or origin["clock_id"] != clock_id:
            raise ValueError("clock domain mismatch")
        if not fresh(command["sample_mono_us"]) or not fresh(origin["sample_mono_us"]):
            raise ValueError("stale/future command")
        if self.authorized and (
            command["publisher_id"] != self.publisher
            or command["control_epoch"] != self.epoch
            or origin["publisher_id"] != self.origin_publisher
        ):
            raise PermissionError("publisher/epoch/origin mismatch")
        if command["session_id"] == self.session and command["sequence"] <= self.last_sequence:
            raise ValueError("stale command sequence")
        if self.authorized and command["session_id"] != self.session and command["sample_mono_us"] <= self.sample_mono_us:
            raise ValueError("regressing producer restart")
        self.authorized = True
        self.publisher = command["publisher_id"]
        self.session = command["session_id"]
        self.epoch = command["control_epoch"]
        self.origin_publisher = origin["publisher_id"]
        self.origin_session = origin["session_id"]
        self.last_sequence = command["sequence"]
        self.last_recv_mono_us = now
        self.sample_mono_us = command["sample_mono_us"]
        self.origin_mono_us = origin["sample_mono_us"]

    def expired(self, now: int, timeout_us: int) -> bool:
        return self.authorized and any(
            sample <= 0 or sample > now or now - sample >= timeout_us
            for sample in (self.last_recv_mono_us, self.sample_mono_us, self.origin_mono_us)
        )


class Rh56FtpNode:
    """Bus adapter; ``links`` maps each side to a pendant HandLink or None."""

    def __init__(self, links: dict[str, Any], publisher_id: str = "rh56ftp_hand",
                 command_timeout_ms: int = 100, feedback_timeout_ms: int = 500,
                 feedback_only: bool = False, clock_id: str | None = None,
                 session_id: str | None = None, read_links: dict[str, Any] | None = None,
                 speed: int = DEFAULT_SPEED, force: int = DEFAULT_FORCE,
                 diagnostic_interval_s: float = 1.0,
                 closing_hold_ms: int = 10000, closing_motion_raw: int = 10,
                 log_format: str = "text", hold_speed: int = DEFAULT_HOLD_SPEED,
                 hold_force: int = DEFAULT_HOLD_FORCE):
        if set(links) != set(SIDES) or not any(links.values()):
            raise ValueError("at least one left/right RH56FTP link is required")
        if command_timeout_ms <= 0 or feedback_timeout_ms <= 0:
            raise ValueError("timeouts must be positive")
        if not math.isfinite(diagnostic_interval_s) or diagnostic_interval_s < 0:
            raise ValueError("diagnostic_interval_s must be finite and nonnegative")
        if isinstance(closing_hold_ms, bool) or not isinstance(closing_hold_ms, int) or closing_hold_ms <= 0:
            raise ValueError("closing_hold_ms must be a positive integer")
        if isinstance(closing_motion_raw, bool) or not isinstance(closing_motion_raw, int) or not 0 <= closing_motion_raw < 1000:
            raise ValueError("closing_motion_raw must be an integer in [0,999]")
        _identifier(publisher_id, "publisher_id")
        self.links = links
        self.read_links = read_links or links
        if set(self.read_links) != set(SIDES):
            raise ValueError("read_links must contain left and right entries")
        self.publisher_id = publisher_id
        self.command_timeout_us = command_timeout_ms * 1000
        self.feedback_timeout_us = feedback_timeout_ms * 1000
        self.feedback_only = feedback_only
        self.speed = _register_setting(speed, "speed", 1000)
        self.force = _register_setting(force, "force", 3000)
        self.hold_speed = _register_setting(hold_speed, "hold_speed", 1000)
        self.hold_force = _register_setting(hold_force, "hold_force", 3000)
        self._settings_applied: dict[str, tuple[bool, ...]] = {}
        self._angle_applied: dict[str, list[int]] = {}
        self.clock_id = clock_id or _clock_id()
        self.session_id = session_id or f"run-{time.time_ns()}-{os.getpid()}"
        self.guard = CommandGuard()
        self.snapshots = {side: Snapshot() for side in SIDES}
        self.last_command: dict[str, Any] | None = None
        self.last_targets: dict[str, list[float]] = {side: [] for side in SIDES}
        self.requested_targets: dict[str, list[float]] = {side: [] for side in SIDES}
        self.closing_hold_us = closing_hold_ms * 1000
        self.closing_motion_raw = closing_motion_raw
        self.hold_control_error = ""
        self.fault_channels: dict[str, dict[int, str]] = {side: {} for side in SIDES}
        self._fault_open_sides: set[str] = set()
        self.closing_holds = {side: [ClosingHold() for _ in range(6)] for side in SIDES}
        self.command_valid = False
        self.state_sequence = 0
        self._safe_applied = False
        self._safe_required = not feedback_only
        self.diag = DiagnosticLog(log_format)
        self.diagnostic_interval_us = int(diagnostic_interval_s * 1000000)
        self._next_diagnostic_us = 0
        self.command_received = self.command_accepted = self.command_rejected = 0
        self.last_received_us = self.last_ack_write_us = 0
        self.last_command_duration_ms = 0.0
        self.last_write_error = ""
        self.last_command_error = ""
        self.write_stats = {side: {"calls": 0, "errors": 0, "duration_ms": 0.0, "max_duration_ms": 0.0}
                            for side in SIDES}
        self.state_sent = self.state_dropped = 0
        self.last_state_sent_us = 0
        self.max_state_gap_ms = 0.0
        self._feedback_status: dict[str, tuple[Any, ...]] = {}
        self._feedback_wait_started_us: int | None = None
        self.reader_thread: threading.Thread | None = None
        # Replace the whole dict so the publisher sees one reader timing snapshot.
        self.reader_timing: dict[str, Any] = {"phase": "not_started", "cycles": 0}

    def connect(self) -> None:
        self.hold_control_error = ""
        self.fault_channels = {side: {} for side in SIDES}
        self._fault_open_sides.clear()
        self._settings_applied.clear()
        self._angle_applied.clear()
        self._reset_closing_holds()
        self.diag.emit("configuration", throttle_us=0, publisher=self.publisher_id, session=self.session_id,
                       configured_mode=POSITION_FORCE_PROTECTION_MODE, speed_raw=self.speed, force_raw=self.force,
                       hold_speed_raw=self.hold_speed, hold_force_raw=self.hold_force,
                       feedback_timeout_ms=self.feedback_timeout_us / 1000,
                       command_timeout_ms=self.command_timeout_us / 1000,
                       closing_hold_ms=self.closing_hold_us / 1000, closing_motion_raw=self.closing_motion_raw,
                       endpoints={side: {"host": getattr(link, "host", None), "port": getattr(link, "port", None)}
                                  for side, link in self.links.items() if link is not None})
        seen: set[int] = set()
        entries = list(self.links.items()) + list(self.read_links.items())
        for side, link in entries:
            if link is None:
                continue
            if id(link) in seen:
                continue
            seen.add(id(link))
            try:
                link.connect()
            except Exception as exc:
                self.snapshots[side].error = str(exc)
                self.diag.emit("modbus_connect_failed", key=f"connect:{side}:{id(link)}", side=side,
                               role="command" if link is self.links[side] else "feedback",
                               host=getattr(link, "host", None), port=getattr(link, "port", None),
                               reason=f"{type(exc).__name__}: {exc}")

    def close(self) -> None:
        self._settings_applied.clear()
        seen: set[int] = set()
        links = list(self.links.values()) + list(self.read_links.values())
        for link in links:
            if link is not None:
                if id(link) in seen:
                    continue
                seen.add(id(link))
                try:
                    link.close()
                except Exception:
                    pass

    def reader_poll_started(self, period_us: int, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        previous = self.reader_timing
        gap = age_ms(now, previous.get("cycle_started_us")) or 0
        idle = age_ms(now, previous.get("cycle_finished_us")) or 0
        due = previous.get("next_poll_due_us", now)
        lag = round(max(0, now - due) / 1000, 3)
        self.reader_timing = {
            **previous, "phase": "reading", "cycles": previous["cycles"] + 1,
            "period_ms": period_us / 1000, "cycle_started_us": now,
            "start_gap_ms": gap, "idle_gap_ms": idle, "wait_overrun_ms": lag,
            "max_start_gap_ms": max(previous.get("max_start_gap_ms", 0), gap),
            "max_wait_overrun_ms": max(previous.get("max_wait_overrun_ms", 0), lag),
        }
        if gap >= max(period_us / 500, self.feedback_timeout_us / 2000) or \
                lag >= max(period_us / 1000, self.feedback_timeout_us / 4000):
            self.diag.emit("feedback_poll_gap", **self.reader_timing)

    def reader_poll_finished(self, period_us: int, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        timing = self.reader_timing
        self.reader_timing = {
            **timing, "phase": "waiting", "cycle_finished_us": now,
            "cycle_duration_ms": age_ms(now, timing.get("cycle_started_us")),
            "next_poll_due_us": max(timing["cycle_started_us"] + period_us, now),
        }

    def reader_diagnostics(self, now: int) -> dict[str, Any]:
        timing = self.reader_timing
        return {
            **timing, "thread_alive": self.reader_thread.is_alive() if self.reader_thread is not None else None,
            "cycle_start_age_ms": age_ms(now, timing.get("cycle_started_us")),
            "cycle_finish_age_ms": age_ms(now, timing.get("cycle_finished_us")),
            "current_wait_overrun_ms": round(max(0, now - timing.get("next_poll_due_us", now)) / 1000, 3)
                if timing["phase"] == "waiting" else None,
        }

    def read_states(self, now: int | None = None) -> None:
        sample_override = now
        for side, link in self.read_links.items():
            if link is None:
                continue
            snapshot = self.snapshots[side]
            started = monotonic_us()
            snapshot.read_start_gap_ms = age_ms(started, snapshot.read_started_us) or 0
            snapshot.max_read_start_gap_ms = max(snapshot.max_read_start_gap_ms, snapshot.read_start_gap_ms)
            snapshot.read_idle_gap_ms = age_ms(started, snapshot.read_finished_us) or 0
            snapshot.read_started_us = started
            snapshot.read_in_progress = True
            finished = 0
            try:
                data = link.read_state()
                finished = monotonic_us()
                if not isinstance(data, dict):
                    raise ValueError("RH56FTP state must be an object")
                for key in ("angle", "force", "current", "err", "status", "temp"):
                    if not isinstance(data.get(key), list) or len(data[key]) != 6:
                        raise ValueError(f"RH56FTP state field {key} must contain six values")
                    for value in data[key]:
                        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                            raise ValueError(f"RH56FTP state field {key} contains a non-finite value")
                # Publish data and its timestamp together. Use this hand's read
                # start (conservative for angle-first reads), not a shared stamp
                # taken before the other hand or a delayed reader thread.
                snapshot = replace(snapshot, data=data,
                                   sample_mono_us=started if sample_override is None else sample_override,
                                   last_success_finished_us=finished,
                                   sample_timestamp_offset_ms=age_ms(finished, started) or 0,
                                   error="", reads=snapshot.reads + 1)
            except Exception as exc:
                finished = finished or monotonic_us()
                snapshot.error = str(exc)
                snapshot.errors += 1
                self.diag.emit("feedback_read_failed", key=f"read_failed:{side}", side=side,
                               reason=f"{type(exc).__name__}: {exc}", reads=snapshot.reads,
                               errors=snapshot.errors, feedback_age_ms=age_ms(monotonic_us(), snapshot.sample_mono_us),
                               host=getattr(link, "host", None), port=getattr(link, "port", None),
                               duration_ms=age_ms(finished, started), traceback=traceback.format_exc(limit=8))
            finally:
                snapshot.read_finished_us = finished or monotonic_us()
                snapshot.read_duration_ms = age_ms(snapshot.read_finished_us, snapshot.read_started_us) or 0
                snapshot.read_in_progress = False
                self.snapshots[side] = snapshot
                if snapshot.sample_timestamp_offset_ms >= self.feedback_timeout_us / 2000 and not snapshot.error:
                    self.diag.emit("feedback_sample_timestamp_offset", key=f"sample_offset:{side}", side=side,
                                   sample_mono_us=snapshot.sample_mono_us,
                                   read_finished_us=snapshot.last_success_finished_us,
                                   sample_timestamp_offset_ms=snapshot.sample_timestamp_offset_ms,
                                   read_duration_ms=snapshot.read_duration_ms)
                if snapshot.read_duration_ms >= self.feedback_timeout_us / 2000:
                    self.diag.emit("feedback_read_slow", key=f"read_slow:{side}", side=side,
                                   duration_ms=snapshot.read_duration_ms,
                                   feedback_timeout_ms=self.feedback_timeout_us / 1000)

    def _write_call(self, side: str, phase: str, operation, target_raw: list[int] | None = None) -> None:
        started = monotonic_us()
        stats = self.write_stats[side]
        stats["calls"] += 1
        try:
            if phase in ("speed", "force"):
                self.diag.emit("setting_write", throttle_us=0, side=side, phase=phase,
                               host=getattr(self.links[side], "host", None),
                               port=getattr(self.links[side], "port", None),
                               channel_names=CHANNEL_NAMES, target_raw=target_raw)
            operation()
        except Exception as exc:
            stats["errors"] += 1
            self.last_write_error = f"{side} {phase}: {type(exc).__name__}: {exc}"
            self.diag.emit("modbus_write_failed", key=f"write_failed:{side}", side=side, phase=phase,
                           target_raw=target_raw, duration_ms=age_ms(monotonic_us(), started),
                           reason=self.last_write_error, host=getattr(self.links[side], "host", None),
                           port=getattr(self.links[side], "port", None),
                           writes=stats["calls"], write_errors=stats["errors"],
                           previous_target_raw=self._angle_applied.get(side),
                           traceback=traceback.format_exc(limit=8))
            raise
        finally:
            stats["phase"] = phase
            stats["duration_ms"] = age_ms(monotonic_us(), started) or 0
            stats["max_duration_ms"] = max(stats["max_duration_ms"], stats["duration_ms"])
            if stats["duration_ms"] >= self.command_timeout_us / 2000:
                self.diag.emit("modbus_write_slow", key=f"write_slow:{side}", side=side, phase=phase,
                               duration_ms=stats["duration_ms"], command_timeout_ms=self.command_timeout_us / 1000)

    def _write_targets(self, raw_by_side: dict[str, list[int]]) -> None:
        if self.feedback_only:
            return
        # Configure every connected hand before moving either hand. Cache
        # successful writes to avoid adding Modbus traffic at command rate.
        for side, link in self.links.items():
            holding = tuple(value == -1 for value in raw_by_side[side])
            if link is not None and self._settings_applied.get(side) != holding:
                try:
                    # Both bus modes produce angle targets. Manual §2.6.20:
                    # mode 0 stops at the angle/force limit; mode 1 regulates
                    # force instead. Restore mode 0 before changing force, even
                    # when a previous node left the device in force control.
                    if side not in self._settings_applied:
                        self._write_call(side, "mode", lambda: link.write_mode_set([POSITION_FORCE_PROTECTION_MODE] * 6))
                    speeds = canonical_to_rh([self.hold_speed if held else self.speed for held in holding])
                    forces = canonical_to_rh([self.hold_force if held else self.force for held in holding])
                    self._write_call(side, "speed", lambda: link.write_speed_set(speeds), rh_to_canonical(speeds))
                    self._write_call(side, "force", lambda: link.write_force_set(forces), rh_to_canonical(forces))
                except Exception as exc:
                    self._settings_applied.pop(side, None)
                    raise RuntimeError(f"{side} speed/force/mode setup failed: {exc}") from exc
                self._settings_applied[side] = holding
        for side, link in self.links.items():
            if link is not None:
                try:
                    targets = raw_by_side[side]
                    previous = self._angle_applied.get(side, [None] * 6)
                    changed = [value if value != old else None for value, old in zip(targets, previous)]
                    if any(value is not None for value in changed):
                        self._write_call(side, "angle", lambda: link.write_angle_set(canonical_to_rh(changed)),
                                         targets)
                        self._angle_applied[side] = list(targets)
                        self.last_ack_write_us = monotonic_us()
                except Exception:
                    # Reapply settings on the next attempt after a write error.
                    self._settings_applied.pop(side, None)
                    self._angle_applied.pop(side, None)
                    raise
        self.last_write_error = ""

    def _observe_faults(self, now: int, pending: dict[str, list[ClosingHold]] | None = None) -> None:
        """Count independent channel faults from one fresh snapshot per hand, never failed phases."""
        for side, link in self.links.items():
            faults = {}
            snapshot = self.snapshots[side]
            if link is not None and snapshot.fresh(now, self.feedback_timeout_us) and not snapshot.error:
                measured = self._canonical_state(snapshot.data)
                for index in range(6):
                    code = measured["error_codes"][index]
                    actual = measured["drive_position_raw"][index]
                    if code:
                        faults[index] = f"device error {code}"
                    elif not 0 <= actual <= 1000:
                        faults[index] = "invalid position feedback"
            self.fault_channels[side] = faults
            if faults and not self.hold_control_error:
                index = next(iter(faults))
                self.hold_control_error = f"{side}[{index}]: {faults[index]}"
                self._safe_applied = False
                channel = (pending or self.closing_holds)[side][index]
                self.diag.emit("hold_control_failed", throttle_us=0, reason=self.hold_control_error,
                               **self._hold_context(side, index, channel, now, channel.requested_raw, snapshot),
                               command=self.command_context(now))
            if len(faults) == 6 and side not in self._fault_open_sides:
                self._fault_open_sides.add(side)
                self._safe_applied = False
        if self.hold_control_error:
            self._safe_required = True

    def _fault_targets(self) -> dict[str, list[int]]:
        return {side: normalized_to_raw(list(DEFAULT_SAFE_POSE)) if side in self._fault_open_sides else [-1] * 6
                for side in SIDES}

    def _report_fault_response(self) -> None:
        for side, link in self.links.items():
            if link is not None:
                event = "fault_safe_pose_applied" if side in self._fault_open_sides else "fault_hold_applied"
                self.diag.emit(event, key=f"{event}:{side}", side=side, reason=self.hold_control_error,
                               fault_count=len(self.fault_channels[side]),
                               fault_channels=list(self.fault_channels[side]))

    def _safe_pose(self, *, force_open: bool = False, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        if not force_open:
            self._observe_faults(now)
        feedback_unavailable = self.guard.authorized and any(
            link is not None and (self.snapshots[side].error or
                                  not self.snapshots[side].fresh(now, self.feedback_timeout_us))
            for side, link in self.links.items())
        fault_response = not force_open and (self.hold_control_error or feedback_unavailable or self.last_write_error)
        if fault_response:
            targets = self._fault_targets()
            for channels in self.closing_holds.values():
                for channel in channels:
                    channel.phase = "failed"
        else:
            self._reset_closing_holds()
            targets = {side: normalized_to_raw(list(DEFAULT_SAFE_POSE)) for side in SIDES}
        self._write_targets(targets)
        if fault_response:
            self._report_fault_response()
        self.last_targets = {side: [v / 1000.0 for v in raw] if -1 not in raw else []
                             for side, raw in targets.items()}
        if not fault_response:
            self.requested_targets = {side: [] for side in SIDES}
        self.command_valid = False
        self._safe_applied = True
        self._safe_required = False

    def _reset_closing_holds(self) -> None:
        self.closing_holds = {side: [ClosingHold() for _ in range(6)] for side in SIDES}

    def _closing_targets(self, requested: dict[str, list[int]], now: int
                         ) -> tuple[dict[str, list[int]], dict[str, list[ClosingHold]]]:
        effective = {side: list(values) for side, values in requested.items()}
        pending = {}
        for side in SIDES:
            snapshot = self.snapshots[side]
            measured = self._canonical_state(snapshot.data) if snapshot.data is not None else None
            actual = measured["drive_position_raw"] if measured and measured["drive_position_normalized"] else None
            fresh = snapshot.fresh(now, self.feedback_timeout_us) and not snapshot.error and actual is not None
            channels = []
            for index in range(6):
                target = requested[side][index]
                channel = replace(self.closing_holds[side][index])
                target_changed = target != channel.requested_raw
                # A reached/stable target accepts new motion in either direction.
                # A blocked channel only accepts motion away from its stop position.
                if (now < channel.last_command_us or
                        now - channel.last_command_us >= self.command_timeout_us or
                        (channel.held_raw is not None and
                         ((channel.held_at_target and target_changed) or
                          channel.residual_direction * (target - channel.held_raw) > 0))):
                    channel = ClosingHold(target_since_us=now, feedback_errors=snapshot.errors)
                if target_changed:
                    channel.stability_samples = ()
                channel.requested_raw = target
                channel.last_command_us = now
                # Further movement into the blocked direction cannot release a hold.
                if channel.held_raw is not None:
                    effective[side][index] = -1
                elif not fresh:
                    channel.close_since_us = channel.last_sample_us = 0
                    channel.stability_samples = ()
                else:
                    stamp = snapshot.sample_mono_us
                    position = actual[index]
                    if (snapshot.errors != channel.feedback_errors or
                            stamp < channel.last_sample_us or
                            (channel.last_sample_us and stamp - channel.last_sample_us >= self.feedback_timeout_us)):
                        channel.close_since_us = 0
                        channel.stability_samples = ()
                    channel.feedback_errors = snapshot.errors
                    residual = position - target
                    direction = 1 if residual >= 0 else -1
                    if direction != channel.residual_direction:
                        channel.close_since_us = 0
                    channel.residual_direction = direction
                    if abs(residual) <= self.closing_motion_raw:
                        channel.close_since_us = 0
                    if stamp >= channel.target_since_us and stamp != channel.last_sample_us:
                        if abs(residual) > self.closing_motion_raw and not channel.close_since_us:
                            channel.close_since_us = stamp
                        # Keep the full observation window, including its boundary
                        # sample. Small steps must not hide slow, continuing motion.
                        samples = channel.stability_samples + ((stamp, position),)
                        cutoff = stamp - self.closing_hold_us
                        first = 0
                        while first + 1 < len(samples) and samples[first + 1][0] <= cutoff:
                            first += 1
                        channel.stability_samples = samples[first:]
                        positions = [value for _, value in channel.stability_samples]
                        stable = (stamp - channel.stability_samples[0][0] >= self.closing_hold_us and
                                  max(positions) - min(positions) < self.closing_motion_raw)
                        residual_held = (channel.close_since_us != 0 and
                                         stamp - channel.close_since_us >= self.closing_hold_us)
                        if residual_held or stable:
                            channel.held_raw = position
                            channel.held_at_target = abs(residual) <= self.closing_motion_raw
                            channel.hold_reason = "position_error" if residual_held else "position_stable"
                            channel.phase = "holding"
                            effective[side][index] = -1
                    channel.last_sample_us = stamp
                channels.append(channel)
            pending[side] = channels
        return effective, pending

    def _hold_context(self, side: str, index: int, channel: ClosingHold, now: int,
                      requested_raw: int | None = None, snapshot: Snapshot | None = None) -> dict[str, Any]:
        """Capture the pre-fault decision, before changing phase or clearing timers."""
        snapshot = self.snapshots[side] if snapshot is None else snapshot
        measured = self._canonical_state(snapshot.data) if snapshot.data else {}
        value = lambda key: measured[key][index] if key in measured else None
        actual = value("drive_position_raw")
        code = value("error_codes")
        return {
            "side": side, "channel": index, "channel_name": CHANNEL_NAMES[index],
            "host": getattr(self.links[side], "host", None), "port": getattr(self.links[side], "port", None),
            "phase": channel.phase, "requested_raw": requested_raw, "actual_raw": actual,
            "position_error_raw": actual - requested_raw if actual is not None and requested_raw is not None else None,
            "held_raw": channel.held_raw,
            "hold_reason": channel.hold_reason,
            "held_at_target": channel.held_at_target,
            "stable_window_ms": (age_ms(now, channel.stability_samples[0][0])
                                 if channel.stability_samples else None),
            "hold_drift_raw": actual - channel.held_raw if actual is not None and channel.held_raw is not None else None,
            "applied_raw": self._angle_applied.get(side, [None] * 6)[index],
            "current_raw": value("current"), "current_register_raw": value("current_register_raw"),
            "current_abs_raw": abs(value("current")) if value("current") is not None else None,
            "force_raw": value("force"),
            "force_abs_raw": abs(value("force")) if value("force") is not None else None,
            "temperature_c": value("temperature"), "error_code": code,
            "error_names": [name for bit, name in DEVICE_ERRORS if code is not None and int(code) & bit],
            "status_code": value("status_codes"), "sample_mono_us": snapshot.sample_mono_us,
            "feedback_age_ms": age_ms(now, snapshot.sample_mono_us),
            "read_errors": snapshot.errors, "last_read_error": snapshot.error,
            "closing_age_ms": age_ms(now, channel.close_since_us),
            "threshold": self.closing_motion_raw,
            "closing_hold_ms": self.closing_hold_us / 1000,
        }

    def command_context(self, now: int, command: dict[str, Any] | None = None) -> dict[str, Any]:
        context = {
            "bound_publisher": self.guard.publisher, "bound_session": self.guard.session,
            "bound_epoch": self.guard.epoch, "accepted_sequence": self.guard.last_sequence,
            "ack_command_age_ms": age_ms(now, self.guard.sample_mono_us),
            "ack_write_age_ms": age_ms(now, self.last_ack_write_us),
            "command_timeout_ms": self.command_timeout_us / 1000,
        }
        if command is not None:
            context.update(sequence=command["sequence"], publisher=command["publisher_id"],
                           session=command["session_id"], epoch=command["control_epoch"],
                           command_age_ms=age_ms(now, command["sample_mono_us"]),
                           origin_age_ms=age_ms(now, command["origin"]["sample_mono_us"]),
                           clock_id=command["clock_id"], origin_clock_id=command["origin"]["clock_id"],
                           expected_clock_id=self.clock_id)
        return context

    def handle_command(self, topic: str, payload: bytes | str, now: int | None = None) -> bool:
        now = monotonic_us() if now is None else now
        self.command_received += 1
        self.last_received_us = now
        if self.feedback_only:
            self.diag.emit("command_ignored", reason="feedback-only mode", received=self.command_received)
            return False
        started = monotonic_us()
        command = None
        try:
            command = decode_command(topic, payload)
            self._apply_command(command, now)
        except Exception as exc:
            self.command_rejected += 1
            self.last_command_error = f"{type(exc).__name__}: {exc}"
            self.diag.emit("reject command", reason=self.last_command_error,
                           rejected=self.command_rejected, **self.command_context(monotonic_us(), command))
            raise
        else:
            self.command_accepted += 1
            self.last_write_error = ""
            self.last_command_error = ""
            return True
        finally:
            self.last_command_duration_ms = age_ms(monotonic_us(), started) or 0
            if self.last_command_duration_ms >= self.command_timeout_us / 2000:
                self.diag.emit("command_processing_slow", duration_ms=self.last_command_duration_ms,
                               **self.command_context(monotonic_us(), command))

    def _apply_command(self, command: dict[str, Any], now: int) -> None:
        candidate = CommandGuard(**self.guard.__dict__)
        candidate.accept(command, now, self.command_timeout_us, self.clock_id)
        raw_by_side: dict[str, list[int]] = {}
        if command["valid"]:
            mode = command["mode"]
            for side in SIDES:
                if mode == "NORMALIZED_POSITION":
                    values = command["hands"][side]["drive_position_normalized"]
                else:
                    values = [1.0 - command["hands"][side]["grasp"]["closure"]] * 6
                raw_by_side[side] = normalized_to_raw(values)
            requested = raw_by_side
            raw_by_side, pending_holds = self._closing_targets(requested, now)
            self._observe_faults(now, pending_holds)
            if self.hold_control_error:
                raw_by_side = self._fault_targets()
                for channels in pending_holds.values():
                    for channel in channels:
                        channel.phase = "failed"
            self._write_targets(raw_by_side)
            # Commit a new hold only after successful writes, preserving retry/ack semantics.
            for side in SIDES:
                for index, channel in enumerate(pending_holds[side]):
                    if channel.phase != self.closing_holds[side][index].phase:
                        self.diag.emit("hold_phase_changed", key=f"hold_phase:{side}:{index}", throttle_us=0,
                                       previous_phase=self.closing_holds[side][index].phase,
                                       hold_speed_raw=self.hold_speed if channel.phase == "holding" else None,
                                       hold_force_raw=self.hold_force if channel.phase == "holding" else None,
                                       **self._hold_context(side, index, channel, now, requested[side][index]))
            self.closing_holds = pending_holds
            if self.hold_control_error:
                self._report_fault_response()
            self.requested_targets = {side: [value / 1000.0 for value in requested[side]] for side in SIDES}
            for side in SIDES:
                data = self.snapshots[side].data
                actual = self._canonical_state(data)["drive_position_raw"] if data else [None] * 6
                positions = [value if value != -1 else pending_holds[side][i].held_raw
                             if pending_holds[side][i].held_raw is not None else actual[i]
                             for i, value in enumerate(raw_by_side[side])]
                self.last_targets[side] = [v / 1000.0 for v in positions] if all(v is not None for v in positions) else []
            self.command_valid = True
            self._safe_applied = False
            self._safe_required = False
        else:
            self._safe_pose(now=now)
            self.last_targets = {side: [] for side in SIDES}
            self.requested_targets = {side: [] for side in SIDES}
        self.guard = candidate
        self.last_command = command

    def supervise(self, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        if self.feedback_only:
            return
        self._observe_faults(now)
        if self.command_valid and self.guard.expired(now, self.command_timeout_us):
            self.diag.emit("command_watchdog_expired", receive_age_ms=age_ms(now, self.guard.last_recv_mono_us),
                           origin_age_ms=age_ms(now, self.guard.origin_mono_us),
                           hold_control_error=self.hold_control_error, **self.command_context(now))
            self._safe_required = True
        if self._safe_required and not self._safe_applied:
            try:
                self._safe_pose(now=now)
            except Exception as exc:
                self.diag.emit("safe_pose_failed", reason=f"{type(exc).__name__}: {exc}")
                # Keep the requirement pending so the next loop retries the
                # protective write instead of declaring safety.
                self._safe_applied = False

    @staticmethod
    def _canonical_state(data: dict[str, Any]) -> dict[str, Any]:
        angle = rh_to_canonical(data["angle"])
        # Modbus returns uint16 words. Firmware reports signed current (e.g.
        # 0xFF80 == -128); the vendor C++ example also casts CURRENT to int16_t.
        # Accept an already decoded backend too, without wrapping invalid values.
        current_register = rh_to_canonical(data["current"])
        current = [v - 65536 if type(v) is int and 32768 <= v <= 65535 else v
                   for v in current_register]
        valid_position = all(isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v <= 1000
                             for v in angle)
        normalized = [float(v) / 1000.0 for v in angle] if valid_position else []
        return {
            "angle_raw": angle,
            "drive_position_raw": angle,
            "drive_position_normalized": normalized,
            "force": rh_to_canonical(data["force"]),
            "current": current,
            "current_register_raw": [v + 65536 if type(v) is int and -32768 <= v < 0 else v
                                     for v in current_register],
            "error_codes": rh_to_canonical(data["err"]),
            "status_codes": rh_to_canonical(data["status"]),
            "temperature": rh_to_canonical(data["temp"]),
        }

    def make_state(self, now: int | None = None) -> dict[str, Any]:
        # Capture published measurements before stamping the envelope, so a
        # concurrent read cannot look like a sample from the future.
        snapshots = dict(self.snapshots)
        now = monotonic_us() if now is None else now
        self.state_sequence += 1
        hands: dict[str, dict[str, Any]] = {}
        configured_fresh = []
        for side in SIDES:
            link = self.links[side]
            snapshot = snapshots[side]
            fresh = link is not None and snapshot.fresh(now, self.feedback_timeout_us)
            if fresh:
                measured = self._canonical_state(snapshot.data or {})
                measurement_valid = bool(measured["drive_position_normalized"]) and not snapshot.error
                if link is not None:
                    configured_fresh.append(measurement_valid)
                error_codes = measured["error_codes"]
                status = "ERROR" if any(error_codes) else ("ACTIVE" if self.command_valid else "READY")
                hand = {
                    "valid": measurement_valid,
                    "status": status,
                    "force": measured["force"],
                    "current": measured["current"],
                    "current_register_raw": measured["current_register_raw"],
                    "error_code": max(error_codes, default=0),
                    "error_codes": error_codes,
                    "status_codes": measured["status_codes"],
                    "temperature": measured["temperature"],
                    "feedback_available": True,
                    "position_source": "angle_act_register",
                    "sample_mono_us": snapshot.sample_mono_us,
                    "sample_time_basis": "host_modbus_read",
                    "feedback_age_ms": (now - (snapshot.sample_mono_us or now)) / 1000.0,
                    "drive_position_raw": measured["drive_position_raw"],
                    "drive_position_normalized": measured["drive_position_normalized"],
                    "commanded_drive_position_normalized": self.last_targets[side],
                    "grasp_verified": False,
                    "feedback_samples": snapshot.reads,
                    "feedback_io_errors": snapshot.errors,
                    "feedback_last_error": snapshot.error,
                }
            else:
                if link is not None:
                    configured_fresh.append(False)
                hand = {
                    "valid": False,
                    "status": "OFFLINE" if link is None else ("STALE" if snapshot.data else "OFFLINE"),
                    "force": [],
                    "current": [],
                    "current_register_raw": [],
                    "error_code": 0,
                    "error_codes": [],
                    "status_codes": [],
                    "temperature": [],
                    "feedback_available": False,
                    "position_source": "angle_act_register",
                    "sample_mono_us": snapshot.sample_mono_us,
                    "sample_time_basis": "host_modbus_read",
                    "feedback_age_ms": ((now - snapshot.sample_mono_us) / 1000.0
                                        if snapshot.sample_mono_us is not None and now >= snapshot.sample_mono_us else None),
                    "drive_position_raw": [],
                    "drive_position_normalized": [],
                    "commanded_drive_position_normalized": self.last_targets[side],
                    "grasp_verified": False,
                    "feedback_samples": snapshot.reads,
                    "feedback_io_errors": snapshot.errors,
                    "feedback_last_error": snapshot.error,
                }
            hands[side] = hand
            hand["requested_drive_position_normalized"] = self.requested_targets[side]
            hand["closing_hold_active"] = [channel.held_raw is not None for channel in self.closing_holds[side]]
            hand["closing_hold_position_normalized"] = [
                channel.held_raw / 1000.0 if channel.held_raw is not None else None
                for channel in self.closing_holds[side]]
            hand["hold_control"] = [{"phase": c.phase}
                                    for c in self.closing_holds[side]]
            hand["fault_protection"] = {"fault_channels": dict(self.fault_channels[side]),
                                        "fault_count": len(self.fault_channels[side]),
                                        "open_requested": side in self._fault_open_sides}
        accepted = None
        if self.guard.authorized:
            accepted = {
                "publisher_id": self.guard.publisher,
                "session_id": self.guard.session,
                "sequence": self.guard.last_sequence,
                "sample_mono_us": self.guard.sample_mono_us,
                "control_epoch": self.guard.epoch,
            }
        return {
            "msg_type": "HandState",
            "version": "1.0",
            "sequence": self.state_sequence,
            "timestamp": time.time_ns() // 1000,
            "sample_mono_us": now,
            "clock_id": self.clock_id,
            "publisher_id": self.publisher_id,
            "session_id": self.session_id,
            # Measurement validity recovers independently of latched motion protection.
            "valid": bool(configured_fresh) and all(configured_fresh),
            "hold_control_error": self.hold_control_error,
            "command_valid": self.command_valid,
            "feedback_only": self.feedback_only,
            "hands": hands,
            "accepted_command": accepted,
        }

    def report_diagnostics(self, state: dict[str, Any], now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        if self._feedback_wait_started_us is None:
            self._feedback_wait_started_us = now
        details = {}
        abnormal = bool(self.hold_control_error or self.last_write_error or self.last_command_error or
                        self.last_command_duration_ms >= self.command_timeout_us / 2000)
        for side in SIDES:
            hand = state["hands"][side]
            snapshot = self.snapshots[side]
            invalid_reason = ""
            if self.links[side] is None:
                invalid_reason = "not_configured"
            elif not snapshot.data:
                invalid_reason = "no_feedback"
            elif not hand["feedback_available"]:
                invalid_reason = "feedback_timestamp_expired_or_future"
            elif snapshot.error:
                invalid_reason = "feedback_read_error"
            elif not hand["valid"]:
                invalid_reason = "angle_register_out_of_range"
            details[side] = {
                "configured": self.links[side] is not None, "valid": hand["valid"], "status": hand["status"],
                "host": getattr(self.links[side], "host", None), "port": getattr(self.links[side], "port", None),
                "invalid_reason": invalid_reason,
                "write": dict(self.write_stats[side]),
                "feedback_age_ms": hand["feedback_age_ms"], "read_duration_ms": snapshot.read_duration_ms,
                "read_in_progress_ms": age_ms(now, snapshot.read_started_us) if snapshot.read_in_progress else None,
                "read_started_us": snapshot.read_started_us, "read_finished_us": snapshot.read_finished_us,
                "last_success_finished_us": snapshot.last_success_finished_us,
                "last_success_finish_age_ms": age_ms(now, snapshot.last_success_finished_us),
                "sample_mono_us": hand["sample_mono_us"],
                "sample_timestamp_offset_ms": snapshot.sample_timestamp_offset_ms,
                "read_start_gap_ms": snapshot.read_start_gap_ms,
                "max_read_start_gap_ms": snapshot.max_read_start_gap_ms,
                "read_idle_gap_ms": snapshot.read_idle_gap_ms,
                "current_read_idle_ms": age_ms(now, snapshot.read_finished_us) if not snapshot.read_in_progress else None,
                "reads": snapshot.reads, "errors": snapshot.errors, "last_error": snapshot.error,
                "actual": hand["drive_position_normalized"], "target": self.last_targets[side],
                "requested_target": self.requested_targets[side],
                "closing_hold_active": hand["closing_hold_active"],
                "angle_raw": hand["drive_position_raw"], "error_codes": hand["error_codes"],
                "current": hand["current"], "force": hand["force"], "temperature": hand["temperature"],
                "current_register_raw": hand["current_register_raw"],
                "hold_control": hand["hold_control"],
            }
            if any(hand["error_codes"]):
                requested = (normalized_to_raw(self.requested_targets[side])
                             if self.requested_targets[side] else [None] * 6)
                applied = self._angle_applied.get(side, [None] * 6)
                self.diag.emit("device_error", key=f"device_error:{side}", reason=str(hand["error_codes"]),
                               side=side, host=details[side]["host"], port=details[side]["port"],
                               command_valid=self.command_valid,
                               channels=[{"channel": i, "channel_name": CHANNEL_NAMES[i], "code": code,
                                          "error_names": [name for bit, name in DEVICE_ERRORS if int(code) & bit],
                                          "unknown_bits": int(code) & ~31,
                                          "current_raw": hand["current"][i], "force_raw": hand["force"][i],
                                          "current_register_raw": hand["current_register_raw"][i],
                                          "temperature_c": hand["temperature"][i], "actual_raw": hand["drive_position_raw"][i],
                                          "requested_raw": requested[i], "applied_raw": applied[i]}
                                         for i, code in enumerate(hand["error_codes"]) if code])
            # A first sample still in flight is normal during reader startup.
            feedback_invalid = not hand["valid"] and (snapshot.data is not None or bool(snapshot.error) or
                now - self._feedback_wait_started_us >= self.feedback_timeout_us)
            side_abnormal = self.links[side] is not None and (
                feedback_invalid or hand["status"] == "ERROR" or bool(snapshot.error) or
                snapshot.read_duration_ms >= self.feedback_timeout_us / 2000 or
                snapshot.sample_timestamp_offset_ms >= self.feedback_timeout_us / 2000 or
                (snapshot.read_in_progress and now - snapshot.read_started_us >= self.feedback_timeout_us / 2))
            abnormal |= side_abnormal
            status = (side_abnormal, hand["valid"], hand["status"], snapshot.error)
            previous = self._feedback_status.get(side)
            self._feedback_status[side] = status
            if side_abnormal and status != previous:
                self.diag.emit("feedback_status_changed", key=f"feedback_status:{side}",
                               throttle_us=0, reason=str(status), side=side,
                               reader=self.reader_diagnostics(now), **details[side])
            elif previous and previous[0] and not side_abnormal:
                self.diag.emit("feedback_recovered", key=f"feedback_status:{side}", throttle_us=0,
                               side=side, feedback_age_ms=hand["feedback_age_ms"],
                               hold_control_error=self.hold_control_error)
        if abnormal and self.diagnostic_interval_us and now >= self._next_diagnostic_us:
            self._next_diagnostic_us = now + self.diagnostic_interval_us
            self.diag.emit("health", throttle_us=0, state_sequence=state["sequence"],
                           state_valid=state["valid"], command_valid=self.command_valid,
                           hold_control_error=self.hold_control_error,
                           feedback_only=self.feedback_only, received=self.command_received,
                           accepted=self.command_accepted, rejected=self.command_rejected,
                           receive_age_ms=age_ms(now, self.last_received_us),
                           last_command_duration_ms=self.last_command_duration_ms,
                           last_write_error=self.last_write_error, last_command_error=self.last_command_error,
                           state_sent=self.state_sent,
                           state_dropped=self.state_dropped, max_state_gap_ms=self.max_state_gap_ms,
                           state_send_age_ms=age_ms(now, self.last_state_sent_us),
                           feedback_timeout_ms=self.feedback_timeout_us / 1000,
                           reader=self.reader_diagnostics(now), hands=details, **self.command_context(now))


def monotonic_us() -> int:
    return time.monotonic_ns() // 1000


def feedback_read_loop(node: Rh56FtpNode, reader_stop: threading.Event, period_us: int) -> None:
    try:
        while not reader_stop.is_set():
            node.reader_poll_started(period_us)
            node.read_states()
            node.reader_poll_finished(period_us)
            remaining_us = node.reader_timing["next_poll_due_us"] - monotonic_us()
            reader_stop.wait(max(0, remaining_us) / 1000000.0)
    except Exception as exc:
        node.diag.emit("feedback_reader_failed", reason=f"{type(exc).__name__}: {exc}",
                       reader=node.reader_diagnostics(monotonic_us()))
    finally:
        node.reader_timing = {**node.reader_timing, "phase": "stopped"}


def run_node(node: Rh56FtpNode, endpoint: str, state_endpoint: str, state_hz: float) -> int:
    try:
        import zmq
    except ImportError as exc:  # pragma: no cover - deployment dependency
        raise RuntimeError("缺少 pyzmq；请在运行 AVIATOR 节点的 Python 环境安装 pyzmq") from exc
    if not math.isfinite(state_hz) or state_hz <= 0 or state_hz > 100:
        raise ValueError("state_hz must be in (0,100]")
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    pub = context.socket(zmq.PUB)
    sub.setsockopt(zmq.RCVHWM, 32)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.SUBSCRIBE, b"hand.command")
    pub.setsockopt(zmq.SNDHWM, 8)
    pub.setsockopt(zmq.LINGER, 0)
    sub.connect(endpoint)
    pub.connect(state_endpoint)
    stopped = False

    def stop(*_args):
        nonlocal stopped
        stopped = True

    old_int = signal.signal(signal.SIGINT, stop)
    old_term = signal.signal(signal.SIGTERM, stop)
    period_us = round(1000000.0 / state_hz)
    next_state_us = monotonic_us()
    reader_stop = threading.Event()

    reader: threading.Thread | None = None
    try:
        node.connect()
        if not node.feedback_only:
            try:
                node._safe_pose(force_open=True)
            except Exception as exc:
                node.diag.emit("startup_safe_pose_failed", reason=f"{type(exc).__name__}: {exc}")
        reader = threading.Thread(target=feedback_read_loop, args=(node, reader_stop, period_us),
                                  name="rh56ftp-state", daemon=True)
        node.reader_thread = reader
        reader.start()
        while not stopped:
            batch_started = monotonic_us()
            batch_count = 0
            for _ in range(32):
                # A command backlog must not postpone feedback publication for
                # up to 32 serial Modbus writes. Finish the current write, then yield.
                if monotonic_us() - batch_started >= min(10000, period_us // 2):
                    break
                if not sub.poll(0):
                    break
                frames = sub.recv_multipart()
                batch_count += 1
                if len(frames) != 2:
                    node.diag.emit("invalid_wire_message", frame_count=len(frames))
                    continue
                try:
                    node.handle_command(frames[0].decode("ascii"), frames[1])
                except UnicodeDecodeError as exc:
                    node.diag.emit("invalid_wire_message", reason=str(exc))
                except Exception:
                    pass  # handle_command already logged the rejection with command/ACK context.
            batch_duration = age_ms(monotonic_us(), batch_started) or 0
            if batch_duration >= node.command_timeout_us / 2000:
                node.diag.emit("command_batch_slow", count=batch_count, duration_ms=batch_duration,
                               **node.command_context(monotonic_us()))
            node.supervise()
            if monotonic_us() >= next_state_us:
                state = node.make_state()
                payload = json.dumps(state, allow_nan=False, separators=(",", ":")).encode()
                try:
                    pub.send_multipart([b"hand.state", payload], zmq.NOBLOCK)
                    sent_at = monotonic_us()
                    gap_ms = age_ms(sent_at, node.last_state_sent_us) or 0
                    node.max_state_gap_ms = max(node.max_state_gap_ms, gap_ms)
                    node.last_state_sent_us = sent_at
                    node.state_sent += 1
                    if gap_ms >= node.feedback_timeout_us / 2000:
                        node.diag.emit("state_publish_gap", gap_ms=gap_ms,
                                       feedback_timeout_ms=node.feedback_timeout_us / 1000)
                except zmq.Again:
                    node.state_dropped += 1
                    node.diag.emit("state_publish_dropped", dropped=node.state_dropped)
                node.report_diagnostics(state)
                next_state_us += period_us
                if next_state_us <= monotonic_us():
                    next_state_us = monotonic_us() + period_us
            time.sleep(0.001)
    finally:
        reader_stop.set()
        if reader is not None:
            reader.join(timeout=5.0)
        try:
            if not node.feedback_only:
                node._safe_pose(force_open=True)
        except Exception as exc:
            node.diag.emit("shutdown_safe_pose_failed", reason=f"{type(exc).__name__}: {exc}")
        node.close()
        sub.close(0)
        pub.close(0)
        context.term()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="YAML 配置文件（speed、force、threshold、right_host、left_host）；显式命令行参数优先")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5556", help="hand.command SUB connects to bus egress")
    parser.add_argument("--state-endpoint", default="tcp://127.0.0.1:5555", help="hand.state PUB connects to bus ingress")
    parser.add_argument("--right-host", default=None)
    parser.add_argument("--left-host", default=None, help="optional second RH56FTP device")
    parser.add_argument("--right-port", type=int, default=None)
    parser.add_argument("--left-port", type=int, default=None)
    parser.add_argument("--publisher-id", default="rh56ftp_hand")
    parser.add_argument("--state-hz", type=float, default=DEFAULT_STATE_HZ,
                        help="状态读取和发布频率（默认 1000/60 Hz，即 60 ms 周期）")
    parser.add_argument("--command-timeout-ms", type=int, default=100)
    parser.add_argument("--feedback-timeout-ms", type=int, default=500)
    parser.add_argument("--modbus-timeout", type=float, default=0.2,
                        help="每个 Modbus TCP 请求超时秒数（读写连接均使用）")
    parser.add_argument("--speed", type=int, default=None,
                        help="运动/抓取速度设定，0..1000（默认 500）")
    parser.add_argument("--force", type=int, default=None,
                        help="运动/抓取力阈值设定，0..3000（默认 1000）")
    parser.add_argument("--hold-speed", type=int, default=None,
                        help="保持速度设定，0..1000（默认 100）")
    parser.add_argument("--hold-force", type=int, default=None,
                        help="保持力阈值设定，0..3000（默认 100）")
    parser.add_argument("--closing-hold-ms", type=int, default=10000,
                        help="持续位置偏差及位置稳定的观察时间（默认 10000 ms）")
    parser.add_argument("--threshold", "--closing-motion-raw", dest="closing_motion_raw", type=int, default=None,
                        help="位置偏差及稳定变化范围阈值，寄存器刻度（默认 10，范围 0..999）")
    parser.add_argument("--diagnostic-interval-s", type=float, default=1.0,
                        help="JSON 模式的异常详情汇总间隔秒数（默认 1；0 关闭汇总）")
    parser.add_argument("--log-format", choices=("text", "json"), default="text",
                        help="日志格式：默认 text 简洁中文；json 保留完整排障上下文")
    parser.add_argument("--feedback-only", action="store_true")
    args = parser.parse_args(argv)
    settings = {}
    if args.config:
        try:
            import yaml
            settings = yaml.safe_load(args.config.read_text(encoding="utf-8"))
            if not isinstance(settings, dict) or set(settings) - {"speed", "force", "threshold", "right_host", "left_host"}:
                raise ValueError("配置必须为仅包含 speed、force、threshold、right_host、left_host 的映射")
        except ImportError:
            parser.error("读取 --config 需要 PyYAML")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    try:
        for name, maximum, grasp_default, hold_default in (
                ("speed", 1000, DEFAULT_SPEED, DEFAULT_HOLD_SPEED),
                ("force", 3000, DEFAULT_FORCE, DEFAULT_HOLD_FORCE)):
            value = settings.get(name, {})
            if not isinstance(value, dict):
                value = {"grasp": value}  # Legacy scalar settings apply to motion.
            if set(value) - {"grasp", "hold"}:
                raise ValueError(f"{name} 仅支持 grasp、hold")
            for phase, option, default in (("grasp", name, grasp_default),
                                            ("hold", f"hold_{name}", hold_default)):
                override = getattr(args, option)
                setattr(args, option, _register_setting(
                    override if override is not None else value.get(phase, default),
                    f"{name}.{phase}", maximum))
        args.closing_motion_raw = _register_setting(
            args.closing_motion_raw if args.closing_motion_raw is not None else settings.get("threshold", 10),
            "threshold", 999)
        for name in ("right_host", "left_host"):
            value = getattr(args, name)
            if value is None and name in settings:
                value = settings[name]
                if not isinstance(value, str):
                    raise ValueError(f"{name} 必须为字符串（空字符串禁用该手）")
                setattr(args, name, value)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.modbus_timeout) or args.modbus_timeout <= 0:
        parser.error("--modbus-timeout 必须为正数")
    if not math.isfinite(args.state_hz) or not 0 < args.state_hz <= 100:
        parser.error("--state-hz 必须在 (0,100]")
    if args.closing_hold_ms <= 0 or not 0 <= args.closing_motion_raw < 1000:
        parser.error("--closing-hold-ms 必须为正数，--closing-motion-raw 必须在 [0,999]")
    if not 0 <= args.speed <= 1000:
        parser.error("--speed 必须为 0..1000 的整数")
    if not 0 <= args.force <= 3000:
        parser.error("--force 必须为 0..3000 的整数")
    if not math.isfinite(args.diagnostic_interval_s) or args.diagnostic_interval_s < 0:
        parser.error("--diagnostic-interval-s 必须为非负有限数")
    hand_ip, hand_port, hand_link = load_handlink()
    right_host = hand_ip if args.right_host is None else args.right_host
    right_port = hand_port if args.right_port is None else args.right_port
    if args.left_host and args.left_port is None:
        args.left_port = right_port
    if not args.left_host:
        args.left_port = None
    command_links = {
        "right": hand_link(right_host, right_port, args.modbus_timeout) if right_host else None,
        "left": hand_link(args.left_host, args.left_port, args.modbus_timeout) if args.left_host else None,
    }
    read_links = {
        "right": hand_link(right_host, right_port, args.modbus_timeout) if right_host else None,
        "left": hand_link(args.left_host, args.left_port, args.modbus_timeout) if args.left_host else None,
    }
    node = Rh56FtpNode(command_links, read_links=read_links, publisher_id=args.publisher_id,
                       command_timeout_ms=args.command_timeout_ms,
                       feedback_timeout_ms=args.feedback_timeout_ms,
                       feedback_only=args.feedback_only, speed=args.speed, force=args.force,
                       hold_speed=args.hold_speed, hold_force=args.hold_force,
                       diagnostic_interval_s=args.diagnostic_interval_s,
                       closing_hold_ms=args.closing_hold_ms, closing_motion_raw=args.closing_motion_raw,
                       log_format=args.log_format)
    parameters = dict(vars(args))
    parameters.update(config=str(args.config.resolve()) if args.config else None,
                      right_host=right_host, right_port=right_port, left_host=args.left_host or "",
                      configured_mode=POSITION_FORCE_PROTECTION_MODE, safe_pose=list(DEFAULT_SAFE_POSE))
    for option, name in (("speed", "speed.grasp"), ("force", "force.grasp"),
                         ("hold_speed", "speed.hold"), ("hold_force", "force.hold"),
                         ("closing_motion_raw", "threshold")):
        parameters[name] = parameters.pop(option)
    node.diag.emit("configuration_loaded", throttle_us=0, parameters=parameters)
    return run_node(node, args.endpoint, args.state_endpoint, args.state_hz)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        Logger.error(f"rh56ftp_hand: {exc}", file=sys.stderr)
        raise SystemExit(1)
