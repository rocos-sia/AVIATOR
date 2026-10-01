#!/usr/bin/env python3
"""RH56FTP Modbus TCP backend for the AVIATOR ``hand.command`` bus.

The RH56FTP pendant is the source of truth for the Modbus register map.  This
node only uses its non-tactile ``read_state`` path and publishes the result as
AVIATOR ``hand.state`` messages.  One RH56FTP device can be used as the right
hand; pass ``--left-host`` as well when two devices are present.
"""
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
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any


MAX_JSON_INTEGER = (1 << 53) - 1
SIDES = ("left", "right")
DEFAULT_SAFE_POSE = (1.0,) * 6
DEFAULT_SPEED = 500
DEFAULT_FORCE = 500
DEFAULT_STATE_HZ = 1000.0 / 60.0
UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$")


def _pendant_import_path() -> None:
    """Make the source tree and an installed sibling package importable."""
    here = Path(__file__).resolve().parent
    candidates = (here, here.parent.parent / "RH56FTP" / "python")
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
        raise RuntimeError("需要 pymodbus，并且 RH56FTP/python/pendant 可导入") from exc
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


def canonical_to_rh(values: list[int] | tuple[int, ...]) -> list[int]:
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
    """Bound repeated faults while keeping reader/main-thread lines intact."""

    def __init__(self):
        self._lock = threading.Lock()
        self._events: dict[str, tuple[int, str, int]] = {}

    def emit(self, event: str, *, key: str | None = None, reason: str = "",
             throttle_us: int = 1000000, **fields: Any) -> None:
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
            print("rh56ftp_hand: " + json.dumps(record, ensure_ascii=False, allow_nan=False),
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
    samples: deque[tuple[int, int]] = field(default_factory=deque)
    held_raw: int | None = None


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
                 closing_hold_ms: int = 5000, closing_motion_raw: int = 10):
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
        self._settings_applied: set[str] = set()
        self.clock_id = clock_id or _clock_id()
        self.session_id = session_id or f"run-{time.time_ns()}-{os.getpid()}"
        self.guard = CommandGuard()
        self.snapshots = {side: Snapshot() for side in SIDES}
        self.last_command: dict[str, Any] | None = None
        self.last_targets: dict[str, list[float]] = {side: [] for side in SIDES}
        self.requested_targets: dict[str, list[float]] = {side: [] for side in SIDES}
        self.closing_hold_us = closing_hold_ms * 1000
        self.closing_motion_raw = closing_motion_raw
        self.closing_holds = {side: [ClosingHold() for _ in range(6)] for side in SIDES}
        self.command_valid = False
        self.state_sequence = 0
        self._safe_applied = False
        self._safe_required = not feedback_only
        self.diag = DiagnosticLog()
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
        self._settings_applied.clear()
        self._reset_closing_holds()
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
                               errors=snapshot.errors, feedback_age_ms=age_ms(monotonic_us(), snapshot.sample_mono_us))
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
            operation()
        except Exception as exc:
            stats["errors"] += 1
            self.last_write_error = f"{side} {phase}: {type(exc).__name__}: {exc}"
            self.diag.emit("modbus_write_failed", key=f"write_failed:{side}", side=side, phase=phase,
                           target_raw=target_raw, duration_ms=age_ms(monotonic_us(), started),
                           reason=self.last_write_error)
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
            if link is not None and side not in self._settings_applied:
                try:
                    self._write_call(side, "speed", lambda: link.write_speed_set([self.speed] * 6))
                    self._write_call(side, "force", lambda: link.write_force_set([self.force] * 6))
                except Exception as exc:
                    raise RuntimeError(f"{side} speed/force setup failed: {exc}") from exc
                self._settings_applied.add(side)
        for side, link in self.links.items():
            if link is not None:
                try:
                    self._write_call(side, "angle", lambda: link.write_angle_set(canonical_to_rh(raw_by_side[side])),
                                     raw_by_side[side])
                except Exception:
                    # Reapply settings on the next attempt after a write error.
                    self._settings_applied.discard(side)
                    raise
        self.last_write_error = ""

    def _safe_pose(self) -> None:
        self._reset_closing_holds()
        raw = normalized_to_raw(list(DEFAULT_SAFE_POSE))
        self._write_targets({side: raw for side in SIDES})
        self.last_targets = {side: [value / 1000.0 for value in raw] for side in SIDES}
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
            channels = [ClosingHold()]  # Thumb rotation is never constrained.
            for index in range(1, 6):
                target = requested[side][index]
                previous = self.closing_holds[side][index]
                channel = replace(previous, samples=deque(previous.samples))
                if (channel.requested_raw != target or now < channel.last_command_us or
                        now - channel.last_command_us >= self.command_timeout_us):
                    channel = ClosingHold(requested_raw=target, target_since_us=now,
                                          feedback_errors=snapshot.errors)
                channel.last_command_us = now
                # Keep the frozen target for this same request, including when
                # feedback is temporarily unavailable. Never chase live position.
                if channel.held_raw is not None:
                    effective[side][index] = channel.held_raw
                elif not fresh:
                    channel.samples.clear()
                    channel.last_sample_us = 0
                else:
                    stamp = snapshot.sample_mono_us
                    position = actual[index]
                    if (snapshot.errors != channel.feedback_errors or
                            stamp < channel.last_sample_us or
                            (channel.last_sample_us and stamp - channel.last_sample_us >= self.feedback_timeout_us)):
                        channel.samples.clear()
                    channel.feedback_errors = snapshot.errors
                    if position - target <= self.closing_motion_raw:
                        # Opening, already at target, or within the residual tolerance.
                        channel.samples.clear()
                    elif stamp >= channel.target_since_us and stamp != channel.last_sample_us:
                        channel.samples.append((stamp, position))
                        cutoff = stamp - self.closing_hold_us
                        # Retain one sample at/before the window boundary, so
                        # sparse polling cannot claim a full five-second window.
                        while len(channel.samples) > 1 and channel.samples[1][0] <= cutoff:
                            channel.samples.popleft()
                        if stamp - channel.samples[0][0] >= self.closing_hold_us:
                            positions = [value for _, value in channel.samples]
                            if max(positions) - min(positions) <= self.closing_motion_raw:
                                channel.held_raw = position
                                effective[side][index] = position
                    channel.last_sample_us = stamp
                channels.append(channel)
            pending[side] = channels
        return effective, pending

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
            self.last_ack_write_us = monotonic_us()
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
            self._write_targets(raw_by_side)
            # Failed writes must not acknowledge or latch a new held position.
            self.closing_holds = pending_holds
            self.requested_targets = {side: [value / 1000.0 for value in requested[side]] for side in SIDES}
            self.last_targets = {
                side: [value / 1000.0 for value in raw_by_side[side]] for side in SIDES
            }
            self.command_valid = True
            self._safe_applied = False
            self._safe_required = False
        else:
            self._safe_pose()
            self.last_targets = {side: [] for side in SIDES}
            self.requested_targets = {side: [] for side in SIDES}
        self.guard = candidate
        self.last_command = command

    def supervise(self, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        if self.command_valid and self.guard.expired(now, self.command_timeout_us):
            self.diag.emit("command_watchdog_expired", receive_age_ms=age_ms(now, self.guard.last_recv_mono_us),
                           origin_age_ms=age_ms(now, self.guard.origin_mono_us), **self.command_context(now))
            self._safe_required = True
        if self._safe_required and not self._safe_applied:
            try:
                self._safe_pose()
            except Exception as exc:
                self.diag.emit("safe_pose_failed", reason=f"{type(exc).__name__}: {exc}")
                # Keep the requirement pending so the next loop retries the
                # protective write instead of declaring safety.
                self._safe_applied = False

    @staticmethod
    def _canonical_state(data: dict[str, Any]) -> dict[str, Any]:
        angle = rh_to_canonical(data["angle"])
        valid_position = all(isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v <= 1000
                             for v in angle)
        normalized = [float(v) / 1000.0 for v in angle] if valid_position else []
        return {
            "angle_raw": angle,
            "drive_position_raw": angle,
            "drive_position_normalized": normalized,
            "force": rh_to_canonical(data["force"]),
            "current": rh_to_canonical(data["current"]),
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
                measurement_valid = bool(measured["drive_position_normalized"])
                if link is not None:
                    configured_fresh.append(measurement_valid)
                error_codes = measured["error_codes"]
                status = "ERROR" if any(error_codes) else ("ACTIVE" if self.command_valid else "READY")
                hand = {
                    "valid": measurement_valid,
                    "status": status,
                    "enabled": self.command_valid,
                    # Keep the pendant field names available for consumers that
                    # already use HandLink.read_state(), in canonical order.
                    "angle": measured["angle_raw"],
                    "force": measured["force"],
                    "current": measured["current"],
                    "err": measured["error_codes"],
                    "error": measured["error_codes"],
                    "status_code": measured["status_codes"],
                    "status_values": measured["status_codes"],
                    "temp": measured["temperature"],
                    "error_code": max(error_codes, default=0),
                    "error_codes": error_codes,
                    "status_codes": measured["status_codes"],
                    "temperature": measured["temperature"],
                    "feedback_available": True,
                    "position_source": "angle_act_register",
                    "sample_mono_us": snapshot.sample_mono_us,
                    "sample_time_basis": "host_modbus_read",
                    "feedback_age_ms": (now - (snapshot.sample_mono_us or now)) / 1000.0,
                    "angle_raw": measured["angle_raw"],
                    "drive_position_raw": measured["drive_position_raw"],
                    "drive_position_normalized": measured["drive_position_normalized"],
                    "commanded_drive_position_normalized": self.last_targets[side],
                    "joint_position": None,
                    "joint_velocity": None,
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
                    "enabled": False,
                    "angle": [],
                    "force": [],
                    "current": [],
                    "err": [],
                    "error": [],
                    "status_code": [],
                    "status_values": [],
                    "temp": [],
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
                    "angle_raw": [],
                    "drive_position_raw": [],
                    "drive_position_normalized": [],
                    "commanded_drive_position_normalized": self.last_targets[side],
                    "joint_position": None,
                    "joint_velocity": None,
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
            "valid": bool(configured_fresh) and all(configured_fresh),
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
        abnormal = bool(self.last_write_error or self.last_command_error or
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
                "angle_raw": hand["angle_raw"], "error_codes": hand["error_codes"],
            }
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
                               reason=str(status), side=side, reader=self.reader_diagnostics(now), **details[side])
        if abnormal and self.diagnostic_interval_us and now >= self._next_diagnostic_us:
            self._next_diagnostic_us = now + self.diagnostic_interval_us
            self.diag.emit("health", throttle_us=0, state_sequence=state["sequence"],
                           state_valid=state["valid"], command_valid=self.command_valid,
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
                node._safe_pose()
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
                node._safe_pose()
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
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="所有已连接手的六路速度设定，0..1000（默认 500）")
    parser.add_argument("--force", type=int, default=DEFAULT_FORCE,
                        help="所有已连接手的六路力阈值设定，0..3000（默认 500）")
    parser.add_argument("--closing-hold-ms", type=int, default=5000,
                        help="五个弯曲通道握紧目标不变且运动很小的观察窗口（默认 5000 ms）")
    parser.add_argument("--closing-motion-raw", type=int, default=10,
                        help="握紧停滞窗口内允许的最大位置范围及目标残差容差（默认 10，范围 0..999）")
    parser.add_argument("--diagnostic-interval-s", type=float, default=1.0,
                        help="异常详情汇总间隔秒数（默认 1；0 只保留异常事件；正常运行不打印）")
    parser.add_argument("--feedback-only", action="store_true")
    args = parser.parse_args(argv)
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
                       diagnostic_interval_s=args.diagnostic_interval_s,
                       closing_hold_ms=args.closing_hold_ms, closing_motion_raw=args.closing_motion_raw)
    return run_node(node, args.endpoint, args.state_endpoint, args.state_hz)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"rh56ftp_hand: {exc}", file=sys.stderr)
        raise SystemExit(1)
