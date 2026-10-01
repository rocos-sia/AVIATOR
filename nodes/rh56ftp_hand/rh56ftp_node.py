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
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MAX_JSON_INTEGER = (1 << 53) - 1
SIDES = ("left", "right")
DEFAULT_SAFE_POSE = (1.0,) * 6
DEFAULT_SPEED = 500
DEFAULT_FORCE = 500
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
    _identifier(message.get("session_id"), "session_id", True)
    _identifier(message.get("control_epoch"), "control_epoch", True)
    if not isinstance(message.get("valid"), bool):
        raise ValueError("valid must be boolean")
    _identifier(message.get("mode"), "mode")

    origin = message.get("origin")
    if not isinstance(origin, dict):
        raise ValueError("origin must be an object")
    _identifier(origin.get("publisher_id"), "origin.publisher_id")
    _identifier(origin.get("session_id"), "origin.session_id", True)
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


@dataclass
class Snapshot:
    data: dict[str, Any] | None = None
    sample_mono_us: int | None = None
    error: str = ""
    reads: int = 0
    errors: int = 0

    def fresh(self, now: int, timeout_us: int) -> bool:
        return self.data is not None and self.sample_mono_us is not None and 0 <= now - self.sample_mono_us < timeout_us


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
            or command["session_id"] != self.session
            or command["control_epoch"] != self.epoch
            or origin["publisher_id"] != self.origin_publisher
            or origin["session_id"] != self.origin_session
        ):
            raise PermissionError("publisher/session/epoch/origin mismatch")
        if command["sequence"] <= self.last_sequence:
            raise ValueError("stale command sequence")
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
                 speed: int = DEFAULT_SPEED, force: int = DEFAULT_FORCE):
        if set(links) != set(SIDES) or not any(links.values()):
            raise ValueError("at least one left/right RH56FTP link is required")
        if command_timeout_ms <= 0 or feedback_timeout_ms <= 0:
            raise ValueError("timeouts must be positive")
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
        self.session_id = session_id or str(uuid.uuid4())
        self.guard = CommandGuard()
        self.snapshots = {side: Snapshot() for side in SIDES}
        self.last_command: dict[str, Any] | None = None
        self.last_targets: dict[str, list[float]] = {side: [] for side in SIDES}
        self.command_valid = False
        self.state_sequence = 0
        self._safe_applied = False
        self._safe_required = not feedback_only

    def connect(self) -> None:
        self._settings_applied.clear()
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

    def read_states(self, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        for side, link in self.read_links.items():
            if link is None:
                continue
            snapshot = self.snapshots[side]
            try:
                data = link.read_state()
                if not isinstance(data, dict):
                    raise ValueError("RH56FTP state must be an object")
                for key in ("angle", "force", "current", "err", "status", "temp"):
                    if not isinstance(data.get(key), list) or len(data[key]) != 6:
                        raise ValueError(f"RH56FTP state field {key} must contain six values")
                    for value in data[key]:
                        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                            raise ValueError(f"RH56FTP state field {key} contains a non-finite value")
                snapshot.data = data
                snapshot.sample_mono_us = now
                snapshot.error = ""
                snapshot.reads += 1
            except Exception as exc:
                snapshot.error = str(exc)
                snapshot.errors += 1

    def _write_targets(self, raw_by_side: dict[str, list[int]]) -> None:
        if self.feedback_only:
            return
        # Configure every connected hand before moving either hand. Cache
        # successful writes to avoid adding Modbus traffic at command rate.
        for side, link in self.links.items():
            if link is not None and side not in self._settings_applied:
                try:
                    link.write_speed_set([self.speed] * 6)
                    link.write_force_set([self.force] * 6)
                except Exception as exc:
                    raise RuntimeError(f"{side} speed/force setup failed: {exc}") from exc
                self._settings_applied.add(side)
        for side, link in self.links.items():
            if link is not None:
                try:
                    link.write_angle_set(canonical_to_rh(raw_by_side[side]))
                except Exception:
                    # Reapply settings on the next attempt after a write error.
                    self._settings_applied.discard(side)
                    raise

    def _safe_pose(self) -> None:
        raw = normalized_to_raw(list(DEFAULT_SAFE_POSE))
        self._write_targets({side: raw for side in SIDES})
        self.command_valid = False
        self._safe_applied = True
        self._safe_required = False

    def handle_command(self, topic: str, payload: bytes | str, now: int | None = None) -> bool:
        if self.feedback_only:
            return False
        now = monotonic_us() if now is None else now
        command = decode_command(topic, payload)
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
            self._write_targets(raw_by_side)
            self.last_targets = {
                side: [value / 1000.0 for value in raw_by_side[side]] for side in SIDES
            }
            self.command_valid = True
            self._safe_applied = False
            self._safe_required = False
        else:
            self._safe_pose()
            self.last_targets = {side: [] for side in SIDES}
        self.guard = candidate
        self.last_command = command
        return True

    def supervise(self, now: int | None = None) -> None:
        now = monotonic_us() if now is None else now
        if self.command_valid and self.guard.expired(now, self.command_timeout_us):
            self._safe_required = True
        if self._safe_required and not self._safe_applied:
            try:
                self._safe_pose()
            except Exception:
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
        now = monotonic_us() if now is None else now
        self.state_sequence += 1
        hands: dict[str, dict[str, Any]] = {}
        configured_fresh = []
        for side in SIDES:
            link = self.links[side]
            snapshot = self.snapshots[side]
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


def monotonic_us() -> int:
    return time.monotonic_ns() // 1000


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
    print(f"rh56ftp_hand: SUB hand.command={endpoint}; PUB hand.state={state_endpoint}", flush=True)
    stopped = False

    def stop(*_args):
        nonlocal stopped
        stopped = True

    old_int = signal.signal(signal.SIGINT, stop)
    old_term = signal.signal(signal.SIGTERM, stop)
    period = 1.0 / state_hz
    next_state = time.monotonic()
    reader_stop = threading.Event()

    def read_loop() -> None:
        period = 1.0 / state_hz
        while not reader_stop.is_set():
            node.read_states()
            reader_stop.wait(period)

    reader: threading.Thread | None = None
    try:
        node.connect()
        if not node.feedback_only:
            try:
                node._safe_pose()
            except Exception as exc:
                print(f"rh56ftp_hand: startup safe pose failed: {exc}", file=sys.stderr)
        reader = threading.Thread(target=read_loop, name="rh56ftp-state", daemon=True)
        reader.start()
        while not stopped:
            now_wall = time.monotonic()
            for _ in range(32):
                if not sub.poll(0):
                    break
                frames = sub.recv_multipart()
                if len(frames) != 2:
                    continue
                try:
                    node.handle_command(frames[0].decode("ascii"), frames[1])
                except Exception as exc:
                    print(f"rh56ftp_hand: reject command: {exc}", file=sys.stderr)
            node.supervise()
            if now_wall >= next_state:
                payload = json.dumps(node.make_state(), allow_nan=False, separators=(",", ":")).encode()
                try:
                    pub.send_multipart([b"hand.state", payload], zmq.NOBLOCK)
                except zmq.Again:
                    pass
                next_state = now_wall + period
            time.sleep(0.001)
    finally:
        reader_stop.set()
        if reader is not None:
            reader.join(timeout=5.0)
        try:
            if not node.feedback_only:
                node._safe_pose()
        except Exception as exc:
            print(f"rh56ftp_hand: safe pose failed: {exc}", file=sys.stderr)
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
    parser.add_argument("--state-hz", type=float, default=10.0)
    parser.add_argument("--command-timeout-ms", type=int, default=100)
    parser.add_argument("--feedback-timeout-ms", type=int, default=500)
    parser.add_argument("--modbus-timeout", type=float, default=0.2,
                        help="每个 Modbus TCP 请求超时秒数（读写连接均使用）")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="所有已连接手的六路速度设定，0..1000（默认 500）")
    parser.add_argument("--force", type=int, default=DEFAULT_FORCE,
                        help="所有已连接手的六路力阈值设定，0..3000（默认 500）")
    parser.add_argument("--feedback-only", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.modbus_timeout) or args.modbus_timeout <= 0:
        parser.error("--modbus-timeout 必须为正数")
    if not 0 <= args.speed <= 1000:
        parser.error("--speed 必须为 0..1000 的整数")
    if not 0 <= args.force <= 3000:
        parser.error("--force 必须为 0..3000 的整数")
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
                       feedback_only=args.feedback_only, speed=args.speed, force=args.force)
    return run_node(node, args.endpoint, args.state_endpoint, args.state_hz)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"rh56ftp_hand: {exc}", file=sys.stderr)
        raise SystemExit(1)
