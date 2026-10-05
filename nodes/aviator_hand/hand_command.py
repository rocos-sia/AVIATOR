#!/usr/bin/env python3
"""Manual hand.command publisher. Run on the same host as inspire_hand_node."""

# Shared logging module in source and installed share/aviator layouts.
import sys as _log_sys
from pathlib import Path as _LogPath
_log_sys.path.insert(0, str(_LogPath(__file__).resolve().parents[1] / "common")
                     if (_LogPath(__file__).resolve().parents[1] / "common").is_dir()
                     else str(_LogPath(__file__).resolve().parents[2] / "common"))
from aviator_logger import Logger


import argparse
import os
import json
import math
import queue
import signal
import socket
import sys
import threading
import time
import uuid
from pathlib import Path


PRESETS = {"open": 1.0, "half": 0.5, "close": 0.0}
HELP = """命令：
  both open                  双手张开（首次请设置 both 或使用命令行双手目标）
  right 0.9                  右手六路设为 0.9，左手保持上次目标
  left 1 1 0.8 1 1 1         只调整左手食指
  both half / both close     双手半握 / 闭合
  stop                       回落节点配置的安全姿态，暂停发布运动目标
  quit                       回落安全姿态并退出
数值 0=闭合，1=张开；顺序：拇指旋转、拇指弯曲、食指、中指、无名指、小指。
目标持续以 50 Hz 发布，直到下一条命令、stop 或退出。help 显示帮助。"""


def positions(text):
    """Parse a preset, one repeated value, or six normalized drive values."""
    parts = text.replace(",", " ").split()
    if len(parts) == 1 and parts[0] in PRESETS:
        return [PRESETS[parts[0]]] * 6
    try:
        values = [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError("目标应为 open/half/close 或 1 个/6 个数字") from exc
    if len(values) == 1:
        values *= 6
    if len(values) != 6 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
        raise ValueError("需要 1 个或 6 个有限数字，范围 [0,1]")
    return values


def update_targets(line, targets):
    side, sep, value = line.strip().partition(" ")
    if side not in ("both", "left", "right") or not sep:
        raise ValueError("使用 both/left/right 加目标，例如 both open")
    values = positions(value)
    if targets is None and side != "both":
        raise ValueError("首次必须指定 both；尚不知道另一只手的目标")
    result = dict(targets or {})
    for name in (("left", "right") if side == "both" else (side,)):
        result[name] = list(values)
    return result


class Commands:
    def __init__(self):
        self.session = f"run-{time.time_ns()}-{os.getpid()}"
        self.epoch = str(uuid.uuid4())
        self.publisher = "hand_manual_test"
        self.clock = socket.gethostname() + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.sequence = 0

    def message(self, targets):
        self.sequence += 1
        mono = time.monotonic_ns() // 1000
        msg = dict(msg_type="HandCommand", version="1.0", sequence=self.sequence,
                   timestamp=time.time_ns() // 1000, sample_mono_us=mono,
                   clock_id=self.clock, publisher_id=self.publisher,
                   session_id=self.session, control_epoch=self.epoch,
                   valid=targets is not None, mode="NORMALIZED_POSITION")
        msg["origin"] = dict(publisher_id=self.publisher, session_id=self.session,
                             sequence=self.sequence, sample_mono_us=mono, clock_id=self.clock)
        if targets is not None:
            msg["hands"] = {side: {"drive_position_normalized": list(targets[side])}
                            for side in ("left", "right")}
        return msg


def read_input(lines):
    for line in sys.stdin:
        lines.put(line.strip())
    lines.put("quit")


def run(args, targets):
    commands = Commands()
    if args.dry_run:
        Logger.output(json.dumps(commands.message(targets), ensure_ascii=False, indent=2))
        return 0
    try:
        import zmq
    except ImportError as exc:
        raise RuntimeError("缺少 pyzmq；使用已安装 pyzmq 的 Python 环境运行") from exc

    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    sub = context.socket(zmq.SUB)
    pub.setsockopt(zmq.SNDHWM, 5)
    sub.setsockopt(zmq.RCVHWM, 20)
    sub.setsockopt(zmq.SUBSCRIBE, b"hand.state")
    lines = queue.Queue()
    sent_any = False
    stopped = threading.Event()
    old_term = signal.signal(signal.SIGTERM, lambda *_: stopped.set())

    def send(value):
        nonlocal sent_any
        pub.send_multipart([b"hand.command", json.dumps(commands.message(value), allow_nan=False).encode()])
        sent_any = True

    def release():
        # Repeat fresh invalidations across multiple bus cycles; watchdog is the fallback.
        if sent_any:
            for _ in range(5):
                send(None)
                time.sleep(0.02)

    try:
        pub.connect(args.endpoint)
        sub.connect(args.state_endpoint)
        Logger.info(f"PUB {args.endpoint}; state {args.state_endpoint}", flush=True)
        Logger.info(f"session={commands.session}\nclock={commands.clock}", flush=True)
        Logger.info("首次使用需启动控制模式手节点；本程序重启会生成新 control_epoch，需重启手节点重新授权。", flush=True)
        # Allow subscriptions to propagate before sending any command.
        time.sleep(0.5)
        if args.interactive:
            Logger.info(HELP, flush=True)
            threading.Thread(target=read_input, args=(lines,), daemon=True).start()
        if targets is not None:
            Logger.info(f"目标 left={targets['left']} right={targets['right']}", flush=True)
        start = time.monotonic()
        active_since = start if targets is not None else None
        last_ack = None
        last_status = 0.0
        next_send = start
        while not stopped.is_set():
            now = time.monotonic()
            if not args.interactive and now - start >= args.duration:
                if last_ack is None:
                    raise RuntimeError("发布结束，但未收到节点接受确认，不能确认指令生效")
                return 0
            try:
                line = lines.get_nowait()
            except queue.Empty:
                line = ""
            if line in ("quit", "q", "exit"):
                return 0
            if line == "help":
                Logger.info(HELP, flush=True)
            elif line == "stop":
                release()
                targets = None
                active_since = last_ack = None
                Logger.info("已发送安全姿态请求；再次运动请先指定 both。", flush=True)
            elif line:
                try:
                    updated = update_targets(line, targets)
                except ValueError as exc:
                    Logger.info(f"输入错误：{exc}", flush=True)
                else:
                    if targets is None:
                        active_since = now
                        last_ack = None
                    targets = updated
                    Logger.info(f"目标 left={targets['left']} right={targets['right']}", flush=True)

            if targets is not None and now >= next_send:
                send(targets)
                next_send = now + 0.02
            # Bounded receive work keeps command publication responsive.
            for _ in range(20):
                if not sub.poll(0):
                    break
                frames = sub.recv_multipart()
                if len(frames) != 2 or frames[0] != b"hand.state":
                    continue
                try:
                    state = json.loads(frames[1])
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(state, dict) or state.get("msg_type") != "HandState":
                    continue
                if targets is not None and state.get("feedback_only"):
                    raise RuntimeError("手节点处于 --feedback-only，只读模式不能运动")
                accepted = state.get("accepted_command") or {}
                if not isinstance(accepted, dict):
                    continue
                ours = (accepted.get("publisher_id") == commands.publisher)
                # Receipt alone is not an ACK: require this publisher and a fresh accepted sample.
                sample = accepted.get("sample_mono_us")
                fresh = (type(sample) is int and 0 <= time.monotonic_ns() // 1000 - sample < 100000)
                if targets is not None and ours and fresh and state.get("command_valid") is True:
                    last_ack = now
                    if now - last_status >= 1:
                        hands = state.get("hands") or {}
                        Logger.info(f"已接受 seq={accepted.get('sequence')} 实际位置 "
                              f"left={hands.get('left', {}).get('drive_position_raw')} "
                              f"right={hands.get('right', {}).get('drive_position_raw')} "
                              f"反馈有效={state.get('valid')}", flush=True)
                        last_status = now
                elif targets is not None and accepted.get("publisher_id") and not ours:
                    raise RuntimeError("手节点已绑定其他发布会话；停止原发布者并重启手节点后重试")
            if targets is not None and now - (last_ack if last_ack is not None else active_since) > 2:
                raise RuntimeError("2 秒未收到有效指令确认；检查 bus、手节点、端点及节点拒绝日志")
            time.sleep(0.002)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            release()
        finally:
            pub.close(linger=0)
            sub.close(linger=0)
            context.term()
            signal.signal(signal.SIGTERM, old_term)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555", help="bus PUB ingress")
    parser.add_argument("--state-endpoint", default="tcp://127.0.0.1:5556", help="bus SUB egress")
    parser.add_argument("--interactive", action="store_true", help="持续读取终端命令；未指定目标时默认启用")
    parser.add_argument("--pose", choices=PRESETS, help="双手预设目标")
    parser.add_argument("--left", help="左手目标，例如 '1,1,0.8,1,1,1'；必须同时提供 --right")
    parser.add_argument("--right", help="右手目标；必须同时提供 --left")
    parser.add_argument("--duration", type=float, default=3.0, help="非交互模式持续秒数，默认 3")
    parser.add_argument("--dry-run", action="store_true", help="只输出一条 JSON，不连接网络")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration 必须为正数")
    if args.pose and (args.left is not None or args.right is not None):
        parser.error("--pose 不能与 --left/--right 混用")
    if (args.left is None) != (args.right is None):
        parser.error("必须同时指定 --left 和 --right")
    try:
        targets = ({"left": positions(args.pose), "right": positions(args.pose)} if args.pose else
                   {"left": positions(args.left), "right": positions(args.right)} if args.left is not None else None)
    except ValueError as exc:
        parser.error(str(exc))
    if targets is None:
        args.interactive = True
    if args.dry_run and targets is None:
        parser.error("--dry-run 需要 --pose 或左右手目标")
    try:
        return run(args, targets)
    except (RuntimeError, OSError) as exc:
        Logger.error(f"hand_command: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
