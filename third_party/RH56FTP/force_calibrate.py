#!/usr/bin/env python3
"""校准指定 RH56FTP 的力传感器；执行前须张开手掌，所有手指无接触、无外力。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent / "python"))
from pendant.handlink import FINGERS, HAND_PORT, HandLink


def check_state(state: dict) -> None:
    """检查张开、静止和无故障；是否接触物体仍须由操作者确认。"""
    for field in ("angle", "current", "err", "force"):
        values = state.get(field)
        if (not isinstance(values, list) or len(values) != 6 or
                any(type(v) is not int for v in values)):
            raise ValueError(f"{field} 反馈格式无效")
    if any(not 0 <= v <= 1000 for v in state["angle"]):
        raise ValueError("位置反馈超出 0..1000")
    # Backend order: little/ring/middle/index/thumb bend/thumb rotation.
    # 950 is a conservative script precheck, not a manufacturer's calibration threshold.
    if any(v < 950 for v in state["angle"][:5]):
        raise ValueError("五个弯曲通道须充分张开（位置 >= 950），请张开手掌后重试")
    if any(state["err"]):
        raise ValueError(f"设备仍有故障，故障码={state['err']}")
    if any(state["current"]):
        raise ValueError(f"电机电流尚未归零，请等待手指停止；电流={state['current']}")


def show_state(label: str, state: dict) -> None:
    print(label)
    print("通道：" + " / ".join(FINGERS))
    for field, name in (("angle", "位置"), ("force", "受力"), ("current", "电流"), ("err", "故障")):
        print(f"{name}：" + " / ".join(str(v) for v in state[field]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="待校准灵巧手的 IP")
    parser.add_argument("--port", type=int, default=HAND_PORT, help="Modbus TCP 端口（默认 6000）")
    parser.add_argument("--timeout", type=float, default=1.0, help="单次请求超时秒数（默认 1）")
    parser.add_argument("--observe-seconds", type=float, default=10.0,
                        help="校准后观察反馈的秒数（默认 10；不是厂家规定的校准耗时）")
    args = parser.parse_args(argv)
    if not args.host.strip():
        parser.error("--host 不能为空")
    if not 1 <= args.port <= 65535:
        parser.error("--port 必须在 1..65535")
    for name in ("timeout", "observe_seconds"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} 必须为有限正数")

    hand = HandLink(args.host, args.port, args.timeout)
    attempted = acknowledged = False
    try:
        print(f"校准设备：{args.host}:{args.port}")
        print("请先停止 Core、手节点和其他控制端，保持手掌张开、所有手指无接触和无外力。")
        hand.connect()
        before = hand.read_state()
        check_state(before)
        show_state("校准前（原始寄存器刻度）：", before)
        answer = input("确认上述条件均满足后输入 CALIBRATE，其他输入取消：")
        if answer.strip() != "CALIBRATE":
            print("已取消，未发送校准指令。")
            return 1
        # The operator may have taken time to prepare the hand; recheck immediately before writing.
        check_state(hand.read_state())
        attempted = True
        hand.force_calibrate()
        acknowledged = True
        print("已向地址 1009 写入 1，Modbus 返回成功；校准指令仅发送一次。", flush=True)
        samples = []
        deadline = time.monotonic() + args.observe_seconds
        while time.monotonic() < deadline:
            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
            state = hand.read_state()
            check_state(state)
            samples.append(state)
        # A very short configured interval may elapse before the loop starts.
        if not samples:
            state = hand.read_state()
            check_state(state)
            samples.append(state)
        show_state("校准后：", samples[-1])
        tail = samples[-5:]
        ranges = [f"{min(s['force'][i] for s in tail)}..{max(s['force'][i] for s in tail)}"
                  for i in range(6)]
        print(f"最后 {len(tail)} 次空载受力范围：" + " / ".join(ranges))
        print("请确认空载受力接近零且稳定。手册未规定完成标志或断电保存方式；断电保留情况需另行验证。")
        return 0
    except (EOFError, KeyboardInterrupt):
        print("校准指令已写入，反馈观察中断。" if acknowledged else
              "校准请求可能已发送，请检查反馈。" if attempted else "已取消，未发送校准指令。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"操作失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        if acknowledged:
            print("校准指令已写入，但反馈检查未完成，请检查设备状态。", file=sys.stderr)
        elif attempted:
            print("设备是否收到校准请求尚不确定，请检查反馈；本次不会自动重试。", file=sys.stderr)
        return 1
    finally:
        hand.close()


if __name__ == "__main__":
    raise SystemExit(main())
