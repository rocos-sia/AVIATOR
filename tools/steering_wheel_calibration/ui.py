#!/usr/bin/env python3
"""一键启动 Bus、Camera 和方向盘两点标定窗口（只采集相机数据）。"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system-config", type=Path, default=ROOT / "config/system.yaml")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/camera.yaml")
    parser.add_argument("--recording-config", type=Path, default=ROOT / "config/recording.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "config/steering_wheel_calibration.yaml")
    parser.add_argument("--bus-bin", type=Path,
                        default=Path(os.environ.get("AVIATOR_BIN", str(ROOT / "build/bin"))) / "aviator_bus")
    parser.add_argument("--camera-python", type=Path,
                        default=Path(os.environ.get("CAMERA_PYTHON", str(Path.home() / "miniconda3/envs/apriltag_realsense/bin/python"))))
    parser.add_argument("--camera-id", default="cockpit")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--sample-interval-ms", type=int, default=20)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--stale-ms", type=int, default=500, help="数据有效期，默认 500 ms")
    parser.add_argument("--capture-timeout-s", type=float, default=15.0)
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=True,
                        help="相机独立视频预览窗口，默认开启")
    parser.add_argument("--dry-run", action="store_true", help="只打印命令，不打开窗口、不启动节点、不写文件")
    args = parser.parse_args(argv)
    if (args.samples <= 0 or args.sample_interval_ms < 0 or args.stale_ms <= 0
            or not 0 < args.capture_timeout_s < float("inf")
            or not 0 <= args.min_confidence <= 1):
        parser.error("采样数、数据有效期和超时必须为正；间隔不能为负；置信度必须在 [0,1]")
    for name in ("system_config", "camera_config", "recording_config", "output", "bus_bin", "camera_python"):
        setattr(args, name, getattr(args, name).expanduser().absolute())
    args.topic = "camera.detection"
    args.frame_id = "camera_color_optical_frame"
    return args


class Window:
    def __init__(self, root, args, worker):
        from tkinter import StringVar, font, ttk
        self.root, self.worker, self.closing = root, worker, False
        root.title("AVIATOR · 方向盘两点标定")
        root.geometry("980x860")
        root.minsize(940, 820)
        style = ttk.Style(root)
        style.theme_use("clam")
        family = "Noto Sans CJK SC" if "Noto Sans CJK SC" in font.families(root) else "sans-serif"
        font.nametofont("TkDefaultFont").configure(family=family, size=10)
        style.configure(".", background="#f5f7fa", foreground="#203247")
        style.configure("Title.TLabel", font=(family, 21, "bold"))
        style.configure("Value.TLabel", font=("DejaVu Sans", 34, "bold"), foreground="#175a91")
        style.configure("TButton", padding=(12, 9))
        footer = ttk.Frame(root, padding=(24, 0, 24, 18))
        footer.pack(side="bottom", fill="x")
        self.stop = ttk.Button(footer, text="停止相机和 Bus", command=self.stop_session)
        self.stop.pack(side="right")
        panel = ttk.Frame(root, padding=24)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="方向盘两点标定", style="Title.TLabel").pack(anchor="w")
        ttk.Label(panel, text=f"相机 {args.camera_id}  ·  AprilTag {args.tag_id}  ·  每点 {args.samples} 帧平均").pack(anchor="w", pady=(6, 18))
        self.tracking = StringVar(value="正在启动 Bus 和相机…")
        ttk.Label(panel, textvariable=self.tracking).pack(anchor="w")
        steps = ttk.LabelFrame(panel, text=" 采集标定点 ", padding=14)
        steps.pack(fill="x", pady=14)
        ttk.Label(steps, text="① 将方向盘放在零位并保持静止。② 转动到明显不同的角度，保持静止后采集。\n第二点采集完成后自动备份并保存 YAML；重新采集第一点会重新开始本轮标定。").pack(anchor="w")
        buttons = ttk.Frame(steps)
        buttons.pack(fill="x", pady=(12, 8))
        self.first = ttk.Button(buttons, text="采集第一个点（零位）", command=lambda: worker.actions.put(1))
        self.first.pack(side="left", padx=(0, 12))
        self.second = ttk.Button(buttons, text="采集第二个点并保存", command=lambda: worker.actions.put(2))
        self.second.pack(side="left")
        self.cancel = ttk.Button(buttons, text="取消采样", command=lambda: worker.actions.put("cancel"))
        self.cancel.pack(side="right")
        self.points = StringVar(value="第一点：未采集    |    第二点：未采集")
        ttk.Label(steps, textvariable=self.points).pack(anchor="w")
        self.progress = ttk.Progressbar(steps, maximum=args.samples)
        self.progress.pack(fill="x", pady=(10, 0))
        readout = ttk.LabelFrame(panel, text=" 当前已保存标定 · 相对零位的实时读数 ", padding=14)
        readout.pack(fill="x")
        self.angle, self.depth = StringVar(value="— °"), StringVar(value="— mm")
        for column, (title, variable) in enumerate((("方向盘角度", self.angle), ("纵深（沿标定轴）", self.depth))):
            readout.columnconfigure(column, weight=1)
            ttk.Label(readout, text=title).grid(row=0, column=column, sticky="w")
            ttk.Label(readout, textvariable=variable, style="Value.TLabel").grid(row=1, column=column, sticky="w", pady=8)
        self.quality = StringVar(value="等待有效标定和检测")
        ttk.Label(readout, textvariable=self.quality).grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Label(panel, text="纵深是相对零位的有符号轴向位移，不是标签到相机的距离。\n正负方向沿用标定轴约定；没有有效检测时不显示旧读数。").pack(anchor="w", pady=12)
        self.message = StringVar()
        ttk.Label(panel, textvariable=self.message, wraplength=850).pack(anchor="w", fill="x")
        self.logs = StringVar()
        ttk.Label(panel, textvariable=self.logs, wraplength=850).pack(anchor="w", pady=8)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()

    def stop_session(self):
        self.worker.stop_requested.set()
        self.stop.configure(state="disabled")

    def close(self):
        self.closing = True
        self.stop_session()

    def refresh(self):
        state = self.worker.snapshot()
        active = state.get("running") and not self.worker.stop_requested.is_set()
        ready = active and state.get("fresh") and not state.get("pending")
        self.first.configure(state="normal" if ready else "disabled")
        self.second.configure(state="normal" if ready and state.get("has_zero") else "disabled")
        self.cancel.configure(state="normal" if active and state.get("pending") else "disabled")
        self.progress["value"] = state.get("count", 0)
        self.points.set(f"第一点：{state.get('first', '未采集')}    |    第二点：{state.get('second', '未采集')}")
        fresh = active and state.get("fresh")
        self.tracking.set(f"TRACKING · 置信度 {state['confidence']:.2f}" if fresh else "无有效检测 / 已停止 · 请检查相机和标签")
        angle, depth = state.get("theta_deg"), state.get("depth_mm")
        self.angle.set(f"{angle:+.2f} °" if active and angle is not None else "— °")
        self.depth.set(f"{depth:+.2f} mm" if active and depth is not None else "— mm")
        if angle is None or not active:
            reason = {"calibration_unavailable": "尚无有效标定，请采集两个点",
                      "calibration_source_mismatch": "已有标定的相机 / 标签不匹配，请重新标定"}.get(state.get("reason"), "等待有效检测和标定")
            self.quality.set(reason)
        else:
            match = state.get("axis_match")
            text = "轴方向一致" if match is True else "旋转较小，暂不验证轴方向" if match is None else "轴方向偏差超过 5°，请检查标定 / 标签固定情况"
            self.quality.set(text + f"  ·  标定 {state.get('calibration_id', '')}")
        self.message.set(state.get("message", ""))
        self.logs.set("运行日志：" + state.get("log_dir", "待创建"))
        if self.closing and not self.worker.is_alive():
            self.root.destroy()
            return
        self.root.after(100, self.refresh)


def main(argv=None):
    args = parse_args(argv)
    from tools.steering_wheel_calibration.ui_backend import Worker, commands
    if args.dry_run:
        plans, endpoint = commands(args, "<本次独立 session UUID>")
        for name, command in plans:
            print(f"启动 {name}: {shlex.join(command)}")
        print(f"标定 UI：{endpoint} → {args.output}")
        return 0
    import tkinter as tk
    root = tk.Tk()
    worker = Worker(args, str(uuid.uuid4()))
    window = Window(root, args, worker)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    worker.start()
    try:
        root.mainloop()
    finally:
        worker.stop_requested.set()
        worker.join()
    return 1 if worker.snapshot().get("error") else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"方向盘标定 UI：{exc}", file=sys.stderr)
        raise SystemExit(1)
