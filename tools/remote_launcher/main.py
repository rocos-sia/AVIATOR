#!/usr/bin/env python3
"""Portable Ubuntu desktop launcher for an already deployed AVIATOR controller."""
import argparse
from dataclasses import asdict
import fcntl
import json
from pathlib import Path
import queue
import sys
import webbrowser

from backend import Controller, Options, STATE_DIR, URL, write_json


def main():
    parser = argparse.ArgumentParser(description="AVIATOR 本机远程启动界面（打开界面不会启动设备）")
    parser.parse_args()
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
    except ImportError:
        print("缺少 Tk，请运行：sudo apt-get install python3-tk openssh-client rsync", file=sys.stderr)
        return 1
    STATE_DIR.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock = (STATE_DIR / "ui.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("远程启动界面已经打开。", file=sys.stderr)
        return 1
    connection = json.loads(Path(__file__).with_name("connection.json").read_text())
    options = Options(target=connection["target"], prefix=connection["prefix"], keyboard=True)
    settings = STATE_DIR / "settings.json"
    if settings.exists():
        try:
            data = json.loads(settings.read_text())
            # Always default to real hands. Prior tests must not silently change the next run.
            options = Options(**{"keyboard": True, **data, "target": connection["target"],
                                 "prefix": connection["prefix"], "fake_hand": False})
        except (ValueError, TypeError):
            pass
    controller = Controller()
    root = tk.Tk()
    root.title("AVIATOR · 远程启动")
    root.geometry("1040x860")
    root.minsize(920, 720)
    from appearance import ACCENT, CARD, MUTED, configure
    configure(root)
    panel = ttk.Frame(root, padding=26)
    panel.pack(fill="both", expand=True)
    panel.columnconfigure(0, weight=1)
    panel.rowconfigure(4, weight=1)
    header = ttk.Frame(panel)
    header.grid(row=0, sticky="ew", pady=(0, 22))
    header.columnconfigure(0, weight=1)
    ttk.Label(header, text="AVIATOR", style="Brand.TLabel").grid(row=0, column=0, sticky="w")
    ttk.Label(header, text="SSH 自动认证", style="Badge.TLabel").grid(row=0, column=1, sticky="e")
    ttk.Label(header, text=f"工控机远程控制   ·   {options.target}   /   {options.prefix}", style="Muted.TLabel").grid(row=1, columnspan=2, sticky="w", pady=(4, 0))

    cards = ttk.Frame(panel)
    cards.grid(row=1, sticky="ew")
    cards.columnconfigure(0, weight=1, uniform="card")
    cards.columnconfigure(1, weight=1, uniform="card")
    controls = ttk.Frame(cards, style="Card.TFrame", padding=20)
    controls.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
    controls.columnconfigure(0, weight=1)
    controls.columnconfigure(1, weight=1)
    ttk.Label(controls, text="运行设置", style="Heading.TLabel").grid(row=0, column=0, sticky="w")
    hand_mode = ttk.Label(controls, text="真实双手", style="Badge.TLabel")
    hand_mode.grid(row=0, column=1, sticky="e")
    ttk.Label(controls, text="启动前请确认现场具备运行条件", style="Note.TLabel").grid(row=1, columnspan=2, sticky="w", pady=(5, 14))
    variables = {name: tk.BooleanVar(value=getattr(options, name))
                 for name in ("rs422", "fake_hand", "tts", "logger", "transfer", "keyboard")}
    widgets = []
    checks = {}
    option_cells = []
    for index, (name, label, note) in enumerate((
            ("keyboard", "本机键盘控制", "点击网关输入窗口后操作"),
            ("rs422", "RS422 输入", "外部串口 · --rs422"),
            ("fake_hand", "模拟双手", "仅替换双手 · --fake-hand"),
            ("tts", "语音提醒", "工控机播放 · --tts"))):
        cell = ttk.Frame(controls, style="Card.TFrame")
        option_cells.append(cell)
        cell.grid(row=2+index//2, column=index%2, sticky="nw", padx=(0, 10), pady=(0, 13))
        widget = ttk.Checkbutton(cell, text=label, variable=variables[name], style="Aviator.TCheckbutton")
        widget.pack(anchor="w")
        ttk.Label(cell, text=note, style="Note.TLabel").pack(anchor="w", padx=(28, 0))
        widgets.append(widget)
        checks[name] = widget
    keyboard = checks["keyboard"]
    ttk.Separator(controls).grid(row=4, columnspan=2, sticky="ew", pady=(0, 10))
    ttk.Label(controls, text="键盘与 RS422 互斥  ·  默认使用真实机械臂", style="Note.TLabel").grid(row=5, columnspan=2, sticky="w")

    data = ttk.Frame(cards, style="Card.TFrame", padding=20)
    data.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
    data.columnconfigure(0, weight=1)
    ttk.Label(data, text="数据记录", style="Heading.TLabel").grid(row=0, sticky="w")
    ttk.Label(data, text="按次保存，保留工控机上的原始数据", style="Note.TLabel").grid(row=1, sticky="w", pady=(5, 14))
    recording = ttk.Checkbutton(data, text="记录数据  ·  --logger", variable=variables["logger"], style="Aviator.TCheckbutton")
    recording.grid(row=2, sticky="w")
    transfer = ttk.Checkbutton(data, text="同时回传到本机", variable=variables["transfer"], style="Aviator.TCheckbutton")
    transfer.grid(row=3, sticky="w", pady=(3, 10))
    widgets.extend([recording, transfer])
    destination = tk.StringVar(value=options.destination)
    ttk.Label(data, text="本机保存目录", style="Note.TLabel").grid(row=4, sticky="w", pady=(0, 5))
    save_row = ttk.Frame(data, style="Card.TFrame")
    save_row.grid(row=5, sticky="ew")
    save_row.columnconfigure(0, weight=1)
    entry = ttk.Entry(save_row, textvariable=destination, width=24)
    entry.grid(row=0, column=0, sticky="ew")

    def browse():
        path = filedialog.askdirectory(initialdir=str(Path(destination.get()).expanduser()))
        if path:
            destination.set(path)

    choose = ttk.Button(save_row, text="选择…", command=browse)
    choose.grid(row=0, column=1, padx=(8, 0))
    widgets.extend([entry, choose])
    preview = tk.StringVar()
    preview_label = ttk.Label(panel, textvariable=preview, style="Muted.TLabel", wraplength=930)
    preview_label.grid(row=2, sticky="ew", pady=(12, 14))
    actions = ttk.Frame(panel, style="Card.TFrame", padding=16)
    actions.grid(row=3, sticky="ew", pady=(0, 16))
    actions.columnconfigure(0, weight=1)
    status_row = ttk.Frame(actions, style="Card.TFrame")
    status_row.grid(row=0, column=0, sticky="w")
    status_dot = ttk.Label(status_row, text="●", style="Card.TLabel", foreground=MUTED)
    status_dot.pack(side="left", padx=(0, 8))
    state = tk.StringVar(value="待启动" if not controller.session_file.exists() else "存在待收尾的会话")
    ttk.Label(status_row, textvariable=state, style="Status.TLabel", wraplength=250).pack(side="left")
    buttons = ttk.Frame(actions, style="Card.TFrame")
    buttons.grid(row=0, column=1, sticky="e")
    closing = [False]

    def get_options():
        return Options(target=connection["target"], prefix=connection["prefix"],
                       destination=destination.get(), **{k: v.get() for k, v in variables.items()})

    def refresh(*_):
        if not variables["logger"].get():
            variables["transfer"].set(False)
        if variables["rs422"].get():
            variables["keyboard"].set(False)
        if not controller.busy:
            transfer.configure(state="normal" if variables["logger"].get() else "disabled")
            keyboard.configure(state="disabled" if variables["rs422"].get() else "normal")
        hand_mode.configure(text="模拟双手" if variables["fake_hand"].get() else "真实双手",
                            foreground="#996328" if variables["fake_hand"].get() else "#246a59",
                            background="#fbefdd" if variables["fake_hand"].get() else "#e1efeb")
        preview.set("启动参数：" + (" ".join(get_options().flags()) or "默认真机模式（真实手）") +
                    ("\n键盘操作请聚焦 flight_gateway 输入窗口，Monitor 浏览器不接收动作按键。" if variables["keyboard"].get()
                     else "\nMonitor 就绪后自动打开；停止时按原顺序收尾并完成数据写盘。"))

    def start(recover=False):
        try:
            selected = get_options()
            write_json(settings, asdict(selected))
            controller.start(selected, connection["password"], recover=recover)
            start_button.configure(state="disabled")
            recover_button.configure(state="disabled")
            stop_button.configure(state="normal")
            for widget in widgets:
                widget.configure(state="disabled")
        except Exception as exc:
            messagebox.showerror("无法启动", str(exc), parent=root)

    start_button = ttk.Button(buttons, text="启动工控机", command=start, style="Primary.TButton")
    start_button.pack(side="left", padx=(0, 8))
    stop_button = ttk.Button(buttons, text="停止并收尾", command=controller.stop, state="disabled", style="Stop.TButton")
    stop_button.pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="打开 Monitor", command=lambda: webbrowser.open(URL)).pack(side="left", padx=(0, 8))
    recover_button = ttk.Button(buttons, text="重连 / 收尾补传", command=lambda: start(True))
    recover_button.pack(side="left")
    sync_state = tk.StringVar(value="启动后在此显示连接与数据回传进度。")
    ttk.Label(actions, textvariable=sync_state, style="Note.TLabel", wraplength=900).grid(row=1, columnspan=2, sticky="w", pady=(12, 0))
    log_card = ttk.Frame(panel, style="Card.TFrame", padding=16)
    log_card.grid(row=4, sticky="nsew")
    log_card.columnconfigure(0, weight=1)
    log_card.rowconfigure(1, weight=1)
    ttk.Label(log_card, text="运行日志", style="Heading.TLabel").grid(row=0, sticky="w", pady=(0, 12))
    log_body = ttk.Frame(log_card, style="Card.TFrame")
    log_body.grid(row=1, sticky="nsew")
    log_body.columnconfigure(0, weight=1)
    log_body.rowconfigure(0, weight=1)
    logs = tk.Text(log_body, width=1, height=1, state="disabled", font=("monospace", 10),
                   background="#142735", foreground="#d6e5ef", insertbackground="white",
                   selectbackground="#355c70", relief="flat", borderwidth=0, highlightthickness=0,
                   padx=14, pady=12, wrap="word")
    logs.grid(row=0, column=0, sticky="nsew")
    scrollbar = ttk.Scrollbar(log_body, command=logs.yview, style="Log.Vertical.TScrollbar")
    scrollbar.grid(row=0, column=1, sticky="ns")
    logs.configure(yscrollcommand=scrollbar.set)
    logs.configure(state="normal")
    logs.insert("end", "等待启动。连接、节点启动和数据回传日志将在这里显示。\n")
    logs.configure(state="disabled")
    empty_log = [True]
    ttk.Label(panel, text="MONITOR   127.0.0.1:8081     ·     就绪后自动在本机浏览器打开", style="Muted.TLabel").grid(row=5, sticky="w", pady=(12, 0))

    def update_status(text):
        state.set(text)
        color = ACCENT if text == "运行中" else "#b17827"
        if text in ("待启动", "已停止"):
            color = MUTED
        elif "失败" in text or "需要重连" in text or "退出码" in text:
            color = "#b55349"
        status_dot.configure(foreground=color)

    def poll():
        try:
            while True:
                event = controller.events.get_nowait()
                kind = event["event"]
                if kind == "log":
                    logs.configure(state="normal")
                    if empty_log[0]:
                        logs.delete("1.0", "end")
                        empty_log[0] = False
                    logs.insert("end", event["text"])
                    if int(logs.index("end-1c").split(".")[0]) > 3000:
                        logs.delete("1.0", "1000.0")
                    logs.see("end")
                    logs.configure(state="disabled")
                elif kind == "state":
                    update_status(event["text"])
                elif kind == "transfer":
                    sync_state.set(event["text"])
                elif kind == "browser" and not closing[0]:
                    webbrowser.open(URL)
                elif kind == "done":
                    start_button.configure(state="normal")
                    recover_button.configure(state="normal")
                    stop_button.configure(state="disabled")
                    for widget in widgets:
                        widget.configure(state="normal")
                    refresh()
                elif kind == "error":
                    closing[0] = False
                    messagebox.showerror("远程运行", event["text"], parent=root)
        except queue.Empty:
            pass
        if closing[0] and not controller.busy:
            root.destroy()
            return
        root.after(100, poll)

    def close():
        controller.close()
        if controller.busy:
            closing[0] = True
            update_status("等待设备停止并写盘后关闭；数据回传可稍后继续")
        else:
            root.destroy()

    compact_mode = [None]

    def resize(event):
        if event.widget is not root:
            return
        preview_label.configure(wraplength=max(700, event.width-80))
        compact = event.height < 800 or event.width < 1000
        if compact == compact_mode[0]:
            return
        compact_mode[0] = compact
        panel.configure(padding=18 if compact else 26)
        header.grid_configure(pady=(0, 14 if compact else 22))
        for card in (controls, data):
            card.configure(padding=14 if compact else 20)
        for cell in option_cells:
            cell.grid_configure(pady=(0, 8 if compact else 13))
        actions.configure(padding=12 if compact else 16)
        log_card.configure(padding=12 if compact else 16)
        preview_label.grid_configure(pady=(8, 10) if compact else (12, 14))
        logs.configure(pady=8 if compact else 12)

    for key in variables:
        variables[key].trace_add("write", refresh)
    refresh()
    root.bind("<Configure>", resize)
    root.protocol("WM_DELETE_WINDOW", close)
    root.after(100, poll)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
