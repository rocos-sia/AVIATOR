#!/usr/bin/env python3
"""Desktop home-position teaching with one persistent SDK connection per arm."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import signal
import sys
import tempfile
import threading

from get_home import ROOT, home_angles, read_posture, save_home, state_helper

sys.path.insert(0, str(ROOT / "tools/grasp_tool_calibration"))
from native_session import NativeSession


class HomeController:
    """Called on one worker; cancel() may interrupt a pending SDK request."""
    def __init__(self, robot_config, dry_run=False, session_factory=NativeSession):
        self.robot_config = robot_config
        self.dry_run = dry_run
        self.session_factory = session_factory
        self.session = None
        self.cancelled = threading.Event()
        self.status = {}
        self.angles = None
        self.log_dir = None
        self.backup = None
        self.cleanup_error = None
        self._load_config()

    def _load_config(self):
        self.robot_original = self.robot_config.read_bytes()
        self.posture, self.original, self.saved = read_posture(self.robot_config)

    def _check_config(self):
        if self.robot_config.read_bytes() != self.robot_original or self.posture.read_bytes() != self.original:
            raise RuntimeError("配置在会话期间已被其他程序修改，请断开后重新连接。")

    def snapshot(self):
        return dict(status=self.status, angles=self.angles, saved=self.saved,
                    posture=str(self.posture), backup=str(self.backup) if self.backup else None,
                    log_dir=str(self.log_dir) if self.log_dir else None)

    def perform(self, action):
        try:
            if self.cancelled.is_set():
                raise RuntimeError("正在关闭界面")
            if action == "disconnect":
                self.close()
            elif action == "connect":
                if self.session is not None:
                    raise RuntimeError("双臂会话已存在")
                self._load_config()
                self.cleanup_error = None
                helper = state_helper("aviator_grasp_tool_session")
                self.log_dir = Path(tempfile.mkdtemp(prefix="aviator-home-ui-"))
                self.session = self.session_factory(helper, self.robot_config, self.dry_run,
                                                    self.log_dir, lambda _: None)
                if self.cancelled.is_set():
                    raise RuntimeError("正在关闭界面")
                self.status = self.session.request("connect")
                self.angles = home_angles(self.session.request("sample"), demo=self.dry_run)
            else:
                if self.session is None:
                    raise RuntimeError("请先连接双臂")
                self._check_config()
                if action == "start":
                    self.angles = None
                    self.status = self.session.request("drag_start", side="both")
                elif action in ("stop", "record"):
                    if action == "record" and self.dry_run:
                        raise RuntimeError("离线演示禁止写入配置")
                    # Stop BOTH arms and obtain a fresh pair; never write a UI preview.
                    self.status = self.session.request("drag_stop", side="both")
                    sample = self.session.request("sample")
                    self.angles = home_angles(sample, demo=self.dry_run)
                    if action == "record":
                        if self.cancelled.is_set():
                            raise RuntimeError("记录已取消")
                        self._check_config()
                        self.backup, _ = save_home(self.posture, self.original, sample)
                        self.original = self.posture.read_bytes()
                        _, _, self.saved = read_posture(self.robot_config)
                elif action == "status":
                    self.status = self.session.request("status", timeout=10)
                    if not all(self.status.get(side, {}).get("connected") for side in ("left", "right")):
                        raise RuntimeError("机械臂连接已中断")
                else:
                    raise ValueError(f"未知操作: {action}")
            return self.snapshot()
        except Exception as error:
            # Rollback includes partial connects/drag starts, not only success paths.
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(f"{error}\n清理失败: {cleanup}") from error
            raise

    def cancel(self):
        self.cancelled.set()
        if self.session is not None:
            self.session.interrupt()

    def close(self):
        try:
            if self.session is not None:
                self.session.close()  # EOF disables owned drag before disconnecting.
            elif self.cleanup_error is not None:
                raise RuntimeError(self.cleanup_error)
        except Exception as error:
            self.cleanup_error = str(error)
            raise
        finally:
            self.session = None
            self.status = {}
            self.angles = None


class HomeWindow:
    def __init__(self, root, controller):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.controller = root, controller
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="home-sdk")
        self.future = None
        self.closing = False
        self.action = None
        self.view = controller.snapshot()
        root.title("AVIATOR · 归位角记录" + (" · 离线演示" if controller.dry_run else ""))
        root.geometry("960x720")
        root.minsize(860, 680)
        root.configure(bg="#edf2f7")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TFrame", background="#edf2f7")
        style.configure("TLabel", background="#edf2f7", foreground="#243449", font=("sans", 11))
        style.configure("Title.TLabel", font=("sans", 23, "bold"))
        style.configure("State.TLabel", font=("sans", 14, "bold"))
        style.configure("TButton", font=("sans", 11), padding=(12, 10))
        style.configure("Record.TButton", background="#1e6b55", foreground="white")
        style.map("Record.TButton", background=[("disabled", "#c4d1cb"), ("active", "#285e4e")])
        style.configure("Treeview", font=("sans", 11), rowheight=31,
                        background="white", fieldbackground="white", foreground="#243449")
        style.configure("Treeview.Heading", font=("sans", 11, "bold"), padding=8)

        panel = ttk.Frame(root, padding=24)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="双臂归位角记录", style="Title.TLabel").pack(anchor="w")
        mode = "离线演示 · 不连接设备，不写入配置" if controller.dry_run else "连接双臂 → 开启拖动 → 调整姿态 → 记录归位角"
        ttk.Label(panel, text=mode).pack(anchor="w", pady=(4, 12))
        ttk.Label(panel, text="开启拖动后，按住各机械臂末端按钮调整姿态。记录时自动停止双臂拖动。\n"
                  "使用前退出其他控制程序；关闭窗口会停止本次拖动并断开连接。",
                  wraplength=870).pack(anchor="w", pady=(0, 14))
        states = ttk.Frame(panel)
        states.pack(fill="x")
        self.arm_states = {}
        for side, title in (("left", "左臂"), ("right", "右臂")):
            var = tk.StringVar(value=f"{title} · 未连接")
            self.arm_states[side] = var
            ttk.Label(states, textvariable=var, style="State.TLabel").pack(side="left", expand=True, anchor="w")
        ttk.Label(panel, text="关节角（°） · 实测列为最近一次停止拖动后的采样").pack(anchor="w", pady=(14, 6))
        self.table = ttk.Treeview(panel, columns=("axis", "left_saved", "left_actual", "right_saved", "right_actual"),
                                  show="headings", height=7, selectmode="none")
        for column, title in zip(self.table["columns"], ("关节", "左臂 · 已保存", "左臂 · 实测", "右臂 · 已保存", "右臂 · 实测")):
            self.table.heading(column, text=title)
            self.table.column(column, width=70 if column == "axis" else 175, anchor="center", stretch=True)
        self.table.pack(fill="x")
        for axis in range(7):
            self.table.insert("", "end", iid=str(axis), values=(f"J{axis + 1}", "—", "—", "—", "—"))
        toolbar = ttk.Frame(panel)
        toolbar.pack(fill="x", pady=(16, 12))
        self.buttons = {}
        for action, label in (("connect", "连接双臂"), ("start", "开启双臂拖动"), ("stop", "停止拖动"),
                              ("record", "记录到配置"), ("disconnect", "断开连接")):
            button = ttk.Button(toolbar, text=label, command=lambda a=action: self.submit(a),
                                style="Record.TButton" if action == "record" else "TButton")
            button.pack(side="left", padx=(0, 8))
            self.buttons[action] = button
        self.path_label = ttk.Label(panel, text="", wraplength=875)
        self.path_label.pack(anchor="w", pady=(0, 8))
        self.message = tk.StringVar(value="准备就绪。点击“连接双臂”后才会连接设备。")
        self.message_label = ttk.Label(panel, textvariable=self.message, wraplength=875, justify="left")
        self.message_label.pack(anchor="w", fill="x")
        root.protocol("WM_DELETE_WINDOW", self.request_close)
        self.render()
        root.after(100, self.tick)
        root.after(1500, self.poll_status)

    def render(self):
        status = self.view["status"]
        connected = all(status.get(side, {}).get("connected") for side in ("left", "right"))
        dragging = any(status.get(side, {}).get("dragging") for side in ("left", "right"))
        labels = {"idle": "已连接 · 静止", "jog": "已连接 · 静止", "drag": "拖动已开启 · 按住末端按钮"}
        for side, title in (("left", "左臂"), ("right", "右臂")):
            arm = status.get(side, {})
            state = labels.get(arm.get("operation_state"), "未连接" if not arm.get("connected") else "状态异常")
            self.arm_states[side].set(f"{title} · {state}")
        for axis in range(7):
            values = [f"J{axis + 1}"]
            for side in ("left", "right"):
                key = side + "_home_deg"
                saved = self.view["saved"].get(key, [])
                values.append(f"{saved[axis]:.4f}" if len(saved) == 7 else "—")
                actual = self.view["angles"]
                values.append(f"{actual[key][axis]:.4f}" if actual and not dragging else "—")
            self.table.item(str(axis), values=values)
        enabled = dict(connect=not connected, start=connected and not dragging,
                       stop=connected and dragging, record=connected and not self.controller.dry_run,
                       disconnect=connected)
        for action, button in self.buttons.items():
            button.configure(state="normal" if enabled[action] and self.future is None and not self.closing else "disabled")
        self.path_label.configure(text=f"配置文件：{self.view['posture']}")

    def submit(self, action):
        if self.future is not None or self.closing:
            return
        self.action = action
        if action != "status":
            self.message_label.configure(foreground="#243449")
            self.message.set({"connect": "正在连接双臂…", "start": "正在开启双臂拖动…",
                              "stop": "正在停止双臂拖动并读取关节角…", "record": "正在停止拖动、读取双臂并写入配置…",
                              "disconnect": "正在结束拖动并断开连接…"}[action])
        self.future = self.pool.submit(self.controller.perform, action)
        self.render()

    def tick(self):
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                result = future.result()
                self.view = result or self.controller.snapshot()
                if not self.closing and self.action != "status":
                    connected_message = ("演示双臂已连接。可体验开启/停止拖动；演示数据不能写入配置。"
                                         if self.controller.dry_run else "双臂已连接。可以开启拖动，或直接记录当前姿态。")
                    message = {"connect": connected_message,
                               "start": "拖动已开启。请按住末端按钮调整双臂姿态。",
                               "stop": "双臂拖动已停止，实测角已刷新。",
                               "record": f"归位角已保存，重启控制程序后生效。\n备份：{self.view['backup']}",
                               "disconnect": "双臂已断开连接。"}[self.action]
                    self.message.set(message)
            except Exception as error:
                self.view = self.controller.snapshot()
                self.message_label.configure(foreground="#b33434")
                self.message.set(f"操作失败：{error}\n日志：{self.view['log_dir'] or '尚未启动机械臂会话'}")
                if self.closing:
                    # Keep cleanup failures visible instead of silently closing the window.
                    self.closing = False
                    self.controller.cancelled.clear()
                    self.controller.cleanup_error = None
                    self.render()
                    self.root.after(100, self.tick)
                    return
            if self.closing:
                self.pool.shutdown(wait=False)
                self.root.destroy()
                return
            self.render()
        self.root.after(100, self.tick)

    def poll_status(self):
        if self.view["status"] and not self.closing:
            self.submit("status")
        self.root.after(1500, self.poll_status)

    def request_close(self):
        if self.closing:
            return
        self.closing = True
        self.message.set("正在停止本次拖动并断开双臂，请稍候…")
        self.controller.cancel()
        # The single worker finishes/cancels its current operation before cleanup.
        self.future = self.pool.submit(self.controller.close)
        self.render()


def run(robot_config, dry_run=False):
    import tkinter as tk

    controller = HomeController(robot_config, dry_run)
    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise RuntimeError(f"无法打开桌面界面，请在图形会话中运行：{error}") from error
    window = HomeWindow(root, controller)
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: window.request_close())
    try:
        root.mainloop()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        controller.cancel()
        window.pool.shutdown(wait=True)
        controller.close()


if __name__ == "__main__":
    from get_home import main
    sys.exit(main(["--ui", *sys.argv[1:]]))
