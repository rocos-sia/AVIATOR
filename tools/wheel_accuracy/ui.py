#!/usr/bin/env python3
"""Manual wheel accuracy panel; the existing gateway/Core own all motion."""
import argparse
import json
import math
import socket
import time
import uuid

DEFAULT_POINTS = ((0, 0), (-25, -42.5), (25, -42.5), (-25, -127.5), (25, -127.5))
ANGLE_LIMIT = math.degrees(.87266)


def target(angle_deg, displacement_mm):
    angle, displacement = float(angle_deg), float(displacement_mm)
    if not (math.isfinite(angle) and math.isfinite(displacement)):
        raise ValueError("请输入有限数字")
    if not -ANGLE_LIMIT <= angle <= ANGLE_LIMIT or not -170 <= displacement <= 0:
        raise ValueError(f"转角须在 ±{ANGLE_LIMIT:.5f}° 内，位移须在 -170～0 mm 内")
    return dict(angle=math.radians(angle), displacement=displacement / 1000)


class Controller:
    """One pending action, no command retries, heartbeat on the UI event loop."""
    def __init__(self, path):
        self.path = path
        self.sock = None
        self.status = {}
        self.pending = None
        self.message = "等待网关连接…"
        self.last_connect = 0.
        self.stable_since = None
        self.selected = None

    @property
    def fresh(self):
        stamp = self.status.get("sample_us", 0)
        age = time.monotonic_ns() // 1000 - stamp
        return self.sock is not None and 0 <= age < 100000 and self.status.get("fresh", False)

    @property
    def state(self):
        return self.status.get("state", {})

    @property
    def phase(self):
        return self.state.get("system", {}).get("state", "WAITING")

    @property
    def ready(self):
        return self.fresh and self.status.get("ready", False)

    def send(self, kind, **fields):
        if self.sock is None:
            raise ValueError("网关未连接")
        packet = dict(kind=kind, sample_us=time.monotonic_ns() // 1000, **fields)
        payload = json.dumps(packet, allow_nan=False).encode()
        if self.sock.send(payload) != len(payload):
            raise OSError("UI packet not sent")

    def command(self, kind, **fields):
        identity = uuid.uuid4().hex
        self.send(kind, id=identity, **fields)
        return identity

    def operation(self, name):
        if not self.ready:
            raise ValueError("等待新鲜 Core 状态与网关就绪")
        if self.pending and name != "exit_control":
            raise ValueError("请等待当前操作完成")
        if self.pending and self.pending["stage"] not in ("tracking",):
            raise ValueError("正在等待操作回复，请稍候")
        identity = self.command("operation", operation=name)
        self.pending = dict(stage="operation", id=identity, operation=name,
                            deadline=time.monotonic() + 180, ack=False)
        self.message = "已提交操作，等待 Core 确认…"

    def move(self, point, label):
        if self.pending or not self.ready:
            raise ValueError("请等待当前操作完成、Core 就绪")
        if self.phase not in ("FOLLOWING", "CONTROL"):
            raise ValueError("请先 Home 并完成抓握")
        if self.phase == "FOLLOWING" and not self.state.get("system", {}).get("settled"):
            raise ValueError("抓握后的刚度切换尚未完成")
        self.selected = (label, point)
        self.stable_since = None
        if self.phase == "FOLLOWING":
            # Continue holding the grasp reference until CONTROL is confirmed;
            # only then send the selected point, avoiding an old joystick target.
            identity = self.command("operation", operation="start_control")
            stage = "start"
        else:
            identity = self.command("target", **point)
            stage = "target"
        self.pending = dict(stage=stage, id=identity, point=point, label=label,
                            deadline=time.monotonic() + 60, ack=False)
        self.message = f"{label}：等待控制确认…"

    def close(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.status = {}
        self.pending = None

    def tick(self):
        now = time.monotonic()
        try:
            if self.sock is None:
                if now - self.last_connect < 1:
                    return
                self.last_connect = now
                candidate = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                try:
                    candidate.connect(self.path)
                    candidate.setblocking(False)
                except OSError:
                    candidate.close()
                    raise
                self.sock = candidate
                self.message = "已连接；等待 Core 初始化，随后点击 Home"
            # If the window/event loop stalls, no independent thread renews the lease.
            self.send("heartbeat")
            for _ in range(64):
                try:
                    packet = self.sock.recv(131072)
                except BlockingIOError:
                    break
                if not packet:
                    raise OSError("网关已断开")
                message = json.loads(packet)
                if message.get("kind") == "status":
                    self.status = message
            self.advance(now)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.close()
            self.message = f"连接中断 / 等待网关：{error}；未自动重发操作"

    def advance(self, now):
        pending = self.pending
        if not pending:
            return
        if self.phase in ("ERROR", "EMERGENCY_STOP") and pending.get("operation") != "reset_error":
            self.message = "执行中止：" + self.state.get("system", {}).get("current_error", self.phase)
            self.pending = None
            return
        if not self.fresh:
            self.message = "Core 状态已过期；操作结果未知，等待反馈后检查状态"
            self.pending = None
            return
        reply = self.status.get("result") or {}
        if reply.get("id") == pending["id"]:
            status = reply.get("status")
            if status in ("REJECTED", "NOT_SENT", "UNKNOWN"):
                self.message = f"{status}：{reply.get('detail', '')}；未自动重试"
                self.pending = None
                return
            if status in ("ACCEPTED", "COMPLETED"):
                pending["ack"] = True
        if now >= pending["deadline"]:
            # End sustained control on a point timeout; never assume arrival.
            self.pending = None
            if self.phase == "CONTROL":
                self.operation("exit_control")
            self.message = "操作超时；已请求退出控制" if self.pending else "操作超时；请检查状态"
            return
        if not pending["ack"]:
            return
        if pending["stage"] == "start" and self.phase == "CONTROL":
            pending.update(stage="target", id=self.command("target", **pending["point"]), ack=False)
            self.message = f"{pending['label']}：正在运动…"
        elif pending["stage"] in ("target", "tracking"):
            pending["stage"] = "tracking"
            if self.phase != "CONTROL":
                self.message = "已退出控制，本次点位运动结束"
                self.pending = None
                return
            point = pending["point"]
            reference = self.state.get("wheel_reference", {})
            arm_fresh = self.state.get("freshness", {}).get("arm", {}).get("valid", False)
            near = arm_fresh and abs(reference.get("angle", math.inf) - point["angle"]) <= math.radians(.05) \
                and abs(reference.get("displacement", math.inf) - point["displacement"]) <= .0001
            if near:
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= .5:
                    self.message = f"{pending['label']}：指令参考已到点，保持中；可进行外部精度测量"
                    self.pending = None
            else:
                self.stable_since = None
                self.message = f"{pending['label']}：正在运动…"
        elif pending["stage"] == "operation":
            destination = dict(enter_standby="STANDBY", grasp_wheel="FOLLOWING", leave_wheel="STANDBY",
                               exit_control="FOLLOWING", reset_error="READY")[pending["operation"]]
            if self.phase == destination and self.state.get("system", {}).get("settled", False):
                self.message = f"操作完成：{destination}"
                self.pending = None


def run_ui(path, mode):
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("AVIATOR · 轮盘运动精度测试")
    root.geometry("940x800")
    root.minsize(900, 780)
    root.configure(background="#eef2f6")
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background="#eef2f6")
    style.configure("TLabel", background="#eef2f6", font=("sans-serif", 11))
    style.configure("TButton", font=("sans-serif", 11), padding=(12, 9))
    style.configure("Title.TLabel", font=("sans-serif", 21, "bold"))
    style.configure("Hint.TLabel", foreground="#526276", font=("sans-serif", 10))
    style.configure("Status.TLabel", foreground="#145aa3", font=("sans-serif", 12, "bold"))
    content = ttk.Frame(root, padding=24)
    content.pack(fill="both", expand=True)
    ttk.Label(content, text="轮盘运动精度测试", style="Title.TLabel").pack(anchor="w")
    ttk.Label(content, text=f"{mode}  ·  手动点位 / Managed Servo", style="Hint.TLabel").pack(anchor="w", pady=(4, 16))
    controller = Controller(path)
    status_text, message_text = tk.StringVar(value="等待连接"), tk.StringVar(value=controller.message)
    ttk.Label(content, textvariable=status_text, style="Status.TLabel").pack(anchor="w")
    actions = ttk.Frame(content)
    actions.pack(fill="x", pady=12)
    buttons = {}

    def perform(action):
        try:
            action()
        except (ValueError, OSError) as error:
            controller.message = str(error)
        message_text.set(controller.message)

    for caption, operation in (("Home", "enter_standby"), ("抓握", "grasp_wheel"), ("松手", "leave_wheel"),
                               ("停止 / 退出控制", "exit_control"), ("复位错误", "reset_error")):
        button = ttk.Button(actions, text=caption, command=lambda op=operation: perform(lambda: controller.operation(op)))
        button.pack(side="left", padx=(0, 8))
        buttons[operation] = button
    ttk.Separator(content).pack(fill="x", pady=8)
    ttk.Label(content, text="目标点位", font=("sans-serif", 14, "bold")).pack(anchor="w", pady=(8, 3))
    ttk.Label(content, text="转角约 ±50°；绝对位移 -170～0 mm。零位是行程端点，中位为 -85 mm。",
              style="Hint.TLabel").pack(anchor="w")
    grid = ttk.Frame(content)
    grid.pack(fill="x", pady=8)
    for column, text in enumerate(("点位", "转角 θ / °", "绝对位移 d / mm", "执行")):
        ttk.Label(grid, text=text, style="Hint.TLabel").grid(row=0, column=column, sticky="w", padx=10, pady=4)
    point_buttons = []
    for row, (angle, displacement) in enumerate(DEFAULT_POINTS, 1):
        label = f"P{row}"
        a, d = tk.StringVar(value=str(angle)), tk.StringVar(value=str(displacement))
        ttk.Label(grid, text=label).grid(row=row, column=0, padx=10)
        ttk.Entry(grid, textvariable=a, width=18, font=("sans-serif", 12)).grid(row=row, column=1, padx=10, pady=4)
        ttk.Entry(grid, textvariable=d, width=18, font=("sans-serif", 12)).grid(row=row, column=2, padx=10, pady=4)
        button = ttk.Button(grid, text=f"运动到 {label}", command=lambda a=a, d=d, label=label:
                            perform(lambda: controller.move(target(a.get(), d.get()), label)))
        button.grid(row=row, column=3, padx=10, pady=4)
        point_buttons.append(button)
    ttk.Separator(content).pack(fill="x", pady=10)
    reference_text = tk.StringVar(value="指令参考：—")
    measurement_text = tk.StringVar(value="轮盘实测：不可用；请使用外部测量")
    selected_text = tk.StringVar(value="本次目标：未选择")
    for variable in (selected_text, reference_text, measurement_text):
        ttk.Label(content, textvariable=variable).pack(anchor="w", pady=3)
    ttk.Label(content, textvariable=message_text, wraplength=870, style="Status.TLabel").pack(anchor="w", pady=(12, 4))
    ttk.Label(content, text="编辑坐标后点击对应按钮执行。指令参考到点不代表实测精度达标。关闭窗口将停止本次启动。",
              wraplength=870, style="Hint.TLabel").pack(anchor="w", side="bottom", pady=(8, 0))

    def update():
        controller.tick()
        fresh, phase = controller.fresh, controller.phase
        pending = controller.pending
        idle = controller.ready and pending is None
        settled = controller.state.get("system", {}).get("settled", False)
        status_text.set(f"{phase}  ·  {'反馈正常' if fresh else '等待 / 反馈过期'}")
        error = controller.state.get("system", {}).get("current_error", "")
        message_text.set(error or controller.message)
        enabled = dict(enter_standby=idle and phase in ("READY", "STANDBY", "FOLLOWING", "CONTROL"),
                       grasp_wheel=idle and phase == "STANDBY" and settled,
                       leave_wheel=idle and phase in ("FOLLOWING", "CONTROL"),
                       exit_control=controller.ready and phase == "CONTROL" and
                                    (not pending or pending["stage"] == "tracking"),
                       reset_error=idle and phase == "ERROR")
        for name, button in buttons.items():
            button.configure(state="normal" if enabled[name] else "disabled")
        for button in point_buttons:
            button.configure(state="normal" if idle and (phase == "CONTROL" or phase == "FOLLOWING" and settled) else "disabled")
        reference = controller.state.get("wheel_reference")
        if fresh and reference and controller.state.get("freshness", {}).get("arm", {}).get("valid"):
            reference_text.set(f"指令参考：θ = {math.degrees(reference['angle']):+.4f}°     d = {reference['displacement'] * 1000:+.3f} mm")
        else:
            reference_text.set("指令参考：—（等待新鲜机械臂反馈）")
        measured = controller.state.get("wheel_measurement", {})
        if fresh and measured.get("valid") and "angle" in measured and "displacement" in measured:
            measurement_text.set(f"轮盘实测：θ = {math.degrees(measured['angle']):+.4f}°     d = {measured['displacement'] * 1000:+.3f} mm")
        else:
            measurement_text.set("轮盘实测：不可用；请使用外部测量")
        if controller.selected:
            name, point = controller.selected
            selected_text.set(f"本次目标：{name}     θ = {math.degrees(point['angle']):+.4f}°     d = {point['displacement'] * 1000:+.3f} mm")
        root.after(20, update)

    def close():
        controller.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    update()
    root.mainloop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--mode", default="真机 + 真实双手")
    args = parser.parse_args()
    run_ui(args.socket, args.mode)
