"""SSH supervisor. Only signal the launcher, which owns ordered device shutdown."""
import argparse
import ctypes
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time

ALLOWED = {"--rs422", "--fake-hand", "--tts", "--logger"}


class NodeDiagnostics:
    """Read this launch's warnings without touching nodes or their log files."""
    nodes = ("hand", "core", "manipulator", "gateway", "camera", "photoelectric", "tts", "logger", "bus")
    levels = re.compile(r"\[(?:warning|warn|error|critical)\]|Traceback \(most recent call last\)", re.I)

    def __init__(self):
        self.directory = None
        self.cursors = {}

    def observe(self, line):
        if self.directory is None and line.startswith("日志目录: "):
            path = Path(line.partition(": ")[2].strip())
            if path.is_absolute() and path.name.startswith("aviator-start."):
                self.directory = path

    def read(self):
        if self.directory is None:
            return []
        output = []
        for node in self.nodes:
            try:
                with (self.directory / (node + ".log")).open("rb") as log:
                    stat = os.fstat(log.fileno())
                    identity = (stat.st_dev, stat.st_ino)
                    previous, offset, pending = self.cursors.get(node, (None, 0, b""))
                    if previous != identity or stat.st_size < offset:
                        offset, pending = 0, b""
                    log.seek(offset)
                    # Bound work even if a node floods its log; never delay the
                    # supervisor's heartbeat/stop handling to drain a file.
                    data = log.read(65536)
                    lines = (pending + data).split(b"\n")
                    self.cursors[node] = (identity, log.tell(), lines.pop()[-65536:])
                for raw in lines:
                    line = raw.decode("utf-8", errors="replace")
                    if self.levels.search(line):
                        output.append(f"[{node}] {line}\n")
            except OSError:
                # Files appear gradually during startup and may disappear later.
                continue
        return output


def check_keyboard(script):
    if "AVIATOR_REMOTE_KEYBOARD" not in script.read_text():
        raise RuntimeError("工控机启动脚本尚未支持本机键盘，请先更新启动脚本。")
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("SSH 未建立 X11 转发：检查工控机已安装 xauth 且 SSH 服务允许 X11Forwarding。未启动设备。")
    try:
        x11 = ctypes.CDLL("libX11.so.6")
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError("无法连接本机键盘窗口的 X11 显示。未启动设备。")
        x11.XCloseDisplay(display)
    except OSError as exc:
        raise RuntimeError("工控机缺少 X11 显示库。未启动设备。") from exc


def identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return [Path("/proc/sys/kernel/random/boot_id").read_text().strip(), fields[19]]
    except (OSError, IndexError):
        return None


def alive(state, key):
    return bool(state.get(key) and state.get(key + "_identity") and
                identity(state[key]) == state[key + "_identity"])


def save(path, state):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, ensure_ascii=False))
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("probe", "run", "status", "stop"))
    parser.add_argument("--prefix", default="/opt/aviator")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--flags", type=json.loads, default=[])
    parser.add_argument("--keyboard", action="store_true")
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9a-f]{32}", args.run_id):
        parser.error("invalid run id")
    if (not isinstance(args.flags, list) or not all(isinstance(f, str) for f in args.flags)
            or not Path(args.prefix).is_absolute() or set(args.flags) - ALLOWED):
        parser.error("invalid prefix or flags")
    if args.keyboard and "--rs422" in args.flags:
        parser.error("本机键盘控制与 RS422 不能同时启用")
    os.umask(0o077)
    directory = Path.home() / ".local/state/aviator-remote" / args.run_id
    statefile = directory / "state.json"
    script = Path(args.prefix) / "share/aviator/scripts/start_aviator.sh"
    if args.keyboard and args.action in ("probe", "run"):
        check_keyboard(script)
    if args.action == "probe":
        if "AVIATOR_REMOTE_UI" not in script.read_text():
            raise RuntimeError("工控机启动脚本尚未支持远程界面，请先重新部署当前代码。")
        subprocess.run(["bash", "-n", str(script)], check=True)
        if "--logger" in args.flags:
            subprocess.run(["rsync", "--version"], stdout=subprocess.DEVNULL, check=True)
        print(json.dumps({"ready": True}))
        return
    if args.action in ("status", "stop"):
        if not statefile.exists():
            print(json.dumps({"phase": "missing"}))
            return
        state = json.loads(statefile.read_text())
        state["agent_alive"] = alive(state, "agent_pid")
        state["child_alive"] = alive(state, "child_pid")
        if args.action == "stop":
            key = "agent_pid" if state["agent_alive"] else "child_pid"
            if alive(state, key):
                os.kill(state[key], signal.SIGTERM)
        print(json.dumps(state))
        return

    directory.mkdir(parents=True, exist_ok=False)
    lock = (directory.parent / "supervisor.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    recording = Path.home() / ".local/share/aviator-remote/recordings" / args.run_id
    if "--logger" in args.flags:
        recording.mkdir(parents=True)
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    signal.signal(signal.SIGHUP, lambda *_: stopping.set())
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    heartbeat = [time.monotonic()]
    connected = [True]
    state = {"phase": "starting", "run_id": args.run_id, "recording": str(recording),
             "agent_pid": os.getpid(), "agent_pid_identity": identity(os.getpid()),
             "log": str(directory / "launcher.log")}
    save(statefile, state)

    def emit(event, **values):
        if connected[0]:
            try:
                print(json.dumps({"event": event, **values}, ensure_ascii=False), flush=True)
            except (BrokenPipeError, OSError):
                connected[0] = False
                stopping.set()

    def commands():
        try:
            for line in sys.stdin:
                command = json.loads(line)
                if command.get("command") == "stop":
                    stopping.set()
                if command.get("command") == "heartbeat":
                    heartbeat[0] = time.monotonic()
        except (ValueError, OSError):
            pass
        finally:
            stopping.set()

    threading.Thread(target=commands, daemon=True).start()
    env = {**os.environ, "AVIATOR_REMOTE_UI": "1", "PYTHONUNBUFFERED": "1",
           "AVIATOR_REMOTE_KEYBOARD": "1" if args.keyboard else "0"}
    if "--logger" in args.flags:
        env["AVIATOR_RECORDING_DIR"] = str(recording)
    child = None
    try:
        if stopping.is_set():
            raise RuntimeError("连接在启动前已关闭，未启动设备。")
        child = subprocess.Popen(["bash", str(script), *args.flags], env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        state.update(child_pid=child.pid, child_pid_identity=identity(child.pid))
        save(statefile, state)
        emit("starting", **state)
        output = queue.Queue(maxsize=1024)

        def read_output():
            for line in iter(child.stdout.readline, b""):
                output.put(line.decode("utf-8", errors="replace"))
            output.put(None)

        threading.Thread(target=read_output, daemon=True).start()
        sent_stop = False
        ended = False
        diagnostics = NodeDiagnostics()
        with (directory / "launcher.log").open("a", buffering=1) as log:
            def relay_diagnostics():
                for message in diagnostics.read():
                    log.write(message)
                    emit("output", text=message)

            while not ended or child.poll() is None:
                if time.monotonic() - heartbeat[0] > 20:
                    stopping.set()
                if stopping.is_set() and not sent_stop:
                    if child.poll() is None:
                        child.send_signal(signal.SIGTERM)
                    sent_stop = True
                    state["phase"] = "stopping"
                    save(statefile, state)
                    emit("stopping")
                relay_diagnostics()
                try:
                    line = output.get(timeout=0.2)
                except queue.Empty:
                    continue
                if line is None:
                    ended = True
                else:
                    diagnostics.observe(line)
                    log.write(line)
                    emit("output", text=line)
                    if "所有节点已启动" in line and not sent_stop:
                        state["phase"] = "running"
                        save(statefile, state)
                        emit("running")
            relay_diagnostics()
        state.update(phase="finished", returncode=child.wait())
    except BaseException as exc:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGTERM)
            child.wait()  # Never interrupt the launcher's ordered cleanup.
        state.update(phase="finished", returncode=1, error=str(exc))
        emit("output", text=str(exc) + "\n")
    finally:
        save(statefile, state)
        emit("exit", **state)


if __name__ == "__main__":
    main()
