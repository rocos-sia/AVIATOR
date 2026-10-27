"""Local SSH transport and run lifecycle; no GUI or third-party Python dependencies."""
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import uuid

URL = "http://127.0.0.1:8081/"
STATE_DIR = Path.home() / ".local/state/aviator-remote-ui"

# Ubuntu/Linux: the SSH master must not survive its owning GUI/worker. Use an
# exec wrapper rather than preexec_fn, which is unsafe in this threaded GUI.
OWNED_EXEC = """
import ctypes, os, signal, sys
libc = ctypes.CDLL(None, use_errno=True)
libc.prctl.argtypes = [ctypes.c_int] + [ctypes.c_ulong] * 4
if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), 'cannot bind SSH lifetime to its owner')
if os.getppid() != int(sys.argv[1]):
    sys.exit(1)
os.execvp(sys.argv[2], sys.argv[2:])
"""


def owned_command(command):
    return [sys.executable, "-c", OWNED_EXEC, str(os.getpid()), *command]


@dataclass
class Options:
    target: str = "rocos@192.168.31.100"
    prefix: str = "/opt/aviator"
    rs422: bool = False
    fake_hand: bool = False
    tts: bool = False
    logger: bool = False
    transfer: bool = False
    keyboard: bool = False
    destination: str = str(Path.home() / "AVIATOR-recordings")

    def flags(self):
        return [flag for flag, enabled in (("--rs422", self.rs422),
                ("--fake-hand", self.fake_hand), ("--tts", self.tts),
                ("--logger", self.logger)) if enabled]

    def validate(self):
        if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", self.target):
            raise ValueError("连接地址请填写 用户名@IPv4地址或主机名。")
        if not Path(self.prefix).is_absolute() or "\n" in self.prefix:
            raise ValueError("工控机安装目录必须是绝对路径。")
        if self.transfer and not self.logger:
            raise ValueError("请先启用记录数据，再选择回传。")
        if self.keyboard and self.rs422:
            raise ValueError("本机键盘控制与 RS422 输入不能同时启用。")
        if self.transfer and not Path(self.destination).expanduser().is_absolute():
            raise ValueError("本机保存目录必须是绝对路径。")


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temporary.chmod(0o600)
    temporary.replace(path)


class TransferCancelled(RuntimeError):
    pass


class Transport:
    def __init__(self, target):
        self.target = target
        self.temp = tempfile.TemporaryDirectory(prefix="aviator-ui-ssh-")
        self.control = str(Path(self.temp.name) / "socket")
        self.keyboard = False
        self.master = None
        self.master_log = None
        self.forwarded = False
        self.options = ["-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=5",
                        "-o", "ServerAliveCountMax=3", "-o", "ControlPath=" + self.control]

    def connect(self, password, keyboard=False):
        self.keyboard = keyboard
        if keyboard and (not os.environ.get("DISPLAY") or not shutil.which("xauth")):
            raise RuntimeError("本机键盘控制需要桌面 DISPLAY 和 xauth；请运行安装包的 install_dependencies.sh。")
        helper = Path(self.temp.name) / "askpass"
        helper.write_text('#!/usr/bin/python3\nimport os\nprint(os.environ.get("AVIATOR_SSH_PASSWORD", ""))\n')
        helper.chmod(0o700)
        env = {**os.environ, "SSH_ASKPASS": str(helper), "SSH_ASKPASS_REQUIRE": "force",
               "DISPLAY": os.environ.get("DISPLAY") or ":0", "AVIATOR_SSH_PASSWORD": password}
        command = ["ssh", *self.options, *self.x11_options(), "-o", "StrictHostKeyChecking=accept-new",
                   "-o", "NumberOfPasswordPrompts=1", "-o", "ControlMaster=yes",
                   "-o", "ControlPersist=no", "-o", "ForkAfterAuthentication=no", "-N", self.target]
        self.master_log = tempfile.TemporaryFile(dir=self.temp.name)
        self.master = subprocess.Popen(owned_command(command), env=env, stdin=subprocess.DEVNULL,
                                       stdout=subprocess.DEVNULL, stderr=self.master_log,
                                       start_new_session=True)
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if self.master.poll() is not None:
                    self.master_log.seek(0)
                    raise RuntimeError(self.master_log.read().decode(errors="replace").strip() or "SSH 连接失败。")
                if Path(self.control).exists():
                    result = subprocess.run(["ssh", *self.options, "-O", "check", self.target],
                                            capture_output=True, timeout=2)
                    if result.returncode == 0:
                        return
                time.sleep(0.05)
            raise RuntimeError("SSH 连接或认证超时。")
        except BaseException:
            self.close()
            raise

    @staticmethod
    def checked(command, **kwargs):
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, **kwargs)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or
                               f"命令失败，退出码 {result.returncode}")
        return result.stdout

    def x11_options(self):
        # Untrusted forwarding hides MIT-SHM, which cannot work across hosts.
        # remote_agent disables SDL's incompatible WM ping for these sessions.
        return ["-X", "-o", "ForwardX11Trusted=no"] if self.keyboard else ["-x"]

    def ssh(self):
        return ["ssh", *self.options, *self.x11_options(), "-o", "BatchMode=yes", "-T"]

    def agent(self, action, options, run_id):
        source = Path(__file__).with_name("remote_agent.py").read_text()
        keyboard = ["--keyboard"] if options.keyboard and action in ("probe", "run") else []
        return [*self.ssh(), self.target, shlex.join([
            "python3", "-u", "-c", source, action, "--prefix", options.prefix,
            "--run-id", run_id, "--flags", json.dumps(options.flags()), *keyboard])]

    def query(self, action, options, run_id):
        return json.loads(self.checked(self.agent(action, options, run_id), timeout=20))

    def forward(self):
        # Refuse to open an unrelated local service if the desired port is busy.
        with socket.socket() as check:
            # Match SSH's bind behavior: a closed connection in TIME_WAIT is
            # reusable, while an existing live listener must still be refused.
            check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                check.bind(("127.0.0.1", 8081))
            except OSError as exc:
                raise RuntimeError("本机 8081 端口已被占用，请先关闭占用它的程序。") from exc
        self.checked(["ssh", *self.options, "-O", "forward", "-o", "ExitOnForwardFailure=yes",
                      "-L", "127.0.0.1:8081:127.0.0.1:8081", self.target], timeout=15)
        self.forwarded = True

    def sync(self, remote, local, final=False, cancel=None):
        command = ["rsync", "-rt", "--partial", "--protect-args", "--timeout=15",
                   "--bwlimit=8192", "--checksum" if final else "--append-verify",
                   "-e", shlex.join(self.ssh()), "--", f"{self.target}:{remote}/", str(local) + "/"]
        if cancel and cancel.is_set():
            raise TransferCancelled("回传已暂停")
        process = subprocess.Popen(owned_command(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True)
        deadline = time.monotonic() + (300 if final else 30)
        try:
            while True:
                if cancel and cancel.is_set():
                    raise TransferCancelled("回传已暂停")
                if time.monotonic() >= deadline:
                    raise RuntimeError("回传超时")
                try:
                    output, error = process.communicate(timeout=.2)
                    break
                except subprocess.TimeoutExpired:
                    continue
            if process.returncode:
                raise RuntimeError(error.strip() or output.strip() or f"回传失败，退出码 {process.returncode}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()

    def close(self):
        try:
            if self.master is not None and self.master.poll() is None:
                # An exit request is not proof of exit: X11/forwarded channels
                # may keep the master alive, or its control socket may be gone.
                if Path(self.control).exists():
                    controls = []
                    if self.forwarded:
                        controls.append(["cancel", "-L", "127.0.0.1:8081:127.0.0.1:8081"])
                    controls.append(["exit"])
                    for command in controls:
                        try:
                            subprocess.run(["ssh", *self.options, "-O", *command, self.target],
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
                        except (OSError, subprocess.TimeoutExpired):
                            pass
                try:
                    self.master.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.master.terminate()
                    try:
                        self.master.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.master.kill()
                        self.master.wait(timeout=2)
                self.forwarded = False
        finally:
            # Keep diagnostics/control files if the owned process is somehow
            # still alive; never abandon it merely because ssh -O returned.
            if self.master is None or self.master.poll() is not None:
                if self.master_log is not None:
                    self.master_log.close()
                self.temp.cleanup()


class Controller:
    def __init__(self, state_dir=STATE_DIR):
        self.state_dir = Path(state_dir)
        self.session_file = self.state_dir / "session.json"
        self.events = queue.Queue()
        self.worker = None
        self.stop_requested = threading.Event()
        self.transfer_dir = self.state_dir / "pending-transfers"
        self.transfers = {}
        self.transfer_lock = threading.Lock()
        self.closing = threading.Event()

    @property
    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def emit(self, event, **values):
        self.events.put({"event": event, **values})

    def log(self, text):
        self.emit("log", text=text)

    def start(self, options, password="", recover=False):
        if self.busy:
            raise RuntimeError("上一次操作仍在进行中。")
        options.validate()
        for program in ("ssh", "rsync") if options.transfer else ("ssh",):
            if not shutil.which(program):
                raise RuntimeError(f"本机缺少 {program}。")
        if self.session_file.exists() and not recover:
            self._archive_session()
        if recover and not self.session_file.exists() and not any(self.transfer_dir.glob("*.json")):
            raise RuntimeError("没有待收尾的会话。")
        self.stop_requested.clear()
        self.worker = threading.Thread(target=self._work, args=(options, password, recover), daemon=True)
        self.worker.start()

    def _archive_session(self):
        archive = self.state_dir / "archived-sessions" / f"{time.time_ns()}.json"
        archive.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.session_file.replace(archive)
        self.log(f"已跳过旧会话收尾，记录保存在 {archive}；继续启动新会话。\n")

    def stop(self):
        self.stop_requested.set()
        self.log("已请求停止；等待工控机按原脚本顺序退出并完成写盘。\n")

    def close(self):
        self.closing.set()
        if self.busy:
            self.stop()
        with self.transfer_lock:
            for _, cancel in self.transfers.values():
                cancel.set()

    def _queue_transfer(self, path, password):
        with self.transfer_lock:
            previous = self.transfers.get(path)
            if self.closing.is_set() or (previous and previous[0].is_alive()):
                return
            cancel = threading.Event()
            worker = threading.Thread(target=self._transfer, args=(path, password, cancel), daemon=True)
            self.transfers[path] = (worker, cancel)
            worker.start()

    def _transfer(self, path, password, cancel):
        transport = None
        label = path.stem[:8]
        try:
            session = json.loads(path.read_text())
            if cancel.is_set():
                raise TransferCancelled("回传已暂停")
            transport = Transport(session["options"]["target"])
            # Data transfer has its own SSH connection, without Monitor/X11 forwarding.
            transport.connect(password)
            password = ""
            self.emit("transfer", text=f"{label}：后台补传中，可启动下一次运行")
            transport.sync(session["result"]["recording"], session["local"], final=True, cancel=cancel)
            write_json(Path(session["local"], "session.json"), session)
            Path(session["local"], ".in-progress").unlink(missing_ok=True)
            path.unlink(missing_ok=True)
            self.emit("transfer", text=f"{label}：数据已回传并完成文件校验")
        except TransferCancelled:
            self.log(f"{label}：回传已暂停，记录已保留，可稍后补传。\n")
        except Exception as exc:
            self.log(f"{label}：后台回传未完成，可稍后补传：{exc}\n")
            self.emit("transfer", text=f"{label}：待补传（不影响启动）")
        finally:
            if transport:
                transport.close()

    def _work(self, options, password, recover):
        transport = None
        sync_stop = threading.Event()
        sync_worker = None
        process = None
        heartbeat_stop = threading.Event()
        heartbeat_worker = None
        try:
            if recover and not self.session_file.exists():
                for path in self.transfer_dir.glob("*.json"):
                    self._queue_transfer(path, password)
                self.emit("state", text="后台补传已安排，可启动新会话")
                return
            self.emit("state", text="连接中")
            if recover:
                session = json.loads(self.session_file.read_text())
                options = Options(**session["options"])
                options.validate()
            else:
                session = {"run_id": uuid.uuid4().hex, "options": asdict(options)}
            run_id = session["run_id"]
            transport = Transport(options.target)
            transport.connect(password, keyboard=options.keyboard and not recover)
            if recover:
                state = transport.query("stop", options, run_id)
                self.emit("state", text="等待远端收尾")
                # Agent completion, not just SSH process exit, confirms ordered cleanup finished.
                deadline = time.monotonic() + 120
                while state["phase"] != "finished" and state["phase"] != "missing":
                    if not state.get("agent_alive") and not state.get("child_alive"):
                        raise RuntimeError("远端管理进程异常退出，无法确认节点已收尾；请检查远端日志和进程。")
                    if time.monotonic() > deadline:
                        raise RuntimeError("远端仍在收尾，请稍后再次重连。")
                    time.sleep(1)
                    state = transport.query("status", options, run_id)
                if state["phase"] == "missing":
                    # UUID was reserved locally but the remote agent never persisted a run.
                    self.session_file.unlink()
                    self.emit("state", text="会话未在远端创建，可重新启动")
                    return
            else:
                transport.query("probe", options, run_id)
                if self.stop_requested.is_set():
                    self.emit("state", text="已取消，未启动")
                    return
                transport.forward()
                if options.transfer:
                    local = Path(options.destination).expanduser() / run_id
                    local.mkdir(parents=True, exist_ok=False)
                    (local / ".in-progress").touch()
                    session["local"] = str(local)
                write_json(self.session_file, session)
                self.log("远端命令：" + shlex.join([options.prefix + "/share/aviator/scripts/start_aviator.sh", *options.flags()]) + "\n")
                if options.keyboard:
                    self.log("本机键盘控制已启用：请点击随后出现的 flight_gateway 输入窗口，再使用原有按键。Monitor 浏览器和此启动界面不接收动作按键。\n")
                self.emit("state", text="启动中")
                process = subprocess.Popen(transport.agent("run", options, run_id),
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, bufsize=1)

                def heartbeat():
                    sent_stop = False
                    while not heartbeat_stop.is_set():
                        try:
                            command = "stop" if self.stop_requested.is_set() and not sent_stop else "heartbeat"
                            process.stdin.write(json.dumps({"command": command}) + "\n")
                            process.stdin.flush()
                            sent_stop |= command == "stop"
                        except (OSError, ValueError):
                            return
                        heartbeat_stop.wait(1)

                heartbeat_worker = threading.Thread(target=heartbeat, daemon=True)
                heartbeat_worker.start()
                state = None
                for line in process.stdout:
                    try:
                        event = json.loads(line)
                    except ValueError:
                        self.log(line)
                        continue
                    kind = event.get("event")
                    if kind == "output":
                        self.log(event["text"])
                    elif kind == "starting":
                        session["remote"] = event["recording"]
                        write_json(self.session_file, session)
                        self.log("工控机启动日志：" + event["log"] + "\n")
                        if options.logger:
                            self.log("工控机数据目录：" + event["recording"] + "\n")
                        if options.transfer:
                            self.log("本机回传目录：" + session["local"] + "\n")

                            def sync_live():
                                while not sync_stop.wait(5):
                                    try:
                                        transport.sync(session["remote"], session["local"], cancel=sync_stop)
                                        self.emit("transfer", text="回传中（录制尚未结束）")
                                    except TransferCancelled:
                                        return
                                    except Exception as exc:
                                        self.log("本轮回传未完成，稍后重试：" + str(exc) + "\n")

                            sync_worker = threading.Thread(target=sync_live, daemon=True)
                            sync_worker.start()
                    elif kind == "running":
                        self.emit("state", text="运行中")
                        try:
                            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                            with opener.open(URL, timeout=5) as response:
                                response.read(1)
                            self.emit("browser")
                        except Exception as exc:
                            self.log("Monitor 本机连接检查失败，可稍后点击打开 Monitor：" + str(exc) + "\n")
                    elif kind == "stopping":
                        self.emit("state", text="停止中，等待设备收尾")
                    elif kind == "exit":
                        state = event
                process.wait(timeout=10)
                if state is None:
                    raise RuntimeError("SSH 连接已断开。工控机连接看护会请求正常停止；请使用“重连并收尾 / 补传”确认。")
            sync_stop.set()
            if sync_worker:
                sync_worker.join()
            if options.transfer:
                pending = self.transfer_dir / (run_id + ".json")
                write_json(pending, {**session, "result": state})
                self._queue_transfer(pending, password)
            self.session_file.unlink(missing_ok=True)
            code = state.get("returncode", 1)
            normal = code == 0 or (code in (130, 143) and (self.stop_requested.is_set() or recover))
            self.emit("state", text="已停止" if normal else f"运行已退出（退出码 {code}，请查看日志）")
        except Exception as exc:
            self.log(str(exc) + "\n")
            self.emit("state", text="需要重连收尾 / 补传" if self.session_file.exists() else "启动失败")
            self.emit("error", text=str(exc))
        finally:
            # Closing stdin triggers the agent's graceful cleanup if this worker fails.
            heartbeat_stop.set()
            if heartbeat_worker:
                heartbeat_worker.join(timeout=2)
            if process and process.stdin:
                process.stdin.close()
            sync_stop.set()
            if sync_worker:
                sync_worker.join()
            if transport:
                try:
                    transport.close()
                except Exception as exc:
                    self.log("SSH 隧道清理失败：" + str(exc) + "\n")
                    self.emit("state", text="SSH 隧道清理失败")
                    self.emit("error", text="SSH 隧道尚未确认退出，请查看日志。")
            if process:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()  # SSH client only; remote agent handles HUP/EOF.
                    process.wait(timeout=5)
                process.stdout.close()
            self.emit("done")
