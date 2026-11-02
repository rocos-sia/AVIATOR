"""Standard-library client for the persistent Rokae SDK session."""
from pathlib import Path
import json
import queue
import signal
import subprocess
import threading
import time


class NativeSession:
    """One SDK-owning child for the whole UI session; JSON requests are serialized."""
    def __init__(self, executable, robot_config, dry_run, log_dir, log, *, error_type=RuntimeError):
        self.error_type = error_type
        self.log = log
        self._lock = threading.Lock()
        self._responses = queue.Queue()
        self._id = 0
        self._broken = False
        self.log_path = Path(log_dir).resolve() / f"arms-{time.time_ns()}.log"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        # xCoreSDK opens ./logs/global_logger_DATE.log during connect and aborts
        # if it cannot write it. Redirecting stderr does not relocate that log.
        # Give each helper a fresh writable working directory alongside its log,
        # including when the UI fell back from root-owned repository logs to /tmp.
        self.sdk_work_dir = self.log_path.with_suffix("")
        (self.sdk_work_dir / "logs").mkdir(parents=True, exist_ok=False)
        executable = Path(executable).resolve()
        robot_config = Path(robot_config).resolve()
        self._errors = self.log_path.open("w", encoding="utf-8")
        self._errors.write(f"SDK working directory: {self.sdk_work_dir}\n")
        self._errors.flush()
        argv = [str(executable), "--robot-config", str(robot_config)]
        if dry_run:
            argv.append("--dry-run")
        try:
            self.process = subprocess.Popen(argv, cwd=self.sdk_work_dir, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=self._errors,
                                            text=True, encoding="utf-8", bufsize=1,
                                            start_new_session=True)
        except BaseException:
            self._errors.close()
            raise
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(262144)
                if not line:
                    break
                if not line.endswith("\n"):
                    raise ValueError("native response too large or incomplete")
                self._responses.put(json.loads(line))
        except Exception as exc:
            self._responses.put(exc)
        finally:
            self._responses.put(RuntimeError(f"机械臂会话已退出，请查看 {self.log_path}"))

    def request(self, op, timeout=60, **fields):
        with self._lock:
            if self._broken or self.process.poll() is not None:
                raise self.error_type(f"机械臂连接不可用，请结束会话；日志 {self.log_path}")
            self._id += 1
            try:
                self.process.stdin.write(json.dumps(dict(id=self._id, op=op, **fields)) + "\n")
                self.process.stdin.flush()
                response = self._responses.get(timeout=timeout)
                if isinstance(response, Exception):
                    raise response
                if response.get("id") != self._id:
                    raise ValueError("机械臂响应序号不匹配")
            except Exception as exc:
                self._broken = True
                raise self.error_type(f"机械臂通信失败（{op}）：{exc}") from exc
            if response.get("ok") is not True:
                raise self.error_type(response.get("error", "机械臂操作失败"))
            return response["result"]

    def close(self):
        # EOF lets the child disable owned drag before disconnecting. SIGTERM has
        # the same orderly path; forced termination is reported, never silent.
        errors = []
        try:
            if self.process.poll() is None:
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=3)
                        errors.append("机械臂 SDK 无响应，已终止自有会话进程；请现场确认机器人状态")
            if self.process.returncode and not (self.process.returncode == 128 + signal.SIGTERM
                    and "Session cleanup complete" in self.log_path.read_text(errors="replace")):
                errors.append(f"机械臂会话退出码 {self.process.returncode}；日志 {self.log_path}")
        finally:
            self._reader.join(timeout=1)
            self.process.stdout.close()
            if not self.process.stdin.closed:
                self.process.stdin.close()
            self._errors.close()
        if errors:
            raise self.error_type("；".join(errors))

    def interrupt(self):
        """Request SDK-owned cleanup even when a JSON request is still waiting."""
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass
