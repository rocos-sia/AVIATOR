"""Own the launcher shell, not its independently managed node process groups."""
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid

from options import Options
from paths import resolve_layout, runtime_dir, user_dir
from protocol import peer_uid, receive, send
from settings import atomic_json

ACTIVE = {'VALIDATING', 'STARTING', 'RUNNING', 'STOPPING'}


def process_identity(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return [Path('/proc/sys/kernel/random/boot_id').read_text().strip(), stat[19]]
    except (OSError, IndexError):
        return None


def script_locked():
    path = Path(os.environ['XDG_RUNTIME_DIR']) / f'aviator-start-{os.getuid()}.lock'
    with path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return False
        except BlockingIOError:
            return True


class Runner:
    def __init__(self, directory):
        self.directory = directory
        self.child = None
        self.log = None
        self.offset = 0
        self.pending = b''
        self.stop_requested = False
        self.quit_requested = False
        self.last_client = time.monotonic()
        self.state = {'status': 'IDLE', 'root': '', 'log_path': '', 'node_log_dir': '',
                      'output': '', 'warning': '', 'pid': None, 'exit_code': None}
        self.state_file = directory / 'state.json'
        if self.state_file.exists():
            try:
                old = json.loads(self.state_file.read_text())
                if old.get('status') in ACTIVE | {'NEEDS_ATTENTION'}:
                    if old.get('status') == 'NEEDS_ATTENTION' or (old.get('pid') and old.get('identity') == process_identity(old['pid'])) or script_locked():
                        self.state.update(old)
                        self.state.update(status='NEEDS_ATTENTION', warning='上次运行管理进程已失联；请检查运行日志及启动锁，未接管遗留进程。')
            except (ValueError, OSError, TypeError):
                self.state.update(status='NEEDS_ATTENTION', warning='上次运行状态文件无法读取，请检查遗留进程。')

    def persist(self):
        atomic_json(self.state_file, {k: v for k, v in self.state.items() if k != 'output'})

    def start(self, data):
        if self.child or self.state['status'] == 'NEEDS_ATTENTION' or script_locked():
            raise ValueError('已有运行或遗留状态待处理，请先停止并检查启动锁。')
        layout = resolve_layout(data['root'])
        options = Options.from_dict(data['options'])
        environment = options.preflight(layout)
        root = user_dir('XDG_STATE_HOME', '.local/state') / 'aviator-launcher/runs'
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        # Only prune our own closed run files; never touch node logs or recordings.
        prior = sorted(root.glob('*.log'), key=lambda p: p.stat().st_mtime)
        for old in prior[:-19]:
            old.unlink()
        run_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8]
        path = root / (run_id + '.log')
        self.log = path.open('ab', buffering=0)
        os.chmod(path, 0o600)
        self.offset, self.pending = 0, b''
        self.stop_requested = False
        self.state = {'status': 'STARTING', 'root': str(layout.root), 'log_path': str(path),
                      'node_log_dir': '', 'output': '', 'warning': '', 'pid': None,
                      'exit_code': None, 'options': options.to_dict(), 'run_id': run_id}
        try:
            self.child = subprocess.Popen(options.command(layout), cwd=layout.root,
                env=environment, stdin=subprocess.DEVNULL,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
        except Exception:
            self.log.close()
            self.log = None
            self.state['status'] = 'FAILED'
            self.persist()
            raise
        self.state.update(pid=self.child.pid, identity=process_identity(self.child.pid))
        self.persist()

    def stop(self):
        if self.child and self.child.poll() is None and not self.stop_requested:
            self.stop_requested = True
            self.state['status'] = 'STOPPING'
            self.child.send_signal(signal.SIGTERM)
            self.persist()

    def recheck(self):
        if self.child or script_locked():
            raise ValueError('脚本或启动锁仍在使用，尚不能清除待处理状态。')
        pid = self.state.get('pid')
        identity = self.state.get('identity')
        if pid and identity and process_identity(pid) == identity:
            raise ValueError('上次脚本进程仍存在，未接管或发送信号。')
        if self.state['status'] == 'NEEDS_ATTENTION' and not identity:
            raise ValueError('缺少可靠的历史进程身份；请人工检查状态文件和遗留进程。')
        self.state.update(status='IDLE', warning='', residual=False, pid=None)
        self.persist()

    def tick(self):
        if not self.child:
            return
        previous = (self.state['status'], self.state['node_log_dir'], self.state['warning'])
        with open(self.state['log_path'], 'rb') as stream:
            stream.seek(self.offset)
            chunk = stream.read(65536)
            self.offset += len(chunk)
        self.state['output'] = (self.state['output'] + chunk.decode('utf-8', 'replace'))[-24000:]
        lines = (self.pending + chunk).split(b'\n')
        self.pending = lines.pop()[-4096:]
        for raw in lines:
            line = raw.decode('utf-8', 'replace')
            if line.startswith('日志目录: '):
                self.state['node_log_dir'] = line.split(': ', 1)[1]
            if line.startswith('所有节点已启动') and not self.stop_requested:
                self.state['status'] = 'RUNNING'
            if line.startswith('TTS 已退出'):
                self.state['warning'] = '离线语音已退出；控制节点继续运行。'
            if 'SIGKILL 后仍未退出' in line:
                self.state['warning'] = '存在未退出的节点，请查看日志。'
                self.state['residual'] = True
            if line.startswith('正在停止本次启动的节点'):
                self.state['status'] = 'STOPPING'
        # Shell diagnostics are low volume, but cap runaway output without a UI pipe.
        if self.log and os.fstat(self.log.fileno()).st_size > 32 * 1024 * 1024:
            os.ftruncate(self.log.fileno(), 0)
            self.offset = 0
            self.state['warning'] = '启动脚本输出超过 32 MiB，已截断日志；状态请结合节点日志核对。'
        code = self.child.poll()
        if code is not None:
            # Drain remaining output on subsequent ticks before finalizing.
            if len(chunk) == 65536:
                return
            self.state['exit_code'] = code
            self.log.close()
            self.log = None
            self.child = None
            if self.state.get('residual') or script_locked():
                self.state['status'] = 'NEEDS_ATTENTION'
                self.state['warning'] = '脚本已退出，但仍有残留节点或启动锁占用。'
            else:
                self.state['status'] = 'IDLE' if self.stop_requested and code in (0, 143, -15) else 'FAILED'
                if self.state['status'] == 'FAILED':
                    self.state['warning'] = f'启动脚本结束（退出码 {code}），请查看输出。'
            self.persist()
        elif previous != (self.state['status'], self.state['node_log_dir'], self.state['warning']):
            self.persist()

    def handle(self, data):
        if not isinstance(data, dict) or data.get('version') != 1:
            raise ValueError('不支持的控制协议')
        action = data.get('action')
        if action == 'start':
            self.start(data)
        elif action == 'stop':
            self.stop()
        elif action == 'recheck':
            self.recheck()
        elif action == 'quit':
            if self.state['status'] == 'NEEDS_ATTENTION':
                raise ValueError('遗留状态待处理，未强制结束任何节点。')
            self.stop()
            self.quit_requested = True
        elif action != 'status':
            raise ValueError('未知控制操作')
        self.last_client = time.monotonic()
        return dict(self.state, external_lock=not self.child and script_locked())


def serve():
    if os.geteuid() == 0:
        raise ValueError('请以普通用户运行启动器')
    directory = runtime_dir()
    with (directory / 'runner.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        runner = Runner(directory)
        path = directory / 'control.sock'
        path.unlink(missing_ok=True)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            os.chmod(path, 0o600)
            server.listen(4)
            server.settimeout(.25)

            def shutdown(*_):
                runner.stop()
                runner.quit_requested = True

            for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
                signal.signal(sig, shutdown)
            try:
                while True:
                    runner.tick()
                    if not runner.child and (runner.quit_requested or time.monotonic() - runner.last_client > 60):
                        break
                    try:
                        client, _ = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(.5)
                        try:
                            if peer_uid(client) != os.getuid():
                                raise ValueError('仅允许当前用户连接')
                            result = runner.handle(receive(client))
                            send(client, {'ok': True, 'state': result})
                        except (OSError, ValueError, KeyError, TypeError) as exc:
                            try:
                                send(client, {'ok': False, 'error': str(exc)})
                            except OSError:
                                pass
            finally:
                path.unlink(missing_ok=True)


if __name__ == '__main__':
    serve()
