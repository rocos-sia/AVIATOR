import json
import os
from pathlib import Path
import queue
import shlex
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1]
ROOT = SOURCE.parents[1]
sys.path.insert(0, str(SOURCE))
from backend import Controller, Options, Transport
from remote_agent import identity, NodeDiagnostics


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="aviator remote tests 空格 ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.home = self.root / "home"
        self.home.mkdir()
        self.prefix = self.root / "prefix"
        self.script = self.prefix / "share/aviator/scripts/start_aviator.sh"
        self.script.parent.mkdir(parents=True)
        self.script.write_text('''#!/bin/bash
# AVIATOR_REMOTE_UI
set -eu
echo "$*" > "$HOME/flags"
echo "$AVIATOR_REMOTE_UI" > "$HOME/remote-mode"
trap 'echo begin > "$HOME/cleanup"; sleep .2; if [[ -n ${AVIATOR_RECORDING_DIR:-} ]]; then printf FINALIZED > "$AVIATOR_RECORDING_DIR/data.mcap"; fi; echo done >> "$HOME/cleanup"; exit 143' TERM
if [[ -n ${AVIATOR_RECORDING_DIR:-} ]]; then printf INCOMPLETE > "$AVIATOR_RECORDING_DIR/data.mcap"; fi
echo '所有节点已启动'
while true; do sleep .1; done
''')
        self.run_id = "a" * 32
        self.env = {**os.environ, "HOME": str(self.home)}
        self.processes = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            if process.stdout:
                process.stdout.close()

    def command(self, action, flags=(), run_id=None):
        return [sys.executable, str(SOURCE / "remote_agent.py"), action,
                "--prefix", str(self.prefix), "--run-id", run_id or self.run_id,
                "--flags", json.dumps(list(flags))]

    def launch(self, flags=(), expiry=None):
        command = self.command("run", flags)
        if expiry is not None:
            code = (SOURCE / "remote_agent.py").read_text().replace(
                "heartbeat[0] > 20", f"heartbeat[0] > {expiry}")
            command = [sys.executable, "-u", "-c", code, *command[2:]]
        p = subprocess.Popen(command, env=self.env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.processes.append(p)
        events = queue.Queue()
        def reader():
            for line in p.stdout:
                events.put(json.loads(line))
        threading.Thread(target=reader, daemon=True).start()
        self.wait_event(events, "running")
        return p, events

    def wait_event(self, events, kind):
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            event = events.get(timeout=max(.1, deadline-time.monotonic()))
            if event.get("event") == kind:
                return event
        self.fail(f"no {kind} event")

    def query(self, action):
        return json.loads(subprocess.check_output(self.command(action), env=self.env, text=True))

    def test_defaults_validation_and_flags(self):
        self.assertEqual(Options().flags(), [])
        opts = Options(rs422=True, fake_hand=True, tts=True, logger=True, transfer=True)
        self.assertEqual(opts.flags(), ["--rs422", "--fake-hand", "--tts", "--logger"])
        opts.validate()
        for opts in (Options(target="-oProxyCommand=bad"), Options(transfer=True), Options(prefix="relative"),
                     Options(rs422=True, keyboard=True)):
            with self.assertRaises(ValueError):
                opts.validate()
        result = subprocess.run(self.command("probe", ["--RS422"]), env=self.env, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.home / "flags").exists())

    def test_keyboard_forwarding_and_recovery_without_display(self):
        transport = Transport("rocos@192.168.11.100")
        self.addCleanup(transport.close)
        Path(transport.control).touch()
        with patch.dict(os.environ, {"DISPLAY": ":99"}), patch("backend.shutil.which", return_value="/usr/bin/xauth"), patch("backend.subprocess.Popen") as spawn, patch("backend.subprocess.run") as check:
            spawn.return_value.poll.return_value = None
            check.return_value.returncode = 0
            transport.connect("unused-test-password", keyboard=True)
            master = spawn.call_args.args[0]
            self.assertIn("-Y", master)
            self.assertIn("ForwardX11Trusted=yes", master)
            self.assertNotIn("-X", master)
            self.assertNotIn("-Nf", master)
            self.assertIn("ForkAfterAuthentication=no", master)
        transport.master = None  # The mocked process never existed.
        command = transport.agent("run", Options(keyboard=True), self.run_id)
        self.assertIn("-Y", command)
        self.assertIn("--keyboard", shlex.split(command[-1]))
        self.assertNotIn("--keyboard", shlex.split(transport.agent("stop", Options(keyboard=True), self.run_id)[-1]))
        with patch.dict(os.environ, {"DISPLAY": ""}):
            with self.assertRaisesRegex(RuntimeError, "DISPLAY"):
                transport.connect("unused", keyboard=True)

    def listener(self, ignore_term=False):
        code = ('import socket,time,signal\n'
                + ('signal.signal(signal.SIGTERM,signal.SIG_IGN)\n' if ignore_term else '')
                + 's=socket.socket();s.bind(("127.0.0.1",0));s.listen();print(s.getsockname()[1],flush=True);time.sleep(60)')
        process = subprocess.Popen([sys.executable, '-u', '-c', code], stdout=subprocess.PIPE, text=True)
        self.processes.append(process)
        return process, int(process.stdout.readline())

    def test_close_releases_listener_even_if_control_socket_is_missing(self):
        transport = Transport('test@localhost')
        self.addCleanup(transport.close)
        transport.master, port = self.listener()
        self.assertFalse(Path(transport.control).exists())
        transport.close()
        self.assertIsNotNone(transport.master.poll())
        self.assertFalse(Path(transport.temp.name).exists())
        with socket.socket() as check:
            check.bind(('127.0.0.1', port))

    def test_close_waits_for_exit_despite_successful_control_request(self):
        transport = Transport('test@localhost')
        self.addCleanup(transport.close)
        transport.master, port = self.listener(ignore_term=True)
        transport.forwarded = True
        Path(transport.control).touch()
        with patch('backend.subprocess.run') as command:
            command.return_value.returncode = 0
            transport.close()
            operations = [call.args[0] for call in command.call_args_list]
            self.assertTrue(any('cancel' in op for op in operations))
            self.assertTrue(any('exit' in op for op in operations))
        self.assertEqual(transport.master.returncode, -signal.SIGKILL)
        with socket.socket() as check:
            check.bind(('127.0.0.1', port))

    def test_failed_connect_reaps_master_and_removes_control_files(self):
        transport = Transport('test@localhost')
        self.addCleanup(transport.close)
        with patch('backend.owned_command', return_value=[sys.executable, '-c',
                'import sys; print("authentication test failed",file=sys.stderr); sys.exit(1)']):
            with self.assertRaisesRegex(RuntimeError, 'authentication test failed'):
                transport.connect('unused')
        self.assertIsNotNone(transport.master.poll())
        self.assertFalse(Path(transport.temp.name).exists())

    def test_master_dies_when_owning_process_is_killed(self):
        child_code = 'import socket,time; s=socket.socket(); s.bind(("127.0.0.1",0)); s.listen(); print(s.getsockname()[1],flush=True); time.sleep(60)'
        parent_code = ('import sys,subprocess,time\nfrom backend import owned_command\n'
                       'p=subprocess.Popen(owned_command([sys.executable,"-u","-c",' + repr(child_code) + ']),stdout=subprocess.PIPE,text=True)\n'
                       'print(str(p.pid)+" "+p.stdout.readline().strip(),flush=True)\ntime.sleep(60)')
        parent = subprocess.Popen([sys.executable, '-u', '-c', parent_code], stdout=subprocess.PIPE,
                                  text=True, env={**os.environ, 'PYTHONPATH': str(SOURCE)})
        self.processes.append(parent)
        pid, port = map(int, parent.stdout.readline().split())
        original_identity = identity(pid)
        try:
            parent.kill(); parent.wait(timeout=3)
            deadline = time.monotonic()+3
            while identity(pid) == original_identity and time.monotonic() < deadline:
                time.sleep(.05)
            self.assertIsNone(identity(pid), 'orphaned master survived its owner')
            with socket.socket() as check:
                check.bind(('127.0.0.1', port))
        finally:
            if identity(pid) == original_identity:
                os.kill(pid, signal.SIGKILL)

    def test_closed_http_connection_does_not_block_next_forward(self):
        transport = Transport('test@localhost')
        self.addCleanup(transport.close)
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                server.bind(('127.0.0.1', 8081))
            except OSError:
                self.skipTest('8081 is used by another program')
            server.listen()
            with socket.create_connection(('127.0.0.1', 8081)) as client:
                accepted, _ = server.accept()
                accepted.close()
                client.recv(1)
        with patch.object(transport, 'checked', return_value=''):
            transport.forward()

    def test_missing_keyboard_display_fails_before_launch(self):
        self.script.write_text(self.script.read_text() + '\n# AVIATOR_REMOTE_KEYBOARD\n')
        for action in ("probe", "run"):
            result = subprocess.run([*self.command(action), "--keyboard"], env={**self.env, "DISPLAY": ""}, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("X11", result.stderr)
            self.assertFalse((self.home / "flags").exists())
            self.assertFalse((self.home / ".local/state/aviator-remote" / self.run_id).exists())

    def test_graceful_stop_preserves_flags_and_finalizes(self):
        p, events = self.launch(["--rs422", "--logger", "--tts"])
        self.assertEqual((self.home / "remote-mode").read_text().strip(), "1")
        self.assertEqual((self.home / "flags").read_text().strip(), "--rs422 --logger --tts")
        state = self.query("status")
        self.assertTrue(state["agent_alive"] and state["child_alive"])
        p.stdin.write('{"command":"stop"}\n'); p.stdin.flush()
        result = self.wait_event(events, "exit")
        self.assertEqual(result["returncode"], 143)
        self.assertEqual((self.home / "cleanup").read_text(), "begin\ndone\n")
        self.assertEqual(Path(result["recording"], "data.mcap").read_text(), "FINALIZED")
        p.wait(timeout=3)
        self.assertEqual(self.query("status")["phase"], "finished")

    def test_node_warnings_reach_ui_and_persist_without_exiting_node(self):
        logs = self.root / 'aviator-start.test'
        logs.mkdir()
        self.script.write_text(self.script.read_text().replace(
            "echo '所有节点已启动'", f"echo {shlex.quote('日志目录: ' + str(logs))}\necho '所有节点已启动'"))
        p, events = self.launch()
        (logs / 'hand.log').write_text('[info] normal\n[error] RS485 failure\n')
        warning = self.wait_event(events, 'output')
        self.assertIn('[hand] [error] RS485 failure', warning['text'])
        self.assertIsNone(p.poll())
        (logs / 'core.log').write_text('[warning] hand completion unconfirmed\n')
        p.stdin.write('{"command":"stop"}\n'); p.stdin.flush()
        result = self.wait_event(events, 'exit')
        saved = Path(result['log']).read_text()
        self.assertIn('[hand] [error] RS485 failure', saved)
        self.assertIn('[core] [warning] hand completion unconfirmed', saved)
        self.assertNotIn('[info] normal', saved)
        self.assertEqual((self.home / 'cleanup').read_text(), 'begin\ndone\n')

    def test_node_diagnostics_partial_lines_and_log_replacement(self):
        logs = self.root / 'aviator-start.test'
        logs.mkdir()
        tail = NodeDiagnostics()
        tail.observe('日志目录: ' + str(logs) + '\n')
        path = logs / 'hand.log'
        path.write_text('[warning] 反馈')
        self.assertEqual(tail.read(), [])
        with path.open('a') as out:
            out.write('失败\n')
        self.assertEqual(tail.read(), ['[hand] [warning] 反馈失败\n'])
        self.assertEqual(tail.read(), [])
        path.rename(logs / 'hand.old')
        path.write_text('[error] replacement\n')
        self.assertEqual(tail.read(), ['[hand] [error] replacement\n'])
        path.write_text('[error] short\n')
        self.assertEqual(tail.read(), ['[hand] [error] short\n'])

    def test_ssh_eof_requests_ordered_stop(self):
        p, events = self.launch()
        p.stdin.close()
        self.wait_event(events, "exit")
        self.assertEqual((self.home / "cleanup").read_text(), "begin\ndone\n")

    def test_heartbeat_timeout_requests_stop(self):
        p, events = self.launch(expiry=.3)
        self.wait_event(events, "exit")
        p.wait(timeout=3)
        self.assertEqual((self.home / "cleanup").read_text(), "begin\ndone\n")

    def test_recovery_stop_and_identity_guard(self):
        p, events = self.launch()
        self.query("stop")
        self.wait_event(events, "exit")
        p.wait(timeout=3)
        statefile = self.home / ".local/state/aviator-remote" / self.run_id / "state.json"
        state = json.loads(statefile.read_text())
        state.update(agent_pid=os.getpid(), agent_pid_identity=["wrong", "identity"])
        statefile.write_text(json.dumps(state))
        self.assertFalse(self.query("stop")["agent_alive"])
        self.assertIsNotNone(identity(os.getpid()))

    def test_transport_quotes_paths_and_final_sync(self):
        transport = Transport("rocos@192.168.11.100")
        self.addCleanup(transport.temp.cleanup)
        with patch.object(transport, "checked", return_value="") as run:
            transport.sync("/tmp/remote path", self.root, final=True)
            command = run.call_args.args[0]
            self.assertIn("--checksum", command)
            self.assertNotIn("--append-verify", command)
            self.assertNotIn("--delete", command)
            self.assertIn("rocos@192.168.11.100:/tmp/remote path/", command)
        remote = shlex.split(transport.agent("run", Options(prefix=str(self.prefix)), self.run_id)[-1])
        self.assertEqual(remote[remote.index("--prefix")+1], str(self.prefix))

    def test_busy_port_rejected(self):
        transport = Transport("rocos@192.168.11.100")
        self.addCleanup(transport.temp.cleanup)
        with socket.socket() as server:
            try:
                server.bind(("127.0.0.1", 8081))
            except OSError:
                pass  # An existing listener also exercises the desired refusal.
            with self.assertRaisesRegex(RuntimeError, "8081"):
                transport.forward()

    def test_controller_final_sync_failure_and_recovery(self):
        testcase = self
        fail_final = [True]
        class LocalTransport:
            def __init__(self, target): pass
            def connect(self, password, keyboard=False): pass
            def forward(self): pass
            def close(self): pass
            def agent(self, action, opts, run_id):
                return ["env", "HOME=" + str(testcase.home), *testcase.command(action, opts.flags(), run_id)]
            def query(self, action, opts, run_id):
                return json.loads(subprocess.check_output(self.agent(action, opts, run_id), text=True))
            def sync(self, remote, local, final=False):
                if final and fail_final[0]:
                    raise OSError("test transfer disconnected")
                subprocess.run(["rsync", "-rt", "--checksum", remote + "/", str(local) + "/"], check=True)
        controller = Controller(self.root / "local-state")
        opts = Options(prefix=str(self.prefix), logger=True, transfer=True, destination=str(self.root / "data"))
        with patch("backend.Transport", LocalTransport), patch("backend.urllib.request.build_opener"):
            controller.start(opts)
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                event = controller.events.get(timeout=5)
                if event == {"event": "state", "text": "运行中"}:
                    break
            else:
                self.fail("controller did not start")
            controller.stop()
            controller.worker.join(timeout=8)
            self.assertFalse(controller.busy)
            self.assertTrue(controller.session_file.exists())
            session = json.loads(controller.session_file.read_text())
            self.assertTrue(Path(session["local"], ".in-progress").exists())
            with self.assertRaises(RuntimeError): controller.start(opts)
            fail_final[0] = False
            controller.start(opts, recover=True)
            controller.worker.join(timeout=8)
            self.assertFalse(controller.busy)
            self.assertFalse(controller.session_file.exists())
            self.assertFalse(Path(session["local"], ".in-progress").exists())
            self.assertEqual(Path(session["local"], "data.mcap").read_text(), "FINALIZED")
            self.assertTrue(Path(session["remote"], "data.mcap").exists())

    def test_package_is_independent(self):
        archive = self.root / "ui.tar.gz"
        subprocess.run([sys.executable, str(ROOT / "scripts/package_remote_launcher.py"),
                        "--output", str(archive)], check=True, stdout=subprocess.DEVNULL)
        with tarfile.open(archive) as package:
            names = package.getnames()
            self.assertFalse(any(name.endswith((".cpp", ".h", ".yaml")) for name in names))
            package.extractall(self.root / "unpacked")
        bundle = self.root / "unpacked/aviator-remote-ui"
        subprocess.run([str(bundle / "start.sh"), "--help"], check=True, stdout=subprocess.DEVNULL,
                       cwd=self.root, env={**os.environ, "PYTHONPATH": ""})
        self.assertTrue((bundle / "tools/remote_launcher/connection.json").exists())


if __name__ == "__main__":
    unittest.main()
